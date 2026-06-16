using System;
using System.Diagnostics;
using Blackbird.Mathematics;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Outcome classification for an intercept solve. Ok = a feasible best solution within budget;
    // BudgetExhausted = the wall-clock cap was hit (any solution found is still returned, best-so-far);
    // NoFeasibleSolution = the sweep produced no valid Lambert arc; InvalidInput = bad arguments.
    public enum InterceptStatus
    {
        Ok,
        BudgetExhausted,
        NoFeasibleSolution,
        InvalidInput
    }

    // Result of an intercept plan: the impulsive burn (world-frame ΔV at the ignition UT) that puts
    // the active vessel onto a conic transfer reaching the target's predicted position, plus the
    // arrival timing and a predicted closest approach. Execution is NOT part of this — Step 4 wires
    // DeltaV/IgnitionUt into the executor.
    public struct InterceptSolution
    {
        public bool Success;
        public InterceptStatus Status;

        public Vector3d DeltaV;                       // world-frame burn applied at IgnitionUt
        public double DeltaVMagnitude;                // |DeltaV| (m/s)
        public double IgnitionUt;                     // when the burn is applied (= "now")
        public double ArrivalUt;                      // when the transfer reaches the target point
        public double TimeOfFlight;                   // ArrivalUt - IgnitionUt (s)
        public double PredictedClosestApproach;       // min transfer-to-target separation over the arc (m)

        public Vector3d TransferDepartureVelocity;    // Lambert V1 (post-burn velocity at ignition)
        public Vector3d TransferArrivalVelocity;      // Lambert V2 (velocity at arrival, used by Step 6 match)
        public int SamplesEvaluated;                  // Lambert solves actually attempted
    }

    // Step 2 intercept planner (conic, on-demand). Given the active vessel's current body-relative
    // state and a way to predict the target's position at any UT, it sweeps candidate arrival times,
    // solves a single-rev Lambert transfer for each, and keeps the lowest-ΔV feasible solution. The
    // burn is impulsive at "now" (IgnitionUt); the sweep varies only the arrival time.
    //
    // Bounded by construction: at most 'arrivalSamples' Lambert solves AND a wall-clock budget; on
    // timeout it returns the best solution found so far flagged BudgetExhausted (contract invariant 6).
    //
    // The pure Solve overload takes target prediction as a delegate so it is fully offline-testable
    // (harness injects TwoBody.Propagate); the Vessel overload injects TrajectoryProvider (Principia-
    // accurate, Stock fallback). We plan with conic math here and let the closed loop (Steps 5-6)
    // absorb the conic-vs-n-body gap — we never integrate n-body to plan a burn.
    public static class InterceptSolver
    {
        // Solves the intercept. All positions are body-relative (subtract the central body's center);
        // velocities are body-centered inertial. referenceNormal (the target's orbit normal) selects
        // the prograde arc for Lambert. targetPositionAt(ut) returns the target's body-relative
        // position at absolute time ut.
        public static InterceptSolution Solve(
            Vector3d activePosition,
            Vector3d activeVelocity,
            double mu,
            Vector3d referenceNormal,
            double ignitionUt,
            Func<double, Vector3d> targetPositionAt,
            double tofMin,
            double tofMax,
            int arrivalSamples,
            bool prograde = true,
            double budgetMilliseconds = 20.0,
            int closestApproachSamples = 48)
        {
            if (mu <= 0.0 || targetPositionAt == null || arrivalSamples < 1 ||
                tofMin <= 0.0 || tofMax < tofMin ||
                !MathHelpers.IsFinite(activePosition) || !MathHelpers.IsFinite(activeVelocity))
            {
                return new InterceptSolution { Success = false, Status = InterceptStatus.InvalidInput };
            }

            Stopwatch clock = Stopwatch.StartNew();
            InterceptSolution best = new InterceptSolution { Success = false, Status = InterceptStatus.NoFeasibleSolution };
            double bestDeltaV = double.PositiveInfinity;
            int evaluated = 0;
            bool budgetHit = false;

            for (int i = 0; i < arrivalSamples; i++)
            {
                if (budgetMilliseconds > 0.0 && clock.Elapsed.TotalMilliseconds > budgetMilliseconds)
                {
                    budgetHit = true;
                    break;
                }

                double fraction = arrivalSamples == 1 ? 0.0 : (double)i / (arrivalSamples - 1);
                double tof = tofMin + fraction * (tofMax - tofMin);
                double arrivalUt = ignitionUt + tof;

                Vector3d targetPosition = targetPositionAt(arrivalUt);
                if (!MathHelpers.IsFinite(targetPosition)) continue;

                LambertResult transfer = LambertSolver.Solve(
                    activePosition, targetPosition, tof, mu, prograde, referenceNormal);
                evaluated++;
                if (!transfer.Success) continue;

                Vector3d deltaV = transfer.V1 - activeVelocity;
                double deltaVMagnitude = deltaV.magnitude;
                if (deltaVMagnitude < bestDeltaV)
                {
                    bestDeltaV = deltaVMagnitude;
                    best = new InterceptSolution
                    {
                        Success = true,
                        Status = InterceptStatus.Ok,
                        DeltaV = deltaV,
                        DeltaVMagnitude = deltaVMagnitude,
                        IgnitionUt = ignitionUt,
                        ArrivalUt = arrivalUt,
                        TimeOfFlight = tof,
                        TransferDepartureVelocity = transfer.V1,
                        TransferArrivalVelocity = transfer.V2
                    };
                }
            }

            best.SamplesEvaluated = evaluated;

            if (best.Success)
            {
                // Compute the predicted closest approach only for the chosen transfer (cheap, once).
                best.PredictedClosestApproach = ClosestApproach(
                    activePosition, best.TransferDepartureVelocity, mu,
                    ignitionUt, best.ArrivalUt, targetPositionAt, closestApproachSamples);

                if (budgetHit) best.Status = InterceptStatus.BudgetExhausted;
            }
            else
            {
                best.Status = budgetHit ? InterceptStatus.BudgetExhausted : InterceptStatus.NoFeasibleSolution;
            }

            return best;
        }

        // In-game convenience overload: measures the active state and predicts the target through the
        // active trajectory provider (Principia-accurate, Stock fallback). Body-relative positions use
        // the central body's predicted position at each UT so a moving parent body is handled correctly.
        public static InterceptSolution Solve(
            Vessel active,
            Vessel target,
            double tofMin,
            double tofMax,
            int arrivalSamples,
            bool prograde = true,
            double budgetMilliseconds = 20.0)
        {
            CelestialBody body = active.mainBody;
            double mu = body.gravParameter;
            double ignitionUt = Planetarium.GetUniversalTime();

            Vector3d activePosition = TrajectoryProvider.GetPosition(active) - BodyPositionAtUt(body, ignitionUt);
            Vector3d activeVelocity = TrajectoryProvider.GetVelocity(active);
            Vector3d referenceNormal = TrajectoryProvider.GetOrbitNormal(target);

            Func<double, Vector3d> targetPositionAt = ut =>
                TrajectoryProvider.GetPositionAtUt(target, ut) - BodyPositionAtUt(body, ut);

            return Solve(activePosition, activeVelocity, mu, referenceNormal, ignitionUt,
                targetPositionAt, tofMin, tofMax, arrivalSamples, prograde, budgetMilliseconds);
        }

        // Minimum separation between the planned transfer and the target, sampled along the arc. With
        // exact conic prediction this is ~0 at arrival (Lambert hits the target point); under Principia
        // it reports the real conic-vs-n-body miss the closed loop will later correct.
        private static double ClosestApproach(
            Vector3d departurePosition,
            Vector3d departureVelocity,
            double mu,
            double ignitionUt,
            double arrivalUt,
            Func<double, Vector3d> targetPositionAt,
            int samples)
        {
            if (samples < 1) samples = 1;
            double tof = arrivalUt - ignitionUt;
            double minDistance = double.PositiveInfinity;

            for (int i = 0; i <= samples; i++)
            {
                double dt = tof * i / samples;
                if (!TwoBody.Propagate(departurePosition, departureVelocity, mu, dt,
                                       out Vector3d transferPosition, out _))
                    continue;

                Vector3d targetPosition = targetPositionAt(ignitionUt + dt);
                double distance = (targetPosition - transferPosition).magnitude;
                if (distance < minDistance) minDistance = distance;
            }

            return minDistance;
        }

        // World position of a body at a UT: uses its own orbit when it has a parent, else its current
        // position (e.g. the Sun). Lets body-relative coordinates stay correct as the parent moves.
        private static Vector3d BodyPositionAtUt(CelestialBody body, double ut)
        {
            return body.orbit != null ? body.orbit.getPositionAtUT(ut) : body.position;
        }
    }
}
