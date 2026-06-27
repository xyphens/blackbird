using System;
using System.Collections.Generic;
using UnityEngine;
using Blackbird.Mathematics;
using Blackbird.Modules;

namespace Blackbird.Rendezvous
{
    // Active phasing: a single tangential burn now that changes the chaser's period so it closes the phase to
    // the target in N orbits, then returns to the burn point to meet it (the existing Warp-to-CA / MatchVelocity
    // / CloseApproach stages take it from there). This is the "drop/raise to phase" lever the user otherwise
    // tunes by hand; the passive Hohmann windows only wait for natural drift, which is glacial near co-altitude.
    public static class PhasingPlanner
    {
        private const double SafetyMarginMeters = 5000.0;
        private const int PhasingCaSamples = 200;   // samples for the honest (J2) predicted-CA of a phasing plan

        // One candidate per orbit-count N (both directions to close the phase), sorted cheapest-first.
        public static List<InterceptSolution> BuildPhasingPlans(IRendezvousWorld world, int maxCount)
        {
            var plans = new List<InterceptSolution>();
            if (world == null) return plans;

            double mu = world.Mu;
            Vector3d r = world.ActivePosition;
            Vector3d v = world.ActiveVelocity;
            double rMag = r.magnitude;
            double vMag = v.magnitude;
            if (mu <= 0.0 || rMag <= 0.0 || vMag <= 0.0) return plans;

            double phaseDeg = OrbitMath.GetPhaseAngleDeg(r, world.TargetPosition, world.ReferenceNormal, Vector3d.zero);
            double signedPhase = MathHelpers.DeltaDegrees(phaseDeg, 0.0);  // degrees the target leads us (short way)
            if (!MathHelpers.IsFinite(signedPhase)) return plans;

            double targetPeriod = OrbitMath.OrbitalPeriod(world.TargetPosition, world.TargetVelocity, mu);
            if (!MathHelpers.IsFinite(targetPeriod) || targetPeriod <= 0.0) return plans;

            double minSafeR = world.BodyRadius + world.AtmosphereDepth + SafetyMarginMeters;
            Vector3d vHat = v / vMag;
            double now = world.UniversalTime;
            double sign = signedPhase >= 0.0 ? 1.0 : -1.0;
            double otherWayPhase = signedPhase - 360.0 * sign;  // long way round (opposite drift direction)

            for (int n = 1; n <= maxCount; n++)
            {
                AddCandidate(world, plans, signedPhase / n, n, mu, rMag, vMag, vHat, v, targetPeriod, minSafeR, now);
                if (Math.Abs(signedPhase) > 1e-6)
                    AddCandidate(world, plans, otherWayPhase / n, n, mu, rMag, vMag, vHat, v, targetPeriod, minSafeR, now);
            }

            plans.Sort((a, b) => a.DeltaVMagnitude.CompareTo(b.DeltaVMagnitude));
            if (plans.Count > maxCount) plans = plans.GetRange(0, maxCount);
            return plans;
        }

        private static void AddCandidate(
            IRendezvousWorld world, List<InterceptSolution> plans, double gainPerOrbitDeg, int orbits,
            double mu, double rMag, double vMag, Vector3d vHat, Vector3d v,
            double targetPeriod, double minSafeR, double now)
        {
            if (Math.Abs(gainPerOrbitDeg) < 1e-4) return;

            // Period that drifts gainPerOrbitDeg relative to the target each orbit (same relation the launch
            // phasing calculator uses); period depends only on speed magnitude at r, so a tangential burn sets it.
            double phasePeriod = targetPeriod * (1.0 - gainPerOrbitDeg / 360.0);
            if (phasePeriod <= 0.0) return;

            double aPhase = Math.Pow(mu * Math.Pow(phasePeriod / (2.0 * Math.PI), 2.0), 1.0 / 3.0);
            double term = 2.0 / rMag - 1.0 / aPhase;       // vis-viva speed² / mu at r on the phasing orbit
            if (term <= 0.0) return;

            double vNew = Math.Sqrt(mu * term);
            double otherApsis = 2.0 * aPhase - rMag;        // r is one apsis of the (tangential-burn) phasing orbit
            if (Math.Min(rMag, otherApsis) < minSafeR) return;   // dips into atmosphere/ground

            double dvMag = vNew - vMag;                     // signed: + prograde (raise), - retrograde (lower)
            Vector3d dv = dvMag * vHat;
            double arrivalUt = now + orbits * phasePeriod;

            // Honest predicted CA: fly the phasing orbit and the target under J2 (conic when J2=0) over the N
            // orbits and take the minimum separation. Conic meets at the burn point (~0); J2 shifts the period
            // so the real meet-up drifts — this surfaces that miss instead of the optimistic 0.
            double predictedCa = ClosestApproachSolver.MinSeparationOverWindow(
                world.ActivePosition, v + dv, now, arrivalUt,
                world.TargetPosition, world.TargetVelocity, now, mu, PhasingCaSamples,
                world.J2, world.J2ReferenceRadius, world.Pole);

            plans.Add(new InterceptSolution
            {
                Success = true,
                Status = InterceptStatus.Ok,
                DeltaV = dv,
                DeltaVMagnitude = Math.Abs(dvMag),
                IgnitionUt = now,
                ArrivalUt = arrivalUt,
                TimeOfFlight = orbits * phasePeriod,
                PredictedClosestApproach = predictedCa,     // real miss under J2 (conic = ~0 at the burn point)
                TransferDepartureVelocity = v + dv,
                TransferArrivalVelocity = Vector3d.zero,
                SamplesEvaluated = 0
            });
        }
    }
}
