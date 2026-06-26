using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    public struct ApproachResult
    {
        public bool Found;
        public double DistanceMeters;
        public double TimeSeconds;   // time from now until the closest approach
    }

    // Finds the next closest approach between two coasting two-body trajectories. The search horizon is the
    // SYNODIC period (when the relative phase realigns), capped at maxHorizon, so an approach many orbits
    // away is still found rather than reporting the edge of a one-period window. A coarse scan brackets the
    // global minimum, then a local refine pins the time. Pure/offline-testable.
    public static class ClosestApproachSolver
    {
        public static ApproachResult FindNextApproach(
            Vector3d activePosition, Vector3d activeVelocity,
            Vector3d targetPosition, Vector3d targetVelocity,
            double mu, double maxHorizonSeconds, int coarseSamples,
            double j2 = 0.0, double referenceRadius = 0.0, Vector3d pole = default(Vector3d))
        {
            if (mu <= 0.0 || coarseSamples < 2 || maxHorizonSeconds <= 0.0)
                return new ApproachResult { Found = false };

            double periodA = OrbitMath.OrbitalPeriod(activePosition, activeVelocity, mu);
            double periodT = OrbitMath.OrbitalPeriod(targetPosition, targetVelocity, mu);

            // Horizon: at least the longer period (to catch a near pass), out to the synodic period
            // (the natural recurrence of close approaches), but never beyond maxHorizon.
            double horizon = maxHorizonSeconds;
            if (MathHelpers.IsFinite(periodA) && MathHelpers.IsFinite(periodT) && periodA > 0.0 && periodT > 0.0)
            {
                double synodic = OrbitMath.SynodicPeriod(periodA, periodT);
                double want = MathHelpers.IsFinite(synodic) ? synodic : maxHorizonSeconds;
                horizon = Math.Min(maxHorizonSeconds, Math.Max(want, Math.Max(periodA, periodT)));
            }

            // With oblateness, conic propagation diverges over a multi-orbit horizon (differential nodal
            // regression / apsidal precession / J2 period shift between two non-identical orbits) — at RSS
            // scale that's hundreds of km of CA error. Integrate both trajectories under J2 instead.
            // j2 == 0 (stock; bodies Principia models as point masses) keeps the cheap closed-form path.
            if (j2 != 0.0 && pole.sqrMagnitude > 0.0)
                return FindNextApproachJ2(activePosition, activeVelocity, targetPosition, targetVelocity,
                    mu, horizon, coarseSamples, j2, referenceRadius, pole);

            // Coarse scan brackets the global minimum, then refine within +/- one coarse step.
            ScanMinimum(activePosition, activeVelocity, targetPosition, targetVelocity, mu,
                0.0, horizon, coarseSamples, out double coarseDist, out double coarseTime);

            double step = horizon / coarseSamples;
            double refineLow = Math.Max(0.0, coarseTime - step);
            double refineHigh = coarseTime + step;
            ScanMinimum(activePosition, activeVelocity, targetPosition, targetVelocity, mu,
                refineLow, refineHigh, coarseSamples, out double fineDist, out double fineTime);

            bool useFine = fineDist <= coarseDist;
            return new ApproachResult
            {
                Found = true,
                DistanceMeters = useFine ? fineDist : coarseDist,
                TimeSeconds = useFine ? fineTime : coarseTime
            };
        }

        // J2-aware closest approach: forward-integrate both states with the RK4 J2 propagator at a uniform
        // step, sampling separation, then parabolic-interpolate distance^2 around the minimum for sub-step
        // timing. Uniform integration (vs. the conic two-phase coarse/refine) because the J2 propagator is
        // incremental — it has no closed-form "state at arbitrary t" to re-evaluate cheaply.
        private static ApproachResult FindNextApproachJ2(
            Vector3d aPos, Vector3d aVel, Vector3d tPos, Vector3d tVel,
            double mu, double horizon, int samples, double j2, double reEq, Vector3d pole)
        {
            double dt = horizon / samples;
            Vector3d ra = aPos, va = aVel, rt = tPos, vt = tVel;

            // Single forward pass; track the minimum sample and its left/right neighbours' distance^2 for
            // the parabolic refine. The right neighbour is captured on the iteration after a new minimum.
            int minIndex = 0;
            double minD2 = (ra - rt).sqrMagnitude;
            double d2Prev = minD2, d2Left = minD2, d2Right = minD2;
            bool wantRight = false;

            for (int i = 1; i <= samples; i++)
            {
                J2Propagator.Step(ref ra, ref va, mu, j2, reEq, pole, dt);
                J2Propagator.Step(ref rt, ref vt, mu, j2, reEq, pole, dt);
                double d2 = (ra - rt).sqrMagnitude;
                if (wantRight) { d2Right = d2; wantRight = false; }
                if (d2 < minD2)
                {
                    minD2 = d2;
                    minIndex = i;
                    d2Left = d2Prev;      // sample i-1
                    d2Right = d2;         // provisional until the next sample lands
                    wantRight = i < samples;
                }
                d2Prev = d2;
            }

            double tMin = minIndex * dt;
            double minDist = Math.Sqrt(minD2);

            // Parabolic vertex of distance^2 through (i-1, i, i+1); offset in [-1,1] samples. Interior only.
            if (minIndex > 0 && minIndex < samples)
            {
                double f0 = d2Left, f1 = minD2, f2 = d2Right;
                double denom = f0 - 2.0 * f1 + f2;
                if (denom > 0.0)
                {
                    double offset = 0.5 * (f0 - f2) / denom;
                    if (offset > -1.0 && offset < 1.0)
                    {
                        tMin = (minIndex + offset) * dt;
                        double vertex = f1 - 0.25 * (f0 - f2) * offset;
                        if (vertex > 0.0) minDist = Math.Sqrt(vertex);
                    }
                }
            }

            return new ApproachResult { Found = true, DistanceMeters = minDist, TimeSeconds = tMin };
        }

        // Minimum separation over [t0, t1], sampled uniformly; returns the distance and the time of it.
        private static void ScanMinimum(
            Vector3d aPos, Vector3d aVel, Vector3d tPos, Vector3d tVel, double mu,
            double t0, double t1, int samples, out double minDistance, out double timeAtMin)
        {
            minDistance = double.PositiveInfinity;
            timeAtMin = t0;
            if (t1 <= t0) { t1 = t0; samples = 0; }

            for (int i = 0; i <= samples; i++)
            {
                double t = samples > 0 ? t0 + (t1 - t0) * i / samples : t0;
                if (!TwoBody.Propagate(aPos, aVel, mu, t, out Vector3d ra, out _)) continue;
                if (!TwoBody.Propagate(tPos, tVel, mu, t, out Vector3d rt, out _)) continue;

                double d = (ra - rt).magnitude;
                if (d < minDistance) { minDistance = d; timeAtMin = t; }
            }
        }
    }
}
