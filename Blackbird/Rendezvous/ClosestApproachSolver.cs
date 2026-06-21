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
            double mu, double maxHorizonSeconds, int coarseSamples)
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
