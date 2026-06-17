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

    // Finds the next closest approach between two coasting two-body trajectories. The search horizon is
    // the SYNODIC period (how long until the relative phase realigns) — NOT a single orbital period —
    // capped at maxHorizon, so a closest approach that is many orbits away is still found instead of the
    // scan reporting the edge of a one-period window (which made the reported time appear frozen). A
    // coarse scan brackets the global minimum, then a local refine pins the time. Pure/offline-testable.
    public static class ClosestApproachSolver
    {
        public static ApproachResult FindNextApproach(
            Vector3d activePosition, Vector3d activeVelocity,
            Vector3d targetPosition, Vector3d targetVelocity,
            double mu, double maxHorizonSeconds, int coarseSamples)
        {
            if (mu <= 0.0 || coarseSamples < 2 || maxHorizonSeconds <= 0.0)
                return new ApproachResult { Found = false };

            double periodA = OrbitalPeriod(activePosition, activeVelocity, mu);
            double periodT = OrbitalPeriod(targetPosition, targetVelocity, mu);

            // Horizon: at least the longer period (to catch a near pass), out to the synodic period
            // (the natural recurrence of close approaches), but never beyond maxHorizon.
            double horizon = maxHorizonSeconds;
            if (MathHelpers.IsFinite(periodA) && MathHelpers.IsFinite(periodT) && periodA > 0.0 && periodT > 0.0)
            {
                double synodic = SynodicPeriod(periodA, periodT);
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

        // Time for the relative phase of two orbits to realign; infinite when the periods are equal.
        private static double SynodicPeriod(double periodA, double periodB)
        {
            double diff = Math.Abs(1.0 / periodA - 1.0 / periodB);
            return diff < 1e-12 ? double.PositiveInfinity : 1.0 / diff;
        }

        private static double OrbitalPeriod(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;

            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;

            double a = -mu / (2.0 * energy);
            return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
        }
    }
}
