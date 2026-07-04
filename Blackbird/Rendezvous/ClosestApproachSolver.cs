using System;
using System.Collections.Generic;
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

    public struct ApproachScan
    {
        // horizon is defined as the look-ahead window
        public ApproachResult[] Passes; // empty when no approach within horizon
        public ApproachResult Next;     // Passes[0] or Found = false
        public ApproachResult Deepest;  // smallest distance; earliest pass on ties
        public int DeepestIndex;        // -1 when empty
        public bool StillConverging;    // deepest is the FINAL pass and minima strictly decrease => true minimum likely beyond the horizon
    }

    // Finds the NEXT closest approach between two coasting trajectories — the next local minimum of range
    // (the next time the pair stops separating and reaches closest), at a fixed absolute event. Returning a
    // fixed event is what makes the prediction STABLE: as the measurement time advances the same approach is
    // returned, its distance unchanged and its time counting down to zero, instead of a grid-quantized value
    // that jumps. A coarse forward sweep (step tied to the orbital period so a fast pass can't be skipped)
    // brackets the first local minimum and early-terminates; a fine sub-pass pins it. Pure/offline-testable.
    //
    // One propagation path: conic two-body when j2 == 0 (stock; bodies Principia models as point masses),
    // RK4 under J2 when j2 != 0 (RSS/Principia oblateness). Both run through the same sweep + refine.
    public static class ClosestApproachSolver
    {
        // full horizon scan of FindNextApproach
        public static ApproachScan ScanApproaches(
            Vector3d activePosition, Vector3d activeVelocity,
            Vector3d targetPosition, Vector3d targetVelocity,
            double mu, double maxHorizonSeconds, int coarseSamples,
            double j2 = 0.0, double referenceRadius = 0.0, Vector3d pole = default(Vector3d))
        {
            ApproachScan scan = new ApproachScan
            {
                Passes = new ApproachResult[0],
                Next = new ApproachResult {  Found = false },
                Deepest = new ApproachResult { Found = false },
                DeepestIndex = -1
            };

            if (mu <= 0.0 || maxHorizonSeconds <= 0.0) return scan;

            bool useJ2 = j2 != 0.0 && pole.sqrMagnitude > 0.0;

            double currentPeriod = OrbitMath.OrbitalPeriod(activePosition, activeVelocity, mu);
            double targetPeriod = OrbitMath.OrbitalPeriod(targetPosition, targetVelocity, mu);
            double minPeriod = double.PositiveInfinity;

            if (MathHelpers.IsFinite(currentPeriod) && currentPeriod > 0.0) minPeriod = Math.Min(minPeriod, currentPeriod);
            if (MathHelpers.IsFinite(targetPeriod) && targetPeriod > 0.0) minPeriod = Math.Min(minPeriod, targetPeriod);

            double periodStep = MathHelpers.IsFinite(minPeriod) ? minPeriod / CoarseStepsPerOrbit : FallbackCoarseStepSeconds;
            double budgetStep = coarseSamples >= 2 ? maxHorizonSeconds / coarseSamples : maxHorizonSeconds;
            double coarseDt = Math.Max(MinStepSeconds, Math.Min(periodStep, budgetStep));

            var passes = new List<ApproachResult>();
            Sample prev = new Sample(activePosition, activeVelocity, targetPosition, targetVelocity, 0.0);

            long maxIters = (long)(maxHorizonSeconds / coarseDt) + 4;

            for (long iter = 0; iter < maxIters && prev.T < maxHorizonSeconds; iter++) {
                if (!Step(prev, coarseDt, mu, useJ2, j2, referenceRadius, pole, out Sample cur)) break;

                if (prev.RangeRate < 0.0 && cur.RangeRate >= 0.0 && -prev.RangeRate > ClosingRateFloorFraction * prev.RangeRateScale) {
                    ApproachResult local = Refine(prev, cur.T - prev.T, mu, useJ2, j2, referenceRadius, pole);
                    if (local.Found) passes.Add(local);
                }

                prev = cur;
            }

            if (passes.Count == 0) return scan;

            // find closest
            int deepest = 0;
            for (int i = 1; i < passes.Count; i++)
            {
                if (passes[i].DistanceMeters < passes[deepest].DistanceMeters) deepest = i;
            }

            // Converging = the minima decrease strictly all the way to a final-pass deepest, so the trend says
            // the true minimum lies past the horizon. The strict-monotone requirement stops round-off ties
            // between equal-depth passes (e.g. same-radius crossing planes) from flagging a stable geometry.
            bool monotone = passes.Count >= 2;
            for (int i = 1; i < passes.Count && monotone; i++)
                monotone = passes[i].DistanceMeters < passes[i - 1].DistanceMeters;

            scan.Passes = passes.ToArray();
            scan.Next = passes[0];
            scan.Deepest = passes[deepest];
            scan.DeepestIndex = deepest;
            scan.StillConverging = monotone && deepest == passes.Count - 1;
            return scan;
        }

        // Coarse bracketing step is at most the shorter period / this, so the sweep cannot step over an
        // approach narrower than ~one such fraction of an orbit.
        private const int CoarseStepsPerOrbit = 64;
        private const int FineStepsPerBracket = 200;        // sub-steps used to pin the minimum within a bracket
        private const double MinStepSeconds = 0.05;         // floor for both passes
        private const double FallbackCoarseStepSeconds = 30.0; // when neither orbit has a finite (bound) period

        // A minimum is the range-rate sign change closing -> separating. Require the closing sample's
        // line-of-sight rate to exceed this fraction of |relPos|*|relVel|, i.e. the relative velocity has a
        // real radial (closing) component; below it the motion is purely tangential = no approach (two craft
        // holding a constant separation), so propagation/round-off noise can't fabricate a minimum.
        private const double ClosingRateFloorFraction = 1e-5;

        public static ApproachResult FindNextApproach(
            Vector3d activePosition, Vector3d activeVelocity,
            Vector3d targetPosition, Vector3d targetVelocity,
            double mu, double maxHorizonSeconds, int coarseSamples,
            double j2 = 0.0, double referenceRadius = 0.0, Vector3d pole = default(Vector3d))
        {
            if (mu <= 0.0 || maxHorizonSeconds <= 0.0)
                return new ApproachResult { Found = false };

            bool useJ2 = j2 != 0.0 && pole.sqrMagnitude > 0.0;

            // Coarse step: the shorter orbital period / CoarseStepsPerOrbit, never coarser than the caller's
            // sample budget allows, floored so a degenerate period can't drive it to zero.
            double periodA = OrbitMath.OrbitalPeriod(activePosition, activeVelocity, mu);
            double periodT = OrbitMath.OrbitalPeriod(targetPosition, targetVelocity, mu);
            double minPeriod = double.PositiveInfinity;
            if (MathHelpers.IsFinite(periodA) && periodA > 0.0) minPeriod = Math.Min(minPeriod, periodA);
            if (MathHelpers.IsFinite(periodT) && periodT > 0.0) minPeriod = Math.Min(minPeriod, periodT);

            double periodStep = MathHelpers.IsFinite(minPeriod) ? minPeriod / CoarseStepsPerOrbit : FallbackCoarseStepSeconds;
            double budgetStep = coarseSamples >= 2 ? maxHorizonSeconds / coarseSamples : maxHorizonSeconds;
            double coarseDt = Math.Max(MinStepSeconds, Math.Min(periodStep, budgetStep));

            // Forward sweep watching the range RATE: a minimum is where it crosses closing (<0) to separating
            // (>=0). The bracketing sample STATE is the restart point for the fine refine. A "closest now,
            // separating" start has a positive rate from t=0, so it is never reported as t=0 — the sweep
            // continues to the next real closing->separating crossing.
            Sample prev = new Sample(activePosition, activeVelocity, targetPosition, targetVelocity, 0.0);

            long maxIters = (long)(maxHorizonSeconds / coarseDt) + 4;
            for (long iter = 0; iter < maxIters && prev.T < maxHorizonSeconds; iter++)
            {
                if (!Step(prev, coarseDt, mu, useJ2, j2, referenceRadius, pole, out Sample cur))
                    break;

                // Crossing closing -> separating with a real closing component at prev (gate rejects the
                // tangential/constant-separation case where round-off could flip the sign).
                if (prev.RangeRate < 0.0 && cur.RangeRate >= 0.0
                    && -prev.RangeRate > ClosingRateFloorFraction * prev.RangeRateScale)
                    return Refine(prev, cur.T - prev.T, mu, useJ2, j2, referenceRadius, pole);

                prev = cur;
            }

            return new ApproachResult { Found = false };
        }

        // Propagate a single state forward (or back) by dt under J2 (conic when j2 == 0), in steps no larger
        // than ~one CoarseStepsPerOrbit fraction of the orbit so the RK4 stays accurate. Public so the planners
        // can advance the target under real oblateness when aiming a transfer.
        public static void Propagate(Vector3d r, Vector3d v, double dt, double mu,
            double j2, double referenceRadius, Vector3d pole, out Vector3d rOut, out Vector3d vOut)
        {
            rOut = r; vOut = v;
            if (dt == 0.0 || mu <= 0.0) return;

            if (!(j2 != 0.0 && pole.sqrMagnitude > 0.0))
            {
                TwoBody.Propagate(r, v, mu, dt, out rOut, out vOut);
                return;
            }

            double period = OrbitMath.OrbitalPeriod(r, v, mu);
            double cap = MathHelpers.IsFinite(period) && period > 0.0 ? period / CoarseStepsPerOrbit : FallbackCoarseStepSeconds;
            int n = Math.Max(1, (int)Math.Ceiling(Math.Abs(dt) / Math.Max(MinStepSeconds, cap)));
            double step = dt / n;
            for (int i = 0; i < n; i++) J2Propagator.Step(ref rOut, ref vOut, mu, j2, referenceRadius, pole, step);
        }

        // Honest predicted closest approach of a planned transfer: the minimum separation between the transfer
        // arc (from its departure state at ignitionUt) and the target over [ignitionUt, arrivalUt], with BOTH
        // propagated under J2 (conic when j2 == 0). The target is coasted from its measurement epoch to ignition
        // first. This is the real miss a conic plan will fly under oblateness — what the panel should show.
        public static double MinSeparationOverWindow(
            Vector3d transferDepPos, Vector3d transferDepVel, double ignitionUt, double arrivalUt,
            Vector3d targetPos, Vector3d targetVel, double measureUt,
            double mu, int samples, double j2 = 0.0, double referenceRadius = 0.0, Vector3d pole = default(Vector3d))
        {
            if (mu <= 0.0) return double.PositiveInfinity;
            bool useJ2 = j2 != 0.0 && pole.sqrMagnitude > 0.0;

            Propagate(targetPos, targetVel, ignitionUt - measureUt, mu, j2, referenceRadius, pole,
                out Vector3d rt, out Vector3d vt);

            Vector3d ra = transferDepPos, va = transferDepVel;
            double minD = (ra - rt).magnitude;
            double window = arrivalUt - ignitionUt;
            if (window <= 0.0) return minD;

            // Cap the step by the orbital period so a multi-orbit window (phasing) isn't stepped OVER — a fixed
            // sample count alone gives ~1000 km/step over several orbits and misses the encounter entirely.
            double periodA = OrbitMath.OrbitalPeriod(ra, va, mu);
            double periodT = OrbitMath.OrbitalPeriod(rt, vt, mu);
            double minPeriod = double.PositiveInfinity;
            if (MathHelpers.IsFinite(periodA) && periodA > 0.0) minPeriod = Math.Min(minPeriod, periodA);
            if (MathHelpers.IsFinite(periodT) && periodT > 0.0) minPeriod = Math.Min(minPeriod, periodT);
            double cap = MathHelpers.IsFinite(minPeriod) ? minPeriod / CoarseStepsPerOrbit : FallbackCoarseStepSeconds;

            int n = Math.Max(samples, (int)Math.Ceiling(window / Math.Max(MinStepSeconds, cap)));
            double dt = window / n;
            for (int i = 0; i < n; i++)
            {
                Advance(ref ra, ref va, ref rt, ref vt, dt, mu, useJ2, j2, referenceRadius, pole);
                double d = (ra - rt).magnitude;
                if (d < minD) minD = d;
            }
            return minD;
        }

        // Re-integrates the bracket [start.T, start.T + window] from the start state at a fine step, tracking
        // the minimum and its neighbours, then parabolic-interpolates distance^2 for sub-step timing.
        private static ApproachResult Refine(
            Sample start, double window, double mu, bool useJ2, double j2, double reEq, Vector3d pole)
        {
            int n = Math.Max(1, (int)Math.Ceiling(window / Math.Max(MinStepSeconds, window / FineStepsPerBracket)));
            double dt = window / n;

            Vector3d ra = start.Ra, va = start.Va, rt = start.Rt, vt = start.Vt;
            double minD2 = (ra - rt).sqrMagnitude, d2Prev = minD2, d2Left = minD2, d2Right = minD2;
            int minIndex = 0;
            bool wantRight = false;

            for (int i = 1; i <= n; i++)
            {
                Advance(ref ra, ref va, ref rt, ref vt, dt, mu, useJ2, j2, reEq, pole);
                double d2 = (ra - rt).sqrMagnitude;
                if (wantRight) { d2Right = d2; wantRight = false; }
                if (d2 < minD2) { minD2 = d2; minIndex = i; d2Left = d2Prev; d2Right = d2; wantRight = i < n; }
                d2Prev = d2;
            }

            double tMin = start.T + minIndex * dt;
            double minDist = Math.Sqrt(minD2);

            if (minIndex > 0 && minIndex < n)
            {
                double f0 = d2Left, f1 = minD2, f2 = d2Right;
                double denom = f0 - 2.0 * f1 + f2;
                if (denom > 0.0)
                {
                    double offset = 0.5 * (f0 - f2) / denom;
                    if (offset > -1.0 && offset < 1.0)
                    {
                        tMin = start.T + (minIndex + offset) * dt;
                        double vertex = f1 - 0.25 * (f0 - f2) * offset;
                        if (vertex > 0.0) minDist = Math.Sqrt(vertex);
                    }
                }
            }

            return new ApproachResult { Found = true, DistanceMeters = minDist, TimeSeconds = tMin };
        }

        // One coarse step from a sample, returning the advanced sample (the input is left untouched so it can
        // remain the bracket's restart state). Returns false if propagation fails.
        private static bool Step(Sample s, double dt, double mu, bool useJ2, double j2, double reEq, Vector3d pole, out Sample next)
        {
            Vector3d ra = s.Ra, va = s.Va, rt = s.Rt, vt = s.Vt;
            if (!Advance(ref ra, ref va, ref rt, ref vt, dt, mu, useJ2, j2, reEq, pole))
            {
                next = default(Sample);
                return false;
            }
            next = new Sample(ra, va, rt, vt, s.T + dt);
            return true;
        }

        // Advances both states one step in place. Conic (exact per step) when not under J2; RK4 otherwise.
        private static bool Advance(ref Vector3d ra, ref Vector3d va, ref Vector3d rt, ref Vector3d vt,
            double dt, double mu, bool useJ2, double j2, double reEq, Vector3d pole)
        {
            if (useJ2)
            {
                J2Propagator.Step(ref ra, ref va, mu, j2, reEq, pole, dt);
                J2Propagator.Step(ref rt, ref vt, mu, j2, reEq, pole, dt);
                return true;
            }

            bool a = TwoBody.Propagate(ra, va, mu, dt, out Vector3d nra, out Vector3d nva);
            bool b = TwoBody.Propagate(rt, vt, mu, dt, out Vector3d nrt, out Vector3d nvt);
            if (!a || !b) return false;
            ra = nra; va = nva; rt = nrt; vt = nvt;
            return true;
        }

        // Sweep sample: both states, the elapsed time, and the relative-range rate at that time. RangeRate has
        // the sign of d|relPos|/dt (dot of relative position and velocity); RangeRateScale = |relPos|*|relVel|
        // is its magnitude bound, used to gate out tangential (no-approach) motion.
        private struct Sample
        {
            public readonly Vector3d Ra, Va, Rt, Vt;
            public readonly double T, RangeRate, RangeRateScale;
            public Sample(Vector3d ra, Vector3d va, Vector3d rt, Vector3d vt, double t)
            {
                Ra = ra; Va = va; Rt = rt; Vt = vt; T = t;
                Vector3d relPos = rt - ra, relVel = vt - va;
                RangeRate = Vector3d.Dot(relPos, relVel);
                RangeRateScale = relPos.magnitude * relVel.magnitude;
            }
        }
    }
}
