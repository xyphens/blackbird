using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Planning
{
    // Pure, KSP-free launch-window solver: given body constants, a target state vector, and a launch site, it
    // returns the best ascending and best descending launch candidate for the next day. No Vessel/Orbit types,
    // so the offline harness drives it with a simulated target. The KSP adapter feeds real values in.
    //
    // Pipeline (per node): find the plane-crossing time -> estimate the insertion state (geometric, downrange
    // arc from ascent duration) -> J2-propagate the target to that time -> measure plane/phase error -> pick a
    // phasing orbit (below if behind, above if ahead) inside the guardrail band -> propagate both forward to
    // score rendezvous quality. Guidance is never invoked; ascent is summarized by AscentDurationSeconds only.
    public static class LaunchWindowSolver
    {
        public struct Inputs
        {
            public double Mu;
            public double BodyRadius;
            public double AtmosphereDepth;
            public double RotationPeriodSeconds;     // sidereal, seconds per body revolution
            public double J2;                        // 0 -> no oblateness (stock)
            public double J2ReferenceRadius;
            public Vector3d Pole;                    // body spin axis (+north), inertial unit — inclination / asc-desc
            public Vector3d RotationAxis;            // body angular-velocity direction (SIGNED rotation sense in the
                                                     // host frame) for carrying the pad forward; falls back to Pole
                                                     // if zero. Separate from Pole because +angle about transform.up
                                                     // rotates the WRONG way under KSP's left-handed cross.

            public double CurrentUt;
            public Vector3d LaunchSitePosition;      // launch pad, body-centred inertial @ CurrentUt (carried forward by body rotation about Pole)

            public Vector3d TargetPosition;          // body-centred inertial @ CurrentUt
            public Vector3d TargetVelocity;
            public Vector3d TargetOrbitNormal;       // PHYSICAL prograde normal (same side as Pole for a prograde
                                                     // orbit). Supplied because cross(r,v) flips sign under KSP's
                                                     // left-handed frame; falls back to cross(r,v) if zero.

            public double AscentDurationSeconds;     // estimated launch -> insertion
            public double RemainingDeltaV;           // total vehicle dV budget
        }

        public struct Candidate
        {
            public bool IsValid;
            public string Reason;
            public string NodeName;                  // "Ascending" / "Descending"

            public double LaunchUt;
            public double SecondsUntilLaunch;
            public double AzimuthDeg;

            public double InsertionUt;
            public double PhasingApoapsisAlt;
            public double PhasingPeriapsisAlt;

            public double PlaneErrorDeg;
            public double PhaseErrorDeg;             // signed: + target ahead (we are behind), - we are ahead
            public double OrbitsToRendezvous;

            public double EstimatedDeltaVUsed;
            public double RemainingDeltaV;
            public double PredictedClosestApproachMeters;

            public double Score;                     // lower is better

            public Vector3d LaunchUtOrbitNormal;     // target orbit normal (raw cross(r,v)) AT the launch UT, not now:
                                                     // the plane the ascent must hit, tracking J2 node precession over the wait
        }

        // Guardrails ("common sense").
        private const double MaxLaunchWaitSeconds = 24.0 * 3600.0;     // never suggest a launch > 1 day out
        private const double LowerBandMarginMeters = 40000.0;          // phasing perigee >= atmosphere + 40 km
        private const double UpperBandAboveTargetMeters = 300000.0;    // phasing apogee <= target Ap + 300 km
        private const int MaxPhasingOrbits = 30;
        private const double RendezvousSampleSeconds = 30.0;           // RK4 step for the scoring propagation

        public static List<Candidate> Solve(Inputs input)
        {
            List<Candidate> result = new List<Candidate>();
            if (input.Mu <= 0.0 || input.BodyRadius <= 0.0 || input.RotationPeriodSeconds <= 0.0)
                return result;

            Vector3d hHat = input.TargetOrbitNormal.sqrMagnitude > 0.0
                ? input.TargetOrbitNormal.normalized
                : Vector3d.Cross(input.TargetPosition, input.TargetVelocity).normalized;
            if (hHat.sqrMagnitude <= 0.0) return result;
            Vector3d pole = input.Pole.sqrMagnitude > 0.0 ? input.Pole.normalized : new Vector3d(0, 0, 1);

            // Target plane sampled forward under J2: the node precesses while we wait for the window, so the
            // crossing search must meet the plane AS IT WILL BE at each future time, not the frozen now-plane
            // (freezing it leaves the launch time/normal stale by the warp duration -> off-plane insertion).
            double horizon = Math.Min(MaxLaunchWaitSeconds, input.RotationPeriodSeconds + 1.0);
            Vector3d[] normalGrid = BuildNormalGrid(input, horizon, CrossingSteps);

            // Two plane crossings (site enters the target plane) within the next day; one is ascending, one
            // descending. Each yields one candidate; we keep the best-scoring of each that survives guardrails.
            foreach (NodeCrossing crossing in FindPlaneCrossings(input, normalGrid, horizon, hHat, pole))
            {
                Candidate c = BuildCandidate(input, hHat, pole, crossing);
                result.Add(c);
            }

            return result;
        }

        private struct NodeCrossing { public double Ut; public bool Ascending; public Vector3d Normal; }

        private const double InPlaneToleranceDeg = 2.0;   // accept a crossing within this of the plane
        private const int CrossingSteps = 1440;           // plane-crossing search resolution over the horizon

        // The site enters the target plane at the local minima of its out-of-plane angle. Two per rotation
        // (ascending and descending sides) when inclination > launch latitude; one tangent minimum when they
        // are equal. A minimum is a usable crossing only if it reaches the plane within tolerance (inclination
        // below the site latitude can never reach it). Classify by whether the site is moving toward +pole.
        private static List<NodeCrossing> FindPlaneCrossings(Inputs input, Vector3d[] grid, double horizon, Vector3d hHat0, Vector3d pole)
        {
            List<NodeCrossing> crossings = new List<NodeCrossing>();
            int steps = CrossingSteps;
            double dt = horizon / steps;
            double tol = Math.Sin(MathHelpers.Deg2Rad(InPlaneToleranceDeg));

            for (int i = 1; i < steps && crossings.Count < 2; i++)
            {
                double fa = OutOfPlane(input, grid, horizon, (i - 1) * dt);
                double fb = OutOfPlane(input, grid, horizon, i * dt);
                double fc = OutOfPlane(input, grid, horizon, (i + 1) * dt);
                if (fb > fa || fb > fc) continue;                       // not a local minimum

                double tc = RefineMinimum(input, grid, horizon, (i - 1) * dt, (i + 1) * dt);
                if (OutOfPlane(input, grid, horizon, tc) > tol) continue;   // minimum doesn't reach the plane

                // Plane as it will be at the crossing (node has precessed). Classify with the +pole-aligned side so
                // the ascending/descending sense is unchanged from the frozen-plane code.
                Vector3d normalAtTc = NormalAt(grid, horizon, tc);
                Vector3d physAtTc = Align(normalAtTc, hHat0);
                // Ascending if the prograde in-plane motion at the site points toward +pole. The site's own
                // latitude is fixed, so it's the orbit's direction over the site that distinguishes the nodes.
                Vector3d siteHat = SitePositionInertial(input, tc).normalized;
                bool ascending = Vector3d.Dot(Vector3d.Cross(physAtTc, siteHat), pole) > 0.0;
                crossings.Add(new NodeCrossing { Ut = input.CurrentUt + tc, Ascending = ascending, Normal = normalAtTc });
            }
            return crossings;
        }

        // |sin(angle between the site direction and the orbit plane at dtFromNow)| = |dot(siteHat, normal(dt))|.
        private static double OutOfPlane(Inputs input, Vector3d[] grid, double horizon, double dtFromNow)
            => Math.Abs(Vector3d.Dot(SitePositionInertial(input, dtFromNow).normalized, NormalAt(grid, horizon, dtFromNow)));

        // Ternary search for the minimum of OutOfPlane over [lo, hi] (unimodal near a crossing).
        private static double RefineMinimum(Inputs input, Vector3d[] grid, double horizon, double lo, double hi)
        {
            for (int i = 0; i < 60; i++)
            {
                double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
                if (OutOfPlane(input, grid, horizon, m1) < OutOfPlane(input, grid, horizon, m2)) hi = m2; else lo = m1;
            }
            return 0.5 * (lo + hi);
        }

        // Target orbit normal (raw cross(r,v), guidance convention) sampled forward under J2 across the horizon.
        // One forward integration; NormalAt interpolates it, so the per-test-time plane lookup is cheap and the
        // crossing search tracks the precessing node instead of a frozen snapshot.
        private static Vector3d[] BuildNormalGrid(Inputs input, double horizon, int steps)
        {
            Vector3d[] grid = new Vector3d[steps + 2];
            Vector3d r = input.TargetPosition, v = input.TargetVelocity;
            grid[0] = NormalOrZero(r, v);
            double h = horizon / steps;
            for (int i = 1; i < grid.Length; i++)
            {
                Propagate(input, r, v, h, RendezvousSampleSeconds, out r, out v);
                grid[i] = NormalOrZero(r, v);
            }
            return grid;
        }

        // Linearly interpolate the sampled normal at dtFromNow (clamped to the grid) and renormalize.
        private static Vector3d NormalAt(Vector3d[] grid, double horizon, double dtFromNow)
        {
            if (grid == null || grid.Length == 0) return Vector3d.zero;
            double h = horizon / CrossingSteps;
            double x = MathHelpers.Clamp(dtFromNow / h, 0.0, grid.Length - 1.0);
            int i0 = (int)Math.Floor(x);
            int i1 = Math.Min(i0 + 1, grid.Length - 1);
            double f = x - i0;
            Vector3d n = grid[i0] * (1.0 - f) + grid[i1] * f;
            return n.sqrMagnitude > 0.0 ? n.normalized : grid[i0];
        }

        private static Vector3d NormalOrZero(Vector3d r, Vector3d v)
        {
            Vector3d n = Vector3d.Cross(r, v);
            return n.sqrMagnitude > 0.0 ? n.normalized : Vector3d.zero;
        }

        // Flip n to the same hemisphere as reference (used to keep the +pole physical side for plane geometry).
        private static Vector3d Align(Vector3d n, Vector3d reference)
            => Vector3d.Dot(n, reference) >= 0.0 ? n : -n;

        // Launch-site inertial position at CurrentUt + dt: the pad carried forward by the body's rotation about
        // its pole. Frame-agnostic (no +Z assumption), so it works with the real KSP pole.
        private static Vector3d SitePositionInertial(Inputs input, double dtFromNow)
        {
            double angle = 2.0 * Math.PI * dtFromNow / input.RotationPeriodSeconds;
            // Rotate about the body's ACTUAL angular-velocity direction (carries the host frame's rotation sense),
            // so the pad is carried the right way; Pole alone gives a sign-flipped (retrograde) sweep in KSP.
            Vector3d axis = input.RotationAxis.sqrMagnitude > 0.0 ? input.RotationAxis.normalized : input.Pole.normalized;
            return RotateAbout(input.LaunchSitePosition, axis, angle);
        }

        private static Candidate BuildCandidate(Inputs input, Vector3d hHat, Vector3d pole, NodeCrossing crossing)
        {
            // Use the plane AS AT the crossing (the node has precessed since now); keep the original +pole side so
            // inclination/insertion geometry is unchanged in convention, only its direction tracks the precession.
            hHat = Align(crossing.Normal, hHat);

            Candidate c = new Candidate
            {
                NodeName = crossing.Ascending ? "Ascending" : "Descending",
                LaunchUt = crossing.Ut,
                SecondsUntilLaunch = crossing.Ut - input.CurrentUt,
                RemainingDeltaV = input.RemainingDeltaV,
                LaunchUtOrbitNormal = crossing.Normal       // raw cross(r,v) at launch UT -> fed to ascent guidance
            };

            if (c.SecondsUntilLaunch < 0.0 || c.SecondsUntilLaunch > MaxLaunchWaitSeconds)
            { c.Reason = "Launch is more than a day out."; return c; }

            double targetRadius = input.TargetPosition.magnitude;
            double targetAlt = targetRadius - input.BodyRadius;
            double targetPeriod = 2.0 * Math.PI * Math.Sqrt(Math.Pow(targetRadius, 3.0) / input.Mu);
            double inclination = Vector3d.Angle(hHat, pole);
            double latitudeDeg = Math.Asin(MathHelpers.Clamp(
                Vector3d.Dot(input.LaunchSitePosition.normalized, pole), -1.0, 1.0)) * 180.0 / Math.PI;
            double ascAzimuth = OrbitMath.GetLaunchAzimuth(inclination, latitudeDeg);
            c.AzimuthDeg = crossing.Ascending ? ascAzimuth : MathHelpers.NormalizeDegrees(180.0 - ascAzimuth);

            // Insertion state: site at the crossing, carried downrange along the plane by the ascent arc, lifted
            // to a nominal insertion radius (the target's, for the phase measurement). Velocity = local circular.
            double siteDt = c.SecondsUntilLaunch;
            Vector3d rSite = SitePositionInertial(input, siteDt);
            Vector3d rSiteHat = rSite.normalized;
            double rInsert = targetRadius;
            double vCirc = Math.Sqrt(input.Mu / rInsert);
            double downrangeRad = 0.5 * vCirc * input.AscentDurationSeconds / rInsert;
            Vector3d rInsHat = RotateAbout(rSiteHat, hHat, downrangeRad);
            Vector3d insertionPos = rInsert * rInsHat;
            Vector3d insertionVel = vCirc * Vector3d.Cross(hHat, rInsHat);

            c.InsertionUt = c.LaunchUt + input.AscentDurationSeconds;

            // Target propagated (J2) to insertion; phase = signed in-plane angle from us to the target.
            Propagate(input, input.TargetPosition, input.TargetVelocity, c.InsertionUt - input.CurrentUt,
                RendezvousSampleSeconds, out Vector3d targetAtInsert, out Vector3d targetVelAtInsert);
            // Insertion normal is built from hHat (+pole-corrected); the target's raw Cross(r,v) follows KSP's
            // opposite-handed convention, so a direct Angle() reads ~180°. Take the acute angle = true plane separation.
            double planeAngle = Vector3d.Angle(Vector3d.Cross(insertionPos, insertionVel),
                                               Vector3d.Cross(targetAtInsert, targetVelAtInsert));
            c.PlaneErrorDeg = Math.Min(planeAngle, 180.0 - planeAngle);
            c.PhaseErrorDeg = SignedInPlaneAngle(rInsHat, targetAtInsert.normalized, hHat);

            // Pick the phasing orbit: below if the target is ahead (we chase), above if behind (we wait).
            if (!SelectPhasing(input, targetAlt, targetPeriod, c.PhaseErrorDeg,
                    out double phasingAlt, out int orbits, out double phasingDeltaV, out string reason))
            { c.Reason = reason; return c; }

            c.PhasingApoapsisAlt = c.PhasingPeriapsisAlt = phasingAlt;
            c.OrbitsToRendezvous = orbits;

            // dV: energy to raise from the pad to the phasing circle, plus the Hohmann to the target circle,
            // plus gravity + drag losses (so the remaining-fuel guardrail is honest, not just ideal).
            double ascentDeltaV = Math.Sqrt(input.Mu * (2.0 / input.BodyRadius - 1.0 / (input.BodyRadius + phasingAlt)));
            c.EstimatedDeltaVUsed = ascentDeltaV + phasingDeltaV + AscentLosses(input);
            c.RemainingDeltaV = input.RemainingDeltaV - c.EstimatedDeltaVUsed;

            // Predicted miss = how far off the rendezvous point the target is after N phasing revolutions. The
            // chaser returns to the burn point each phasing period; under J2 the target won't be exactly there,
            // and that residual (along-track) is the RSS-sensitive number a Keplerian plan would miss by.
            double phasingRadius = input.BodyRadius + phasingAlt;
            double phasingPeriod = 2.0 * Math.PI * Math.Sqrt(phasingRadius * phasingRadius * phasingRadius / input.Mu);
            Vector3d phasingPos = phasingRadius * rInsHat;
            Vector3d phasingVel = Math.Sqrt(input.Mu / phasingRadius) * Vector3d.Cross(hHat, rInsHat);
            c.PredictedClosestApproachMeters = PhaseClosureMissMeters(input,
                phasingPos, phasingVel, targetAtInsert, targetVelAtInsert, orbits * phasingPeriod, targetRadius);

            c.Score = ScoreCandidate(c, targetAlt);
            c.IsValid = true;
            c.Reason = string.Empty;
            return c;
        }

        // Choose the circular phasing altitude that closes the phase in the fewest safe orbits. Direction obeys
        // the guardrail: behind (target ahead) -> only lower/faster orbits; ahead -> only higher/slower orbits.
        private static bool SelectPhasing(
            Inputs input, double targetAlt, double targetPeriod, double signedPhaseDeg,
            out double phasingAlt, out int orbits, out double deltaV, out string reason)
        {
            phasingAlt = 0.0; orbits = 0; deltaV = 0.0; reason = string.Empty;

            double minAlt = input.AtmosphereDepth + LowerBandMarginMeters;
            double maxAlt = targetAlt + UpperBandAboveTargetMeters;
            bool behind = signedPhaseDeg > 0.0;   // target ahead -> we catch up from a lower orbit

            double phaseToClose = behind ? signedPhaseDeg : -signedPhaseDeg; // magnitude closed per the chosen sense
            Candidate best = default(Candidate);
            bool found = false; double bestScore = double.PositiveInfinity;

            for (int n = 1; n <= MaxPhasingOrbits; n++)
            {
                // Lower orbit gains phase (catch up); higher orbit loses it (let target catch up).
                double gainPerOrbit = behind ? (phaseToClose / n) : -(phaseToClose / n);
                double period = targetPeriod * (1.0 - gainPerOrbit / 360.0);
                if (period <= 0.0) continue;

                double sma = Math.Pow(input.Mu * Math.Pow(period / (2.0 * Math.PI), 2.0), 1.0 / 3.0);
                double alt = sma - input.BodyRadius;

                // Guardrails: stay in band, and never go higher when behind / lower when ahead.
                if (alt < minAlt || alt > maxAlt) continue;
                if (behind && alt > targetAlt) continue;
                if (!behind && alt < targetAlt) continue;

                double dv = HohmannDeltaV(input.Mu, input.BodyRadius + alt, input.BodyRadius + targetAlt);
                double waitHours = n * period / 3600.0;
                double score = dv + waitHours * 20.0;   // prefer cheap + soon; full scoring happens upstream
                if (score < bestScore)
                {
                    bestScore = score; found = true;
                    best = new Candidate { PhasingApoapsisAlt = alt, OrbitsToRendezvous = n, EstimatedDeltaVUsed = dv };
                }
            }

            if (!found) { reason = "No phasing orbit closes the phase inside the guardrail band."; return false; }
            phasingAlt = best.PhasingApoapsisAlt; orbits = (int)best.OrbitsToRendezvous; deltaV = best.EstimatedDeltaVUsed;
            return true;
        }

        private static double ScoreCandidate(Candidate c, double targetAlt)
        {
            double waitHours = (c.InsertionUt - c.LaunchUt + c.OrbitsToRendezvous * 0.0) / 3600.0
                               + c.SecondsUntilLaunch / 3600.0;
            double fuelPenalty = c.RemainingDeltaV < 0.0 ? 1e6 : 0.0;             // can't afford it
            double caPenalty = c.PredictedClosestApproachMeters / 1000.0;          // km of miss
            return c.EstimatedDeltaVUsed + waitHours * 50.0 + caPenalty + fuelPenalty;
        }

        // Along-track miss after the phasing: J2-propagate both craft over the N phasing periods, then take the
        // residual in-plane angle between them (the chaser is back at the burn point) times the target radius.
        private static double PhaseClosureMissMeters(
            Inputs input, Vector3d ra, Vector3d va, Vector3d rt, Vector3d vt, double horizon, double targetRadius)
        {
            double t = 0.0;
            while (t < horizon)
            {
                double h = Math.Min(RendezvousSampleSeconds, horizon - t);
                J2Propagator.Step(ref ra, ref va, input.Mu, input.J2, input.J2ReferenceRadius, input.Pole, h);
                J2Propagator.Step(ref rt, ref vt, input.Mu, input.J2, input.J2ReferenceRadius, input.Pole, h);
                t += h;
            }
            Vector3d hHat = Vector3d.Cross(rt, vt).normalized;
            double residualDeg = SignedInPlaneAngle(ra.normalized, rt.normalized, hHat);
            return Math.Abs(residualDeg) * Math.PI / 180.0 * targetRadius;
        }

        private static void Propagate(Inputs input, Vector3d r0, Vector3d v0, double dt, double step,
            out Vector3d r, out Vector3d v)
        {
            r = r0; v = v0;
            double t = 0.0;
            while (t < dt)
            {
                double h = Math.Min(step, dt - t);
                J2Propagator.Step(ref r, ref v, input.Mu, input.J2, input.J2ReferenceRadius, input.Pole, h);
                t += h;
            }
        }

        // Coarse ascent losses: gravity loss ~ a fraction of g*burn-time (the gravity-turn vertical component),
        // drag loss ~ scales with atmosphere depth. Both no-op as the body shrinks/loses atmosphere (Kerbin <
        // RSS). Coefficients are deliberately rough and meant to be calibrated against a real flight.
        private const double GravityLossFactor = 0.20;
        private const double DragLossPerMeter = 0.0014;

        private static double AscentLosses(Inputs input)
        {
            double gSurface = input.Mu / (input.BodyRadius * input.BodyRadius);
            double gravityLoss = GravityLossFactor * gSurface * input.AscentDurationSeconds;
            double dragLoss = DragLossPerMeter * input.AtmosphereDepth;
            return gravityLoss + dragLoss;
        }

        private static double HohmannDeltaV(double mu, double r1, double r2)
        {
            double v1 = Math.Sqrt(mu / r1), v2 = Math.Sqrt(mu / r2);
            double a = (r1 + r2) / 2.0;
            double vt1 = Math.Sqrt(mu * (2.0 / r1 - 1.0 / a));
            double vt2 = Math.Sqrt(mu * (2.0 / r2 - 1.0 / a));
            return Math.Abs(vt1 - v1) + Math.Abs(v2 - vt2);
        }

        // Rotate a unit vector about an axis by angle (Rodrigues; axis assumed unit).
        private static Vector3d RotateAbout(Vector3d v, Vector3d axis, double angleRad)
        {
            double c = Math.Cos(angleRad), s = Math.Sin(angleRad);
            return v * c + Vector3d.Cross(axis, v) * s + axis * (Vector3d.Dot(axis, v) * (1.0 - c));
        }

        // Signed angle (deg) from a to b measured prograde about the normal: + means b is ahead of a.
        private static double SignedInPlaneAngle(Vector3d aHat, Vector3d bHat, Vector3d normal)
        {
            double dot = MathHelpers.Clamp(Vector3d.Dot(aHat, bHat), -1.0, 1.0);
            double ang = Math.Acos(dot) * 180.0 / Math.PI;
            return Vector3d.Dot(normal, Vector3d.Cross(aHat, bHat)) < 0.0 ? -ang : ang;
        }
    }
}
