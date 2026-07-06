using System;
using Blackbird.FuelSim;
using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using UnityEngine;

namespace Blackbird.PsgHarness
{
    internal static class Program
    {
        private const double KerbinMu = 3.5316e12;
        private const double KerbinRadius = 600000.0;
        private const double KerbinRotationPeriod = 21549.425;
        private const double TargetInsertionAltitude = 81000.0;

        // RSS Earth (Principia sol_gravity_model): GM matches the in-game log; J2 = -sqrt(5)*C-bar(2,0)
        // with C-bar(2,0) = -4.8416945732e-04; Re = geopotential reference_radius (equatorial).
        private const double EarthMu = 398600435436096.0;
        private const double EarthMeanRadius = 6371000.0;     // KSP body Radius (mean) — used for altitudes
        private const double EarthJ2 = 1.082636e-03;
        private const double EarthRefRadius = 6378136.3;       // equatorial reference_radius

        private static int Main()
        {
            Console.WriteLine("BlackBird PSG Harness");
            Console.WriteLine();

            RunFuelSimChecks();
            RunHeadingHoldCheck();
            RunEarthJ2Check();
            RunEarthShapeSweep();
            RunCutoffLeakDemo();
            RunJ2CutoffCheck();
            RunTerminalShapeGateCheck();
            RunRssBootStallReplay();
            RunBootConvergenceSweep();
            RunTerminalSteeringGateReplay();
            //ReplayLoggedFalconHeavySolve();

            Console.WriteLine("Scenario: stock Kerbin, equatorial 81 km insertion");
            Console.WriteLine();

            PsgProblem problem = CreateKerbinScenario();
            if (problem == null || !problem.IsValid)
            {
                Console.WriteLine("Problem unavailable: " + (problem != null ? problem.ReasonUnavailable : "null"));
                return 2;
            }

            var optimizer = new PsgOptimizer();
            DateTime started = DateTime.UtcNow;
            PsgOptimizationResult result = optimizer.Solve(problem, null);
            TimeSpan elapsed = DateTime.UtcNow - started;

            Console.WriteLine("Status: " + result.Status);
            Console.WriteLine("Success: " + result.Success);
            Console.WriteLine("Iterations: " + result.Iterations);
            Console.WriteLine("Termination: " + result.TerminationType);
            Console.WriteLine("Violation: " + result.ConstraintViolation.ToString("E6"));
            Console.WriteLine("Elapsed: " + elapsed.TotalSeconds.ToString("F2") + " s");
            Console.WriteLine();

            if (result.Solution == null)
            {
                return 1;
            }

            PrintSolution(problem, result.Solution);
            return result.Success ? 0 : 1;
        }

        // Guards the hold-heading fix for the terminal "sweep": near cutoff PSG's yaw goes ill-conditioned and can
        // command a heading tens of degrees off the launch azimuth (logged 2026-07-03 Saturn V: +4..+60..-59 deg,
        // wrecking the insertion). PoweredAscentGuidance now keeps PSG's pitch but pins heading to the plane-defining
        // launch azimuth. This replays the logged yaw excursions through the exact production composition and asserts
        // the flown heading stays on the azimuth while PSG's pitch is preserved.
        private static void RunHeadingHoldCheck()
        {
            Console.WriteLine("Heading-hold steering (PSG terminal yaw sweep pinned to launch azimuth):");

            // A consistent local frame off the equator (pole +Y), like the RSS launch site. The property under test
            // is frame-invariant: hold-heading must return the launch azimuth at PSG's pitch no matter how far the
            // optimizer's yaw has wandered.
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);
            Vector3d relPos = new Vector3d(5.0e6, 3.05e6, 0.0);   // ~31 deg latitude
            PoweredAscentGuidance.LocalSteeringFrame(relPos, pole, out Vector3d up, out Vector3d north, out Vector3d east);

            double launchAzimuth = 88.6;   // the plane-defining azimuth (from the flight)
            double psgPitch = 5.0;         // shallow terminal pitch PSG holds while the yaw wanders

            Console.WriteLine("    PSG yaw off-az   held heading   pitch");
            double worstHeadingErr = 0.0, worstPitchErr = 0.0;
            foreach (double yawOffset in new[] { 0.0, 4.0, 8.0, 15.0, 31.0, 60.0, -59.0, 177.0 })
            {
                double swungHeading = MathHelpers.NormalizeDegrees(launchAzimuth + yawOffset);
                Vector3d psgDirection = PoweredAscentGuidance.ComposeSteering(swungHeading, psgPitch, up, north, east);

                // The exact production path: hold heading to the azimuth, keep PSG's pitch.
                Vector3d held = PoweredAscentGuidance.HoldHeadingSteering(psgDirection, launchAzimuth, up, north, east);
                PoweredAscentGuidance.DecomposeSteering(held, up, north, east, out double heldPitch, out double heldHeading);

                double headingErr = Math.Abs(MathHelpers.DeltaDegrees(heldHeading, launchAzimuth));
                double pitchErr = Math.Abs(heldPitch - psgPitch);
                worstHeadingErr = Math.Max(worstHeadingErr, headingErr);
                worstPitchErr = Math.Max(worstPitchErr, pitchErr);
                Console.WriteLine($"    {yawOffset,10:F1}   {heldHeading,10:F3}   {heldPitch,6:F2}");
            }

            bool pass = worstHeadingErr < 1e-6 && worstPitchErr < 1e-6;
            Console.WriteLine(pass
                ? "  [PASS] heading pinned to launch azimuth, PSG pitch preserved for every yaw excursion"
                : $"  [FAIL] worst heading err {worstHeadingErr:E2} deg, worst pitch err {worstPitchErr:E2} deg");
            Console.WriteLine();
            if (!pass) throw new Exception("Heading-hold steering regression.");
        }

        // Offline validation of J2Propagator against the flight: feed a near-circular ~186 km insertion
        // state (the solved terminal point from a logged RSS ascent, KSP world frame) and confirm the
        // J2-propagated periapsis sits ~9 km below the two-body osculating Pe (Principia showed ~9 km),
        // while a J2=0 control reproduces the osculating Pe (no spurious integration drop).
        private static void RunEarthJ2Check()
        {
            Console.WriteLine("J2 propagator check (RSS Earth, logged ~186 km insertion state):");

            Vector3d r = new Vector3d(4616976.15665178, 2999292.9189727, 3559341.11249668);
            Vector3d v = new Vector3d(-4248.80365397141, -1106.60175804457, 6443.79036928515);
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0); // Earth spin axis in KSP world frame (BodyAngularVelocity = [0, 7.29e-5, 0])

            double oscPe = OrbitSummary.FromState(EarthMu, EarthMeanRadius, r, v).PeriapsisAlt;
            double j2Pe = J2Propagator.NextPeriapsisRadius(r, v, EarthMu, EarthJ2, EarthRefRadius, pole, 6000.0, 20.0) - EarthMeanRadius;
            double kepPe = J2Propagator.NextPeriapsisRadius(r, v, EarthMu, 0.0, EarthRefRadius, pole, 6000.0, 20.0) - EarthMeanRadius;

            // This is the single number flight applies in BOTH places: the optimizer target bias
            // (TerminalJ2PeriapsisOffset) and the propagate-Pe cutoff (IsPsgTerminalComplete). They share this
            // propagation, so they stay consistent — disagreement between them was the perpetual-burn bug.
            double flightBias = kepPe - j2Pe;

            Console.WriteLine("  osculating Pe (two-body):      " + (oscPe / 1000.0).ToString("F2") + " km");
            Console.WriteLine("  propagated Pe (J2=0 control):   " + (kepPe / 1000.0).ToString("F2") + " km  (should match osculating; stock J2=0 path)");
            Console.WriteLine("  propagated Pe (J2 on):          " + (j2Pe / 1000.0).ToString("F2") + " km");
            Console.WriteLine("  flight Pe bias = cutoff offset: " + (flightBias / 1000.0).ToString("F2") + " km  (expect ~7-11 km)");
            Console.WriteLine("  control error (Kepler - osc):   " + (kepPe - oscPe).ToString("F1") + " m  (should be ~0; =0 bias in stock)");
            Console.WriteLine();
        }

        // Does inserting osculating-CIRCULAR at radius R actually yield a circular REAL orbit under J2,
        // or a skewed one? Models the optimizer's terminal state (|r|=R, FPA=0, v=circular) and propagates.
        private static void RunEarthShapeSweep()
        {
            Console.WriteLine("Osculating-circular insertion -> real (J2) orbit shape:");

            Vector3d rLog = new Vector3d(4616976.15665178, 2999292.9189727, 3559341.11249668);
            Vector3d vLog = new Vector3d(-4248.80365397141, -1106.60175804457, 6443.79036928515);
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);

            Vector3d rHat = rLog.normalized;
            Vector3d n    = Vector3d.Cross(rLog, vLog).normalized;   // orbital-plane normal
            Vector3d tHat = Vector3d.Cross(n, rHat).normalized;      // prograde tangent

            Console.WriteLine("  oscAlt(km)  realPe(km)  realAp(km)  realEcc");
            for (double oscAltKm = 185.0; oscAltKm <= 200.0; oscAltKm += 2.5)
            {
                double R = EarthMeanRadius + oscAltKm * 1000.0;
                Vector3d r = R * rHat;
                Vector3d v = Math.Sqrt(EarthMu / R) * tHat;          // osculating-circular state

                double minR, maxR;
                J2Propagator.RadiusExtremes(r, v, EarthMu, EarthJ2, EarthRefRadius, pole, 7000.0, 10.0, out minR, out maxR);

                double pe  = (minR - EarthMeanRadius) / 1000.0;
                double ap  = (maxR - EarthMeanRadius) / 1000.0;
                double ecc = (maxR - minR) / (maxR + minR);
                Console.WriteLine($"  {oscAltKm,8:F1}  {pe,9:F2}  {ap,9:F2}  {ecc,8:F5}");
            }
            Console.WriteLine();
        }

        // Why does the flight Pe not improve despite the +10 km optimizer bias? The in-flight PRIMARY cutoff
        // (PoweredAscentGuidance: e >= TerminalSpecificEnergy) is shape-blind: it fires the instant total
        // specific energy reaches the biased-195 target, regardless of WHERE / what shape. This holds the
        // biased terminal energy e(195-circ) FIXED and shows real Pe swing from ~175 (cut early, still low &
        // eccentric) to ~185 (cut at circular-195). Same energy, 10 km of real Pe -> energy alone cannot pin Pe.
        private static void RunCutoffLeakDemo()
        {
            Console.WriteLine("Energy-cutoff leak (all states share the biased 195-circular specific energy):");

            Vector3d rLog = new Vector3d(4616976.15665178, 2999292.9189727, 3559341.11249668);
            Vector3d vLog = new Vector3d(-4248.80365397141, -1106.60175804457, 6443.79036928515);
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);

            Vector3d rHat = rLog.normalized;
            Vector3d n    = Vector3d.Cross(rLog, vLog).normalized;
            Vector3d tHat = Vector3d.Cross(n, rHat).normalized;

            double targetPeAlt = 185000.0;
            double biasedAlt = 195000.0;                       // osc target after +10 km J2 bias (from sweep)
            double Rbias = EarthMeanRadius + biasedAlt;
            double eTarget = -EarthMu / (2.0 * Rbias);          // TerminalSpecificEnergy the flight cuts on
            double hTarget = Math.Sqrt(EarthMu * Rbias);        // |h| of the intended circular-195 terminal

            Console.WriteLine("  TerminalSpecificEnergy target = " + eTarget.ToString("F1") + " J/kg, |h| target = " + hTarget.ToString("F0"));
            Console.WriteLine("  cutRadiusAlt(km)  fpa(deg)  osc(Pe x Ap)        |h|/hTgt   realPe(km)  energyCutFires");
            // Sweep the radius at which energy crosses the target, horizontal burn (the realistic direct-ascent
            // terminal is near-horizontal). Each state is built to have EXACTLY eTarget, so the energy cutoff
            // would fire at every row -- but real Pe is only on target at the top (circular-195) row.
            foreach (double cutAltKm in new[] { 185.0, 187.5, 190.0, 192.5, 195.0 })
            {
                double r = EarthMeanRadius + cutAltKm * 1000.0;
                double speed = Math.Sqrt(2.0 * (eTarget + EarthMu / r)); // speed giving exactly eTarget at this r
                Vector3d rv = r * rHat;
                Vector3d vv = speed * tHat;                              // horizontal (FPA = 0)

                double e = 0.5 * vv.sqrMagnitude - EarthMu / r;
                double h = Vector3d.Cross(rv, vv).magnitude;
                var osc = OrbitSummary.FromState(EarthMu, EarthMeanRadius, rv, vv);
                double minR, maxR;
                J2Propagator.RadiusExtremes(rv, vv, EarthMu, EarthJ2, EarthRefRadius, pole, 7000.0, 10.0, out minR, out maxR);
                double realPe = (minR - EarthMeanRadius) / 1000.0;
                bool fires = e >= eTarget - 1.0;

                Console.WriteLine($"  {cutAltKm,14:F1}  {0.0,7:F1}  {osc.PeriapsisAlt/1000.0,7:F1} x {osc.ApoapsisAlt/1000.0,-7:F1}  {h/hTarget,8:F4}  {realPe,9:F2}   {fires}");
            }
            // |h|/hTgt stays ~1.0000 across all rows: at LEO radius these 185x205..195x195 orbits are barely
            // eccentric, so |h| can NOT discriminate them -- an energy-AND-|h| cutoff would accept all five.
            // The real Pe is NOT a fixed osc-minus-10km: it depends on the insertion true-anomaly/latitude
            // (osc-circular-195 -> 184.9, but osc-185x205 with the cut AT osc-periapsis -> 185.0). Only forward
            // propagation of the MEASURED state (IsPsgTerminalComplete) captures this; a scalar apsis bias does not.
            Console.WriteLine("  -> Same energy, same |h|, real Pe spans ~5 km purely from insertion geometry.");
            Console.WriteLine("     target real Pe = " + (targetPeAlt/1000.0).ToString("F0") + " km; only propagate-real-Pe is geometry-complete.");
            Console.WriteLine();
        }

        // Replays the EXACT terminal PSG problem from the Falcon Heavy 216x145 flight (psg.log 15:34:59, the
        // last solve before the energy cutoff). In flight, solve cycles were ~38 s apart -> the terminal phase
        // flew open-loop on a stale solution. This times the solve offline so we can confirm the cost and
        // attack it without burning 20-minute live tests. Note: this PsgProblem.Create overload leaves J2=0
        // (only the VesselState overload sets it); negligible for SOLVE TIME (one extra cheap RHS term).
        private static void ReplayLoggedFalconHeavySolve()
        {
            Console.WriteLine("Falcon Heavy terminal solve replay (logged 3-phase problem that took ~38 s in flight):");

            Vector3d bodyAngularVelocity = new Vector3d(0.0, 7.292115373194e-05, 0.0);
            PsgBodyModel body = PsgBodyModel.Create(EarthMu, EarthMeanRadius, bodyAngularVelocity);

            PsgInitialState initial = PsgInitialState.Create(
                new Vector3d(4587864.75776515, 3017246.10103716, 3589562.88275476),
                new Vector3d(-4365.59637197712, -1122.52209244581, 6044.49602198621),
                49795.5860495567,   // CurrentMassKg
                0.0);

            // 3 future powered phases as logged (masses tons = kg/1000, thrust kN = N/1000).
            PoweredStageInfo[] stages =
            {
                MakeStage(3, 0, 49.8145980834961, 30.4665699005127, 805.0, 345.0, 0.447204968944099, 181.833511352539),
                MakeStage(2, 1, 62.8518829345703, 37.4051895141602, 805.0, 345.0, 0.447204968944099, 239.148986816406),
                MakeStage(1, 2, 36.8051948547363, 22.7383918762207, 805.0, 345.0, 0.447204968944099, 132.200347900391),
            };
            PsgPhase[] all3 = PsgPhase.FromPoweredStages(stages);
            PsgPhase[] only1 = PsgPhase.FromPoweredStages(new[] { stages[0] }); // active insertion stage only

            // Proper (nonzero) plane normal from the current orbital plane (cross(r,v)) vs the logged degenerate [0,0,0].
            Vector3d r0 = new Vector3d(4587864.75776515, 3017246.10103716, 3589562.88275476);
            Vector3d v0 = new Vector3d(-4365.59637197712, -1122.52209244581, 6044.49602198621);
            Vector3d properNormal = Vector3d.Cross(r0, v0).normalized;

            Vector3d thrustDir = new Vector3d(0.0400270908160885, 0.202135614934217, 0.978539230269266);

            // Isolate the two cost levers: phase count (3 future stages vs the 1 that actually fires) and the
            // degenerate target normal (zero vs a real plane). One cold solve each (each 3-phase run is ~60 s).
            PsgPhase[] only2 = PsgPhase.FromPoweredStages(new[] { stages[0], stages[1] }); // staging spans circularization

            // Per-stage vacuum dv (what the dv-sufficiency trim keys on) vs the ~200 m/s velocity-to-go at insertion.
            Console.WriteLine("  per-stage vac dv: " + string.Join(", ", System.Array.ConvertAll(all3, p => (p.VacuumSpecificImpulseSeconds * 9.80665 * Math.Log(p.StartMassKg / p.EndMassKg)).ToString("F0") + " m/s")));
            // Same 3 stages, but masses chained so each starts where the previous ended (ratios/dv preserved).
            // Isolates whether the non-convergence is the non-physical KSP masses or the optimizer itself.
            double ratio2 = 62.8518829345703 / 37.4051895141602;
            double ratio1 = 36.8051948547363 / 22.7383918762207;
            double s2Start = 30.4665699005127, s2End = s2Start / ratio2;
            double s3Start = s2End,             s3End = s3Start / ratio1;
            PsgPhase[] chained3 = PsgPhase.FromPoweredStages(new[]
            {
                stages[0],
                MakeStage(2, 1, s2Start, s2End, 805.0, 345.0, 0.447204968944099, 239.148986816406),
                MakeStage(1, 2, s3Start, s3End, 805.0, 345.0, 0.447204968944099, 132.200347900391),
            });

            Console.WriteLine("  per-stage vac dv: " + string.Join(", ", System.Array.ConvertAll(all3, p => (p.VacuumSpecificImpulseSeconds * 9.80665 * Math.Log(p.StartMassKg / p.EndMassKg)).ToString("F0") + " m/s")));
            TimeSolve("3 phases (as flown, bad masses)", initial, body, all3,     Vector3d.zero, thrustDir);
            TimeSolve("3 phases (chained masses)",       initial, body, chained3, Vector3d.zero, thrustDir);
            TimeSolve("2 phases (stage spans circ)",     initial, body, only2,    Vector3d.zero, thrustDir);
            TimeSolve("1 phase  (active only)",          initial, body, only1,    Vector3d.zero, thrustDir);
            Console.WriteLine();
        }

        // Replays the EXACT logged PsgProblem from the 2026-06-27 RSS ascent that bailed (psg-failure-bailout):
        // a 45 km / 2-phase state (booster 45 MN, TWR 1.76, then upper 11.8 MN) targeting ~209 km circular. In
        // flight the boot SQP stalled ("PSG boot did not satisfy constraints pf~0.19 @terminal"). This is a
        // FEASIBLE state (booster TWR>1), so a failure here is the optimizer/seed, not the craft — it lets us
        // iterate the boot fix offline instead of burning 20-minute live launches.
        private static void RunRssBootStallReplay()
        {
            Console.WriteLine("RSS boot-stall replay (logged 2026-06-27 ascent that bailed, 45 km / 2-phase, feasible):");

            PsgBodyModel body = PsgBodyModel.Create(EarthMu, EarthMeanRadius, new Vector3d(0.0, 7.292115373194e-05, 0.0));

            PsgInitialState initial = PsgInitialState.Create(
                new Vector3d(5304419.33476153, 3072084.97239432, 1896010.65653861),
                new Vector3d(560.949892246852, 477.639164070807, 1060.58923208426),
                2703361.99331284,
                130189266.174328);

            // Two future powered phases as logged (MakeStage takes tons / kN): booster then upper.
            PsgPhase[] phases = PsgPhase.FromPoweredStages(new[]
            {
                MakeStage(2, 0, 2705.09228515625, 2396.17993164063, 45110.58984375,  347.0, 0.400000008659275, 58.2568321228027),
                MakeStage(1, 1, 1662.83056640625,  131.999877929688, 11767.9794921875, 380.0, 1.0,              484.765045166016),
            });

            Vector3d normal = new Vector3d(0.275233953926253, -0.877917033896904, -0.391800908880752);
            Vector3d thrustDir = new Vector3d(0.506295701186978, 0.400220608707339, 0.76386394555936);

            // As flown: real target normal -> FPA5 boot stalled @terminal in flight and bailed suborbital. With the
            // plane-relaxed retry it must now bootstrap (current plane) and reach orbit instead of failing.
            PsgTarget target = PsgTarget.Create(EarthMu, 6579940.52501887, 6579940.52501887, 6579940.52501887,
                normal, 28.6078919043405, 194.745041994206, true);
            PsgProblem problem = PsgProblem.Create(initial, body, target, phases, thrustDir);
            if (problem == null || !problem.IsValid)
            {
                Console.WriteLine("  problem invalid: " + (problem != null ? problem.ReasonUnavailable : "null"));
                Console.WriteLine();
                return;
            }

            DateTime t0 = DateTime.UtcNow;
            PsgOptimizationResult result = new PsgOptimizer().Solve(problem, null);
            double sec = (DateTime.UtcNow - t0).TotalSeconds;
            Console.WriteLine($"  success={result.Success} iters={result.Iterations} viol={result.ConstraintViolation:E2} {sec,5:F2}s | {result.Status}");
            Console.WriteLine(result.Success
                ? "  [PASS] previously-bailing geometry now bootstraps to orbit (plane-relaxed retry)"
                : "  [FAIL] still bails suborbital");
            Console.WriteLine();
        }

        // Terminal-steering-gate replay — the regression guard for the end-of-burn pitch spike. Replays the
        // logged 2026-06-28 18:36:53 circularization: clean steering glides <= 0.11 deg/s, then in the last ~1 s
        // the orbit error -> 0 makes the terminal thrust direction ill-conditioned and successive solves command
        // ~39 deg/s swings (the +25 deg pitch-up). The gate must reject those (unflyable) and hold the last clean
        // vector, WITHOUT wrongly holding any of the clean glide. Asserts both, plus the hold/leak boundary.
        private struct Sample
        {
            public double T; public Vector3d Dir;
            public Sample(double t, double x, double y, double z) { T = t; Dir = new Vector3d(x, y, z).normalized; }
        }

        private static void RunTerminalSteeringGateReplay()
        {
            Console.WriteLine("Terminal steering gate replay (logged 2026-06-28 18:36:53 circ; +25 deg pitch spike in last ~1 s):");

            // First-point steering of each logged solve (UniversalTime - t0, InertialThrustDirection). The first 6
            // are the clean glide; index 6 (t=304.60) is the last clean solve; 7.. are the degenerate terminal swings.
            Sample[] s =
            {
                new Sample(155.1000, 0.998135414, 0.045678263, 0.040486926),
                new Sample(160.1400, 0.997988188, 0.040995160, 0.048362930),
                new Sample(165.1800, 0.997763614, 0.036395139, 0.056063933),
                new Sample(170.2200, 0.997453218, 0.031731583, 0.063876323),
                new Sample(195.2400, 0.994584553, 0.007918362, 0.103628499),
                new Sample(256.2600, 0.977798014, -0.052317655, 0.202913545),
                new Sample(304.6015, 0.953837646, -0.101506549, 0.282648484),  // last clean
                new Sample(305.1215, 0.995753173, -0.062458335, -0.067635605),  // spike onset
                new Sample(305.6415, 0.997016683, -0.047891720, -0.060531953),
                new Sample(306.1615, 0.996303652, -0.073387445, -0.044646576),
                new Sample(306.6815, 0.993296730, 0.017522518, 0.114256582),
                new Sample(307.2015, 0.892176674, -0.116349010, 0.436444374),
                new Sample(307.7215, 0.860051051, 0.072408243, 0.505043795),
            };
            const int lastClean = 6;
            Vector3d cleanRef = s[lastClean].Dir;

            // Worst clean glide rate (deg/s) over the clean run — the gate cap must stay above this.
            double maxCleanRate = 0.0;
            for (int i = 1; i <= lastClean; i++)
                maxCleanRate = Math.Max(maxCleanRate, Vector3d.Angle(s[i - 1].Dir, s[i].Dir) / (s[i].T - s[i - 1].T));

            // Raw (no gate): how far the terminal swings stray from the last clean vector.
            double rawMax = 0.0;
            for (int i = lastClean + 1; i < s.Length; i++)
                rawMax = Math.Max(rawMax, Vector3d.Angle(cleanRef, s[i].Dir));

            // Representative heavy-upper-stage slew cap (deg/s): well above the 0.11 clean glide, well below the
            // ~39 spike. Derived as the midpoint in log-space of those two — not eyeballed to a round number.
            double capDegPerSec = Math.Sqrt(maxCleanRate * (rawMax / (s[lastClean + 1].T - s[lastClean].T)));
            double capRad = MathHelpers.Deg2Rad(capDegPerSec);

            // Run the gate over the full sequence.
            TerminalSteeringGate gate = new TerminalSteeringGate();
            bool allCleanAccepted = true;
            double gatedMaxSpike = 0.0;   // max excursion from clean over the spike window
            for (int i = 0; i < s.Length; i++)
            {
                Vector3d flown = gate.Update(s[i].Dir, s[i].T, capRad);
                bool accepted = (flown - s[i].Dir).sqrMagnitude < 1e-12;
                if (i <= lastClean && !accepted) allCleanAccepted = false;
                if (i > lastClean) gatedMaxSpike = Math.Max(gatedMaxSpike, Vector3d.Angle(cleanRef, flown));
            }

            Console.WriteLine($"  clean glide max rate = {maxCleanRate:F2} deg/s | terminal spike ~{rawMax / (s[lastClean + 1].T - s[lastClean].T):F0} deg/s | cap = {capDegPerSec:F2} deg/s");
            Console.WriteLine($"  raw (no gate) max steering excursion from last clean = {rawMax:F1} deg");
            Console.WriteLine($"  gated         max steering excursion from last clean = {gatedMaxSpike:F1} deg");
            Console.WriteLine(allCleanAccepted
                ? "  [PASS] every clean solve accepted (gate never starves legitimate slow steering)"
                : "  [FAIL] a clean solve was wrongly held");
            Console.WriteLine(gatedMaxSpike < 1.0
                ? "  [PASS] terminal spike rejected; flown steering held at last clean vector (< 1 deg)"
                : $"  [FAIL] terminal spike leaked through ({gatedMaxSpike:F1} deg excursion)");

            // Hold/leak boundary: sweep the cap and report max spike excursion. Holds (~0) until the cap exceeds
            // the spike rate, then leaks to the raw excursion. Confirms the gate keys on physical followability.
            Console.Write("  cap sweep (deg/s -> excursion deg): ");
            foreach (double cap in new[] { 1.0, 3.0, 8.0, 20.0, 40.0, 60.0 })
            {
                TerminalSteeringGate g = new TerminalSteeringGate();
                double mx = 0.0;
                for (int i = 0; i < s.Length; i++)
                {
                    Vector3d flown = g.Update(s[i].Dir, s[i].T, MathHelpers.Deg2Rad(cap));
                    if (i > lastClean) mx = Math.Max(mx, Vector3d.Angle(cleanRef, flown));
                }
                Console.Write($"{cap:F0}->{mx:F1}  ");
            }
            Console.WriteLine();
            Console.WriteLine();
        }

        // Convergence-robustness sweep — the regression guard for the boot-stall class. The bug only bit because
        // every prior PSG scenario fed a PLANE-MATCHED state; a real launch arrives off the target plane (RAAN
        // miss). Take the feasible 45 km ascent and rotate the TARGET plane away from the craft's reachable plane
        // (0..45 deg) across two booster TWRs, and assert the boot reaches orbit at every feasible combo. A stall
        // fails here in seconds, offline, instead of in a 20-minute live launch.
        private static void RunBootConvergenceSweep()
        {
            Console.WriteLine("PSG boot convergence sweep (plane error x TWR on a feasible RSS ascent):");

            PsgBodyModel body = PsgBodyModel.Create(EarthMu, EarthMeanRadius, new Vector3d(0.0, 7.292115373194e-05, 0.0));
            Vector3d r = new Vector3d(5304419.33476153, 3072084.97239432, 1896010.65653861);
            Vector3d v = new Vector3d(560.949892246852, 477.639164070807, 1060.58923208426);
            Vector3d thrustDir = new Vector3d(0.506295701186978, 0.400220608707339, 0.76386394555936);
            PsgInitialState initial = PsgInitialState.Create(r, v, 2703361.99331284, 130189266.174328);

            Vector3d currentNormal = Vector3d.Cross(r, v).normalized;   // the plane the craft can actually reach
            Vector3d tiltAxis = r.normalized;                           // tilt about radial -> relative inclination
            double[] planeErrorsDeg = { 0.0, 10.0, 20.0, 30.0, 45.0 };
            double[] twrScales = { 1.0, 0.7 };                          // both keep booster TWR > 1 (feasible)

            int failures = 0, total = 0;
            foreach (double twr in twrScales)
            {
                PsgPhase[] phases = PsgPhase.FromPoweredStages(new[]
                {
                    MakeStage(2, 0, 2705.09228515625, 2396.17993164063, 45110.58984375 * twr, 347.0, 0.400000008659275, 58.2568321228027),
                    MakeStage(1, 1, 1662.83056640625,  131.999877929688, 11767.9794921875,     380.0, 1.0,              484.765045166016),
                });

                foreach (double deg in planeErrorsDeg)
                {
                    Vector3d targetNormal = RotateAbout(currentNormal, tiltAxis, deg * Math.PI / 180.0);
                    PsgTarget target = PsgTarget.Create(EarthMu, 6579940.52501887, 6579940.52501887, 6579940.52501887,
                        targetNormal, 28.6, 194.7, true);
                    PsgProblem problem = PsgProblem.Create(initial, body, target, phases, thrustDir);

                    total++;
                    PsgOptimizationResult result = problem != null && problem.IsValid ? new PsgOptimizer().Solve(problem, null) : null;
                    bool ok = result != null && result.Success;
                    if (!ok) failures++;
                    Console.WriteLine($"  twr x{twr:F1}  plane {deg,4:F0} deg : {(ok ? "[PASS]" : "[FAIL]")}  iters={result?.Iterations}  viol={result?.ConstraintViolation:E2}");
                }
            }

            Console.WriteLine(failures == 0
                ? $"  SWEEP PASS: boot reached orbit on all {total} feasible cases"
                : $"  SWEEP FAIL: {failures}/{total} cases bailed (boot did not converge)");
            Console.WriteLine();
        }

        // Rodrigues rotation (QuaternionD is Unity-native and crashes offline, so do it by hand).
        private static Vector3d RotateAbout(Vector3d vec, Vector3d axis, double angleRad)
        {
            axis = axis.normalized;
            double c = Math.Cos(angleRad), s = Math.Sin(angleRad);
            return vec * c + Vector3d.Cross(axis, vec) * s + axis * (Vector3d.Dot(axis, vec) * (1.0 - c));
        }

        private static void TimeSolve(string label, PsgInitialState initial, PsgBodyModel body, PsgPhase[] phases, Vector3d normal, Vector3d thrustDir)
        {
            PsgTarget target = PsgTarget.Create(EarthMu, 6556000.0, 6556000.0, 6556000.0, normal, 28.6046612017644, 208.786926541584, true);
            PsgProblem problem = PsgProblem.Create(initial, body, target, phases, thrustDir);
            if (problem == null || !problem.IsValid)
            {
                Console.WriteLine("  " + label + ": problem invalid (" + (problem != null ? problem.ReasonUnavailable : "null") + ")");
                return;
            }

            DateTime t0 = DateTime.UtcNow;
            PsgOptimizationResult result = new PsgOptimizer().Solve(problem, null); // cold; matches in-flight worst case
            double sec = (DateTime.UtcNow - t0).TotalSeconds;
            Console.WriteLine($"  {label,-34}: {sec,7:F2} s  success={result.Success}  iters={result.Iterations}  viol={result.ConstraintViolation:E2}");
        }

        private static PoweredStageInfo MakeStage(int kspStage, int phaseIndex, double startMassTon, double endMassTon, double thrustKn, double ispSec, double minThrottle, double burnTimeSec)
        {
            return new PoweredStageInfo
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                Stage = kspStage,
                PhaseIndex = phaseIndex,
                IsCurrentOrFutureStage = true,
                StartMass = startMassTon,
                EndMass = endMassTon,
                VacuumSpecificImpulse = ispSec,
                CurrentSpecificImpulse = ispSec,
                VacuumThrust = thrustKn,
                CurrentThrust = thrustKn,
                MinimumThrust = thrustKn * minThrottle,
                MinimumThrottle = minThrottle,
                BurnTimeSeconds = burnTimeSec,
                VacuumDeltaV = 0.0,
                CurrentDeltaV = 0.0
            };
        }

        // Exercises the in-flight cutoff criterion (energyReached AND J2-realPe >= targetPe) along a prograde
        // circularizing burn at the 185 km insertion radius. Shows the OLD energy-only cut would fire at the
        // very first row (circular-185, real Pe ~175) while the NEW combined criterion holds until real Pe
        // reaches 185 -- the whole point of the J2PeriapsisRadius gate.
        private static void RunJ2CutoffCheck()
        {
            Console.WriteLine("J2 realPe cutoff (prograde burn past circular-185; eTarget = unbiased circular-185):");

            Vector3d rLog = new Vector3d(4616976.15665178, 2999292.9189727, 3559341.11249668);
            Vector3d vLog = new Vector3d(-4248.80365397141, -1106.60175804457, 6443.79036928515);
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);

            Vector3d rHat = rLog.normalized;
            Vector3d n    = Vector3d.Cross(rLog, vLog).normalized;
            Vector3d tHat = Vector3d.Cross(n, rHat).normalized;

            double targetRadius = EarthMeanRadius + 185000.0;

            // Converged-insertion model: craft on the osculating-CIRCULAR family, climbing radius (what the
            // optimizer steers toward). Compare three acceptance tests: realPe>=target (current code),
            // realAp>=target, and real mean a >=target (size). The size test centers the real orbit on 185.
            Console.WriteLine("  oscAlt  realPe   realAp   realMean(a)   firstCut");
            bool peCut = false, apCut = false, meanCut = false;
            for (double oscAltKm = 185.0; oscAltKm <= 200.0; oscAltKm += 1.0)
            {
                double R = EarthMeanRadius + oscAltKm * 1000.0;
                Vector3d rv = R * rHat;
                Vector3d vv = Math.Sqrt(EarthMu / R) * tHat;   // osculating-circular at this radius
                double minR, maxR;
                J2Propagator.RadiusExtremes(rv, vv, EarthMu, EarthJ2, EarthRefRadius, pole, 7000.0, 10.0, out minR, out maxR);
                double realPeKm = (minR - EarthMeanRadius) / 1000.0;
                double realApKm = (maxR - EarthMeanRadius) / 1000.0;
                double realMeanKm = ((minR + maxR) * 0.5 - EarthMeanRadius) / 1000.0;

                string tag = "";
                if (!apCut   && maxR >= targetRadius)               { apCut = true;   tag += " AP-cut(real " + realPeKm.ToString("F0") + "x" + realApKm.ToString("F0") + ")"; }
                if (!meanCut && (minR + maxR) * 0.5 >= targetRadius) { meanCut = true; tag += " MEAN-cut(real " + realPeKm.ToString("F0") + "x" + realApKm.ToString("F0") + ")"; }
                if (!peCut   && minR >= targetRadius)               { peCut = true;   tag += " PE-cut(real " + realPeKm.ToString("F0") + "x" + realApKm.ToString("F0") + ")"; }

                Console.WriteLine($"  {oscAltKm,5:F1}  {realPeKm,7:F1}  {realApKm,7:F1}  {realMeanKm,10:F1}  {tag}");
            }
            Console.WriteLine("  -> AP-cut: real ~175x185 (the old behavior).  PE-cut: real ~185x195 (Pe nailed, Ap high).");
            Console.WriteLine("     MEAN(size)-cut: real ~180x190, centered on 185 but still ~10 km spread.");
            Console.WriteLine("     The ~10 km spread is J2 on an osc-circular insertion; a cutoff can position it, not remove it.");
            Console.WriteLine();
        }

        // Guards the terminal-cut decision (DecideTerminalCut + TerminalShapeBandMeters): energy+mean-radius
        // alone completed a 89x322 km orbit against a ~200 km circular target (2026-07-05 flight, e=0.018,
        // real Pe read 18 km) — the banded real-Pe term must BLOCK that cut while still passing the validated
        // converged insertions in RSS (osc-circular family) and stock (J2=0, floor band), and must not
        // recreate the 06-24 hard-Pe runaway (the block is grace-bounded in GetCommand, decision-only here).
        private static void RunTerminalShapeGateCheck()
        {
            Console.WriteLine("Terminal shape gate (banded real-Pe blocks SMA-right/eccentric cuts):");

            // Inclined basis from the logged RSS insertion state (~29 deg) — the equator maximizes the J2
            // radial breathing (~2x the inclined value), which misrepresents the flights being guarded.
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);
            Vector3d rLog = new Vector3d(4616976.15665178, 2999292.9189727, 3559341.11249668);
            Vector3d vLog = new Vector3d(-4248.80365397141, -1106.60175804457, 6443.79036928515);
            Vector3d rHat = rLog.normalized;
            Vector3d tHat = Vector3d.Cross(Vector3d.Cross(rLog, vLog).normalized, rHat).normalized;
            bool pass = true;

            // (A) The 2026-07-05 failure: conic 89.4x322.9 (a on target) vs ~200 km circular target.
            {
                double rPe = EarthMeanRadius + 89435.0, rAp = EarthMeanRadius + 322938.0;
                double a = 0.5 * (rPe + rAp);
                Vector3d r = rPe * rHat;
                Vector3d v = Math.Sqrt(EarthMu * (2.0 / rPe - 1.0 / a)) * tHat;
                double targetRadius = EarthMeanRadius + 200000.0, targetPeRadius = targetRadius;
                double energy = -EarthMu / (2.0 * a), targetEnergy = -EarthMu / (2.0 * targetRadius);

                J2Propagator.RadiusExtremes(r, v, EarthMu, EarthJ2, EarthRefRadius, pole, 7000.0, 10.0, out double minR, out double maxR);
                double band = PoweredAscentGuidance.TerminalShapeBandMeters(EarthJ2, EarthRefRadius, rPe);
                var cut = PoweredAscentGuidance.DecideTerminalCut(energy, targetEnergy, minR, maxR, targetRadius, targetPeRadius, band);

                bool oldGateWouldCut = energy >= targetEnergy && 0.5 * (minR + maxR) >= targetRadius;
                bool ok = cut == PoweredAscentGuidance.TerminalCutDecision.BlockedOnShape && oldGateWouldCut;
                pass &= ok;
                Console.WriteLine($"  (A) 89x322 vs 200-circ: old gate cuts={oldGateWouldCut}, new={cut}  band={band / 1000.0:F1} km  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (B) Validated converged RSS insertion: osc-circular ~190.5 km (real ~180x190, mean on 185).
            {
                double R = EarthMeanRadius + 190500.0;
                Vector3d r = R * rHat;
                Vector3d v = Math.Sqrt(EarthMu / R) * tHat;
                double targetRadius = EarthMeanRadius + 185000.0, targetPeRadius = targetRadius;
                double energy = -EarthMu / (2.0 * R), targetEnergy = -EarthMu / (2.0 * targetRadius);

                J2Propagator.RadiusExtremes(r, v, EarthMu, EarthJ2, EarthRefRadius, pole, 7000.0, 10.0, out double minR, out double maxR);
                double band = PoweredAscentGuidance.TerminalShapeBandMeters(EarthJ2, EarthRefRadius, R);
                var cut = PoweredAscentGuidance.DecideTerminalCut(energy, targetEnergy, minR, maxR, targetRadius, targetPeRadius, band);

                bool ok = cut == PoweredAscentGuidance.TerminalCutDecision.Complete;
                pass &= ok;
                Console.WriteLine($"  (B) converged RSS insertion (real {(minR - EarthMeanRadius) / 1000.0:F0}x{(maxR - EarthMeanRadius) / 1000.0:F0} vs 185): {cut}  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (C) Stock (J2=0): floor band keeps a sub-km-short Pe acceptable — no behavior regression.
            {
                double rPe = KerbinRadius + 149200.0, rAp = KerbinRadius + 151000.0;
                double a = 0.5 * (rPe + rAp);
                Vector3d r = rPe * rHat;
                Vector3d v = Math.Sqrt(KerbinMu * (2.0 / rPe - 1.0 / a)) * tHat;
                double targetRadius = KerbinRadius + 150000.0, targetPeRadius = targetRadius;
                double energy = -KerbinMu / (2.0 * a), targetEnergy = -KerbinMu / (2.0 * targetRadius);

                J2Propagator.RadiusExtremes(r, v, KerbinMu, 0.0, KerbinRadius, pole, 7000.0, 10.0, out double minR, out double maxR);
                double band = PoweredAscentGuidance.TerminalShapeBandMeters(0.0, KerbinRadius, rPe);
                var cut = PoweredAscentGuidance.DecideTerminalCut(energy, targetEnergy, minR, maxR, targetRadius, targetPeRadius, band);

                bool ok = cut == PoweredAscentGuidance.TerminalCutDecision.Complete && band == 5000.0;
                pass &= ok;
                Console.WriteLine($"  (C) stock 149.2x151 vs 150-circ: {cut}  band={band / 1000.0:F1} km (floor)  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (D) Stock, shape wrong (100x201 vs 150-circ, same energy): must block — the guard is not J2-only.
            {
                double rPe = KerbinRadius + 100000.0, rAp = KerbinRadius + 201000.0;
                double a = 0.5 * (rPe + rAp);
                Vector3d r = rPe * rHat;
                Vector3d v = Math.Sqrt(KerbinMu * (2.0 / rPe - 1.0 / a)) * tHat;
                double targetRadius = KerbinRadius + 150000.0, targetPeRadius = targetRadius;
                double energy = -KerbinMu / (2.0 * a), targetEnergy = -KerbinMu / (2.0 * targetRadius);

                J2Propagator.RadiusExtremes(r, v, KerbinMu, 0.0, KerbinRadius, pole, 7000.0, 10.0, out double minR, out double maxR);
                double band = PoweredAscentGuidance.TerminalShapeBandMeters(0.0, KerbinRadius, rPe);
                var cut = PoweredAscentGuidance.DecideTerminalCut(energy, targetEnergy, minR, maxR, targetRadius, targetPeRadius, band);

                bool ok = cut == PoweredAscentGuidance.TerminalCutDecision.BlockedOnShape;
                pass &= ok;
                Console.WriteLine($"  (D) stock 100x201 vs 150-circ: {cut}  {(ok ? "[ok]" : "[FAIL]")}");
            }

            Console.WriteLine(pass
                ? "  [PASS] shape gate blocks eccentric cuts, passes converged insertions in RSS and stock"
                : "  [FAIL] terminal shape gate regression");
            Console.WriteLine();
            if (!pass) throw new Exception("Terminal shape gate regression.");
        }

        private static PsgProblem CreateKerbinScenario()
        {
            double rotationRate = 2.0 * Math.PI / KerbinRotationPeriod;
            Vector3d bodyAngularVelocity = new Vector3d(0.0, 0.0, rotationRate);
            PsgBodyModel body = PsgBodyModel.Create(KerbinMu, KerbinRadius, bodyAngularVelocity);

            Vector3d padPosition = new Vector3d(KerbinRadius, 0.0, 0.0);
            Vector3d padVelocity = Vector3d.Cross(bodyAngularVelocity, padPosition);
            PsgInitialState initial = PsgInitialState.Create(
                padPosition,
                padVelocity,
                219800.0,
                0.0);

            double insertionRadius = KerbinRadius + TargetInsertionAltitude;
            Vector3d orbitNormal = new Vector3d(0.0, 0.0, 1.0);
            PsgTarget target = PsgTarget.Create(
                KerbinMu,
                insertionRadius,
                insertionRadius,
                insertionRadius,
                orbitNormal,
                0.0,
                0.0,
                false);

            var stage = new PoweredStageInfo
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                Stage = 0,
                PhaseIndex = 0,
                IsCurrentOrFutureStage = true,
                StartMass = 219.8,
                EndMass = 39.8,
                VacuumSpecificImpulse = 295.0,
                CurrentSpecificImpulse = 295.0,
                VacuumThrust = 3749.44,
                CurrentThrust = 3749.44,
                MinimumThrust = 0.0,
                MinimumThrottle = 0.0,
                BurnTimeSeconds = 139.0,
                VacuumDeltaV = 4941.0,
                CurrentDeltaV = 4941.0
            };

            PsgPhase[] phases = PsgPhase.FromPoweredStages(new[] { stage });
            Vector3d initialThrustDirection = padPosition.normalized;

            return PsgProblem.Create(initial, body, target, phases, initialThrustDirection);
        }

        private static void PrintSolution(PsgProblem problem, PsgSolution solution)
        {
            PsgSolutionPoint terminal = solution.TerminalState();
            if (terminal == null)
            {
                Console.WriteLine("No terminal state.");
                return;
            }

            OrbitSummary orbit = OrbitSummary.FromState(
                problem.BodyGravParameter,
                problem.BodyRadiusMeters,
                terminal.RelativePosition,
                terminal.RelativeVelocity);

            Console.WriteLine("Solution:");
            Console.WriteLine("  tgo: " + solution.TimeToGo(solution.StartUniversalTime).ToString("F2") + " s");
            Console.WriteLine("  terminal radius: " + terminal.RelativePosition.magnitude.ToString("F1") + " m");
            Console.WriteLine("  terminal speed: " + terminal.RelativeVelocity.magnitude.ToString("F2") + " m/s");
            Console.WriteLine("  AP: " + orbit.ApoapsisAlt.ToString("F1") + " m");
            Console.WriteLine("  PE: " + orbit.PeriapsisAlt.ToString("F1") + " m");
            Console.WriteLine("  eccentricity: " + orbit.Eccentricity.ToString("F6"));
            Console.WriteLine();

            Console.WriteLine("Guidance samples:");
            int sampleCount = 8;
            for (int i = 0; i <= sampleCount; i++)
            {
                double t = solution.StartUniversalTime +
                           solution.TimeToGo(solution.StartUniversalTime) * i / sampleCount;
                PsgGuidanceVector guidance = solution.InertialGuidance(t);
                Console.WriteLine(
                    "  t+" + (t - solution.StartUniversalTime).ToString("F1").PadLeft(6) +
                    "s throttle=" + guidance.Throttle.ToString("F2") +
                    " dir=(" +
                    guidance.InertialDirection.x.ToString("F3") + ", " +
                    guidance.InertialDirection.y.ToString("F3") + ", " +
                    guidance.InertialDirection.z.ToString("F3") + ")");
            }
        }

        // FuelSim replaces stock DeltaVStageInfo, which mis-attributes shared/incompatible fuel and
        // inflates stageBurnTime by 1/minThrottle. Four hand-computed cases: (A) tanker payload exclusion
        // plus the x6 burn-time regression, (B) serial staging with engine activation and honest mass
        // chaining, (C) crossfeed drop-tank with equal-split draining and the never-drop-reachable-fuel
        // staging rule, (D) RealFuels residuals floor.
        private static void RunFuelSimChecks()
        {
            Console.WriteLine("FuelSim stage propellant simulation:");
            bool pass = true;

            const int Fuel = 1;               // propellant the engines burn
            const int PayloadFuel = 2;        // incompatible fuel carried as cargo
            const double Density = 0.001;     // tons/unit -> 1 unit = 1 kg

            // (A) Saturn-V-tanker repro: 5 t usable fuel, 98 t MethaLox payload the engine cannot burn.
            {
                var vessel = new SimVessel();
                vessel.SetCurrentStage(0);

                SimPart core = MakeSimPart(vessel, "core", 30.0, 0);
                AddSimResource(core, Fuel, 5000.0, Density);
                SimPart payload = MakeSimPart(vessel, "payload", 2.0, 0);
                AddSimResource(payload, PayloadFuel, 98000.0, Density);
                LinkParts(core, payload);

                AddSimEngine(core, Fuel, Density, FlowMode.StagePriorityFlow, 0.275, 1.0 / 6.0, 436.0);
                vessel.FinalizeBuild();

                SimStageStats[] stats = StagePropellantSimulator.Run(vessel);
                SimStageStats s = stats[0];

                double expectStart = (30.0 + 5.0 + 2.0 + 98.0) * 1000.0;
                double expectTime = 5.0 / 0.275;

                bool ok = stats.Length == 1
                          && Near(s.StartMassKg, expectStart, 1e-6)
                          && Near(s.BurnablePropellantKg, 5000.0, 1e-6)
                          && Near(s.EndMassKg, expectStart - 5000.0, 1e-6)
                          && Near(s.FullThrottleBurnTimeSeconds, expectTime, 1e-9)
                          && Near(s.MinimumThrottle, 1.0 / 6.0, 1e-9)
                          && Near(s.VacuumIspSeconds, 436.0, 1e-6);
                pass &= ok;
                Console.WriteLine($"  (A) tanker: burnable={s.BurnablePropellantKg:F1} kg (expect 5000), burnT={s.FullThrottleBurnTimeSeconds:F2} s (expect {expectTime:F2}; x6 bug would say {expectTime * 6.0:F0}), minThl={s.MinimumThrottle:F4}  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (B) serial staging: booster burns out, decoupler drops it, upper engine lights.
            {
                var vessel = new SimVessel();
                vessel.SetCurrentStage(1);

                SimPart upper = MakeSimPart(vessel, "upper", 4.0, 0);
                AddSimResource(upper, Fuel, 3000.0, Density);
                SimPart decoupler = MakeSimPart(vessel, "decoupler", 0.5, 0);
                SimPart booster = MakeSimPart(vessel, "booster", 6.0, 1);
                AddSimResource(booster, Fuel, 6000.0, Density);
                LinkParts(upper, decoupler);
                LinkParts(decoupler, booster);
                decoupler.Decoupler = new Decoupler { Staged = true, StagingEnabled = true, AttachedPart = booster };

                AddSimEngine(upper, Fuel, Density, FlowMode.NoFlow, 0.15, 0.0, 320.0);
                AddSimEngine(booster, Fuel, Density, FlowMode.NoFlow, 0.2, 0.0, 300.0);
                vessel.FinalizeBuild();

                SimStageStats[] stats = StagePropellantSimulator.Run(vessel);

                // stage 1: 19.5 t total, booster burns its 6 t in 30 s; stage 0: booster's 6 t dry mass
                // is dropped, upper burns its 3 t in 20 s.
                bool ok = stats.Length == 2
                          && stats[0].Stage == 1
                          && Near(stats[0].StartMassKg, 19500.0, 1e-6)
                          && Near(stats[0].BurnablePropellantKg, 6000.0, 1e-6)
                          && Near(stats[0].FullThrottleBurnTimeSeconds, 30.0, 1e-9)
                          && stats[1].Stage == 0
                          && Near(stats[1].StartMassKg, 7500.0, 1e-6)
                          && Near(stats[1].BurnablePropellantKg, 3000.0, 1e-6)
                          && Near(stats[1].FullThrottleBurnTimeSeconds, 20.0, 1e-9);
                pass &= ok;
                Console.WriteLine($"  (B) serial staging: S1 start={stats[0].StartMassKg:F0} kg burnT={stats[0].FullThrottleBurnTimeSeconds:F1} s, S0 start={stats[1].StartMassKg:F0} kg burnT={stats[1].FullThrottleBurnTimeSeconds:F1} s  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (C) crossfeed drop-tank: one engine feeding from its own tank plus a droppable tank at equal
            // priority. Draining splits equally; staging must wait for the drop tank to run dry.
            {
                var vessel = new SimVessel();
                vessel.SetCurrentStage(1);

                SimPart core = MakeSimPart(vessel, "core", 10.0, 1);
                AddSimResource(core, Fuel, 10000.0, Density);
                SimPart decoupler = MakeSimPart(vessel, "decoupler", 0.4, 0);
                SimPart dropTank = MakeSimPart(vessel, "droptank", 1.5, 0);
                AddSimResource(dropTank, Fuel, 2000.0, Density);
                LinkParts(core, decoupler);
                LinkParts(decoupler, dropTank);
                decoupler.Decoupler = new Decoupler { Staged = true, StagingEnabled = true, AttachedPart = dropTank };

                AddSimEngine(core, Fuel, Density, FlowMode.StackPrioritySearch, 0.1, 0.0, 350.0);
                core.CrossFeedPartSet.Add(core);
                core.CrossFeedPartSet.Add(dropTank);
                vessel.FinalizeBuild();

                SimStageStats[] stats = StagePropellantSimulator.Run(vessel);

                // stage 1 ends when the drop tank empties: 2 t from the drop tank + 2 t from the core
                // (equal split at 50 units/s each). Stage 0 = the core's remaining 8 t, drop tank gone.
                bool ok = stats.Length == 2
                          && Near(stats[0].BurnablePropellantKg, 4000.0, 1e-6)
                          && Near(stats[0].FullThrottleBurnTimeSeconds, 40.0, 1e-6)
                          && Near(stats[1].StartMassKg, 18400.0, 1e-6)
                          && Near(stats[1].BurnablePropellantKg, 8000.0, 1e-6)
                          && Near(stats[1].FullThrottleBurnTimeSeconds, 80.0, 1e-6);
                pass &= ok;
                Console.WriteLine($"  (C) drop-tank: S1 burnable={stats[0].BurnablePropellantKg:F0} kg/{stats[0].FullThrottleBurnTimeSeconds:F1} s (priority-bug would stage at 2000), S0 start={stats[1].StartMassKg:F0} kg burnable={stats[1].BurnablePropellantKg:F0} kg  {(ok ? "[ok]" : "[FAIL]")}");
            }

            // (D) RealFuels residuals: 5% of tank capacity is unusable and stays aboard as mass.
            {
                var vessel = new SimVessel();
                vessel.SetCurrentStage(0);

                SimPart core = MakeSimPart(vessel, "core", 30.0, 0);
                AddSimResource(core, Fuel, 5000.0, Density);
                AddSimEngine(core, Fuel, Density, FlowMode.StagePriorityFlow, 0.275, 0.0, 436.0, 0.05);
                vessel.FinalizeBuild();

                SimStageStats[] stats = StagePropellantSimulator.Run(vessel);
                SimStageStats s = stats[0];

                bool ok = Near(s.BurnablePropellantKg, 4750.0, 1e-6)
                          && Near(s.EndMassKg, 30000.0 + 250.0, 1e-6)
                          && Near(s.FullThrottleBurnTimeSeconds, 4.75 / 0.275, 1e-6);
                pass &= ok;
                Console.WriteLine($"  (D) residuals 5%: burnable={s.BurnablePropellantKg:F1} kg (expect 4750), end={s.EndMassKg:F1} kg (expect 30250)  {(ok ? "[ok]" : "[FAIL]")}");
            }

            Console.WriteLine(pass
                ? "  [PASS] payload excluded, burn times honest, staging and residual rules hold"
                : "  [FAIL] FuelSim regression");
            Console.WriteLine();
            if (!pass) throw new Exception("FuelSim regression.");
        }

        private static SimPart MakeSimPart(SimVessel vessel, string name, double dryTons, int inverseStage)
        {
            var part = new SimPart(vessel, name)
            {
                DryMassTons = dryTons,
                InverseStage = inverseStage,
                StagingOn = true,
                IsRoot = vessel.Parts.Count == 0
            };
            vessel.Parts.Add(part);
            return part;
        }

        private static void AddSimResource(SimPart part, int id, double amountUnits, double densityTonsPerUnit)
        {
            part.Resources[id] = new Resource
            {
                Id = id,
                Amount = amountUnits,
                MaxAmount = amountUnits,
                Density = densityTonsPerUnit,
                Free = false
            };
        }

        private static void LinkParts(SimPart a, SimPart b)
        {
            a.Links.Add(b);
            b.Links.Add(a);
        }

        private static SimEngine AddSimEngine(
            SimPart part, int resourceId, double densityTonsPerUnit, FlowMode flowMode,
            double maxFlowTonsPerSec, double minThrottle, double vacIsp, double residuals = 0.0)
        {
            var engine = new SimEngine
            {
                Part = part,
                IsEnabled = true,
                IsOperational = false,
                ThrustLimiter = 1.0,
                MaxFuelFlowTons = maxFlowTonsPerSec,
                MinFuelFlowTons = maxFlowTonsPerSec * minThrottle,
                VacuumIsp = vacIsp,
                ModuleResiduals = residuals
            };
            engine.Propellants.Add(new EnginePropellant
            {
                ResourceId = resourceId,
                Ratio = 1.0,
                DensityTonsPerUnit = densityTonsPerUnit,
                FlowMode = flowMode
            });
            engine.Initialize();
            part.Engines.Add(engine);
            return engine;
        }

        private static bool Near(double actual, double expected, double tolerance)
        {
            return Math.Abs(actual - expected) <= tolerance;
        }

        private sealed class OrbitSummary
        {
            public double ApoapsisAlt { get; private set; }
            public double PeriapsisAlt { get; private set; }
            public double Eccentricity { get; private set; }

            public static OrbitSummary FromState(
                double mu,
                double bodyRadius,
                Vector3d relativePosition,
                Vector3d relativeVelocity)
            {
                double r = relativePosition.magnitude;
                double v2 = relativeVelocity.sqrMagnitude;
                Vector3d h = Vector3d.Cross(relativePosition, relativeVelocity);
                double h2 = h.sqrMagnitude;
                double energy = 0.5 * v2 - mu / r;

                if (energy >= 0.0 || h2 <= 0.0)
                {
                    return new OrbitSummary
                    {
                        ApoapsisAlt = double.PositiveInfinity,
                        PeriapsisAlt = double.NaN,
                        Eccentricity = double.NaN
                    };
                }

                double semiMajorAxis = -mu / (2.0 * energy);
                double eccentricitySquared = Math.Max(0.0, 1.0 + 2.0 * energy * h2 / (mu * mu));
                double eccentricity = Math.Sqrt(eccentricitySquared);

                return new OrbitSummary
                {
                    ApoapsisAlt = semiMajorAxis * (1.0 + eccentricity) - bodyRadius,
                    PeriapsisAlt = semiMajorAxis * (1.0 - eccentricity) - bodyRadius,
                    Eccentricity = eccentricity
                };
            }
        }
    }
}
