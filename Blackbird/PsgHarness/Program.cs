using System;
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

            RunEarthJ2Check();
            RunEarthShapeSweep();
            RunCutoffLeakDemo();
            RunJ2CutoffCheck();
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
                KspStage = kspStage,
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
                KspStage = 0,
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
