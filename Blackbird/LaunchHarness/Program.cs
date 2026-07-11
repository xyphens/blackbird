using System;
using System.Collections.Generic;
using Blackbird.Guidance;
using Blackbird.Planning;
using UnityEngine;

namespace Blackbird.LaunchHarness
{
    // Offline launch-window check: builds RSS-Earth body constants + a simulated target orbit + the Cape, runs
    // LaunchWindowSolver, and prints the best ascending / descending candidate. No KSP runtime — it exercises
    // the pure solver only, so we can sanity-check the candidate logic before wiring it into the in-game panel.
    internal static class Program
    {
        // RSS Earth.
        private const double EarthMu = 3.986004418e14;
        private const double EarthRadius = 6378136.3;
        private const double EarthAtmosphere = 140000.0;
        private const double EarthSiderealDay = 86164.0905;
        private const double EarthJ2 = 1.082636e-03;

        // Cape Canaveral.
        private const double CapeLatitude = 28.6;
        private const double CapeLongitude = -80.6;

        private static int Main()
        {
            // Target: 400 km circular at 28.6 deg inclination. Three scenarios vary where the target sits
            // relative to the site's plane-crossing so we see "behind" (chase low) and "ahead" (wait high).
            Scenario("i=51.6, target trailing (insert ahead -> wait high)", 51.6, 0.0, 200.0, EarthJ2, CapeLatitude);
            Scenario("i=51.6, target leading (chase from low)", 51.6, 0.0, 20.0, EarthJ2, CapeLatitude);
            Scenario("i=28.6 == Cape lat (tangent / due-east window)", 28.6, 0.0, 95.0, EarthJ2, CapeLatitude);
            // KSC-on-the-equator case from the in-game test: near-equatorial target, pad at lat ~0. Heading
            // must come out ~90 (east), NOT ~270 (the left-handed-frame retrograde bug).
            Scenario("i=0.18 near-equatorial, pad lat ~0 (heading must be ~90)", 0.18, 0.0, 200.0, EarthJ2, 0.1);
            // Control: same geometry with J2=0 must close to ~0 km (proves the phasing math; isolates J2 as the
            // sole source of the multi-orbit miss above).
            Scenario("CONTROL J2=0 (Keplerian closure, expect predCA ~0)", 51.6, 0.0, 20.0, 0.0, CapeLatitude);

            CheckEccentricTargetUsesSemiMajorOrbit();
            CheckLaunchPlaneError();
            CheckPrecessionAwareWindow();
            CheckSaturnVLaunchWindow();
            CheckAscentPitchSchedule();
            CheckAscentPredictorReproducesFlight();
            CheckAscentShootingSolver();
            return 0;
        }

        // P0 calibration: proves the AscentIntegrator physics (central gravity + RSS drag + variable mass) by
        // reproducing the flown Starship ascent from trajectory.log. Driven by the flown thrust ATTITUDE (backed out
        // of the log's 2D momentum balance), the engine must reproduce the flown surface flight-path angle and speed
        // -- headline: too-vertical 76 deg at 34 km. Validates the engine, not a fitted constant; drag is a ~2%
        // perturbation here, so the FPA reproduction is a gravity+steering check. Tolerances: 1.5 deg FPA / 4% speed.
        private static void CheckAscentPredictorReproducesFlight()
        {
            Console.WriteLine();
            Console.WriteLine("=== Ascent predictor: reproduce flown Starship ascent (RK4 engine calibration) ===");

            // Vehicle: Starship+SuperHeavy stage 1 from the log (thrust ~74.3 MN, mdot ~22192 kg/s => Isp ~341 s).
            const double thrust = 74.3e6, mdot = 22192.0, cdA = 55.0;
            const double startAltM = 137.4, startMassKg = 5209193.0;

            // Flown thrust attitude (pitch from local vertical, deg) vs altitude (m), backed out of the log.
            AttPoint[] attitude =
            {
                new AttPoint(138, 0.31),   new AttPoint(2147, 0.12),  new AttPoint(4181, 0.46),  new AttPoint(6213, 2.85),
                new AttPoint(8419, 4.25),  new AttPoint(10659, 5.63), new AttPoint(12792, 7.00), new AttPoint(15182, 8.55),
                new AttPoint(17292, 9.92), new AttPoint(19586, 11.40),new AttPoint(22074, 13.01),new AttPoint(24763, 14.76),
                new AttPoint(26916, 16.15),new AttPoint(29192, 17.63),new AttPoint(31592, 19.20),new AttPoint(34119, 20.85),
                new AttPoint(36778, 22.61),new AttPoint(39570, 24.45),new AttPoint(42497, 26.38),new AttPoint(44525, 27.71),
                new AttPoint(46615, 29.09),new AttPoint(48768, 30.50),new AttPoint(50983, 31.97),new AttPoint(53261, 33.47),
                new AttPoint(55602, 35.02),new AttPoint(58006, 36.62),new AttPoint(60473, 38.25),new AttPoint(63004, 39.93),
                new AttPoint(65596, 41.64),new AttPoint(68251, 43.41),new AttPoint(70968, 45.21),new AttPoint(73744, 47.07),
            };

            var cfg = new AscentDynamicsConfig
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                DragAreaCd = cdA,
                Stages = new[] { new AscentStage { ThrustNewtons = thrust, MassFlowKgPerSec = mdot, PropellantKg = 3.0e6, JettisonMassKg = 0.0 } },
                DensityAtAltitude = RssAtmosphere.Density,
                SteeringPitchFromVerticalDeg = st => InterpAttitude(attitude, st.AltitudeMeters(EarthRadius)),
            };

            var integrator = new AscentIntegrator(cfg);
            AscentState s0 = new AscentState { T = 0, X = EarthRadius + startAltM, Y = 0, Vx = 0, Vy = 0, MassKg = startMassKg };
            List<AscentState> path = integrator.Integrate(s0, 0.25, st => st.AltitudeMeters(EarthRadius) >= 62000.0, 4000);

            // Flown reference checkpoints (alt m, surface speed, surface FPA deg) from trajectory.log.
            Ref[] refs =
            {
                new Ref(6000, 280.2, 88.93),  new Ref(12000, 434.0, 85.81), new Ref(20000, 603.2, 82.17),
                new Ref(34000, 872.4, 76.21), new Ref(45000, 1079.9, 71.58),new Ref(60000, 1343.7, 65.73),
            };

            foreach (Ref r in refs)
            {
                AscentState sim = StateAtAltitude(path, r.AltM);
                double fpa = sim.FlightPathAngleDeg, vs = sim.SurfaceSpeed;
                double dFpa = fpa - r.FpaDeg, dVsPct = 100.0 * (vs - r.Vs) / r.Vs;
                bool pass = Math.Abs(dFpa) <= 1.5 && Math.Abs(dVsPct) <= 4.0;

                Console.WriteLine(string.Format(
                    "  {0,3:F0} km  sim FPA {1,5:F1}  Vs {2,5:F0}  |  flown FPA {3,5:F1}  Vs {4,5:F0}  |  dFPA {5,5:F2}  dVs {6,5:F1}%  {7}",
                    r.AltM / 1000.0, fpa, vs, r.FpaDeg, r.Vs, dFpa, dVsPct, pass ? "PASS" : "FAIL"));
                if (!pass) throw new Exception(string.Format(
                    "Ascent predictor did not reproduce flight at {0:F0} km (dFPA {1:F2} deg, dVs {2:F1}%).",
                    r.AltM / 1000.0, dFpa, dVsPct));
            }
        }

        // The kick-angle shooting solver run against the validated AscentIntegrator with the fuller phased steering
        // law. Asserts STRUCTURAL invariants that hold regardless of the (still-open) handover/target choice, so the
        // check never needs rewriting when those settle: (1) the kick->orbit-FPA map is monotonic; (2) orbit-frame FPA
        // is below surface-frame FPA at handover (body co-rotation tilts velocity toward horizontal); (3) bisection
        // lands a requested orbit-FPA target and reproduces it on re-integration; (4) the no-solution guard fires for
        // an unreachable bracket. The steering here is the predictive kick law, so this exercises the engine as a
        // true forward predictor. The 60 deg target is only a probe value, not a committed guidance number.
        private static void CheckAscentShootingSolver()
        {
            Console.WriteLine();
            Console.WriteLine("=== Ascent shooting solver: kick angle -> orbit-frame FPA at handover (fuller phased law) ===");

            var io = new AscentShootingInputs
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                DragAreaCd = 55.0,
                Stages = new[] { new AscentStage { ThrustNewtons = 74.3e6, MassFlowKgPerSec = 22192.0, PropellantKg = 3.0e6, JettisonMassKg = 0.0 } },
                DensityAtAltitude = RssAtmosphere.Density,
                InitialState = new AscentState { T = 0, X = EarthRadius + 137.4, Y = 0, Vx = 0, Vy = 0, MassKg = 5209193.0 },
                TowerClearanceAltitudeMeters = 1000.0,
                HandoverAltitudeMeters = 45000.0,
                TargetFlightPathAngleDeg = 60.0,   // probe target (orbit frame); not a committed value
                MaxKickDeg = 45.0,
                StepSeconds = 0.5,
                MaxSteps = 4000,
                RotationRateRadPerSec = 2.0 * Math.PI / EarthSiderealDay,
                LaunchLatitudeCos = Math.Cos(CapeLatitude * Math.PI / 180.0),
                DynamicPressureShiftPa = 2000.0
            };

            // (1) monotonic in kick, and (2) orbit FPA < surface FPA at handover (co-rotation invariant)
            double prevOrbit = double.MaxValue;
            foreach (double k in new[] { 3.0, 4.0, 5.0, 8.0, 12.0 })
            {
                AscentHandoverPrediction p = AscentShootingSolver.PredictHandover(io, k);
                Console.WriteLine(string.Format("  kick {0,4:F0}  ->  surface {1,6:F2}  orbit {2,6:F2}", k, p.SurfaceFpaDeg, p.OrbitFpaDeg));
                if (p.OrbitFpaDeg > prevOrbit + 1e-6) throw new Exception("Kick->orbit-FPA map is not monotonic.");
                if (p.OrbitFpaDeg > p.SurfaceFpaDeg + 1e-6) throw new Exception("Orbit FPA is not below surface FPA (co-rotation invariant violated).");
                prevOrbit = p.OrbitFpaDeg;
            }

            // (3) converge to the probe orbit-FPA target, and reproduce it on an independent re-integration
            AscentShootingResult r = AscentShootingSolver.Solve(io);
            bool solveOk = r.Converged && Math.Abs(r.AchievedFlightPathAngleDeg - io.TargetFlightPathAngleDeg) <= 0.3
                           && r.KickAngleDeg > 0.0 && r.KickAngleDeg < io.MaxKickDeg;
            Console.WriteLine(string.Format("  solve orbit-FPA {0:F0} deg: kick {1:F3}  achieved {2:F3}  [{3}]  {4}",
                io.TargetFlightPathAngleDeg, r.KickAngleDeg, r.AchievedFlightPathAngleDeg, r.Status, solveOk ? "PASS" : "FAIL"));
            if (!solveOk) throw new Exception("Shooting solve did not reach target orbit FPA.");
            double recheck = AscentShootingSolver.PredictHandover(io, r.KickAngleDeg).OrbitFpaDeg;
            if (Math.Abs(recheck - io.TargetFlightPathAngleDeg) > 0.3) throw new Exception("Solved kick does not reproduce target orbit FPA.");

            // (4) no-solution guard: a bracket too small to turn far enough must be flagged, not silently clamped
            io.MaxKickDeg = 3.0;
            io.TargetFlightPathAngleDeg = 20.0;   // demand a hard turn the 3 deg bracket cannot deliver by 45 km
            AscentShootingResult g = AscentShootingSolver.Solve(io);
            bool guardOk = !g.Converged && g.Status.IndexOf("TWR", StringComparison.OrdinalIgnoreCase) >= 0;
            Console.WriteLine(string.Format("  guard (maxKick 3 deg, target 20 deg): converged={0}  status={1}  {2}", g.Converged, g.Status, guardOk ? "PASS" : "FAIL"));
            if (!guardOk) throw new Exception("No-solution guard did not fire for an unreachable target.");
        }

        private struct AttPoint { public double Alt; public double PsiDeg; public AttPoint(double a, double p) { Alt = a; PsiDeg = p; } }
        private struct Ref { public double AltM; public double Vs; public double FpaDeg; public Ref(double a, double v, double f) { AltM = a; Vs = v; FpaDeg = f; } }

        // Linear interpolation of the backed-out attitude table (pitch from vertical) at an altitude.
        private static double InterpAttitude(AttPoint[] t, double alt)
        {
            if (alt <= t[0].Alt) return t[0].PsiDeg;
            if (alt >= t[t.Length - 1].Alt) return t[t.Length - 1].PsiDeg;
            for (int i = 0; i < t.Length - 1; i++)
            {
                if (alt < t[i].Alt || alt > t[i + 1].Alt) continue;
                double f = (alt - t[i].Alt) / (t[i + 1].Alt - t[i].Alt);
                return t[i].PsiDeg + f * (t[i + 1].PsiDeg - t[i].PsiDeg);
            }
            return t[t.Length - 1].PsiDeg;
        }

        // First stepped state at or above the target altitude (0.25 s steps, so no interpolation needed).
        private static AscentState StateAtAltitude(List<AscentState> path, double altM)
        {
            for (int i = 1; i < path.Count; i++)
                if (path[i].AltitudeMeters(EarthRadius) >= altM) return path[i];
            return path[path.Count - 1];
        }

        // Characterizes the pre-PSG gravity-turn pitch schedule. Golden values pin the CURRENT commanded curve
        // (turn start/end + pitch at 34 km) so the magic-number constants in GetTurnStartAltitude /
        // GetHorizontalFlightAltitude can be reworked later without silently drifting the flown profile. RSS
        // Earth's 34 km pitch must also land in the real-world 55-72 deg band (flight data ~60-68). Pure geometry.
        private static void CheckAscentPitchSchedule()
        {
            Console.WriteLine();
            Console.WriteLine("=== Ascent pitch-schedule characterization (locks the validated gravity turn) ===");

            var cases = new[]
            {
                new { Name = "RSS Earth 209km", R = EarthRadius, Atm = 140000.0, Ap = 209000.0, Pe = 209000.0, RealWorld = true,  Start = 2100.0, End = 119000.0, P34 = 65.4 },
                new { Name = "RSS Earth 350km", R = EarthRadius, Atm = 140000.0, Ap = 350000.0, Pe = 350000.0, RealWorld = true,  Start = 2100.0, End = 140000.0, P34 = 69.2 },
                new { Name = "Kerbin 100km",    R = 600000.0,   Atm =  70000.0, Ap = 100000.0, Pe = 100000.0, RealWorld = false, Start =  480.0, End =  59500.0, P34 = 38.9 },
                new { Name = "Kerbin 80km",     R = 600000.0,   Atm =  70000.0, Ap =  80000.0, Pe =  80000.0, RealWorld = false, Start =  480.0, End =  59500.0, P34 = 38.9 },
            };

            foreach (var c in cases)
            {
                double insertion = Math.Max(100.0, Math.Min(c.Ap, c.Pe));
                double start = AscentProfileSolver.GetTurnStartAltitude(0.0, c.R, c.Atm, insertion);
                double end = Math.Max(start + 1000.0, AscentProfileSolver.GetHorizontalFlightAltitude(c.Atm, insertion));
                double p12 = AscentProfileSolver.GetSolvedPitchAtAltitude(12000.0, start, end);
                double p34 = AscentProfileSolver.GetSolvedPitchAtAltitude(34000.0, start, end);
                double p50 = AscentProfileSolver.GetSolvedPitchAtAltitude(50000.0, start, end);

                bool golden = Math.Abs(start - c.Start) < 1.0 && Math.Abs(end - c.End) < 1.0 && Math.Abs(p34 - c.P34) < 0.3;
                bool realWorldOk = !c.RealWorld || (p34 >= 55.0 && p34 <= 72.0);
                bool monotonic = p12 >= p34 && p34 >= p50;
                bool pass = golden && realWorldOk && monotonic;

                Console.WriteLine(string.Format(
                    "  {0,-16} start {1,5:F0}m  end {2,6:F0}m | pitch@ 12km {3,4:F1}  34km {4,4:F1}  50km {5,4:F1}  {6}",
                    c.Name, start, end, p12, p34, p50, pass ? "PASS" : "FAIL"));
                if (!pass) throw new Exception(string.Format(
                    "Pitch-schedule characterization drift: {0} (start {1:F0}/exp {2:F0}, end {3:F0}/exp {4:F0}, p34 {5:F2}/exp {6:F2})",
                    c.Name, start, c.Start, end, c.End, p34, c.P34));
            }
        }

        // Faithful replay of the 2026-07-03 Saturn V flight in KSP's OWN frame (+Y pole, left-handed cross), using
        // the exact pad vector and target orbit-normal pulled from psg.log. In flight the pad sat 25.4 deg OUT of
        // the target plane at the launch instant (measured from the log) -> 33.7 deg insertion wedge. This asks the
        // pure solver: given this geometry, WHERE are the real crossings, and is the pad actually in-plane there?
        // We independently re-derive pad-out-of-plane at each recommended UT (not trusting the candidate's own
        // PlaneErrorDeg), then repeat with the PHYSICAL (+pole) normal to see whether feeding the raw KSP anti-pole
        // normal (what LaunchPlanner does via GetOrbitNormal) corrupts the window.
        private static void CheckSaturnVLaunchWindow()
        {
            Console.WriteLine();
            Console.WriteLine("=== Saturn V replay (KSP +Y frame; pad + target normal from psg.log 19:13) ===");

            Vector3d pole = new Vector3d(0, 1, 0);                 // BodyAngularVelocity dir from the log
            Vector3d pad  = new Vector3d(-1791948.50966043, 3050623.42134499, 5298465.21216374); // pad @ launch, body-rel
            Vector3d nGame = new Vector3d(0.456838686690156, -0.877865370566086, 0.143703881311822); // as fed to PSG (anti-pole)

            double padLat = Math.Asin(Math.Max(-1.0, Math.Min(1.0, Vector3d.Dot(pad.normalized, pole)))) * 180.0 / Math.PI;
            double normAngle = Vector3d.Angle(nGame, pole);
            double padOopNow = Math.Asin(Clamp01(Math.Abs(Vector3d.Dot(pad.normalized, nGame.normalized)))) * 180.0 / Math.PI;
            Console.WriteLine(string.Format(
                "  pad latitude {0:F2} deg | target normal {1:F1} deg from +pole ({2}) | pad out-of-plane NOW {3:F2} deg (flight: 25.42)",
                padLat, normAngle, normAngle > 90.0 ? "ANTI-pole, raw KSP cross" : "+pole physical", padOopNow));

            RunSaturnCase("as-flown (raw anti-pole normal)", pole, pad, nGame);
            RunSaturnCase("physical (+pole) normal",         pole, pad, -nGame);
        }

        private static void RunSaturnCase(string label, Vector3d pole, Vector3d pad, Vector3d targetNormal)
        {
            Console.WriteLine("  -- " + label + " --");

            // Circular 350 km mothership whose Vector3d.Cross(r,v) reproduces targetNormal (BAC-CAB is handedness-
            // consistent as long as one cross convention is used throughout, which Unity/KSP guarantee).
            double r = EarthRadius + 350000.0;
            Vector3d rHat = Vector3d.Cross(pole, targetNormal).normalized;      // a point on the target plane (its node line)
            Vector3d targetPos = r * rHat;
            Vector3d targetVel = Math.Sqrt(EarthMu / r) * Vector3d.Cross(targetNormal, rHat).normalized;

            LaunchWindowSolver.Inputs input = new LaunchWindowSolver.Inputs
            {
                Mu = EarthMu, BodyRadius = EarthRadius, AtmosphereDepth = EarthAtmosphere,
                RotationPeriodSeconds = EarthSiderealDay, J2 = EarthJ2, J2ReferenceRadius = EarthRadius,
                Pole = pole, RotationAxis = pole,
                CurrentUt = 0.0, LaunchSitePosition = pad,
                TargetPosition = targetPos, TargetVelocity = targetVel,
                TargetOrbitNormal = targetNormal,
                TargetSemiMajorRadius = r, TargetPeriodSeconds = 2.0 * Math.PI * Math.Sqrt(r * r * r / EarthMu),
                AscentDurationSeconds = 500.0, RemainingDeltaV = 9500.0
            };

            List<LaunchWindowSolver.Candidate> candidates = LaunchWindowSolver.Solve(input);
            if (candidates.Count == 0) { Console.WriteLine("     (no plane crossings found)"); return; }

            foreach (LaunchWindowSolver.Candidate c in candidates)
            {
                if (!c.IsValid) { Console.WriteLine(string.Format("     {0,-11} INVALID: {1}", c.NodeName, c.Reason)); continue; }

                // Independent truth: carry the pad forward by body rotation to the recommended UT and measure its
                // true out-of-plane angle vs the target plane. The solver's PlaneErrorDeg must agree with this.
                Vector3d padAtLaunch = RotateAbout(pad, pole, 2.0 * Math.PI * c.SecondsUntilLaunch / EarthSiderealDay);
                double oopIndep = Math.Asin(Clamp01(Math.Abs(Vector3d.Dot(padAtLaunch.normalized, targetNormal.normalized)))) * 180.0 / Math.PI;
                string verdict = oopIndep < 2.0 ? "OK (pad in plane)" : "BAD (pad NOT in plane)";
                Console.WriteLine(string.Format(
                    "     {0,-11} launch +{1,6:F1} min  hdg {2,5:F1}  candPlaneErr {3,4:F1}  indepPadOOP {4,5:F2} deg -> {5}",
                    c.NodeName, c.SecondsUntilLaunch / 60.0, c.AzimuthDeg, c.PlaneErrorDeg, oopIndep, verdict));
            }
        }

        private static void CheckEccentricTargetUsesSemiMajorOrbit()
        {
            Console.WriteLine();
            Console.WriteLine("=== Eccentric target sizing (phasing uses semi-major orbit, not current radius) ===");

            double peAlt = 300000.0;
            double apAlt = 900000.0;
            double semiMajorRadius = EarthRadius + 0.5 * (peAlt + apAlt);
            double period = 2.0 * Math.PI * Math.Sqrt(semiMajorRadius * semiMajorRadius * semiMajorRadius / EarthMu);

            bool found = false;
            for (double trueAnomalyDeg = 0.0; trueAnomalyDeg < 360.0; trueAnomalyDeg += 15.0)
            {
                StateFromElements(semiMajorRadius, (apAlt - peAlt) / (2.0 * EarthRadius + apAlt + peAlt),
                    51.6, 0.0, trueAnomalyDeg, EarthMu, out Vector3d targetPos, out Vector3d targetVel);

                LaunchWindowSolver.Inputs corrected = EccentricBaseInput(targetPos, targetVel);
                corrected.TargetSemiMajorRadius = semiMajorRadius;
                corrected.TargetPeriodSeconds = period;

                LaunchWindowSolver.Inputs legacy = corrected;
                legacy.TargetSemiMajorRadius = 0.0;
                legacy.TargetPeriodSeconds = 0.0;

                LaunchWindowSolver.Candidate cNew = FirstValid(LaunchWindowSolver.Solve(corrected));
                LaunchWindowSolver.Candidate cOld = FirstValid(LaunchWindowSolver.Solve(legacy));
                if (!cNew.IsValid || !cOld.IsValid) continue;

                double delta = Math.Abs(cOld.PhasingApoapsisAlt - cNew.PhasingApoapsisAlt);
                if (delta < 200000.0) continue;

                found = true;
                bool newInBand = cNew.PhasingApoapsisAlt <= 0.5 * (peAlt + apAlt) + 300000.0;
                bool oldWasCurrentRadiusBiased = delta > 200000.0;
                string verdict = newInBand && oldWasCurrentRadiusBiased ? "PASS" : "FAIL";
                Console.WriteLine(string.Format(
                    "  nu {0,5:F0} deg  corrected {1,5:F0} km  legacy {2,5:F0} km  delta {3,5:F0} km  -> {4}",
                    trueAnomalyDeg, cNew.PhasingApoapsisAlt / 1000.0, cOld.PhasingApoapsisAlt / 1000.0,
                    delta / 1000.0, verdict));

                if (verdict != "PASS") throw new Exception("Eccentric target sizing regression.");
                break;
            }

            if (!found) throw new Exception("Eccentric target sizing check did not find an adversarial case.");
        }

        private static LaunchWindowSolver.Inputs EccentricBaseInput(Vector3d targetPos, Vector3d targetVel)
        {
            return new LaunchWindowSolver.Inputs
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                AtmosphereDepth = EarthAtmosphere,
                RotationPeriodSeconds = EarthSiderealDay,
                J2 = 0.0,
                J2ReferenceRadius = EarthRadius,
                Pole = Pole,
                CurrentUt = 0.0,
                LaunchSitePosition = SiteVector(EarthRadius, CapeLatitude, CapeLongitude),
                TargetPosition = targetPos,
                TargetVelocity = targetVel,
                TargetOrbitNormal = Vector3d.Cross(targetPos, targetVel).normalized,
                AscentDurationSeconds = 500.0,
                RemainingDeltaV = 9500.0
            };
        }

        private static LaunchWindowSolver.Candidate FirstValid(List<LaunchWindowSolver.Candidate> candidates)
        {
            foreach (LaunchWindowSolver.Candidate c in candidates)
                if (c.IsValid) return c;
            return default(LaunchWindowSolver.Candidate);
        }

        // The crossing search must meet the target plane AS IT WILL BE at the launch UT, not the frozen now-plane.
        // Two invariants, both adversarial: (1) at the chosen launch UT the pad lies in the candidate's returned
        // AT-LAUNCH plane (out-of-plane ~0) — the whole point; (2) that normal has rotated vs the now-normal by the
        // analytic J2 secular amount sin(i)*|Omega_dot|*wait. Control J2=0 must show ZERO rotation (no regression in
        // the stock case, and proves the propagation adds no spurious precession).
        private static void CheckPrecessionAwareWindow()
        {
            Console.WriteLine();
            Console.WriteLine("=== Precession-aware launch window (pad in AT-LAUNCH plane; normal tracks J2 node) ===");
            RunPrecessionCase("J2 on (RSS)", 51.6, EarthJ2);
            RunPrecessionCase("J2 off control", 51.6, 0.0);
        }

        private static void RunPrecessionCase(string label, double inclinationDeg, double j2)
        {
            double r = EarthRadius + 400000.0;
            StateFromElements(r, inclinationDeg, 0.0, 200.0, EarthMu, out Vector3d tPos, out Vector3d tVel);
            Vector3d nowNormal = Vector3d.Cross(tPos, tVel).normalized;

            LaunchWindowSolver.Inputs input = new LaunchWindowSolver.Inputs
            {
                Mu = EarthMu, BodyRadius = EarthRadius, AtmosphereDepth = EarthAtmosphere,
                RotationPeriodSeconds = EarthSiderealDay, J2 = j2, J2ReferenceRadius = EarthRadius, Pole = Pole,
                CurrentUt = 0.0, LaunchSitePosition = SiteVector(EarthRadius, CapeLatitude, CapeLongitude),
                TargetPosition = tPos, TargetVelocity = tVel,
                TargetOrbitNormal = nowNormal,
                AscentDurationSeconds = 500.0, RemainingDeltaV = 9500.0
            };

            double inc = inclinationDeg * Math.PI / 180.0;
            double n = Math.Sqrt(EarthMu / (r * r * r));
            double raanRate = -1.5 * n * j2 * (EarthRadius / r) * (EarthRadius / r) * Math.Cos(inc); // rad/s, secular

            foreach (LaunchWindowSolver.Candidate c in LaunchWindowSolver.Solve(input))
            {
                if (!c.IsValid) continue;
                Vector3d padAtLaunch = RotateAbout(input.LaunchSitePosition, Pole,
                    2.0 * Math.PI * c.SecondsUntilLaunch / EarthSiderealDay).normalized;
                double padOutOfPlaneDeg = Math.Asin(Clamp01(Math.Abs(
                    Vector3d.Dot(padAtLaunch, c.LaunchUtOrbitNormal)))) * 180.0 / Math.PI;
                double measuredDeg = Math.Acos(Math.Max(-1.0, Math.Min(1.0,
                    Vector3d.Dot(nowNormal, c.LaunchUtOrbitNormal)))) * 180.0 / Math.PI;
                double dRaan = Math.Abs(raanRate) * c.SecondsUntilLaunch;
                double expectedDeg = 2.0 * Math.Asin(Clamp01(Math.Sin(inc) * Math.Sin(dRaan / 2.0))) * 180.0 / Math.PI;

                bool inPlane = padOutOfPlaneDeg < 0.2;
                bool precessionOk = j2 == 0.0 ? measuredDeg < 0.05
                                              : Math.Abs(measuredDeg - expectedDeg) < Math.Max(0.3, 0.5 * expectedDeg);
                string verdict = inPlane && precessionOk ? "PASS" : "FAIL";
                Console.WriteLine(string.Format(
                    "  [{0,-15}] {1,-11} wait {2,5:F0} min  pad-out-of-plane {3:F3}  precessed {4:F3} (expect {5:F3})  -> {6}",
                    label, c.NodeName, c.SecondsUntilLaunch / 60.0, padOutOfPlaneDeg, measuredDeg, expectedDeg, verdict));
            }
        }

        private static double Clamp01(double x) => x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);

        private static Vector3d RotateAbout(Vector3d v, Vector3d axis, double angleRad)
        {
            axis = axis.normalized;
            double c = Math.Cos(angleRad), s = Math.Sin(angleRad);
            return v * c + Vector3d.Cross(axis, v) * s + axis * (Vector3d.Dot(axis, v) * (1.0 - c));
        }

        private static readonly Vector3d Pole = new Vector3d(0, 0, 1);

        // Attributes the ~10 deg rel-inc / ~22 deg RAAN launch-plane miss to its real source. The Earth-rotation
        // azimuth correction is sub-degree for an i~lat tangent launch, so this measures rel-inc vs the two real
        // suspects — launch-TIMING slip (huge dRAAN/dt at the tangent) and J2 DIFFERENTIAL nodal regression over
        // the phasing time — plus the azimuth axis as a control. Pure geometry/secular model, no KSP runtime.
        private static void CheckLaunchPlaneError()
        {
            Console.WriteLine();
            Console.WriteLine("=== Launch-plane error attribution (i~lat tangent: inc 28.64, lat 28.6, RSS) ===");

            double mu = EarthMu, R = EarthRadius;
            double omega = 2.0 * Math.PI / EarthSiderealDay;
            double phi = 28.6, iT = 28.64;
            double rIns = R + 200000.0;
            double v = Math.Sqrt(mu / rIns);
            double vRot = omega * R * Math.Cos(phi * Math.PI / 180.0);

            double beta0Rad = Math.Asin(Math.Min(1.0, Math.Cos(iT * Math.PI / 180.0) / Math.Cos(phi * Math.PI / 180.0)));
            double beta0 = beta0Rad * 180.0 / Math.PI;             // uncorrected inertial azimuth, flown as surface hdg

            // Target plane = what a PERFECT, no-rotation launch at the planned heading achieves. Self-consistent
            // with AchievedNormal by construction, so flying beta0 with vRot=0 gives rel-inc 0 (model self-check)
            // and any nonzero result is purely the physical effect under study.
            Vector3d nTarget = AchievedNormal(phi, beta0, 0.0, v, 0.0);

            // Axis A: azimuth. Uncorrected = the planned inertial azimuth flown as a surface heading. Best = the
            // surface heading that actually hits the target plane (the true Earth-rotation correction), found by
            // search so the self-check is exact. The gap is the correctable plane error — tangent geometry blows
            // a sub-degree heading change up into several degrees of rel-inc.
            double riUncorr = RelIncDeg(AchievedNormal(phi, beta0, 0.0, v, vRot), nTarget);
            double bestBeta = beta0, bestRi = double.MaxValue;
            for (double b = beta0 - 10.0; b <= beta0 + 10.0; b += 0.005)
            {
                double ri = RelIncDeg(AchievedNormal(phi, b, 0.0, v, vRot), nTarget);
                if (ri < bestRi) { bestRi = ri; bestBeta = b; }
            }
            Console.WriteLine($"  azimuth:  uncorrected hdg {beta0:F2} deg -> rel-inc {riUncorr:F2} deg" +
                              $"   |   best hdg {bestBeta:F2} deg -> rel-inc {bestRi:F3} deg   (correctable {riUncorr - bestRi:F2} deg)");

            // Axis B: launch-timing slip (fly the planned heading from a pad rotated by omega*dt).
            Console.WriteLine("  timing slip (fly planned hdg):");
            foreach (double dt in new[] { -600.0, -300.0, -120.0, -60.0, 0.0, 60.0, 120.0, 300.0, 600.0 })
            {
                double lonDeg = omega * dt * 180.0 / Math.PI;
                Console.WriteLine($"    dt {dt,6:F0} s ({lonDeg,5:F2} deg) -> rel-inc {RelIncDeg(AchievedNormal(phi, beta0, lonDeg, v, vRot), nTarget),6:F2} deg");
            }

            // Axis C: J2 differential nodal regression (chaser 281 km vs target 310 km) over phasing time.
            Console.WriteLine("  J2 differential regression (chaser 281km vs target 310km, i=28.6):");
            double odChaser = NodalRate(mu, R, EarthJ2, R + 281000.0, phi);
            double odTarget = NodalRate(mu, R, EarthJ2, R + 310000.0, phi);
            foreach (double hours in new[] { 1.0, 3.0, 6.0, 12.0 })
            {
                double dRaan = (odChaser - odTarget) * hours * 3600.0;
                Console.WriteLine($"    {hours,4:F0} h  dRAAN {dRaan * 180.0 / Math.PI,6:F2} deg -> rel-inc {RelIncFromRaan(iT, dRaan),5:F2} deg");
            }
        }

        // Orbital plane normal achieved by flying surface heading betaDeg from a pad at lonDeg, with Earth rotation
        // added to the inertial velocity (the open-loop launch the guidance actually flies).
        private static Vector3d AchievedNormal(double latDeg, double betaDeg, double lonDeg, double v, double vRot)
        {
            Vector3d r = SiteVector(EarthRadius, latDeg, lonDeg);
            Vector3d up = r.normalized;
            Vector3d east = Vector3d.Cross(Pole, up).normalized;
            Vector3d north = Vector3d.Cross(up, east);
            double b = betaDeg * Math.PI / 180.0;
            Vector3d vel = v * (Math.Sin(b) * east + Math.Cos(b) * north) + vRot * east;
            return Vector3d.Cross(r, vel).normalized;
        }

        private static double RelIncDeg(Vector3d a, Vector3d b)
        {
            double ang = Vector3d.Angle(a, b);
            return Math.Min(ang, 180.0 - ang);
        }

        // Secular J2 nodal regression rate (rad/s) for a circular orbit.
        private static double NodalRate(double mu, double R, double j2, double a, double iDeg)
        {
            double n = Math.Sqrt(mu / (a * a * a));
            return -1.5 * n * j2 * (R / a) * (R / a) * Math.Cos(iDeg * Math.PI / 180.0);
        }

        // Relative inclination (deg) between two planes of equal inclination iDeg differing by dRaan (rad).
        private static double RelIncFromRaan(double iDeg, double dRaanRad)
        {
            double i = iDeg * Math.PI / 180.0;
            double c = Math.Cos(i) * Math.Cos(i) + Math.Sin(i) * Math.Sin(i) * Math.Cos(dRaanRad);
            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, c))) * 180.0 / Math.PI;
        }

        private static void Scenario(string title, double inclinationDeg, double lanDeg, double argLatDeg, double j2, double launchLatDeg)
        {
            Console.WriteLine();
            Console.WriteLine("=== " + title + " ===");

            double r = EarthRadius + 400000.0;
            StateFromElements(r, inclinationDeg, lanDeg, argLatDeg, EarthMu,
                out Vector3d targetPos, out Vector3d targetVel);

            LaunchWindowSolver.Inputs input = new LaunchWindowSolver.Inputs
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                AtmosphereDepth = EarthAtmosphere,
                RotationPeriodSeconds = EarthSiderealDay,
                J2 = j2,
                J2ReferenceRadius = EarthRadius,
                Pole = new Vector3d(0, 0, 1),

                CurrentUt = 0.0,
                LaunchSitePosition = SiteVector(EarthRadius, launchLatDeg, CapeLongitude),

                TargetPosition = targetPos,
                TargetVelocity = targetVel,
                TargetOrbitNormal = Vector3d.Cross(targetPos, targetVel).normalized,

                AscentDurationSeconds = 500.0,
                RemainingDeltaV = 9500.0
            };

            List<LaunchWindowSolver.Candidate> candidates = LaunchWindowSolver.Solve(input);
            if (candidates.Count == 0) { Console.WriteLine("  (no plane crossings found)"); return; }

            foreach (LaunchWindowSolver.Candidate c in candidates) Print(c);
        }

        private static void Print(LaunchWindowSolver.Candidate c)
        {
            if (!c.IsValid)
            {
                Console.WriteLine(string.Format("  {0,-11}  launch in {1,5:F0} min  -> INVALID: {2}",
                    c.NodeName, c.SecondsUntilLaunch / 60.0, c.Reason));
                return;
            }
            Console.WriteLine(string.Format(
                "  {0,-11}  launch in {1,5:F0} min  hdg {2,5:F1}  phase {3,7:F1} deg  plane {4,4:F1} deg",
                c.NodeName, c.SecondsUntilLaunch / 60.0, c.AzimuthDeg, c.PhaseErrorDeg, c.PlaneErrorDeg));
            Console.WriteLine(string.Format(
                "               phasing {0,4:F0} km x {1,4:F0} orbits  dV {2,5:F0} m/s (rem {3,5:F0})  predCA {4,7:F0} km  score {5:F0}",
                c.PhasingApoapsisAlt / 1000.0, c.OrbitsToRendezvous, c.EstimatedDeltaVUsed,
                c.RemainingDeltaV, c.PredictedClosestApproachMeters / 1000.0, c.Score));
        }

        // Launch-site inertial position (pole = +Z, longitude from +X) at the harness epoch.
        private static Vector3d SiteVector(double radius, double latDeg, double lonDeg)
        {
            double lat = latDeg * Math.PI / 180.0, lon = lonDeg * Math.PI / 180.0;
            double cosLat = Math.Cos(lat);
            return radius * new Vector3d(cosLat * Math.Cos(lon), cosLat * Math.Sin(lon), Math.Sin(lat));
        }

        // Circular state vector from inclination / RAAN / argument-of-latitude (deg).
        private static void StateFromElements(double radius, double incDeg, double lanDeg, double argLatDeg,
            double mu, out Vector3d position, out Vector3d velocity)
        {
            double i = incDeg * Math.PI / 180.0;
            double O = lanDeg * Math.PI / 180.0;
            double u = argLatDeg * Math.PI / 180.0;
            double cosI = Math.Cos(i), sinI = Math.Sin(i);

            Vector3d rHat = new Vector3d(
                Math.Cos(O) * Math.Cos(u) - Math.Sin(O) * Math.Sin(u) * cosI,
                Math.Sin(O) * Math.Cos(u) + Math.Cos(O) * Math.Sin(u) * cosI,
                Math.Sin(u) * sinI);
            Vector3d vHat = new Vector3d(
                -Math.Cos(O) * Math.Sin(u) - Math.Sin(O) * Math.Cos(u) * cosI,
                -Math.Sin(O) * Math.Sin(u) + Math.Cos(O) * Math.Cos(u) * cosI,
                Math.Cos(u) * sinI);

            position = radius * rHat;
            velocity = Math.Sqrt(mu / radius) * vHat;
        }

        private static void StateFromElements(double semiMajorRadius, double eccentricity, double incDeg,
            double lanDeg, double trueAnomalyDeg, double mu, out Vector3d position, out Vector3d velocity)
        {
            double i = incDeg * Math.PI / 180.0;
            double O = lanDeg * Math.PI / 180.0;
            double nu = trueAnomalyDeg * Math.PI / 180.0;
            double cosI = Math.Cos(i), sinI = Math.Sin(i);
            double p = semiMajorRadius * (1.0 - eccentricity * eccentricity);
            double r = p / (1.0 + eccentricity * Math.Cos(nu));

            Vector3d pHat = new Vector3d(Math.Cos(O), Math.Sin(O), 0.0);
            Vector3d qHat = new Vector3d(-Math.Sin(O) * cosI, Math.Cos(O) * cosI, sinI);

            position = r * (Math.Cos(nu) * pHat + Math.Sin(nu) * qHat);
            velocity = Math.Sqrt(mu / p) * (-Math.Sin(nu) * pHat + (eccentricity + Math.Cos(nu)) * qHat);
        }
    }
}
