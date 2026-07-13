using System;
using System.Collections.Generic;
using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.OpenLoop;
using Blackbird.Planning;
using Blackbird.Psg;
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
            CheckAscentTargetFromPsg();
            CheckAscentApoapsisLoft();
            CheckSolveAcrossBodies();
            CheckAscentIloadEmbedFrame();
            CheckAscentIloadPsgPadBoot();
            CheckAscentIloadEval();
            CheckAscentIloadBuildSweep();
            CheckAscentIloadFailurePaths();
            CheckAscentIloadLowTwrVehicle();
            return 0;
        }

        private const double KerbinMu = 3.5316e12;
        private const double KerbinRadius = 600000.0;
        private const double KerbinSiderealDay = 21549.425;

        // Proves the forward-shooting root-finder (AscentShootingSolver's engine) adapts the kick per BODY with no
        // hardcoded shape constant. For the same body-general target (a real-world surface FPA at the same fraction of
        // the atmosphere), the solver must find a DIFFERENT kick on Kerbin than on Earth -- Kerbin needs a larger kick
        // for the same handover (its shorter atmosphere / lower orbital speed under-turns) -- and both profiles must
        // reach the target without lofting (coast apoapsis climbing, bounded). This is the go/no-go that a fixed
        // kshape would fail and the solver passes: one mechanism, two bodies, zero body-specific tuning.
        private static void CheckSolveAcrossBodies()
        {
            Console.WriteLine();
            Console.WriteLine("=== Solve across bodies (kick adapts per body; no fixed kshape) ===");

            const double targetSurfaceFpa = 55.0;   // real-world-like handover angle; same on both bodies
            double[] earthKicks = new double[3];
            double[] kerbinKicks = new double[3];
            double[] twrs = { 1.5, 1.8, 2.2 };

            Console.WriteLine("  body    TWR   kick   surfaceFPA(target 55)   coastApo(km)");
            for (int i = 0; i < twrs.Length; i++)
            {
                earthKicks[i] = RunBodyCase("Earth ", twrs[i], EarthMu, EarthRadius, EarthSiderealDay,
                    RssAtmosphere.Density, 5209193.0, 341.0, 55.0, 34000.0, targetSurfaceFpa);
                kerbinKicks[i] = RunBodyCase("Kerbin", twrs[i], KerbinMu, KerbinRadius, KerbinSiderealDay,
                    KerbinAtmosphere.DensityAt, 100000.0, 300.0, 1.0, 17000.0, targetSurfaceFpa);
            }

            bool adapts = true;
            for (int i = 0; i < twrs.Length; i++)
                if (!(kerbinKicks[i] > earthKicks[i] + 1.0)) adapts = false;   // Kerbin needs a materially larger kick

            Console.WriteLine("  " + (adapts ? "PASS" : "FAIL") + " (Kerbin kick > Earth kick for every TWR -> the solver adapts per body)");
            if (!adapts) throw new Exception("Solve did not adapt the kick across bodies (a fixed kshape would be wrong).");
        }

        // Solves the kick for one (body, vehicle, TWR) to a surface-FPA target, then reports + validates the profile.
        private static double RunBodyCase(string body, double twr, double mu, double bodyRadius, double siderealDay,
            Func<double, double> density, double massKg, double ispSeconds, double dragAreaCd, double sampleAltM, double targetFpa)
        {
            double g = mu / (bodyRadius * bodyRadius);
            double thrust = twr * massKg * g;
            double mdot = thrust / (ispSeconds * 9.80665);

            var io = new AscentShootingInputs
            {
                Mu = mu,
                BodyRadius = bodyRadius,
                DragAreaCd = dragAreaCd,
                Stages = new[] { new AscentStage { ThrustNewtons = thrust, MassFlowKgPerSec = mdot, PropellantKg = massKg * 0.62, JettisonMassKg = 0.0 } },
                DensityAtAltitude = density,
                InitialState = new AscentState { T = 0, X = bodyRadius + 137.0, Y = 0, Vx = 0, Vy = 0, MassKg = massKg },
                TowerClearanceAltitudeMeters = 1000.0,
                HandoverAltitudeMeters = sampleAltM,          // measure the target FPA here
                TargetFlightPathAngleDeg = targetFpa,
                MaxKickDeg = 30.0,
                StepSeconds = 0.25,
                MaxSteps = 8000,
                RotationRateRadPerSec = 2.0 * Math.PI / siderealDay,
                LaunchLatitudeCos = 1.0,                       // both pads near the equator here
                DynamicPressureShiftPa = 2000.0
            };

            double kick = SolveKickForSurfaceFpa(io, targetFpa);
            AscentHandoverPrediction p = AscentShootingSolver.PredictHandover(io, kick);
            double coastApo = CoastApoapsisAt(io, kick);

            bool nonLoft = coastApo > 0.0 && coastApo < 100000.0;   // bounded + climbing (not escaping/absurd)
            bool ok = kick > 0.0 && kick < io.MaxKickDeg && Math.Abs(p.SurfaceFpaDeg - targetFpa) <= 0.5 && nonLoft;
            Console.WriteLine(string.Format("  {0}  {1:F1}  {2,5:F2}   {3,6:F1}                 {4,6:F0}   {5}",
                body, twr, kick, p.SurfaceFpaDeg, coastApo, ok ? "ok" : "BAD"));
            if (!ok) throw new Exception(string.Format("Cross-body case failed: {0} TWR {1:F1} (kick {2:F2}, FPA {3:F1}, coastApo {4:F0}).",
                body, twr, kick, p.SurfaceFpaDeg, coastApo));
            return kick;
        }

        // Harness bisection on the app engine (PredictHandover) targeting SURFACE FPA -- the same forward-shooting
        // root-find as AscentShootingSolver.Solve, with a surface predicate for the body-general target.
        private static double SolveKickForSurfaceFpa(AscentShootingInputs io, double targetFpa)
        {
            double lo = 0.0, hi = io.MaxKickDeg, mid = 0.0;
            for (int i = 0; i < 60; i++)
            {
                mid = 0.5 * (lo + hi);
                double fpa = AscentShootingSolver.PredictHandover(io, mid).SurfaceFpaDeg;
                if (Math.Abs(fpa - targetFpa) <= 0.03) break;
                if (fpa > targetFpa) lo = mid; else hi = mid;   // more vertical than target -> kick harder
            }
            return mid;
        }

        // Coast apoapsis (km) at the handover altitude for a solved kick -- the non-loft signal.
        private static double CoastApoapsisAt(AscentShootingInputs io, double kick)
        {
            var cfg = new AscentDynamicsConfig
            {
                Mu = io.Mu, BodyRadius = io.BodyRadius, DragAreaCd = io.DragAreaCd, Stages = io.Stages,
                DensityAtAltitude = io.DensityAtAltitude,
                SteeringPitchFromVerticalDeg = st => AscentShootingSolver.PhasedSteeringPitchDeg(st, io, kick)
            };
            var integrator = new AscentIntegrator(cfg);
            AscentState s = io.InitialState;
            for (int i = 0; i < io.MaxSteps; i++)
            {
                if (s.AltitudeMeters(io.BodyRadius) >= io.HandoverAltitudeMeters) break;
                s = integrator.Step(s, io.StepSeconds);
            }
            return CoastApoapsisKm(s, io.Mu, io.BodyRadius, io.RotationRateRadPerSec, io.LaunchLatitudeCos);
        }

        // PSG optimal orbit-frame FPA (deg) vs altitude (m), from trajectory.log proj rows. Rises to a ~28.5 deg peak
        // near 52 km (co-rotation makes it near-horizontal at the pad), then eases toward insertion. Reference fixture.
        private static readonly PsgFpaPoint[] PsgExpectedFpa =
        {
            new PsgFpaPoint(137, 2.38),    new PsgFpaPoint(318, 4.26),    new PsgFpaPoint(846, 7.29),
            new PsgFpaPoint(1739, 9.88),   new PsgFpaPoint(3075, 12.27),  new PsgFpaPoint(4927, 14.46),
            new PsgFpaPoint(7384, 16.51),  new PsgFpaPoint(10546, 18.43), new PsgFpaPoint(14531, 20.27),
            new PsgFpaPoint(19467, 22.02), new PsgFpaPoint(25507, 23.71), new PsgFpaPoint(32819, 25.35),
            new PsgFpaPoint(41606, 26.93), new PsgFpaPoint(52098, 28.48), new PsgFpaPoint(64589, 27.81),
            new PsgFpaPoint(107367, 25.79),new PsgFpaPoint(145279, 21.95),
        };

        // PSG optimal coast apoapsis (km) vs altitude (m), from the same proj rows (energy+angular-momentum of the
        // drag-free optimal state). The non-lofted reference: the atmospheric law must not bank apoapsis above this.
        private static readonly PsgCoastPoint[] PsgExpectedCoastApoKm =
        {
            new PsgCoastPoint(137, 0.2),    new PsgCoastPoint(317, 0.4),    new PsgCoastPoint(845, 1.1),
            new PsgCoastPoint(1739, 2.3),   new PsgCoastPoint(3075, 4.2),   new PsgCoastPoint(4926, 7.0),
            new PsgCoastPoint(7384, 11.0),  new PsgCoastPoint(10545, 16.4), new PsgCoastPoint(14530, 23.7),
            new PsgCoastPoint(19466, 33.5), new PsgCoastPoint(25507, 46.4), new PsgCoastPoint(32819, 63.6),
            new PsgCoastPoint(41606, 86.5), new PsgCoastPoint(52098, 117.1),new PsgCoastPoint(64589, 146.8),
            new PsgCoastPoint(107367, 182.9),new PsgCoastPoint(145279, 205.7),
        };

        // Sources the shooting solver's target from PSG's OWN expected trajectory instead of a hardcoded FPA. PSG's
        // projected (drag-free optimal) path -- from the reference flight's proj rows in trajectory.log, orbit-frame
        // FPA = asin(dAlt/dt / |Vinertial|) -- is the state the atmospheric law should hand off onto, so there is no
        // loft/discontinuity. In flight this target is read live from PsgSolution (RelativePosition/RelativeVelocity);
        // this table is the offline fixture that validates the coupling. Asserts a representative handover converges
        // to a sane kick hitting PSG's target; reports the mid-climb tracking deviation as a diagnostic (a single kick
        // runs MORE horizontal than PSG's optimal mid-climb -- anti-loft -- then converges to PSG's FPA at handover).
        private static void CheckAscentTargetFromPsg()
        {
            Console.WriteLine();
            Console.WriteLine("=== Ascent target from PSG optimal (couple solver target to PSG's expected FPA) ===");

            PsgFpaPoint[] psg = PsgExpectedFpa;

            AscentShootingInputs BaseInputs() => new AscentShootingInputs
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                DragAreaCd = 55.0,
                Stages = new[] { new AscentStage { ThrustNewtons = 74.3e6, MassFlowKgPerSec = 22192.0, PropellantKg = 3.0e6, JettisonMassKg = 0.0 } },
                DensityAtAltitude = RssAtmosphere.Density,
                InitialState = new AscentState { T = 0, X = EarthRadius + 137.4, Y = 0, Vx = 0, Vy = 0, MassKg = 5209193.0 },
                TowerClearanceAltitudeMeters = 1000.0,
                MaxKickDeg = 45.0,
                StepSeconds = 0.5,
                MaxSteps = 6000,
                RotationRateRadPerSec = 2.0 * Math.PI / EarthSiderealDay,
                LaunchLatitudeCos = Math.Cos(CapeLatitude * Math.PI / 180.0),
                DynamicPressureShiftPa = 2000.0
            };

            foreach (double handover in new[] { 40000.0, 50000.0, 60000.0, 70000.0 })
            {
                AscentShootingInputs io = BaseInputs();
                io.HandoverAltitudeMeters = handover;
                io.TargetFlightPathAngleDeg = InterpPsgFpa(psg, handover);

                AscentShootingResult r = AscentShootingSolver.Solve(io);
                double climbDev = MaxClimbDeviationFromPsg(io, r.KickAngleDeg, psg);
                Console.WriteLine(string.Format(
                    "  handover {0,2:F0} km  PSG target {1,5:F1}  ->  kick {2,5:F2}  achieved {3,5:F2}  [{4}]  midclimb max|traj-PSG| {5,4:F1} deg",
                    handover / 1000.0, io.TargetFlightPathAngleDeg, r.KickAngleDeg, r.AchievedFlightPathAngleDeg, r.Status, climbDev));
            }

            // Assert the representative 50 km handover converges to a sane kick hitting PSG's own target.
            AscentShootingInputs io50 = BaseInputs();
            io50.HandoverAltitudeMeters = 50000.0;
            io50.TargetFlightPathAngleDeg = InterpPsgFpa(psg, 50000.0);
            AscentShootingResult r50 = AscentShootingSolver.Solve(io50);
            bool ok = r50.Converged
                      && r50.KickAngleDeg > 0.0 && r50.KickAngleDeg < io50.MaxKickDeg
                      && Math.Abs(r50.AchievedFlightPathAngleDeg - io50.TargetFlightPathAngleDeg) <= 0.3;
            Console.WriteLine("  " + (ok ? "PASS" : "FAIL") + " (50 km handover solves to PSG target with a sane kick)");
            if (!ok) throw new Exception("PSG-coupled target did not solve to a sane kick at the 50 km handover.");
        }

        private struct PsgFpaPoint { public double Alt; public double FpaDeg; public PsgFpaPoint(double a, double f) { Alt = a; FpaDeg = f; } }

        // Linear interpolation of PSG's expected orbit-FPA table at an altitude.
        private static double InterpPsgFpa(PsgFpaPoint[] t, double alt)
        {
            if (alt <= t[0].Alt) return t[0].FpaDeg;
            if (alt >= t[t.Length - 1].Alt) return t[t.Length - 1].FpaDeg;
            for (int i = 0; i < t.Length - 1; i++)
            {
                if (alt < t[i].Alt || alt > t[i + 1].Alt) continue;
                double f = (alt - t[i].Alt) / (t[i + 1].Alt - t[i].Alt);
                return t[i].FpaDeg + f * (t[i + 1].FpaDeg - t[i].FpaDeg);
            }
            return t[t.Length - 1].FpaDeg;
        }

        // Max deviation (deg) of the kicked trajectory's orbit-frame FPA from PSG's expected FPA over the climb.
        private static double MaxClimbDeviationFromPsg(AscentShootingInputs io, double kickDeg, PsgFpaPoint[] psg)
        {
            var cfg = new AscentDynamicsConfig
            {
                Mu = io.Mu,
                BodyRadius = io.BodyRadius,
                DragAreaCd = io.DragAreaCd,
                Stages = io.Stages,
                DensityAtAltitude = io.DensityAtAltitude,
                SteeringPitchFromVerticalDeg = st => AscentShootingSolver.PhasedSteeringPitchDeg(st, io, kickDeg)
            };

            var integrator = new AscentIntegrator(cfg);
            AscentState s = io.InitialState;
            double maxDev = 0.0;
            for (int i = 0; i < io.MaxSteps; i++)
            {
                double alt = s.AltitudeMeters(io.BodyRadius);
                if (alt >= io.HandoverAltitudeMeters) break;
                if (alt >= 2000.0)
                {
                    double orbit = AscentShootingSolver.OrbitFlightPathAngleDeg(s, io.RotationRateRadPerSec, io.LaunchLatitudeCos);
                    double dev = Math.Abs(orbit - InterpPsgFpa(psg, alt));
                    if (dev > maxDev) maxDev = dev;
                }
                s = integrator.Step(s, io.StepSeconds);
            }
            return maxDev;
        }

        // Apoapsis-loft check: does the single-kick law bank more apoapsis than PSG's non-lofted optimal? Coast
        // apoapsis (energy + angular momentum of the current orbit-frame state) is compared to PSG's optimal over the
        // climb. The solved single-kick tracks AT OR BELOW PSG (anti-loft) -> a second kick-altitude parameter is not
        // needed. A deliberately lofted late-kick (vertical hold to 30 km) is included so the metric is proven to
        // detect real loft, not pass vacuously.
        private static void CheckAscentApoapsisLoft()
        {
            Console.WriteLine();
            Console.WriteLine("=== Ascent apoapsis-loft (single-kick vs PSG optimal; is a 2nd parameter needed?) ===");

            const double handover = 50000.0;

            AscentShootingInputs Inputs() => new AscentShootingInputs
            {
                Mu = EarthMu, BodyRadius = EarthRadius, DragAreaCd = 55.0,
                Stages = new[] { new AscentStage { ThrustNewtons = 74.3e6, MassFlowKgPerSec = 22192.0, PropellantKg = 3.0e6, JettisonMassKg = 0.0 } },
                DensityAtAltitude = RssAtmosphere.Density,
                InitialState = new AscentState { T = 0, X = EarthRadius + 137.4, Y = 0, Vx = 0, Vy = 0, MassKg = 5209193.0 },
                TowerClearanceAltitudeMeters = 1000.0,
                HandoverAltitudeMeters = handover,
                TargetFlightPathAngleDeg = InterpPsgFpa(PsgExpectedFpa, handover),
                MaxKickDeg = 45.0, StepSeconds = 0.5, MaxSteps = 6000,
                RotationRateRadPerSec = 2.0 * Math.PI / EarthSiderealDay,
                LaunchLatitudeCos = Math.Cos(CapeLatitude * Math.PI / 180.0),
                DynamicPressureShiftPa = 2000.0
            };

            // Single kick solved to PSG's target: must not bank apoapsis above PSG's optimal.
            AscentShootingInputs io = Inputs();
            AscentShootingResult r = AscentShootingSolver.Solve(io);
            double singleOverLoft = MaxCoastOverLoftKm(io, r.KickAngleDeg, PsgExpectedCoastApoKm);

            // Deliberately lofted counterexample: vertical hold all the way to 30 km, then kick -> should over-loft.
            AscentShootingInputs lofted = Inputs();
            lofted.TowerClearanceAltitudeMeters = 30000.0;
            double loftedOverLoft = MaxCoastOverLoftKm(lofted, 8.0, PsgExpectedCoastApoKm);

            Console.WriteLine(string.Format("  single-kick (K {0:F2} -> PSG {1:F1} deg): max coast-apoapsis over-loft vs PSG = {2:F1} km",
                r.KickAngleDeg, io.TargetFlightPathAngleDeg, singleOverLoft));
            Console.WriteLine(string.Format("  late-kick (vertical to 30 km, deliberately lofted): max over-loft vs PSG = {0:F1} km", loftedOverLoft));

            bool singleOk = singleOverLoft <= 3.0;          // single kick does not loft
            bool detects = loftedOverLoft >= 4.0;           // metric flags a genuinely lofted ascent (non-vacuous)
            Console.WriteLine("  " + (singleOk && detects ? "PASS" : "FAIL") + " (single-kick anti-loft; loft metric discriminates)");
            if (!singleOk) throw new Exception(string.Format("Single-kick lofts above PSG optimal by {0:F1} km.", singleOverLoft));
            if (!detects) throw new Exception("Loft metric failed to flag a deliberately lofted ascent (would be vacuous).");
        }

        private struct PsgCoastPoint { public double Alt; public double CoastApoKm; public PsgCoastPoint(double a, double c) { Alt = a; CoastApoKm = c; } }

        private static double InterpPsgCoast(PsgCoastPoint[] t, double alt)
        {
            if (alt <= t[0].Alt) return t[0].CoastApoKm;
            if (alt >= t[t.Length - 1].Alt) return t[t.Length - 1].CoastApoKm;
            for (int i = 0; i < t.Length - 1; i++)
            {
                if (alt < t[i].Alt || alt > t[i + 1].Alt) continue;
                double f = (alt - t[i].Alt) / (t[i + 1].Alt - t[i].Alt);
                return t[i].CoastApoKm + f * (t[i + 1].CoastApoKm - t[i].CoastApoKm);
            }
            return t[t.Length - 1].CoastApoKm;
        }

        // Coast apoapsis altitude (km) of the state's orbit-frame conic (surface velocity + co-rotation).
        private static double CoastApoapsisKm(AscentState s, double mu, double bodyRadius, double rotationRate, double latCos)
        {
            double r = s.Radius;
            double ex = -s.Y / r, ey = s.X / r;
            double vrot = rotationRate * r * latCos;
            double ox = s.Vx + vrot * ex, oy = s.Vy + vrot * ey;
            double energy = 0.5 * (ox * ox + oy * oy) - mu / r;
            if (energy >= 0.0) return double.PositiveInfinity;   // escape
            double h = Math.Abs(s.X * oy - s.Y * ox);
            double a = -mu / (2.0 * energy);
            double e = Math.Sqrt(Math.Max(0.0, 1.0 + 2.0 * energy * h * h / (mu * mu)));
            return (a * (1.0 + e) - bodyRadius) / 1000.0;
        }

        // Max amount (km) the kicked trajectory's coast apoapsis exceeds PSG's optimal over the climb (loft signal).
        private static double MaxCoastOverLoftKm(AscentShootingInputs io, double kickDeg, PsgCoastPoint[] psgCoast)
        {
            var cfg = new AscentDynamicsConfig
            {
                Mu = io.Mu, BodyRadius = io.BodyRadius, DragAreaCd = io.DragAreaCd, Stages = io.Stages,
                DensityAtAltitude = io.DensityAtAltitude,
                SteeringPitchFromVerticalDeg = st => AscentShootingSolver.PhasedSteeringPitchDeg(st, io, kickDeg)
            };

            var integrator = new AscentIntegrator(cfg);
            AscentState s = io.InitialState;
            double maxOver = double.NegativeInfinity;
            for (int i = 0; i < io.MaxSteps; i++)
            {
                double alt = s.AltitudeMeters(io.BodyRadius);
                if (alt >= io.HandoverAltitudeMeters) break;
                if (alt >= 4000.0)
                {
                    double apo = CoastApoapsisKm(s, io.Mu, io.BodyRadius, io.RotationRateRadPerSec, io.LaunchLatitudeCos);
                    double over = apo - InterpPsgCoast(psgCoast, alt);
                    if (over > maxOver) maxOver = over;
                }
                s = integrator.Step(s, io.StepSeconds);
            }
            return maxOver;
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

        // ================= I-load / chi-table open-loop ascent (P3) =================
        // Fixture: the flown Starship stack + 210 km circular target exactly as PSG logged them on the pad
        // (psg.log 2026-07-12 18:48:17 problem snapshot; KSP world axes: +Y pole, LEFT-handed frame).
        private const double IloadMu = 398600435436096.0;
        private const double IloadBodyRadius = 6371000.0;
        private const double IloadOmegaY = 7.292115373194e-05;
        private static readonly Vector3d IloadPad = new Vector3d(5269748.13480735, 3050629.29654613, 1874773.26428383);
        // Logged obt_velocity of the vessel AT REST on the pad — the ground truth for the embed's co-rotation term.
        private static readonly Vector3d IloadPadRestVelocity = new Vector3d(-136.710617086584, 0.0, 384.276119338767);
        private const double IloadLiftoffMassKg = 4801932.15098977;
        private const double IloadPadAltMeters = 134.85194942262;
        private const double IloadUt = 131990860.243475;
        private const double IloadHandoffAltMeters = 43000.0;   // 0.002 pressure fraction, flight-validated
        private const double IloadTargetRadius = 6581000.0;     // 210 km circular
        // Upper stage from the 18:53:06 IGNITION snapshot, not the pad snapshot: pre-launch, stock
        // DeltaVStageInfo overstates the future stage (start 1535.9/end 416.7/isp 368) — at ignition the
        // fuelsim snaps to the physical stage (booster dry ~317 t actually jettisons; real dV ~7.9 km/s).
        private const double IloadUpperStartTons = 1219.52827792525;
        private const double IloadUpperEndTons = 145.047253524969;

        private static PoweredStageInfo MakeIloadStage(int kspStage, int phaseIndex, double startMassTons,
            double endMassTons, double thrustKn, double ispSec, double minThrottle)
        {
            double mdotTonsPerSec = thrustKn / (ispSec * MathHelpers.StandardGravity);
            return new PoweredStageInfo
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                Stage = kspStage,
                PhaseIndex = phaseIndex,
                IsCurrentOrFutureStage = true,
                StartMass = startMassTons,
                EndMass = endMassTons,
                VacuumSpecificImpulse = ispSec,
                CurrentSpecificImpulse = ispSec,
                VacuumThrust = thrustKn,
                CurrentThrust = thrustKn,
                MinimumThrust = thrustKn * minThrottle,
                MinimumThrottle = minThrottle,
                BurnTimeSeconds = (startMassTons - endMassTons) / mdotTonsPerSec,
                VacuumDeltaV = 0.0,
                CurrentDeltaV = 0.0
            };
        }

        // Inputs exactly as BeginOpenLoopBuild fills them in-game: pad position and body spin in KSP world axes,
        // due-east launch (heading 90 at the Cape), target plane = the plane the resting pad already lies in
        // (Cross(r, v_rest), the tangent-launch case actually flown). boosterThrustScale synthesizes the
        // second, lower-TWR vehicle for the generality check — everything else derives from the stage list.
        private static OpenLoopInputs MakeIloadInputs(double boosterThrustScale = 1.0)
        {
            Vector3d pole = new Vector3d(0.0, 1.0, 0.0);
            Vector3d omega = pole * IloadOmegaY;
            Vector3d restVelocity = Vector3d.Cross(IloadPad, omega);      // = the logged rest obt_velocity
            Vector3d targetNormal = Vector3d.Cross(IloadPad, restVelocity).normalized;

            return new OpenLoopInputs
            {
                Mu = IloadMu,
                BodyRadiusMeters = IloadBodyRadius,
                DragAreaCd = 55.0,
                DensityAtAltitude = RssAtmosphere.Density,
                Stages = new[]
                {
                    MakeIloadStage(4, 0, 4799.94562259349, 1536.49514218216, 74432.4743886079 * boosterThrustScale, 347.000005723145, 0.4),
                    MakeIloadStage(2, 1, IloadUpperStartTons, IloadUpperEndTons, 11767.9762330475, 380.000006267421, 0.400000005662442)
                },
                LiftoffMassKg = IloadLiftoffMassKg,
                PadAltitudeMeters = IloadPadAltMeters,
                UniversalTime = IloadUt,
                PadRelativePosition = IloadPad,
                DownrangeDirection = restVelocity.normalized,
                BodyAngularVelocity = omega,
                HandoffAltitudeMeters = IloadHandoffAltMeters,
                PitchOverSpeedMps = 100.0,
                HoldVerticalUntilAltMeters = 0.0,
                Target = PsgTarget.Create(IloadMu, IloadTargetRadius, IloadTargetRadius, IloadTargetRadius,
                    targetNormal, 28.6083883588767, 236.977771247498, false, false)
            };
        }

        // PSG solved straight from the pad rest state (the in-game boot solve, which the build REQUIRES).
        // Anchors the fixture's feasibility independent of the integrator, and proves the harness's
        // by-construction target normal is the SAME plane flight's inc/LAN-constrained target resolves to.
        private static void CheckAscentIloadPsgPadBoot()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load PSG pad-boot (fixture feasibility; fixed normal == flown inc/LAN plane) ===");
            OpenLoopInputs io = MakeIloadInputs();
            PsgInitialState initial = PsgInitialState.Create(IloadPad, IloadPadRestVelocity, IloadLiftoffMassKg, IloadUt);
            PsgBodyModel body = PsgBodyModel.Create(IloadMu, IloadBodyRadius, new Vector3d(0.0, IloadOmegaY, 0.0));
            PsgPhase[] phases = PsgPhase.FromPoweredStages(io.Stages);

            PsgTarget lanTarget = PsgTarget.Create(IloadMu, IloadTargetRadius, IloadTargetRadius, IloadTargetRadius,
                Vector3d.zero, 28.6083883588767, 236.977771247498, true, false);   // exactly as flight FromPlan logged it

            var masses = new List<double>();
            foreach (var pair in new[] { new { Label = "fixed pad-rest normal", T = io.Target }, new { Label = "flown inc/LAN (free plane)", T = lanTarget } })
            {
                PsgProblem prob = PsgProblem.Create(initial, body, pair.T, phases, IloadPad.normalized);
                PsgOptimizationResult res = new PsgOptimizer().Solve(prob, null);
                if (res == null || !res.Success || res.Solution == null) throw new Exception("PSG pad boot did not converge (" + pair.Label + ").");
                PsgSolutionPoint term = res.Solution.TerminalState();
                Vector3d n = Vector3d.Cross(term.RelativePosition, term.RelativeVelocity).normalized;
                double planeToMine = Vector3d.Angle(n, io.Target.TargetOrbitNormal);
                Console.WriteLine(string.Format("  {0,-28}: injected {1:F1} t (floor {2:F1}, margin {3:F1} t)  iters {4}  tgo {5:F0} s  plane-vs-mine {6:F3} deg",
                    pair.Label, term.MassKg / 1000.0, IloadUpperEndTons, term.MassKg / 1000.0 - IloadUpperEndTons, res.Iterations,
                    res.Solution.TimeToGo(IloadUt), planeToMine));
                if (planeToMine > 0.1) throw new Exception("Pad-boot terminal plane does not match the fixture normal (" + pair.Label + ").");
                if (term.MassKg / 1000.0 - IloadUpperEndTons < 150.0) throw new Exception("Fixture lost its feasibility margin (" + pair.Label + ").");
                masses.Add(term.MassKg);
            }
            if (Math.Abs(masses[0] - masses[1]) > 2000.0)
                throw new Exception("Fixed-normal and inc/LAN targets disagree — the harness target no longer matches flight semantics.");
        }

        // The embed IS the flight-frame contract: PSG receives body-centered position + INERTIAL velocity in
        // KSP's left-handed world frame, where a surface-fixed point moves at Cross(r, omega) — pinned here
        // against the flown log's rest state, so a handedness/sign regression fails offline, not in a launch.
        private static void CheckAscentIloadEmbedFrame()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load embed frame (2D->3D embed reproduces the flown pad rest state) ===");

            OpenLoopInputs io = MakeIloadInputs();
            AscentState rest = new AscentState { T = 0, X = IloadBodyRadius + IloadPadAltMeters, Y = 0, Vx = 0, Vy = 0, MassKg = IloadLiftoffMassKg };
            OpenLoopTrajectory.EmbedState(rest, io, out Vector3d r3, out Vector3d v3);

            double posErr = (r3 - IloadPad).magnitude;
            double velErr = (v3 - IloadPadRestVelocity).magnitude;
            Console.WriteLine(string.Format("  rest embed: |dPos| {0:E2} m  |dVel| {1:E3} m/s (logged obt_velocity)", posErr, velErr));
            if (posErr > 0.01) throw new Exception("Embed does not reproduce the logged pad position.");
            if (velErr > 0.01) throw new Exception("Embed does not reproduce the logged rest inertial velocity (co-rotation sign/frame).");

            // Moving state: in-plane components must survive the embed exactly, and the co-rotation term must
            // be the only out-of-plane contribution (orthonormal er/et; guards future refactors).
            AscentState moving = new AscentState { T = 120, X = IloadBodyRadius + 40000.0, Y = 55000.0, Vx = 900.0, Vy = 1300.0, MassKg = 2.5e6 };
            OpenLoopTrajectory.EmbedState(moving, io, out Vector3d rm, out Vector3d vm);
            Vector3d er = IloadPad.normalized;
            Vector3d et = (io.DownrangeDirection - Vector3d.Dot(io.DownrangeDirection, er) * er).normalized;
            Vector3d surf = vm - Vector3d.Cross(rm, io.BodyAngularVelocity);
            bool ok = Math.Abs(rm.magnitude - moving.Radius) < 0.01
                      && Math.Abs(Vector3d.Dot(surf, er) - moving.Vx) < 1e-6
                      && Math.Abs(Vector3d.Dot(surf, et) - moving.Vy) < 1e-6;
            Console.WriteLine("  " + (ok ? "PASS" : "FAIL") + " (moving-state embed: exact in-plane components + radius)");
            if (!ok) throw new Exception("Embed corrupted the in-plane state components.");
        }

        // The contract's headline check: ONE fixed pitch rate end-to-end (integrate -> embed -> PSG solve) on the
        // flown vehicle. Asserts PSG convergence, terminal orbit = the 210 km circular target, terminal plane
        // parallel to the target normal (Cross(r,v) convention), and an injected mass inside the physical window.
        private static void CheckAscentIloadEval()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load eval (fixed 0.7 deg/s: integrate -> embed -> PSG -> injected mass) ===");

            OpenLoopInputs io = MakeIloadInputs();
            DateTime t0 = DateTime.UtcNow;
            OpenLoopCandidate c = OpenLoopTrajectory.EvaluateCandidate(io, 0.7);
            Console.WriteLine(string.Format("  valid={0} ({1})  {2:F1} s wall", c.Valid, c.Valid ? "ok" : c.Reason, (DateTime.UtcNow - t0).TotalSeconds));
            if (!c.Valid) throw new Exception("Fixed-rate eval did not produce a PSG-convergent trajectory: " + c.Reason);

            PsgSolutionPoint term = c.PsgSolution.TerminalState();
            double rMag = term.RelativePosition.magnitude;
            double vMag = term.RelativeVelocity.magnitude;
            double vCirc = Math.Sqrt(IloadMu / rMag);
            double radialSpeed = Vector3d.Dot(term.RelativePosition.normalized, term.RelativeVelocity);
            Vector3d termNormal = Vector3d.Cross(term.RelativePosition, term.RelativeVelocity).normalized;
            double planeErrDeg = Vector3d.Angle(termNormal, io.Target.TargetOrbitNormal);

            Console.WriteLine(string.Format("  injected {0,7:F1} t   tHandoff {1,5:F1} s   psgTgo {2,5:F1} s   iters {3}",
                c.InjectedMassKg / 1000.0, c.TimeToHandoffSeconds, c.PsgTimeToGoSeconds, c.PsgIterations));
            Console.WriteLine(string.Format("  terminal: alt {0,6:F1} km  |v| {1,6:F1} (circ {2,6:F1})  vRadial {3,5:F2} m/s  plane err {4:F3} deg",
                (rMag - IloadBodyRadius) / 1000.0, vMag, vCirc, radialSpeed, planeErrDeg));

            if (planeErrDeg > 0.2) throw new Exception(string.Format("Terminal plane is {0:F2} deg off the target normal.", planeErrDeg));
            if (Math.Abs(rMag - IloadTargetRadius) > 2000.0) throw new Exception("Terminal radius missed the 210 km circular target.");
            if (Math.Abs(vMag - vCirc) > 5.0 || Math.Abs(radialSpeed) > 5.0) throw new Exception("Terminal velocity is not circular-orbital.");
            if (c.InjectedMassKg <= IloadUpperEndTons * 1000.0 || c.InjectedMassKg >= IloadUpperStartTons * 1000.0)
                throw new Exception(string.Format("Injected mass {0:F0} kg outside the physical window ({1:F0}..{2:F0}).",
                    c.InjectedMassKg, IloadUpperEndTons * 1000.0, IloadUpperStartTons * 1000.0));
            if (c.TimeToHandoffSeconds < 30.0 || c.TimeToHandoffSeconds > 400.0) throw new Exception("Time to handoff is not sane.");
            if (c.PsgTimeToGoSeconds < 60.0 || c.PsgTimeToGoSeconds > 1200.0) throw new Exception("PSG time-to-go is not sane.");

            // The integrated path itself: reaches the handoff, burns monotonically, never pitches below horizon.
            OpenLoopSample last = c.Path[c.Path.Count - 1];
            bool pathOk = last.AltMeters >= IloadHandoffAltMeters;
            for (int i = 1; i < c.Path.Count && pathOk; i++)
                pathOk = c.Path[i].MassKg < c.Path[i - 1].MassKg
                         && c.Path[i].TimeSec > c.Path[i - 1].TimeSec
                         && c.Path[i].PitchDeg >= 0.0 && c.Path[i].PitchDeg <= 90.0;
            Console.WriteLine("  " + (pathOk ? "PASS" : "FAIL") + " (path reaches handoff; mass/time monotonic; pitch in [0,90])");
            if (!pathOk) throw new Exception("Integrated path violates a physical invariant.");
        }

        // The 1-D optimizer: sweep the SAME 9-point coarse grid Build uses (warm-chained like Build) to prove the
        // injected-mass objective is single-basin over the rate band, then require Build's winner to sit in that
        // basin's interior and beat every coarse sample. Also pins the table/lookup semantics the flight code
        // consumes (PitchDeg below/inside/beyond the table) and proves the warm-start plumbing does real work.
        private static void CheckAscentIloadBuildSweep()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load Build sweep (unimodality; winner beats coarse grid; table + lookup semantics) ===");

            OpenLoopInputs io = MakeIloadInputs();
            double[] rates = new double[9];
            var scan = new OpenLoopCandidate[9];
            PsgSolution warm = null;
            for (int i = 0; i < 9; i++)
            {
                rates[i] = 0.2 + (2.2 - 0.2) * i / 8.0;
                scan[i] = OpenLoopTrajectory.EvaluateCandidate(io, rates[i], warm);
                if (scan[i].Valid) warm = scan[i].PsgSolution;
                Console.WriteLine(string.Format("  rate {0,4:F2}  {1}", rates[i],
                    scan[i].Valid ? string.Format("injected {0,7:F1} t  iters {1}", scan[i].InjectedMassKg / 1000.0, scan[i].PsgIterations)
                                  : "INVALID: " + scan[i].Reason));
            }

            int first = -1, lastValid = -1, peak = -1;
            for (int i = 0; i < 9; i++)
            {
                if (!scan[i].Valid) continue;
                if (first < 0) first = i;
                lastValid = i;
                if (peak < 0 || scan[i].InjectedMassKg > scan[peak].InjectedMassKg) peak = i;
            }
            if (first < 0 || lastValid - first + 1 < 3) throw new Exception("Fewer than 3 valid coarse samples across the rate band.");
            for (int i = first; i <= lastValid; i++)
                if (!scan[i].Valid) throw new Exception(string.Format("Invalid candidate at rate {0:F2} INSIDE the valid band (non-contiguous).", rates[i]));

            const double slackKg = 2000.0;   // PSG solve-to-solve noise allowance
            for (int i = first; i < peak; i++)
                if (scan[i].InjectedMassKg > scan[i + 1].InjectedMassKg + slackKg)
                    throw new Exception(string.Format("Objective not single-basin: dip rising toward peak at rate {0:F2}.", rates[i]));
            for (int i = peak; i < lastValid; i++)
                if (scan[i + 1].InjectedMassKg > scan[i].InjectedMassKg + slackKg)
                    throw new Exception(string.Format("Objective not single-basin: rise falling off peak at rate {0:F2}.", rates[i + 1]));
            Console.WriteLine(string.Format("  single-basin over [{0:F2}..{1:F2}], peak at {2:F2} deg/s", rates[first], rates[lastValid], rates[peak]));

            DateTime t0 = DateTime.UtcNow;
            OpenLoopTrajectory plan = OpenLoopTrajectory.Build(io);
            Console.WriteLine(string.Format("  Build: valid={0} ({1})  {2:F1} s wall", plan.IsValid,
                plan.IsValid ? string.Format("rate {0:F3} deg/s, injected {1:F1} t", plan.PitchRateDegPerSecond, plan.PredictedInjectedMassKg / 1000.0)
                             : plan.ReasonUnavailable, (DateTime.UtcNow - t0).TotalSeconds));
            if (!plan.IsValid) throw new Exception("Build failed on the flown fixture: " + plan.ReasonUnavailable);
            if (plan.PitchRateDegPerSecond <= 0.2 || plan.PitchRateDegPerSecond >= 2.2)
                throw new Exception("Winning rate railed against the search band.");
            // Profile sanity bands (David, 2026-07-13): the winning chi table must look like a real RSS ascent,
            // not PSG's flat-and-fast drag-free optimum — pitch 58-66 deg around 38 km, 42-48 deg at the ~43 km
            // handoff. The AoA cap in the ramp law is what enforces this; these bands catch it regressing.
            double pitchAt38 = InterpTracePitchAtAltitude(plan.Trace, 38000.0);
            if (pitchAt38 < 58.0 || pitchAt38 > 66.0)
                throw new Exception(string.Format("Pitch {0:F1} deg at 38 km is outside the 58-66 deg real-ascent band.", pitchAt38));
            double handoffPitch = plan.TablePitchDeg[plan.TablePitchDeg.Length - 1];
            if (handoffPitch < 42.0 || handoffPitch > 48.0)
                throw new Exception(string.Format("Handoff pitch {0:F1} deg is outside the 42-48 deg band.", handoffPitch));

            // The AoA cap holds on the actual winning trajectory: finite-difference FPA from the trace vs the
            // commanded pitch, everywhere past pitch-over (2 deg cap + discretization slop).
            double maxAoa = MaxTraceAoaDeg(plan.Trace);
            Console.WriteLine(string.Format("  pitch@38km {0:F1} deg   handoff pitch {1:F1} deg   max |AoA| {2:F2} deg", pitchAt38, handoffPitch, maxAoa));
            if (maxAoa > 2.5)
                throw new Exception(string.Format("Winning trajectory flies {0:F1} deg AoA — the ramp AoA cap is not holding.", maxAoa));
            if (Math.Abs(plan.PitchRateDegPerSecond - rates[peak]) > 0.30)
                throw new Exception(string.Format("Winner {0:F2} deg/s is not in the coarse peak's cell ({1:F2}).", plan.PitchRateDegPerSecond, rates[peak]));
            if (plan.PredictedInjectedMassKg < scan[peak].InjectedMassKg - 1.0)
                throw new Exception("Golden-section refinement lost mass vs the coarse peak.");

            // Table invariants + the exact lookup semantics AscentGuidance flies.
            double[] s = plan.TableSpeedMps, p = plan.TablePitchDeg;
            if (s.Length != p.Length || s.Length < 50) throw new Exception("Chi table is degenerate.");
            if (s[0] < io.PitchOverSpeedMps) throw new Exception("Table starts below the pitch-over speed.");
            for (int i = 1; i < s.Length; i++)
            {
                if (s[i] <= s[i - 1]) throw new Exception("Table speeds are not strictly ascending.");
                if (p[i] > p[i - 1] + 0.3) throw new Exception(string.Format("Pitch table rises {0:F2}->{1:F2} deg at {2:F0} m/s (not a gravity turn).", p[i - 1], p[i], s[i]));
                if (p[i] < 0.0 || p[i] > 90.0) throw new Exception("Table pitch outside [0,90].");
            }
            if (p[p.Length - 1] > 65.0) throw new Exception(string.Format("Handoff pitch {0:F1} deg — table never really turned.", p[p.Length - 1]));
            int k = s.Length / 2;
            bool lookupOk = plan.PitchDeg(0.0) == 90.0
                            && plan.PitchDeg(s[0] - 1.0) == 90.0
                            && plan.PitchDeg(1e9) == p[p.Length - 1]
                            && Math.Abs(plan.PitchDeg(0.5 * (s[k] + s[k + 1])) - 0.5 * (p[k] + p[k + 1])) < 1e-9;
            Console.WriteLine("  " + (lookupOk ? "PASS" : "FAIL") + " (lookup: 90 below table, last beyond, exact midpoint interpolation)");
            if (!lookupOk) throw new Exception("PitchDeg lookup semantics broken.");

            if (plan.HandoffAltitudeMeters != io.HandoffAltitudeMeters) throw new Exception("Handoff altitude not passed through.");
            if (!(plan.PredictedTimeToOrbitSeconds > plan.PredictedTimeToHandoffSeconds && plan.PredictedTimeToHandoffSeconds > 0.0))
                throw new Exception("Predicted times are inconsistent.");
            if (plan.Trace == null || plan.Trace.Length < 50 || plan.Trace[plan.Trace.Length - 1].AltMeters < plan.HandoffAltitudeMeters)
                throw new Exception("Trace does not reach the handoff altitude.");

            // The winning open-loop trajectory itself, decimated for the eyeball.
            Console.WriteLine("      t      alt     downrange   srfSpd   pitch    mass");
            for (int i = 0; i < plan.Trace.Length; i += Math.Max(1, plan.Trace.Length / 15))
            {
                OpenLoopSample w = plan.Trace[i];
                Console.WriteLine(string.Format("  {0,5:F1}s  {1,6:F1}km  {2,7:F1}km  {3,6:F0}m/s  {4,5:F1}d  {5,6:F0}t",
                    w.TimeSec, w.AltMeters / 1000.0, w.DownrangeMeters / 1000.0, w.SurfaceSpeedMps, w.PitchDeg, w.MassKg / 1000.0));
            }
            OpenLoopSample h = plan.Trace[plan.Trace.Length - 1];
            Console.WriteLine(string.Format("  {0,5:F1}s  {1,6:F1}km  {2,7:F1}km  {3,6:F0}m/s  {4,5:F1}d  {5,6:F0}t  <- handoff to PSG",
                h.TimeSec, h.AltMeters / 1000.0, h.DownrangeMeters / 1000.0, h.SurfaceSpeedMps, h.PitchDeg, h.MassKg / 1000.0));

            // Warm-start does real work: re-solving the winner from its own solution must not iterate more.
            OpenLoopCandidate cold = OpenLoopTrajectory.EvaluateCandidate(io, plan.PitchRateDegPerSecond);
            OpenLoopCandidate hot = OpenLoopTrajectory.EvaluateCandidate(io, plan.PitchRateDegPerSecond, cold.PsgSolution);
            Console.WriteLine(string.Format("  warm-start: cold {0} iters -> warm {1} iters (masses {2:F1} / {3:F1} t)",
                cold.PsgIterations, hot.PsgIterations, cold.InjectedMassKg / 1000.0, hot.InjectedMassKg / 1000.0));
            if (!cold.Valid || !hot.Valid) throw new Exception("Winner re-evaluation failed.");
            if (hot.PsgIterations > cold.PsgIterations) throw new Exception("Warm-started solve iterated MORE than cold (warm-start not plumbed).");
            if (Math.Abs(hot.InjectedMassKg - cold.InjectedMassKg) > 2000.0) throw new Exception("Warm and cold solves disagree on the objective.");
        }

        // Commanded pitch (above horizon) from the plan trace, linearly interpolated at an altitude.
        private static double InterpTracePitchAtAltitude(OpenLoopSample[] trace, double altMeters)
        {
            for (int i = 1; i < trace.Length; i++)
            {
                if (trace[i].AltMeters < altMeters) continue;
                double f = (altMeters - trace[i - 1].AltMeters) / Math.Max(1e-9, trace[i].AltMeters - trace[i - 1].AltMeters);
                return trace[i - 1].PitchDeg + f * (trace[i].PitchDeg - trace[i - 1].PitchDeg);
            }
            return trace[trace.Length - 1].PitchDeg;
        }

        // Max |commanded pitch - velocity FPA| over the trace past pitch-over: the flown angle of attack,
        // with FPA finite-differenced from consecutive samples (asin(dAlt / (v*dt))).
        private static double MaxTraceAoaDeg(OpenLoopSample[] trace)
        {
            double max = 0.0;
            for (int i = 1; i < trace.Length; i++)
            {
                OpenLoopSample a = trace[i - 1], b = trace[i];
                double dt = b.TimeSec - a.TimeSec;
                double v = 0.5 * (a.SurfaceSpeedMps + b.SurfaceSpeedMps);
                if (dt <= 0.0 || v < 150.0 || b.PitchDeg >= 89.9) continue;   // skip vertical hold + start-up noise
                double sinFpa = MathHelpers.Clamp((b.AltMeters - a.AltMeters) / (v * dt), -1.0, 1.0);
                double fpa = MathHelpers.Rad2Deg(Math.Asin(sinFpa));
                double aoa = Math.Abs(fpa - 0.5 * (a.PitchDeg + b.PitchDeg));
                if (aoa > max) max = aoa;
            }
            return max;
        }

        // Every declared failure path returns Fail(reason)/invalid-candidate — never a throw, never a bogus plan.
        private static void CheckAscentIloadFailurePaths()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load failure paths (validation, integrator divergence, PSG non-convergence) ===");

            void ExpectBuildFail(string label, OpenLoopInputs io, string reason)
            {
                OpenLoopTrajectory t = OpenLoopTrajectory.Build(io);
                bool ok = !t.IsValid && t.ReasonUnavailable == reason;
                Console.WriteLine(string.Format("  {0,-22} -> {1}  ({2})", label, ok ? "PASS" : "FAIL", t.ReasonUnavailable));
                if (!ok) throw new Exception(string.Format("Failure path '{0}' returned '{1}', expected '{2}'.", label, t.ReasonUnavailable, reason));
            }

            ExpectBuildFail("null inputs", null, "null inputs");
            OpenLoopInputs noTarget = MakeIloadInputs(); noTarget.Target = null;
            ExpectBuildFail("no PSG target", noTarget, "no PSG target");
            OpenLoopInputs noStages = MakeIloadInputs(); noStages.Stages = new PoweredStageInfo[0];
            ExpectBuildFail("no stages", noStages, "no stages");
            OpenLoopInputs badHandoff = MakeIloadInputs(); badHandoff.HandoffAltitudeMeters = badHandoff.PadAltitudeMeters;
            ExpectBuildFail("handoff below pad", badHandoff, "bad handoff altitude");
            OpenLoopInputs noAtmo = MakeIloadInputs(); noAtmo.DensityAtAltitude = null;
            ExpectBuildFail("no atmosphere", noAtmo, "no atmosphere model");

            // Integrator divergence: a 6 deg/s ramp flips the vehicle horizontal long before the handoff.
            OpenLoopCandidate flip = OpenLoopTrajectory.EvaluateCandidate(MakeIloadInputs(), 6.0);
            bool flipOk = !flip.Valid && flip.Reason == "pitched past horizontal";
            Console.WriteLine(string.Format("  {0,-22} -> {1}  ({2})", "6 deg/s over-rotation", flipOk ? "PASS" : "FAIL", flip.Reason));
            if (!flipOk) throw new Exception("Over-rotation did not fail with 'pitched past horizontal': " + flip.Reason);

            // PSG non-convergence: a retrograde-ish target plane this stack cannot reach must be reported, not
            // returned as a plan. (Flipped normal = ~57 deg of plane error at the tangent geometry.)
            // Rate 0.7 is integrator-feasible (proven above), so the rejection must come from the PSG stage —
            // either non-convergence or the propellant-feasibility gate, never a bogus valid plan.
            OpenLoopInputs badPlane = MakeIloadInputs();
            badPlane.Target = PsgTarget.Create(IloadMu, IloadTargetRadius, IloadTargetRadius, IloadTargetRadius,
                -badPlane.Target.TargetOrbitNormal, 151.4, 236.977771247498, false, false);
            OpenLoopCandidate unreach = OpenLoopTrajectory.EvaluateCandidate(badPlane, 0.7);
            bool unreachOk = !unreach.Valid
                             && (unreach.Reason == "PSG did not converge" || unreach.Reason == "PSG solution exceeds available propellant");
            Console.WriteLine(string.Format("  {0,-22} -> {1}  ({2})", "unreachable plane", unreachOk ? "PASS" : "FAIL", unreach.Reason));
            if (!unreachOk) throw new Exception(string.Format("Unreachable plane was not rejected at the PSG stage: valid={0}, reason='{1}'.", unreach.Valid, unreach.Reason));
        }

        // Generality (zero per-vehicle constants): the same Build on a synthetic low-TWR variant of the stack
        // (booster thrust x0.75 -> liftoff TWR ~1.19, everything else derived from the stage list) must still
        // produce a valid, in-band, physically-windowed plan. The real second vehicle flies in P4.
        private static void CheckAscentIloadLowTwrVehicle()
        {
            Console.WriteLine();
            Console.WriteLine("=== I-load low-TWR vehicle (booster thrust x0.75; no per-vehicle constants) ===");

            OpenLoopInputs io = MakeIloadInputs(0.75);
            DateTime t0 = DateTime.UtcNow;
            OpenLoopTrajectory plan = OpenLoopTrajectory.Build(io);
            Console.WriteLine(string.Format("  Build: valid={0} ({1})  {2:F1} s wall", plan.IsValid,
                plan.IsValid ? string.Format("rate {0:F3} deg/s, injected {1:F1} t, handoff T+{2:F0}s", plan.PitchRateDegPerSecond,
                    plan.PredictedInjectedMassKg / 1000.0, plan.PredictedTimeToHandoffSeconds)
                             : plan.ReasonUnavailable, (DateTime.UtcNow - t0).TotalSeconds));
            if (!plan.IsValid) throw new Exception("Build failed on the low-TWR vehicle: " + plan.ReasonUnavailable);
            if (plan.PitchRateDegPerSecond <= 0.2 || plan.PitchRateDegPerSecond >= 2.2)
                throw new Exception("Low-TWR winning rate railed against the search band.");
            if (plan.PredictedInjectedMassKg <= IloadUpperEndTons * 1000.0 || plan.PredictedInjectedMassKg >= IloadUpperStartTons * 1000.0)
                throw new Exception("Low-TWR injected mass outside the physical window.");
            double[] s = plan.TableSpeedMps;
            for (int i = 1; i < s.Length; i++)
                if (s[i] <= s[i - 1]) throw new Exception("Low-TWR table speeds are not strictly ascending.");
            if (s.Length < 50) throw new Exception("Low-TWR chi table is degenerate.");
        }
    }
}
