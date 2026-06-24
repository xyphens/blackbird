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
