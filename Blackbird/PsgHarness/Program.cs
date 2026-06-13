using System;
using Blackbird.Guidance;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.PsgHarness
{
    internal static class Program
    {
        private const double KerbinMu = 3.5316e12;
        private const double KerbinRadius = 600000.0;
        private const double KerbinRotationPeriod = 21549.425;
        private const double TargetInsertionAltitude = 81000.0;

        private static int Main()
        {
            Console.WriteLine("BlackBird PSG Harness");
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
