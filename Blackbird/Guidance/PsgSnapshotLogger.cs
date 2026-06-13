using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace Blackbird.Guidance
{
    public static class PsgSnapshotLogger
    {
        private const string PreferredDirectory = @"D:\SteamLibrary\steamapps\common\Kerbal Space Program Development\glog";
        private const string DirectoryName = "Blackbird";
        private const string FileName = "blackbird-psg.log";
        private static readonly object Sync = new object();
        private static string _announcedPath;

        public static void Write(PsgProblem problem, string status)
        {
            if (problem == null || !problem.IsValid) return;

            string text = Serialize(problem, status);
            Append(text);
        }

        public static void WriteResult(PsgProblem problem, PsgOptimizationResult result)
        {
            if (result == null) return;

            string text = SerializeResult(problem, result);
            Append(text);
        }

        private static void Append(string text)
        {
            Exception lastException = null;

            foreach (string directory in GetLogDirectories())
            {
                try
                {
                    Directory.CreateDirectory(directory);

                    string path = Path.Combine(directory, FileName);

                    lock (Sync)
                    {
                        File.AppendAllText(path, text + Environment.NewLine);
                    }

                    if (_announcedPath != path)
                    {
                        _announcedPath = path;
                        Debug.Log("[BlackBird] PSG log: " + path);
                    }

                    return;
                }
                catch (Exception ex)
                {
                    lastException = ex;
                }
            }

            Debug.Log(
                "[BlackBird] PSG snapshot write failed: " +
                (lastException != null ? lastException.Message : "no log directory available"));
        }

        private static string[] GetLogDirectories()
        {
            string root = KSPUtil.ApplicationRootPath;
            if (string.IsNullOrEmpty(root)) root = Directory.GetCurrentDirectory();

            return new[]
            {
                PreferredDirectory,
                Path.Combine(root, DirectoryName)
            };
        }

        private static string Serialize(PsgProblem problem, string status)
        {
            var sb = new StringBuilder();

            sb.AppendLine("# BlackBird PSG log v2");
            sb.AppendLine("record.type=problem");
            sb.AppendLine("record.utc=" + DateTime.UtcNow.ToString("O", System.Globalization.CultureInfo.InvariantCulture));
            sb.AppendLine("status=" + Escape(status));
            sb.AppendLine("ut=" + Format(problem.InitialUniversalTime));
            sb.AppendLine("body.mu=" + Format(problem.BodyGravParameter));
            sb.AppendLine("body.radius=" + Format(problem.BodyRadiusMeters));
            AppendVector(sb, "body.angularVelocity", problem.BodyAngularVelocityRadiansPerSecond);
            AppendVector(sb, "initial.r", problem.InitialRelativePositionMeters);
            AppendVector(sb, "initial.v", problem.InitialRelativeVelocityMetersPerSecond);
            AppendVector(sb, "initial.u", problem.InitialThrustDirection);
            sb.AppendLine("initial.massKg=" + Format(problem.InitialMassKg));
            sb.AppendLine("initial.altitude=" + Format(problem.InitialAltitudeMeters));
            sb.AppendLine("initial.verticalSpeed=" + Format(problem.InitialVerticalSpeedMetersPerSecond));
            sb.AppendLine("initial.apAlt=" + Format(problem.InitialApoapsisAltMeters));
            sb.AppendLine("initial.peAlt=" + Format(problem.InitialPeriapsisAltMeters));

            sb.AppendLine("target.peRadius=" + Format(problem.Target.PeriapsisRadiusMeters));
            sb.AppendLine("target.apRadius=" + Format(problem.Target.ApoapsisRadiusMeters));
            sb.AppendLine("target.attachmentRadius=" + Format(problem.Target.AttachmentRadiusMeters));
            sb.AppendLine("target.inclinationDeg=" + Format(problem.Target.InclinationDeg));
            sb.AppendLine("target.lanDeg=" + Format(problem.Target.LanDeg));
            sb.AppendLine("target.useLan=" + problem.Target.UseLanConstraint);
            AppendVector(sb, "target.normal", problem.Target.TargetOrbitNormal);
            AppendVector(sb, "target.h", problem.Target.TargetAngularMomentumVector);
            sb.AppendLine("target.energy=" + Format(problem.Target.TargetSpecificEnergy));

            int count = problem.Phases != null ? problem.Phases.Length : 0;
            sb.AppendLine("phase.count=" + count);

            for (int i = 0; i < count; i++)
            {
                PsgPhase phase = problem.Phases[i];
                string prefix = "phase." + i + ".";

                sb.AppendLine(prefix + "kspStage=" + phase.KspStage);
                sb.AppendLine(prefix + "phaseIndex=" + phase.PhaseIndex);
                sb.AppendLine(prefix + "startMassKg=" + Format(phase.StartMassKg));
                sb.AppendLine(prefix + "endMassKg=" + Format(phase.EndMassKg));
                sb.AppendLine(prefix + "vacuumThrustN=" + Format(phase.VacuumThrustNewtons));
                sb.AppendLine(prefix + "vacuumIsp=" + Format(phase.VacuumSpecificImpulseSeconds));
                sb.AppendLine(prefix + "currentIsp=" + Format(phase.CurrentSpecificImpulseSeconds));
                sb.AppendLine(prefix + "massFlowKgS=" + Format(phase.MassFlowKgPerSecond));
                sb.AppendLine(prefix + "nominalBurnTime=" + Format(phase.NominalBurnTimeSeconds));
                sb.AppendLine(prefix + "minBurnTime=" + Format(phase.MinimumBurnTimeSeconds));
                sb.AppendLine(prefix + "maxBurnTime=" + Format(phase.MaximumBurnTimeSeconds));
                sb.AppendLine(prefix + "minThrottle=" + Format(phase.MinimumThrottle));
                sb.AppendLine(prefix + "isCoast=" + phase.IsCoast);
                sb.AppendLine(prefix + "isUnguided=" + phase.IsUnguided);
                sb.AppendLine(prefix + "allowShutdown=" + phase.AllowShutdown);
                sb.AppendLine(prefix + "massContinuity=" + phase.EnforceMassContinuity);
            }

            return sb.ToString();
        }

        private static string SerializeResult(PsgProblem problem, PsgOptimizationResult result)
        {
            var sb = new StringBuilder();

            sb.AppendLine("# BlackBird PSG log v2");
            sb.AppendLine("record.type=result");
            sb.AppendLine("record.utc=" + DateTime.UtcNow.ToString("O", System.Globalization.CultureInfo.InvariantCulture));
            sb.AppendLine("status=" + Escape(result.Status));
            sb.AppendLine("success=" + result.Success);
            sb.AppendLine("iterations=" + result.Iterations);
            sb.AppendLine("termination=" + result.TerminationType);
            sb.AppendLine("violation=" + Format(result.ConstraintViolation));

            if (problem != null && problem.IsValid)
            {
                sb.AppendLine("ut=" + Format(problem.InitialUniversalTime));
                sb.AppendLine("body.mu=" + Format(problem.BodyGravParameter));
                sb.AppendLine("body.radius=" + Format(problem.BodyRadiusMeters));
            }

            PsgSolution solution = result.Solution;
            if (solution == null || !solution.IsValid)
            {
                return sb.ToString();
            }

            sb.AppendLine("solution.startUt=" + Format(solution.StartUniversalTime));
            sb.AppendLine("solution.finalUt=" + Format(solution.FinalUniversalTime));
            sb.AppendLine("solution.tgo=" + Format(solution.FinalUniversalTime - solution.StartUniversalTime));
            sb.AppendLine("solution.terminalH=" + Format(solution.TerminalAngularMomentum));
            sb.AppendLine("solution.terminalEnergy=" + Format(solution.TerminalSpecificEnergy));

            PsgGuidanceVector firstGuidance = solution.InertialGuidance(solution.StartUniversalTime);
            if (firstGuidance != null && firstGuidance.IsValid)
            {
                AppendVector(sb, "solution.first.u", firstGuidance.InertialDirection);
                sb.AppendLine("solution.first.throttle=" + Format(firstGuidance.Throttle));
            }

            PsgSolutionPoint terminal = solution.TerminalState();
            if (terminal != null)
            {
                AppendVector(sb, "solution.terminal.r", terminal.RelativePosition);
                AppendVector(sb, "solution.terminal.v", terminal.RelativeVelocity);
                AppendVector(sb, "solution.terminal.u", terminal.InertialThrustDirection);
                sb.AppendLine("solution.terminal.throttle=" + Format(terminal.Throttle));

                if (problem != null && problem.IsValid)
                {
                    AppendOrbitSummary(sb, problem, terminal);
                }
            }

            int pointCount = solution.Points != null ? solution.Points.Length : 0;
            sb.AppendLine("solution.point.count=" + pointCount);

            for (int i = 0; i < pointCount; i++)
            {
                PsgSolutionPoint point = solution.Points[i];
                string prefix = "solution.point." + i + ".";

                sb.AppendLine(prefix + "ut=" + Format(point.UniversalTime));
                sb.AppendLine(prefix + "phaseIndex=" + point.PhaseIndex);
                sb.AppendLine(prefix + "kspStage=" + point.KspStage);
                sb.AppendLine(prefix + "throttle=" + Format(point.Throttle));
                AppendVector(sb, prefix + "r", point.RelativePosition);
                AppendVector(sb, prefix + "v", point.RelativeVelocity);
                AppendVector(sb, prefix + "u", point.InertialThrustDirection);
            }

            return sb.ToString();
        }

        private static void AppendOrbitSummary(StringBuilder sb, PsgProblem problem, PsgSolutionPoint terminal)
        {
            double r = terminal.RelativePosition.magnitude;
            double v2 = terminal.RelativeVelocity.sqrMagnitude;
            Vector3d h = Vector3d.Cross(terminal.RelativePosition, terminal.RelativeVelocity);
            double h2 = h.sqrMagnitude;
            double energy = 0.5 * v2 - problem.BodyGravParameter / Math.Max(1e-9, r);

            if (energy >= 0.0 || h2 <= 0.0)
            {
                sb.AppendLine("solution.terminal.apAlt=Infinity");
                sb.AppendLine("solution.terminal.peAlt=NaN");
                sb.AppendLine("solution.terminal.eccentricity=NaN");
                return;
            }

            double semiMajorAxis = -problem.BodyGravParameter / (2.0 * energy);
            double eccentricitySquared = Math.Max(
                0.0,
                1.0 + 2.0 * energy * h2 / (problem.BodyGravParameter * problem.BodyGravParameter));
            double eccentricity = Math.Sqrt(eccentricitySquared);

            sb.AppendLine("solution.terminal.apAlt=" + Format(semiMajorAxis * (1.0 + eccentricity) - problem.BodyRadiusMeters));
            sb.AppendLine("solution.terminal.peAlt=" + Format(semiMajorAxis * (1.0 - eccentricity) - problem.BodyRadiusMeters));
            sb.AppendLine("solution.terminal.eccentricity=" + Format(eccentricity));
        }

        private static void AppendVector(StringBuilder sb, string key, Vector3d value)
        {
            sb.AppendLine(key + ".x=" + Format(value.x));
            sb.AppendLine(key + ".y=" + Format(value.y));
            sb.AppendLine(key + ".z=" + Format(value.z));
        }

        private static string Format(double value)
        {
            return value.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
        }

        private static string Escape(string value)
        {
            return string.IsNullOrEmpty(value)
                ? string.Empty
                : value.Replace("\r", " ").Replace("\n", " ");
        }
    }
}
