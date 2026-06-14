using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PsgOptimizer
    {
        private enum ObjectiveType
        {
            MaximumEnergy,
            MinimumThrustAcceleration
        }

        private const int SimpsonNodesPerPhase = 8;
        private const int KnotsPerPhase = SimpsonNodesPerPhase * 2 - 1;
        private const double DiffStep = 1e-7;
        private const double FeasibilityTolerance = 1e-4;
        private const int MaxIterations = 1200;

        public PsgOptimizationResult Solve(PsgProblem problem, PsgSolution warmStart)
        {
            if (problem == null || !problem.IsValid)
            {
                return Failure(problem != null ? problem.ReasonUnavailable : "PSG problem is unavailable.");
            }

            if (problem.Phases == null || problem.Phases.Length == 0)
            {
                return Failure("PSG problem has no phases.");
            }

            if (warmStart != null && warmStart.IsValid)
            {
                PsgOptimizationResult result = RunConvergedSolve(problem, warmStart);
                if (result != null && result.Success) return result;
            }

            return RunInitialBootstrapping(problem);
        }

        private PsgOptimizationResult RunInitialBootstrapping(PsgProblem problem)
        {
            PsgOptimizationResult boot = RunPass(
                problem,
                null,
                true,
                IsFixedBurnTime(problem.Phases) ? ObjectiveType.MaximumEnergy : ObjectiveType.MinimumThrustAcceleration,
                "PSG boot");

            if (boot == null || !boot.Success || boot.Solution == null)
            {
                return boot ?? Failure("PSG bootstrapping failed.");
            }

            PsgTerminal finalTerminal = PsgTerminal.Create(problem, PsgScale.FromProblem(problem), IsFixedBurnTime(problem.Phases));
            if (finalTerminal.UsesFlightPathAngle)
            {
                boot.Status = "PSG bootstrapped " + boot.Status;
                return boot;
            }

            PsgOptimizationResult relaxed = RunPass(
                problem,
                boot.Solution,
                false,
                ObjectiveType.MinimumThrustAcceleration,
                "PSG terminal relaxation");

            if (relaxed != null && relaxed.Success && relaxed.Solution != null)
            {
                relaxed.Status = "PSG bootstrapped " + relaxed.Status;
                return relaxed;
            }

            boot.Status = "PSG bootstrapped FPA solution; terminal relaxation failed";
            return boot;
        }

        private PsgOptimizationResult RunConvergedSolve(PsgProblem problem, PsgSolution warmStart)
        {
            return RunPass(
                problem,
                warmStart,
                false,
                IsFixedBurnTime(problem.Phases) ? ObjectiveType.MaximumEnergy : ObjectiveType.MinimumThrustAcceleration,
                "PSG converged update");
        }

        private PsgOptimizationResult RunPass(
            PsgProblem problem,
            PsgSolution warmStart,
            bool forceFlightPathAngleTerminal,
            ObjectiveType objective,
            string passName)
        {
            var context = new SolveContext(problem, warmStart, forceFlightPathAngleTerminal, objective);
            double[] x = context.CreateInitialGuess();
            double[] lowerBounds = context.CreateLowerBounds();
            double[] upperBounds = context.CreateUpperBounds();
            double[] variableScale = context.CreateVariableScale();
            double[] constraintLower = context.CreateConstraintLowerBounds();
            double[] constraintUpper = context.CreateConstraintUpperBounds();

            try
            {
                alglib.minnlcstate state;
                alglib.minnlcreport report = new alglib.minnlcreport();

                alglib.minnlccreatef(context.VariableCount, x, DiffStep, out state);
                alglib.minnlcsetbc(state, lowerBounds, upperBounds);
                alglib.minnlcsetscale(state, variableScale);
                alglib.minnlcsetnlc2(state, constraintLower, constraintUpper);
                alglib.minnlcsetalgosqp(state);
                alglib.minnlcsetcond3(state, 0.0, 1e-5, MaxIterations);
                alglib.minnlcoptimize(state, context.Evaluate, null, null);
                alglib.minnlcresultsbuf(state, ref x, report);

                ConstraintViolationReport violations = context.MeasureConstraintViolation(x, constraintLower, constraintUpper);
                PsgSolution solution = context.CreateSolution(x, report, violations.Maximum);
                bool success = report.terminationtype > 0 && violations.PrimalFeasibility <= FeasibilityTolerance;

                return new PsgOptimizationResult
                {
                    Success = success,
                    Status = success
                        ? passName + " converged " + violations.ToStatusString()
                        : passName + " did not satisfy constraints " + violations.ToStatusString(),
                    Solution = success ? solution : null,
                    Iterations = report.iterationscount,
                    TerminationType = report.terminationtype,
                    ConstraintViolation = violations.PrimalFeasibility
                };
            }
            catch (Exception ex)
            {
                return Failure(passName + " failed: " + ex.Message);
            }
        }

        private static PsgOptimizationResult Failure(string status)
        {
            return new PsgOptimizationResult
            {
                Success = false,
                Status = string.IsNullOrEmpty(status) ? "PSG optimizer failed." : status,
                Solution = null,
                ConstraintViolation = double.PositiveInfinity
            };
        }

        private static bool IsFixedBurnTime(PsgPhase[] phases)
        {
            if (phases == null) return true;

            for (int i = 0; i < phases.Length; i++)
            {
                if (phases[i].AllowShutdown && !phases[i].IsCoast) return false;
            }

            return true;
        }

        private sealed class SolveContext
        {
            private readonly PsgProblem _problem;
            private readonly PsgSolution _warmStart;
            private readonly PsgScale _scale;
            private readonly PhaseLayout[] _layouts;
            private readonly PsgTerminal _terminal;
            private readonly bool _fixedBurnTime;
            private readonly ObjectiveType _objective;

            public int VariableCount { get; private set; }
            public int ConstraintCount { get; private set; }

            public SolveContext(
                PsgProblem problem,
                PsgSolution warmStart,
                bool forceFlightPathAngleTerminal,
                ObjectiveType objective)
            {
                _problem = problem;
                _warmStart = warmStart;
                _scale = PsgScale.FromProblem(problem);
                _layouts = CreateLayouts(problem.Phases);
                VariableCount = _layouts.Length == 0 ? 0 : _layouts[_layouts.Length - 1].EndVariableIndex;
                _fixedBurnTime = IsFixedBurnTime(problem.Phases);
                PsgTerminal terminal = PsgTerminal.Create(problem, _scale, _fixedBurnTime);
                _terminal = forceFlightPathAngleTerminal ? terminal.GetFlightPathAngleTerminal() : terminal;
                _objective = objective;
                ConstraintCount = CountConstraints(problem.Phases, _terminal);
            }

            public double[] CreateInitialGuess()
            {
                double[] x = new double[VariableCount];
                if (_warmStart != null && _warmStart.IsValid && TryTranscribeWarmStart(x))
                {
                    return x;
                }

                if (TryCreateShootingInitialGuess(x))
                {
                    return x;
                }

                Vector3d r0 = _problem.InitialRelativePositionMeters / _scale.Length;
                Vector3d v0 = _problem.InitialRelativeVelocityMetersPerSecond / _scale.Velocity;
                Vector3d u0 = _problem.InitialThrustDirection.sqrMagnitude > 0.0
                    ? _problem.InitialThrustDirection.normalized
                    : v0.normalized;

                Vector3d normal = _terminal.TargetNormal.sqrMagnitude > 0.0
                    ? _terminal.TargetNormal.normalized
                    : Vector3d.Cross(r0, v0).normalized;

                if (normal.sqrMagnitude <= 0.0)
                {
                    normal = Vector3d.Cross(r0.normalized, u0.normalized).normalized;
                }

                Vector3d finalRDirection = normal.sqrMagnitude > 0.0
                    ? Vector3d.Exclude(normal, r0).normalized
                    : r0.normalized;

                if (finalRDirection.sqrMagnitude <= 0.0) finalRDirection = r0.normalized;

                double finalRadius = GetInitialAttachmentRadius() / _scale.Length;
                Vector3d finalR = finalRDirection * finalRadius;
                Vector3d tangent = normal.sqrMagnitude > 0.0
                    ? Vector3d.Cross(normal, finalRDirection).normalized
                    : Vector3d.Exclude(finalRDirection, v0).normalized;

                if (Vector3d.Dot(tangent, v0) < 0.0) tangent = -tangent;
                if (tangent.sqrMagnitude <= 0.0) tangent = u0.normalized;

                double finalSpeed = GetInitialTerminalSpeed(finalRadius);
                Vector3d finalV = tangent * finalSpeed;

                double totalTime = Math.Max(1e-6, TotalInitialDuration());
                double elapsed = 0.0;
                double massStart = _problem.InitialMassKg / _scale.Mass;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double duration = ClampDuration(p, GetInitialPhaseDuration(phase));
                    x[layout.DurationIndex] = duration / _scale.Time;

                    double scaledPhaseStartMass = p == 0
                        ? massStart
                        : phase.StartMassKg / _scale.Mass;
                    double scaledMdot = phase.MassFlowKgPerSecond * _scale.Time / _scale.Mass;

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        double local = KnotsPerPhase > 1 ? (double)k / (KnotsPerPhase - 1) : 0.0;
                        double global = OrbitMath.Clamp((elapsed + local * duration) / totalTime, 0.0, 1.0);
                        Vector3d r = Lerp(r0, finalR, global);
                        Vector3d v = Lerp(v0, finalV, global);
                        Vector3d u = Lerp(u0, tangent, global);
                        if (u.sqrMagnitude <= 0.0) u = tangent;

                        SetVector(x, layout.RIndex(k), r);
                        SetVector(x, layout.VIndex(k), v);
                        x[layout.MIndex(k)] = phase.IsCoast
                            ? scaledPhaseStartMass
                            : Math.Max(phase.EndMassKg / _scale.Mass, scaledPhaseStartMass - scaledMdot * (duration / _scale.Time) * local);
                        SetVector(x, layout.UIndex(k), u.normalized);
                    }

                    elapsed += duration;
                }

                return x;
            }

            public double[] CreateLowerBounds()
            {
                double[] lower = CreateFilledArray(VariableCount, -100.0);

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        lower[layout.MIndex(k)] = 0.0;
                        SetVector(lower, layout.UIndex(k), new Vector3d(-2.0, -2.0, -2.0));
                    }

                    double minDuration;
                    double maxDuration;
                    GetPhaseDurationBounds(phase, out minDuration, out maxDuration);
                    lower[layout.DurationIndex] = minDuration / _scale.Time;
                }

                FreezeInitialState(lower);
                FreezePhaseStartMasses(lower);
                return lower;
            }

            public double[] CreateUpperBounds()
            {
                double[] upper = CreateFilledArray(VariableCount, 100.0);

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double scaledStartMass = phase.StartMassKg / _scale.Mass;

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        upper[layout.MIndex(k)] = scaledStartMass;
                        SetVector(upper, layout.UIndex(k), new Vector3d(2.0, 2.0, 2.0));
                    }

                    double minDuration;
                    double maxDuration;
                    GetPhaseDurationBounds(phase, out minDuration, out maxDuration);
                    upper[layout.DurationIndex] = maxDuration / _scale.Time;
                }

                FreezeInitialState(upper);
                FreezePhaseStartMasses(upper);
                return upper;
            }

            public double[] CreateVariableScale()
            {
                double[] scale = CreateFilledArray(VariableCount, 1.0);
                for (int p = 0; p < _layouts.Length; p++)
                {
                    scale[_layouts[p].DurationIndex] = Math.Max(0.1, GetInitialPhaseDuration(_problem.Phases[p]) / _scale.Time);
                }

                return scale;
            }

            public double[] CreateConstraintLowerBounds()
            {
                double[] lower = new double[ConstraintCount];
                int ci = 0;
                AddControlBounds(lower, ref ci, true);
                AddPhaseBounds(lower, ref ci);
                AddTerminalBounds(lower, ref ci, 0.0);
                return lower;
            }

            public double[] CreateConstraintUpperBounds()
            {
                double[] upper = new double[ConstraintCount];
                int ci = 0;
                AddControlBounds(upper, ref ci, false);
                AddPhaseBounds(upper, ref ci);
                AddTerminalBounds(upper, ref ci, 0.0);
                return upper;
            }

            public void Evaluate(double[] x, double[] f, object obj)
            {
                f[0] = Objective(x);
                int ci = 1;
                EvaluateControlConstraints(x, f, ref ci);
                for (int p = 0; p < _layouts.Length; p++)
                {
                    EvaluateDynamicConstraints(x, f, ref ci, p);
                    EvaluateStagingConstraint(x, f, ref ci, p);
                    EvaluateContinuityConstraints(x, f, ref ci, p);
                }
                EvaluateTerminalConstraints(x, f, ref ci);
            }

            public ConstraintViolationReport MeasureConstraintViolation(
                double[] x,
                double[] constraintLower,
                double[] constraintUpper)
            {
                double[] f = new double[ConstraintCount + 1];
                Evaluate(x, f, null);

                var report = new ConstraintViolationReport();
                for (int i = 0; i < ConstraintCount; i++)
                {
                    double value = f[i + 1];
                    double violation = 0.0;
                    if (value < constraintLower[i])
                    {
                        violation = constraintLower[i] - value;
                    }
                    else if (value > constraintUpper[i])
                    {
                        violation = value - constraintUpper[i];
                    }

                    report.Maximum = Math.Max(report.Maximum, violation);
                    report.PrimalFeasibility += violation * violation;
                }

                report.PrimalFeasibility = Math.Sqrt(report.PrimalFeasibility);
                return report;
            }

            public PsgSolution CreateSolution(double[] x, alglib.minnlcreport report, double violation)
            {
                var points = new List<PsgSolutionPoint>();
                var segments = new List<PsgSolutionSegment>();
                double universalTime = _problem.InitialUniversalTime;
                bool[] preciseShutdown;
                bool[] terminalStage;
                AnalyzeStages(x, out preciseShutdown, out terminalStage);

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double duration = Math.Max(0.0, x[layout.DurationIndex] * _scale.Time);
                    double step = KnotsPerPhase > 1 ? duration / (KnotsPerPhase - 1) : 0.0;
                    double startUt = universalTime;

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        if (p > 0 && k == 0) continue;

                        Vector3d u = GetVector(x, layout.UIndex(k));
                        double control = u.magnitude;
                        double throttle = phase.MinimumThrottle < 1.0
                            ? (control - phase.MinimumThrottle) / (1.0 - phase.MinimumThrottle)
                            : 1.0;

                        points.Add(new PsgSolutionPoint
                        {
                            UniversalTime = universalTime + step * k,
                            PhaseIndex = p,
                            KspStage = phase.KspStage,
                            RelativePosition = GetVector(x, layout.RIndex(k)) * _scale.Length,
                            RelativeVelocity = GetVector(x, layout.VIndex(k)) * _scale.Velocity,
                            MassKg = x[layout.MIndex(k)] * _scale.Mass,
                            InertialThrustDirection = u.sqrMagnitude > 0.0 ? u.normalized : Vector3d.zero,
                            Throttle = phase.IsCoast ? 0.0 : OrbitMath.Clamp(throttle, 0.01, 1.0)
                        });
                    }

                    universalTime += duration;
                    segments.Add(new PsgSolutionSegment
                    {
                        PhaseIndex = p,
                        KspStage = phase.KspStage,
                        StartUniversalTime = startUt,
                        EndUniversalTime = universalTime,
                        IsCoast = phase.IsCoast,
                        AllowShutdown = phase.AllowShutdown,
                        PreciseShutdown = preciseShutdown[p],
                        TerminalStage = terminalStage[p]
                    });
                }

                return new PsgSolution
                {
                    IsValid = true,
                    Status = "PSG solution",
                    CreatedUniversalTime = _problem.InitialUniversalTime,
                    StartUniversalTime = _problem.InitialUniversalTime,
                    FinalUniversalTime = points.Count > 0 ? points[points.Count - 1].UniversalTime : _problem.InitialUniversalTime,
                    TerminalAngularMomentum = _terminal.TargetAngularMomentum * _scale.Length * _scale.Velocity,
                    TerminalSpecificEnergy = _terminal.TargetSpecificEnergy * _scale.Velocity * _scale.Velocity,
                    Iterations = report.iterationscount,
                    TerminationType = report.terminationtype,
                    ConstraintViolation = violation,
                    Points = points.ToArray(),
                    Segments = segments.ToArray()
                };
            }

            private bool TryTranscribeWarmStart(double[] x)
            {
                if (_warmStart == null || _warmStart.Points == null || _warmStart.Points.Length == 0) return false;

                double universalTime = _problem.InitialUniversalTime;
                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double duration = ClampDuration(p, GetInitialPhaseDuration(phase));
                    if (_warmStart.Segments != null && p < _warmStart.Segments.Length)
                    {
                        if (_problem.InitialUniversalTime >= _warmStart.Segments[p].EndUniversalTime)
                            return false;
                        duration = Math.Max(0.05, _warmStart.Segments[p].EndUniversalTime - _warmStart.Segments[p].StartUniversalTime);
                    }

                    x[layout.DurationIndex] = duration / _scale.Time;
                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        double t = universalTime + duration * k / (KnotsPerPhase - 1);
                        PsgSolutionPoint point = _warmStart.GetPointAtUniversalTime(t);
                        if (point == null) return false;

                        SetVector(x, layout.RIndex(k), point.RelativePosition / _scale.Length);
                        SetVector(x, layout.VIndex(k), point.RelativeVelocity / _scale.Velocity);
                        x[layout.MIndex(k)] = point.MassKg > 0.0
                            ? point.MassKg / _scale.Mass
                            : GetPhaseMassAtFraction(phase, (double)k / (KnotsPerPhase - 1)) / _scale.Mass;
                        SetVector(x, layout.UIndex(k), point.InertialThrustDirection.sqrMagnitude > 0.0
                            ? point.InertialThrustDirection.normalized
                            : _problem.InitialThrustDirection.normalized);
                    }

                    universalTime += duration;
                }

                return true;
            }

            private bool TryCreateShootingInitialGuess(double[] x)
            {
                if (_layouts.Length == 0) return false;

                Vector3d r = _problem.InitialRelativePositionMeters / _scale.Length;
                Vector3d v = _problem.InitialRelativeVelocityMetersPerSecond / _scale.Velocity;
                double mass = _problem.InitialMassKg / _scale.Mass;
                Vector3d u = GuessInitialGuidanceDirection(r, v);

                if (r.sqrMagnitude <= 0.0 || u.sqrMagnitude <= 0.0 || mass <= 0.0) return false;

                double targetEnergy = _terminal.TargetSpecificEnergy;
                bool targetEnergyReached = SpecificEnergy(r, v) >= targetEnergy;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];

                    if (p == 0 || !phase.EnforceMassContinuity)
                    {
                        mass = phase.StartMassKg / _scale.Mass;
                    }

                    double duration = targetEnergyReached && !phase.IsCoast
                        ? 0.0
                        : EstimateShootingDuration(phase, r, v, mass, u, targetEnergy);

                    duration = ClampDuration(p, duration * _scale.Time) / _scale.Time;
                    x[layout.DurationIndex] = duration;

                    Vector3d phaseR = r;
                    Vector3d phaseV = v;
                    double phaseM = mass;
                    double step = KnotsPerPhase > 1 ? duration / (KnotsPerPhase - 1) : 0.0;

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        if (k > 0)
                        {
                            PropagateGuess(phase, u, step, ref phaseR, ref phaseV, ref phaseM);
                        }

                        SetVector(x, layout.RIndex(k), phaseR);
                        SetVector(x, layout.VIndex(k), phaseV);
                        x[layout.MIndex(k)] = phase.IsCoast ? mass : Math.Max(phase.EndMassKg / _scale.Mass, phaseM);
                        SetVector(x, layout.UIndex(k), u);
                    }

                    r = phaseR;
                    v = phaseV;
                    mass = phaseM;
                    targetEnergyReached = targetEnergyReached || (!phase.IsCoast && SpecificEnergy(r, v) >= targetEnergy);
                }

                return true;
            }

            private Vector3d GuessInitialGuidanceDirection(Vector3d r, Vector3d v)
            {
                Vector3d up = r.sqrMagnitude > 0.0 ? r.normalized : Vector3d.zero;
                Vector3d normal = _terminal.TargetNormal.sqrMagnitude > 0.0
                    ? _terminal.TargetNormal.normalized
                    : Vector3d.Cross(r, v).normalized;

                Vector3d horizontal = normal.sqrMagnitude > 0.0
                    ? Vector3d.Cross(normal, up).normalized
                    : Vector3d.Exclude(up, _problem.InitialThrustDirection).normalized;

                if (horizontal.sqrMagnitude <= 0.0)
                {
                    horizontal = Vector3d.Exclude(up, v).normalized;
                }

                if (horizontal.sqrMagnitude <= 0.0)
                {
                    return _problem.InitialThrustDirection.sqrMagnitude > 0.0
                        ? _problem.InitialThrustDirection.normalized
                        : up;
                }

                if (Vector3d.Dot(horizontal, v) < 0.0) horizontal = -horizontal;

                return (horizontal + up).normalized;
            }

            private double EstimateShootingDuration(
                PsgPhase phase,
                Vector3d r,
                Vector3d v,
                double mass,
                Vector3d u,
                double targetEnergy)
            {
                double minDuration;
                double maxDuration;
                GetPhaseDurationBounds(phase, out minDuration, out maxDuration);

                double duration = phase.IsCoast
                    ? minDuration + 0.5 * (maxDuration - minDuration)
                    : Math.Min(maxDuration, Math.Max(minDuration, phase.NominalBurnTimeSeconds));

                double scaledDuration = Math.Max(0.0, duration / _scale.Time);
                if (phase.IsCoast || scaledDuration <= 0.0) return scaledDuration;

                double e0 = SpecificEnergy(r, v);
                if (e0 >= targetEnergy) return 0.0;

                const int steps = 80;
                double step = scaledDuration / steps;
                double elapsed = 0.0;
                double previousEnergy = e0;

                for (int i = 0; i < steps; i++)
                {
                    PropagateGuess(phase, u, step, ref r, ref v, ref mass);
                    elapsed += step;

                    double energy = SpecificEnergy(r, v);
                    if (energy >= targetEnergy)
                    {
                        double span = energy - previousEnergy;
                        double fraction = span > 1e-12
                            ? OrbitMath.Clamp((targetEnergy - previousEnergy) / span, 0.0, 1.0)
                            : 1.0;

                        return Math.Max(0.0, elapsed - step + step * fraction);
                    }

                    previousEnergy = energy;
                }

                return scaledDuration;
            }

            private void PropagateGuess(
                PsgPhase phase,
                Vector3d u,
                double dt,
                ref Vector3d r,
                ref Vector3d v,
                ref double mass)
            {
                if (dt <= 0.0) return;

                Vector3d r1 = r;
                Vector3d v1 = v;
                double m1 = mass;
                Vector3d a1 = GuessAcceleration(phase, r1, v1, m1, u);
                double md1 = phase.IsCoast ? 0.0 : -PhaseMassFlow(phase);

                Vector3d r2 = r + v1 * (0.5 * dt);
                Vector3d v2 = v + a1 * (0.5 * dt);
                double m2 = Math.Max(1e-6, mass + md1 * 0.5 * dt);
                Vector3d a2 = GuessAcceleration(phase, r2, v2, m2, u);
                double md2 = phase.IsCoast ? 0.0 : -PhaseMassFlow(phase);

                Vector3d r3 = r + v2 * (0.5 * dt);
                Vector3d v3 = v + a2 * (0.5 * dt);
                double m3 = Math.Max(1e-6, mass + md2 * 0.5 * dt);
                Vector3d a3 = GuessAcceleration(phase, r3, v3, m3, u);
                double md3 = phase.IsCoast ? 0.0 : -PhaseMassFlow(phase);

                Vector3d r4 = r + v3 * dt;
                Vector3d v4 = v + a3 * dt;
                double m4 = Math.Max(1e-6, mass + md3 * dt);
                Vector3d a4 = GuessAcceleration(phase, r4, v4, m4, u);
                double md4 = phase.IsCoast ? 0.0 : -PhaseMassFlow(phase);

                r += dt / 6.0 * (v1 + 2.0 * v2 + 2.0 * v3 + v4);
                v += dt / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
                mass = Math.Max(phase.EndMassKg / _scale.Mass, mass + dt / 6.0 * (md1 + 2.0 * md2 + 2.0 * md3 + md4));
            }

            private Vector3d GuessAcceleration(PsgPhase phase, Vector3d r, Vector3d v, double mass, Vector3d u)
            {
                double rMag = Math.Max(1e-9, r.magnitude);
                Vector3d gravity = -r / (rMag * rMag * rMag);
                if (phase.IsCoast) return gravity;

                return gravity + PhaseThrust(phase, rMag) / Math.Max(1e-6, mass) * u;
            }

            private static double SpecificEnergy(Vector3d r, Vector3d v)
            {
                return 0.5 * v.sqrMagnitude - 1.0 / Math.Max(1e-9, r.magnitude);
            }

            private double Objective(double[] x)
            {
                if (_objective == ObjectiveType.MaximumEnergy)
                {
                    PhaseLayout last = _layouts[_layouts.Length - 1];
                    Vector3d r = GetVector(x, last.RIndex(KnotsPerPhase - 1));
                    Vector3d v = GetVector(x, last.VIndex(KnotsPerPhase - 1));
                    return -SpecificEnergy(r, v);
                }

                double value = 0.0;
                for (int p = 0; p < _layouts.Length; p++)
                {
                    PsgPhase phase = _problem.Phases[p];
                    if (phase.IsCoast || !phase.AllowShutdown) continue;

                    PhaseLayout layout = _layouts[p];
                    double thrust = PhaseVacuumThrust(phase);
                    double h6 = x[layout.DurationIndex] / ((SimpsonNodesPerPhase - 1) * 6.0);

                    for (int k = 0; k < KnotsPerPhase; k++)
                    {
                        double weight = k == 0 || k == KnotsPerPhase - 1 ? 1.0 : (k % 2 == 0 ? 2.0 : 4.0);
                        double mass = Math.Max(1e-6, x[layout.MIndex(k)]);
                        value += weight * thrust * h6 / mass;
                    }
                }

                return value;
            }

            private void EvaluateControlConstraints(double[] x, double[] f, ref int ci)
            {
                for (int p = 0; p < _layouts.Length; p++)
                {
                    PsgPhase phase = _problem.Phases[p];
                    if (phase.IsCoast && !phase.IsUnguided) continue;

                    PhaseLayout layout = _layouts[p];
                    int count = phase.IsUnguided ? 1 : KnotsPerPhase;
                    for (int k = 0; k < count; k++)
                    {
                        f[ci++] = GetVector(x, layout.UIndex(k)).magnitude;
                    }
                }
            }

            private void EvaluateDynamicConstraints(double[] x, double[] f, ref int ci, int p)
            {
                PhaseLayout layout = _layouts[p];
                PsgPhase phase = _problem.Phases[p];
                double duration = Math.Max(1e-9, x[layout.DurationIndex]);
                double h = duration / (SimpsonNodesPerPhase - 1);
                double h6 = h / 6.0;
                double h8 = h * 0.125;

                for (int n = 0; n < SimpsonNodesPerPhase - 1; n++)
                {
                    int k0 = 2 * n;
                    int k1 = k0 + 1;
                    int k2 = k0 + 2;

                    Vector3d r0 = GetVector(x, layout.RIndex(k0));
                    Vector3d r1 = GetVector(x, layout.RIndex(k1));
                    Vector3d r2 = GetVector(x, layout.RIndex(k2));
                    Vector3d v0 = GetVector(x, layout.VIndex(k0));
                    Vector3d v1 = GetVector(x, layout.VIndex(k1));
                    Vector3d v2 = GetVector(x, layout.VIndex(k2));
                    Vector3d a0 = Acceleration(x, p, k0);
                    Vector3d a1 = Acceleration(x, p, k1);
                    Vector3d a2 = Acceleration(x, p, k2);

                    AppendVector(f, ref ci, r2 - r0 - h6 * (v0 + 4.0 * v1 + v2));
                    AppendVector(f, ref ci, r1 - 0.5 * (r0 + r2) - h8 * (v0 - v2));
                    AppendVector(f, ref ci, v2 - v0 - h6 * (a0 + 4.0 * a1 + a2));
                    AppendVector(f, ref ci, v1 - 0.5 * (v0 + v2) - h8 * (a0 - a2));

                    if (!phase.IsCoast)
                    {
                        double mi = p > 0 && phase.EnforceMassContinuity
                            ? x[layout.MIndex(0)]
                            : phase.StartMassKg / _scale.Mass;
                        double mdot = PhaseMassFlow(phase);
                        f[ci++] = x[layout.MIndex(k1)] - mi + (n + 0.5) * h * mdot;
                        f[ci++] = x[layout.MIndex(k2)] - mi + (n + 1.0) * h * mdot;
                    }
                }
            }

            private void EvaluateStagingConstraint(double[] x, double[] f, ref int ci, int p)
            {
                PsgPhase phase = _problem.Phases[p];
                if (phase.IsCoast || !phase.AllowShutdown || phase.EnforceMassContinuity)
                {
                    f[ci++] = 0.0;
                    return;
                }

                int nextBurn = NextAdjustableBurn(p);
                if (nextBurn < 0)
                {
                    f[ci++] = 0.0;
                    return;
                }

                int nextNextBurn = NextAdjustableBurn(nextBurn);
                if (nextNextBurn < 0 && _problem.Phases[nextBurn].EnforceMassContinuity)
                {
                    f[ci++] = 0.0;
                    return;
                }

                double thisBt = x[_layouts[p].DurationIndex];
                double nextBt = x[_layouts[nextBurn].DurationIndex];

                bool combineThis = nextNextBurn > 0 && _problem.Phases[nextBurn].EnforceMassContinuity;
                bool combineNext = nextNextBurn > 0 && !combineThis && _problem.Phases[nextNextBurn].EnforceMassContinuity;

                if (combineThis)
                {
                    thisBt += nextBt;
                    nextBt = x[_layouts[nextNextBurn].DurationIndex];
                }
                else if (combineNext)
                {
                    nextBt += x[_layouts[nextNextBurn].DurationIndex];
                }

                double remainingThisBurn = phase.NominalBurnTimeSeconds / _scale.Time - thisBt;
                double u = nextBt * nextBt + remainingThisBurn * remainingThisBurn + 2e-6;
                f[ci++] = Math.Sqrt(u) - (nextBt + remainingThisBurn);
            }

            private void EvaluateContinuityConstraints(double[] x, double[] f, ref int ci, int p)
            {
                if (p == 0) return;

                PhaseLayout previous = _layouts[p - 1];
                PhaseLayout current = _layouts[p];
                AppendVector(f, ref ci, GetVector(x, previous.RIndex(KnotsPerPhase - 1)) - GetVector(x, current.RIndex(0)));
                AppendVector(f, ref ci, GetVector(x, previous.VIndex(KnotsPerPhase - 1)) - GetVector(x, current.VIndex(0)));
                AppendVector(f, ref ci, GetVector(x, previous.UIndex(KnotsPerPhase - 1)) - GetVector(x, current.UIndex(0)));

                if (_problem.Phases[p].EnforceMassContinuity)
                {
                    f[ci++] = x[previous.MIndex(KnotsPerPhase - 1)] - x[current.MIndex(0)];
                }
            }

            private void EvaluateTerminalConstraints(double[] x, double[] f, ref int ci)
            {
                PhaseLayout last = _layouts[_layouts.Length - 1];
                Vector3d r = GetVector(x, last.RIndex(KnotsPerPhase - 1));
                Vector3d v = GetVector(x, last.VIndex(KnotsPerPhase - 1));
                _terminal.Evaluate(r, v, f, ref ci);
            }

            private Vector3d Acceleration(double[] x, int phaseIndex, int knot)
            {
                PsgPhase phase = _problem.Phases[phaseIndex];
                PhaseLayout layout = _layouts[phaseIndex];
                Vector3d r = GetVector(x, layout.RIndex(knot));
                double rMag = Math.Max(1e-9, r.magnitude);
                Vector3d gravity = -r / (rMag * rMag * rMag);

                if (phase.IsCoast) return gravity;

                Vector3d u = GetControlVector(x, phaseIndex, knot);
                double mass = Math.Max(1e-6, x[layout.MIndex(knot)]);
                return gravity + PhaseThrust(phase, rMag) / mass * u;
            }

            private Vector3d GetControlVector(double[] x, int phaseIndex, int knot)
            {
                PsgPhase phase = _problem.Phases[phaseIndex];
                PhaseLayout layout = _layouts[phaseIndex];

                return phase.IsUnguided || phase.IsCoast
                    ? GetVector(x, layout.UIndex(0))
                    : GetVector(x, layout.UIndex(knot));
            }

            private void AnalyzeStages(double[] x, out bool[] preciseShutdown, out bool[] terminalStage)
            {
                preciseShutdown = new bool[_problem.Phases.Length];
                terminalStage = new bool[_problem.Phases.Length];
                int optimizedShutdownIndex = -1;
                int terminalStageIndex = -1;
                bool pruningStages = false;

                for (int p = 0; p < _problem.Phases.Length; p++)
                {
                    PsgPhase phase = _problem.Phases[p];
                    double duration = x[_layouts[p].DurationIndex] * _scale.Time;
                    bool freeBurnTimeLeft = phase.NominalBurnTimeSeconds - duration > 1e-3;
                    bool prunableStage = pruningStages && duration < 1e-3;

                    if (phase.AllowShutdown && !prunableStage)
                    {
                        optimizedShutdownIndex = p;
                    }

                    if (!phase.AllowShutdown || !prunableStage)
                    {
                        terminalStageIndex = p;
                    }

                    if (phase.AllowShutdown && freeBurnTimeLeft)
                    {
                        pruningStages = true;
                    }
                }

                if (optimizedShutdownIndex >= 0) preciseShutdown[optimizedShutdownIndex] = true;
                if (terminalStageIndex >= 0) terminalStage[terminalStageIndex] = true;
            }

            private void AddControlBounds(double[] bounds, ref int ci, bool lower)
            {
                for (int p = 0; p < _problem.Phases.Length; p++)
                {
                    PsgPhase phase = _problem.Phases[p];
                    if (phase.IsCoast && !phase.IsUnguided) continue;

                    double value = lower ? phase.MinimumThrottle : 1.0;
                    int count = phase.IsUnguided ? 1 : KnotsPerPhase;
                    for (int k = 0; k < count; k++)
                    {
                        bounds[ci++] = value;
                    }
                }
            }

            private void AddPhaseBounds(double[] bounds, ref int ci)
            {
                for (int p = 0; p < _problem.Phases.Length; p++)
                {
                    int dynamic = (SimpsonNodesPerPhase - 1) * (12 + (_problem.Phases[p].IsCoast ? 0 : 2));
                    for (int i = 0; i < dynamic; i++) bounds[ci++] = 0.0;

                    bounds[ci++] = 0.0;

                    if (p > 0)
                    {
                        for (int i = 0; i < 9; i++) bounds[ci++] = 0.0;
                        if (_problem.Phases[p].EnforceMassContinuity) bounds[ci++] = 0.0;
                    }
                }
            }

            private void AddTerminalBounds(double[] bounds, ref int ci, double value)
            {
                for (int i = 0; i < _terminal.ConstraintCount; i++)
                {
                    bounds[ci++] = value;
                }
            }

            private void FreezeInitialState(double[] bounds)
            {
                if (_layouts.Length == 0) return;

                PhaseLayout first = _layouts[0];
                SetVector(bounds, first.RIndex(0), _problem.InitialRelativePositionMeters / _scale.Length);
                SetVector(bounds, first.VIndex(0), _problem.InitialRelativeVelocityMetersPerSecond / _scale.Velocity);

                if (_problem.InitialThrustDirection.sqrMagnitude > 0.0
                    && _problem.Phases.Length > 0
                    && !_problem.Phases[0].IsCoast)
                {
                    SetVector(bounds, first.UIndex(0), _problem.InitialThrustDirection.normalized);
                }
            }

            private void FreezePhaseStartMasses(double[] bounds)
            {
                for (int p = 0; p < _layouts.Length; p++)
                {
                    if (p > 0 && _problem.Phases[p].EnforceMassContinuity) continue;
                    bounds[_layouts[p].MIndex(0)] = _problem.Phases[p].StartMassKg / _scale.Mass;
                }
            }

            private int NextAdjustableBurn(int phaseIndex)
            {
                for (int p = phaseIndex + 1; p < _problem.Phases.Length; p++)
                {
                    PsgPhase phase = _problem.Phases[p];
                    if (phase.IsCoast || !phase.AllowShutdown) continue;
                    return p;
                }

                return -1;
            }

            private double TotalInitialDuration()
            {
                double total = 0.0;
                for (int i = 0; i < _problem.Phases.Length; i++)
                {
                    total += ClampDuration(i, GetInitialPhaseDuration(_problem.Phases[i]));
                }

                return total;
            }

            private double GetInitialPhaseDuration(PsgPhase phase)
            {
                return OrbitMath.IsFinite(phase.NominalBurnTimeSeconds) && phase.NominalBurnTimeSeconds > 0.0
                    ? phase.NominalBurnTimeSeconds
                    : Math.Max(0.1, phase.MaximumBurnTimeSeconds);
            }

            private double ClampDuration(int phaseIndex, double duration)
            {
                double min;
                double max;
                GetPhaseDurationBounds(_problem.Phases[phaseIndex], out min, out max);
                return OrbitMath.Clamp(duration, min, max);
            }

            private void GetPhaseDurationBounds(PsgPhase phase, out double minimum, out double maximum)
            {
                minimum = Math.Max(0.0, phase.MinimumBurnTimeSeconds);
                maximum = phase.MaximumBurnTimeSeconds;
                if (!OrbitMath.IsFinite(maximum) || maximum <= 0.0)
                {
                    maximum = phase.AllowShutdown
                        ? double.PositiveInfinity
                        : Math.Max(1.0, phase.NominalBurnTimeSeconds);
                }

                maximum = Math.Max(minimum + 0.05, maximum);
            }

            private double GetInitialAttachmentRadius()
            {
                double pe = _problem.Target.PeriapsisRadiusMeters;
                double ap = _problem.Target.ApoapsisRadiusMeters;
                if (ap < pe)
                {
                    double temp = ap;
                    ap = pe;
                    pe = temp;
                }

                double ecc = (ap - pe) / (ap + pe);
                if (_problem.Target.UseAttachmentRadius || ecc >= 1e-4)
                {
                    return OrbitMath.Clamp(_problem.Target.AttachmentRadiusMeters, pe, ap);
                }

                return pe;
            }

            private double GetInitialTerminalSpeed(double scaledRadius)
            {
                double physicalRadius = scaledRadius * _scale.Length;
                double pe = _problem.Target.PeriapsisRadiusMeters;
                double ap = _problem.Target.ApoapsisRadiusMeters;
                double sma = (pe + ap) * 0.5;
                return Math.Sqrt(Math.Max(0.0, _problem.BodyGravParameter * (2.0 / physicalRadius - 1.0 / sma))) / _scale.Velocity;
            }

            private double GetPhaseMassAtFraction(PsgPhase phase, double fraction)
            {
                return phase.StartMassKg - (phase.StartMassKg - phase.EndMassKg) * OrbitMath.Clamp(fraction, 0.0, 1.0);
            }

            private double PhaseMassFlow(PsgPhase phase)
            {
                return phase.MassFlowKgPerSecond * _scale.Time / _scale.Mass;
            }

            private double PhaseVacuumThrust(PsgPhase phase)
            {
                return phase.VacuumThrustNewtons * _scale.Time * _scale.Time / (_scale.Length * _scale.Mass);
            }

            private double PhaseThrust(PsgPhase phase, double scaledRadius)
            {
                if (_problem.AtmosphereScaleHeightMeters <= 0.0)
                {
                    return PhaseVacuumThrust(phase);
                }

                double h0 = _problem.AtmosphereScaleHeightMeters / _scale.Length;
                if (h0 <= 0.0) return PhaseVacuumThrust(phase);

                double initialRadius = Math.Max(1e-9, _problem.InitialRelativePositionMeters.magnitude / _scale.Length);
                double atmosphereFraction = Math.Exp(-(scaledRadius - initialRadius) / h0);
                atmosphereFraction = OrbitMath.Clamp(atmosphereFraction, 0.0, 1.0);

                double vexVacuum = phase.ExhaustVelocityVacuumMetersPerSecond / _scale.Velocity;
                double vexCurrent = phase.ExhaustVelocityCurrentMetersPerSecond > 0.0
                    ? phase.ExhaustVelocityCurrentMetersPerSecond / _scale.Velocity
                    : vexVacuum;

                double exhaustVelocity = vexCurrent + (vexVacuum - vexCurrent) * (1.0 - atmosphereFraction);
                return PhaseMassFlow(phase) * exhaustVelocity;
            }

            private static PhaseLayout[] CreateLayouts(PsgPhase[] phases)
            {
                var layouts = new PhaseLayout[phases.Length];
                int index = 0;

                for (int p = 0; p < phases.Length; p++)
                {
                    layouts[p] = new PhaseLayout(index);
                    index = layouts[p].EndVariableIndex;
                }

                return layouts;
            }

            private static int CountConstraints(PsgPhase[] phases, PsgTerminal terminal)
            {
                int count = 0;
                for (int p = 0; p < phases.Length; p++)
                {
                    if (phases[p].IsCoast && !phases[p].IsUnguided) continue;
                    count += phases[p].IsUnguided ? 1 : KnotsPerPhase;
                }

                for (int p = 0; p < phases.Length; p++)
                {
                    count += (SimpsonNodesPerPhase - 1) * (12 + (phases[p].IsCoast ? 0 : 2));
                    count += 1;
                    if (p > 0)
                    {
                        count += 9;
                        if (phases[p].EnforceMassContinuity) count += 1;
                    }
                }

                return count + terminal.ConstraintCount;
            }

            private static void AppendVector(double[] f, ref int ci, Vector3d value)
            {
                f[ci++] = value.x;
                f[ci++] = value.y;
                f[ci++] = value.z;
            }

            private static Vector3d GetVector(double[] x, int index)
            {
                return new Vector3d(x[index], x[index + 1], x[index + 2]);
            }

            private static void SetVector(double[] x, int index, Vector3d value)
            {
                x[index] = value.x;
                x[index + 1] = value.y;
                x[index + 2] = value.z;
            }

            private static Vector3d Lerp(Vector3d a, Vector3d b, double t)
            {
                return a + (b - a) * t;
            }

            private static double[] CreateFilledArray(int length, double value)
            {
                double[] array = new double[length];
                for (int i = 0; i < array.Length; i++) array[i] = value;
                return array;
            }
        }

        private sealed class ConstraintViolationReport
        {
            public double Maximum { get; set; }
            public double PrimalFeasibility { get; set; }

            public string ToStatusString()
            {
                return string.Format("pf={0:E1} max={1:E1}", PrimalFeasibility, Maximum);
            }
        }

        private sealed class PhaseLayout
        {
            private const int ValuesPerKnot = 10;
            private readonly int _baseIndex;

            public PhaseLayout(int baseIndex)
            {
                _baseIndex = baseIndex;
            }

            public int DurationIndex
            {
                get { return _baseIndex; }
            }

            public int EndVariableIndex
            {
                get { return _baseIndex + 1 + KnotsPerPhase * ValuesPerKnot; }
            }

            public int RIndex(int knot)
            {
                return KnotIndex(knot);
            }

            public int VIndex(int knot)
            {
                return KnotIndex(knot) + 3;
            }

            public int MIndex(int knot)
            {
                return KnotIndex(knot) + 6;
            }

            public int UIndex(int knot)
            {
                return KnotIndex(knot) + 7;
            }

            private int KnotIndex(int knot)
            {
                return _baseIndex + 1 + knot * ValuesPerKnot;
            }
        }
    }
}
