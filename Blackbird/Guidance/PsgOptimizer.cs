using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PsgOptimizer
    {
        // TODO(MechJeb parity): this is a compact direct-collocation solver, not the full MechJeb PSG transcription.
        // Replace it with phase/interpolant layout parity, analytic sparse derivatives, terminal constraint families,
        // and SolutionBuilder-style shutdown/coast/terminal-stage metadata.
        private const int NodesPerPhase = 5;
        private const double DiffStep = 1e-6;
        // TODO(MechJeb parity): align constraint scaling and success criteria with MechJeb's normalized primal feasibility.
        private const double FeasibilityTolerance = 5e-3;
        private const int MaxIterations = 250;

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

            var context = new SolveContext(problem, warmStart);
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
                double violation = violations.Maximum;
                PsgSolution solution = context.CreateSolution(x, report, violation);
                bool success = report.terminationtype > 0 && violation <= FeasibilityTolerance;

                return new PsgOptimizationResult
                {
                    Success = success,
                    Status = success
                        ? "PSG converged " + violations.ToStatusString()
                        : "PSG did not satisfy constraints " + violations.ToStatusString(),
                    Solution = success ? solution : null,
                    Iterations = report.iterationscount,
                    TerminationType = report.terminationtype,
                    ConstraintViolation = violation
                };
            }
            catch (Exception ex)
            {
                return Failure("PSG optimizer failed: " + ex.Message);
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

        private sealed class SolveContext
        {
            private readonly PsgProblem _problem;
            private readonly PsgSolution _warmStart;
            private readonly Scale _scale;
            private readonly PhaseLayout[] _layouts;
            private readonly Vector3d _targetAngularMomentum;
            private readonly double _targetSpecificEnergy;

            public int VariableCount { get; private set; }
            public int ConstraintCount { get; private set; }

            public SolveContext(PsgProblem problem, PsgSolution warmStart)
            {
                _problem = problem;
                _warmStart = warmStart;
                _scale = Scale.FromProblem(problem);
                _layouts = CreateLayouts(problem.Phases);
                VariableCount = _layouts.Length == 0 ? 0 : _layouts[_layouts.Length - 1].EndVariableIndex;
                ConstraintCount = CountConstraints(problem.Phases.Length);
                _targetAngularMomentum = GetTargetAngularMomentum(problem, _scale);
                _targetSpecificEnergy = problem.Target.TargetSpecificEnergy / (_scale.Velocity * _scale.Velocity);
            }

            public double[] CreateInitialGuess()
            {
                double[] x = new double[VariableCount];

                if (_warmStart != null && _warmStart.IsValid && TryTranscribeWarmStart(x))
                {
                    return x;
                }

                Vector3d r0 = _problem.InitialRelativePositionMeters / _scale.Length;
                Vector3d v0 = _problem.InitialRelativeVelocityMetersPerSecond / _scale.Velocity;
                Vector3d u0 = _problem.InitialThrustDirection.sqrMagnitude > 0.0
                    ? _problem.InitialThrustDirection.normalized
                    : v0.normalized;

                Vector3d normal = _targetAngularMomentum.sqrMagnitude > 0.0
                    ? _targetAngularMomentum.normalized
                    : Vector3d.Cross(r0, v0).normalized;

                Vector3d finalRDirection = normal.sqrMagnitude > 0.0
                    ? Vector3d.Exclude(normal, r0).normalized
                    : r0.normalized;

                if (finalRDirection.sqrMagnitude <= 0.0) finalRDirection = r0.normalized;

                double finalRadius = _problem.Target.AttachmentRadiusMeters / _scale.Length;
                Vector3d tangent = normal.sqrMagnitude > 0.0
                    ? Vector3d.Cross(normal, finalRDirection).normalized
                    : Vector3d.Exclude(finalRDirection, v0).normalized;

                if (Vector3d.Dot(tangent, v0) < 0.0) tangent = -tangent;
                if (tangent.sqrMagnitude <= 0.0) tangent = v0.normalized;

                double finalSpeedSquared = Math.Max(0.0, 2.0 * (_targetSpecificEnergy + 1.0 / finalRadius));
                Vector3d finalR = finalRDirection * finalRadius;
                Vector3d finalV = tangent * Math.Sqrt(finalSpeedSquared);

                double totalTime = Math.Max(1e-6, TotalNominalTime());
                double elapsed = 0.0;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double phaseTime = GetInitialPhaseDuration(phase);
                    x[layout.DurationIndex] = phaseTime / _scale.Time;

                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        double local = NodesPerPhase > 1 ? (double)k / (NodesPerPhase - 1) : 0.0;
                        double global = OrbitMath.Clamp((elapsed + local * phaseTime) / totalTime, 0.0, 1.0);
                        Vector3d r = Lerp(r0, finalR, global);
                        Vector3d v = Lerp(v0, finalV, global);
                        Vector3d u = Lerp(u0, tangent, global);
                        if (u.sqrMagnitude <= 0.0) u = tangent;

                        SetVector(x, layout.RIndex(k), r);
                        SetVector(x, layout.VIndex(k), v);
                        SetVector(x, layout.UIndex(k), u.normalized);
                    }

                    elapsed += phaseTime;
                }

                return x;
            }

            public double[] CreateLowerBounds()
            {
                double[] lower = CreateFilledArray(VariableCount, -10.0);

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];

                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        SetVector(lower, layout.UIndex(k), new Vector3d(-1.2, -1.2, -1.2));
                    }

                    double minDuration;
                    double maxDuration;
                    GetPhaseDurationBounds(p, phase, out minDuration, out maxDuration);
                    lower[layout.DurationIndex] = minDuration / _scale.Time;
                }

                FreezeInitialState(lower);
                return lower;
            }

            public double[] CreateUpperBounds()
            {
                double[] upper = CreateFilledArray(VariableCount, 10.0);

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];

                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        SetVector(upper, layout.UIndex(k), new Vector3d(1.2, 1.2, 1.2));
                    }

                    double minDuration;
                    double maxDuration;
                    GetPhaseDurationBounds(p, phase, out minDuration, out maxDuration);
                    upper[layout.DurationIndex] = maxDuration / _scale.Time;
                }

                FreezeInitialState(upper);
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

                AddDynamicBounds(lower, ref ci, 0.0);
                AddContinuityBounds(lower, ref ci, 0.0);
                AddControlBounds(lower, ref ci, 1.0);
                AddTerminalBounds(lower, ref ci, 0.0);

                return lower;
            }

            public double[] CreateConstraintUpperBounds()
            {
                double[] upper = new double[ConstraintCount];
                int ci = 0;

                AddDynamicBounds(upper, ref ci, 0.0);
                AddContinuityBounds(upper, ref ci, 0.0);
                AddControlBounds(upper, ref ci, 1.0);
                AddTerminalBounds(upper, ref ci, 0.0);

                return upper;
            }

            public void Evaluate(double[] x, double[] f, object obj)
            {
                f[0] = Objective(x);
                int ci = 1;

                EvaluateDynamicConstraints(x, f, ref ci);
                EvaluateContinuityConstraints(x, f, ref ci);
                EvaluateControlConstraints(x, f, ref ci);
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
                int ci = 0;
                int dynamicCount = _problem.Phases.Length * (NodesPerPhase - 1) * 6;
                int continuityCount = Math.Max(0, _problem.Phases.Length - 1) * 6;
                int controlCount = _problem.Phases.Length * NodesPerPhase;
                int terminalCount = 6;

                report.Dynamic = MeasureRange(f, constraintLower, constraintUpper, ci, dynamicCount);
                ci += dynamicCount;
                report.Continuity = MeasureRange(f, constraintLower, constraintUpper, ci, continuityCount);
                ci += continuityCount;
                report.Control = MeasureRange(f, constraintLower, constraintUpper, ci, controlCount);
                ci += controlCount;
                report.Terminal = MeasureRange(f, constraintLower, constraintUpper, ci, terminalCount);

                report.Maximum = Math.Max(
                    Math.Max(report.Dynamic, report.Continuity),
                    Math.Max(report.Control, report.Terminal));

                return report;
            }

            public PsgSolution CreateSolution(double[] x, alglib.minnlcreport report, double violation)
            {
                // TODO(MechJeb parity): preserve optimized shutdown, coast, staging, and terminal-stage metadata
                // instead of flattening the trajectory into guidance points only.
                var points = new List<PsgSolutionPoint>();
                double universalTime = _problem.InitialUniversalTime;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    PsgPhase phase = _problem.Phases[p];
                    double duration = Math.Max(0.0, x[layout.DurationIndex] * _scale.Time);
                    double step = NodesPerPhase > 1 ? duration / (NodesPerPhase - 1) : 0.0;

                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        if (p > 0 && k == 0) continue;

                        Vector3d u = GetVector(x, layout.UIndex(k));
                        points.Add(new PsgSolutionPoint
                        {
                            UniversalTime = universalTime + step * k,
                            PhaseIndex = p,
                            KspStage = phase.KspStage,
                            RelativePosition = GetVector(x, layout.RIndex(k)) * _scale.Length,
                            RelativeVelocity = GetVector(x, layout.VIndex(k)) * _scale.Velocity,
                            InertialThrustDirection = u.sqrMagnitude > 0.0 ? u.normalized : Vector3d.zero,
                            Throttle = phase.IsCoast ? 0.0 : 1.0
                        });
                    }

                    universalTime += duration;
                }

                return new PsgSolution
                {
                    IsValid = true,
                    Status = "PSG solution",
                    CreatedUniversalTime = _problem.InitialUniversalTime,
                    StartUniversalTime = _problem.InitialUniversalTime,
                    FinalUniversalTime = points.Count > 0 ? points[points.Count - 1].UniversalTime : _problem.InitialUniversalTime,
                    TerminalAngularMomentum = (_targetAngularMomentum * _scale.Length * _scale.Velocity).magnitude,
                    TerminalSpecificEnergy = _problem.Target.TargetSpecificEnergy,
                    Iterations = report.iterationscount,
                    TerminationType = report.terminationtype,
                    ConstraintViolation = violation,
                    Points = points.ToArray()
                };
            }

            private bool TryTranscribeWarmStart(double[] x)
            {
                if (_warmStart.Points == null || _warmStart.Points.Length == 0) return false;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    double phaseStart = p == 0 ? _problem.InitialUniversalTime : GetPhaseStartTimeFromWarmStart(p);
                    double phaseEnd = GetPhaseEndTimeFromWarmStart(p);
                    double duration = Math.Max(0.1, phaseEnd - phaseStart);
                    x[layout.DurationIndex] = duration / _scale.Time;

                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        double t = phaseStart + duration * k / (NodesPerPhase - 1);
                        PsgSolutionPoint point = NearestWarmPoint(t);
                        if (point == null) return false;

                        SetVector(x, layout.RIndex(k), point.RelativePosition / _scale.Length);
                        SetVector(x, layout.VIndex(k), point.RelativeVelocity / _scale.Velocity);
                        SetVector(x, layout.UIndex(k), point.InertialThrustDirection.sqrMagnitude > 0.0
                            ? point.InertialThrustDirection.normalized
                            : _problem.InitialThrustDirection.normalized);
                    }
                }

                return true;
            }

            private double GetPhaseStartTimeFromWarmStart(int phaseIndex)
            {
                if (_warmStart == null || _warmStart.Points == null) return _problem.InitialUniversalTime;

                for (int i = 0; i < _warmStart.Points.Length; i++)
                {
                    if (_warmStart.Points[i].PhaseIndex == phaseIndex) return _warmStart.Points[i].UniversalTime;
                }

                return _problem.InitialUniversalTime;
            }

            private double GetPhaseEndTimeFromWarmStart(int phaseIndex)
            {
                if (_warmStart == null || _warmStart.Points == null) return _problem.InitialUniversalTime + GetInitialPhaseDuration(_problem.Phases[phaseIndex]);

                for (int i = _warmStart.Points.Length - 1; i >= 0; i--)
                {
                    if (_warmStart.Points[i].PhaseIndex == phaseIndex) return _warmStart.Points[i].UniversalTime;
                }

                return _problem.InitialUniversalTime + GetInitialPhaseDuration(_problem.Phases[phaseIndex]);
            }

            private PsgSolutionPoint NearestWarmPoint(double universalTime)
            {
                PsgSolutionPoint best = null;
                double bestDistance = double.PositiveInfinity;

                for (int i = 0; i < _warmStart.Points.Length; i++)
                {
                    double distance = Math.Abs(_warmStart.Points[i].UniversalTime - universalTime);
                    if (distance >= bestDistance) continue;

                    best = _warmStart.Points[i];
                    bestDistance = distance;
                }

                return best;
            }

            private double Objective(double[] x)
            {
                double value = 0.0;
                int terminalPhase = _layouts.Length - 1;

                for (int p = 0; p < _layouts.Length; p++)
                {
                    if (p == terminalPhase)
                    {
                        value += x[_layouts[p].DurationIndex];
                    }

                    for (int k = 1; k < NodesPerPhase; k++)
                    {
                        Vector3d prev = GetVector(x, _layouts[p].UIndex(k - 1));
                        Vector3d current = GetVector(x, _layouts[p].UIndex(k));
                        value += 0.002 * (current - prev).sqrMagnitude;
                    }
                }

                return value;
            }

            private void EvaluateDynamicConstraints(double[] x, double[] f, ref int ci)
            {
                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    double duration = Math.Max(1e-9, x[layout.DurationIndex]);
                    double h = duration / (NodesPerPhase - 1);

                    for (int k = 0; k < NodesPerPhase - 1; k++)
                    {
                        Vector3d r0 = GetVector(x, layout.RIndex(k));
                        Vector3d r1 = GetVector(x, layout.RIndex(k + 1));
                        Vector3d v0 = GetVector(x, layout.VIndex(k));
                        Vector3d v1 = GetVector(x, layout.VIndex(k + 1));
                        Vector3d a0 = Acceleration(x, p, k);
                        Vector3d a1 = Acceleration(x, p, k + 1);

                        Vector3d rDefect = r1 - r0 - 0.5 * h * (v0 + v1);
                        Vector3d vDefect = v1 - v0 - 0.5 * h * (a0 + a1);

                        SetConstraintVector(f, ref ci, rDefect);
                        SetConstraintVector(f, ref ci, vDefect);
                    }
                }
            }

            private void EvaluateContinuityConstraints(double[] x, double[] f, ref int ci)
            {
                for (int p = 1; p < _layouts.Length; p++)
                {
                    PhaseLayout previous = _layouts[p - 1];
                    PhaseLayout current = _layouts[p];

                    SetConstraintVector(f, ref ci, GetVector(x, previous.RIndex(NodesPerPhase - 1)) - GetVector(x, current.RIndex(0)));
                    SetConstraintVector(f, ref ci, GetVector(x, previous.VIndex(NodesPerPhase - 1)) - GetVector(x, current.VIndex(0)));
                }
            }

            private void EvaluateControlConstraints(double[] x, double[] f, ref int ci)
            {
                for (int p = 0; p < _layouts.Length; p++)
                {
                    PhaseLayout layout = _layouts[p];
                    for (int k = 0; k < NodesPerPhase; k++)
                    {
                        Vector3d u = GetVector(x, layout.UIndex(k));
                        f[ci++] = u.magnitude;
                    }
                }
            }

            private void EvaluateTerminalConstraints(double[] x, double[] f, ref int ci)
            {
                // TODO(MechJeb parity): replace this h/energy/radius/radial approximation with the terminal
                // class selected by AscentBuilder: FlightPathAngle5/4/3 or Kepler5/4/3.
                PhaseLayout last = _layouts[_layouts.Length - 1];
                Vector3d r = GetVector(x, last.RIndex(NodesPerPhase - 1));
                Vector3d v = GetVector(x, last.VIndex(NodesPerPhase - 1));
                Vector3d h = Vector3d.Cross(r, v);
                double energy = 0.5 * v.sqrMagnitude - 1.0 / Math.Max(1e-9, r.magnitude);
                double attachmentRadius = _problem.Target.AttachmentRadiusMeters / _scale.Length;
                double radialVelocity = Vector3d.Dot(r, v) / Math.Max(1e-9, r.magnitude);

                SetConstraintVector(f, ref ci, h - _targetAngularMomentum);
                f[ci++] = energy - _targetSpecificEnergy;
                f[ci++] = r.magnitude - attachmentRadius;
                f[ci++] = radialVelocity;
            }

            private Vector3d Acceleration(double[] x, int phaseIndex, int nodeIndex)
            {
                PsgPhase phase = _problem.Phases[phaseIndex];
                PhaseLayout layout = _layouts[phaseIndex];
                Vector3d r = GetVector(x, layout.RIndex(nodeIndex));
                double rMag = Math.Max(1e-9, r.magnitude);
                Vector3d gravity = -r / (rMag * rMag * rMag);

                if (phase.IsCoast) return gravity;

                Vector3d u = GetVector(x, layout.UIndex(nodeIndex));
                if (u.sqrMagnitude > 0.0) u = u.normalized;

                double localTime = x[layout.DurationIndex] * nodeIndex / (NodesPerPhase - 1);
                double mass = Math.Max(1e-6, PhaseStartMass(phase) - PhaseMassFlow(phase) * localTime);
                double thrust = PhaseVacuumThrust(phase);

                return gravity + thrust / mass * u;
            }

            private void FreezeInitialState(double[] bounds)
            {
                if (_layouts.Length == 0) return;

                PhaseLayout first = _layouts[0];
                SetVector(bounds, first.RIndex(0), _problem.InitialRelativePositionMeters / _scale.Length);
                SetVector(bounds, first.VIndex(0), _problem.InitialRelativeVelocityMetersPerSecond / _scale.Velocity);
            }

            private double TotalNominalTime()
            {
                double total = 0.0;
                for (int i = 0; i < _problem.Phases.Length; i++)
                {
                    total += GetInitialPhaseDuration(_problem.Phases[i]);
                }

                return total;
            }

            private double GetInitialPhaseDuration(PsgPhase phase)
            {
                return OrbitMath.IsFinite(phase.NominalBurnTimeSeconds) && phase.NominalBurnTimeSeconds > 0.0
                    ? phase.NominalBurnTimeSeconds
                    : Math.Max(0.1, phase.MaximumBurnTimeSeconds);
            }

            private void GetPhaseDurationBounds(int phaseIndex, PsgPhase phase, out double minimum, out double maximum)
            {
                double nominal = GetInitialPhaseDuration(phase);

                // TODO(MechJeb parity): derive MinT/MaxT from phase shutdown rules and minimum throttle,
                // then let terminal-stage pruning decide which final phase owns precise shutdown.
                if (phaseIndex < _problem.Phases.Length - 1)
                {
                    minimum = nominal;
                    maximum = nominal;
                    return;
                }

                minimum = Math.Max(0.1, phase.MinimumBurnTimeSeconds);
                maximum = Math.Max(minimum, nominal);
            }

            private double PhaseStartMass(PsgPhase phase)
            {
                return phase.StartMassKg / _scale.Mass;
            }

            private double PhaseMassFlow(PsgPhase phase)
            {
                return phase.MassFlowKgPerSecond * _scale.Time / _scale.Mass;
            }

            private double PhaseVacuumThrust(PsgPhase phase)
            {
                return phase.VacuumThrustNewtons * _scale.Time * _scale.Time / (_scale.Length * _scale.Mass);
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

            private static int CountConstraints(int phaseCount)
            {
                int dynamic = phaseCount * (NodesPerPhase - 1) * 6;
                int continuity = Math.Max(0, phaseCount - 1) * 6;
                int control = phaseCount * NodesPerPhase;
                int terminal = 6;
                return dynamic + continuity + control + terminal;
            }

            private static Vector3d GetTargetAngularMomentum(PsgProblem problem, Scale scale)
            {
                Vector3d h = problem.Target.TargetAngularMomentumVector;
                if (h.sqrMagnitude <= 0.0)
                {
                    Vector3d currentH = Vector3d.Cross(
                        problem.InitialRelativePositionMeters,
                        problem.InitialRelativeVelocityMetersPerSecond);

                    double hMagnitude = Math.Sqrt(
                        problem.BodyGravParameter *
                        (2.0 * problem.Target.ApoapsisRadiusMeters * problem.Target.PeriapsisRadiusMeters /
                         (problem.Target.ApoapsisRadiusMeters + problem.Target.PeriapsisRadiusMeters)));

                    h = currentH.sqrMagnitude > 0.0 ? currentH.normalized * hMagnitude : Vector3d.zero;
                }

                return h / (scale.Length * scale.Velocity);
            }

            private void AddDynamicBounds(double[] bounds, ref int ci, double value)
            {
                for (int i = 0; i < _problem.Phases.Length * (NodesPerPhase - 1) * 6; i++)
                {
                    bounds[ci++] = value;
                }
            }

            private void AddContinuityBounds(double[] bounds, ref int ci, double value)
            {
                for (int i = 0; i < Math.Max(0, _problem.Phases.Length - 1) * 6; i++)
                {
                    bounds[ci++] = value;
                }
            }

            private void AddControlBounds(double[] bounds, ref int ci, double value)
            {
                for (int i = 0; i < _problem.Phases.Length * NodesPerPhase; i++)
                {
                    bounds[ci++] = value;
                }
            }

            private static void AddTerminalBounds(double[] bounds, ref int ci, double value)
            {
                for (int i = 0; i < 6; i++)
                {
                    bounds[ci++] = value;
                }
            }

            private static void SetConstraintVector(double[] f, ref int ci, Vector3d value)
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

            private static double MeasureRange(
                double[] f,
                double[] lower,
                double[] upper,
                int start,
                int count)
            {
                double max = 0.0;

                for (int i = start; i < start + count; i++)
                {
                    double value = f[i + 1];
                    double violation = 0.0;

                    if (value < lower[i])
                    {
                        violation = lower[i] - value;
                    }
                    else if (value > upper[i])
                    {
                        violation = value - upper[i];
                    }

                    max = Math.Max(max, violation);
                }

                return max;
            }
        }

        private sealed class ConstraintViolationReport
        {
            public double Maximum { get; set; }
            public double Dynamic { get; set; }
            public double Continuity { get; set; }
            public double Control { get; set; }
            public double Terminal { get; set; }

            public string ToStatusString()
            {
                return string.Format(
                    "max={0:E1} dyn={1:E1} cont={2:E1} ctrl={3:E1} term={4:E1}",
                    Maximum,
                    Dynamic,
                    Continuity,
                    Control,
                    Terminal);
            }
        }

        private sealed class PhaseLayout
        {
            private const int ValuesPerNode = 9;
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
                get { return _baseIndex + 1 + NodesPerPhase * ValuesPerNode; }
            }

            public int RIndex(int node)
            {
                return NodeIndex(node);
            }

            public int VIndex(int node)
            {
                return NodeIndex(node) + 3;
            }

            public int UIndex(int node)
            {
                return NodeIndex(node) + 6;
            }

            private int NodeIndex(int node)
            {
                return _baseIndex + 1 + node * ValuesPerNode;
            }
        }

        private sealed class Scale
        {
            public double Length { get; private set; }
            public double Velocity { get; private set; }
            public double Time { get; private set; }
            public double Mass { get; private set; }

            public static Scale FromProblem(PsgProblem problem)
            {
                double length = Math.Max(1.0, problem.InitialRelativePositionMeters.magnitude);
                double velocity = Math.Sqrt(problem.BodyGravParameter / length);
                double time = length / velocity;
                double mass = Math.Max(1.0, problem.InitialMassKg);

                return new Scale
                {
                    Length = length,
                    Velocity = velocity,
                    Time = time,
                    Mass = mass
                };
            }
        }
    }
}
