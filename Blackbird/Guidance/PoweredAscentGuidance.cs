using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Psg;
using System;
using System.Threading.Tasks;
using UnityEngine;

namespace Blackbird.Guidance
{
    [System.Serializable]
    public sealed class PoweredAscentGuidance
    {
        private const double SolveIntervalSeconds = 5.0;
        private const double RetryIntervalSeconds = 1.0;
        private const double TerminalSolveHorizonSeconds = 10.0;
        private const double TerminalSolveIntervalSeconds = 0.5;
        private const double SolutionStaleSeconds = 20.0;
        private const double ExpiredSolutionGraceSeconds = 0.25;
        private const double TerminalOverrunGraceSeconds = 30.0;
        //private const double TerminalSteeringFreezeRemainingVelocity = 15.0; // hold steering over this remaining m/s dV (prevent PSG doing a weird flip-up thing)

        // this effectively asks our guidance computer to go into overdrive calculating a more optimal orbit
        private const double TerminalPeMarginMeters = 30000.0; // begin propagating once osc Pe within 30 km
        private const double TerminalPropagateMaxSeconds = 6000.0;  // > one LEO period
        private const double TerminalPropagateStepSeconds = 20.0;    // RK4 step

        // Safety margin on how much stage dv we require to reach orbit
        private const double StageTrimVelocityMargin = 1.25;

        private const int MaxConsecutiveOptimizerFailures = 5;
        private int _optimizerFailCount = 0;

        private readonly PsgOptimizer _optimizer = new PsgOptimizer();
        private readonly BlackbirdLog bbLogger = new BlackbirdLog(LogContext.Psg);
        private PoweredGuidancePhase _phase = PoweredGuidancePhase.Unavailable;
        private bool _complete;
        private Task<PsgOptimizationResult> _solveTask;
        private PsgProblem _pendingProblem;
        private PsgSolution _solution;
        private TerminalSteeringGate _terminalGate;
        private double _lastSolveRequestUt = double.NegativeInfinity;
        private string _optimizerStatus = "PSG idle";
        private int _optimizerIterations;
        private double _constraintViolation = double.NaN;

        private int _lastVesselStage = int.MinValue;
        private bool _runColdSolve;

        // expose guidance state to listeners
        public double PredictedApoapsisAlt { get; private set; } = double.NaN;
        public double PredictedPeriapsisAlt { get; private set; } = double.NaN;
        private double _lastPredictionUt = double.NegativeInfinity;
        private const double PredictionIntervalSeconds = 0.5;

        public void Reset()
        {
            _phase = PoweredGuidancePhase.Unavailable;
            _complete = false;
            _optimizerFailCount = 0;
            _solveTask = null;
            _pendingProblem = null;
            _solution = null;
            _terminalGate.Reset();
            _lastSolveRequestUt = double.NegativeInfinity;
            _optimizerStatus = "PSG idle";
            _optimizerIterations = 0;
            _constraintViolation = double.NaN;

            PredictedApoapsisAlt = double.NaN;
            PredictedPeriapsisAlt = double.NaN;
            _lastPredictionUt = double.NegativeInfinity;

            _lastVesselStage = int.MinValue;
            _runColdSolve = false;
        }

        // Logs measured |h| and specific energy at shutdown against the solved terminal targets — the
        // per-flight accuracy scorecard. Scalars only; never pass VesselState (its live Vessel reference
        // makes the reflection serializer throw, and Log.Write swallows it, dropping the whole line).
        private void LogCompletion(string tag, VesselState vesselState)
        {
            Vector3d relPos = vesselState.Position - vesselState.Body.position;
            double r = Math.Max(1e-9, relPos.magnitude);
            double h = Vector3d.Cross(relPos, vesselState.OrbitalVelocity).magnitude;
            double e = 0.5 * vesselState.OrbitalVelocity.sqrMagnitude - vesselState.BodyGravParameter / r;
            double targetH = _solution != null ? _solution.TerminalAngularMomentum : double.NaN;
            double targetE = _solution != null ? _solution.TerminalSpecificEnergy : double.NaN;
            bbLogger.Write($"[{tag}] h={h:F0} (target {targetH:F0}, dH={h - targetH:F0}) e={e:F1} (target {targetE:F1}, dE={e - targetE:F1}) ap={vesselState.CurrentApoapsisAlt:F0} pe={vesselState.CurrentPeriapsisAlt:F0}");
        }

        public PoweredGuidanceCommand GetCommand(
            VesselState vesselState,
            LaunchPlan launchPlan,
            AscentProfile ascentProfile,
            double profilePitchDeg,
            double profileHeadingDeg,
            double profileThrottle)
        {
            if (vesselState == null || ascentProfile == null || !ascentProfile.IsValid)
            {
                return CreateUnavailable(profilePitchDeg, profileHeadingDeg, profileThrottle);
            }

            double targetAp = ascentProfile.TargetApoapsisAlt;
            double targetPe = ascentProfile.TargetPeriapsisAlt;
            double apError = targetAp - vesselState.CurrentApoapsisAlt;
            double peError = targetPe - vesselState.CurrentPeriapsisAlt;

            if (!HasUsableOrbitState(vesselState, targetAp, targetPe))
            {
                return CreateUnavailable(profilePitchDeg, profileHeadingDeg, profileThrottle);
            }

            Vector3d initialThrustDirection = GetCurrentThrustDirection(vesselState, profileHeadingDeg, profilePitchDeg);
            UpdatePsgSolution(vesselState, launchPlan, ascentProfile, initialThrustDirection);
            UpdateOrbitPrediction(vesselState);
            PinSolutionToGroundedTime(vesselState);

            // energy-based completion shutoff
            if (!_complete && _solution != null && _solution.IsValid)
            {
                Vector3d rRel = vesselState.Position - vesselState.Body.position;
                double rMag = rRel.magnitude;
                if (rMag > 0.0)
                {
                    double targetRadius = vesselState.BodyRadius + 0.5 * (ascentProfile.TargetPeriapsisAlt + ascentProfile.TargetApoapsisAlt);
                    double e = 0.5 * vesselState.OrbitalVelocity.sqrMagnitude - vesselState.BodyGravParameter / rMag;
                    double targetPeRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;
                    bool energyReached = MathHelpers.IsFinite(_solution.TerminalSpecificEnergy) && e >= _solution.TerminalSpecificEnergy;
                    if (energyReached && J2MeanRadius(vesselState) >= targetRadius) { 
                        //if (energyReached && J2PeriapsisRadius(vesselState) >= targetPeRadius)
                        _complete = true;
                        _phase = PoweredGuidancePhase.Complete;
                        LogCompletion("psg-energy-complete", vesselState);
                    }
                }
            }

            // runaway backstop (stop the engines from burning perpetually if optimizer fails or gets stuck)
            if (!_complete
                && _optimizerFailCount >= MaxConsecutiveOptimizerFailures
                && vesselState.CurrentApoapsisAlt > ascentProfile.TargetApoapsisAlt)
            {
                _complete = true;
                _phase = PoweredGuidancePhase.Complete;
                LogCompletion("psg-failure-bailout", vesselState);
            }

            double velocityToGo = _solution != null && _solution.IsValid
                ? _solution.VelocityToGo(vesselState.UniversalTime)
                : EstimateVelocityToGo(vesselState, ascentProfile);
            double timeToGo = _solution != null && _solution.IsValid
                ? _solution.TimeToGo(vesselState.UniversalTime)
                : EstimateTimeToGoSeconds(vesselState, velocityToGo);

            //LogVelocityCheck(vesselState, timeToGo);

            if (_complete)
            {
                _phase = PoweredGuidancePhase.Complete;
                return CreateCommand(
                    PoweredGuidancePhase.Complete,
                    "PSG terminal guidance complete",
                    0.0,
                    profileHeadingDeg,
                    0.0,
                    apError,
                    peError,
                    0.0,
                    0.0,
                    true);
            }

            if (_solution != null && _solution.IsValid)
            {
                bool isExpired = IsSolutionExpired(vesselState.UniversalTime);

                if (IsPsgTerminalComplete(vesselState, ascentProfile))
                {
                    _complete = true;
                    _phase = PoweredGuidancePhase.Complete;
                    LogCompletion("psg-complete", vesselState);
                    return CreateCommand(
                        PoweredGuidancePhase.Complete,
                        "PSG terminal guidance complete",
                        0.0,
                        profileHeadingDeg,
                        0.0,
                        apError,
                        peError,
                        0.0,
                        0.0,
                        true);
                }

                if (isExpired && vesselState.UniversalTime > _solution.FinalUniversalTime + TerminalOverrunGraceSeconds)
                {
                    _complete = true;
                    _phase = PoweredGuidancePhase.Complete;
                    LogCompletion("psg-overrun", vesselState);
                    return CreateCommand(
                        PoweredGuidancePhase.Complete,
                        "PSG terminal overrun",
                        0.0,
                        profileHeadingDeg,
                        0.0,
                        apError,
                        peError,
                        0.0,
                        0.0,
                        true);
                }

                PsgGuidanceVector guidance = _solution.InertialGuidance(vesselState.UniversalTime);
                if (guidance != null && guidance.IsValid)
                {
                    // Hold the last followable steering: reject an ill-conditioned terminal solve that commands a
                    // turn the craft can't fly (orbit error -> 0 makes the thrust direction singular near cutoff).
                    double gateMaxRate = _solution.TimeToGo(vesselState.UniversalTime) <= TerminalSolveHorizonSeconds
                                        ? AttitudeControl.MaxControlRateRadPerSec(vesselState.Vessel)
                                        : 0.0;

                    guidance.InertialDirection = isExpired && _terminalGate.HasHeld
                        ? _terminalGate.Held
                        : _terminalGate.Update(
                            guidance.InertialDirection,
                            vesselState.UniversalTime,
                            gateMaxRate);

                    GetPitchHeadingFromInertial(vesselState, guidance.InertialDirection, out double psgPitch, out double psgHeading);

                    double commandedThrottle = _solution.TimeToGo(vesselState.UniversalTime) <= 0.0 ? 1.0 : guidance.Throttle;
                    _phase = PoweredGuidancePhase.PoweredGuidance;
                    string guidanceStatus = isExpired ? "PSG guidance overrun" :
                        IsSolutionStale(vesselState.UniversalTime) ? "PSG guidance stale" : "PSG guidance";

                    return CreateCommand(
                        PoweredGuidancePhase.PoweredGuidance,
                        guidanceStatus,
                        MathHelpers.Clamp(psgPitch, -30.0, 90.0),
                        psgHeading,
                        commandedThrottle, // was guidance.Throttle
                        apError,
                        peError,
                        timeToGo,
                        velocityToGo,
                        false,
                        true,
                        guidance.InertialDirection);
                }
            }

            _phase = profilePitchDeg >= 80.0
                ? PoweredGuidancePhase.VerticalAscent
                : PoweredGuidancePhase.PitchProgram;

            return CreateCommand(
                _phase,
                _solveTask != null ? "PSG solving" : _optimizerStatus,
                MathHelpers.Clamp(profilePitchDeg, -30.0, 90.0), 
                profileHeadingDeg,
                profileThrottle,
                apError,
                peError,
                timeToGo,
                velocityToGo,
                false);
        }

        private void UpdateOrbitPrediction(VesselState vs)
        {
            double rMag = (vs.Position - vs.Body.position).magnitude;
            if (rMag <= 0.0) return;
            double e = 0.5 * vs.OrbitalVelocity.sqrMagnitude - vs.BodyGravParameter / rMag;
            if (e >= 0.0) { PredictedApoapsisAlt = PredictedPeriapsisAlt = double.NaN; return; }  // not bound yet
            if (vs.UniversalTime - _lastPredictionUt < PredictionIntervalSeconds) return;
            _lastPredictionUt = vs.UniversalTime;

            BodyOblateness.Oblateness ob = BodyOblateness.For(vs.Body);
            Vector3d up = vs.Body.transform.up;
            Vector3d pole = new Vector3d(up.x, up.y, up.z).normalized;
            Vector3d r = vs.Position - vs.Body.position;
            double minR, maxR;
            J2Propagator.RadiusExtremes(r, vs.OrbitalVelocity, vs.BodyGravParameter,
                ob.J2, ob.ReferenceRadiusMeters, pole,
                TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds, out minR, out maxR);
            PredictedPeriapsisAlt = minR - vs.BodyRadius;
            PredictedApoapsisAlt = maxR - vs.BodyRadius;
        }

        private double J2MeanRadius(VesselState vs)
        {
            BodyOblateness.Oblateness ob = BodyOblateness.For(vs.Body);
            Vector3d up = vs.Body.transform.up;
            Vector3d pole = new Vector3d(up.x, up.y, up.z).normalized;
            Vector3d r = vs.Position - vs.Body.position;
            double minR, maxR;
            J2Propagator.RadiusExtremes(
                r, vs.OrbitalVelocity, vs.BodyGravParameter,
                ob.J2, ob.ReferenceRadiusMeters, pole,
                TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds, out minR, out maxR);
            return 0.5 * (minR + maxR);
        }

        private double J2PeriapsisRadius(VesselState vs)
        {
            BodyOblateness.Oblateness ob = BodyOblateness.For(vs.Body);
            Vector3d up = vs.Body.transform.up;
            Vector3d pole = new Vector3d(up.x, up.y, up.z).normalized;
            Vector3d r = vs.Position - vs.Body.position;
            return J2Propagator.NextPeriapsisRadius(
                r, vs.OrbitalVelocity, vs.BodyGravParameter,
                ob.J2, ob.ReferenceRadiusMeters, pole,
                TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds);
        }

        // J2 short-period periapsis offset (Keplerian Pe − real J2 Pe) for the current orbit
        private double TerminalJ2PeriapsisOffset(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetPeRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;
            double oscPeRadius = vesselState.BodyRadius + vesselState.CurrentPeriapsisAlt;
            if (!MathHelpers.IsFinite(oscPeRadius) || oscPeRadius < targetPeRadius - TerminalPeMarginMeters) return 0.0;

            BodyOblateness.Oblateness ob = BodyOblateness.For(vesselState.Body);
            if (ob.J2 == 0.0) return 0.0;

            Vector3 up = vesselState.Body.transform.up;
            Vector3d pole = new Vector3d(up.x, up.y, up.z).normalized;
            Vector3d r = vesselState.Position - vesselState.Body.position;

            double kepPe = J2Propagator.NextPeriapsisRadius(r, vesselState.OrbitalVelocity, vesselState.BodyGravParameter, 0.0, ob.ReferenceRadiusMeters, pole, TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds);
            double j2Pe = J2Propagator.NextPeriapsisRadius(r, vesselState.OrbitalVelocity, vesselState.BodyGravParameter, ob.J2, ob.ReferenceRadiusMeters, pole, TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds);
            return Math.Max(0.0, kepPe - j2Pe);
        }

        private void UpdatePsgSolution(
            VesselState vesselState,
            LaunchPlan launchPlan,
            AscentProfile ascentProfile,
            Vector3d initialThrustDirection)
        {

            // bridge to allow optimizer to continue if vessel stages mid-circularization
            int currentStage = vesselState.PoweredStages != null && vesselState.PoweredStages.Length > 0
                ? vesselState.PoweredStages[0].KspStage : _lastVesselStage;

            if (currentStage != _lastVesselStage && _lastVesselStage != int.MinValue)
            {
                _optimizerFailCount = 0;
                _runColdSolve = true;     // next solve ignores the stale warm-start
            }

            _lastVesselStage = currentStage;

            if (_solveTask != null && _solveTask.IsCompleted)
            {
                PsgOptimizationResult result = _solveTask.Result;
                PsgProblem completedProblem = _pendingProblem;
                _solveTask = null;
                _pendingProblem = null;

                _optimizerStatus = result != null ? result.Status : "PSG solver returned no result";
                _optimizerIterations = result != null ? result.Iterations : 0;
                _constraintViolation = result != null ? result.ConstraintViolation : double.NaN;
                bbLogger.Write(completedProblem, result);

                if (result != null && result.Success && result.Solution != null)
                {
                    _optimizerFailCount = 0;
                    _solution = result.Solution;
                    // No lock reset here: the gate self-limits (it only rejects unflyable commands), so it tracks
                    // the live direction every frame and needs no terminal-window arming.
                }
                else
                {
                    _optimizerFailCount++;
                }
            }

            if (_solveTask != null) return;

            double interval = GetSolveIntervalSeconds(vesselState.UniversalTime);
            if (vesselState.UniversalTime - _lastSolveRequestUt < interval) return;
            double j2Bias = TerminalJ2PeriapsisOffset(vesselState, ascentProfile);

            PsgTarget target = PsgTarget.FromPlan(vesselState, launchPlan, ascentProfile, j2Bias);

            if (target == null || !target.IsValid)
            {
                _optimizerStatus = target != null ? target.ReasonUnavailable : "PSG target unavailable";
                return;
            }

            //PsgPhase[] phases = PsgPhase.FromPoweredStages(vesselState.PoweredStages);
            PoweredStageInfo[] ascentStages = TrimStagesToOrbit(vesselState.PoweredStages, EstimateVelocityToGo(vesselState, ascentProfile));
            PsgPhase[] phases = PsgPhase.FromPoweredStages(ascentStages);

            if (phases == null || phases.Length == 0)
            {
                _optimizerStatus = "No powered PSG phases";
                return;
            }

            PsgProblem problem = PsgProblem.Create(vesselState, target, phases, initialThrustDirection);
            if (problem == null || !problem.IsValid)
            {
                _optimizerStatus = problem != null ? problem.ReasonUnavailable : "PSG problem unavailable";
                return;
            }

            PsgSolution warmStart = _runColdSolve ? null : _solution;
            _runColdSolve = false;

            _lastSolveRequestUt = vesselState.UniversalTime;
            _optimizerStatus = "PSG solving";
            _pendingProblem = problem;
            bbLogger.Write(problem);
            _solveTask = Task.Run(() => _optimizer.Solve(problem, warmStart));
        }

        private void PinSolutionToGroundedTime(VesselState vesselState)
        {
            if (_solution == null || !_solution.IsValid || vesselState == null || vesselState.Vessel == null) return;

            Vessel.Situations situation = vesselState.Vessel.situation;
            if (situation != Vessel.Situations.PRELAUNCH &&
                situation != Vessel.Situations.LANDED &&
                situation != Vessel.Situations.SPLASHED)
            {
                return;
            }

            _solution.ShiftStartUniversalTime(vesselState.UniversalTime);
        }

        private bool IsSolutionStale(double universalTime)
        {
            return _solution == null ||
                   !_solution.IsValid ||
                   universalTime - _solution.CreatedUniversalTime > SolutionStaleSeconds;
        }

        private bool IsSolutionExpired(double universalTime)
        {
            return _solution != null &&
                   _solution.IsValid &&
                   universalTime > _solution.FinalUniversalTime + ExpiredSolutionGraceSeconds;
        }

        private double GetSolveIntervalSeconds(double universalTime)
        {
            if (_solution == null || !_solution.IsValid) return RetryIntervalSeconds;

            double timeToGo = _solution.TimeToGo(universalTime);
            return timeToGo <= TerminalSolveHorizonSeconds
                ? TerminalSolveIntervalSeconds
                : SolveIntervalSeconds;
        }

        //private bool IsPsgTerminalComplete(VesselState vesselState)
        //{
        //    if (_solution == null || !_solution.IsValid) return false;
        //    if (IsSolutionExpired(vesselState.UniversalTime)) return false;

        //    // if (vesselState.UniversalTime >= _solution.FinalUniversalTime) return true;

        //    Vector3d relativePosition = vesselState.Position - vesselState.Body.position;
        //    return _solution.TerminalGuidanceSatisfied(relativePosition, vesselState.OrbitalVelocity, vesselState.BodyGravParameter);
        //}

        private bool IsPsgTerminalComplete(VesselState vesselState, AscentProfile ascentProfile)
        {
            if (_solution == null || !_solution.IsValid) return false;
            if (IsSolutionExpired(vesselState.UniversalTime)) return false;

            double targetPeRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;

            // wait until we're in range
            double osculatingPeRadius = vesselState.BodyRadius + vesselState.CurrentPeriapsisAlt;
            if (!MathHelpers.IsFinite(osculatingPeRadius) ||
                osculatingPeRadius < targetPeRadius - TerminalPeMarginMeters) return false;

            BodyOblateness.Oblateness ob = BodyOblateness.For(vesselState.Body);
            Vector3d up = vesselState.Body.transform.up; // Claude used Vector3 for this but i disagree
            Vector3d pole = new Vector3d(up.x, up.y, up.z).normalized;
            Vector3d r = vesselState.Position - vesselState.Body.position;

            double realPeRadius = J2Propagator.NextPeriapsisRadius(
                r, vesselState.OrbitalVelocity, vesselState.BodyGravParameter,
                ob.J2, ob.ReferenceRadiusMeters, pole,
                TerminalPropagateMaxSeconds, TerminalPropagateStepSeconds);

            return realPeRadius >= targetPeRadius;
        }

        private Vector3d GetCurrentThrustDirection(VesselState vesselState, double profileHeadingDeg, double profilePitchDeg)
        {
            if (_solution != null && _solution.IsValid)
            {
                PsgGuidanceVector guidance = _solution.InertialGuidance(vesselState.UniversalTime);
                if (guidance != null && guidance.IsValid && guidance.InertialDirection.sqrMagnitude > 0.0)
                    return guidance.InertialDirection.normalized;
            }
            return GetSurfaceCommandDirection(vesselState, profileHeadingDeg, profilePitchDeg);
        }

        private static Vector3d GetSurfaceCommandDirection(
            VesselState vesselState,
            double headingDeg,
            double pitchDeg)
        {
            if (vesselState == null || vesselState.Body == null) return Vector3d.zero;

            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vesselState.Body.transform.up).normalized;
            if (north.sqrMagnitude <= 0.0) return up;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            double headingRad = MathHelpers.Deg2Rad(headingDeg);
            double pitchRad = MathHelpers.Deg2Rad(pitchDeg);
            Vector3d horizontal = north * Math.Cos(headingRad) + east * Math.Sin(headingRad);

            return (horizontal * Math.Cos(pitchRad) + up * Math.Sin(pitchRad)).normalized;
        }

        private static void GetPitchHeadingFromInertial(
            VesselState vesselState,
            Vector3d inertialDirection,
            out double pitchDeg,
            out double headingDeg)
        {
            pitchDeg = 90.0;
            headingDeg = 90.0;

            if (vesselState == null || vesselState.Body == null || inertialDirection.sqrMagnitude <= 0.0) return;

            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vesselState.Body.transform.up).normalized;
            if (north.sqrMagnitude <= 0.0) return;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            Vector3d direction = inertialDirection.normalized;
            Vector3d horizontal = Vector3d.Exclude(up, direction);

            pitchDeg = Math.Asin(MathHelpers.Clamp(Vector3d.Dot(direction, up), -1.0, 1.0)) * 180.0 / Math.PI;

            if (horizontal.sqrMagnitude > 0.0)
            {
                Vector3d horizontalDirection = horizontal.normalized;
                headingDeg = MathHelpers.NormalizeDegrees(
                    Math.Atan2(
                        Vector3d.Dot(horizontalDirection, east),
                        Vector3d.Dot(horizontalDirection, north)) *
                    180.0 / Math.PI);
            }
        }

        private static bool HasUsableOrbitState(VesselState vesselState, double targetAp, double targetPe)
        {
            return vesselState.Body != null &&
                   MathHelpers.IsFinite(vesselState.BodyRadius) &&
                   MathHelpers.IsFinite(vesselState.BodyGravParameter) &&
                   //MathHelpers.IsFinite(vesselState.CurrentApoapsisAlt) &&
                   //MathHelpers.IsFinite(vesselState.CurrentPeriapsisAlt) &&
                   MathHelpers.IsFinite(targetAp) &&
                   MathHelpers.IsFinite(targetPe);
        }

        private static PoweredStageInfo[] TrimStagesToOrbit(PoweredStageInfo[] stages, double velocityToGo)
        {
            if (stages == null || stages.Length <= 1) return stages;
            if (!MathHelpers.IsFinite(velocityToGo)) return stages;

            double needed = velocityToGo * StageTrimVelocityMargin;
            double cumulativeDv = 0.0;
            int count = 0;
            for (int i = 0; i < stages.Length; i++)
            {
                count++;
                cumulativeDv += StageVacuumDeltaV(stages[i]);
                if (cumulativeDv >= needed) break;
            }

            if (count >= stages.Length) return stages; // need them all (or estimate exceeds total available dv)

            var trimmed = new PoweredStageInfo[count];
            System.Array.Copy(stages, trimmed, count);
            return trimmed;
        }

        private static double StageVacuumDeltaV(PoweredStageInfo stage)
        {
            if (stage == null) return 0.0;
            if (!MathHelpers.IsFinite(stage.StartMass) || !MathHelpers.IsFinite(stage.EndMass) ||
                stage.EndMass <= 0.0 || stage.StartMass <= stage.EndMass ||
                !MathHelpers.IsFinite(stage.VacuumSpecificImpulse) || stage.VacuumSpecificImpulse <= 0.0)
            {
                return 0.0;
            }
            return stage.VacuumSpecificImpulse * 9.80665 * Math.Log(stage.StartMass / stage.EndMass);
        }

        private static double EstimateVelocityToGo(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetSpeed = OrbitMath.GetCircularVelocity(
                vesselState.Body,
                (ascentProfile.TargetApoapsisAlt + ascentProfile.TargetPeriapsisAlt) * 0.5);
            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            double currentHorizontal = Vector3d.Exclude(up, vesselState.OrbitalVelocity).magnitude;

            return MathHelpers.IsFinite(targetSpeed)
                ? Math.Max(0.0, targetSpeed - currentHorizontal)
                : double.NaN;
        }

        private static double EstimateTimeToGoSeconds(VesselState vesselState, double velocityToGo)
        {
            if (!MathHelpers.IsFinite(velocityToGo) || velocityToGo <= 0.0) return 0.0;
            if (!MathHelpers.IsFinite(vesselState.AvailableThrust) || vesselState.AvailableThrust <= 0.0) return double.NaN;
            if (!MathHelpers.IsFinite(vesselState.TotalMass) || vesselState.TotalMass <= 0.0) return double.NaN;

            double acceleration = vesselState.AvailableThrust / vesselState.TotalMass;
            return acceleration > 0.0 ? velocityToGo / acceleration : double.NaN;
        }

        private static PoweredGuidanceCommand CreateUnavailable(double pitchDeg, double headingDeg, double throttle)
        {
            return CreateCommand(
                PoweredGuidancePhase.Unavailable,
                "Guidance unavailable",
                pitchDeg,
                headingDeg,
                throttle,
                double.NaN,
                double.NaN,
                double.NaN,
                double.NaN,
                false);
        }

        private static PoweredGuidanceCommand CreateCommand(
            PoweredGuidancePhase phase,
            string status,
            double pitchDeg,
            double headingDeg,
            double throttle,
            double apError,
            double peError,
            double timeToGo,
            double velocityToGo,
            bool isComplete)
        {
            return CreateCommand(
                phase,
                status,
                pitchDeg,
                headingDeg,
                throttle,
                apError,
                peError,
                timeToGo,
                velocityToGo,
                isComplete,
                false,
                Vector3d.zero,
                string.Empty,
                0,
                double.NaN);
        }

        private PoweredGuidanceCommand CreateCommand(
            PoweredGuidancePhase phase,
            string status,
            double pitchDeg,
            double headingDeg,
            double throttle,
            double apError,
            double peError,
            double timeToGo,
            double velocityToGo,
            bool isComplete,
            bool hasInertialDirection,
            Vector3d inertialDirection)
        {
            return CreateCommand(
                phase,
                status,
                pitchDeg,
                headingDeg,
                throttle,
                apError,
                peError,
                timeToGo,
                velocityToGo,
                isComplete,
                hasInertialDirection,
                inertialDirection,
                _optimizerStatus,
                _optimizerIterations,
                _constraintViolation);
        }

        private static PoweredGuidanceCommand CreateCommand(
            PoweredGuidancePhase phase,
            string status,
            double pitchDeg,
            double headingDeg,
            double throttle,
            double apError,
            double peError,
            double timeToGo,
            double velocityToGo,
            bool isComplete,
            bool hasInertialDirection,
            Vector3d inertialDirection,
            string optimizerStatus,
            int optimizerIterations,
            double constraintViolation)
        {
            return new PoweredGuidanceCommand
            {
                Phase = phase,
                Status = status,
                PitchDeg = pitchDeg,
                HeadingDeg = MathHelpers.NormalizeDegrees(headingDeg),
                Throttle = MathHelpers.Clamp(throttle, 0.0, 1.0),
                HasInertialDirection = hasInertialDirection,
                InertialDirection = hasInertialDirection ? inertialDirection.normalized : Vector3d.zero,
                ApoapsisErrorMeters = apError,
                PeriapsisErrorMeters = peError,
                TimeToGoSeconds = timeToGo,
                VelocityToGoMetersPerSecond = velocityToGo,
                OptimizerStatus = optimizerStatus,
                OptimizerIterations = optimizerIterations,
                SolutionConstraintViolation = constraintViolation,
                IsComplete = isComplete
            };
        }
    }
}
