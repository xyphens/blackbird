using System;
using System.Threading.Tasks;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PoweredAscentGuidance
    {
        private const double SolveIntervalSeconds = 5.0;
        private const double RetryIntervalSeconds = 1.0;
        private const double TerminalSolveHorizonSeconds = 10.0;
        private const double TerminalSolveIntervalSeconds = 0.5;
        private const double SolutionStaleSeconds = 20.0;
        private const double ExpiredSolutionGraceSeconds = 0.25;
        private const double TerminalGuidanceLockSeconds = 2.0;

        private readonly PsgOptimizer _optimizer = new PsgOptimizer();
        private PoweredGuidancePhase _phase = PoweredGuidancePhase.Unavailable;
        private bool _complete;
        private Task<PsgOptimizationResult> _solveTask;
        private PsgProblem _pendingProblem;
        private PsgSolution _solution;
        private Vector3d _lockedTerminalDirection = Vector3d.zero;
        private bool _hasLockedTerminalDirection;
        private double _lastSolveRequestUt = double.NegativeInfinity;
        private string _optimizerStatus = "PSG idle";
        private int _optimizerIterations;
        private double _constraintViolation = double.NaN;

        public void Reset()
        {
            _phase = PoweredGuidancePhase.Unavailable;
            _complete = false;
            _solveTask = null;
            _pendingProblem = null;
            _solution = null;
            _lockedTerminalDirection = Vector3d.zero;
            _hasLockedTerminalDirection = false;
            _lastSolveRequestUt = double.NegativeInfinity;
            _optimizerStatus = "PSG idle";
            _optimizerIterations = 0;
            _constraintViolation = double.NaN;
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

            Vector3d initialThrustDirection = GetSurfaceCommandDirection(vesselState, profileHeadingDeg, profilePitchDeg);
            UpdatePsgSolution(vesselState, launchPlan, ascentProfile, initialThrustDirection);
            PinSolutionToGroundedTime(vesselState);

            double velocityToGo = _solution != null && _solution.IsValid
                ? _solution.VelocityToGo(vesselState.UniversalTime)
                : EstimateVelocityToGo(vesselState, ascentProfile);
            double timeToGo = _solution != null && _solution.IsValid
                ? _solution.TimeToGo(vesselState.UniversalTime)
                : EstimateTimeToGoSeconds(vesselState, velocityToGo);

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

            if (_solution != null && _solution.IsValid && !IsSolutionExpired(vesselState.UniversalTime))
            {
                Vector3d relativePosition = vesselState.Position - vesselState.Body.position;
                if (IsPsgTerminalComplete(vesselState, relativePosition))
                {
                    _complete = true;
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

                PsgGuidanceVector guidance = _solution.InertialGuidance(vesselState.UniversalTime);
                if (guidance != null && guidance.IsValid)
                {
                    if (timeToGo <= TerminalGuidanceLockSeconds)
                    {
                        if (!_hasLockedTerminalDirection)
                        {
                            _lockedTerminalDirection = guidance.InertialDirection.normalized;
                            _hasLockedTerminalDirection = true;
                        }

                        guidance.InertialDirection = _lockedTerminalDirection;
                    }
                    else
                    {
                        _hasLockedTerminalDirection = false;
                        _lockedTerminalDirection = Vector3d.zero;
                    }

                    double psgPitch;
                    double psgHeading;
                    GetPitchHeadingFromInertial(vesselState, guidance.InertialDirection, out psgPitch, out psgHeading);

                    _phase = PoweredGuidancePhase.PoweredGuidance;
                    return CreateCommand(
                        PoweredGuidancePhase.PoweredGuidance,
                        IsSolutionStale(vesselState.UniversalTime) ? "PSG guidance stale" : "PSG guidance",
                        ClampPitchForControl(psgPitch),
                        psgHeading,
                        guidance.Throttle,
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
                ClampPitchForControl(profilePitchDeg),
                profileHeadingDeg,
                profileThrottle,
                apError,
                peError,
                timeToGo,
                velocityToGo,
                false);
        }

        private void UpdatePsgSolution(
            VesselState vesselState,
            LaunchPlan launchPlan,
            AscentProfile ascentProfile,
            Vector3d initialThrustDirection)
        {
            if (_solveTask != null && _solveTask.IsCompleted)
            {
                PsgOptimizationResult result = _solveTask.Result;
                PsgProblem completedProblem = _pendingProblem;
                _solveTask = null;
                _pendingProblem = null;

                _optimizerStatus = result != null ? result.Status : "PSG solver returned no result";
                _optimizerIterations = result != null ? result.Iterations : 0;
                _constraintViolation = result != null ? result.ConstraintViolation : double.NaN;
                PsgSnapshotLogger.WriteResult(completedProblem, result);

                if (result != null && result.Success && result.Solution != null)
                {
                    _solution = result.Solution;
                    if (_solution.TimeToGo(vesselState.UniversalTime) > TerminalGuidanceLockSeconds)
                    {
                        _hasLockedTerminalDirection = false;
                        _lockedTerminalDirection = Vector3d.zero;
                    }
                }
            }

            if (_solveTask != null) return;

            double interval = GetSolveIntervalSeconds(vesselState.UniversalTime);
            if (vesselState.UniversalTime - _lastSolveRequestUt < interval) return;

            PsgTarget target = PsgTarget.FromPlan(vesselState, launchPlan, ascentProfile);
            if (target == null || !target.IsValid)
            {
                _optimizerStatus = target != null ? target.ReasonUnavailable : "PSG target unavailable";
                return;
            }

            PsgPhase[] phases = PsgPhase.FromPoweredStages(vesselState.PoweredStages);
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

            PsgSolution warmStart = _solution;
            _lastSolveRequestUt = vesselState.UniversalTime;
            _optimizerStatus = "PSG solving";
            _pendingProblem = problem;
            PsgSnapshotLogger.Write(problem, "solve requested");
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

        private bool IsPsgTerminalComplete(VesselState vesselState, Vector3d relativePosition)
        {
            if (_solution == null || !_solution.IsValid) return false;

            Vector3d terminalPosition;
            Vector3d terminalVelocity;
            PredictNextPhysicsState(vesselState, relativePosition, out terminalPosition, out terminalVelocity);

            return _solution.TerminalGuidanceSatisfied(terminalPosition, terminalVelocity, vesselState.UniversalTime);
        }

        private static void PredictNextPhysicsState(
            VesselState vesselState,
            Vector3d relativePosition,
            out Vector3d predictedPosition,
            out Vector3d predictedVelocity)
        {
            predictedPosition = relativePosition;
            predictedVelocity = vesselState != null ? vesselState.OrbitalVelocity : Vector3d.zero;

            Vessel vessel = vesselState != null ? vesselState.Vessel : null;
            if (vessel == null) return;

            double dt = Math.Max(0.0, TimeWarp.fixedDeltaTime);
            Vector3d acceleration = vessel.acceleration_immediate;

            predictedPosition = relativePosition + vesselState.OrbitalVelocity * dt + 0.5 * acceleration * dt * dt;
            predictedVelocity = vesselState.OrbitalVelocity + acceleration * dt;
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

            double headingRad = headingDeg * Math.PI / 180.0;
            double pitchRad = pitchDeg * Math.PI / 180.0;
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

            pitchDeg = Math.Asin(OrbitMath.Clamp(Vector3d.Dot(direction, up), -1.0, 1.0)) * 180.0 / Math.PI;

            if (horizontal.sqrMagnitude > 0.0)
            {
                Vector3d horizontalDirection = horizontal.normalized;
                headingDeg = OrbitMath.NormalizeDegrees(
                    Math.Atan2(
                        Vector3d.Dot(horizontalDirection, east),
                        Vector3d.Dot(horizontalDirection, north)) *
                    180.0 / Math.PI);
            }
        }

        private static bool HasUsableOrbitState(VesselState vesselState, double targetAp, double targetPe)
        {
            return vesselState.Body != null &&
                   OrbitMath.IsFinite(vesselState.BodyRadius) &&
                   OrbitMath.IsFinite(vesselState.BodyGravParameter) &&
                   OrbitMath.IsFinite(vesselState.CurrentApoapsisAlt) &&
                   OrbitMath.IsFinite(vesselState.CurrentPeriapsisAlt) &&
                   OrbitMath.IsFinite(targetAp) &&
                   OrbitMath.IsFinite(targetPe);
        }

        private static double EstimateVelocityToGo(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetSpeed = OrbitMath.GetCircularVelocity(
                vesselState.Body,
                (ascentProfile.TargetApoapsisAlt + ascentProfile.TargetPeriapsisAlt) * 0.5);
            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            double currentHorizontal = Vector3d.Exclude(up, vesselState.OrbitalVelocity).magnitude;

            return OrbitMath.IsFinite(targetSpeed)
                ? Math.Max(0.0, targetSpeed - currentHorizontal)
                : double.NaN;
        }

        private static double EstimateTimeToGoSeconds(VesselState vesselState, double velocityToGo)
        {
            if (!OrbitMath.IsFinite(velocityToGo) || velocityToGo <= 0.0) return 0.0;
            if (!OrbitMath.IsFinite(vesselState.AvailableThrust) || vesselState.AvailableThrust <= 0.0) return double.NaN;
            if (!OrbitMath.IsFinite(vesselState.TotalMass) || vesselState.TotalMass <= 0.0) return double.NaN;

            double acceleration = vesselState.AvailableThrust / vesselState.TotalMass;
            return acceleration > 0.0 ? velocityToGo / acceleration : double.NaN;
        }

        private static double ClampPitchForControl(double pitchDeg)
        {
            return OrbitMath.Clamp(pitchDeg, -30.0, 90.0);
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
                HeadingDeg = OrbitMath.NormalizeDegrees(headingDeg),
                Throttle = OrbitMath.Clamp(throttle, 0.0, 1.0),
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
