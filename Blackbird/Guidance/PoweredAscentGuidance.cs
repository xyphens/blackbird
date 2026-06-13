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
        // TODO(MechJeb parity): keep this AP/PE box as a fallback guard only.
        // Nominal PSG cutoff should be owned by terminal guidance: terminal-class target constraints,
        // next-tick invariant crossing, terminal-stage state, and bounded overburn protection.
        private const double MinimumTerminalToleranceMeters = 500.0;
        private const double MaximumTerminalToleranceMeters = 5000.0;
        private const double TerminalToleranceFraction = 0.025;

        // Fallback-only: used when PSG has no usable solution and the legacy pitch profile must hand off to heuristic insertion guidance.
        private const double PitchProgramApoapsisMarginFraction = 0.12;
        // TODO(MechJeb parity): replace timer polling with a GuidanceController/PSGGlueBall-style
        // state machine that separates burning, coasting, terminal, staging, and RCS decisions.
        private const double SolveIntervalSeconds = 5.0;
        private const double RetryIntervalSeconds = 2.0;
        // MechJeb enters terminal guidance around 10s remaining and locks the inertial heading for the final 2s.
        private const double TerminalSolveHorizonSeconds = 15.0;
        private const double TerminalSolveIntervalSeconds = 0.75;
        private const double SolutionStaleSeconds = 20.0;
        private const double ExpiredSolutionGraceSeconds = 0.25;
        private const double TerminalGuidanceLockSeconds = 2.0;

        private readonly PsgOptimizer _optimizer = new PsgOptimizer();
        private PoweredGuidancePhase _phase = PoweredGuidancePhase.Unavailable;
        private bool _complete;
        private bool _insertionCutoff;
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
            _insertionCutoff = false;
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

        // Produces the powered-ascent command; LaunchHandler remains responsible for applying it to the vessel.
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

            double apTolerance = GetTerminalTolerance(targetAp);
            double peTolerance = GetTerminalTolerance(targetPe);
            double velocityToGo = _solution != null && _solution.IsValid
                ? _solution.VelocityToGo(vesselState.UniversalTime)
                : EstimateVelocityToGo(vesselState, ascentProfile);
            double timeToGo = _solution != null && _solution.IsValid
                ? _solution.TimeToGo(vesselState.UniversalTime)
                : EstimateTimeToGoSeconds(vesselState, velocityToGo);

            if (_complete)
            {
                _complete = true;
                _phase = PoweredGuidancePhase.Complete;
                return CreateCommand(
                    PoweredGuidancePhase.Complete,
                    "Insertion target reached",
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
                Vector3d relativePosition = vesselState.Position - vesselState.Body.position;
                if (IsPsgTerminalComplete(
                    vesselState,
                    relativePosition,
                    apError,
                    peError,
                    apTolerance,
                    peTolerance))
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
                if (guidance != null && guidance.IsValid && !IsSolutionExpired(vesselState.UniversalTime))
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

            if (IsOrbitInsideTerminalBox(apError, peError, apTolerance, peTolerance))
            {
                _complete = true;
                _phase = PoweredGuidancePhase.Complete;
                return CreateCommand(
                    PoweredGuidancePhase.Complete,
                    "Insertion target reached",
                    0.0,
                    profileHeadingDeg,
                    0.0,
                    apError,
                    peError,
                    0.0,
                    0.0,
                    true);
            }

            if (_insertionCutoff || IsPeriapsisAtInsertionTarget(peError, peTolerance))
            {
                _insertionCutoff = true;
                _phase = PoweredGuidancePhase.InsertionCutoff;
                return CreateCommand(
                    PoweredGuidancePhase.InsertionCutoff,
                    "Insertion cutoff - periapsis reached",
                    0.0,
                    profileHeadingDeg,
                    0.0,
                    apError,
                    peError,
                    timeToGo,
                    velocityToGo,
                    false);
            }

            PoweredGuidancePhase phase = SelectPhase(vesselState, ascentProfile, profilePitchDeg, apError, apTolerance);
            double pitch = phase == PoweredGuidancePhase.VerticalAscent || phase == PoweredGuidancePhase.PitchProgram
                ? profilePitchDeg
                : GetPoweredGuidancePitchDeg(vesselState, ascentProfile, apError, peError, apTolerance, peTolerance);

            double throttle = GetPoweredThrottle(phase, vesselState, ascentProfile, apError, peError, apTolerance, peTolerance, profileThrottle);

            _phase = phase;

            return CreateCommand(
                phase,
                GetPhaseStatus(phase),
                ClampPitchForControl(pitch),
                profileHeadingDeg,
                throttle,
                apError,
                peError,
                timeToGo,
                velocityToGo,
                false,
                false,
                Vector3d.zero);
        }

        // Keeps early flight on the planned gravity turn, then hands over to orbital-element feedback.
        private PoweredGuidancePhase SelectPhase(
            VesselState vesselState,
            AscentProfile ascentProfile,
            double profilePitchDeg,
            double apError,
            double apTolerance)
        {
            if (_phase == PoweredGuidancePhase.PoweredGuidance || _phase == PoweredGuidancePhase.Terminal)
            {
                // Fallback-only: PSG terminal completion is handled before this path.
                return Math.Abs(apError) <= apTolerance * 2.0
                    ? PoweredGuidancePhase.Terminal
                    : PoweredGuidancePhase.PoweredGuidance;
            }

            // Fallback-only: keep the legacy profile vertical until the pitch curve commits to the gravity turn.
            if (profilePitchDeg >= 80.0 && vesselState.AltitudeMeters < GetTurnCommitAltitude(ascentProfile))
            {
                return PoweredGuidancePhase.VerticalAscent;
            }

            double pitchProgramMargin = Math.Max(apTolerance, ascentProfile.TargetApoapsisAlt * PitchProgramApoapsisMarginFraction);
            if (profilePitchDeg > 3.0 && apError > pitchProgramMargin)
            {
                return PoweredGuidancePhase.PitchProgram;
            }

            // Fallback-only: PSG terminal completion is handled before this path.
            return Math.Abs(apError) <= apTolerance * 2.0
                ? PoweredGuidancePhase.Terminal
                : PoweredGuidancePhase.PoweredGuidance;
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

        private bool IsPsgTerminalComplete(
            VesselState vesselState,
            Vector3d relativePosition,
            double apError,
            double peError,
            double apTolerance,
            double peTolerance)
        {
            if (_solution == null || !_solution.IsValid) return false;

            Vector3d terminalPosition;
            Vector3d terminalVelocity;
            PredictNextPhysicsState(vesselState, relativePosition, out terminalPosition, out terminalVelocity);

            // MechJeb-style cutoff is angular-momentum crossing against the selected terminal target.
            // Remaining parity work is in selecting/building that terminal target, not in AP/PE tuning here.
            return _solution.TerminalGuidanceSatisfied(terminalPosition, terminalVelocity);
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

            // Mirrors MechJeb's terminal cutoff prediction for the next physics tick. RCS/staging-specific
            // two-tick handling belongs with the terminal state machine once it exists.
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

        // Fallback-only steering: chase the target orbit's angular momentum while damping radial motion.
        private static double GetPoweredGuidancePitchDeg(
            VesselState vesselState,
            AscentProfile ascentProfile,
            double apError,
            double peError,
            double apTolerance,
            double peTolerance)
        {
            double targetOrbitPitch = GetTargetOrbitPitchDeg(vesselState, ascentProfile);
            double elementPitch = GetOrbitalElementCorrectionPitchDeg(vesselState, ascentProfile, apError, peError, apTolerance, peTolerance);

            // Fallback-only: PSG supplies the solved thrust-vector direction when available.
            return OrbitMath.Clamp(targetOrbitPitch * 0.65 + elementPitch * 0.35, -12.0, 28.0);
        }

        // Points toward the velocity vector that the requested AP/PE would have at the current radius.
        private static double GetTargetOrbitPitchDeg(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetApRadius = vesselState.BodyRadius + ascentProfile.TargetApoapsisAlt;
            double targetPeRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;
            double currentRadius = (vesselState.Position - vesselState.Body.position).magnitude;

            if (currentRadius <= 0.0 || targetApRadius <= 0.0 || targetPeRadius <= 0.0)
            {
                return 0.0;
            }

            double semiMajorAxis = (targetApRadius + targetPeRadius) * 0.5;
            double semiLatusRectum = 2.0 * targetApRadius * targetPeRadius / (targetApRadius + targetPeRadius);
            double targetHorizontalSpeed = Math.Sqrt(vesselState.BodyGravParameter * semiLatusRectum) / currentRadius;
            double currentHorizontalSpeed = GetInertialHorizontalSpeed(vesselState);
            double horizontalToGo = targetHorizontalSpeed - currentHorizontalSpeed;
            double desiredRadialSpeed = GetDesiredRadialSpeed(vesselState, ascentProfile);
            double radialToGo = desiredRadialSpeed - GetInertialRadialSpeed(vesselState);

            if (!OrbitMath.IsFinite(horizontalToGo) || !OrbitMath.IsFinite(radialToGo))
            {
                return 0.0;
            }

            // Fallback-only: avoid near-zero denominator jitter in early heuristic guidance.
            double forwardToGo = Math.Max(50.0, Math.Abs(horizontalToGo));
            return Math.Atan2(radialToGo, forwardToGo) * 180.0 / Math.PI;
        }

        // Adds direct AP/PE feedback so finite burns do not run away while chasing only periapsis.
        private static double GetOrbitalElementCorrectionPitchDeg(
            VesselState vesselState,
            AscentProfile ascentProfile,
            double apError,
            double peError,
            double apTolerance,
            double peTolerance)
        {
            double apScale = Math.Max(apTolerance * 4.0, ascentProfile.TargetApoapsisAlt * 0.20);
            double peScale = Math.Max(peTolerance * 4.0, ascentProfile.TargetPeriapsisAlt * 0.20);

            double pitch = 0.0;

            // Fallback-only: proportional AP/PE shaping when no PSG vector is available.
            pitch += OrbitMath.Clamp(apError / apScale, -1.0, 1.0) * 18.0;

            if (peError > peTolerance)
            {
                pitch -= OrbitMath.Clamp(peError / peScale, 0.0, 1.0) * 5.0;
            }

            // Fallback-only: descent guard to prevent digging into the atmosphere during missed insertions.
            if (vesselState.VerticalSpeed < -5.0 && vesselState.AltitudeMeters < ascentProfile.TargetPeriapsisAlt)
            {
                pitch += 6.0;
            }

            return OrbitMath.Clamp(pitch, -12.0, 28.0);
        }

        // Fallback-only radial-speed schedule instead of coasting to apoapsis.
        private static double GetDesiredRadialSpeed(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetRadius = vesselState.BodyRadius +
                                  (ascentProfile.TargetApoapsisAlt + ascentProfile.TargetPeriapsisAlt) * 0.5;
            double currentRadius = (vesselState.Position - vesselState.Body.position).magnitude;
            double altitudeError = targetRadius - currentRadius;

            // Fallback-only: PSG supplies time-to-go/radius behavior when available.
            return OrbitMath.Clamp(altitudeError / 45.0, -75.0, 220.0);
        }

        // Treats throttle as start/stop so engines without useful throttling remain compatible.
        private static double GetPoweredThrottle(
            PoweredGuidancePhase phase,
            VesselState vesselState,
            AscentProfile ascentProfile,
            double apError,
            double peError,
            double apTolerance,
            double peTolerance,
            double profileThrottle)
        {
            if (phase == PoweredGuidancePhase.Complete || phase == PoweredGuidancePhase.Unavailable) return 0.0;

            if (phase == PoweredGuidancePhase.VerticalAscent || phase == PoweredGuidancePhase.PitchProgram)
            {
                return profileThrottle > 0.0 ? 1.0 : 0.0;
            }

            return 1.0;
        }

        private static bool IsPeriapsisAtInsertionTarget(double peError, double peTolerance)
        {
            return peError <= peTolerance;
        }

        private static bool IsOrbitInsideTerminalBox(double apError, double peError, double apTolerance, double peTolerance)
        {
            return Math.Abs(apError) <= apTolerance && Math.Abs(peError) <= peTolerance;
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

        private static double GetTurnCommitAltitude(AscentProfile ascentProfile)
        {
            // Fallback-only: turn start comes from the ascent profile while PSG is unavailable.
            if (ascentProfile == null || ascentProfile.Points == null || ascentProfile.Points.Length == 0) return 1000.0;

            return Math.Max(1000.0, ascentProfile.Points[0].AltitudeMeters + 500.0);
        }

        private static double GetTerminalTolerance(double targetAltitude)
        {
            // TODO(MechJeb parity): remove this from nominal PSG completion once terminal guidance owns cutoff.
            // It should remain only as a fallback/emergency guard for non-PSG insertion guidance.
            return OrbitMath.Clamp(
                Math.Abs(targetAltitude) * TerminalToleranceFraction,
                MinimumTerminalToleranceMeters,
                MaximumTerminalToleranceMeters);
        }

        private static double GetInertialRadialSpeed(VesselState vesselState)
        {
            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            return Vector3d.Dot(vesselState.OrbitalVelocity, up);
        }

        private static double GetInertialHorizontalSpeed(VesselState vesselState)
        {
            Vector3d up = (vesselState.Position - vesselState.Body.position).normalized;
            return Vector3d.Exclude(up, vesselState.OrbitalVelocity).magnitude;
        }

        private static double EstimateVelocityToGo(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetSpeed = OrbitMath.GetCircularVelocity(
                vesselState.Body,
                (ascentProfile.TargetApoapsisAlt + ascentProfile.TargetPeriapsisAlt) * 0.5);
            double currentHorizontal = GetInertialHorizontalSpeed(vesselState);

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

        private static string GetPhaseStatus(PoweredGuidancePhase phase)
        {
            switch (phase)
            {
                case PoweredGuidancePhase.VerticalAscent:
                    return "Vertical ascent";
                case PoweredGuidancePhase.PitchProgram:
                    return "Pitch program";
                case PoweredGuidancePhase.PoweredGuidance:
                    return "Powered guidance";
                case PoweredGuidancePhase.Terminal:
                    return "Terminal insertion";
                case PoweredGuidancePhase.Complete:
                    return "Insertion target reached";
                case PoweredGuidancePhase.InsertionCutoff:
                    return "Insertion cutoff - periapsis reached";
                default:
                    return "Guidance unavailable";
            }
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
