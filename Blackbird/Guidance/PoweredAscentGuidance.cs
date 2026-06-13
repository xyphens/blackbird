using System;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PoweredAscentGuidance
    {
        // FIXME: temporary heuristic tolerances until terminal error is derived from vehicle controllability.
        private const double MinimumTerminalToleranceMeters = 500.0;
        private const double MaximumTerminalToleranceMeters = 5000.0;

        // FIXME: temporary handoff margin until pitch-program exit is based on AP growth rate and time-to-go.
        private const double PitchProgramApoapsisMarginFraction = 0.12;

        private PoweredGuidancePhase _phase = PoweredGuidancePhase.Unavailable;
        private bool _complete;
        private bool _insertionCutoff;

        public void Reset()
        {
            _phase = PoweredGuidancePhase.Unavailable;
            _complete = false;
            _insertionCutoff = false;
        }

        // Produces the powered-ascent command; LaunchHandler remains responsible for applying it to the vessel.
        public PoweredGuidanceCommand GetCommand(
            VesselState vesselState,
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

            double apTolerance = GetTerminalTolerance(targetAp);
            double peTolerance = GetTerminalTolerance(targetPe);
            double velocityToGo = EstimateVelocityToGo(vesselState, ascentProfile);
            double timeToGo = EstimateTimeToGoSeconds(vesselState, velocityToGo);

            if (_complete || IsOrbitInsideTerminalBox(apError, peError, apTolerance, peTolerance))
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
                false);
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
                // FIXME: temporary terminal handoff band; replace with a solved guidance convergence check.
                return Math.Abs(apError) <= apTolerance * 2.0
                    ? PoweredGuidancePhase.Terminal
                    : PoweredGuidancePhase.PoweredGuidance;
            }

            // FIXME: temporary vertical-ascent gate; replace with a profile/atmosphere/TWR-derived condition.
            if (profilePitchDeg >= 80.0 && vesselState.AltitudeMeters < GetTurnCommitAltitude(ascentProfile))
            {
                return PoweredGuidancePhase.VerticalAscent;
            }

            double pitchProgramMargin = Math.Max(apTolerance, ascentProfile.TargetApoapsisAlt * PitchProgramApoapsisMarginFraction);
            if (profilePitchDeg > 3.0 && apError > pitchProgramMargin)
            {
                return PoweredGuidancePhase.PitchProgram;
            }

            // FIXME: temporary terminal handoff band; replace with a solved guidance convergence check.
            return Math.Abs(apError) <= apTolerance * 2.0
                ? PoweredGuidancePhase.Terminal
                : PoweredGuidancePhase.PoweredGuidance;
        }

        // A PVG-style steering law: chase the target orbit's angular momentum while damping radial motion.
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

            // FIXME: temporary blend/clamp; replace with solved thrust-vector direction from the guidance law.
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

            // FIXME: temporary floor to avoid near-zero denominator jitter in early guidance.
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

            // FIXME: temporary proportional gains for AP/PE shaping; replace with PVG terminal constraints.
            pitch += OrbitMath.Clamp(apError / apScale, -1.0, 1.0) * 18.0;

            if (peError > peTolerance)
            {
                pitch -= OrbitMath.Clamp(peError / peScale, 0.0, 1.0) * 5.0;
            }

            // FIXME: temporary descent guard to prevent digging into the atmosphere during missed insertions.
            if (vesselState.VerticalSpeed < -5.0 && vesselState.AltitudeMeters < ascentProfile.TargetPeriapsisAlt)
            {
                pitch += 6.0;
            }

            return OrbitMath.Clamp(pitch, -12.0, 28.0);
        }

        // Uses the target insertion altitude as a radial-speed schedule instead of coasting to apoapsis.
        private static double GetDesiredRadialSpeed(VesselState vesselState, AscentProfile ascentProfile)
        {
            double targetRadius = vesselState.BodyRadius +
                                  (ascentProfile.TargetApoapsisAlt + ascentProfile.TargetPeriapsisAlt) * 0.5;
            double currentRadius = (vesselState.Position - vesselState.Body.position).magnitude;
            double altitudeError = targetRadius - currentRadius;

            // FIXME: temporary radial-speed schedule; replace with time-to-go/radius solution.
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
            // FIXME: temporary fallback/offset until turn start is generated by the ascent solver.
            if (ascentProfile == null || ascentProfile.Points == null || ascentProfile.Points.Length == 0) return 1000.0;

            return Math.Max(1000.0, ascentProfile.Points[0].AltitudeMeters + 500.0);
        }

        private static double GetTerminalTolerance(double targetAltitude)
        {
            // FIXME: temporary percentage tolerance; derive from minimum controllable impulse and orbit noise.
            return OrbitMath.Clamp(Math.Abs(targetAltitude) * 0.025, MinimumTerminalToleranceMeters, MaximumTerminalToleranceMeters);
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
            return new PoweredGuidanceCommand
            {
                Phase = phase,
                Status = status,
                PitchDeg = pitchDeg,
                HeadingDeg = OrbitMath.NormalizeDegrees(headingDeg),
                Throttle = OrbitMath.Clamp(throttle, 0.0, 1.0),
                ApoapsisErrorMeters = apError,
                PeriapsisErrorMeters = peError,
                TimeToGoSeconds = timeToGo,
                VelocityToGoMetersPerSecond = velocityToGo,
                IsComplete = isComplete
            };
        }
    }
}
