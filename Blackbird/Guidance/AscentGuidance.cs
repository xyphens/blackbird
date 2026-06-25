using System;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;
using Blackbird.Modules;

namespace Blackbird.Guidance
{
    public sealed class AscentGuidance
    {
        private readonly PoweredAscentGuidance _poweredGuidance = new PoweredAscentGuidance();
        private readonly ClassicAscentGuidance2 _classicGuidance = new ClassicAscentGuidance2();

        private bool IsRSS = false;
        private bool IsPrincipia = false;
        private double _holdPitchUntilAlt = 0.0;
        private double _minVrfSpeedToPitch = 100.0;

        // important to call this before the class is used
        public void Refresh(bool isRss, bool isPrincipia, double holdUntilAltitude, double minVrfSp = 100.0)
        {
            _holdPitchUntilAlt = holdUntilAltitude;
            IsRSS = isRss;
            IsPrincipia = isPrincipia;
            _minVrfSpeedToPitch = minVrfSp;
        }
        public void Reset()
        {
            _poweredGuidance.Reset();
            _classicGuidance.Reset();
        }

        // Produces current flight commands from the selected launch profile and guidance mode.
        public AscentGuidanceInfo GetGuidance(
            Vessel vessel,
            LaunchPlan plan,
            double manualPitchCommandDeg,
            double manualHeadingCommandDeg,
            double manualThrottleCommand,
            double manualRollCommand,
            GuidanceMode guidanceMode)
        {
            if (vessel == null || plan == null) return null;

            VesselState vesselState = VesselState.FromVessel(vessel);
            LaunchCandidate selectedCandidate = plan.SelectedCandidate;
            AscentProfile ascentProfile = selectedCandidate != null ? selectedCandidate.AscentProfile : plan.AscentProfile;

            double profilePitch = GetProfilePitchDeg(vesselState, ascentProfile);
            double profileHeading = GetProfileHeadingDeg(vessel, plan, vesselState, ascentProfile);
            double profileThrottle = GetProfileThrottle(vesselState, ascentProfile);

            // fixme: stop burning rcs across the coast phase
            PoweredGuidanceCommand poweredCommand = !IsRSS
                                    ? _classicGuidance.GetCommand(vesselState, ascentProfile, profilePitch, profileHeading, plan.TargetOrbitNormal)
                                    : _poweredGuidance.GetCommand(vesselState, plan, ascentProfile, profilePitch, profileHeading, profileThrottle);

            // NaN when no target orbit yet (pre-launch / no target) — classic guidance skips the
            // inclination term for a non-finite inclination rather than dereferencing a null TargetOrbit.
            double targetInclinationDeg = plan.TargetOrbit != null ? plan.TargetOrbit.InclinationDeg : double.NaN;
            // fixme: update this logic to stop burning rcs across the coast phase and instead only do it at a set time before prograde burn

            //PoweredGuidanceCommand poweredCommand = !IsRSS
            //    ? _classicGuidance.GetCommand(vesselState, ascentProfile, profilePitch, profileHeading, targetInclinationDeg)
            //    : _poweredGuidance.GetCommand(vesselState, plan, ascentProfile, profilePitch, profileHeading, profileThrottle);
            string guidancePhase = poweredCommand != null ? poweredCommand.Status : "Unavailable";

            double currentHeading = GetCurrentHeadingDeg(vessel);
            double currentPitch = GetCurrentPitchDeg(vessel);
            double currentThrottle = vessel.ctrlState != null ? vessel.ctrlState.mainThrottle : 0.0;
            double currentRoll = vessel.ctrlState != null ? vessel.ctrlState.roll : 0.0; 

            double commandHeading;
            double commandPitch;
            double commandThrottle;
            double commandRoll;

            bool holdVSpeed = false;

            if (guidanceMode == GuidanceMode.Autopilot)
            {
                holdVSpeed = vessel.srfSpeed < _minVrfSpeedToPitch || vessel.altitude < _holdPitchUntilAlt;

                commandHeading = poweredCommand != null
                    ? poweredCommand.HeadingDeg
                    : MathHelpers.NormalizeDegrees(profileHeading);
                if (holdVSpeed)
                {
                    commandPitch = 90.0;
                } else
                {
                    // note: replaced ClampPitchForAutopilot/ClampPitchForControl with MathHelpers.Clamp
                    commandPitch = poweredCommand != null
                                ? MathHelpers.Clamp(poweredCommand.PitchDeg, -30.0, 90.0)
                                : MathHelpers.Clamp(profilePitch, -30.0, 90.0);
                }

                commandThrottle = poweredCommand != null ? poweredCommand.Throttle : profileThrottle;
                commandRoll = 0.0; // todo: i dont think autopilot would need roll?
            }
            else if (guidanceMode == GuidanceMode.Manual)
            {
                commandHeading = manualHeadingCommandDeg;
                commandPitch = manualPitchCommandDeg;
                commandThrottle = manualThrottleCommand;
                commandRoll = manualRollCommand;
            }
            else
            {
                commandHeading = currentHeading;
                commandPitch = currentPitch;
                commandThrottle = currentThrottle;
                commandRoll = currentRoll;
            }

            double headingError = MathHelpers.DeltaDegrees(currentHeading, commandHeading);
            double pitchError = MathHelpers.DeltaDegrees(currentPitch, commandPitch);

            return new AscentGuidanceInfo
            {
                GuidanceMode = guidanceMode,
                GuidancePhase = guidancePhase,
                IsGuidanceComplete = poweredCommand != null && poweredCommand.IsComplete,

                ProfilePitchDeg = profilePitch,
                ProfileHeadingDeg = profileHeading,
                ProfileThrottle = profileThrottle,

                CommandPitchDeg = commandPitch,
                CommandHeadingDeg = commandHeading,
                CommandThrottle = commandThrottle,
                CommandRoll = commandRoll,
                HasInertialDirection = !holdVSpeed && poweredCommand != null && poweredCommand.HasInertialDirection,
                InertialDirection = poweredCommand != null ? poweredCommand.InertialDirection : Vector3d.zero,

                CurrentPitchDeg = currentPitch,
                CurrentHeadingDeg = currentHeading,

                PitchErrorDeg = pitchError,
                HeadingErrorDeg = headingError,

                TargetApoapsisAlt = ascentProfile != null ? ascentProfile.TargetApoapsisAlt : plan.RecommendedApAlt,
                TargetPeriapsisAlt = ascentProfile != null ? ascentProfile.TargetPeriapsisAlt : plan.RecommendedPeAlt,
                ApoapsisErrorMeters = poweredCommand != null ? poweredCommand.ApoapsisErrorMeters : double.NaN,
                PeriapsisErrorMeters = poweredCommand != null ? poweredCommand.PeriapsisErrorMeters : double.NaN,
                GuidanceTimeToGoSeconds = poweredCommand != null ? poweredCommand.TimeToGoSeconds : double.NaN,
                GuidanceVelocityToGoMetersPerSecond = poweredCommand != null
                    ? poweredCommand.VelocityToGoMetersPerSecond
                    : double.NaN,
                GuidanceConstraintViolation = poweredCommand != null
                    ? poweredCommand.SolutionConstraintViolation
                    : double.NaN,
                GuidanceOptimizerIterations = poweredCommand != null
                    ? poweredCommand.OptimizerIterations
                    : 0,
                GuidanceOptimizerStatus = poweredCommand != null
                    ? poweredCommand.OptimizerStatus
                    : string.Empty,

                PredictedApoapsisAlt = IsPrincipia ? _poweredGuidance.PredictedApoapsisAlt
                             : (ascentProfile != null ? ascentProfile.PredictedApoapsisAlt : double.NaN),
                PredictedPeriapsisAlt = IsPrincipia ? _poweredGuidance.PredictedPeriapsisAlt
                              : (ascentProfile != null ? ascentProfile.PredictedPeriapsisAlt : double.NaN),

                EstimatedDeltaVUsed = selectedCandidate != null ? selectedCandidate.EstimatedDeltaVUsed : double.NaN,
                EstimatedRemainingDeltaV = selectedCandidate != null
                    ? selectedCandidate.EstimatedRemainingDeltaV
                    : vesselState.RemainingDeltaV,
                VesselRemainingDeltaV = vesselState != null ? vesselState.RemainingDeltaV : double.NaN,
                EstimatedInsertionTimeSeconds = selectedCandidate != null
                    ? selectedCandidate.EstimatedInsertionTimeSeconds
                    : double.NaN,
                EstimatedOrbitsToRendezvous = selectedCandidate != null
                    ? selectedCandidate.EstimatedOrbitsToRendezvous
                    : double.NaN,

                PlaneErrorDeg = selectedCandidate != null ? selectedCandidate.PlaneErrorDeg : double.NaN,
                PhaseErrorDeg = selectedCandidate != null ? selectedCandidate.PhaseErrorDeg : double.NaN,
                RelativeDistanceMeters = selectedCandidate != null ? selectedCandidate.RelativeDistanceMeters : double.NaN
            };
        }

        // Reads the selected profile throttle, falling back to full thrust before insertion.
        private static double GetProfileThrottle(VesselState vesselState, AscentProfile ascentProfile)
        {
            if (vesselState == null || ascentProfile == null) return 1.0;

            double throttle = ascentProfile.GetThrottleAtAltitude(vesselState.AltitudeMeters);
            return MathHelpers.IsFinite(throttle) ? MathHelpers.Clamp(throttle, 0.0, 1.0) : 1.0;
        }

        // Reads the selected profile pitch, falling back to vertical hold if no profile is available.
        private static double GetProfilePitchDeg(VesselState vesselState, AscentProfile ascentProfile)
        {
            if (vesselState == null || ascentProfile == null) return 90.0;

            double pitch = ascentProfile.GetPitchAtAltitude(vesselState.AltitudeMeters);
            return MathHelpers.IsFinite(pitch) ? pitch : 90.0;
        }

        // Reads the selected profile heading, falling back to launch azimuth/current heading if needed.
        private static double GetProfileHeadingDeg(
            Vessel vessel,
            LaunchPlan plan,
            VesselState vesselState,
            AscentProfile ascentProfile)
        {
            if (vesselState != null && ascentProfile != null)
            {
                double heading = ascentProfile.GetHeadingAtAltitude(vesselState.AltitudeMeters);
                if (MathHelpers.IsFinite(heading)) return heading;
            }

            return double.IsNaN(plan.LaunchAzimuthDeg)
                ? GetFallbackLaunchHeading(vessel, plan)
                : plan.LaunchAzimuthDeg;
        }

        // Computes vessel pitch relative to the local horizon.
        private static double GetCurrentPitchDeg(Vessel vessel)
        {
            Vector3d up = (TrajectoryProvider.GetPosition(vessel) - vessel.mainBody.position).normalized;
            Vector3d forward = vessel.ReferenceTransform.up.normalized;
            double angleFromUp = Vector3d.Angle(forward, up);
            return 90.0 - angleFromUp;
        }

        // Computes a usable launch heading when the selected plan does not provide one.
        private static double GetFallbackLaunchHeading(Vessel vessel, LaunchPlan plan)
        {
            if (vessel == null || plan == null || plan.TargetOrbit == null) return double.NaN;

            double azimuth = OrbitMath.GetLaunchAzimuth(plan.TargetOrbit.InclinationDeg, vessel.latitude);

            if (!double.IsNaN(azimuth)) return azimuth;

            double currentHeading = GetCurrentHeadingDeg(vessel);

            if (!double.IsNaN(currentHeading)) return currentHeading;

            return 90.0;
        }

        // Computes current vessel compass heading from local north/east axes.
        private static double GetCurrentHeadingDeg(Vessel vessel)
        {
            Vector3d up = (TrajectoryProvider.GetPosition(vessel) - vessel.mainBody.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vessel.mainBody.transform.up).normalized;
            Vector3d east = Vector3d.Cross(up, north);
            Vector3d forward = Vector3d.Exclude(up, vessel.ReferenceTransform.up).normalized;

            double northComponent = Vector3d.Dot(forward, north);
            double eastComponent = Vector3d.Dot(forward, east);

            double headingRad = Math.Atan2(eastComponent, northComponent);

            return MathHelpers.NormalizeDegrees(headingRad * 180.0 / Math.PI);
        }
    }
}
