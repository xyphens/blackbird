using System;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;

namespace Blackbird.Guidance
{
    public sealed class AscentGuidance
    {
        private readonly PoweredAscentGuidance _poweredGuidance = new PoweredAscentGuidance();

        public void Reset()
        {
            _poweredGuidance.Reset();
        }

        // Produces current flight commands from the selected launch profile and guidance mode.
        public AscentGuidanceInfo GetGuidance(
            Vessel vessel,
            LaunchPlan plan,
            double manualPitchCommandDeg,
            double manualHeadingCommandDeg,
            GuidanceMode guidanceMode)
        {
            if (vessel == null || plan == null) return null;

            VesselState vesselState = VesselState.FromVessel(vessel);
            LaunchCandidate selectedCandidate = plan.SelectedCandidate;
            AscentProfile ascentProfile = selectedCandidate != null ? selectedCandidate.AscentProfile : plan.AscentProfile;

            double profilePitch = GetProfilePitchDeg(vesselState, ascentProfile);
            double profileHeading = GetProfileHeadingDeg(vessel, plan, vesselState, ascentProfile);
            double profileThrottle = GetProfileThrottle(vesselState, ascentProfile);
            PoweredGuidanceCommand poweredCommand = _poweredGuidance.GetCommand(
                vesselState,
                plan,
                ascentProfile,
                profilePitch,
                profileHeading,
                profileThrottle);
            string guidancePhase = poweredCommand != null ? poweredCommand.Status : "Unavailable";

            double currentHeading = GetCurrentHeadingDeg(vessel);
            double currentPitch = GetCurrentPitchDeg(vessel);

            double commandHeading;
            double commandPitch;
            double commandThrottle = vessel.ctrlState != null ? vessel.ctrlState.mainThrottle : 0.0;

            if (guidanceMode == GuidanceMode.Autopilot)
            {
                commandHeading = poweredCommand != null
                    ? poweredCommand.HeadingDeg
                    : OrbitMath.NormalizeDegrees(profileHeading);
                commandPitch = poweredCommand != null
                    ? ClampPitchForAutopilot(poweredCommand.PitchDeg)
                    : ClampPitchForAutopilot(profilePitch);
                commandThrottle = poweredCommand != null ? poweredCommand.Throttle : profileThrottle;
            }
            else if (guidanceMode == GuidanceMode.Guidance)
            {
                commandHeading = manualHeadingCommandDeg;
                commandPitch = manualPitchCommandDeg;
            }
            else
            {
                commandHeading = currentHeading;
                commandPitch = currentPitch;
            }

            double headingError = OrbitMath.DeltaDegrees(currentHeading, commandHeading);
            double pitchError = OrbitMath.DeltaDegrees(currentPitch, commandPitch);

            return new AscentGuidanceInfo
            {
                GuidanceMode = guidanceMode,
                GuidancePhase = guidancePhase,

                ProfilePitchDeg = profilePitch,
                ProfileHeadingDeg = profileHeading,
                ProfileThrottle = profileThrottle,

                CommandPitchDeg = commandPitch,
                CommandHeadingDeg = commandHeading,
                CommandThrottle = commandThrottle,
                HasInertialDirection = poweredCommand != null && poweredCommand.HasInertialDirection,
                InertialDirection = poweredCommand != null ? poweredCommand.InertialDirection : Vector3d.zero,

                CurrentPitchDeg = currentPitch,
                CurrentHeadingDeg = currentHeading,

                PitchErrorDeg = pitchError,
                HeadingErrorDeg = headingError,

                PitchInstruction = "Pitch towards " + commandPitch.ToString("F1") + "°",
                HeadingInstruction = "Head towards " + commandHeading.ToString("F1") + "°",

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

                PredictedApoapsisAlt = ascentProfile != null ? ascentProfile.PredictedApoapsisAlt : double.NaN,
                PredictedPeriapsisAlt = ascentProfile != null ? ascentProfile.PredictedPeriapsisAlt : double.NaN,

                EstimatedDeltaVUsed = selectedCandidate != null ? selectedCandidate.EstimatedDeltaVUsed : double.NaN,
                EstimatedRemainingDeltaV = selectedCandidate != null
                    ? selectedCandidate.EstimatedRemainingDeltaV
                    : vesselState.RemainingDeltaV,
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
            return OrbitMath.IsFinite(throttle) ? OrbitMath.Clamp(throttle, 0.0, 1.0) : 1.0;
        }

        // Reads the selected profile pitch, falling back to vertical hold if no profile is available.
        private static double GetProfilePitchDeg(VesselState vesselState, AscentProfile ascentProfile)
        {
            if (vesselState == null || ascentProfile == null) return 90.0;

            double pitch = ascentProfile.GetPitchAtAltitude(vesselState.AltitudeMeters);
            return OrbitMath.IsFinite(pitch) ? pitch : 90.0;
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
                if (OrbitMath.IsFinite(heading)) return heading;
            }

            return double.IsNaN(plan.LaunchAzimuthDeg)
                ? GetFallbackLaunchHeading(vessel, plan)
                : plan.LaunchAzimuthDeg;
        }

        // Keeps autopilot pitch commands within the range the attitude controller can sensibly track.
        private static double ClampPitchForAutopilot(double pitchDeg)
        {
            return Math.Max(-30.0, Math.Min(90.0, pitchDeg));
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

            return OrbitMath.NormalizeDegrees(headingRad * 180.0 / Math.PI);
        }
    }
}
