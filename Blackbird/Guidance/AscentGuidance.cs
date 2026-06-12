using System;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Enums;

namespace Blackbird.Guidance
{
    public sealed class AscentGuidance
    {
        // TODO: hard-coded for now, but will be user inputs eventually
        private static readonly PitchProfilePoint[] RssPitchProfile =
        {
            new PitchProfilePoint { AltitudeMeters = 0.0, PitchDegrees = 90.0 },
            new PitchProfilePoint { AltitudeMeters = 1000.0, PitchDegrees = 85.0 },
            new PitchProfilePoint { AltitudeMeters = 5000.0, PitchDegrees = 70.0 },
            new PitchProfilePoint { AltitudeMeters = 10000.0, PitchDegrees = 55.0 },
            new PitchProfilePoint { AltitudeMeters = 20000.0, PitchDegrees = 35.0 },
            new PitchProfilePoint { AltitudeMeters = 35000.0, PitchDegrees = 20.0 },
            new PitchProfilePoint { AltitudeMeters = 50000.0, PitchDegrees = 10.0 },
            new PitchProfilePoint { AltitudeMeters = 70000.0, PitchDegrees = 0.0 }
        };

         private static readonly PitchProfilePoint[] StockPitchProfile =
         {
            new PitchProfilePoint { AltitudeMeters = 0.0, PitchDegrees = 90.0 },
            new PitchProfilePoint { AltitudeMeters = 500.0, PitchDegrees = 85.0 },
            new PitchProfilePoint { AltitudeMeters = 1500.0, PitchDegrees = 75.0 },
            new PitchProfilePoint { AltitudeMeters = 5000.0, PitchDegrees = 55.0 },
            new PitchProfilePoint { AltitudeMeters = 10000.0, PitchDegrees = 35.0 },
            new PitchProfilePoint { AltitudeMeters = 20000.0, PitchDegrees = 15.0 },
            new PitchProfilePoint { AltitudeMeters = 35000.0, PitchDegrees = 5.0 },
            new PitchProfilePoint { AltitudeMeters = 45000.0, PitchDegrees = 0.0 }
        };
        public AscentGuidanceInfo GetGuidance(Vessel vessel, LaunchPlan plan, double manualPitchCommandDeg, double manualHeadingCommandDeg, GuidanceMode guidanceMode)
        {
            if (vessel == null || plan == null) return null;

            double targetAzimuth = double.IsNaN(plan.LaunchAzimuthDeg) ? GetFallbackLaunchHeading(vessel, plan) : plan.LaunchAzimuthDeg;
            double targetPitch = GetTargetPitchDeg(vessel.altitude, plan);

            double currentHeading = GetCurrentHeadingDeg(vessel);
            double currentPitch = GetCurrentPitchDeg(vessel);

            double commandHeading = 0.0;
            double commandPitch = 0.0;

            // heading guidance
            if (guidanceMode == GuidanceMode.Autopilot) {
                commandHeading = OrbitMath.NormalizeDegrees(targetAzimuth);
                commandPitch = Math.Max(-30.0, Math.Min(90.0, targetPitch));  // allows autopilot to dip below horizon for circ.
            } else if (guidanceMode == GuidanceMode.Guidance)
            {
                commandHeading = manualHeadingCommandDeg;
                commandPitch = manualPitchCommandDeg;
            } else
            {
                commandHeading = currentHeading;
                commandPitch = currentPitch;
            }

            double headingError = OrbitMath.DeltaDegrees(currentHeading, commandHeading);

            double targetLan = plan.TargetOrbit.LanDeg;
            double currentLan = vessel.orbit.LAN;

            double lanError = OrbitMath.DeltaDegrees(currentLan, targetLan);
            double pitchError = OrbitMath.DeltaDegrees(currentPitch, commandPitch);

            return new AscentGuidanceInfo
            {
                TargetAzimuthDeg = targetAzimuth,
                CurrentHeadingDeg = currentHeading,
                CommandHeadingDeg = commandHeading,
                HeadingErrorDeg = headingError,

                TargetLanDeg = targetLan,
                CurrentLanDeg = currentLan,
                LanErrorDeg = lanError,

                HeadingInstruction = "Head towards " + commandHeading.ToString("F1") + "°",

                TargetPitchDeg = targetPitch,
                CommandPitchDeg = commandPitch,
                CurrentPitchDeg = currentPitch,
                PitchErrorDeg = pitchError,

                GuidanceMode = guidanceMode,
                PitchInstruction = "Pitch towards " + commandPitch.ToString("F1") + "°",
            };
        }

        // altitude is in meters
        private static double GetTargetPitchDeg(double altitude, LaunchPlan plan) {
            
            PitchProfilePoint[] profile = plan.ScaleLabel == PlanetScale.PlanetScaleEnum.RSS ? RssPitchProfile : StockPitchProfile;
            return InterpolatePitchProfile(altitude, profile);
        }

        private static double InterpolatePitchProfile(
            double altitudeMeters,
            PitchProfilePoint[] profile)
        {
            if (altitudeMeters <= profile[0].AltitudeMeters)
            {
                return profile[0].PitchDegrees;
            }

            for (int i = 0; i < profile.Length - 1; i++)
            {
                PitchProfilePoint a = profile[i];
                PitchProfilePoint b = profile[i + 1];

                if (altitudeMeters >= a.AltitudeMeters &&
                    altitudeMeters <= b.AltitudeMeters)
                {
                    double t =
                        (altitudeMeters - a.AltitudeMeters) /
                        (b.AltitudeMeters - a.AltitudeMeters);

                    return a.PitchDegrees +
                        ((b.PitchDegrees - a.PitchDegrees) * t);
                }
            }

            return profile[profile.Length - 1].PitchDegrees;
        }
        private static double GetCurrentPitchDeg(Vessel vessel) { 
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            Vector3d forward = vessel.ReferenceTransform.up.normalized;
            double angleFromUp = Vector3d.Angle(forward, up);
            return 90.0 - angleFromUp;
        }

        private static double GetFallbackLaunchHeading(Vessel vessel, LaunchPlan plan) { 
            if (vessel == null || plan == null) return double.NaN;

            double inclination = plan.TargetOrbit.InclinationDeg;
            double latitude = vessel.latitude;
            double azimuth = OrbitMath.GetLaunchAzimuth(inclination, latitude);

            if (!double.IsNaN(azimuth)) return azimuth;

            double currentHeading = GetCurrentHeadingDeg(vessel);

            if (!double.IsNaN(currentHeading)) return currentHeading;

            return 90.0;
        }

        private static double GetCurrentHeadingDeg(Vessel vessel)
        {
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
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
