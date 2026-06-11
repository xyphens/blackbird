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

        public AscentGuidanceInfo GetGuidance(Vessel vessel, LaunchPlan plan)
        {
            if (vessel == null || plan == null) return null;

            double targetAzimuth = plan.LaunchAzimuthDeg;
            double targetLan = plan.TargetOrbit.LanDeg;

            double currentLan = vessel.orbit.LAN;

            double lanError = OrbitMath.DeltaDegrees(currentLan, targetLan);

            double targetPitch = GetTargetPitchDeg(vessel.altitude, plan);
            double currentPitch = GetCurrentPitchDeg(vessel);
            double pitchError = targetPitch - currentPitch;

            return new AscentGuidanceInfo
            {
                TargetAzimuthDeg = targetAzimuth,
                TargetLanDeg = targetLan,
                CurrentLanDeg = currentLan,
                LanErrorDeg = lanError,
                HeadingInstruction = GetHeadingInstruction(targetAzimuth),
                PitchInstruction = "Pitch to " + targetPitch.ToString("F1") + "°",
                TargetPitchDeg = targetPitch,
                CurrentPitchDeg = currentPitch,
                PitchErrorDeg = pitchError,
            };
        }
        private static string GetHeadingInstruction(double targetAzimuthDeg)
        {
            if (double.IsNaN(targetAzimuthDeg)) return "Heading unavailable";
            return "Fly heading: " + targetAzimuthDeg.ToString("F1") + "°";
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
    }
}
