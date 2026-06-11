using System;

namespace Blackbird.Mathematics
{
    internal class OrbitMath
    {
        public static double GetPhaseAngleDeg(Vessel active, Vessel target)
        {
            // position of our celestial
            Vector3d bodyPos = active.mainBody.position;

            Vector3d activeVector = active.GetWorldPos3D() - bodyPos;
            Vector3d targetVector = target.GetWorldPos3D() - bodyPos;

            double angle = Vector3d.Angle(activeVector, targetVector);

            Vector3d cross = Vector3d.Cross(activeVector, targetVector);
            double sign = Math.Sign(Vector3d.Dot(cross, target.orbit.GetOrbitNormal()));

            return NormalizeDegrees(angle * sign);
        }

        // find the Azimuth (plane) in degrees we want to launch into
        public static double GetLaunchAzimuth(double activeVesselLatitude, double targetInclination)
        {
            
            double incRadians = targetInclination * Math.PI / 180.0;
            double latRadians = activeVesselLatitude * Math.PI / 180.0;

            if (Math.Abs(incRadians) < Math.Abs(latRadians))
            {
                return double.NaN;
            }

            double cosAzimuth = Math.Cos(incRadians) / Math.Cos(latRadians);
            cosAzimuth = Math.Max(-1.0, Math.Min(1.0, cosAzimuth));

            double azRadians = Math.Asin(cosAzimuth);
            return azRadians * 180.0 / Math.PI;
        }

        // convert negative degrees to a real radian
        public static double NormalizeDegrees(double degrees)
        {
            degrees %= 360.0;
            if (degrees < 0)
            {
                degrees += 360.0;
            }

            return degrees;
        }

        public static double DeltaDegrees(double fromDeg, double toDeg) { 
            double delta = NormalizeDegrees(toDeg -  fromDeg);
            return delta > 180.0 ? delta - 360.0 : delta;
        }

        public static double TimeToLongitudeSeconds(double currentLongitudeDeg, double targetLongitudeDeg, double rotationPeriodSeconds)
        {
            double deltaDeg = NormalizeDegrees(targetLongitudeDeg - currentLongitudeDeg);
            return deltaDeg / 360.0 * rotationPeriodSeconds;
        }
        public static double GetBodyFixedLongitudeAtTime(
            double inertialLongitudeDeg,
            double universalTimeSeconds,
            double rotationPeriodSeconds)
        {
            double rotationDeg = universalTimeSeconds / rotationPeriodSeconds * 360.0;
            return NormalizeDegrees(inertialLongitudeDeg - rotationDeg);
        }
    }
}
