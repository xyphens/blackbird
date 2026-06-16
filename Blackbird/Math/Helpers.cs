using System;

namespace Blackbird.Mathematics
{
    internal class MathHelpers
    {
        // convert negative degrees to a real radian
        public static double NormalizeDegrees(double degrees)
        {
            degrees %= 360.0;
            if (degrees < 0) degrees += 360.0;

            return degrees;
        }

        public static double DeltaDegrees(double fromDeg, double toDeg)
        {
            double delta = NormalizeDegrees(toDeg - fromDeg);
            return delta > 180.0 ? delta - 360.0 : delta;
        }

        public static double TimeToLongitudeSeconds(double currentLongitudeDeg, double targetLongitudeDeg, double rotationPeriodSeconds)
        {
            double deltaDeg = NormalizeDegrees(targetLongitudeDeg - currentLongitudeDeg);
            return deltaDeg / 360.0 * rotationPeriodSeconds;
        }
        public static double Clamp(double value, double min, double max) => value < min ? min : value > max ? max : value;
        public static double ClampPi(double value, double tau)
        {
            value %= tau;
            value = value < 0.0 ? value + tau : value;

            if (value >= tau) value = 0.0;

            return value > Math.PI ? value - tau : value;
        }

        public static double BoundAcos(double value)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                return double.NaN;

            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, value)));
        }
        public static double Clamp2Pi(double value, double tau)
        {
            value %= tau;
            value = value < 0.0 ? value + tau : value;
            return value >= tau ? 0.0 : value;
        }
        public static double Rad2Deg(double value) => value * 180.0 / Math.PI;
        public static double Deg2Rad(double value) => value * Math.PI / 180.0;
        public static bool IsFinite(Vector3d vec) => IsFinite(vec.x) && IsFinite(vec.y) && IsFinite(vec.z); // overloaded
        public static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value); // overloaded
        public static bool IsNonZeroFinite(Vector3d v) => v != Vector3d.zero && IsFinite(v);

        public static double ApplyDeadband(double value, double deadband) => Math.Abs(value) < deadband ? 0.0 : value - Math.Sign(value) * deadband;
    }
}
