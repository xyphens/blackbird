using System;

namespace Blackbird.Mathematics
{
    public static class MathHelpers
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
        public static double Clamp01(double value) => Clamp(value, 0.0, 1.0);
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
        public static double MaxMagnitude(this Vector3d vector) => Math.Max(Math.Max(Math.Abs(vector.x), Math.Abs(vector.y)), Math.Abs(vector.z));
        public static double ApplyDeadband(double value, double deadband) => Math.Abs(value) < deadband ? 0.0 : value - Math.Sign(value) * deadband;
        public class MovingAverage
        {
            private readonly double[] _store;
            private readonly int _storeSize;
            private int _nextIndex;

            public double Value
            {
                get
                {
                    double tmp = 0;
                    for (int i = 0; i < _store.Length; i++)
                    {
                        tmp += _store[i];
                    }

                    return tmp / _storeSize;
                }
                set
                {
                    _store[_nextIndex] = value;
                    _nextIndex = (_nextIndex + 1) % _storeSize;
                }
            }

            public MovingAverage(int size = 10, double startingValue = 0)
            {
                _storeSize = size;
                _store = new double[size];
                Force(startingValue);
            }

            private void Force(double newValue)
            {
                for (int i = 0; i < _storeSize; i++)
                {
                    _store[i] = newValue;
                }
            }

            public static implicit operator double(MovingAverage v) => v.Value;
        }
    }
}
