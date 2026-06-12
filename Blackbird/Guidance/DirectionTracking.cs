using System;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class DirectionTracking
    {
        private const double Tau = 2.0 * Math.PI;
        private const double DegToRad = Math.PI / 180.0;

        private QuaternionD _previousRotation;
        public Vector3d TrackedRotation;

        public DirectionTracking()
        {
            Reset();
        }

        public Vector3d Update(QuaternionD currentRotation)
        {
            if (NeedsInitialization())
            {
                _previousRotation = currentRotation;
                TrackedRotation = Vector3d.zero;
                return TrackedRotation;
            }

            QuaternionD deltaRotation =
                QuaternionD.Inverse(_previousRotation) * currentRotation;

            Vector3d deltaEuler = EulerAngles(deltaRotation);

            double deltaPitch = ClampPi(deltaEuler.x * DegToRad);
            double deltaYaw = -ClampPi(deltaEuler.y * DegToRad);
            double deltaRoll = ClampPi(deltaEuler.z * DegToRad);

            TrackedRotation += new Vector3d(deltaPitch, deltaRoll, deltaYaw);

            _previousRotation = currentRotation;

            return TrackedRotation;
        }

        public void Desired(
            QuaternionD desiredRotation,
            out Vector3d desired,
            out Vector3d error,
            out double distance)
        {
            QuaternionD deltaRotation =
                QuaternionD.Inverse(_previousRotation) * desiredRotation;

            Vector3d deltaEuler = EulerAngles(deltaRotation);

            double deltaPitch = ClampPi(deltaEuler.x * DegToRad);
            double deltaYaw = -ClampPi(deltaEuler.y * DegToRad);
            double deltaRoll = ClampPi(deltaEuler.z * DegToRad);

            error = new Vector3d(deltaPitch, deltaRoll, deltaYaw);
            desired = TrackedRotation + error;

            distance = SafeAcos(Math.Cos(deltaPitch) * Math.Cos(deltaYaw));
        }

        public void Reset()
        {
            _previousRotation = new QuaternionD(0.0, 0.0, 0.0, 0.0);
            TrackedRotation = Vector3d.zero;
        }

        public void Reset(int index)
        {
            TrackedRotation[index] = 0.0;
        }

        private bool NeedsInitialization()
        {
            return _previousRotation == new QuaternionD(0.0, 0.0, 0.0, 0.0);
        }

        private static double ClampPi(double value)
        {
            value %= Tau;
            value = value < 0.0 ? value + Tau : value;

            if (value >= Tau) value = 0.0;

            return value > Math.PI ? value - Tau : value;
        }

        private static double SafeAcos(double value)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                return double.NaN;

            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, value)));
        }

        private static Vector3d EulerAngles(QuaternionD q)
        {
            double magnitude = Math.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

            if (magnitude < 2.2204460492503131e-16) return Vector3d.zero;

            if (Math.Abs(magnitude - 1.0) > 1e-10)
            {
                q.x /= magnitude;
                q.y /= magnitude;
                q.z /= magnitude;
                q.w /= magnitude;
            }

            double sqw = q.w * q.w;
            double sqx = q.x * q.x;
            double sqy = q.y * q.y;
            double sqz = q.z * q.z;

            double unit = sqx + sqy + sqz + sqw;
            double test = q.x * q.w - q.y * q.z;

            if (test > 0.499999999 * unit)
            {
                double yaw = 2.0 * Math.Atan2(q.y, q.w);
                return new Vector3d(90.0, Rad2Deg(Clamp2Pi(yaw)), 0.0);
            }

            if (test < -0.499999999 * unit)
            {
                double yaw = -2.0 * Math.Atan2(q.y, q.w);
                return new Vector3d(270.0, Rad2Deg(Clamp2Pi(yaw)), 0.0);
            }

            double pitch = Math.Asin(2.0 * test / unit);
            double yawNormal =
                Math.Atan2(
                    2.0 * (q.x * q.z + q.w * q.y),
                    sqw - sqx - sqy + sqz);

            double roll =
                Math.Atan2(
                    2.0 * (q.x * q.y + q.w * q.z),
                    sqw - sqx + sqy - sqz);

            return new Vector3d(
                Rad2Deg(Clamp2Pi(pitch)),
                Rad2Deg(Clamp2Pi(yawNormal)),
                Rad2Deg(Clamp2Pi(roll)));
        }

        private static double Clamp2Pi(double value)
        {
            value %= Tau;
            value = value < 0.0 ? value + Tau : value;
            return value >= Tau ? 0.0 : value;
        }

        private static double Rad2Deg(double value)
        {
            return value * 180.0 / Math.PI;
        }
    }
}