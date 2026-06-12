using System;
using UnityEngine;
using Blackbird.Mathematics;
/**
 * Converts quaternion attitude into the internal axis order
 **/
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

        /**
         * Tracks the vessel's current accumulated pitch/roll/yaw rotation
         **/
        public Vector3d Update(QuaternionD currentRotation)
        {
            if (NeedsInitialization())
            {
                _previousRotation = currentRotation;
                TrackedRotation = Vector3d.zero;
                return TrackedRotation;
            }

            QuaternionD deltaRotation = QuaternionD.Inverse(_previousRotation) * currentRotation;

            Vector3d deltaEuler = OrbitMath.EulerAngles(deltaRotation, Tau);

            double deltaPitch = OrbitMath.ClampPi(deltaEuler.x * DegToRad, Tau);
            double deltaYaw = -OrbitMath.ClampPi(deltaEuler.y * DegToRad, Tau);
            double deltaRoll = OrbitMath.ClampPi(deltaEuler.z * DegToRad, Tau);

            TrackedRotation += new Vector3d(deltaPitch, deltaRoll, deltaYaw);

            _previousRotation = currentRotation;

            return TrackedRotation;
        }

        /**
         * Compares current attitude to the request attitude and yields:
         * - desired rotation state (double)
         * - pitch/roll/yaw error in radians (double)
         * - distance (non-roll pointing error)
         **/
        public void Desired(QuaternionD desiredRotation, out Vector3d desired, out Vector3d error, out double distance)
        {
            QuaternionD deltaRotation = QuaternionD.Inverse(_previousRotation) * desiredRotation;

            Vector3d deltaEuler = OrbitMath.EulerAngles(deltaRotation, Tau);

            double deltaPitch = OrbitMath.ClampPi(deltaEuler.x * DegToRad, Tau);
            double deltaYaw = -OrbitMath.ClampPi(deltaEuler.y * DegToRad, Tau);
            double deltaRoll = OrbitMath.ClampPi(deltaEuler.z * DegToRad, Tau);

            error = new Vector3d(deltaPitch, deltaRoll, deltaYaw);
            desired = TrackedRotation + error;

            distance = OrbitMath.SafeAcos(Math.Cos(deltaPitch) * Math.Cos(deltaYaw));
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
    }
}