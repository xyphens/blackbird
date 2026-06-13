using System;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Mathematics
{
    internal class OrbitMath
    {
        // Computes surface gravity from the body's gravitational parameter and radius.
        public static double GetSurfaceGravity(CelestialBody body)
        {
            if (body == null || body.Radius <= 0.0) return double.NaN;

            return body.gravParameter / (body.Radius * body.Radius);
        }

        // Computes circular orbital velocity at an altitude above the body's reference radius.
        public static double GetCircularVelocity(CelestialBody body, double altitudeMeters)
        {
            if (body == null) return double.NaN;

            double radius = body.Radius + altitudeMeters;
            if (radius <= 0.0) return double.NaN;

            return Math.Sqrt(body.gravParameter / radius);
        }

        // Computes semi-major axis from apoapsis/periapsis altitudes around a body.
        public static double GetSemiMajorAxis(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null) return double.NaN;

            double apoapsisRadius = body.Radius + apoapsisAlt;
            double periapsisRadius = body.Radius + periapsisAlt;

            if (apoapsisRadius <= 0.0 || periapsisRadius <= 0.0) return double.NaN;

            return (apoapsisRadius + periapsisRadius) * 0.5;
        }

        // Computes Keplerian orbital period from apoapsis/periapsis altitudes.
        public static double GetOrbitalPeriod(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null || body.gravParameter <= 0.0) return double.NaN;

            double semiMajorAxis = GetSemiMajorAxis(body, apoapsisAlt, periapsisAlt);
            if (!IsFinite(semiMajorAxis) || semiMajorAxis <= 0.0) return double.NaN;

            return 2.0 * Math.PI * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / body.gravParameter);
        }

        // Propagates an orbit to universal time and returns the world-space position.
        public static Vector3d GetOrbitPositionAtUt(Orbit orbit, double universalTime)
        {
            if (orbit == null || orbit.referenceBody == null) return Vector3d.zero;

            return orbit.referenceBody.position + orbit.getRelativePositionAtUT(universalTime);
        }

        // Converts a world-space position into altitude above the body's reference radius.
        public static double GetAltitudeAtPosition(CelestialBody body, Vector3d position)
        {
            if (body == null) return double.NaN;

            return (position - body.position).magnitude - body.Radius;
        }

        // Computes signed phase angle between active and target positions in an orbital plane.
        public static double GetPhaseAngleDeg(
            Vector3d activePosition,
            Vector3d targetPosition,
            Vector3d orbitNormal,
            Vector3d bodyPosition)
        {
            Vector3d activeVector = activePosition - bodyPosition;
            Vector3d targetVector = targetPosition - bodyPosition;

            if (activeVector.sqrMagnitude <= 0.0 || targetVector.sqrMagnitude <= 0.0)
            {
                return double.NaN;
            }

            double angle = Vector3d.Angle(activeVector, targetVector);
            Vector3d cross = Vector3d.Cross(activeVector, targetVector);
            double sign = Math.Sign(Vector3d.Dot(cross, orbitNormal));

            return NormalizeDegrees(angle * sign);
        }

        // Estimates the two-impulse Hohmann transfer dV between coplanar circular altitudes.
        public static double EstimateHohmannDeltaV(
            CelestialBody body,
            double fromCircularAltitude,
            double toCircularAltitude)
        {
            if (body == null || body.gravParameter <= 0.0) return double.NaN;

            double r1 = body.Radius + fromCircularAltitude;
            double r2 = body.Radius + toCircularAltitude;
            if (r1 <= 0.0 || r2 <= 0.0) return double.NaN;

            double mu = body.gravParameter;
            double transferSemiMajorAxis = (r1 + r2) * 0.5;

            double v1 = Math.Sqrt(mu / r1);
            double v2 = Math.Sqrt(mu / r2);
            double transferPeriapsisVelocity = Math.Sqrt(mu * (2.0 / r1 - 1.0 / transferSemiMajorAxis));
            double transferApoapsisVelocity = Math.Sqrt(mu * (2.0 / r2 - 1.0 / transferSemiMajorAxis));

            return Math.Abs(transferPeriapsisVelocity - v1) + Math.Abs(v2 - transferApoapsisVelocity);
        }

        public static double GetPhaseAngleDeg(Vessel active, Vessel target)
        {
            // position of our celestial
            Vector3d bodyPos = active.mainBody.position;

            return GetPhaseAngleDeg(
                TrajectoryProvider.GetPosition(active),
                TrajectoryProvider.GetPosition(target),
                TrajectoryProvider.GetOrbitNormal(target),
                bodyPos);
        }

        // find the Azimuth (plane) in degrees we want to launch into
        public static double GetLaunchAzimuth(double targetInclination, double activeVesselLatitude)
        {
            double incRad = targetInclination * Math.PI / 180.0;
            double latRad = activeVesselLatitude * Math.PI / 180.0;

            double cosLatitude = Math.Cos(latRad);

            if (Math.Abs(cosLatitude) < 1e-9) return double.NaN;

            double argument = Math.Cos(incRad) / cosLatitude;

            if (argument > 1.0 && argument < 1.000001)
            {
                argument = 1.0;
            }
            else if (argument < -1.0 && argument > -1.000001)
            {
                argument = -1.0;
            }

            if (argument > 1.0)
            {
                return 90.0;
            }

            if (argument < -1.0)
            {
                return 270.0;
            }

            double azimuthRad = Math.Asin(argument);

            return NormalizeDegrees(azimuthRad * 180.0 / Math.PI);
        }

        // Computes launch heading from the target orbit plane at the current launch position.
        public static double GetLaunchHeadingFromOrbitNormal(
            Vector3d surfaceUp,
            Vector3d surfaceNorth,
            Vector3d orbitNormal,
            bool ascending)
        {
            if (surfaceUp.sqrMagnitude <= 0.0 || surfaceNorth.sqrMagnitude <= 0.0 || orbitNormal.sqrMagnitude <= 0.0)
            {
                return double.NaN;
            }

            Vector3d up = surfaceUp.normalized;
            Vector3d north = Vector3d.Exclude(up, surfaceNorth).normalized;
            if (north.sqrMagnitude <= 0.0) return double.NaN;

            Vector3d east = Vector3d.Cross(up, north).normalized;
            Vector3d normal = orbitNormal.normalized;
            Vector3d direction = ascending
                ? Vector3d.Cross(up, normal)
                : Vector3d.Cross(normal, up);

            direction = Vector3d.Exclude(up, direction).normalized;
            if (direction.sqrMagnitude <= 0.0) return double.NaN;

            double northComponent = Vector3d.Dot(direction, north);
            double eastComponent = Vector3d.Dot(direction, east);
            double headingRad = Math.Atan2(eastComponent, northComponent);

            return NormalizeDegrees(headingRad * 180.0 / Math.PI);
        }

        // convert negative degrees to a real radian
        public static double NormalizeDegrees(double degrees)
        {
            degrees %= 360.0;
            if (degrees < 0) degrees += 360.0;

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
        public static double Clamp(double value, double min, double max)
        {
            return value < min ? min : value > max ? max : value;
        }
        public static double ClampPi(double value, double tau)
        {
            value %= tau;
            value = value < 0.0 ? value + tau : value;

            if (value >= tau) value = 0.0;

            return value > Math.PI ? value - tau : value;
        }

        public static double SafeAcos(double value)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                return double.NaN;

            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, value)));
        }

        public static Vector3d EulerAngles(QuaternionD q, double tau)
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
                return new Vector3d(90.0, Rad2Deg(Clamp2Pi(yaw, tau)), 0.0);
            }

            if (test < -0.499999999 * unit)
            {
                double yaw = -2.0 * Math.Atan2(q.y, q.w);
                return new Vector3d(270.0, Rad2Deg(Clamp2Pi(yaw, tau)), 0.0);
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
                Rad2Deg(Clamp2Pi(pitch, tau)),
                Rad2Deg(Clamp2Pi(yawNormal, tau)),
                Rad2Deg(Clamp2Pi(roll, tau)));
        }

        public static double Clamp2Pi(double value, double tau)
        {
            value %= tau;
            value = value < 0.0 ? value + tau : value;
            return value >= tau ? 0.0 : value;
        }
        public static double Rad2Deg(double value)
        {
            return value * 180.0 / Math.PI;
        }
        public static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
        public static double ApplyDeadband(double value, double deadband)
        {
            return Math.Abs(value) < deadband ? 0.0 : value - Math.Sign(value) * deadband;
        }
    }
}
