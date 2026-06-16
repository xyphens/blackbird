using System;
using Blackbird.Trajectory;
using UnityEngine;
using static SpaceObjectCollider;

namespace Blackbird.Mathematics
{
    public struct M3
    {
        public double m00, m01, m02;
        public double m10, m11, m12;
        public double m20, m21, m22;

        public M3(double m00, double m01, double m02,
                  double m10, double m11, double m12,
                  double m20, double m21, double m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        // m * v: each result component is a row of m dotted with v.
        public static Vector3d operator *(M3 m, Vector3d v)
        {
            return new Vector3d(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z);
        }
    }

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

        // Orbital speed at apoapsis for the closed orbit defined by these apsis altitudes
        // (vis-viva). Reduces to circular velocity when apoapsisAlt == periapsisAlt. Returns
        // NaN for non-closed (parabolic/hyperbolic) inputs, i.e. a <= 0.
        public static double GetApoapsisVelocity(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null) return double.NaN;

            double apoapsisRadius = body.Radius + apoapsisAlt;
            double semiMajorAxis = GetSemiMajorAxis(body, apoapsisAlt, periapsisAlt);
            if (!IsFinite(semiMajorAxis) || semiMajorAxis <= 0.0 || apoapsisRadius <= 0.0)
                return double.NaN;

            double v2 = body.gravParameter * (2.0 / apoapsisRadius - 1.0 / semiMajorAxis);
            return v2 > 0.0 ? Math.Sqrt(v2) : double.NaN;
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

        public static double TimeToNextApoapsis(Orbit orbit, double ut)
        {
            return orbit.eccentricity < 1 ? orbit.TimeOfTrueAnomaly(Math.PI, ut) : 0;
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
                ? Vector3d.Cross(normal, up)
                : Vector3d.Cross(up, normal);

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
        public static double Deg2Rad(double value)
        {
            return value * Math.PI / 180.0;
        }
        public static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
        public static double ApplyDeadband(double value, double deadband)
        {
            return Math.Abs(value) < deadband ? 0.0 : value - Math.Sign(value) * deadband;
        }
        public static Vector3d DeltaVToCircularize(Orbit o, double ut)
        {
            (Vector3d pos, Vector3d vector) = RightHandVectorsAtUt(o, ut);
            return dvToCircularize(o.referenceBody.gravParameter, pos, vector);
        }

        private static Vector3d dvToCircularize(double mu, Vector3d r, Vector3d v)
        {
            if (mu > 0 && !double.IsNaN(mu) && !double.IsInfinity(mu) 
                && IsFinite(v) 
                && IsNonZeroFinite(r))
            {
                var h = Vector3d.Cross(r, v);
                return CircularVelocityFromHorizontalVector(mu, r, h) - v;
            }

            return new Vector3d();
        }

        private static Vector3d CircularVelocityFromHorizontalVector(double mu, Vector3d r, Vector3d h) => Vector3d.Cross(h, r).normalized * CircularVelocity(mu, r.magnitude);
        private static double CircularVelocity(double mu, double r) => Math.Sqrt(mu / r);

        public static Vector3d DeltaVToChangeInclination(Orbit o, double ut, double newInclination)
        {
            (Vector3d pos, Vector3d vector) = RightHandVectorsAtUt(o, ut);
            return dvToChangeInclination(pos, vector, newInclination);
        }
        private static Vector3d dvToChangeInclination(Vector3d position, Vector3d velocity, double targetInclination)
        {
            Vector3d dV = new Vector3d();
            // todo: check if pos is nonzero and finite, check if velocity and targetinclination are finite
            if (IsNonZeroFinite(position) && IsFinite(velocity) && IsFinite(targetInclination))
            {
                dV = VelocityForInclination(position, velocity, targetInclination) - velocity;
            }

            // returns an empty vector if not finite
            return IsFinite(dV) ? dV : new Vector3d();
        }

        // Body-relative position & orbital velocity at UT in the SwapYZ ("right-hand", z-up) frame —
        // MechJeb's SwappedRelativePositionAtUT / SwappedOrbitalVelocityAtUT exactly. This MUST use the
        // same .xzy convention as PerturbedOrbit/OrbitFromVectors below; the previous version used
        // Planetarium.Zup (a different z-up frame), and mixing the two rotated every dV by a fixed ~54°.
        private static (Vector3d pos, Vector3d vel) RightHandVectorsAtUt(Orbit o, double ut)
        {
            return (o.getRelativePositionAtUT(ut).xzy, o.getOrbitalVelocityAtUT(ut).xzy);
        }

        private static Vector3d VelocityForInclination(Vector3d position, Vector3d velocity, double targetInclination)
        {
            Vector3d v0 = BodyCenteredInertialToPlane(position, velocity);
            double horizonMag = new Vector3d(v0.x, v0.y).magnitude;
            Vector3d vf = PlaneHeadingForInclination(targetInclination, position) * horizonMag;
            vf.z = v0.z;
            vf = PlaneToBodyCenteredInertial(position, vf); // ENU -> ECI (inverse of the line above)
            return vf;
        }

        private static double AngleForInclination(double inclination, double latitude)
        {
            double cosAngle = Math.Cos(inclination) / Math.Cos(latitude);

            if (Math.Abs(cosAngle) > 1.0)
            {
                return Math.Abs(ClampPi(inclination, 2 * Math.PI)) < Math.PI * 0.5 ? 0 : Deg2Rad(180);
            }

            double angle = Math.Acos(cosAngle);

            if (inclination < 0) angle *= -1;

            return angle;
        }

        private static Vector3d PlaneHeadingForInclination(double inclination, Vector3d position)
        {
            double angle = AngleForInclination(inclination, LatFromBCI(position));
            return new Vector3d(Math.Cos(angle), Math.Sin(angle), 0);
        }
        // ECI -> east/north/up
        // claude review
        private static Vector3d BodyCenteredInertialToPlane(Vector3d pos, Vector3d vector)
        {
            double lat = LatFromBCI(pos);
            double lon = LonFromBCI(pos);

            double sinLat = Math.Sin(lat);
            double sinLon = Math.Sin(lon);
            double cosLat = Math.Cos(lat);
            double cosLon = Math.Cos(lon);

            var mtx = new M3(
                -sinLon, cosLon, 0.0,
                -sinLat * cosLon, -sinLat * sinLon, cosLat,
                cosLat * cosLon, cosLat * sinLon, sinLat
                );

            return mtx * vector;
        }

        public static Vector3d PlaneToBodyCenteredInertial(Vector3d pos, Vector3d vector)
        {
            double lat = LatFromBCI(pos);
            double lon = LonFromBCI(pos);

            double slat = Math.Sin(lat);
            double slng = Math.Sin(lon);
            double clat = Math.Cos(lat);
            double clng = Math.Cos(lon);

            var m = new M3(
                -slng, -slat * clng, clat * clng,
                clng, -slat * slng, clat * slng,
                0, clat, slat
            );

            return m * vector;
        }

        private static double LatFromBCI(Vector3d r) => !IsFinite(r) ? double.NaN : Math.Asin(Clamp(r.z / r.magnitude, -1.0, 1.0));
        private static double LonFromBCI(Vector3d r) => Math.Atan2(r.y, r.x);

        public static bool IsFinite(Vector3d v) => IsFinite(v[0]) && IsFinite(v[1]) && IsFinite(v[2]);
        public static bool IsNonZeroFinite(Vector3d v) => v != Vector3d.zero && IsFinite(v);
        public static Orbit PerturbedOrbit(Orbit o, double ut, Vector3d dV) => OrbitFromVectors(WorldPositionAtUt(o, ut), o.getOrbitalVelocityAtUT(ut).xzy + dV, o.referenceBody, ut);

        private static Vector3d WorldPositionAtUt(Orbit o, double ut) => o.referenceBody.position + o.getRelativePositionAtUT(ut).xzy;

        public static Orbit OrbitFromVectors(Vector3d position, Vector3d velocity, CelestialBody body, double ut)
        {
            var result = new Orbit();
            result.UpdateFromStateVectors((position - body.position).xzy, velocity.xzy, body, ut);
            if (double.IsNaN(result.argumentOfPeriapsis))
            {
                Vector3d vecToAscendingNode = Quaternion.AngleAxis(-(float)result.LAN, Planetarium.up) * Planetarium.right;
                Vector3d vectorToPeriapsis = result.eccVec.xzy;
                double cosArgOfPeriapsis = Vector3d.Dot(vecToAscendingNode, vectorToPeriapsis) / (vecToAscendingNode.magnitude * vectorToPeriapsis.magnitude);
                result.argumentOfPeriapsis = cosArgOfPeriapsis > 1 
                                            ? 0 
                                            : cosArgOfPeriapsis < -1 
                                                ? 180 
                                                : Math.Acos(cosArgOfPeriapsis);
            }

            return result;
        }
    }
}
