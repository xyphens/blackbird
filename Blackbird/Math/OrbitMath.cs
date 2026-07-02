using Blackbird.Models;
using Blackbird.Trajectory;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static SpaceObjectCollider;
using static Vect;

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

    // mass length velocity
    public readonly struct  MLV 
    {
        public readonly double M;
        public readonly double L;
        public readonly double V;

        public MLV(double m, double l, double v)
        {
            M = m;
            L = l; 
            V = v; 
        }

        public double TimeScale => L / V;
        public double Accel => V / TimeScale;
        public double Force => M * Accel;
        public double MDot => M / TimeScale;
        public double Area => L * L;
        public double Volume => Area * L;
        public double Density => M / Volume;
        public double Pressure => Force / Area;

        public static MLV Init(double mu, double r0, double m0 = 1.0)
        {
            double mS = m0;
            double lS = r0;
            double vS = Math.Sqrt(mu / lS);
            return new MLV(mS, lS, vS);
        }

        public MLV Convert(MLV other)
        {
            return new MLV(
                other.M / M,
                other.L / L,
                other.V / V
               );
        }
    }

    internal class OrbitMath
    {
        // Diagnostic log for the Hohmann optimizer (HOHMANN-OPT lines in rendezvous.log): per-window
        // dt1/tt/dv1/dv2 + the chosen transfer, so a bad plan can be traced to window selection vs the solve.
        private static readonly Blackbird.Logging.BlackbirdLog HohmannLog =
            new Blackbird.Logging.BlackbirdLog(Blackbird.Logging.LogContext.Rendezvous);

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

        // todo: not used
        public static double GetApoapsisVelocity(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null) return double.NaN;

            double apoapsisRadius = body.Radius + apoapsisAlt;
            double semiMajorAxis = GetSemiMajorAxis(body, apoapsisAlt, periapsisAlt);
            if (!MathHelpers.IsFinite(semiMajorAxis) || semiMajorAxis <= 0.0 || apoapsisRadius <= 0.0)
                return double.NaN;

            double v2 = body.gravParameter * (2.0 / apoapsisRadius - 1.0 / semiMajorAxis);
            return v2 > 0.0 ? Math.Sqrt(v2) : double.NaN;
        }

        
        public static double GetSemiMajorAxis(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null) return double.NaN;

            double apoapsisRadius = body.Radius + apoapsisAlt;
            double periapsisRadius = body.Radius + periapsisAlt;

            if (apoapsisRadius <= 0.0 || periapsisRadius <= 0.0) return double.NaN;

            return (apoapsisRadius + periapsisRadius) * 0.5;
        }

        // Computes Keplerian orbital period from apoapsis/periapsis altitudes.
        // todo: not used
        public static double GetOrbitalPeriod(CelestialBody body, double apoapsisAlt, double periapsisAlt)
        {
            if (body == null || body.gravParameter <= 0.0) return double.NaN;

            double semiMajorAxis = GetSemiMajorAxis(body, apoapsisAlt, periapsisAlt);
            if (!MathHelpers.IsFinite(semiMajorAxis) || semiMajorAxis <= 0.0) return double.NaN;

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

            return MathHelpers.NormalizeDegrees(angle * sign);
        }

        public static double GetRelativeInclination(Vessel vessel, Vessel target) // fixme: decide if GetOrbitNormal is better
        {
            return Math.Abs(Vector3d.Angle(TrajectoryProvider.GetKspOrbitNormal(vessel), TrajectoryProvider.GetKspOrbitNormal(target)));
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
            double incRad = MathHelpers.Deg2Rad(targetInclination);
            double latRad = MathHelpers.Deg2Rad(activeVesselLatitude);

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

            return MathHelpers.NormalizeDegrees(azimuthRad * 180.0 / Math.PI);
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

            return MathHelpers.NormalizeDegrees(headingRad * 180.0 / Math.PI);
        }

        public static double GetBodyFixedLongitudeAtTime(
            double inertialLongitudeDeg,
            double universalTimeSeconds,
            double rotationPeriodSeconds)
        {
            double rotationDeg = universalTimeSeconds / rotationPeriodSeconds * 360.0;
            return MathHelpers.NormalizeDegrees(inertialLongitudeDeg - rotationDeg);
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
                return new Vector3d(90.0, MathHelpers.Rad2Deg(MathHelpers.Clamp2Pi(yaw, tau)), 0.0);
            }

            if (test < -0.499999999 * unit)
            {
                double yaw = -2.0 * Math.Atan2(q.y, q.w);
                return new Vector3d(270.0, MathHelpers.Rad2Deg(MathHelpers.Clamp2Pi(yaw, tau)), 0.0);
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
                MathHelpers.Rad2Deg(MathHelpers.Clamp2Pi(pitch, tau)),
                MathHelpers.Rad2Deg(MathHelpers.Clamp2Pi(yawNormal, tau)),
                MathHelpers.Rad2Deg(MathHelpers.Clamp2Pi(roll, tau)));
        }

        public static Vector3d DeltaVToCircularize(Orbit o, double ut)
        {
            (Vector3d pos, Vector3d vector) = RightHandVectorsAtUt(o, ut);
            return dvToCircularize(o.referenceBody.gravParameter, pos, vector).xzy;
        }

        private static Vector3d dvToCircularize(double mu, Vector3d r, Vector3d v)
        {
            if (mu > 0 && !double.IsNaN(mu) && !double.IsInfinity(mu) 
                && MathHelpers.IsFinite(v) 
                && MathHelpers.IsNonZeroFinite(r))
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
            if (MathHelpers.IsNonZeroFinite(position) && MathHelpers.IsFinite(velocity) && MathHelpers.IsFinite(targetInclination))
            {
                dV = VelocityForInclination(position, velocity, targetInclination) - velocity;
            }

            // returns an empty vector if not finite
            return MathHelpers.IsFinite(dV) ? dV : new Vector3d();
        }

        // Body-relative position & orbital velocity at UT in the SwapYZ ("right-hand", z-up) frame —
        // MechJeb's SwappedRelativePositionAtUT / SwappedOrbitalVelocityAtUT exactly. This MUST use the
        // same .xzy convention as PerturbedOrbit/OrbitFromVectors below; the previous version used
        // Planetarium.Zup (a different z-up frame), and mixing the two rotated every dV by a fixed ~54°.
        private static (Vector3d pos, Vector3d vel) RightHandVectorsAtUt(Orbit o, double ut)
        {
            o.GetOrbitalStateVectorsAtTrueAnomaly(o.TrueAnomalyAtT(o.getObtAtUT(ut)), ut, false, out Vector3d pos, out Vector3d vel);
            return (Planetarium.Zup.WorldToLocal(pos), Planetarium.Zup.WorldToLocal(vel));
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
                return Math.Abs(MathHelpers.ClampPi(inclination, 2 * Math.PI)) < Math.PI * 0.5 ? 0 : MathHelpers.Deg2Rad(180);
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

        public static (double ut, double distance) NextClosestApproach(Orbit vessel, Orbit target)
        {
            double ut = Planetarium.GetUniversalTime();

            double caTime = ut;
            double caDistance = double.MaxValue;
            
            // track over parabolic orbit or use the vessel's current orbital period if its circular
            double arcInterval = vessel.eccentricity > 1 ? 100 / vessel.meanMotion : vessel.period;
            double _minTime = ut;
            double _maxTime = ut + arcInterval;

            int divisions = 20; // break orbit into slices for search
            for (int i = 0; i < 8; i++)
            {
                double dt = (_maxTime - _minTime) / divisions;
                for (int j = 0; j < divisions; j++)
                {
                    double t = _minTime + j * dt;
                    double distance = GetSeparation(vessel, target, t);
                    if (distance < caDistance)
                    {
                        caDistance = distance;
                        caTime = t;
                    }
                }

                _minTime = MathHelpers.Clamp(caTime - dt, ut, ut + arcInterval);
                _maxTime = MathHelpers.Clamp(caTime + dt, ut, ut + arcInterval);
            }

            return (caTime, caDistance);
        }

        // A Hohmann transfer is two impulses: dv1 at ut1 injects onto the transfer ellipse; dv2 at ut2
        // (= ut1 + transfer time) circularizes/matches at the target. With Capture = true the optimizer
        // minimizes |dv1| + |dv2| (full rendezvous); dv2 is what our Match Velocity stage executes.
        //
        // Vessel overload: extracts state via RightHandVectorsAtUt, so the returned ΔV is in THAT frame --
        // convert to world before applying. To stay in your own frame, use the state-vector overload below.
        public static (Vector3d dv1, double ut1, Vector3d dv2, double ut2) DeltaVForHohmannTransfer(Vessel vessel, Vessel target, bool coplanar = false)
        {
            double ut = Planetarium.GetUniversalTime();
            (Vector3d r1, Vector3d v1) = RightHandVectorsAtUt(vessel.orbit, ut);
            (Vector3d r2, Vector3d v2) = RightHandVectorsAtUt(target.orbit, ut);
            double mu = vessel.orbit.referenceBody.gravParameter;
            return DeltaVForHohmannTransfer(ut, r1, v1, r2, v2, mu, coplanar);
        }

        // State-vector core (frame-agnostic): _r1/_v1 = chaser body-relative position + inertial velocity,
        // _r2/_v2 = target, all in ONE consistent inertial frame. Returns (dv1, ut1, dv2, ut2) in that SAME
        // frame, with ABSOLUTE UTs. Feed KSP-world vectors and the ΔV comes back KSP-world (no .xzy needed).
        public static (Vector3d dv1, double ut1, Vector3d dv2, double ut2) DeltaVForHohmannTransfer(
            double ut, Vector3d _r1, Vector3d _v1, Vector3d _r2, Vector3d _v2, double mu, bool coplanar = false)
        {
            List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2, double total)> windows =
                CollectHohmannWindows(ut, _r1, _v1, _r2, _v2, mu, coplanar);

            if (windows.Count > 0)
            {
                // Neither the first feasible window (depart-now = high ΔV at a bad phase) nor the strict global-ΔV
                // minimum (chases marginal savings many hours out): the EARLIEST window within WINDOW_TOL of the
                // global best. The comparison is scale-invariant, so real-unit totals are fine.
                const double WINDOW_TOL = 0.20;
                double globalBest = double.PositiveInfinity;
                for (int i = 0; i < windows.Count; i++)
                    if (windows[i].total < globalBest) globalBest = windows[i].total;

                double threshold = globalBest * (1.0 + WINDOW_TOL);
                int pick = 0;
                double pickUt1 = double.PositiveInfinity;
                for (int i = 0; i < windows.Count; i++)
                {
                    if (windows[i].total <= threshold && windows[i].ut1 < pickUt1)
                    {
                        pick = i;
                        pickUt1 = windows[i].ut1;
                    }
                }

                var w = windows[pick];
                HohmannLog.Write("HOHMANN-OPT", "CHOSEN",
                    "dt1=" + (w.ut1 - ut).ToString("F0") + "s",
                    "dv1=" + w.dv1.magnitude.ToString("F2"),
                    "dv2=" + w.dv2.magnitude.ToString("F2"),
                    "total=" + w.total.ToString("F2"),
                    "globalBest=" + globalBest.ToString("F2"));
                return (w.dv1, w.ut1, w.dv2, w.ut2);
            }

            // No future-departure solution within the search horizon -- signal "no plan" (ut1 = -inf).
            HohmannLog.Write("HOHMANN-OPT", "NO FEASIBLE FUTURE-DEPARTURE SOLUTION");
            return (Vector3d.zero, double.NegativeInfinity, Vector3d.zero, double.NegativeInfinity);
        }

        // Up to maxCount eligible Hohmann transfer windows for the user to pick from. Near-duplicate windows
        // from the multi-start (same basin, different start) are merged keeping the cheaper; the maxCount
        // lowest-ΔV distinct windows are returned sorted by departure time (soonest first).
        public static List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2)> DeltaVForHohmannTransferCandidates(
            double ut, Vector3d _r1, Vector3d _v1, Vector3d _r2, Vector3d _v2, double mu, int maxCount, bool coplanar = false)
        {
            var result = new List<(Vector3d, double, Vector3d, double)>();
            if (maxCount <= 0) return result;

            List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2, double total)> windows =
                CollectHohmannWindows(ut, _r1, _v1, _r2, _v2, mu, coplanar);
            if (windows.Count == 0) return result;

            // Merge windows whose departures fall within ~2% of the search period (same optimizer basin).
            double synodic = SynodicPeriod(OrbitalPeriod(_r1, _v1, mu), OrbitalPeriod(_r2, _v2, mu));
            double searchPeriod = MathHelpers.IsFinite(synodic)
                ? synodic
                : Math.Max(OrbitalPeriod(_r1, _v1, mu), OrbitalPeriod(_r2, _v2, mu));
            double mergeTol = MathHelpers.IsFinite(searchPeriod) ? Math.Max(60.0, searchPeriod * 0.02) : 60.0;

            var deduped = new List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2, double total)>();
            for (int i = 0; i < windows.Count; i++)
            {
                var w = windows[i];
                int existing = -1;
                for (int j = 0; j < deduped.Count; j++)
                    if (Math.Abs(deduped[j].ut1 - w.ut1) < mergeTol) { existing = j; break; }

                if (existing < 0) deduped.Add(w);
                else if (w.total < deduped[existing].total) deduped[existing] = w;
            }

            // Keep the maxCount cheapest distinct windows, present soonest-first.
            deduped.Sort((a, b) => a.total.CompareTo(b.total));
            int take = Math.Min(maxCount, deduped.Count);
            List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2, double total)> chosen = deduped.GetRange(0, take);
            chosen.Sort((a, b) => a.ut1.CompareTo(b.ut1));

            for (int i = 0; i < chosen.Count; i++)
                result.Add((chosen[i].dv1, chosen[i].ut1, chosen[i].dv2, chosen[i].ut2));

            return result;
        }

        public static List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2)> DeltaVForHohmannTransferCandidates(
            Vessel vessel, Vessel target, int maxCount, bool coplanar = false)
        {
            double ut = Planetarium.GetUniversalTime();
            (Vector3d r1, Vector3d v1) = RightHandVectorsAtUt(vessel.orbit, ut);
            (Vector3d r2, Vector3d v2) = RightHandVectorsAtUt(target.orbit, ut);
            double mu = vessel.orbit.referenceBody.gravParameter;
            return DeltaVForHohmannTransferCandidates(ut, r1, v1, r2, v2, mu, maxCount, coplanar);
        }


        private static List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2, double total)> CollectHohmannWindows(
            double ut, Vector3d _r1, Vector3d _v1, Vector3d _r2, Vector3d _v2, double mu, bool coplanar)
        {
            var windows = new List<(Vector3d, double, Vector3d, double, double)>();

            // Canonical-units scale (mu -> 1) for optimizer conditioning; characteristic length = geometric
            // mean of the two radii.
            MLV scale = MLV.Init(mu, Math.Sqrt(_r1.magnitude * _r2.magnitude));

            // Real synodic period (seconds): the recurrence window for the relative geometry; drives the march.
            double synodicPeriodReal = SynodicPeriod(OrbitalPeriod(_r1, _v1, mu), OrbitalPeriod(_r2, _v2, mu));

            // Optional coplanar projection: rotate the target into the chaser's plane (2D transfer). OFF by
            // default (MJ's rendezvous path lets Lambert handle 3D). QuaternionD is a Unity-native call that
            // crashes offline, so the harness path must keep coplanar = false.
            Vector3d tr2 = _r2, tv2 = _v2;
            if (coplanar)
            {
                Vector3d hhat1 = Vector3d.Cross(_r1, _v1).normalized;
                Vector3d hhat2 = Vector3d.Cross(_r2, _v2).normalized;
                QuaternionD coPlanar = QuaternionD.FromToRotation(hhat2, hhat1);
                tr2 = coPlanar * _r2;
                tv2 = coPlanar * _v2;
            }

            Vector3d r1 = _r1 / scale.L;
            Vector3d v1 = _v1 / scale.V;
            Vector3d r2 = tr2 / scale.L;
            Vector3d v2 = tv2 / scale.V;

            var optArgs = new AlgLibArgs { R1 = r1, V1 = v1, R2 = r2, V2 = v2, Mu = 1.0, Capture = true };

            // Analytic Hohmann transfer-time guess (canonical units).
            (_, _, double ttGuess, _) = GetHohmannXferParams(1.0, r1, r2);

            // dt and tt unconstrained; offset locked to 0 (MJ's rendezvous path). +/-Inf needs no scaling.
            double[] lBnd = { double.NegativeInfinity, double.NegativeInfinity, 0.0 };
            double[] uBnd = { double.PositiveInfinity, double.PositiveInfinity, 0.0 };

            // For (near-)equal periods (co-altitude) the synodic period is infinite and would poison the step
            // cap and march; fall back to the longer orbital period. stpMax = 0 means "no limit" to alglib.
            double searchPeriod = MathHelpers.IsFinite(synodicPeriodReal)
                ? synodicPeriodReal
                : Math.Max(OrbitalPeriod(_r1, _v1, mu), OrbitalPeriod(_r2, _v2, mu));
            double stpMax = MathHelpers.IsFinite(searchPeriod) ? (searchPeriod / scale.TimeScale) / 2.0 : 0.0;

            const int MAX_GLOBAL_ITERATIONS = 50;
            const double DIFF = 1e-6;
            const double EPS = 1e-9;
            const int MAX_ITERATIONS = 1000;

            double dtGuess = 0.0;
            for (int i = 0; i < MAX_GLOBAL_ITERATIONS; i++)
            {
                double[] x = { dtGuess / scale.TimeScale, ttGuess, 0.0 };

                alglib.minbleiccreatef(x, DIFF, out alglib.minbleicstate state);
                alglib.minbleicsetbc(state, lBnd, uBnd);
                alglib.minbleicsetcond(state, 0.0, 0.0, EPS, MAX_ITERATIONS);
                alglib.minbleicsetstpmax(state, stpMax);

                alglib.minbleicoptimize(state, LambertSolver.NlpFunction, null, optArgs);
                alglib.minbleicresults(state, out x, out alglib.minbleicreport rep);

                if (rep.terminationtype < 0)
                    throw new Exception($"Hohmann transfer solver terminated abnormally: {rep.terminationtype}");

                (Vector3d dv1, Vector3d dv2) = LambertSolver.LambertHohmann(x[0], x[1], x[2], optArgs);
                double candDt1 = x[0] * scale.TimeScale;

                HohmannLog.Write("HOHMANN-OPT", "window " + i,
                    "dtGuess=" + dtGuess.ToString("F0") + "s",
                    "dt1=" + candDt1.ToString("F0") + "s",
                    "tt=" + (x[1] * scale.TimeScale).ToString("F0") + "s",
                    "dv1=" + (dv1.magnitude * scale.V).ToString("F2"),
                    "dv2=" + (dv2.magnitude * scale.V).ToString("F2"));

                if (candDt1 > 0.0)
                {
                    Vector3d dv1World = dv1 * scale.V;
                    Vector3d dv2World = dv2 * scale.V;
                    double dt2 = (x[0] + x[1]) * scale.TimeScale;
                    windows.Add((dv1World, ut + candDt1, dv2World, ut + dt2,
                        dv1World.magnitude + dv2World.magnitude));
                }

                dtGuess += searchPeriod * 0.10;
            }

            return windows;
        }

        private static (double dv1, double dv2, double tt, double alpha) GetHohmannXferParams(double mu, Vector3d r1, Vector3d r2)
        {
            const double C = 0.35355339059327373;
            double r1M = r1.magnitude;
            double r2M = r2.magnitude;
            double rsum = r1M + r2M;
            double c1 = Math.Sqrt(2.0 * r2M / rsum);
            double c2 = Math.Sqrt(2.0 * r1M / rsum);
            double dv1 = Math.Sqrt(mu / r1M) * (c1 - 1);
            double dv2 = Math.Sqrt(mu / r2M) * (1 - c2);
            double tt = Math.PI * Math.Sqrt(rsum * rsum * rsum / (8 * mu));
            double c3 = r1M / r2M + 1;
            double alpha = Math.PI * (1 - C * Math.Sqrt(c3 * c3 * c3));
            return (dv1, dv2, tt, alpha);
        }

        public static double GetSeparation(Orbit vessel, Orbit target, double ut) => (WorldPositionAtUt(vessel, ut) - WorldPositionAtUt(target, ut)).magnitude;

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

        private static double LatFromBCI(Vector3d r) => !MathHelpers.IsFinite(r) ? double.NaN : Math.Asin(MathHelpers.Clamp(r.z / r.magnitude, -1.0, 1.0));
        private static double LonFromBCI(Vector3d r) => Math.Atan2(r.y, r.x);

        public static Orbit PerturbedOrbit(Orbit o, double ut, Vector3d dV) => OrbitFromVectors(WorldPositionAtUt(o, ut), o.getOrbitalVelocityAtUT(ut).xzy + dV, o.referenceBody, ut);

        private static Vector3d WorldPositionAtUt(Orbit o, double ut) => o.referenceBody.position + o.getRelativePositionAtUT(ut).xzy;

        // Time for the relative phase of two orbits to realign; infinite when the periods are equal.
        public static double SynodicPeriod(double periodA, double periodB)
        {
            double diff = Math.Abs(1.0 / periodA - 1.0 / periodB);
            return diff < 1e-12 ? double.PositiveInfinity : 1.0 / diff;
        }

        public static double OrbitalPeriod(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;

            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;

            double a = -mu / (2.0 * energy);
            return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
        }

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

        public bool IsBurnSafe(Vessel vessel, Vector3d burn)
        {
            // david todo: implement
            return false;
        }

        public static (double ap, double pe) GetTrueOrbit(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null) return (double.NaN, double.NaN);
            CelestialBody body = vessel.mainBody;
            //CelestialBody disturbingBody = FlightGlobals.Bodies.FirstOrDefault(b => b.name == "Moon");
            
            double mu = vessel.mainBody.gravParameter;
            double bodyRadius = body.Radius;

            // position relative to planet's center
            Vector3d r = vessel.GetWorldPos3D() - body.position;
            double r_mag = r.magnitude;

            // velocity vector relative to planet's center
            //Vector3d v = vessel.obt_velocity;
            Vector3d v = vessel.velocityD + (vessel.GetTotalMass() > 0 ? vessel.perturbation : Vector3d.zero);
            double v_mag = v.magnitude;

            // orbital energy
            double specificEnergy = (v_mag * v_mag / 2.0) - (mu / r_mag);

            // sma
            double semiMajorAxis = -mu / (2.0 * specificEnergy);

            // angular momentum
            Vector3d h = Vector3d.Cross(r, v);

            // eccentricity vector
            Vector3d vCrossH = Vector3d.Cross(v, h);
            Vector3d e_vector = (vCrossH / mu) - (r / r_mag);
            double eccentricity = e_vector.magnitude;

            double apRadius = 0.0;
            double peRadius = 0.0;

            if (eccentricity < 1.0)
            {
                // elliptical or circular orbit
                peRadius = semiMajorAxis * (1.0 - eccentricity);
                apRadius = semiMajorAxis * (1.0 + eccentricity);
            } else
            {
                // hyperbolic or parabolic escape trajectory
                peRadius = semiMajorAxis * (1.0 - eccentricity);
                apRadius = double.PositiveInfinity;
            }

            // convert from absolute CoM to altitudes
            double currentApA = apRadius - bodyRadius;
            double currentPeA = peRadius - bodyRadius;

            return (currentApA, currentPeA);
        }
    }
}
