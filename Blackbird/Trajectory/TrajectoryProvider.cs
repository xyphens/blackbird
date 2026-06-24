using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{

    public static class TrajectoryProvider
    {
        public static TrajectoryState GetCurrentState(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null)
                return CreateUnavailable(vessel, "Vessel or reference body is unavailable.");

            CelestialBody body = vessel.mainBody;
            Vector3d worldPosition = vessel.GetWorldPos3D();
            Vector3d relativePosition = worldPosition - body.position;

            return new TrajectoryState
            {
                IsValid = true,
                Vessel = vessel,
                ReferenceBody = body,
                UniversalTime = Planetarium.GetUniversalTime(),
                WorldPosition = worldPosition,
                WorldVelocity = vessel.obt_velocity,
                RelativePosition = relativePosition,
                RelativeVelocity = vessel.obt_velocity,
                AltitudeMeters = OrbitMath.GetAltitudeAtPosition(body, worldPosition),
                LatitudeDeg = vessel.latitude,
                LongitudeDeg = vessel.longitude
            };
        }

        public static OrbitInfo GetOrbitInfo(Vessel vessel)
        {
            return vessel != null ? OrbitInfo.Create(vessel.orbit) : null;
        }

        public static Vector3d GetPosition(Vessel vessel)
        {
            return vessel != null ? vessel.GetWorldPos3D() : Vector3d.zero;
        }

        public static Vector3d GetPositionAtUt(Vessel vessel, double universalTime)
        {
            if (vessel == null || vessel.orbit == null) return Vector3d.zero;
            return OrbitMath.GetOrbitPositionAtUt(vessel.orbit, universalTime);
        }

        public static Vector3d GetVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.obt_velocity : Vector3d.zero;
        }

        public static Vector3d GetSurfaceVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.srf_velocity : Vector3d.zero;
        }

        public static Vector3d GetOrbitalVelocity(Vessel vessel)
        {
            return vessel == null ? Vector3d.zero : vessel.orbit.GetVel();
        }

        public static Vector3d GetOrbitNormal(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null) return Vector3d.zero;
            
            Vector3d relativePosition = GetPosition(vessel) - vessel.mainBody.position;
            Vector3d relativeVelocity = GetVelocity(vessel);
            Vector3d normal = Vector3d.Cross(relativePosition, relativeVelocity);

            return normal.sqrMagnitude > 0.0 ? normal.normalized : Vector3d.zero;
        }

        public static Vector3d GetKspOrbitNormal(Vessel vessel)
        {
            return vessel.orbit.GetOrbitNormal();
        }

        public static double GetApoapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.ApA : double.NaN;
        }

        public static double GetPeriapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.PeA : double.NaN;
        }

        private static TrajectoryState CreateUnavailable(Vessel vessel, string reason)
        {
            return new TrajectoryState
            {
                IsValid = false,
                ReasonUnavailable = reason,
                Vessel = vessel,
                UniversalTime = Planetarium.GetUniversalTime()
            };
        }
    }
}
