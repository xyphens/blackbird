using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{
    // Single source of vessel trajectory reads
    // Principia provider split: the Principia provider was inert (its reflection bound the wrong assembly
    // name, so IsAvailable was always false and it silently fell back to stock), so the dual-provider
    // abstraction was removed. All reads are stock / KSP patched-conic — which equal the true instantaneous
    // state even under Principia, since KSP's vessel.orbit is the osculating orbit Principia maintains.
    // NOTE: GetPositionAtUt is two-body conic — exact in stock, an approximation under n-body. If genuine
    // Principia n-body reads are ever wired, branch to a guarded reflection shim from here.
    public static class TrajectoryProvider
    {
        public static string ActiveSourceName { get { return "Stock"; } }

        // Captures the current KSP patched-conic vessel state.
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
                Source = ActiveSourceName,
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

        // Reads the stock osculating orbit elements reported by KSP.
        public static OrbitInfo GetOrbitInfo(Vessel vessel)
        {
            return vessel != null ? OrbitInfo.Create(vessel.orbit) : null;
        }

        // Returns the vessel's current world position.
        public static Vector3d GetPosition(Vessel vessel)
        {
            return vessel != null ? vessel.GetWorldPos3D() : Vector3d.zero;
        }

        // Propagates a stock patched-conic orbit to the requested universal time.
        public static Vector3d GetPositionAtUt(Vessel vessel, double universalTime)
        {
            if (vessel == null || vessel.orbit == null) return Vector3d.zero;
            return OrbitMath.GetOrbitPositionAtUt(vessel.orbit, universalTime);
        }

        // Returns the stock orbital velocity currently reported by KSP.
        public static Vector3d GetVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.obt_velocity : Vector3d.zero;
        }

        // Returns the stock surface-relative velocity currently reported by KSP.
        public static Vector3d GetSurfaceVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.srf_velocity : Vector3d.zero;
        }

        // Returns stock two-body osculating orbital velocity from KSP.
        public static Vector3d GetOrbitalVelocity(Vessel vessel)
        {
            return vessel == null ? Vector3d.zero : vessel.orbit.GetVel();
        }

        // Returns the stock orbital plane normal.
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

        // Returns stock apoapsis altitude for orbit summaries and UI.
        public static double GetApoapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.ApA : double.NaN;
        }

        // Returns stock periapsis altitude for orbit summaries and UI.
        public static double GetPeriapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.PeA : double.NaN;
        }

        private static TrajectoryState CreateUnavailable(Vessel vessel, string reason)
        {
            return new TrajectoryState
            {
                IsValid = false,
                Source = ActiveSourceName,
                ReasonUnavailable = reason,
                Vessel = vessel,
                UniversalTime = Planetarium.GetUniversalTime()
            };
        }
    }
}
