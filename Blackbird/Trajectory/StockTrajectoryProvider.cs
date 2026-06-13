using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{
    public sealed class StockTrajectoryProvider : ITrajectoryProvider
    {
        public string SourceName { get { return "Stock"; } }
        public bool IsAvailable { get { return true; } }

        // Captures the current stock/KSP patched-conic vessel state.
        public TrajectoryState GetCurrentState(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null)
            {
                return CreateUnavailable(vessel, "Vessel or reference body is unavailable.");
            }

            CelestialBody body = vessel.mainBody;
            Vector3d worldPosition = vessel.GetWorldPos3D();
            Vector3d relativePosition = worldPosition - body.position;

            return new TrajectoryState
            {
                IsValid = true,
                Source = SourceName,
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
        public OrbitInfo GetOrbitInfo(Vessel vessel)
        {
            return vessel != null ? OrbitInfo.Create(vessel.orbit) : null;
        }

        // Returns the vessel's current stock world position.
        public Vector3d GetPosition(Vessel vessel)
        {
            return vessel != null ? vessel.GetWorldPos3D() : Vector3d.zero;
        }

        // Propagates a stock patched-conic orbit to the requested universal time.
        public Vector3d GetPositionAtUt(Vessel vessel, double universalTime)
        {
            if (vessel == null || vessel.orbit == null) return Vector3d.zero;

            return OrbitMath.GetOrbitPositionAtUt(vessel.orbit, universalTime);
        }

        // Returns the stock orbital velocity currently reported by KSP.
        public Vector3d GetVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.obt_velocity : Vector3d.zero;
        }

        // Returns the stock surface-relative velocity currently reported by KSP.
        public Vector3d GetSurfaceVelocity(Vessel vessel)
        {
            return vessel != null ? vessel.srf_velocity : Vector3d.zero;
        }

        // Returns the stock orbital plane normal.
        public Vector3d GetOrbitNormal(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null) return Vector3d.zero;

            Vector3d relativePosition = GetPosition(vessel) - vessel.mainBody.position;
            Vector3d relativeVelocity = GetVelocity(vessel);
            Vector3d normal = Vector3d.Cross(relativePosition, relativeVelocity);

            return normal.sqrMagnitude > 0.0
                ? normal.normalized
                : Vector3d.zero;
        }

        // Returns stock apoapsis altitude for orbit summaries and UI.
        public double GetApoapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.ApA : double.NaN;
        }

        // Returns stock periapsis altitude for orbit summaries and UI.
        public double GetPeriapsisAlt(Vessel vessel)
        {
            return vessel != null && vessel.orbit != null ? vessel.orbit.PeA : double.NaN;
        }

        private TrajectoryState CreateUnavailable(Vessel vessel, string reason)
        {
            return new TrajectoryState
            {
                IsValid = false,
                Source = SourceName,
                ReasonUnavailable = reason,
                Vessel = vessel,
                UniversalTime = Planetarium.GetUniversalTime()
            };
        }
    }
}
