using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{
    public static class TrajectoryProvider
    {
        private static readonly StockTrajectoryProvider Stock = new StockTrajectoryProvider();
        private static readonly PrincipiaTrajectoryProvider Principia = new PrincipiaTrajectoryProvider();

        public static string ActiveSourceName
        {
            get { return Principia.IsAvailable ? Principia.SourceName : Stock.SourceName; }
        }

        // Returns the best available current vessel state, preferring Principia when it is running.
        public static TrajectoryState GetCurrentState(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetCurrentState(vessel)
                : Stock.GetCurrentState(vessel);
        }

        // Returns orbit summary data from the active provider.
        public static OrbitInfo GetOrbitInfo(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetOrbitInfo(vessel)
                : Stock.GetOrbitInfo(vessel);
        }

        // Returns current world position from the active provider.
        public static Vector3d GetPosition(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetPosition(vessel)
                : Stock.GetPosition(vessel);
        }

        // Returns a future position from the active provider, falling back to stock patched conics.
        public static Vector3d GetPositionAtUt(Vessel vessel, double universalTime)
        {
            return Principia.IsAvailable
                ? Principia.GetPositionAtUt(vessel, universalTime)
                : Stock.GetPositionAtUt(vessel, universalTime);
        }

        // Returns current velocity from the active provider.
        public static Vector3d GetVelocity(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetVelocity(vessel)
                : Stock.GetVelocity(vessel);
        }

        // Returns current surface-relative velocity from the active provider.
        public static Vector3d GetSurfaceVelocity(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetSurfaceVelocity(vessel)
                : Stock.GetSurfaceVelocity(vessel);
        }

        // Returns orbit plane normal from the active provider.
        public static Vector3d GetOrbitNormal(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetOrbitNormal(vessel)
                : Stock.GetOrbitNormal(vessel);
        }

        // Returns apoapsis altitude from the active provider.
        public static double GetApoapsisAlt(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetApoapsisAlt(vessel)
                : Stock.GetApoapsisAlt(vessel);
        }

        // Returns periapsis altitude from the active provider.
        public static double GetPeriapsisAlt(Vessel vessel)
        {
            return Principia.IsAvailable
                ? Principia.GetPeriapsisAlt(vessel)
                : Stock.GetPeriapsisAlt(vessel);
        }
    }
}
