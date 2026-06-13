using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{
    public interface ITrajectoryProvider
    {
        string SourceName { get; }
        bool IsAvailable { get; }

        TrajectoryState GetCurrentState(Vessel vessel);
        OrbitInfo GetOrbitInfo(Vessel vessel);
        Vector3d GetPosition(Vessel vessel);
        Vector3d GetPositionAtUt(Vessel vessel, double universalTime);
        Vector3d GetVelocity(Vessel vessel);
        Vector3d GetSurfaceVelocity(Vessel vessel);
        Vector3d GetOrbitNormal(Vessel vessel);
        double GetApoapsisAlt(Vessel vessel);
        double GetPeriapsisAlt(Vessel vessel);
    }
}
