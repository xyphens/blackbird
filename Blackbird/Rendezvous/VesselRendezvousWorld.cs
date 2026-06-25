using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // In-game implementation of the executor's state seam: reads measured state through
    // TrajectoryProvider (Principia-accurate when present, Stock fallback). Body-relative positions
    // subtract the central body's current position; velocities come back body-centered inertial.
    // Thin adapter — its correctness (notably the velocity frame) is validated in-game, not by the
    // offline harness, which substitutes its own IRendezvousWorld.
    public sealed class VesselRendezvousWorld : IRendezvousWorld
    {
        private readonly Vessel _active;
        private readonly Vessel _target;

        public VesselRendezvousWorld(Vessel active, Vessel target)
        {
            _active = active;
            _target = target;
        }

        public double UniversalTime => Planetarium.GetUniversalTime();
        public double Mu => _active.mainBody.gravParameter;
        public Vector3d ActivePosition => TrajectoryProvider.GetPosition(_active) - _active.mainBody.position;
        public Vector3d ActiveVelocity => TrajectoryProvider.GetVelocity(_active);
        public Vector3d TargetPosition => TrajectoryProvider.GetPosition(_target) - _active.mainBody.position;
        public Vector3d TargetVelocity => TrajectoryProvider.GetVelocity(_target);
        public Vector3d ReferenceNormal => TrajectoryProvider.GetOrbitNormal(_target);
        public double BodyRadius => _active.mainBody.Radius;
        public double AtmosphereDepth => _active.mainBody.atmosphere ? _active.mainBody.atmosphereDepth : 0.0;
    }
}
