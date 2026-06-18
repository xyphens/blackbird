using Blackbird.Trajectory;
using System;
using Blackbird.Logging;

namespace Blackbird.Rendezvous
{
    // Snapshot of the chaser/target relative geometry plus the target-centered LVLH frame.
    // Convention (per the rendezvous contract): the relative vectors point FROM the active
    // (chaser) vessel TO the target, i.e. relative = target - active. So a positive RangeRate
    // means the gap is opening; negative means closing.
    [System.Serializable]
    public struct RelativeState
    {
        public Vector3d RelativePositionWorld;  // target - active, world frame (m)
        public Vector3d RelativeVelocityWorld;  // target - active, world frame (m/s)

        public LvlhFrame Frame;                 // target-centered LVLH basis

        public Vector3d RelativePositionLvlh;   // RelativePositionWorld in LVLH (radial, alongTrack, crossTrack)
        public Vector3d RelativeVelocityLvlh;   // RelativeVelocityWorld in LVLH

        public double Range;                    // |RelativePositionWorld| (m)
        public double RangeRate;                // d(Range)/dt; <0 closing, >0 separating (m/s)

        // Pure computation from raw state vectors. No KSP/Unity runtime dependency beyond
        // Vector3d, so it is directly unit-testable for a known geometry.
        // bodyPositionWorld is the central body's center; the LVLH frame is built from the
        // target's body-relative position and inertial velocity.
        public static RelativeState Compute(
            Vector3d activePositionWorld,
            Vector3d activeVelocityWorld,
            Vector3d targetPositionWorld,
            Vector3d targetVelocityWorld,
            Vector3d bodyPositionWorld)
        {
            Vector3d relPos = targetPositionWorld - activePositionWorld;
            Vector3d relVel = targetVelocityWorld - activeVelocityWorld;

            LvlhFrame frame = LvlhFrame.Build(targetPositionWorld - bodyPositionWorld, targetVelocityWorld);

            double range = relPos.magnitude;
            double rangeRate = range > 0.0 ? Vector3d.Dot(relPos, relVel) / range : 0.0;

            RelativeState state = new RelativeState
            {
                RelativePositionWorld = relPos,
                RelativeVelocityWorld = relVel,
                Frame = frame,
                RelativePositionLvlh = frame.ToLocal(relPos),
                RelativeVelocityLvlh = frame.ToLocal(relVel),
                Range = range,
                RangeRate = rangeRate
            };

            return state;
        }

        // In-game convenience overload: measures current state from the active trajectory
        // provider (Principia-accurate, Stock fallback) per the contract's "measure with
        // Principia" invariant.
        public static RelativeState Compute(Vessel active, Vessel target)
        {
            return Compute(
                TrajectoryProvider.GetPosition(active),
                TrajectoryProvider.GetVelocity(active),
                TrajectoryProvider.GetPosition(target),
                TrajectoryProvider.GetVelocity(target),
                target.mainBody.position);
        }
    }
}
