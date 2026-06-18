using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Docking
{
    // A docking port reduced to the two world-frame quantities the guidance needs: where it is and which
    // way it faces. Axis is the OUTWARD normal of the port's mating face (in-game: nodeTransform.forward) —
    // the direction you approach FROM. For two ports to mate, the chaser port must sit on the target port's
    // axis (zero lateral offset) and face back down it, i.e. chaser axis anti-parallel to target axis.
    public struct PortState
    {
        public Vector3d Position;   // world (m)
        public Vector3d Axis;       // world unit, outward face normal

        public PortState(Vector3d position, Vector3d axis)
        {
            Position = position;
            Axis = axis.magnitude > 1e-9 ? axis / axis.magnitude : axis;
        }
    }

    // Pure docking geometry between the chaser port and the target port: frame-independent (world vectors
    // in, scalars + world vectors out) so it is offline-testable like RelativeState. It decomposes the
    // chaser's offset from the target port into ALONG-AXIS (how far in front of the port face) and LATERAL
    // (how far off the centerline), gives the standoff approach waypoint out on the axis, and the attitude
    // alignment error. The docking stage (D2) steers on these; the in-game seam (D3) supplies the port
    // transforms from the live ModuleDockingNodes.
    public struct DockingGeometry
    {
        public double AxialDistance;      // chaser offset projected on the target axis; >0 = in front of the face
        public double LateralOffset;      // perpendicular distance from the port centerline (m)
        public Vector3d LateralDirection; // unit, points from the centerline out to the chaser (zero if on-axis)
        public Vector3d ApproachWaypoint; // standoff point on the axis: targetPort + axis*standoff (world)
        public double AlignmentErrorDeg;  // angle between chaser axis and the desired -targetAxis (0 = mated heading)
        public double Range;              // |chaserPort - targetPort| (m)

        // standoffDistance: how far out along the target axis the safe approach waypoint sits. The chaser
        // first flies to ApproachWaypoint (on-axis, clear of the target), then translates straight in.
        public static DockingGeometry Compute(PortState chaserPort, PortState targetPort, double standoffDistance)
        {
            Vector3d axis = targetPort.Axis;                            // outward normal of the target port face
            Vector3d offset = chaserPort.Position - targetPort.Position; // target port -> chaser port

            double axial = Vector3d.Dot(offset, axis);                 // along the axis (+ = in front of the face)
            Vector3d lateralVec = offset - axis * axial;               // perpendicular (off-centerline) component
            double lateral = lateralVec.magnitude;

            // Desired chaser heading is anti-parallel to the target axis (the two port faces meet head-on).
            double alignDot = MathHelpers.Clamp(Vector3d.Dot(chaserPort.Axis, -axis), -1.0, 1.0);
            double alignDeg = Math.Acos(alignDot) * 180.0 / Math.PI;

            return new DockingGeometry
            {
                AxialDistance = axial,
                LateralOffset = lateral,
                LateralDirection = lateral > 1e-6 ? lateralVec / lateral : Vector3d.zero,
                ApproachWaypoint = targetPort.Position + axis * standoffDistance,
                AlignmentErrorDeg = alignDeg,
                Range = offset.magnitude
            };
        }
    }
}
