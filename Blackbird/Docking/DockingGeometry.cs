using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Docking
{
    public struct Box3d
    {
        public Vector3d center;
        public Vector3d size;
    }

    public struct VectorPair
    {
        public Vector3 P1;

        public Vector3 P2;

        public VectorPair(Vector3 point1, Vector3 point2)
        {
            P1 = point1;
            P2 = point2;
        }
    }

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
    // alignment error.
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
            double alignDeg = MathHelpers.Rad2Deg(Math.Acos(alignDot));
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

        public static VectorPair _getBoundingBox(Part part)
        {
            Vector3 minBounds = new Vector3();
            Vector3 maxBounds = new Vector3();

            foreach (Transform t in part.FindModelComponents<Transform>())
            {
                MeshFilter mf = t.GetComponent<MeshFilter>();
                if (mf == null)
                    continue;
                Mesh m = mf.mesh;

                if (m == null)
                    continue;

                Matrix4x4 matrix = part.vessel.transform.worldToLocalMatrix * t.localToWorldMatrix;

                foreach (Vector3 vertex in m.vertices)
                {
                    Vector3 v = matrix.MultiplyPoint3x4(vertex);
                    maxBounds.x = Mathf.Max(maxBounds.x, v.x);
                    minBounds.x = Mathf.Min(minBounds.x, v.x);
                    maxBounds.y = Mathf.Max(maxBounds.y, v.y);
                    minBounds.y = Mathf.Min(minBounds.y, v.y);
                    maxBounds.z = Mathf.Max(maxBounds.z, v.z);
                    minBounds.z = Mathf.Min(minBounds.z, v.z);
                }
            }

            return new VectorPair(maxBounds, minBounds);
        }

        public static Box3d GetBoundingBox(Vessel vessel)
        {
            Vector3 minBounds = new Vector3();
            Vector3 maxBounds = new Vector3();

            for (int i = 0; i < vessel.Parts.Count; i++) {
                Part part = vessel.parts[i];
                VectorPair partBox = _getBoundingBox(part);

                maxBounds.x = Mathf.Max(maxBounds.x, partBox.P1.x);
                minBounds.x = Mathf.Min(minBounds.x, partBox.P2.x);
                maxBounds.y = Mathf.Max(maxBounds.y, partBox.P1.y);
                minBounds.y = Mathf.Min(minBounds.y, partBox.P2.y);
                maxBounds.z = Mathf.Max(maxBounds.z, partBox.P1.z);
                minBounds.z = Mathf.Min(minBounds.z, partBox.P2.z);
            }

            Box3d box = new Box3d();

            box.center = new Vector3d((maxBounds.x + minBounds.x) / 2, (maxBounds.y + minBounds.y) / 2, (maxBounds.z + minBounds.z) / 2);
            box.size = new Vector3d(Math.Abs(box.center.x - maxBounds.x), Math.Abs(box.center.y - maxBounds.y), Math.Abs(box.center.z - maxBounds.z));

            return box;
        }
     }
}
