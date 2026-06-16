using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Target-centered LVLH / Hill frame: an orthonormal right-handed basis defined by the
    // target's instantaneous state. Component order is (R, V, H):
    //   RBar - radial:      unit vector from the central body's center toward the target (outward +)
    //   VBar - along-track:  in-plane, perpendicular to R, in the direction of orbital motion
    //   HBar - cross-track:  orbit normal, (r x v) normalized
    // Right-handed in (R, V, H): RBar x VBar = HBar (since VBar = HBar x RBar).
    // Pure value type: no KSP/Unity runtime dependency beyond Vector3d, so it is unit-testable.
    public struct LvlhFrame
    {
        public Vector3d RBar;
        public Vector3d VBar;
        public Vector3d HBar;

        // Builds the frame from the target's body-relative position and inertial velocity.
        // targetPositionFromBody = targetWorldPosition - bodyWorldPosition.
        public static LvlhFrame Build(Vector3d targetPositionFromBody, Vector3d targetVelocity)
        {
            Vector3d rBar = targetPositionFromBody.normalized;
            Vector3d hBar = Vector3d.Cross(targetPositionFromBody, targetVelocity).normalized;
            Vector3d vBar = Vector3d.Cross(hBar, rBar).normalized;

            return new LvlhFrame { RBar = rBar, VBar = vBar, HBar = hBar };
        }

        // Decomposes a world-frame vector into LVLH components (radial, alongTrack, crossTrack).
        public Vector3d ToLocal(Vector3d worldVector)
        {
            return new Vector3d(
                Vector3d.Dot(worldVector, RBar),
                Vector3d.Dot(worldVector, VBar),
                Vector3d.Dot(worldVector, HBar));
        }

        // Reconstructs a world-frame vector from LVLH components (radial, alongTrack, crossTrack).
        public Vector3d ToWorld(Vector3d localVector)
        {
            return localVector.x * RBar + localVector.y * VBar + localVector.z * HBar;
        }
    }
}
