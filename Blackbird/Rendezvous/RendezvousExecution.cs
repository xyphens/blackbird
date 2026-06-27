using Blackbird.Modules;
namespace Blackbird.Rendezvous
{
    // Per-tick output of the executor: the steering/throttle command plus the current phase/stage for
    // display and logging. When HasBurn is false the caller holds attitude and cuts throttle (idle).
    public struct RendezvousCommand
    {
        public InterceptPhase Phase;
        public RendezvousMethod Method;
        public bool HasBurn;             // false => idle/coast: no steering, zero throttle
        public Vector3d ThrustDirection; // world-frame unit vector (meaningful only when HasBurn)
        public double Throttle;          // 0..1 (meaningful only when HasBurn)
        public string Status;            // human-readable, for UI/log
    }

    // Post-burn summary of a completed intercept burn, recorded by the executor at cutoff for diagnostics:
    // it quantifies how well the burn matched its plan (over/under-burn, direction error) and is paired by
    // the actuation layer with the predicted-vs-achieved closest approach. Purely for logging/analysis.
    public struct InterceptBurnReport
    {
        public double PlannedDvMagnitude;       // |ΔV| the plan asked for (m/s)
        public Vector3d PlannedDvVector;        // planned ΔV, world frame
        public double DeliveredAlongAxis;       // velocity change projected onto the planned axis at cutoff (m/s)
        public Vector3d DeliveredVector;        // actual total velocity change over the burn, world frame
        public double VelocityResidual;         // |target departure velocity - achieved velocity| at cutoff (m/s)
        public double PredictedClosestApproach; // CA the plan predicted (m)
        public string CutoffReason;             // which cutoff fired (reached / stalled / overshot)
    }

    // The state seam between the executor and the world. In-game it is backed by Vessels +
    // TrajectoryProvider (VesselRendezvousWorld); offline the harness supplies a two-body-propagated fake,
    // which is what makes the closed-loop logic offline-testable. All positions are body-relative (central
    // body subtracted); velocities are body-centered inertial — the frame the conic math expects.
    public interface IRendezvousWorld
    {
        double UniversalTime { get; }
        double Mu { get; }
        Vector3d ActivePosition { get; }
        Vector3d ActiveVelocity { get; }
        Vector3d TargetPosition { get; }
        Vector3d TargetVelocity { get; }
        Vector3d ReferenceNormal { get; }   // target orbit normal, for transfer-direction disambiguation
        double BodyRadius { get; }           // central body radius (m), for phasing-orbit safety floor
        double AtmosphereDepth { get; }      // atmosphere top altitude (m), 0 if airless

        // Oblateness model for J2-aware planning/prediction. J2 == 0 => point-mass (Stock; bodies Principia
        // models as point masses), so every J2 path falls back to the conic two-body result.
        double J2 { get; }                   // unnormalized J2 (0 = none)
        double J2ReferenceRadius { get; }    // equatorial reference radius (m) the J2 term is defined against
        Vector3d Pole { get; }               // body spin axis (unit), for the J2 acceleration
    }
}
