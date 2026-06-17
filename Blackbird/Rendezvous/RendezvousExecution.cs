using UnityEngine;

namespace Blackbird.Rendezvous
{
    // The ordered terminal-rendezvous stages. The executor advances through them in this order, with
    // a manual user gate between each (contract invariant 1: closed-loop within a stage, manual gates
    // between stages). Burns for each stage are wired in later contract steps (4, 6, 7).
    public enum RendezvousStage
    {
        Intercept,      // Step 4: burn onto the Lambert transfer toward the target
        MatchVelocity,  // Step 6: null relative velocity at closest approach
        CloseApproach   // Step 7: close to ~100 m and station-keep / hand off
    }

    // Lifecycle phase within the sequence. Idle/Coast are the between-stage rest states where the user
    // is expected to act (Execute the current stage); Executing is the autopilot-driven active state.
    public enum RendezvousPhase
    {
        Idle,        // first stage not started yet; awaiting Execute
        Executing,   // closed-loop guidance running for the current stage
        Coast,       // a stage finished and the NEXT stage is queued (Stage already advanced); awaiting Execute
        Complete,    // final stage finished; control handed back to the player
        Aborted      // user aborted; no commands issued until Reset
    }

    // Per-tick output of the executor: the steering/throttle command plus the current phase/stage for
    // display and logging. When HasBurn is false the caller holds attitude and cuts throttle (idle).
    public struct RendezvousCommand
    {
        public RendezvousPhase Phase;
        public RendezvousStage Stage;
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

    // The state seam between the executor and the world. In-game this is backed by Vessels +
    // TrajectoryProvider (VesselRendezvousWorld); offline the harness supplies a two-body-propagated
    // fake. Routing the executor through this interface (rather than touching Vessel directly) is what
    // makes the Steps 4-7 closed-loop logic offline-testable. All positions are body-relative (central
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
    }
}
