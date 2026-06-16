using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Step 3 terminal-rendezvous executor SKELETON: the staged phase state machine, mirroring
    // ClassicAscentGuidance's plan/execute/cutoff structure but generalized to the user-gated
    // intercept -> match-velocity -> close sequence.
    //
    // Gating model (contract invariant 1): the user manually Arm()s and Trigger()s each stage
    // (gates BETWEEN stages); the closed loop runs automatically WITHIN a stage (Executing) and
    // self-terminates on its cutoff condition, dropping to Coast (or Complete after the last stage).
    //
    // NO burns are wired yet. Each stage's StepStage() is a no-op that reports complete immediately,
    // so the machine can be driven end-to-end and validated now. Steps 4/6/7 replace those bodies
    // with the real closed loop and gate completion on delivered-ΔV / matched velocity / separation.
    public sealed class TerminalRendezvousExecutor
    {
        public RendezvousPhase Phase { get; private set; }
        public RendezvousStage Stage { get; private set; }
        public bool IsComplete => Phase == RendezvousPhase.Complete;

        public TerminalRendezvousExecutor()
        {
            Reset();
        }

        // Returns to the initial Idle state at the first stage.
        public void Reset()
        {
            Phase = RendezvousPhase.Idle;
            Stage = RendezvousStage.Intercept;
        }

        // User gate: arm the next stage. Valid from Idle (arms the first stage) or Coast (advances to
        // and arms the following stage). Returns false when not in an armable state.
        public bool Arm()
        {
            if (Phase == RendezvousPhase.Idle)
            {
                Phase = RendezvousPhase.Armed;   // Stage is already the first/current stage
                return true;
            }
            if (Phase == RendezvousPhase.Coast)
            {
                Stage = NextStage(Stage);
                Phase = RendezvousPhase.Armed;
                return true;
            }
            return false;
        }

        // User gate: begin the armed stage's closed loop. Valid only from Armed.
        public bool Trigger()
        {
            if (Phase != RendezvousPhase.Armed) return false;
            Phase = RendezvousPhase.Executing;
            return true;
        }

        // User action: abort the sequence. No further commands are issued until Reset().
        public void Abort()
        {
            Phase = RendezvousPhase.Aborted;
        }

        // Per-tick update. Runs the active stage when Executing and advances the phase on completion;
        // otherwise returns an idle (no-burn) command describing the current state.
        public RendezvousCommand Update(IRendezvousWorld world)
        {
            switch (Phase)
            {
                case RendezvousPhase.Executing:
                    if (world == null) return Idle("executing — no world state");
                    RendezvousCommand cmd = StepStage(world, out bool stageComplete);
                    if (stageComplete) CompleteStage();
                    cmd.Phase = Phase;   // reflect the post-transition phase/stage
                    cmd.Stage = Stage;
                    return cmd;

                case RendezvousPhase.Armed:
                    return Idle("armed " + Stage + " — awaiting trigger");
                case RendezvousPhase.Coast:
                    return Idle("coasting after " + Stage + " — arm next stage");
                case RendezvousPhase.Complete:
                    return Idle("rendezvous complete — control handed back");
                case RendezvousPhase.Aborted:
                    return Idle("aborted");
                default: // Idle
                    return Idle("idle — arm a stage");
            }
        }

        // Executes one tick of the current stage. SKELETON: produces no burn and reports the stage
        // complete immediately (no work wired). Steps 4/6/7 replace this body with the real closed
        // loop and set stageComplete only on that stage's true cutoff condition.
        private RendezvousCommand StepStage(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = true;

            string label;
            switch (Stage)
            {
                case RendezvousStage.Intercept:     label = "intercept (stub: no burn wired)"; break;
                case RendezvousStage.MatchVelocity: label = "match velocity (stub: no burn wired)"; break;
                default:                            label = "close approach (stub: no burn wired)"; break;
            }
            return Idle(label);
        }

        // Advances out of a finished Executing stage: Coast after a non-final stage, Complete after
        // the last one (CloseApproach).
        private void CompleteStage()
        {
            Phase = Stage == RendezvousStage.CloseApproach
                ? RendezvousPhase.Complete
                : RendezvousPhase.Coast;
        }

        private static RendezvousStage NextStage(RendezvousStage stage)
        {
            switch (stage)
            {
                case RendezvousStage.Intercept:     return RendezvousStage.MatchVelocity;
                case RendezvousStage.MatchVelocity: return RendezvousStage.CloseApproach;
                default:                            return RendezvousStage.CloseApproach;
            }
        }

        // Builds a no-burn command stamped with the current phase/stage.
        private RendezvousCommand Idle(string status)
        {
            return new RendezvousCommand
            {
                Phase = Phase,
                Stage = Stage,
                HasBurn = false,
                ThrustDirection = Vector3d.zero,
                Throttle = 0.0,
                Status = status
            };
        }
    }
}
