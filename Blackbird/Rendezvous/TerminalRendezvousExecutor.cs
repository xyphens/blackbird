using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Terminal-rendezvous executor: the staged phase state machine, mirroring ClassicAscentGuidance's
    // plan/execute/cutoff structure, generalized to the user-gated intercept -> match-velocity -> close
    // sequence.
    //
    // Gating model (contract invariant 1): the user manually Arm()s and Trigger()s each stage (gates
    // BETWEEN stages); the closed loop runs automatically WITHIN a stage (Executing) and self-terminates
    // on its cutoff condition, dropping to Coast (or Complete after the last stage).
    //
    // Step 4 wires the INTERCEPT stage: on arm it plans a conic Lambert intercept (InterceptSolver) for
    // a UI preview; on the first executing tick it re-plans from the current measured state, freezes the
    // world-frame ΔV, then steers along it at (tapered) full throttle and cuts when the delivered ΔV
    // along that vector reaches the planned magnitude — exactly ClassicAscentGuidance's node executor,
    // and Principia-safe (no patched-conic node). MatchVelocity/CloseApproach remain stubs (Steps 6/7).
    public sealed class TerminalRendezvousExecutor
    {
        // --- intercept tuning (public so callers/tests can adjust) -------------------------------
        public int InterceptArrivalSamples = 60;          // Lambert solves per plan (bounded)
        public double InterceptBudgetMilliseconds = 20.0; // wall-clock cap per plan
        public double InterceptTofMinFraction = 0.05;     // arrival sweep, as a fraction of the orbital period
        public double InterceptTofMaxFraction = 0.95;

        private const double NegligibleDeltaV = 1e-3;             // m/s; below this the burn is a no-op
        private const double BurnTaperBandMetersPerSecond = 5.0;  // throttle tapers over the last few m/s
        private const double BurnMinThrottle = 0.05;

        public RendezvousPhase Phase { get; private set; }
        public RendezvousStage Stage { get; private set; }
        public bool IsComplete => Phase == RendezvousPhase.Complete;

        // Latest intercept plan (preview while Armed, frozen plan while Executing). For UI/logging.
        public bool HasInterceptPlan { get; private set; }
        public InterceptSolution InterceptPlan { get; private set; }

        // Burn execution state for the intercept stage.
        private bool _armedPlanComputed;          // preview plan computed for the current Armed phase
        private bool _burnArmed;                   // burn baseline captured for the current Executing phase
        private Vector3d _burnStartVelocity;       // orbital velocity at ignition, for delivered-ΔV cutoff
        private double _plannedDvMagnitude;        // |ΔV| to deliver
        private Vector3d _plannedDvUnit;           // frozen world-frame burn direction

        public TerminalRendezvousExecutor()
        {
            Reset();
        }

        // Returns to the initial Idle state at the first stage and clears any cached plan/burn state.
        public void Reset()
        {
            Phase = RendezvousPhase.Idle;
            Stage = RendezvousStage.Intercept;
            ClearInterceptState();
            HasInterceptPlan = false;
            InterceptPlan = default(InterceptSolution);
        }

        // User gate: arm the next stage. Valid from Idle (arms the first stage) or Coast (advances to
        // and arms the following stage). Returns false when not in an armable state.
        public bool Arm()
        {
            if (Phase == RendezvousPhase.Idle)
            {
                Phase = RendezvousPhase.Armed;       // Stage is already the first/current stage
                ClearInterceptState();
                return true;
            }
            if (Phase == RendezvousPhase.Coast)
            {
                Stage = NextStage(Stage);
                Phase = RendezvousPhase.Armed;
                ClearInterceptState();
                return true;
            }
            return false;
        }

        // User gate: begin the armed stage's closed loop. Valid only from Armed. The freshest plan is
        // taken on the first executing tick, so re-arm the burn baseline here.
        public bool Trigger()
        {
            if (Phase != RendezvousPhase.Armed) return false;
            Phase = RendezvousPhase.Executing;
            _burnArmed = false;
            return true;
        }

        // User action: abort the sequence. No further commands are issued until Reset().
        public void Abort()
        {
            Phase = RendezvousPhase.Aborted;
            ClearInterceptState();
        }

        // Per-tick update. Plans a preview while Armed, runs the active stage while Executing (advancing
        // the phase on completion), and otherwise returns an idle (no-burn) command for the current state.
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
                    if (world != null && Stage == RendezvousStage.Intercept && !_armedPlanComputed)
                    {
                        InterceptPlan = PlanIntercept(world);
                        HasInterceptPlan = InterceptPlan.Success;
                        _armedPlanComputed = true;
                    }
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

        // Dispatches one tick to the active stage. Intercept is wired (Step 4); the others are stubs
        // that complete instantly until Steps 6/7 fill them in.
        private RendezvousCommand StepStage(IRendezvousWorld world, out bool stageComplete)
        {
            if (Stage == RendezvousStage.Intercept)
                return StepIntercept(world, out stageComplete);

            stageComplete = true;
            return Idle(Stage + " (stub: no burn wired)");
        }

        // Intercept stage closed loop: freeze a fresh plan on entry, then steer along the planned ΔV and
        // cut when the velocity change delivered ALONG that vector reaches the planned magnitude. The
        // delivered measure uses orbital velocity (so it also counts the small gravity contribution over
        // the burn, like ClassicAscentGuidance); the residual is corrected by the Step 5 coast re-solve.
        private RendezvousCommand StepIntercept(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

            if (!_burnArmed)
            {
                InterceptSolution plan = PlanIntercept(world);
                InterceptPlan = plan;
                HasInterceptPlan = plan.Success;
                if (!plan.Success)
                    return Idle("intercept: no feasible plan (" + plan.Status + ")");

                _plannedDvMagnitude = plan.DeltaVMagnitude;
                _plannedDvUnit = plan.DeltaV.normalized;
                _burnStartVelocity = world.ActiveVelocity;
                _burnArmed = true;
            }

            if (_plannedDvMagnitude <= NegligibleDeltaV)
            {
                stageComplete = true;
                return Idle("intercept: negligible ΔV");
            }

            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);
            if (delivered >= _plannedDvMagnitude)
            {
                stageComplete = true;
                return Idle("intercept: burn complete");
            }

            double remaining = _plannedDvMagnitude - delivered;
            double throttle = MathHelpers.Clamp(remaining / BurnTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(_plannedDvUnit, throttle,
                string.Format("intercept burn {0:F1}/{1:F1} m/s", delivered, _plannedDvMagnitude));
        }

        // Plans a conic intercept from the current measured state. Target prediction is two-body
        // propagation of the measured target state (contract invariant 2: plan conic, measure with
        // Principia via the world, let the closed loop absorb the gap). The arrival sweep spans a
        // fraction of the active orbit's period.
        private InterceptSolution PlanIntercept(IRendezvousWorld world)
        {
            double period = OrbitalPeriod(world.ActivePosition, world.ActiveVelocity, world.Mu);
            double tofMin, tofMax;
            if (MathHelpers.IsFinite(period) && period > 0.0)
            {
                tofMin = InterceptTofMinFraction * period;
                tofMax = InterceptTofMaxFraction * period;
            }
            else
            {
                tofMin = 60.0;
                tofMax = 3600.0;
            }

            double t0 = world.UniversalTime;
            double mu = world.Mu;
            Vector3d targetPosition = world.TargetPosition;
            Vector3d targetVelocity = world.TargetVelocity;

            Func<double, Vector3d> targetPositionAt = ut =>
            {
                TwoBody.Propagate(targetPosition, targetVelocity, mu, ut - t0, out Vector3d rt, out _);
                return rt;
            };

            return InterceptSolver.Solve(world.ActivePosition, world.ActiveVelocity, mu,
                world.ReferenceNormal, t0, targetPositionAt, tofMin, tofMax,
                InterceptArrivalSamples, true, InterceptBudgetMilliseconds);
        }

        // Keplerian period from a state vector; NaN if the orbit is unbound.
        private static double OrbitalPeriod(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;

            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;

            double a = -mu / (2.0 * energy);
            return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
        }

        // Advances out of a finished Executing stage: Coast after a non-final stage, Complete after the
        // last one (CloseApproach).
        private void CompleteStage()
        {
            ClearInterceptState();
            Phase = Stage == RendezvousStage.CloseApproach
                ? RendezvousPhase.Complete
                : RendezvousPhase.Coast;
        }

        private void ClearInterceptState()
        {
            _armedPlanComputed = false;
            _burnArmed = false;
            _burnStartVelocity = Vector3d.zero;
            _plannedDvMagnitude = 0.0;
            _plannedDvUnit = Vector3d.zero;
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

        // Builds a steering+throttle command stamped with the current phase/stage.
        private RendezvousCommand Burn(Vector3d direction, double throttle, string status)
        {
            return new RendezvousCommand
            {
                Phase = Phase,
                Stage = Stage,
                HasBurn = true,
                ThrustDirection = direction.normalized,
                Throttle = MathHelpers.Clamp(throttle, 0.0, 1.0),
                Status = status
            };
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
