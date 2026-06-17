using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Terminal-rendezvous executor: the staged phase state machine, mirroring ClassicAscentGuidance's
    // plan/execute/cutoff structure, generalized to the intercept -> match-velocity -> close sequence.
    //
    // One user gate per stage: Execute() starts the current stage's closed loop; it runs automatically
    // and self-terminates on its cutoff condition. On completion the Stage advances and the phase drops
    // to Coast (so Stage always names the NEXT thing to Execute) — or Complete after the last stage.
    //
    // Step 4 wires the INTERCEPT stage: on the first executing tick it plans a fresh conic Lambert
    // intercept (InterceptSolver), freezes the world-frame ΔV, steers along it with throttle tapered
    // over the last few m/s, and cuts when the delivered ΔV along that vector reaches the planned
    // magnitude — ClassicAscentGuidance's node executor, Principia-safe. Attitude-before-throttle
    // (orient, then burn) is enforced by the actuation layer (RendezvousHandler), which holds throttle
    // until the craft is aligned. MatchVelocity/CloseApproach remain stubs (Steps 6/7).
    public sealed class TerminalRendezvousExecutor
    {
        // --- intercept tuning (public so callers/tests can adjust) -------------------------------
        public int InterceptArrivalSamples = 60;          // Lambert solves per plan (bounded)
        public double InterceptBudgetMilliseconds = 20.0; // wall-clock cap per plan
        public double InterceptTofMinFraction = 0.05;     // arrival sweep, as a fraction of the orbital period
        public double InterceptTofMaxFraction = 0.95;

        private const double MinUsefulDeltaV = 0.5;              // m/s; a plan below this is a no-op/degenerate
        private const double AlreadyCloseMeters = 2000.0;        // skip intercept only if genuinely this close
        private const double BurnTaperBandMetersPerSecond = 5.0; // throttle tapers over the last few m/s
        private const double BurnMinThrottle = 0.05;
        private const double CutoffEpsilonMetersPerSecond = 0.15; // complete when within this of planned ΔV
        private const double PeakDropMetersPerSecond = 0.5;       // complete if delivered falls back from its peak
        private const double BurnStallSeconds = 2.0;             // complete if delivered plateaus this long
        private const double BurnProgressThreshold = 1.0;        // m/s; only arm the stall timer once truly thrusting

        public RendezvousPhase Phase { get; private set; }
        public RendezvousStage Stage { get; private set; }
        public bool IsComplete => Phase == RendezvousPhase.Complete;

        // Latest intercept plan (a continuously-refreshed preview while idle/coast, or the frozen plan
        // while executing). For UI/logging.
        public bool HasInterceptPlan { get; private set; }
        public InterceptSolution InterceptPlan { get; private set; }

        // Burn execution state for the intercept stage.
        private bool _burnArmed;                   // burn baseline captured for the current Executing phase
        private Vector3d _burnStartVelocity;       // orbital velocity at ignition, for delivered-ΔV cutoff
        private double _plannedDvMagnitude;        // |ΔV| to deliver
        private Vector3d _plannedDvUnit;           // frozen world-frame burn direction
        private double _maxDeliveredDv;            // peak delivered-along-axis, to detect saturation/overshoot
        private double _lastProgressUt;            // UT of the last delivered-ΔV increase, for stall detection

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

        // User gate: start the current stage's closed loop. Valid from Idle (first stage) or Coast (the
        // queued next stage). Returns false when not in an executable state. The freshest plan is taken
        // on the first executing tick.
        public bool Execute()
        {
            if (Phase != RendezvousPhase.Idle && Phase != RendezvousPhase.Coast) return false;
            Phase = RendezvousPhase.Executing;
            _burnArmed = false;
            return true;
        }

        // Called by the actuation layer (RendezvousHandler) on every frame it is HOLDING throttle while
        // it orients/stabilizes the craft. It re-pins the delivered-ΔV baseline to the current velocity so
        // the cutoff measures only velocity change AFTER ignition — without this, gravity rotating the
        // velocity during a multi-second orient looks like "delivered ΔV" and can trip the cutoff before
        // the engine ever fires. No-op unless an intercept burn is armed.
        public void HoldBurnBaseline(Vector3d currentVelocity)
        {
            if (Phase != RendezvousPhase.Executing || Stage != RendezvousStage.Intercept || !_burnArmed) return;
            _burnStartVelocity = currentVelocity;
            _maxDeliveredDv = 0.0;
        }

        // User action: abort the sequence. No further commands are issued until Reset().
        public void Abort()
        {
            Phase = RendezvousPhase.Aborted;
            ClearInterceptState();
        }

        // Refreshes the cached plan for the CURRENT (not-yet-executed) stage so the UI can show its ΔV
        // and predicted closest approach before the user commits. No phase change. Called (throttled) by
        // the handler while idle/coasting.
        public void RefreshPlanPreview(IRendezvousWorld world)
        {
            if (world == null) return;
            if (Stage != RendezvousStage.Intercept) return;
            if (Phase != RendezvousPhase.Idle && Phase != RendezvousPhase.Coast) return;

            InterceptPlan = PlanIntercept(world);
            HasInterceptPlan = InterceptPlan.Success;
        }

        // Per-tick update. Runs the active stage while Executing (advancing the phase/stage on
        // completion); otherwise returns an idle (no-burn) command for the current state.
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

                case RendezvousPhase.Coast:
                    return Idle("coasting — Execute " + Stage);
                case RendezvousPhase.Complete:
                    return Idle("rendezvous complete — control handed back");
                case RendezvousPhase.Aborted:
                    return Idle("aborted");
                default: // Idle
                    return Idle("idle — Execute " + Stage);
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

                // A plan with no meaningful ΔV is either "already there" or a degenerate/no-op solution.
                // Only treat it as done when we're genuinely close; otherwise report rather than silently
                // completing (which would skip a burn that never happened and jump to Match Velocity).
                if (plan.DeltaVMagnitude <= MinUsefulDeltaV)
                {
                    double range = (world.TargetPosition - world.ActivePosition).magnitude;
                    if (range <= AlreadyCloseMeters)
                    {
                        stageComplete = true;
                        return Idle("intercept: already within close range");
                    }
                    return Idle("intercept: no useful burn found (too far / wrong geometry) - Abort or Reset");
                }

                _plannedDvMagnitude = plan.DeltaVMagnitude;
                _plannedDvUnit = plan.DeltaV.normalized;
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);
            if (delivered > _maxDeliveredDv)
            {
                _maxDeliveredDv = delivered;
                _lastProgressUt = world.UniversalTime;
            }

            // Three ways the burn finishes — all guarantee termination (no flooring at min throttle):
            //  1. reached: delivered is within an epsilon of planned.
            //  2. stalled: once truly thrusting, delivered stops climbing for a while (min throttle can't
            //     overcome gravity along the axis) — take what we got; the coast re-solve trims the rest.
            //  3. peaked: delivered fell back from its max (axis saturated / velocity rotated past it).
            if (delivered >= _plannedDvMagnitude - CutoffEpsilonMetersPerSecond)
            {
                stageComplete = true;
                return Idle("intercept: burn complete");
            }
            if (_maxDeliveredDv > BurnProgressThreshold && world.UniversalTime - _lastProgressUt > BurnStallSeconds)
            {
                stageComplete = true;
                return Idle(string.Format("intercept: cutoff (delivered stalled at {0:F1}/{1:F1} m/s)",
                    _maxDeliveredDv, _plannedDvMagnitude));
            }
            if (delivered < _maxDeliveredDv - PeakDropMetersPerSecond)
            {
                stageComplete = true;
                return Idle(string.Format("intercept: cutoff (delivered peaked at {0:F1} m/s)", _maxDeliveredDv));
            }

            double remaining = _plannedDvMagnitude - delivered;
            double throttle = MathHelpers.Clamp(remaining / BurnTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(_plannedDvUnit, throttle,
                string.Format("intercept burn {0:F1}/{1:F1} m/s", delivered, _plannedDvMagnitude));
        }

        // Plans a conic intercept from the current measured state. Target prediction is two-body
        // propagation of the measured target state (contract invariant 2). The arrival sweep spans a
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

        // Advances out of a finished Executing stage. Non-final stages advance Stage to the next one and
        // drop to Coast (so Stage names what to Execute next); the last stage (CloseApproach) Completes.
        private void CompleteStage()
        {
            ClearInterceptState();
            if (Stage == RendezvousStage.CloseApproach)
            {
                Phase = RendezvousPhase.Complete;
            }
            else
            {
                Stage = NextStage(Stage);
                Phase = RendezvousPhase.Coast;
            }
        }

        private void ClearInterceptState()
        {
            _burnArmed = false;
            _burnStartVelocity = Vector3d.zero;
            _plannedDvMagnitude = 0.0;
            _plannedDvUnit = Vector3d.zero;
            _maxDeliveredDv = 0.0;
            _lastProgressUt = 0.0;
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
