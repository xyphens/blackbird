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
    // until the craft is aligned.
    //
    // Step 6 wires the MATCH-VELOCITY stage: at/near closest approach it cancels the chaser's velocity
    // relative to the target by burning opposite the relative-velocity vector, re-aimed every tick (not a
    // frozen axis), and cuts when the relative speed is essentially nulled.
    //
    // Step 7 wires the CLOSE-APPROACH stage (terminal goal): a closing-velocity controller that tracks a
    // commanded approach speed along the line of sight (tapered to zero as the range nears a standoff
    // distance), nulling the velocity error each tick so lateral drift is corrected too. It completes —
    // ending the whole sequence and handing control back — once parked within the standoff band and
    // matched.
    public sealed class TerminalRendezvousExecutor
    {
        // --- intercept tuning (public so callers/tests can adjust) -------------------------------
        public int InterceptArrivalSamples = 60;          // Lambert solves per plan (bounded)
        public double InterceptBudgetMilliseconds = 20.0; // wall-clock cap per plan
        public double InterceptTofMinFraction = 0.05;     // arrival sweep, as a fraction of the orbital period
        public double InterceptTofMaxFraction = 0.95;

        // Ignition-time-drift correction (set by the actuation layer to the estimated orient time): the
        // plan is solved from the active state coasted forward by this much, so the frozen ΔV matches the
        // state the engine actually fires from. 0 = plan from the measured state (offline/harness default).
        public double IgnitionLeadSeconds = 0.0;
        private const double IgnitionLeadMaxSeconds = 300.0;   // cap the forward coast at 5 min

        private const double MinUsefulDeltaV = 0.5;              // m/s; a plan below this is a no-op/degenerate
        private const double AlreadyCloseMeters = 2000.0;        // skip intercept only if genuinely this close
        private const double BurnTaperBandMetersPerSecond = 5.0; // throttle tapers over the last few m/s
        private const double BurnMinThrottle = 0.05;
        private const double CutoffEpsilonMetersPerSecond = 0.15; // complete when within this of planned ΔV
        private const double PeakDropMetersPerSecond = 0.5;       // complete if delivered falls back from its peak
        private const double BurnStallSeconds = 2.0;             // complete if delivered plateaus this long
        private const double BurnProgressThreshold = 1.0;        // m/s; only arm the stall timer once truly thrusting
        private const double BurnStallProgressDeadband = 0.2;    // min delivered gain that counts as real progress

        // --- match-velocity tuning ---------------------------------------------------------------
        private const double MatchVelocityToleranceMetersPerSecond = 0.15; // nulled when rel speed within this
        private const double MatchTaperBandMetersPerSecond = 3.0;          // throttle tapers over the last few m/s
        private const double MatchStallSeconds = 2.0;                      // cut if rel speed stops dropping...
        private const double MatchStallSpeedFloor = 1.0;                   // ...and is already near nulled

        // --- close-approach tuning --------------------------------------------------------------
        private const double CloseStandoffMeters = 100.0;          // park ~this far from the target
        private const double CloseStandoffBandMeters = 30.0;       // complete once within standoff + this
        private const double CloseMatchedSpeedMetersPerSecond = 0.5; // ...and relative speed below this
        private const double CloseApproachGain = 0.2;              // commanded closing speed per metre of range error (1/s)
        private const double CloseMaxApproachSpeedMetersPerSecond = 5.0;  // cap on commanded closing speed (braking margin)
        private const double CloseVelDeadbandMetersPerSecond = 0.1;       // no burn within this velocity error
        private const double CloseTaperBandMetersPerSecond = 3.0;         // throttle tapers over this much velocity error

        public RendezvousPhase Phase { get; private set; }
        public RendezvousStage Stage { get; private set; }
        public bool IsComplete => Phase == RendezvousPhase.Complete;

        // Latest intercept plan (a continuously-refreshed preview while idle/coast, or the frozen plan
        // while executing). For UI/logging.
        public bool HasInterceptPlan { get; private set; }
        public InterceptSolution InterceptPlan { get; private set; }

        // Post-burn report from the last completed intercept burn (for the actuation layer's diagnostic).
        // Set at cutoff; survives the stage transition; cleared on Reset/Execute.
        public bool HasLastInterceptBurnReport { get; private set; }
        public InterceptBurnReport LastInterceptBurnReport { get { return _lastInterceptBurnReport; } }
        private InterceptBurnReport _lastInterceptBurnReport;

        // Burn execution state for the intercept stage.
        private bool _burnArmed;                   // burn baseline captured for the current Executing phase
        private Vector3d _burnStartVelocity;       // orbital velocity at ignition, for delivered-ΔV cutoff
        private double _plannedDvMagnitude;        // |ΔV| to deliver
        private Vector3d _plannedDvUnit;           // frozen world-frame burn direction
        private double _maxDeliveredDv;            // peak delivered-along-axis, to detect saturation/overshoot
        private double _lastProgressDelivered;     // delivered at the last DEADBAND-sized gain, for stall detection
        private double _lastProgressUt;            // UT of that last meaningful gain, for stall detection

        // Burn execution state for the match-velocity stage.
        private bool _matchArmed;                  // match baseline captured for the current Executing phase
        private double _matchMinRelSpeed;          // smallest relative speed reached, for stall detection
        private double _matchLastProgressUt;       // UT of the last new minimum, for stall detection

        public TerminalRendezvousExecutor()
        {
            Reset();
        }

        // Returns to the initial Idle state at the first stage and clears any cached plan/burn state.
        public void Reset()
        {
            Phase = RendezvousPhase.Idle;
            Stage = RendezvousStage.Intercept;
            ClearBurnState();
            HasInterceptPlan = false;
            InterceptPlan = default(InterceptSolution);
            HasLastInterceptBurnReport = false;
            _lastInterceptBurnReport = default(InterceptBurnReport);
        }

        // User gate: start the current stage's closed loop. Valid from Idle (first stage) or Coast (the
        // queued next stage). Returns false when not in an executable state. The freshest plan is taken
        // on the first executing tick.
        public bool Execute()
        {
            if (Phase != RendezvousPhase.Idle && Phase != RendezvousPhase.Coast) return false;
            Phase = RendezvousPhase.Executing;
            _burnArmed = false;
            _matchArmed = false;
            HasLastInterceptBurnReport = false;   // a new burn invalidates the previous report
            return true;
        }

        // User gate: execute a SPECIFIC stage right now, regardless of the ordered flow — so the operator
        // can jump straight to Match Velocity (e.g. to kill a dangerous closing rate) or re-run any stage.
        // Valid from any state except mid-burn (Executing) or Aborted (which requires Reset first).
        public bool Execute(RendezvousStage stage)
        {
            if (Phase == RendezvousPhase.Executing || Phase == RendezvousPhase.Aborted) return false;
            Stage = stage;
            Phase = RendezvousPhase.Executing;
            _burnArmed = false;
            _matchArmed = false;
            HasLastInterceptBurnReport = false;
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
            _lastProgressDelivered = 0.0;
        }

        // User action: abort the sequence. No further commands are issued until Reset().
        public void Abort()
        {
            Phase = RendezvousPhase.Aborted;
            ClearBurnState();
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

        // Dispatches one tick to the active stage. All three stages are wired: Intercept (Step 4),
        // MatchVelocity (Step 6), CloseApproach (Step 7).
        private RendezvousCommand StepStage(IRendezvousWorld world, out bool stageComplete)
        {
            if (Stage == RendezvousStage.Intercept)
                return StepIntercept(world, out stageComplete);
            if (Stage == RendezvousStage.MatchVelocity)
                return StepMatchVelocity(world, out stageComplete);
            if (Stage == RendezvousStage.CloseApproach)
                return StepCloseApproach(world, out stageComplete);

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
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);

            // Peak tracking (for the peak-drop cutoff): the highest delivered ΔV seen along the axis.
            if (delivered > _maxDeliveredDv) _maxDeliveredDv = delivered;

            // Progress tracking (for the stall cutoff) uses a DEADBAND: only a gain of at least
            // BurnStallProgressDeadband since the last mark counts as progress. THIS is the fix for the
            // "stuck at min throttle forever" bug — a flooring engine adds a hair of delivered ΔV almost
            // every frame (a microscopic new maximum), which under the old "new max resets the timer"
            // rule kept the stall timer alive indefinitely so it never tripped. Requiring a meaningful
            // gain lets a slow crawl (gravity along the axis ≈ min thrust) trip the stall instead.
            if (delivered > _lastProgressDelivered + BurnStallProgressDeadband)
            {
                _lastProgressDelivered = delivered;
                _lastProgressUt = world.UniversalTime;
            }

            // Three ways the burn finishes — all guarantee termination (no flooring at min throttle):
            //  1. reached: delivered is within an epsilon of planned.
            //  2. stalled: once truly thrusting, delivered stops making meaningful progress for a while
            //     (min throttle can't overcome gravity along the axis) — take what we got; the coast
            //     re-solve / match-velocity stage trims the rest.
            //  3. peaked: delivered fell back from its max (axis saturated / velocity rotated past it).
            if (delivered >= _plannedDvMagnitude - CutoffEpsilonMetersPerSecond)
                return FinishInterceptBurn(world, delivered, "intercept: burn complete", out stageComplete);

            // Arm the stall once we are genuinely thrusting; scaled down for small plans so a burn that
            // hangs below 1 m/s (a short re-plan when already close) can still stall out.
            double stallArmThreshold = Math.Min(BurnProgressThreshold, 0.4 * _plannedDvMagnitude);
            if (_maxDeliveredDv > stallArmThreshold && world.UniversalTime - _lastProgressUt > BurnStallSeconds)
                return FinishInterceptBurn(world, delivered, string.Format(
                    "intercept: cutoff (delivered stalled at {0:F1}/{1:F1} m/s)",
                    _maxDeliveredDv, _plannedDvMagnitude), out stageComplete);

            if (delivered < _maxDeliveredDv - PeakDropMetersPerSecond)
                return FinishInterceptBurn(world, delivered, string.Format(
                    "intercept: cutoff (delivered peaked at {0:F1} m/s)", _maxDeliveredDv), out stageComplete);

            double remaining = _plannedDvMagnitude - delivered;
            double throttle = MathHelpers.Clamp(remaining / BurnTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(_plannedDvUnit, throttle,
                string.Format("intercept burn {0:F1}/{1:F1} m/s", delivered, _plannedDvMagnitude));
        }

        // Completes the intercept burn: records the post-burn report (planned vs delivered ΔV + the plan's
        // predicted CA, for the actuation layer to pair with the achieved CA) and returns the idle command.
        private RendezvousCommand FinishInterceptBurn(
            IRendezvousWorld world, double delivered, string status, out bool stageComplete)
        {
            stageComplete = true;
            _lastInterceptBurnReport = new InterceptBurnReport
            {
                PlannedDvMagnitude = _plannedDvMagnitude,
                PlannedDvVector = _plannedDvUnit * _plannedDvMagnitude,
                DeliveredAlongAxis = delivered,
                DeliveredVector = world.ActiveVelocity - _burnStartVelocity,
                PredictedClosestApproach = InterceptPlan.PredictedClosestApproach,
                CutoffReason = status
            };
            HasLastInterceptBurnReport = true;
            return Idle(status);
        }

        // Match-velocity stage closed loop: at/near closest approach, cancel the chaser's velocity
        // relative to the target. Unlike the intercept (a frozen-axis burn), the thrust direction is
        // re-aimed every tick straight opposite the CURRENT relative velocity, so residual components that
        // shift as the burn proceeds are nulled too. The cutoff is frame-independent — it uses relative
        // speed directly, with no delivered-ΔV baseline needed, because this close gravity acts on both
        // craft nearly identically (so it cancels out of the relative velocity).
        private RendezvousCommand StepMatchVelocity(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

            // Our velocity relative to the target; nulling this leaves us station-keeping alongside it.
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            double relSpeed = relVel.magnitude;

            if (!_matchArmed)
            {
                _matchMinRelSpeed = relSpeed;
                _matchLastProgressUt = world.UniversalTime;
                _matchArmed = true;
            }
            if (relSpeed < _matchMinRelSpeed)
            {
                _matchMinRelSpeed = relSpeed;
                _matchLastProgressUt = world.UniversalTime;
            }

            // Done when relative velocity is essentially nulled.
            if (relSpeed <= MatchVelocityToleranceMetersPerSecond)
            {
                stageComplete = true;
                return Idle(string.Format("match velocity: nulled ({0:F2} m/s)", relSpeed));
            }

            // Stall guard: once already near nulled, if relative speed stops dropping for a while the min
            // throttle / engine floor can't do better — accept it (the close stage trims the remainder).
            // Gated below a small floor so it never fires during the initial orient (when the handler
            // holds throttle and relative speed is still large and unchanging).
            if (relSpeed <= MatchStallSpeedFloor
                && world.UniversalTime - _matchLastProgressUt > MatchStallSeconds)
            {
                stageComplete = true;
                return Idle(string.Format("match velocity: cutoff (stalled at {0:F2} m/s)", relSpeed));
            }

            // Burn opposite the relative velocity, re-aimed each tick; taper throttle over the last few m/s.
            Vector3d thrustDir = (-relVel).normalized;
            double throttle = MathHelpers.Clamp(relSpeed / MatchTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(thrustDir, throttle, string.Format("match velocity {0:F2} m/s remaining", relSpeed));
        }

        // Close-approach stage closed loop (terminal goal): bring the chaser in to a standoff distance
        // (~100 m) of the target with the relative velocity controlled, then hand control back.
        //
        // It is a closing-velocity controller, NOT an open-loop burn: each tick it commands a desired
        // relative velocity that points straight up the line of sight (chaser -> target) at a speed which
        // tapers linearly to zero as the range approaches the standoff distance (and is capped). Thrust
        // nulls the error between that commanded velocity and the actual relative velocity — which also
        // cancels any lateral drift, since the command has no lateral component. As the craft overshoots
        // from closing to braking the error vector flips, so the actuation layer naturally flies the
        // accelerate -> coast -> flip -> brake profile. Completes (ending the sequence) once parked inside
        // the standoff band and matched. Stateless and frame-independent, like the match-velocity stage.
        private RendezvousCommand StepCloseApproach(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;   // chaser -> target
            double range = relPos.magnitude;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;   // chaser relative to target
            double relSpeed = relVel.magnitude;

            // Done when parked within the standoff band and nearly matched — hand control back.
            if (range <= CloseStandoffMeters + CloseStandoffBandMeters
                && relSpeed <= CloseMatchedSpeedMetersPerSecond)
            {
                stageComplete = true;
                return Idle(string.Format(
                    "close approach: parked at {0:F0} m ({1:F2} m/s) - control returned", range, relSpeed));
            }

            // Commanded closing speed: tapers to zero as range nears the standoff distance, capped above.
            // Inside the standoff (negative error) it clamps to zero, so the controller just holds station
            // rather than pushing closer.
            Vector3d losUnit = range > 1e-6 ? relPos / range : Vector3d.zero;
            double rangeError = range - CloseStandoffMeters;
            double commandedClosingSpeed = MathHelpers.Clamp(
                rangeError * CloseApproachGain, 0.0, CloseMaxApproachSpeedMetersPerSecond);

            Vector3d desiredRelVel = losUnit * commandedClosingSpeed;
            Vector3d velError = desiredRelVel - relVel;
            double errMag = velError.magnitude;

            // Within the deadband, HOLD attitude along the line of sight at zero throttle (a burn command
            // with 0 throttle) rather than going Idle. Returning Idle would make the actuation layer treat
            // it as "release control" and reset the orient gate, so every tiny correction re-ran the full
            // orient+settle dwell — the cause of the many stop-start micro-burns on approach. Holding keeps
            // the craft pointed and settled so the next correction fires immediately.
            if (errMag <= CloseVelDeadbandMetersPerSecond)
                return Burn(losUnit, 0.0,
                    string.Format("close approach: holding at {0:F0} m ({1:F2} m/s)", range, relSpeed));

            double closingSpeed = Vector3d.Dot(relVel, losUnit);   // actual (positive = closing), for status
            double throttle = MathHelpers.Clamp(errMag / CloseTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(velError, throttle, string.Format(
                "close approach: {0:F0} m, closing {1:F2}/{2:F2} m/s", range, closingSpeed, commandedClosingSpeed));
        }

        // Plans a conic intercept from the current measured state. Target prediction is two-body
        // propagation of the measured target state (contract invariant 2). The arrival sweep spans a
        // fraction of the active orbit's period.
        private InterceptSolution PlanIntercept(IRendezvousWorld world)
        {
            double mu = world.Mu;
            double measureUt = world.UniversalTime;

            // Ignition-time-drift correction: the burn doesn't start until the craft has oriented, so plan
            // from the active state COASTED FORWARD to the estimated ignition time, and reference the
            // arrival sweep to that ignition. The frozen ΔV then matches the state the engine actually
            // fires from rather than the state measured (seconds-to-minutes) earlier. Lead is supplied by
            // the actuation layer (estimated slew time); 0 offline (no orient delay).
            double lead = MathHelpers.Clamp(IgnitionLeadSeconds, 0.0, IgnitionLeadMaxSeconds);
            double ignitionUt = measureUt + lead;

            Vector3d ignitionPos, ignitionVel;
            if (lead > 0.0)
                TwoBody.Propagate(world.ActivePosition, world.ActiveVelocity, mu, lead,
                    out ignitionPos, out ignitionVel);
            else
            {
                ignitionPos = world.ActivePosition;
                ignitionVel = world.ActiveVelocity;
            }

            double period = OrbitalPeriod(ignitionPos, ignitionVel, mu);
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

            Vector3d targetPosition = world.TargetPosition;
            Vector3d targetVelocity = world.TargetVelocity;

            // Target prediction propagates from the MEASUREMENT epoch, so absolute arrival UTs (which are
            // >= ignitionUt) map to the correct elapsed time.
            Func<double, Vector3d> targetPositionAt = ut =>
            {
                TwoBody.Propagate(targetPosition, targetVelocity, mu, ut - measureUt, out Vector3d rt, out _);
                return rt;
            };

            return InterceptSolver.Solve(ignitionPos, ignitionVel, mu,
                world.ReferenceNormal, ignitionUt, targetPositionAt, tofMin, tofMax,
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
            ClearBurnState();
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

        private void ClearBurnState()
        {
            _burnArmed = false;
            _burnStartVelocity = Vector3d.zero;
            _plannedDvMagnitude = 0.0;
            _plannedDvUnit = Vector3d.zero;
            _maxDeliveredDv = 0.0;
            _lastProgressDelivered = 0.0;
            _lastProgressUt = 0.0;

            _matchArmed = false;
            _matchMinRelSpeed = 0.0;
            _matchLastProgressUt = 0.0;
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
