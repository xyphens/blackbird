using System;
using System.ComponentModel.Design;
using Blackbird.Docking;
using Blackbird.Mathematics;
using UnityEngine;
using VehiclePhysics;

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
        private const double MatchSteerLockMetersPerSecond = 0.5;          // below this, freeze the thrust direction

        // --- close-approach tuning --------------------------------------------------------------
        private double ClosestApproach = double.PositiveInfinity;          // live predicted CA (fed in by the handler)
        private double TimeToClosestApproach = double.PositiveInfinity;    // seconds until that predicted CA
        // We trust the predicted CA to trigger a COAST only when NEARBY: within a few km, two close craft share
        // their perturbations so the relative two-body CA barely drifts even under Principia, however far out in
        // time it is. Beyond that the projection can't be trusted (and the closing controller is the right tool).
        private const double CaTrustRangeMeters = 5000.0;     // only coast on a projected CA when within this range
        public const double ParkingDistanceDefaultMeters = 10.0;    // default park distance from the target
        // Settable park distance ("match velocities at X m" from the UI); set back to the default when the UI option is off
        public double ParkingDistance = ParkingDistanceDefaultMeters;
        public bool UseMatchVelocitiesDuringApproach = true;
        // Extra slack past the park distance allowed when declaring "parked". 0 = stop exactly at the desired
        // distance: any positive value just leaves us DesiredDistance + X short of where we asked to be.
        private const double ParkingDistanceBuffer = 0.0;
        private const double RendezParkedSpeedMetersPerSecond = 0.5; // ...and relative speed below this
        // Commanded closing speed = clamp(range * gain, 0, maxSpeed). Both are user-settable (UI) so a long-
        // range close can be flown as a few LARGE burns (raise maxSpeed) instead of a forever-crawl at the cap
        // that chips ~10 m per burn and dumps RCS. The auto-BRAKE point scales with the closing speed, so a
        // higher cap just means it accelerates, coasts, then brakes earlier — at the cost of looser precision.
        public const double RendezDistanceApproachGainDefault = 0.2;
        public double RendezDistanceApproachGain = RendezDistanceApproachGainDefault;             // closing speed per metre of range
        public const double RendezMaxApproachSpeedDefaultMetersPerSecond = 5.0;
        public double RendezMaxApproachSpeedMetersPerSecond = RendezMaxApproachSpeedDefaultMetersPerSecond;  // cap on commanded closing speed
        private const double RendezBurnDeadbandMetersPerSecond = 0.1;       // base (close-in) velocity-error deadband
        // ...relaxed with range: holding the closing velocity to 0.1 m/s at km distance just burns RCS fighting
        // orbital drift for no gain (the "micro-burn at 4 km" problem), so the deadband grows with distance up
        // to a cap and tightens back to the base value as we close in for precision.
        private const double RendezBurnDeadbandPerMeter = 0.0005;           // deadband added per metre of range
        private const double RendezBurnDeadbandMaxMetersPerSecond = 2.0;    // cap on the relaxed deadband
        private const double RendezThrottleTaperMetersPerSecond = 3.0;         // throttle tapers over this much velocity error
        // Braking lead: we must START the retro/match burn early enough to null the closing rate before the
        // parking distance, or we overshoot / collide. The distance needed = (coast while we flip to retro) +
        // (decelerate to a stop) = v*slewLead + v²/2a. Both inputs are vessel-specific (TWR + how far we must
        // rotate), so the handler sets them each tick; these are conservative offline/harness defaults.
        public double BrakingDecelMetersPerSecondSquared = 5.0;   // available deceleration (thrust / mass)
        public double BrakingSlewLeadSeconds = 3.0;               // time to flip to retrograde-relative before thrust

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

        // Burn execution state for the intercept stage (planned-ΔV guidance).
        private bool _burnArmed;                   // burn target captured for the current Executing phase
        private Vector3d _burnStartVelocity;       // velocity at ignition; delivered ΔV is measured from this
        private Vector3d _targetDepartureVelocity; // transfer V1; kept for the post-burn velocity-residual diagnostic
        private double _plannedDvMagnitude;        // |planned ΔV| — the amount to deliver along the burn axis
        private Vector3d _plannedDvUnit;           // planned ΔV direction — the (fixed) burn axis we steer along
        private double _maxDeliveredDv;            // peak delivered-along-axis, for the peak-drop cutoff
        private double _lastProgressDelivered;     // delivered at the last DEADBAND-sized gain, for stall detection
        private double _lastProgressUt;            // UT of that last meaningful gain, for stall detection

        // Burn execution state for the match-velocity stage.
        private bool _matchArmed;                  // match baseline captured for the current Executing phase
        private double _matchMinRelSpeed;          // smallest relative speed reached, for stall detection
        private double _matchLastProgressUt;       // UT of the last new minimum, for stall detection
        private Vector3d _matchSteerDirection;     // thrust direction, frozen once relative speed is small

        // Close-approach terminal brake: latched once we reach the (speed-dependent) brake point, so we keep
        // braking to a stop instead of popping back into the closing controller as the brake trigger shrinks.
        private bool _closeBraking;

        // Docking stage: the active gated leg (Approach -> Final -> Contact) and the live port transforms fed
        // in by the handler each tick. PortsValid is false until the operator has targeted a docking port and
        // is controlling from one of their own (the handler supplies them); StepDocking idles until then.
        private DockingLeg _dockingLeg;
        private bool _dockingPortsValid;
        private PortState _chaserPort;
        private PortState _targetPort;
        public DockingLeg DockingLeg => _dockingLeg;

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
            _dockingLeg = DockingLeg.Approach;
            _dockingPortsValid = false;
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
            _closeBraking = false;
            HasLastInterceptBurnReport = false;
            return true;
        }

        // User gate for the docking stage. Starting docking fresh (from any non-busy state) resets to the
        // first leg (Approach); resuming from Coast after a leg finished continues with the queued leg, so the
        // operator clicks once per gate (Approach -> Final -> Contact). Invalid mid-burn or while Aborted.
        public bool ExecuteDocking()
        {
            if (Phase == RendezvousPhase.Executing || Phase == RendezvousPhase.Aborted) return false;
            if (Stage == RendezvousStage.Docking && Phase == RendezvousPhase.Coast)
            {
                Phase = RendezvousPhase.Executing;   // resume the next queued leg
                return true;
            }
            Stage = RendezvousStage.Docking;
            _dockingLeg = DockingLeg.Approach;
            Phase = RendezvousPhase.Executing;
            ClearBurnState();
            return true;
        }

        // The handler feeds the live docking-port transforms each tick (world frame). When invalid (no target
        // port / not controlling from a port) the docking stage idles with guidance for the operator.
        public void SetDockingPorts(bool valid, PortState chaserPort, PortState targetPort)
        {
            _dockingPortsValid = valid;
            _chaserPort = chaserPort;
            _targetPort = targetPort;
        }

        // Called by the actuation layer (RendezvousHandler) on every frame it is HOLDING throttle while it
        // orients/stabilizes the craft. The real burn has not started yet, so re-pin the ignition velocity
        // (delivered is measured from it), the peak/progress trackers, AND the stall timer to NOW — otherwise
        // a multi-second orient would run the stall clock out, or the small velocity gravity adds during the
        // orient would count as delivered ΔV, and the burn would "complete" before the engine ever lit. No-op
        // unless an intercept burn is armed. (The planned-ΔV axis/magnitude are fixed and need no re-pinning.)
        public void HoldBurnBaseline(Vector3d currentVelocity, double ut)
        {
            if (Phase != RendezvousPhase.Executing || Stage != RendezvousStage.Intercept || !_burnArmed) return;
            // Re-pin the ignition velocity and the delivered/stall trackers to NOW: the real burn hasn't
            // started while we hold throttle to orient, so delivered (measured from _burnStartVelocity) stays
            // ~0 and the stall timer can't run out before the engine lights.
            _burnStartVelocity = currentVelocity;
            _maxDeliveredDv = 0.0;
            _lastProgressDelivered = 0.0;
            _lastProgressUt = ut;
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
        public RendezvousCommand Update(IRendezvousWorld world,
            double closestApproach = double.PositiveInfinity, double timeToClosestApproach = double.PositiveInfinity)
        {
            ClosestApproach = closestApproach;             // live predicted CA + time-to-CA, for the coast gate
            TimeToClosestApproach = timeToClosestApproach;

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
            if (Stage == RendezvousStage.Docking)
                return StepDocking(world, out stageComplete);

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

                // Steer along the PLANNED ΔV vector and deliver its magnitude — the impulsive node executor
                // (same as ClassicAscentGuidance). The plan already coasts to the estimated ignition state
                // (IgnitionLeadSeconds), so plan.DeltaV is the correct burn for where the engine actually
                // lights. NOTE: do NOT target "reach V1 from the current state" — |V1 − v| is inflated by the
                // velocity gravity adds over the ignition lead (tens of m/s), which made small corrections
                // over-burn 3-4x in the wrong direction and pushed CA back out (the "throttle-down raises CA"
                // symptom). The residual perpendicular gravity component is left for match velocity / a re-plan.
                _targetDepartureVelocity = plan.TransferDepartureVelocity;   // kept only for the velocity-residual diagnostic
                _plannedDvMagnitude = plan.DeltaVMagnitude;
                _plannedDvUnit = plan.DeltaV.normalized;
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            // Delivered ΔV ALONG the fixed planned-ΔV axis since ignition (orbital velocity, so it also counts
            // the small gravity contribution over the burn — like ClassicAscentGuidance). The axis is the
            // planned ΔV direction and never re-aims, so the craft holds attitude, delivered grows
            // monotonically toward the planned magnitude, and the cutoffs below fire cleanly.
            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);

            // Peak tracking (for the peak-drop cutoff): the highest delivered ΔV seen along the axis.
            if (delivered > _maxDeliveredDv) _maxDeliveredDv = delivered;

            // Progress tracking (for the stall cutoff) uses a DEADBAND: only a gain of at least
            // BurnStallProgressDeadband counts. A flooring engine adds a hair of delivered ΔV almost every
            // frame; without the deadband those microscopic new maxima would keep the stall timer alive
            // forever so it never tripped. Requiring a meaningful gain lets a slow crawl trip the stall.
            if (delivered > _lastProgressDelivered + BurnStallProgressDeadband)
            {
                _lastProgressDelivered = delivered;
                _lastProgressUt = world.UniversalTime;
            }

            // Three terminations, all guaranteeing the burn ends:
            //  1. reached: delivered is within an epsilon of the planned along-axis ΔV.
            //  2. stalled: once truly thrusting, delivered stops making meaningful progress for a while
            //     (min throttle can't overcome gravity along the axis) — take what we got; match / re-plan trims.
            //  3. peaked: delivered fell back from its max (axis saturated / velocity rotated past it).
            if (delivered >= _plannedDvMagnitude - CutoffEpsilonMetersPerSecond)
                return FinishInterceptBurn(world, delivered, "intercept: burn complete", out stageComplete);

            // Arm the stall only once genuinely thrusting; scaled down for small plans (a short re-plan when
            // already close) so a burn that hangs below 1 m/s can still stall out. Delivered is measured from
            // the (orient-repinned) ignition velocity, so the orient coast can't falsely arm this.
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
                string.Format("intercept burn: {0:F1}/{1:F1} m/s delivered", delivered, _plannedDvMagnitude));
        }

        // Completes the intercept burn: records the post-burn report (planned ΔV, actual delivered velocity
        // change, the velocity residual vs the target, and the plan's predicted CA for the actuation layer
        // to pair with the achieved CA) and returns the idle command.
        private RendezvousCommand FinishInterceptBurn(
            IRendezvousWorld world, double delivered, string status, out bool stageComplete)
        {
            stageComplete = true;
            Vector3d deliveredVector = world.ActiveVelocity - _burnStartVelocity;
            // True velocity error from the target departure velocity (the perpendicular gravity component the
            // frozen axis leaves behind) — for the POSTBURN diagnostic, distinct from delivered-along-axis.
            double velocityResidual = (_targetDepartureVelocity - world.ActiveVelocity).magnitude;
            _lastInterceptBurnReport = new InterceptBurnReport
            {
                PlannedDvMagnitude = _plannedDvMagnitude,
                PlannedDvVector = _plannedDvUnit * _plannedDvMagnitude,
                DeliveredAlongAxis = delivered,
                DeliveredVector = deliveredVector,
                VelocityResidual = velocityResidual,
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
                _matchSteerDirection = relSpeed > 1e-6 ? (-relVel).normalized : Vector3d.zero;
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

            // Burn opposite the relative velocity. Re-aim only while the relative speed is still large enough
            // that its DIRECTION is meaningful; below the lock threshold the near-zero relVel direction is
            // noise-dominated and swings (and an overshoot flips it ~180°), which would pivot the craft past
            // the orient gate and fire a needless SECOND burn. Once locked, hold the last good direction —
            // throttle still tapers on the (frame-independent) magnitude and the cutoff is magnitude-based, so
            // freezing the direction changes neither, it just stops the end-of-burn pivot.
            if (relSpeed > MatchSteerLockMetersPerSecond)
                _matchSteerDirection = (-relVel).normalized;
            Vector3d thrustDir = _matchSteerDirection != Vector3d.zero ? _matchSteerDirection : (-relVel).normalized;
            double throttle = MathHelpers.Clamp(relSpeed / MatchTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(thrustDir, throttle, string.Format("match velocity {0:F2} m/s remaining", relSpeed));
        }

        // Final-approach closed loop, in states ordered PARKED -> TERMINAL(coast/haven-burn) -> BRAKE -> CLOSE:
        //   PARKED (within the parking band AND matched): done — hand control back.
        //   TERMINAL (projected CA already <= band, nearby): our trajectory already reaches the band, so do NOT
        //     brake early or actively close — ride it in. Orient to the match attitude at ZERO throttle (in
        //     position, no slew left), and null the relative velocity only once we ARRIVE inside the parking
        //     band (the "safe haven" burn). Checked BEFORE brake so a low-TWR brake point can't fire a full
        //     match burn hundreds of metres short and wreck a good closest approach.
        //   BRAKE (CA not yet in band, within the speed-dependent brake point): kill the relative velocity so we
        //     stop by the parking distance. Brake point = ParkingDistance + (flip-to-retro coast) + (decel-to-
        //     stop), so a fast / low-TWR craft starts braking earlier. Latched once entered so the shrinking
        //     trigger can't bounce us back into CLOSE and start pushing closer again.
        //   CLOSE (otherwise): command a closing velocity toward the target (tapered to the parking distance,
        //     capped) and burn to match it, which also nulls lateral drift.
        private RendezvousCommand StepCloseApproach(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;   // chaser -> target
            double distanceToTarget = relPos.magnitude;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;   // chaser relative to target
            double relSpeed = relVel.magnitude;
            Vector3d bearing = distanceToTarget > 1e-6 ? relPos / distanceToTarget : Vector3d.zero;

            double parkedBand = ParkingDistance + ParkingDistanceBuffer;   // close enough to declare "parked"

            // PARKED: within the band AND matched -> done, hand control back.
            if (distanceToTarget <= parkedBand && relSpeed <= RendezParkedSpeedMetersPerSecond)
            {
                _closeBraking = false;
                stageComplete = true;
                return Idle(string.Format(
                    "close approach: parked at {0:F0} m ({1:F2} m/s) - control returned", distanceToTarget, relSpeed));
            }

            // Is our CURRENT trajectory already going to carry us inside the parking band? Trust the two-body
            // projection only when NEARBY (CaTrustRangeMeters): at a few km two close craft share their
            // perturbations, so the relative CA barely drifts even under Principia, however far out in TIME it is.
            bool caInBand = MathHelpers.IsFinite(ClosestApproach)
                && ClosestApproach <= parkedBand && distanceToTarget <= CaTrustRangeMeters;

            if (UseMatchVelocitiesDuringApproach)
            {
                // TERMINAL TRAJECTORY (CA already in band): do NOT brake early or actively close — ride the
                // trajectory in. ORIENT to the match attitude (anti relative-velocity) at ZERO throttle so the
                // craft is already in position with no slew left to do, and null the relative velocity only once
                // we ARRIVE at the safe haven (inside the parking band). caInBand guarantees the closest approach
                // is <= the band, so coasting always reaches the band — "arrived" needs no separate CA test.
                //
                // THIS IS THE FIX for the early-burn bug: the brake point below = ParkingDistance + (flip coast)
                // + (decel-to-stop) balloons on a low-TWR craft, so BRAKE used to latch hundreds of metres out
                // and fire a full match burn that stopped the craft dead far short of the target, wrecking a good
                // 8 m closest approach. Orient early = good; burn early = bad. (Checked before BRAKE so it wins.)
                if (caInBand)
                {
                    if (_closeBraking || distanceToTarget <= parkedBand)
                    {
                        _closeBraking = true;   // latch: keep nulling to a stop, don't bounce back out
                        return StepMatchVelocity(world, out stageComplete);
                    }
                    Vector3d holdDir = relSpeed > 1e-6 ? (-relVel).normalized : bearing;
                    return Burn(holdDir, 0.0, string.Format(
                        "close approach: terminal trajectory ({0:F0} m, CA {1:F0} m in band) - oriented, holding for the safe-haven burn",
                        distanceToTarget, ClosestApproach));
                }

                // CA NOT yet in band: actively close (CLOSE controller below) and brake to a stop at the parking
                // distance. The brake point = ParkingDistance + (flip-to-retro coast) + (decel-to-stop), so a
                // fast / low-TWR craft starts braking earlier; latched once entered so the shrinking trigger
                // can't bounce us back into the closing controller.
                double closingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));   // toward-target component
                double brakingDistance = closingSpeed * BrakingSlewLeadSeconds
                    + closingSpeed * closingSpeed / (2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared));
                double brakeTrigger = ParkingDistance + brakingDistance;
                if (distanceToTarget <= brakeTrigger) _closeBraking = true;
                if (_closeBraking)
                    return StepMatchVelocity(world, out stageComplete);
            }
            else if (caInBand)
            {
                // Match-velocities OFF ("close as possible / impact"): keep the guard so the closing controller
                // doesn't fire a pursuit burn that wrecks an already-good incoming trajectory — just hold heading.
                return Burn(bearing, 0.0, string.Format(
                    "close approach: coasting to terminal ({0:F0} m, CA {1:F0} m already in band)",
                    distanceToTarget, ClosestApproach));
            }

            // CLOSE: command a closing speed toward the target, tapered to zero at the parking distance and
            // capped; burn to null the gap between that and our actual relative velocity (also kills lateral drift).
            double remainingDistance = distanceToTarget - ParkingDistance;
            double commandedClosingSpeed = MathHelpers.Clamp(
                remainingDistance * RendezDistanceApproachGain, 0.0, RendezMaxApproachSpeedMetersPerSecond);

            Vector3d desiredRelVel = bearing * commandedClosingSpeed;
            Vector3d commandedVelocityGap = desiredRelVel - relVel;
            double closingSpeedVelocityGap = commandedVelocityGap.magnitude;

            // Within the deadband: hold heading at zero throttle instead of going Idle (Idle would make the
            // actuation layer release control and reset the orient gate — the cause of the stop-start micro-
            // burns); this keeps us pointed so the next correction fires without re-orienting. The deadband is
            // relaxed with range so we don't micro-correct the closing velocity to 0.1 m/s when km out.
            double velocityDeadband = MathHelpers.Clamp(distanceToTarget * RendezBurnDeadbandPerMeter,
                RendezBurnDeadbandMetersPerSecond, RendezBurnDeadbandMaxMetersPerSecond);
            if (closingSpeedVelocityGap <= velocityDeadband)
                return Burn(bearing, 0.0,
                    string.Format("close approach: holding at {0:F0} m ({1:F2} m/s)", distanceToTarget, relSpeed));

            double throttle = MathHelpers.Clamp(closingSpeedVelocityGap / RendezThrottleTaperMetersPerSecond, BurnMinThrottle, 1.0);
            double actualClosingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));   // toward-target component, for status
            return Burn(commandedVelocityGap, throttle, string.Format(
                "close approach: {0:F0} m, closing {1:F2}/{2:F2} m/s", distanceToTarget, actualClosingSpeed, commandedClosingSpeed));
        }

        // Docking stage closed loop: hold the head-on mated heading and RCS-translate the chaser port onto the
        // target port axis, leg by leg (Approach -> Final -> Contact). The translation/alignment math is the
        // pure DockingController; this wires it to the gated-leg state machine. Idles with operator guidance
        // until the handler reports valid port transforms.
        private RendezvousCommand StepDocking(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;
            if (!_dockingPortsValid)
                return Idle("docking: target a docking port and 'Control From Here' on yours");

            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;   // chaser relative to target
            DockingCommand dc = DockingController.Compute(_chaserPort, _targetPort, relVel, _dockingLeg);

            if (dc.LegComplete)
            {
                stageComplete = true;   // CompleteStage advances the leg (or finishes docking at Contact)
                return Idle(dc.Status + " — leg complete");
            }

            return DockCommand(dc);
        }

        // Builds a docking command: hold attitude along the mated heading (no main engine) and request the
        // RCS translation velocity. Throttle stays 0 — docking is RCS-only.
        private RendezvousCommand DockCommand(DockingCommand dc)
        {
            return new RendezvousCommand
            {
                Phase = Phase,
                Stage = Stage,
                HasBurn = true,                        // attitude is actively driven (to the mated heading)
                ThrustDirection = dc.FacingWorld.normalized,
                Throttle = 0.0,                        // RCS-only; no main engine
                HasTranslation = true,
                TranslationVelocityWorld = dc.TranslationVelocityWorld,
                Status = dc.Status
            };
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
            if (Stage == RendezvousStage.Docking)
            {
                // Docking advances by LEG, not by stage: each finished leg coasts awaiting the next gate;
                // Contact (the last leg) ends the sequence.
                if (_dockingLeg == DockingLeg.Contact)
                {
                    Phase = RendezvousPhase.Complete;
                }
                else
                {
                    _dockingLeg = DockingController.NextLeg(_dockingLeg);
                    Phase = RendezvousPhase.Coast;
                }
            }
            else if (Stage == RendezvousStage.CloseApproach)
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
            _targetDepartureVelocity = Vector3d.zero;
            _plannedDvMagnitude = 0.0;
            _plannedDvUnit = Vector3d.zero;
            _maxDeliveredDv = 0.0;
            _lastProgressDelivered = 0.0;
            _lastProgressUt = 0.0;

            _matchArmed = false;
            _matchMinRelSpeed = 0.0;
            _matchLastProgressUt = 0.0;
            _matchSteerDirection = Vector3d.zero;

            _closeBraking = false;
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
