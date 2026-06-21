using System;
using Blackbird.Mathematics;
using Blackbird.Modules;

namespace Blackbird.Rendezvous
{
    // Rendezvous execution state machine. Each method (Intercept / MatchVelocity / CloseApproach) runs
    // independently via Execute(method): a closed loop that self-terminates on its cutoff, then drops to
    // Coast (Complete after CloseApproach). Burns are a steering direction + throttle (the actuation layer
    // orients then throttles); no patched-conic nodes, so it is Principia-safe. Docking is a separate module.
    public sealed class TerminalRendezvousExecutor
    {
        // --- intercept tuning (public so callers/tests can adjust) ---
        public int InterceptArrivalSamples = 60;          // Lambert solves per plan
        public double InterceptBudgetMilliseconds = 20.0; // wall-clock cap per plan
        public double InterceptTofMinFraction = 0.05;     // arrival sweep, fraction of the orbital period
        public double InterceptTofMaxFraction = 0.95;

        // Plan from the active state coasted forward by this much, so the frozen ΔV matches the state the
        // engine actually fires from after the orient delay. Set by the actuation layer; 0 offline.
        public double IgnitionLeadSeconds = 0.0;
        private const double IgnitionLeadMaxSeconds = 300.0;

        private const double MinUsefulDeltaV = 0.5;              // a plan below this is a no-op/degenerate
        private const double AlreadyCloseMeters = 2000.0;        // only skip intercept if genuinely this close
        private const double BurnTaperBandMetersPerSecond = 5.0; // throttle tapers over the last few m/s
        private const double BurnMinThrottle = 0.05;
        private const double CutoffEpsilonMetersPerSecond = 0.15; // done within this of planned ΔV
        private const double PeakDropMetersPerSecond = 0.5;       // done if delivered falls back from its peak
        private const double BurnStallSeconds = 2.0;             // done if delivered plateaus this long
        private const double BurnProgressThreshold = 1.0;        // only arm the stall timer once truly thrusting
        private const double BurnStallProgressDeadband = 0.2;    // min delivered gain that counts as progress

        // --- match-velocity tuning ---
        private const double MatchVelocityToleranceMetersPerSecond = 0.15; // nulled within this
        private const double MatchTaperBandMetersPerSecond = 3.0;
        private const double MatchStallSeconds = 2.0;                      // cut if rel speed stops dropping...
        private const double MatchStallSpeedFloor = 1.0;                   // ...and is already near nulled

        // --- close-approach tuning ---
        private double ClosestApproach = double.PositiveInfinity;   // live predicted CA, fed in by the handler
        private const double CaTrustRangeMeters = 5000.0;          // only trust the projected CA within this range
        public const double ParkingDistanceDefaultMeters = 10.0;
        public double ParkingDistance = ParkingDistanceDefaultMeters;   // "match velocities at X m" (UI)
        public bool UseDistanceForMatchVelocities = true;
        private const double ParkingDistanceBuffer = 0.0;          // extra slack when declaring "parked"
        private const double RendezParkedSpeedMetersPerSecond = 0.5;
        // Commanded closing speed = clamp(range * gain, 0, maxSpeed); both user-settable (UI).
        public const double RendezDistanceApproachGainDefault = 0.2;
        public double RendezDistanceApproachGain = RendezDistanceApproachGainDefault;
        public const double RendezMaxApproachSpeedDefaultMetersPerSecond = 5.0;
        public double RendezMaxApproachSpeedMetersPerSecond = RendezMaxApproachSpeedDefaultMetersPerSecond;
        // Velocity-error deadband, relaxed with range (holding 0.1 m/s at km distance just burns RCS fighting
        // orbital drift), tightening back to the base value close in.
        private const double RendezBurnDeadbandMetersPerSecond = 0.1;
        private const double RendezBurnDeadbandPerMeter = 0.0005;
        private const double RendezBurnDeadbandMaxMetersPerSecond = 2.0;
        private const double RendezThrottleTaperMetersPerSecond = 3.0;
        // Brake point = ParkingDistance + (flip-to-retro coast) + (decel-to-stop) = D + v*slewLead + v²/2a.
        // Both inputs are vessel-specific (TWR, slew time); the handler sets them each tick.
        public double BrakingDecelMetersPerSecondSquared = 5.0;
        public double BrakingSlewLeadSeconds = 3.0;

        // Latest plan: a live preview while idle/coast, or the frozen plan while executing.
        public bool HasInterceptPlan { get; private set; }

        // Post-burn report from the last intercept burn, for the actuation layer's diagnostic.
        public bool HasLastInterceptBurnReport { get; private set; }
        public InterceptBurnReport LastInterceptBurnReport { get { return _lastInterceptBurnReport; } }
        private InterceptBurnReport _lastInterceptBurnReport;

        // Intercept burn state.
        private bool _burnArmed;
        private Vector3d _burnStartVelocity;       // velocity at ignition; delivered ΔV is measured from this
        private Vector3d _targetDepartureVelocity; // transfer V1; for the post-burn velocity-residual diagnostic
        private double _plannedDvMagnitude;        // amount to deliver along the burn axis
        private Vector3d _plannedDvUnit;           // fixed burn axis we steer along
        private double _maxDeliveredDv;            // peak delivered-along-axis, for the peak-drop cutoff
        private double _lastProgressDelivered;     // delivered at the last meaningful gain, for stall detection
        private double _lastProgressUt;

        // Hohmann transfer (secondary planner). False = original single-rev intercept (PlanIntercept) for
        // preview and execution. The Hohmann picks a FUTURE departure, so the burn coasts to _plannedIgnitionUt.
        //public bool UseHohmannPlanner = false;
        private double _plannedIgnitionUt;
        private double _burnIgnitionUt;            // centered-burn start = planned ignition - half the burn duration
        private bool _hohmannPreviewComputed;      // compute the (costly) Hohmann preview once, then cache
        public double PlannedIgnitionUt => _plannedIgnitionUt;
        public bool BurnArmed => _burnArmed;

        // Ship acceleration (thrust/mass), fed by the actuation layer, used to center the intercept burn:
        // ignite half the burn duration before the planned ignition so the burn straddles that instant. 0 = unknown.
        public double BurnAccelMetersPerSecondSquared = 0.0;

        // Match-velocity burn state.
        private bool _matchArmed;
        private double _matchMinRelSpeed;          // smallest relative speed reached, for stall detection
        private double _matchLastProgressUt;
        private Vector3d _matchSteerDirection;     // thrust direction, frozen once relative speed is small

        // in range to start matching velocities
        private bool _closeBraking;

        SharedState bbState;

        public TerminalRendezvousExecutor() { }

        public void Init(SharedState s) => bbState = s;

        // Back to Idle at the first stage, clearing cached plan/burn state.
        public void Reset()
        {
            bbState.RendezvousMethod = RendezvousMethod.None;
            bbState.InterceptPhase = InterceptPhase.Idle;
            if (bbState.InterceptPhase == InterceptPhase.Complete) bbState.ActiveModule = BlackbirdModule.None;
            ClearBurnState();
            HasInterceptPlan = false;
            _burnArmed = false;
            _matchArmed = false;
            _closeBraking = false;
            bbState.InterceptSolution = default(InterceptSolution);
            HasLastInterceptBurnReport = false;
            _lastInterceptBurnReport = default(InterceptBurnReport);
        }

        // Start the current stage's loop. Valid from Idle (first stage) or Coast (the queued next stage).
        public bool Execute()
        {
            if (bbState.InterceptPhase != InterceptPhase.Idle && bbState.InterceptPhase != InterceptPhase.Coast) return false;
            bbState.InterceptPhase = InterceptPhase.Executing;
            bbState.ActiveModule = BlackbirdModule.Rendezvous;
            _burnArmed = false;
            _matchArmed = false;
            HasLastInterceptBurnReport = false;
            return true;
        }

        // cancel an existing method if active then run a new one
        public bool ForceExecute(RendezvousMethod method)
        {
            //if (bbState.InterceptPhase == InterceptPhase.Executing || bbState.InterceptPhase == InterceptPhase.Aborted) return false;
            bbState.RendezvousMethod = method;
            bbState.InterceptPhase = InterceptPhase.Executing;
            bbState.ActiveModule = BlackbirdModule.Rendezvous;

            _burnArmed = false;
            _matchArmed = false;
            _closeBraking = false;
            HasLastInterceptBurnReport = false;
            return true;
        }

        // Called every frame the actuation layer is HOLDING throttle to orient before an intercept burn:
        // re-pin the ignition velocity and the delivered/stall trackers to NOW, so the gravity gained while
        // orienting isn't counted as delivered ΔV and the stall timer can't run out before the engine lights.
        public void HoldBurnBaseline(Vector3d currentVelocity, double ut)
        {
            if (bbState.InterceptPhase != InterceptPhase.Executing || bbState.RendezvousMethod != RendezvousMethod.Intercept || !_burnArmed) return;
            _burnStartVelocity = currentVelocity;
            _maxDeliveredDv = 0.0;
            _lastProgressDelivered = 0.0;
            _lastProgressUt = ut;
        }

        // Abort: no further commands until Reset().
        public void Abort()
        {
            bbState.InterceptPhase = InterceptPhase.Aborted;
            if (bbState.InterceptPhase == InterceptPhase.Complete) bbState.ActiveModule = BlackbirdModule.None;
            ClearBurnState();
        }

        // Invalidate the cached Hohmann preview so the next RefreshPlanPreview solves once more.
        // todo: generate a few and let user pick
        public void RequestPlanRefresh() { _hohmannPreviewComputed = false; }

        // Refresh the cached plan for the current (not-yet-executed) stage so the UI can show ΔV / predicted
        // CA before the user commits. No phase change. The Hohmann path computes once and caches.
        public void RefreshPlanPreview(IRendezvousWorld world)
        {
            // Intercept is the only previewable plan, so compute it whenever idle/coast — regardless of the
            // currently-selected method (which is None until the user actually Executes a stage).
            if (world == null) return;
            if (bbState.InterceptPhase != InterceptPhase.Idle && bbState.InterceptPhase != InterceptPhase.Coast) return;

            if (bbState.InterceptMethod == InterceptMethod.Hohmann)
            {
                if (_hohmannPreviewComputed) return;
                _hohmannPreviewComputed = true;
                bbState.InterceptSolution = BuildHohmannPlan(world);
                HasInterceptPlan = bbState.InterceptSolution.Success;
                return;
            }

            bbState.InterceptSolution = PlanIntercept(world);
            HasInterceptPlan = bbState.InterceptSolution.Success;
        }

        // Intercept-shaped plan from the MJ-derived two-impulse Hohmann transfer. The state-vector core runs in
        // the world's KSP frame, so dv1 is already world-frame. Fail-soft: any throw / non-sane result => Success false.
        private InterceptSolution BuildHohmannPlan(IRendezvousWorld world)
        {
            try
            {
                (Vector3d dv1, double ut1, Vector3d dv2, double ut2) = OrbitMath.DeltaVForHohmannTransfer(
                    world.UniversalTime, world.ActivePosition, world.ActiveVelocity,
                    world.TargetPosition, world.TargetVelocity, world.Mu);

                double dvMag = dv1.magnitude;
                bool sane = ut1 > world.UniversalTime
                            && !double.IsNaN(dvMag) && !double.IsInfinity(dvMag) && dvMag < 1e8;

                return new InterceptSolution
                {
                    Success = sane,
                    Status = sane ? InterceptStatus.Ok : InterceptStatus.NoFeasibleSolution,
                    DeltaV = dv1,
                    DeltaVMagnitude = dvMag,
                    IgnitionUt = ut1,
                    ArrivalUt = ut2,
                    TimeOfFlight = ut2 - ut1,
                    PredictedClosestApproach = 0.0,                      // Hohmann arrives at the target by construction
                    TransferDepartureVelocity = world.ActiveVelocity + dv1,
                    TransferArrivalVelocity = Vector3d.zero,
                    SamplesEvaluated = 0
                };
            }
            catch
            {
                return default(InterceptSolution);
            }
        }

        // Per-tick update. Runs the active stage while Executing (advancing on completion); otherwise idle.
        public RendezvousCommand Update(IRendezvousWorld world,
            double closestApproach = double.PositiveInfinity, double timeToClosestApproach = double.PositiveInfinity)
        {
            ClosestApproach = closestApproach;

            switch (bbState.InterceptPhase)
            {
                case InterceptPhase.Executing:
                    if (world == null) return Idle("executing — no world state");
                    RendezvousCommand cmd = StepStage(world, out bool stageComplete);
                    if (stageComplete) CompleteStage();
                    cmd.Phase = bbState.InterceptPhase;
                    cmd.Method = bbState.RendezvousMethod;
                    return cmd;

                case InterceptPhase.Coast:
                    return Idle("coasting — Execute " + bbState.RendezvousMethod);
                case InterceptPhase.Complete:
                    return Idle("rendezvous complete — control handed back");
                case InterceptPhase.Aborted:
                    return Idle("aborted");
                default:
                    return Idle("idle — Execute " + bbState.RendezvousMethod);
            }
        }

        private RendezvousCommand StepStage(IRendezvousWorld world, out bool stageComplete)
        {
            
            // we no longer "auto-advance" to a next stage, just flag it as complete
            if (bbState.RendezvousMethod == RendezvousMethod.Intercept)
                return StepIntercept(world, out stageComplete);
            if (bbState.RendezvousMethod == RendezvousMethod.MatchVelocity)
                return UseDistanceForMatchVelocities 
                    ? StepCloseApproach(world, out stageComplete, true) // piggy-back off of CA's braking logic if not an instant command
                    : StepMatchVelocity(world, out stageComplete);
            if (bbState.RendezvousMethod == RendezvousMethod.CloseApproach)
                return StepCloseApproach(world, out stageComplete, false);

            stageComplete = true;
            return Idle("Idle");
        }

        // Intercept loop: freeze a fresh plan on entry, steer along the planned ΔV axis, cut when the velocity
        // change delivered ALONG that axis reaches the planned magnitude. Delivered uses orbital velocity, so it
        // counts the small gravity contribution over the burn; the perpendicular residual is left for match/re-plan.
        private RendezvousCommand StepIntercept(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;
            if (!_burnArmed)
            {
                // Hohmann: REUSE the cached preview plan so Execute fires at the same departure window the user previewed
                InterceptSolution solution = bbState.InterceptMethod == InterceptMethod.Hohmann
                    ? (HasInterceptPlan ? bbState.InterceptSolution : BuildHohmannPlan(world))
                    : PlanIntercept(world);
                bbState.InterceptSolution = solution;
                HasInterceptPlan = solution.Success;

                if (!solution.Success) return Idle("intercept: no feasible plan (" + solution.Status + ")");

                // No-ΔV plan: done if we're genuinely close, otherwise report rather than silently completing
                if (solution.DeltaVMagnitude <= MinUsefulDeltaV)
                {
                    double range = (world.TargetPosition - world.ActivePosition).magnitude;
                    if (range <= AlreadyCloseMeters)
                    {
                        stageComplete = true;
                        return Idle("intercept: already within close range");
                    }
                    return Idle("intercept: no useful burn found (too far / wrong geometry) - Abort or Reset");
                }

                // Steer along the planned ΔV and deliver its magnitude — NOT "reach V1 from the current state"
                // (|V1 − v| is inflated by the ignition-lead gravity, which over-burns small corrections).
                _targetDepartureVelocity = solution.TransferDepartureVelocity;
                _plannedDvMagnitude = solution.DeltaVMagnitude;
                _plannedDvUnit = solution.DeltaV.normalized;
                _plannedIgnitionUt = solution.IgnitionUt;
                double halfBurn = BurnAccelMetersPerSecondSquared > 0.0
                    ? 0.5 * _plannedDvMagnitude / BurnAccelMetersPerSecondSquared : 0.0;
                _burnIgnitionUt = _plannedIgnitionUt - halfBurn;
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            // Hohmann departs in the future: pre-orient during the coast
            if (bbState.InterceptMethod == InterceptMethod.Hohmann && world.UniversalTime < _burnIgnitionUt)
            {
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                return Burn(_plannedDvUnit, 0.0, "intercept: orienting, ignition in "
                    + (_burnIgnitionUt - world.UniversalTime).ToString("F0") + "s");
            }

            // Delivered ΔV along the fixed planned-ΔV axis since ignition.
            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);
            if (delivered > _maxDeliveredDv) _maxDeliveredDv = delivered;

            // Progress for the stall cutoff uses a deadband: a flooring engine adds a hair of delivered ΔV
            // almost every frame, so only a gain of at least BurnStallProgressDeadband resets the stall timer.
            if (delivered > _lastProgressDelivered + BurnStallProgressDeadband)
            {
                _lastProgressDelivered = delivered;
                _lastProgressUt = world.UniversalTime;
            }

            // Three terminations: reached planned ΔV; stalled (no progress once thrusting); or peaked (fell back).
            if (delivered >= _plannedDvMagnitude - CutoffEpsilonMetersPerSecond)
                return FinishInterceptBurn(world, delivered, "intercept: burn complete", out stageComplete);

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

        // Records the post-burn report (planned vs delivered, the velocity residual, the plan's predicted CA).
        private RendezvousCommand FinishInterceptBurn(
            IRendezvousWorld world, double delivered, string status, out bool stageComplete)
        {
            stageComplete = true;
            Vector3d deliveredVector = world.ActiveVelocity - _burnStartVelocity;
            double velocityResidual = (_targetDepartureVelocity - world.ActiveVelocity).magnitude;
            _lastInterceptBurnReport = new InterceptBurnReport
            {
                PlannedDvMagnitude = _plannedDvMagnitude,
                PlannedDvVector = _plannedDvUnit * _plannedDvMagnitude,
                DeliveredAlongAxis = delivered,
                DeliveredVector = deliveredVector,
                VelocityResidual = velocityResidual,
                PredictedClosestApproach = bbState.InterceptSolution.PredictedClosestApproach,
                CutoffReason = status
            };
            HasLastInterceptBurnReport = true;
            return Idle(status);
        }

        // Match-velocity loop: cancel the chaser's velocity relative to the target, re-aiming opposite the
        // CURRENT relative velocity each tick. The cutoff is frame-independent (relative speed directly) since
        // this close gravity acts almost identically on both craft.
        private RendezvousCommand StepMatchVelocity(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

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

            if (relSpeed <= MatchVelocityToleranceMetersPerSecond)
            {
                // velocity already matched
                stageComplete = true;
                return Idle(string.Format("match velocity: nulled ({0:F2} m/s)", relSpeed));
            }

            // Stall guard: once near nulled, if relative speed stops dropping the engine floor can't do better;
            // accept it (close approach trims the rest). Floored so it can't fire during the initial orient.
            if (relSpeed <= MatchStallSpeedFloor && world.UniversalTime - _matchLastProgressUt > MatchStallSeconds)
            {
                stageComplete = true;
                return Idle(string.Format("match velocity: cutoff (stalled at {0:F2} m/s)", relSpeed));
            }

            // steer + burn in the vel null directory
            _matchSteerDirection = (-relVel).normalized;
            Vector3d thrustDir = _matchSteerDirection != Vector3d.zero ? _matchSteerDirection : (-relVel).normalized;
            double throttle = MathHelpers.Clamp(relSpeed / MatchTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(thrustDir, throttle, string.Format("match velocity {0:F2} m/s remaining", relSpeed));
        }

        // Final-approach loop, states ordered PARKED -> TERMINAL -> BRAKE -> CLOSE:
        //   PARKED   within the band and matched: done.
        //   TERMINAL projected CA already in band (nearby): ride it in — orient to the match attitude at zero
        //            throttle and null only on arrival inside the band. Checked before BRAKE so a low-TWR brake
        //            point can't fire a full match burn hundreds of metres short and wreck a good CA.
        //   BRAKE    within the speed-dependent brake point: null the relative velocity to stop by the parking
        //            distance. Latched so the shrinking trigger can't bounce back into CLOSE.
        //   CLOSE    command a closing velocity toward the target (tapered to the parking distance, capped) and
        //            burn to match it, which also nulls lateral drift.
        private RendezvousCommand StepCloseApproach(IRendezvousWorld world, out bool stageComplete, bool matchVelocityOnly)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;   // chaser -> target
            double distanceToTarget = relPos.magnitude;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;   // chaser relative to target
            double relSpeed = relVel.magnitude;
            Vector3d bearing = distanceToTarget > 1e-6 ? relPos / distanceToTarget : Vector3d.zero;

            double parkedBand = ParkingDistance + ParkingDistanceBuffer;

            // this technically prevents an MV from going if we still have relVel with the target (RendezParkedSpeedMetersPerSecond >= 0.1)
            // so we're only going to Idle if user isn't forcing a match velocity
            if (distanceToTarget <= parkedBand && relSpeed <= RendezParkedSpeedMetersPerSecond && !matchVelocityOnly)
            {
                _closeBraking = false;
                stageComplete = true;
                return Idle(string.Format("parked at {0:F0} m ({1:F2} m/s)", distanceToTarget, relSpeed));
            }

            // only doing velocity match, do not continue CA logic after this
            if (matchVelocityOnly)
            {
                Vector3d holdDir = relSpeed > 1e-6 ? (-relVel).normalized : bearing;
                // get in position
                if (UseDistanceForMatchVelocities && distanceToTarget > parkedBand)
                {
                    // target is too far away to burn, just get into position
                    return Burn(holdDir, 0.0, string.Format(
                        "holding position for burn({0:F0} m away, ParkingDistance {1:F0})",
                        distanceToTarget, ClosestApproach));
                } else if (distanceToTarget <= parkedBand)
                {
                    // user isn't waiting for a distance, or we're in our parking range = match velocities
                    return StepMatchVelocity(world, out stageComplete);
                }

                return Idle("idling...");
            }

            // Is the current trajectory already going to carry us inside the band? Trust the projection only
            // when nearby (a few km), where the two craft share their perturbations.
            bool caInBand = MathHelpers.IsFinite(ClosestApproach) && ClosestApproach <= parkedBand && distanceToTarget <= CaTrustRangeMeters;

            if (UseDistanceForMatchVelocities)
            {
                if (caInBand)
                {
                    if (_closeBraking || distanceToTarget <= parkedBand)
                    {
                        _closeBraking = true;
                        return StepMatchVelocity(world, out stageComplete);
                    }
                    Vector3d holdDir = relSpeed > 1e-6 ? (-relVel).normalized : bearing;
                    return Burn(holdDir, 0.0, string.Format(
                        "close approach: terminal trajectory ({0:F0} m, CA {1:F0} m in band) - holding for the safe-haven burn",
                        distanceToTarget, ClosestApproach));
                }

                double closingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));
                double brakingDistance = closingSpeed * BrakingSlewLeadSeconds
                    + closingSpeed * closingSpeed / (2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared));
                double brakeTrigger = ParkingDistance + brakingDistance;
                if (distanceToTarget <= brakeTrigger) _closeBraking = true;
                if (_closeBraking) return StepMatchVelocity(world, out stageComplete);
            }
            else if (caInBand)
            {
                return Burn(bearing, 0.0, string.Format(
                    "close approach: coasting to terminal ({0:F0} m, CA {1:F0} m already in band)",
                    distanceToTarget, ClosestApproach));
            }

            // CLOSE: command a closing speed toward the target, then burn to null the gap to our actual relative
            // velocity (also kills lateral drift). Capped by the brake-to-rest limit v = sqrt(2·a·d) — the
            // fastest we can still stop by the parking distance — so a high or lowered Max Closing Speed can't
            // force a violent terminal brake.
            double remainingDistance = Math.Max(0.0, distanceToTarget - ParkingDistance);
            double brakeToRestSpeed = Math.Sqrt(
                2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared) * remainingDistance);
            double commandedClosingSpeed = MathHelpers.Clamp(
                Math.Min(remainingDistance * RendezDistanceApproachGain, brakeToRestSpeed),
                0.0, RendezMaxApproachSpeedMetersPerSecond);

            Vector3d desiredRelVel = bearing * commandedClosingSpeed;
            Vector3d commandedVelocityGap = desiredRelVel - relVel;
            double closingSpeedVelocityGap = commandedVelocityGap.magnitude;

            // Within the deadband: hold heading at zero throttle, NOT Idle — Idle releases the actuation orient
            // gate so the next correction re-orients (stop-start micro-burn). Deadband relaxes with range.
            double velocityDeadband = MathHelpers.Clamp(distanceToTarget * RendezBurnDeadbandPerMeter,
                RendezBurnDeadbandMetersPerSecond, RendezBurnDeadbandMaxMetersPerSecond);
            if (closingSpeedVelocityGap <= velocityDeadband)
                return Burn(bearing, 0.0,
                    string.Format("close approach: holding at {0:F0} m ({1:F2} m/s)", distanceToTarget, relSpeed));

            double throttle = MathHelpers.Clamp(closingSpeedVelocityGap / RendezThrottleTaperMetersPerSecond, BurnMinThrottle, 1.0);
            double actualClosingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));
            return Burn(commandedVelocityGap, throttle, string.Format(
                "close approach: {0:F0} m, closing {1:F2}/{2:F2} m/s", distanceToTarget, actualClosingSpeed, commandedClosingSpeed));
        }

        // Conic intercept from the current measured state, with the target two-body-propagated. The arrival
        // sweep spans a fraction of the active orbit's period.
        private InterceptSolution PlanIntercept(IRendezvousWorld world)
        {
            double mu = world.Mu;
            double measureUt = world.UniversalTime;

            // Plan from the active state coasted forward to the estimated ignition time, so the frozen ΔV
            // matches the state the engine fires from rather than the (earlier) measured state.
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

            // Target prediction propagates from the MEASUREMENT epoch, so absolute arrival UTs map correctly.
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

        // A finished method drops to Coast, ready for the next user-chosen method; CloseApproach Completes.
        private void CompleteStage()
        {
            ClearBurnState();
            bbState.InterceptPhase = bbState.RendezvousMethod == RendezvousMethod.CloseApproach
                ? InterceptPhase.Complete
                : InterceptPhase.Coast;
            if (bbState.InterceptPhase == InterceptPhase.Complete) bbState.ActiveModule = BlackbirdModule.None;
        }

        private void ClearBurnState()
        {
            _burnArmed = false;
            _hohmannPreviewComputed = false;
            _burnIgnitionUt = 0.0;
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

        private RendezvousCommand Burn(Vector3d direction, double throttle, string status)
        {
            return new RendezvousCommand
            {
                Phase = bbState.InterceptPhase,
                Method = bbState.RendezvousMethod,
                HasBurn = true,
                ThrustDirection = direction.normalized,
                Throttle = MathHelpers.Clamp(throttle, 0.0, 1.0),
                Status = status
            };
        }

        private RendezvousCommand Idle(string status)
        {
            return new RendezvousCommand
            {
                Phase = bbState.InterceptPhase,
                Method = bbState.RendezvousMethod,
                HasBurn = false,
                ThrustDirection = Vector3d.zero,
                Throttle = 0.0,
                Status = status
            };
        }
    }
}
