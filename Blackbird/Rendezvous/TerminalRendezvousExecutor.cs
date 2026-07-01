using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using Blackbird.Modules;

namespace Blackbird.Rendezvous
{
    // Rendezvous execution state machine. Each method (Intercept / MatchVelocity / CloseApproach) runs independently via Execute(method)
    public sealed class TerminalRendezvousExecutor
    {
        // --- intercept tuning (public so callers/tests can adjust) ---
        public int InterceptArrivalSamples = 60;          // Lambert solves per plan
        public double InterceptBudgetMilliseconds = 20.0; // wall-clock cap per plan
        public int InterceptCaSamples = 120;              // samples for the honest (J2) predicted-CA of the chosen plan
        public double InterceptTofMinFraction = 0.05;     // arrival sweep, fraction of the orbital period
        public double InterceptTofMaxFraction = 0.95;

        // Plan from the active state coasted forward by this much, so the frozen ΔV matches the state the
        // engine actually fires from after the orient delay. Set by the actuation layer; 0 offline.
        public double IgnitionLeadSeconds = 0.0;
        private const double IgnitionLeadMaxSeconds = 300.0;

        // Hohmann coast-to-ignition: don't orient (or burn RCS) until ignition is within the orient window;
        // inside the final window the orient is just a drift correction before the burn.
        private const double HohmannPreOrientWindowSeconds = 600.0;  // 10 min: start the orientation maneuver
        private const double HohmannFinalOrientSeconds = 15.0;       // 15 s: corrective orientation
        private const double HohmannReSolveIntervalSeconds = 0.5;    // re-solve the burn at most this often in the window

        private const double MinUsefulDeltaV = 0.5;              // a plan below this is a no-op/degenerate
        private const double AlreadyCloseMeters = 2000.0;        // only skip intercept if genuinely this close
        private const double BurnTaperBandMetersPerSecond = 5.0; // throttle tapers over the last few m/s
        private const double BurnMinThrottle = 0.05;
        private const double CutoffEpsilonMetersPerSecond = 0.15; // done within this of planned ΔV
        private const double PeakDropMetersPerSecond = 0.5;       // done if delivered falls back from its peak
        private const double BurnStallSeconds = 2.0;             // done if delivered plateaus this long
        private const double BurnProgressThreshold = 1.0;        // only arm the stall timer once truly thrusting
        private const double BurnStallProgressDeadband = 0.2;    // min delivered gain that counts as progress

        // --- close-approach tuning ---
        public const double ParkingDistanceDefaultMeters = 10.0;
        public double ParkingDistance = ParkingDistanceDefaultMeters;   // "match velocities at X m" (UI)
        public bool UseDistanceForMatchVelocities = true;
        private double burnCaWhenMeters = 0.0;
        private const double ParkingDistanceBuffer = 5.0;          // slack added to the parking distance for the "in band" test
        // Final-approach closing speed = min(brake-to-rest to the parking distance, max-speed cap). Gain retained
        // for the UI/handler but no longer shapes the profile (one burn, not a per-tick proportional crawl).
        public const double RendezDistanceApproachGainDefault = 0.2;
        public double RendezDistanceApproachGain = RendezDistanceApproachGainDefault;
        public const double RendezMaxApproachSpeedDefaultMetersPerSecond = 5.0;
        public double RendezMaxApproachSpeedMetersPerSecond = RendezMaxApproachSpeedDefaultMetersPerSecond;
        // Brake point = ParkingDistance + decel-to-stop = D + v²/2a. Decel is vessel-specific (thrust/mass); the
        // handler sets it each tick. The flip-to-retro is absorbed by the pre-oriented hold, not a distance budget.
        public double BrakingDecelMetersPerSecondSquared = 5.0;

        // Latest plan: a live preview while idle/coast, or the frozen plan while executing.
        public bool HasInterceptPlan { get; private set; }
        public bool HaveHohmannTransfer { get; private set; }

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
        private double _hohmannArrivalUt;          // fixed transfer arrival UT, held across re-solves
        private double _lastReSolveUt;             // throttle for the Hohmann re-solve-at-ignition
        public double PlannedIgnitionUt => _plannedIgnitionUt;
        public bool BurnArmed => _burnArmed;

        // Ship acceleration (thrust/mass), fed by the actuation layer, used to center the intercept burn:
        // ignite half the burn duration before the planned ignition so the burn straddles that instant. 0 = unknown.
        public double BurnAccelMetersPerSecondSquared = 0.0;

        // Final-approach sub-phase latch: the single closing burn has finished, so we now hold retrograde and,
        // when parking is enabled, fire the one kill burn at the brake point.
        private bool _faClosingDone;

        SharedState bbState;

        public TerminalRendezvousExecutor() { }

        public void Init(SharedState s) => bbState = s;

        // Back to Idle at the first stage, clearing cached plan/burn state.
        public void Reset()
        {
            bbState.RendezvousMethod = RendezvousMethod.None;
            bbState.InterceptPhase = InterceptPhase.Idle;
            bbState.RendezvousEnabled = false;
            if (bbState.ActiveModule == BlackbirdModule.Rendezvous) bbState.ActiveModule = BlackbirdModule.None;
            ClearBurnState();
            HasInterceptPlan = false;
            _burnArmed = false;
            _faClosingDone = false;
            burnCaWhenMeters = 0.0;
            HaveHohmannTransfer = false;
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
            burnCaWhenMeters = 0.0;
            _burnArmed = false;
            _faClosingDone = false;
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
            _faClosingDone = false;
            burnCaWhenMeters = 0.0;
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
            bbState.RendezvousEnabled = false;
            if (bbState.ActiveModule == BlackbirdModule.Rendezvous) bbState.ActiveModule = BlackbirdModule.None;
            ClearBurnState();
        }

        // How many eligible Hohmann windows to surface for the user to choose from.
        private const int HohmannCandidateCount = 5;

        // Refresh the cached plan for the current (not-yet-executed) stage so the UI can show ΔV / predicted
        // CA before the user commits. No phase change. The Hohmann path computes once and caches.
        public void RefreshPlanPreview(IRendezvousWorld world, InterceptMethod method)
        {
            HaveHohmannTransfer = false;
            if (world == null) return;
            //if (bbState.InterceptPhase != InterceptPhase.Idle && bbState.InterceptPhase != InterceptPhase.Coast) return;

            if (method == InterceptMethod.Hohmann)
            {
                // Surface several windows for the user to pick; default to the soonest until they select one.
                bbState.InterceptCandidates = BuildHohmannPlans(world, HohmannCandidateCount);
                bbState.SelectedInterceptCandidateIndex = bbState.InterceptCandidates.Count > 0 ? 0 : -1;
                bbState.InterceptSolution = bbState.InterceptCandidates.Count > 0
                    ? bbState.InterceptCandidates[0]
                    : BuildHohmannPlan(world);
                HasInterceptPlan = bbState.InterceptSolution.Success;
                HaveHohmannTransfer = true;
                return;
            }

            if (method == InterceptMethod.Phasing)
            {
                // Single tangential burn NOW to set the phasing period; meet the target back at the burn point
                // after N orbits, then the MatchVelocity / CloseApproach stages finish it.
                bbState.InterceptCandidates = PhasingPlanner.BuildPhasingPlans(world, HohmannCandidateCount);
                bbState.SelectedInterceptCandidateIndex = bbState.InterceptCandidates.Count > 0 ? 0 : -1;
                bbState.InterceptSolution = bbState.InterceptCandidates.Count > 0
                    ? bbState.InterceptCandidates[0]
                    : default(InterceptSolution);
                HasInterceptPlan = bbState.InterceptSolution.Success;
                return;
            }

            bbState.InterceptCandidates = new List<InterceptSolution>();
            bbState.SelectedInterceptCandidateIndex = -1;
            bbState.InterceptSolution = PlanIntercept(world);
            HasInterceptPlan = bbState.InterceptSolution.Success;
        }

        // Honest predicted closest approach for a transfer that applies impulse dv1 at ignitionUt and arrives
        // at arrivalUt: coast the chaser to ignition, add dv1, then fly the transfer and the target under J2
        // (conic when J2 == 0) and take the minimum separation. This is the real miss a conic plan flies under
        // oblateness — what the panel should show — replacing the optimistic "arrives by construction" 0.
        private double HonestPredictedCa(IRendezvousWorld world, Vector3d dv1, double ignitionUt, double arrivalUt)
        {
            double mu = world.Mu, now = world.UniversalTime;
            ClosestApproachSolver.Propagate(world.ActivePosition, world.ActiveVelocity, ignitionUt - now, mu,
                world.J2, world.J2ReferenceRadius, world.Pole, out Vector3d chaserPos, out Vector3d chaserVel);
            return ClosestApproachSolver.MinSeparationOverWindow(chaserPos, chaserVel + dv1, ignitionUt, arrivalUt,
                world.TargetPosition, world.TargetVelocity, now, mu, InterceptCaSamples,
                world.J2, world.J2ReferenceRadius, world.Pole);
        }

        // Re-solve the Hohmann departure burn from the current MEASURED state: a fresh Lambert from the chaser
        // at the fixed ignition UT to the J2-propagated target at the fixed arrival UT (both coasted under J2
        // from now). Updates the burn vector/magnitude and the centered ignition in place. No-op on a
        // non-finite/absurd solve so the prior frozen vector stands. Kills the open-loop "fly an hours-old
        // conic vector" staleness; both UTs are fixed so only the vector moves (no moving-target chase).
        private void ReSolveHohmannBurn(IRendezvousWorld world)
        {
            double mu = world.Mu, now = world.UniversalTime;
            double tof = _hohmannArrivalUt - _plannedIgnitionUt;
            if (tof <= 0.0) return;

            ClosestApproachSolver.Propagate(world.ActivePosition, world.ActiveVelocity, _plannedIgnitionUt - now, mu,
                world.J2, world.J2ReferenceRadius, world.Pole, out Vector3d chaserIg, out Vector3d chaserIgVel);
            ClosestApproachSolver.Propagate(world.TargetPosition, world.TargetVelocity, _hohmannArrivalUt - now, mu,
                world.J2, world.J2ReferenceRadius, world.Pole, out Vector3d targetArr, out _);

            LambertResult lam = LambertSolver.Solve(chaserIg, targetArr, tof, mu, true, world.ReferenceNormal);
            if (!lam.Success) return;

            Vector3d dv1 = lam.V1 - chaserIgVel;
            double dvMag = dv1.magnitude;
            if (!MathHelpers.IsFinite(dvMag) || dvMag < 1e-6 || dvMag > 1e8) return;   // keep prior on a bad solve

            _plannedDvUnit = dv1 / dvMag;
            _plannedDvMagnitude = dvMag;
            _targetDepartureVelocity = lam.V1;
            double halfBurn = BurnAccelMetersPerSecondSquared > 0.0 ? 0.5 * dvMag / BurnAccelMetersPerSecondSquared : 0.0;
            _burnIgnitionUt = _plannedIgnitionUt - halfBurn;
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
                    TotalDeltaVMagnitude = dvMag + dv2.magnitude,   // dv1 + dv2 (arrival incl. plane change)
                    IgnitionUt = ut1,
                    ArrivalUt = ut2,
                    TimeOfFlight = ut2 - ut1,
                    PredictedClosestApproach = HonestPredictedCa(world, dv1, ut1, ut2),  // real miss under J2 (conic = ~0)
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

        // Up to maxCount eligible Hohmann windows mapped to InterceptSolutions for the UI to choose from. Same
        // window->solution mapping as BuildHohmannPlan; only sane future-departure windows are kept.
        private List<InterceptSolution> BuildHohmannPlans(IRendezvousWorld world, int maxCount)
        {
            var plans = new List<InterceptSolution>();
            try
            {
                List<(Vector3d dv1, double ut1, Vector3d dv2, double ut2)> windows =
                    OrbitMath.DeltaVForHohmannTransferCandidates(
                        world.UniversalTime, world.ActivePosition, world.ActiveVelocity,
                        world.TargetPosition, world.TargetVelocity, world.Mu, maxCount);

                for (int i = 0; i < windows.Count; i++)
                {
                    (Vector3d dv1, double ut1, Vector3d dv2, double ut2) = windows[i];
                    double dvMag = dv1.magnitude;
                    bool sane = ut1 > world.UniversalTime
                                && !double.IsNaN(dvMag) && !double.IsInfinity(dvMag) && dvMag < 1e8;
                    if (!sane) continue;

                    plans.Add(new InterceptSolution
                    {
                        Success = true,
                        Status = InterceptStatus.Ok,
                        DeltaV = dv1,
                        DeltaVMagnitude = dvMag,
                        TotalDeltaVMagnitude = dvMag + dv2.magnitude,   // dv1 + dv2 (arrival incl. plane change)
                        IgnitionUt = ut1,
                        ArrivalUt = ut2,
                        TimeOfFlight = ut2 - ut1,
                        PredictedClosestApproach = HonestPredictedCa(world, dv1, ut1, ut2),  // real miss under J2 (conic = ~0)
                        TransferDepartureVelocity = world.ActiveVelocity + dv1,
                        TransferArrivalVelocity = Vector3d.zero,
                        SamplesEvaluated = 0
                    });
                }
            }
            catch { }
            return plans;
        }

        // Per-tick update. Runs the active stage while Executing (advancing on completion); otherwise idle
        public RendezvousCommand Update(IRendezvousWorld world,
            double closestApproach = double.PositiveInfinity, double timeToClosestApproach = double.PositiveInfinity)
        {
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
                    return Idle("coasting — Execute " + bbState.RendezvousMethod);   // still owns control between stages
                case InterceptPhase.Complete:
                    ReleaseModule();
                    return Idle("rendezvous complete — control handed back");
                case InterceptPhase.Aborted:
                    ReleaseModule();
                    return Idle("aborted");
                default:
                    ReleaseModule();
                    return Idle("idle — Execute " + bbState.RendezvousMethod);
            }
        }

        // Release the rendezvous lock + control authority (handing back to the user) when a run ends.
        private void ReleaseModule()
        {
            bbState.RendezvousEnabled = false;
            if (bbState.ActiveModule == BlackbirdModule.Rendezvous) bbState.ActiveModule = BlackbirdModule.None;
        }

        private RendezvousCommand StepStage(IRendezvousWorld world, out bool stageComplete)
        {
            if (bbState.RendezvousMethod == RendezvousMethod.Intercept)
                return StepIntercept(world, out stageComplete);
            if (bbState.RendezvousMethod == RendezvousMethod.MatchVelocity)
                return UseDistanceForMatchVelocities
                    ? BrakeAtTerminalDistance(world, out stageComplete) // "match velocities at X": close, ride in, park
                    : StepMatchVelocity(world, out stageComplete);      // unchecked: immediate null
            if (bbState.RendezvousMethod == RendezvousMethod.FinalApproach)
                return StepFinalApproach(world, out stageComplete, UseDistanceForMatchVelocities); // chase; box checked -> auto-park, else hand back

            stageComplete = true;
            ReleaseModule();
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
                // Hohmann/Phasing: REUSE the cached preview plan so Execute fires the burn the user previewed/chose
                // (Phasing ignites now; Hohmann coasts to its window). Single-phase re-solves a fresh Lambert.
                InterceptSolution solution =
                    bbState.InterceptMethod == InterceptMethod.Hohmann ? (HasInterceptPlan ? bbState.InterceptSolution : BuildHohmannPlan(world))
                    : bbState.InterceptMethod == InterceptMethod.Phasing ? bbState.InterceptSolution
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
                _hohmannArrivalUt = solution.ArrivalUt;
                _lastReSolveUt = double.NegativeInfinity;   // re-solve on the first orient-window tick
                double halfBurn = BurnAccelMetersPerSecondSquared > 0.0
                    ? 0.5 * _plannedDvMagnitude / BurnAccelMetersPerSecondSquared : 0.0;
                _burnIgnitionUt = _plannedIgnitionUt - halfBurn;
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            // Hohmann departs in the future. Keep the delivered-ΔV baseline pinned to now so the coast/orient
            // isn't counted as burn progress, then orient only when the burn is near: coast (no RCS) when far
            // out, slew at the orient window, fine-correct in the last seconds. Avoids burning RCS for an hour
            // holding a stale attitude.
            if (bbState.InterceptMethod == InterceptMethod.Hohmann && world.UniversalTime < _burnIgnitionUt)
            {
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;

                double timeToIgnition = _burnIgnitionUt - world.UniversalTime;
                if (timeToIgnition > HohmannPreOrientWindowSeconds)
                    return Idle(string.Format("intercept: coasting to ignition in {0:F0}s", timeToIgnition));

                // Re-solve the burn from the (fresh) measured state to the J2 target at the FIXED arrival, for
                // departure at the FIXED ignition — only the vector/magnitude move, so it can't chase a moving
                // target. Throttled; frozen once inside the commit zone; a stale/absurd solve keeps the prior.
                bool committed = timeToIgnition <= HohmannFinalOrientSeconds;
                if (!committed && world.UniversalTime - _lastReSolveUt >= HohmannReSolveIntervalSeconds)
                {
                    _lastReSolveUt = world.UniversalTime;
                    ReSolveHohmannBurn(world);
                }

                string orientPhase = committed ? "final orientation" : "orienting";
                return Burn(_plannedDvUnit, 0.0,
                    string.Format("intercept: {0}, ignition in {1:F0}s", orientPhase, timeToIgnition));
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
            bool fallbackArmed = _maxDeliveredDv > stallArmThreshold;

            if (fallbackArmed && _maxDeliveredDv > stallArmThreshold && world.UniversalTime - _lastProgressUt > BurnStallSeconds)
                return FinishInterceptBurn(world, delivered, string.Format(
                    "intercept: cutoff (delivered stalled at {0:F1}/{1:F1} m/s)",
                    _maxDeliveredDv, _plannedDvMagnitude), out stageComplete);

            if (fallbackArmed && delivered < _maxDeliveredDv - PeakDropMetersPerSecond)
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

        // Drive ONE burn to completion along a FIXED axis captured at ignition. No per-tick re-aim (which near
        // convergence swings the null axis and re-fires — the "MV flips and burns again" bug), and once armed it
        // runs to the delivered-ΔV cutoff regardless of the evolving state. Same latch as the intercept burn
        // (reached ΔV / stalled once thrusting / peaked and fell back). One burn only: the caller latches the
        // stage complete on `done` and never re-enters.
        private RendezvousCommand DriveSingleBurn(IRendezvousWorld world, Vector3d dvVector, string label, out bool done)
        {
            done = false;

            if (!_burnArmed)
            {
                _plannedDvMagnitude = dvVector.magnitude;
                _plannedDvUnit = _plannedDvMagnitude > 1e-9 ? dvVector / _plannedDvMagnitude : Vector3d.zero;
                _burnStartVelocity = world.ActiveVelocity;
                _maxDeliveredDv = 0.0;
                _lastProgressDelivered = 0.0;
                _lastProgressUt = world.UniversalTime;
                _burnArmed = true;
            }

            // Nothing worth burning: accept and finish (still counts as the one allowed burn for this stage).
            if (_plannedDvMagnitude <= MinUsefulDeltaV)
            {
                done = true;
                return Idle(string.Format("{0}: matched ({1:F2} m/s)", label, _plannedDvMagnitude));
            }

            double delivered = Vector3d.Dot(world.ActiveVelocity - _burnStartVelocity, _plannedDvUnit);
            if (delivered > _maxDeliveredDv) _maxDeliveredDv = delivered;
            if (delivered > _lastProgressDelivered + BurnStallProgressDeadband)
            {
                _lastProgressDelivered = delivered;
                _lastProgressUt = world.UniversalTime;
            }

            bool stallArmed = _maxDeliveredDv > Math.Min(BurnProgressThreshold, 0.4 * _plannedDvMagnitude);
            if (delivered >= _plannedDvMagnitude - CutoffEpsilonMetersPerSecond
                || (stallArmed && world.UniversalTime - _lastProgressUt > BurnStallSeconds)
                || (stallArmed && delivered < _maxDeliveredDv - PeakDropMetersPerSecond))
            {
                done = true;
                return Idle(string.Format("{0}: burn complete ({1:F1}/{2:F1} m/s)", label, delivered, _plannedDvMagnitude));
            }

            double remaining = _plannedDvMagnitude - delivered;
            double throttle = MathHelpers.Clamp(remaining / BurnTaperBandMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(_plannedDvUnit, throttle,
                string.Format("{0}: {1:F1}/{2:F1} m/s delivered", label, delivered, _plannedDvMagnitude));
        }

        // Immediate match velocity (box unchecked): cancel the relative velocity with EXACTLY ONE burn. The null
        // axis is captured at ignition (this close gravity acts near-identically on both craft, so it barely
        // rotates during the burn); no re-aim, and no follow-up burn regardless of the residual.
        private RendezvousCommand StepMatchVelocity(IRendezvousWorld world, out bool stageComplete)
        {
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            return DriveSingleBurn(world, -relVel, "match velocity", out stageComplete);
        }

        // "Match velocities at X" (box checked): HOLD retrograde at true zero throttle (no re-regulation) until the
        // gap reaches the brake point, then fire the ONE kill burn to completion. Brake point = D + v²/2a from the
        // MEASURED closing speed (no CA prediction needed); the flip-to-retro is absorbed by the pre-oriented hold.
        // Never chases the target, and never fires a second burn.
        private RendezvousCommand BrakeAtTerminalDistance(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;   // chaser -> target
            double distanceToTarget = relPos.magnitude;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;   // chaser relative to target
            Vector3d bearing = distanceToTarget > 1e-6 ? relPos / distanceToTarget : Vector3d.zero;

            double parkedBand = ParkingDistance + ParkingDistanceBuffer;

            // At/inside the brake point (or already committed to the burn): the single kill burn, run to completion.
            if (_burnArmed || distanceToTarget <= Math.Max(parkedBand, burnCaWhenMeters))
                return DriveSingleBurn(world, -relVel, "match velocity: braking", out stageComplete);

            // Not there yet: track the brake point from the measured closing speed and HOLD retrograde. No thrust.
            double closingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));
            double stoppingDistance = closingSpeed * closingSpeed / (2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared));
            burnCaWhenMeters = ParkingDistance + stoppingDistance;

            Vector3d holdDir = relVel.sqrMagnitude > 1e-12 ? (-relVel).normalized : bearing;
            return Burn(holdDir, 0.0, string.Format(
                "match velocity: holding, brake at {0:F0} m ({1:F0} m, {2:F2} m/s closing)",
                burnCaWhenMeters, distanceToTarget, closingSpeed));
        }

        // Final Approach: ONE closing burn to a bounded closing speed, then "become the match-velocity" — hold
        // retrograde while coasting in and (box checked) fire ONE kill burn at the brake point; box unchecked hands
        // back after the closing burn so the user brakes with Match Velocity. No per-tick regulation: the old
        // deadband controller toggled the throttle in/out of the band every frame ("holding" status while the
        // engine kept pulsing) — that path is gone.
        private RendezvousCommand StepFinalApproach(IRendezvousWorld world, out bool stageComplete, bool autoPark)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;
            double distanceToTarget = relPos.magnitude;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            Vector3d bearing = distanceToTarget > 1e-6 ? relPos / distanceToTarget : Vector3d.zero;

            // Phase 1 — the single closing burn. Close as fast as brake-to-rest to the parking distance allows,
            // capped by the max-speed setting. One burn both sets the closing speed and nulls lateral drift.
            if (!_faClosingDone)
            {
                double remainingDistance = Math.Max(0.0, distanceToTarget - ParkingDistance);
                double brakeToRestSpeed = Math.Sqrt(
                    2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared) * remainingDistance);
                double closingSpeed = Math.Min(brakeToRestSpeed, RendezMaxApproachSpeedMetersPerSecond);
                Vector3d closingDv = bearing * closingSpeed - relVel;

                RendezvousCommand cmd = DriveSingleBurn(world, closingDv, "final approach: closing", out bool closeDone);
                if (closeDone)
                {
                    _faClosingDone = true;
                    _burnArmed = false;   // re-arm the latch for the kill burn
                    if (!autoPark)        // box unchecked: hand back after the one closing burn
                    {
                        stageComplete = true;
                        return Idle(string.Format(
                            "final approach: closing burn done ({0:F2} m/s) - use Match Velocity to brake",
                            relVel.magnitude));
                    }
                }
                return cmd;
            }

            // Phase 3 — at/inside the brake point (or already committed): the single kill burn to completion.
            double parkedBand = ParkingDistance + ParkingDistanceBuffer;
            if (_burnArmed || distanceToTarget <= Math.Max(parkedBand, burnCaWhenMeters))
                return DriveSingleBurn(world, -relVel, "final approach: braking", out stageComplete);

            // Phase 2 — hold retrograde at true zero throttle, coasting to the brake point. No burns, no pulsing.
            double closingSpeedNow = Math.Max(0.0, Vector3d.Dot(relVel, bearing));
            double stoppingDistance = closingSpeedNow * closingSpeedNow / (2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared));
            burnCaWhenMeters = ParkingDistance + stoppingDistance;

            Vector3d holdDir = relVel.sqrMagnitude > 1e-12 ? (-relVel).normalized : bearing;
            return Burn(holdDir, 0.0, string.Format(
                "final approach: holding, brake at {0:F0} m ({1:F0} m, {2:F2} m/s closing)",
                burnCaWhenMeters, distanceToTarget, closingSpeedNow));
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
            // Under oblateness (J2 != 0) it integrates the target under J2 so Lambert aims where the target
            // REALLY will be (the dominant RSS error is the target drifting off the conic path over hours).
            double j2 = world.J2, reEq = world.J2ReferenceRadius;
            Vector3d pole = world.Pole;
            Func<double, Vector3d> targetPositionAt = ut =>
            {
                ClosestApproachSolver.Propagate(targetPosition, targetVelocity, ut - measureUt, mu,
                    j2, reEq, pole, out Vector3d rt, out _);
                return rt;
            };

            InterceptSolution solution = InterceptSolver.Solve(ignitionPos, ignitionVel, mu,
                world.ReferenceNormal, ignitionUt, targetPositionAt, tofMin, tofMax,
                InterceptArrivalSamples, true, InterceptBudgetMilliseconds);

            // Replace the solver's optimistic (conic) predicted CA with the honest miss the conic transfer will
            // actually fly under J2 — both transfer and target propagated under oblateness.
            if (solution.Success)
                solution.PredictedClosestApproach = ClosestApproachSolver.MinSeparationOverWindow(
                    ignitionPos, solution.TransferDepartureVelocity, ignitionUt, solution.ArrivalUt,
                    targetPosition, targetVelocity, measureUt, mu, InterceptCaSamples, j2, reEq, pole);

            return solution;
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
            bbState.InterceptPhase = bbState.RendezvousMethod == RendezvousMethod.FinalApproach
                ? InterceptPhase.Complete
                : InterceptPhase.Coast;
            if (bbState.InterceptPhase == InterceptPhase.Complete) ReleaseModule();
        }

        private void ClearBurnState()
        {
            _burnArmed = false;
            HaveHohmannTransfer = false;
            _burnIgnitionUt = 0.0;
            _hohmannArrivalUt = 0.0;
            _lastReSolveUt = 0.0;
            _burnStartVelocity = Vector3d.zero;
            _targetDepartureVelocity = Vector3d.zero;
            _plannedDvMagnitude = 0.0;
            _plannedDvUnit = Vector3d.zero;
            _maxDeliveredDv = 0.0;
            _lastProgressDelivered = 0.0;
            _lastProgressUt = 0.0;

            _faClosingDone = false;

            burnCaWhenMeters = 0.0;
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
