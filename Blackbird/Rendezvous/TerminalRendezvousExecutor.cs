using Blackbird.Mathematics;
using Blackbird.Modules;
using System;
using System.Collections.Generic;
using UnityEngine;

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

        // --- match-velocity tuning ---
        private const double MatchVelocityToleranceMetersPerSecond = 0.15; // nulled within this
        private const double MatchThrottleLimitMetersPerSecond = 3.0;      // prevents us from firing at full throtle at/above this m/s
        private const double MatchStallSeconds = 2.0;                      // cut if rel speed stops dropping...
        private const double MatchStallSpeedFloor = 1.0;                   // ...and is already near nulled

        // --- final approach tuning ---
        public double ParkingDistanceMeters = 100.0; // we will ALWAYS stop by at least this distance if MV is used
        public bool ParkingDistanceEnabled = false; // used to determine a) if E: MV should wait or b) if E: FA should flip + MV;  NOTE: does not determine if we use ParkingDistanceMeters or not, just whether we flip
        private double burnMvAtDistance = 0.0;
        // Final-approach closing speed = min(brake-to-rest to the parking distance, Max Closing Speed)
        //public const double RendezDistanceApproachGainDefault = 0.2;
        private const double RendezThrottleTaperMetersPerSecond = 3.0;   // closing-burn throttle taper band
        // Brake point = ParkingDistance + decel-to-stop = D + v²/2a. Decel is vessel-specific (thrust/mass); updated every tick of the burn
        public double BrakingDecelMetersPerSecondSquared = 5.0;
        public double FlipSlewTimeSeconds = 0.0; // 180° flip time (retrograde reorientation), fed each tick by the handler like the decel
        public bool KeepFaAxesFrozen = false; // false = track target while burning, true = pick one estimated axis and hold it over burn
        private bool _mvTerminalBraking = false;
        // frozen-axis final approach
        private bool _frozenApproachArmed = false;
        private Vector3d _faDvUnit;
        private double _faDvMagnitude;
        private Vector3d _faRelVelAtIgnition;
        private double _faMaxDelivered;

        // --- plans & transfers ---
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

        // Match-velocity burn state.
        private bool _matchArmed;
        private double _matchMinRelSpeed;          // smallest relative speed reached, for stall detection
        private double _matchLastProgressUt;
        private Vector3d _matchNullDir0;           // null direction at ignition; used to detect an overshoot flip

        // Final Approach: the single closing burn has reached the commanded closing speed, so we now "become the
        // match-velocity" (hold retrograde and, box checked, kill at the brake point). No more closing regulation.
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
            _matchArmed = false;
            _mvTerminalBraking = false;
            burnMvAtDistance = 0.0;
            BrakingDecelMetersPerSecondSquared = 5.0;
            HaveHohmannTransfer = false;
            ResetFinalApproach();
            bbState.InterceptSolution = default(InterceptSolution);
            HasLastInterceptBurnReport = false;
            _lastInterceptBurnReport = default(InterceptBurnReport);
        }

        private void ResetFinalApproach()
        {
            _faClosingDone = false;
            _frozenApproachArmed = false;
            _faDvUnit = Vector3d.zero;
            _faDvMagnitude = 0.0;
            _faRelVelAtIgnition = Vector3d.zero;
            _faMaxDelivered = 0.0;
        }

        // Start the current stage's loop. Valid from Idle (first stage) or Coast (the queued next stage).
        public bool Execute()
        {
            if (bbState.InterceptPhase != InterceptPhase.Idle) return false;
            bbState.InterceptPhase = InterceptPhase.Executing;
            bbState.ActiveModule = BlackbirdModule.Rendezvous;
            burnMvAtDistance = 0.0;
            BrakingDecelMetersPerSecondSquared = 5.0;
            _burnArmed = false;
            _matchArmed = false;
            ResetFinalApproach();
            HasLastInterceptBurnReport = false;
            return true;
        }

        // cancel an existing method if active then run a new one
        public bool ForceExecute(RendezvousMethod method)
        {
            bbState.RendezvousMethod = method;
            bbState.InterceptPhase = InterceptPhase.Executing;
            bbState.ActiveModule = BlackbirdModule.Rendezvous;

            _burnArmed = false;
            _matchArmed = false;
            _faClosingDone = false;
            burnMvAtDistance = 0.0;
            ResetFinalApproach();
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
            ResetFinalApproach();
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
                return ParkingDistanceEnabled
                    ? BrakeAtTerminalDistance(world, out stageComplete) // "match velocities at X": close, ride in, park
                    : StepMatchVelocity(world, out stageComplete);      // unchecked: immediate null
            if (bbState.RendezvousMethod == RendezvousMethod.FinalApproach)
                return StepFinalApproach(world, out stageComplete, ParkingDistanceEnabled); // chase; box checked -> auto-park, else hand back

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

        // Match-velocity loop: cancel the chaser's velocity relative to the target, re-aiming opposite the
        // CURRENT relative velocity each tick. The cutoff is frame-independent (relative speed directly) since
        // this close gravity acts almost identically on both craft.
        private RendezvousCommand StepMatchVelocity(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            double relSpeed = relVel.magnitude;
            Vector3d nullDir = relSpeed > 1e-6 ? (-relVel).normalized : Vector3d.zero;

            if (!_matchArmed)
            {
                _matchMinRelSpeed = relSpeed;
                _matchLastProgressUt = world.UniversalTime;
                _matchNullDir0 = nullDir;
                _matchArmed = true;
            }

            if (relSpeed < _matchMinRelSpeed)
            {
                _matchMinRelSpeed = relSpeed;
                _matchLastProgressUt = world.UniversalTime;
            }

            // Once near the null, if the re-aim would flip past 90° from where we started, we've overshot the
            // match — stop and accept it rather than turn around and chase a second burn (which added speed).
            bool overshotFlip = relSpeed <= MatchStallSpeedFloor && Vector3d.Dot(nullDir, _matchNullDir0) < 0.0;

            if (relSpeed <= MatchVelocityToleranceMetersPerSecond // velocity already matched
                || overshotFlip                                    // overshot the null: don't chase a second burn
                || (relSpeed <= MatchStallSpeedFloor && world.UniversalTime - _matchLastProgressUt > MatchStallSeconds)) // stall guard (rel speed stops dropping after consecutive MV's
            {
                // velocity already matched (or overshot / stalled): one burn, accept the result
                stageComplete = true;
                return Idle(string.Format("match velocity: nulled ({0:F2} m/s)", relSpeed));
            }

            // steer + burn in the vel null direction (closed-loop: re-aimed each tick)
            // david: this just prevents us from burning full-throttle if m/s < 3
            double throttle = MathHelpers.Clamp(relSpeed / MatchThrottleLimitMetersPerSecond, BurnMinThrottle, 1.0);
            return Burn(nullDir, throttle, string.Format("match velocity {0:F2} m/s remaining", relSpeed));
        }

        // "Match velocities at X" (MV with the box checked): brake an incoming/closing target to rest at the
        // parking distance.  holds retrograde if we're not at the terminal distance yet.
        private RendezvousCommand BrakeAtTerminalDistance(IRendezvousWorld world, out bool stageComplete)
        {
            stageComplete = false;
            Vector3d relPos = world.TargetPosition - world.ActivePosition;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            Vector3d bearing = relPos.magnitude > 1e-6 ? relPos / relPos.magnitude : Vector3d.zero;

            if (!_mvTerminalBraking)
            {
                // where do I need to be to have room to kill velocity and stop by the park distance?
                double closingSpeed = Math.Max(0.0, Vector3d.Dot(relVel, bearing));
                burnMvAtDistance = ParkingDistanceMeters
                    + closingSpeed * closingSpeed / (2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared));

                if (relPos.magnitude > Math.Max(ParkingDistanceMeters, burnMvAtDistance))
                {
                    // not there yet: hold retrograde and wait
                    Vector3d holdDir = relVel.sqrMagnitude > 1e-12 ? (-relVel).normalized : bearing;
                    return Burn(holdDir, 0.0, string.Format(
                        "match velocity: holding, brake at {0:F0} m ({1:F0} m, {2:F2} m/s closing)",
                        burnMvAtDistance, relPos.magnitude, closingSpeed));
                }

                _mvTerminalBraking = true;   // reached the mark — from here it's just "kill velocity now"
            }

            return StepMatchVelocity(world, out stageComplete);   // committed: full null to completion, no re-gating
        }

        private double SafeClosingSpeed(double distanceGap)
        {
            double a = Math.Max(0.01, BrakingDecelMetersPerSecondSquared);
            double tSlew = Math.Max(0.0, FlipSlewTimeSeconds);
            // david todo: may need a lower speed when testing in RSS
            return 0.5 * a * (-tSlew + Math.Sqrt(tSlew * tSlew + 4.0 * distanceGap / a));
        }

        public static bool WouldDeorbit(IRendezvousWorld world, Vector3d deltaV)
        {
            double periapsis = OrbitMath.PeriapsisRadius(world.ActivePosition, world.ActiveVelocity + deltaV, world.Mu);
            return periapsis < world.BodyRadius + Math.Max(0.0, world.AtmosphereDepth);
        }

        public Vector3d FinalApproachClosingDeltaV(IRendezvousWorld world)
        {
            Vector3d relPos = world.TargetPosition - world.ActivePosition;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;
            Vector3d bearing = relPos.magnitude > 1e-6 ? relPos / relPos.magnitude : Vector3d.zero;
            double vc = SafeClosingSpeed(Math.Max(0.0, relPos.magnitude - ParkingDistanceMeters));
            return bearing * vc - relVel;   // the one closing burn
        }

        private RendezvousCommand StepFinalApproach(IRendezvousWorld world, out bool stageComplete, bool autoPark)
        {
            stageComplete = false;

            Vector3d relPos = world.TargetPosition - world.ActivePosition;
            Vector3d relVel = world.ActiveVelocity - world.TargetVelocity;

            if (!_faClosingDone)
            {
                Vector3d bearing = relPos.magnitude > 1e-6 ? relPos / relPos.magnitude : Vector3d.zero;

                // current distance - park at
                double remainingDistance = Math.Max(0.0, relPos.magnitude - ParkingDistanceMeters);
                //double brakeToRestSpeed = Math.Sqrt(2.0 * Math.Max(0.01, BrakingDecelMetersPerSecondSquared) * remainingDistance);

                // might want to freeze this if axis is locked
                double commandedClosingSpeed = SafeClosingSpeed(remainingDistance);

                Vector3d dVBudget = bearing * commandedClosingSpeed - relVel; // cost of burn
                
                // the burn is done continuously, so we arm the initial params, and then this function is called in main loop to continue burn up until our dV budget
                if (KeepFaAxesFrozen)
                {
                    if (!_frozenApproachArmed)
                    {
                        _faDvMagnitude = dVBudget.magnitude;
                        _faDvUnit = _faDvMagnitude > 1e-6 ? dVBudget / _faDvMagnitude : bearing;
                        _faRelVelAtIgnition = relVel;
                        _faMaxDelivered = 0.0;
                        _frozenApproachArmed = true;
                    }

                    double delivered = Vector3d.Dot(relVel - _faRelVelAtIgnition, _faDvUnit);
                    if (delivered > _faMaxDelivered) _faMaxDelivered = delivered;

                    bool reached = delivered >= _faDvMagnitude - MatchVelocityToleranceMetersPerSecond;
                    bool peaked = _faMaxDelivered > BurnProgressThreshold
                                   && delivered < _faMaxDelivered - PeakDropMetersPerSecond;

                    if (!reached && !peaked)
                    {
                        double remaining = _faDvMagnitude - delivered;
                        double throttle = MathHelpers.Clamp(
                            remaining / RendezThrottleTaperMetersPerSecond, BurnMinThrottle, 1.0);
                        return Burn(_faDvUnit, throttle, string.Format(
                            "final approach [frozen]: {0:F1}/{1:F1} m/s along axis at {2:F0} m",
                            delivered, _faDvMagnitude, relPos.magnitude));
                    }

                    _faClosingDone = true;
                } else
                {
                    double closingSpeed = Vector3d.Dot(relVel, bearing);

                    if (Math.Abs(closingSpeed - commandedClosingSpeed) > MatchVelocityToleranceMetersPerSecond)
                    {
                        double throttle = MathHelpers.Clamp(dVBudget.magnitude / RendezThrottleTaperMetersPerSecond, BurnMinThrottle, 1.0);
                        return Burn(dVBudget, throttle, string.Format(
                            "final approach [tracking]: closing {0:F2}/{1:F2} m/s at {2:F0} m",
                            closingSpeed, commandedClosingSpeed, relPos.magnitude));
                    }

                    _faClosingDone = true;
                }
            }

            if (autoPark) return BrakeAtTerminalDistance(world, out stageComplete);

            stageComplete = true;
            return Idle(string.Format("final approach: done ({0:F2} m/s)", relVel.magnitude));
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
                : InterceptPhase.Idle;
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

            _matchArmed = false;
            _mvTerminalBraking = false;
            _matchMinRelSpeed = 0.0;
            _matchLastProgressUt = 0.0;
            _matchNullDir0 = Vector3d.zero;
            _faClosingDone = false;

            burnMvAtDistance = 0.0;
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
