using System;
using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using Blackbird.Trajectory;
using Blackbird.Helpers;
using UnityEngine;
using Blackbird.Modules;

namespace Blackbird.Rendezvous
{
    // Couples the terminal-rendezvous executor to the live vessel (rendezvous analogue of LaunchHandler):
    // each frame builds the world seam, runs the executor, caches the command + relative state; the
    // fly-by-wire pass actuates steering/throttle only while a stage is burning. Engage gates the whole thing.
    public sealed class RendezvousHandler
    {
        private SharedState bbState;
        private WarpHelper warp;

        private readonly TerminalRendezvousExecutor _executor = new TerminalRendezvousExecutor();
        private readonly AttitudeControl _attitude = new AttitudeControl();
        private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Rendezvous);

        private RendezvousCommand _command;
        private bool _hasCommand;
        private bool _engaged;
        private bool _burningLastApply;   // were we actuating thrust on the previous fly-by-wire pass?

        // Soft-enable RCS during a maneuver so torque-poor craft can still point (engine off during the orient
        // hold gives no gimbal torque, and reaction wheels alone are often too weak), restoring the player's
        // setting on handback.
        private bool _rcsForced;
        private bool _rcsPriorState;

        // Live closest-approach monitor: recomputed off the draw path on a throttle, searching out to the
        // synodic period (capped) so time-to-CA actually counts down rather than pinning at one period.
        private const double CaRecomputeIntervalSeconds = 0.5;
        private const int CaSampleCount = 240;
        // Horizon for the next-approach search. Must exceed the synodic period of nearly-matched rendezvous
        // orbits (which the old 6 h cap truncated, pinning time-to-CA and never letting it count to zero). The
        // solver early-terminates at the first real approach, so this is only the no-near-approach scan bound.
        private const double CaMaxHorizonSeconds = 24.0 * 3600.0;
        private double _lastCaComputeUt = double.NegativeInfinity;

        // Plan preview refresh throttle (so the panel shows ΔV/CA before the user Executes).
        private const double PreviewIntervalSeconds = 0.5;
        private double _lastPreviewUt = double.NegativeInfinity;

        // Close-approach braking params (decel from TWR, slew lead to flip retro) refreshed on this throttle.
        private const double BrakeParamsIntervalSeconds = 0.5;
        private double _lastBrakeParamsUt = double.NegativeInfinity;

        // Burn-log throttle so a multi-second burn doesn't write megabytes to the glog.
        private const double BurnLogIntervalSeconds = 0.25;
        private double _lastBurnLogUt = double.NegativeInfinity;

        // Orient-then-stabilize-then-burn gate (BurnSettleGate): hold throttle until pointed AND rotation has
        // settled to the craft's control-authority-scaled "still" rate for the dwell, else a burn fired
        // mid-slew flings off-axis (a heavy/powerful craft coasts through alignment, so a fixed rate bound lets
        // it ignite while still turning). Threshold is near-zero for heavy craft, the legacy bound for nimble.
        private BurnSettleTracker _settle;
        private bool _wasExecuting;   // edge-detect entry into a stage burn (for one-shot diagnostics)

        // Warp-to-closest-approach: absolute target UT, auto-stopped a lead short so the craft can pre-orient.
        // The lead is a fixed minimum for arrival-fired stages; for Match Velocity it is the estimated slew to
        // the retro-relative-velocity attitude (+settle/margin).
        private const double WarpLeadMinSeconds = 15.0;    // minimum lead before the event (any stage)
        private const double WarpLeadMaxSeconds = 300.0;   // cap, so a huge slew estimate can't strand the warp
        private const double OrientPaddingSeconds = 3.0;   // safety margin on top of the estimated slew + dwell
        private double _warpTargetUt;
        private double _warpLeadSeconds = WarpLeadMinSeconds;   // lead actually used for the active warp
        private bool _warpingToCa;                              // CA warp (re-targeted live) vs Hohmann ignition warp
        public bool Warping { get; private set; }

        // User floor (seconds) on the warp-stop lead, 0 = auto. The auto lead is slew + half the match burn so
        // the craft can orient and the null straddles the closest approach; this only ever raises it.
        public double WarpLeadInputSeconds { get; set; } = 30.0;

        //public RendezvousPhase Phase => _executor.Phase;
        //public RendezvousStage Stage => _executor.Stage;
        public bool HasInterceptPlan => _executor.HasInterceptPlan; // todo: replace with sharedstate
        public bool HaveHohmannPlan => _executor.HaveHohmannTransfer;
        public RendezvousCommand Command => _command;
        public bool HasCommand => _hasCommand;
        public RelativeState Relative { get; private set; }
        public bool HasRelative { get; private set; }
        public Vessel Target { get; private set; }

        // Live (continuously recomputed) closest approach from the CURRENT state, and the time until it.
        public double LiveClosestApproachMeters { get; private set; } = double.NaN;
        public double LiveTimeToClosestApproachSeconds { get; private set; } = double.NaN;

        // Live CA captured at burn start so POSTBURN-DIAG can report before -> after.
        private double _caBeforeBurnMeters = double.NaN;

        // UI actuation feedback while a burn is commanded: orienting vs thrusting, settling after alignment,
        // and the current attitude error to the burn vector.
        public bool Orienting { get; private set; }
        public bool Stabilizing { get; private set; }
        public double AlignmentErrorDeg { get; private set; } = double.NaN;

        public void ToggleEngage(bool status)
        {
            _engaged = status;
            if (!_engaged) _attitude.Reset();
        }

        // User gates (pass-through to the executor). Executing a stage cancels any warp.
        public bool Execute() {
            StopWarp();
            bool executing = _executor.Execute();
            bbState.RendezvousEnabled = executing;
            return executing;
        }
        // Execute a specific stage out of order (e.g. Match Velocity any time, to kill a closing rate). Preempts
        // whatever was running (warp, intercept, close approach) and forces a fresh orient to the new burn
        // vector so Match Velocity always re-points to the terminal/retrograde direction before firing.
        public bool Execute(RendezvousMethod method) {
            StopWarp();
            _settle.Reset();
            bool executing = _executor.ForceExecute(method);
            bbState.RendezvousEnabled = executing;
            return executing;
        }

        // Close-approach park distance ("match velocities at X m"); the default is restored when the UI
        // option is off so a one-off custom value doesn't persist into the next approach.
        public double ParkingDistanceMeters
        {
            get { return _executor.ParkingDistance; }
            set { _executor.ParkingDistance = value; }
        }
        public bool AutoMatchVelocityDistance
        {
            get { return _executor.UseDistanceForMatchVelocities; }
            set { _executor.UseDistanceForMatchVelocities = value;  }
        }
        public const double CloseStandoffDefaultMeters = TerminalRendezvousExecutor.ParkingDistanceDefaultMeters;

        // Close-approach closing-speed tuning (UI-settable): raise the max speed to close a long-range gap as a
        // few large burns instead of a slow capped crawl. Gain = closing speed per metre of range.
        public double CloseApproachGain
        {
            get { return _executor.RendezDistanceApproachGain; }
            set { _executor.RendezDistanceApproachGain = value; }
        }
        public double CloseApproachMaxSpeedMetersPerSecond
        {
            get { return _executor.RendezMaxApproachSpeedMetersPerSecond; }
            set { _executor.RendezMaxApproachSpeedMetersPerSecond = value; }
        }

        // Invalidate the cached plan preview so it recomputes once (planner switched / manual refresh).
        public const double CloseApproachGainDefault = TerminalRendezvousExecutor.RendezDistanceApproachGainDefault;
        public const double CloseApproachMaxSpeedDefault = TerminalRendezvousExecutor.RendezMaxApproachSpeedDefaultMetersPerSecond;
        public void Abort() { _executor.Abort(); _attitude.Reset(); _settle.Reset(); StopWarp(); bbState.RendezvousEnabled = false; }
        public void ResetSequence() { _executor.Reset(); _attitude.Reset(); _settle.Reset(); StopWarp(); }

        // Stop cleanly after losing the control authority (e.g. Docking Assume Control) mid-stage: drop to
        // idle, release attitude/warp. ActiveModule is already owned by the new module, so it is not touched.
        private void ReleaseControl()
        {
            _executor.Reset();
            _attitude.Reset();
            _settle.Reset();
            StopWarp();
            bbState.RendezvousEnabled = false;
        }

        public void Init(SharedState s) {
            bbState = s;
            warp = new WarpHelper();
            _executor.Init(bbState);
            _executor.Reset();
        }

        // Warp toward the predicted closest approach (shared safe-warp ladder), auto-stopping a lead short
        // and cancelling the moment a burn starts. No-op without a CA estimate.
        public void WarpToClosestApproach()
        {
            if (bbState.InterceptPhase == InterceptPhase.Executing) return;
            double timeToCa = LiveTimeToClosestApproachSeconds;
            double lead = ComputeWarpLeadSeconds();
            if (!MathHelpers.IsFinite(timeToCa) || timeToCa <= lead) return;

            _warpLeadSeconds = lead;
            _warpTargetUt = Planetarium.GetUniversalTime() + timeToCa;
            _warpingToCa = true;
            Warping = true;
        }

        // Warp to a planned transfer ignition UT (Hohmann departs in the future, not at the live CA),
        // stopping a lead short so the craft can orient; the coast-to-ignition gate fires the burn at ignition.
        public void WarpToIgnition(double ignitionUt)
        {
            if (bbState.InterceptPhase == InterceptPhase.Executing && !CoastingToIgnition) return;
            double now = Planetarium.GetUniversalTime();
            double dt = ignitionUt - now;
            double lead = ComputeIgnitionWarpLeadSeconds();
            if (!MathHelpers.IsFinite(dt) || dt <= lead) return;

            _warpLeadSeconds = lead;
            _warpTargetUt = ignitionUt;
            _warpingToCa = false;
            Warping = true;
        }

        // Warp lead before transfer ignition = estimated slew (+margin) plus half the burn duration, so the
        // centered burn straddles the planned ignition. Clamped like the other leads.
        private double ComputeIgnitionWarpLeadSeconds()
        {
            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null || !_executor.HasInterceptPlan) return WarpLeadMinSeconds;
            double padding = OrientPaddingSeconds + BurnSettleGate.StabilizeDwellSeconds;
            double slew = AttitudeControl.EstimateSlewTimeSeconds(active, bbState.InterceptSolution.DeltaV, padding);
            double halfBurn = HalfBurnSeconds(active, bbState.InterceptSolution.DeltaVMagnitude);
            double auto = MathHelpers.Clamp(slew + halfBurn, WarpLeadMinSeconds, WarpLeadMaxSeconds);
            return Math.Max(Math.Max(0.0, WarpLeadInputSeconds), auto);
        }

        // Half the intercept burn duration = 0.5 * ΔV / (thrust/mass). 0 if thrust/mass is unavailable.
        private static double HalfBurnSeconds(Vessel active, double dvMagnitude)
        {
            VesselState vs = VesselState.FromVessel(active);
            if (vs == null || !MathHelpers.IsFinite(vs.AvailableThrust) || vs.AvailableThrust <= 0.0
                || !MathHelpers.IsFinite(vs.TotalMass) || vs.TotalMass <= 0.0) return 0.0;
            double accel = vs.AvailableThrust / vs.TotalMass;
            return accel > 0.0 ? 0.5 * dvMagnitude / accel : 0.0;
        }

        // The Hohmann's frozen future-departure UT once a burn is armed (for the warp-to-ignition button).
        public double PlannedIgnitionUt => _executor.PlannedIgnitionUt;

        // True while the Hohmann intercept is armed and holding for its future ignition — Executing but not
        // yet burning, so warping toward the ignition window is allowed.
        
        public bool CoastingToIgnition =>
            bbState.InterceptPhase == InterceptPhase.Executing
            && bbState.RendezvousMethod == RendezvousMethod.Intercept
            && bbState.InterceptMethod == InterceptMethod.Hohmann
            && _executor.BurnArmed
            && Planetarium.GetUniversalTime() < _executor.PlannedIgnitionUt;

        // Warp lead before the closest approach: a small fixed lead for arrival-fired stages; for Match
        // Velocity the estimated slew to the retro-relative attitude (+settle dwell + margin), clamped.
        private double ComputeWarpLeadSeconds()
        {
            double floor = Math.Max(0.0, WarpLeadInputSeconds);
            if (bbState.RendezvousMethod != RendezvousMethod.MatchVelocity || !HasRelative)
                return Math.Max(floor, WarpLeadMinSeconds);

            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null) return Math.Max(floor, WarpLeadMinSeconds);

            // Burn direction that nulls the relative velocity = (targetVel - activeVel) = RelativeVelocityWorld.
            // Lead = slew to that attitude + half the null-burn duration, so the warp stops with time to orient
            // AND the burn (ignited at CA - halfBurn) straddles the closest approach.
            Vector3d burnDirection = Relative.RelativeVelocityWorld;
            double padding = OrientPaddingSeconds + BurnSettleGate.StabilizeDwellSeconds;
            double slew = AttitudeControl.EstimateSlewTimeSeconds(active, burnDirection, padding);
            double halfBurn = HalfBurnSeconds(active, Relative.RelativeVelocityWorld.magnitude);
            double auto = MathHelpers.Clamp(slew + halfBurn, WarpLeadMinSeconds, WarpLeadMaxSeconds);
            return Math.Max(floor, auto);
        }

        public void StopWarp()
        {
            if (Warping) WarpHelper.Stop();
            Warping = false;
            _warpingToCa = false;
            _warpTargetUt = 0.0;
        }

        // Per-frame tick (from BlackBird.Update): computes the command + relative state and logs while
        // executing. Bounded; does not actuate (that is ApplyFlightControls).
        public void Update(Vessel active, Vessel target)
        {
            Target = target;
            _hasCommand = false;
            HasRelative = false;

            if (active == null || target == null || ReferenceEquals(active, target) || bbState == null)
            {
                StopWarp();
                return;
            }

            // Computed whenever a target exists so the panel can be watched before engaging; CA scan throttled.
            Relative = RelativeState.Compute(active, target);
            HasRelative = true;

            double now = Planetarium.GetUniversalTime();
            if (now - _lastCaComputeUt >= CaRecomputeIntervalSeconds)
            {
                ComputeLiveClosestApproach(active, target);
                _lastCaComputeUt = now;
            }

            // Warp-to-CA monitoring: back off the rate as the event nears, stop just short, bail on a burn.
            if (Warping)
            {
                // Re-target the CA warp to the live (stable) CA UT when we have a fresh estimate, so we stop a
                // proper lead short of the REAL event rather than a frozen snapshot. A momentary NotFound keeps
                // the last good target so the warp doesn't bail and force a manual restart. The Hohmann ignition
                // warp keeps its fixed target.
                if (_warpingToCa && MathHelpers.IsFinite(LiveTimeToClosestApproachSeconds))
                {
                    _warpTargetUt = now + LiveTimeToClosestApproachSeconds;
                    _warpLeadSeconds = ComputeWarpLeadSeconds();
                }

                double secondsToWarpTarget = _warpTargetUt - now;

                // Stop on a burn or at the lead; let the warp run through the Hohmann coast-to-ignition
                // (Executing but not burning) toward the ignition window.
                bool burning = bbState.InterceptPhase == InterceptPhase.Executing && !CoastingToIgnition;
                if (burning || secondsToWarpTarget <= _warpLeadSeconds)
                    StopWarp();
                else
                    warp.BetterWarpToUt(_warpTargetUt, active);
            }
            
            // ActiveModule is the control authority: step + actuate only while we own it. Losing it mid-stage
            // (e.g. Docking Assume Control) stops us cleanly.
            bool owns = bbState.ActiveModule == BlackbirdModule.Rendezvous;
            if (!owns && (bbState.InterceptPhase == InterceptPhase.Executing || bbState.InterceptPhase == InterceptPhase.Coast))
                ReleaseControl();

            // Preview needs a world too; build it when the panel is engaged (planning) or we own control.
            if (!_engaged && !owns) return;

            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);

            if (bbState.InterceptPhase != InterceptPhase.Executing && now - _lastPreviewUt >= PreviewIntervalSeconds)
            {
                // no longer generating a plan this way
                // Feed the ignition-time-drift lead = estimated slew to the burn vector (+settle/margin), so
                // the frozen plan matches the state the engine fires from; refines over successive previews.
                if (bbState.RendezvousMethod == RendezvousMethod.Intercept && _executor.HasInterceptPlan)
                {
                    double padding = OrientPaddingSeconds + BurnSettleGate.StabilizeDwellSeconds;
                    _executor.IgnitionLeadSeconds = AttitudeControl.EstimateSlewTimeSeconds(
                        active, bbState.InterceptSolution.DeltaV, padding);

                    // Feed ship accel so the executor can center the burn (ignite half the burn duration early).
                    VesselState vs = VesselState.FromVessel(active);
                    if (vs != null && MathHelpers.IsFinite(vs.AvailableThrust) && vs.AvailableThrust > 0.0
                        && MathHelpers.IsFinite(vs.TotalMass) && vs.TotalMass > 0.0)
                        _executor.BurnAccelMetersPerSecondSquared = vs.AvailableThrust / vs.TotalMass;
                }
            }

            // Past here we step the executor and drive controls; only when we own control.
            if (!owns) return;

            // Feed the brake its braking-distance inputs: available decel (thrust/mass) and the slew time to flip
            // retrograde-relative. Both the Final Approach stage and the Match-Velocity-at-distance path run the
            // brake/close controller, so feed for either. Throttled; a bad reading keeps the last.
            if ((bbState.RendezvousMethod == RendezvousMethod.FinalApproach
                 || bbState.RendezvousMethod == RendezvousMethod.MatchVelocity)
                && now - _lastBrakeParamsUt >= BrakeParamsIntervalSeconds)
            {
                _lastBrakeParamsUt = now;
                VesselState vs = VesselState.FromVessel(active);
                if (vs != null && MathHelpers.IsFinite(vs.AvailableThrust) && vs.AvailableThrust > 0.0
                    && MathHelpers.IsFinite(vs.TotalMass) && vs.TotalMass > 0.0)
                    _executor.BrakingDecelMetersPerSecondSquared = vs.AvailableThrust / vs.TotalMass;

                Vector3d brakeDir = TrajectoryProvider.GetVelocity(target) - TrajectoryProvider.GetVelocity(active);
                if (brakeDir.sqrMagnitude > 0.0)
                    _executor.BrakingSlewLeadSeconds =
                        AttitudeControl.EstimateSlewTimeSeconds(active, brakeDir, OrientPaddingSeconds);
            }


            // Feed the live predicted CA + time-to-CA so the close-approach stage can decide whether to coast
            // (projection reaches the parking band) or keep closing.
            double closestApproach = MathHelpers.IsFinite(LiveClosestApproachMeters) ? LiveClosestApproachMeters : double.NaN;
            double timeToClosestApproach = MathHelpers.IsFinite(LiveTimeToClosestApproachSeconds)
                ? LiveTimeToClosestApproachSeconds : double.NaN;
            _command = _executor.Update(world, closestApproach, timeToClosestApproach);
            _hasCommand = true;

            // One-shot diagnostic on entering a stage burn: dumps the measured state (SMA-from-state vs stock
            // orbit SMA, plan ΔV) — the frame-consistency check the offline harness can't make.
            bool executingNow = bbState.InterceptPhase == InterceptPhase.Executing;
            if (executingNow && !_wasExecuting) LogExecuteDiagnostic(active, target, world);
            if (!executingNow && _wasExecuting) LogPostBurnDiagnostic(active, target);
            _wasExecuting = executingNow;

            // Throttle the burn log so a multi-second burn doesn't write megabytes.
            if (executingNow && now - _lastBurnLogUt >= BurnLogIntervalSeconds)
            {
                _log.Write(bbState.RendezvousMethod.ToString(), _command, Relative);
                _lastBurnLogUt = now;
            }
        }

        // Logs a consistency snapshot at burn start: |r|, |v|, SMA-from-state vs the stock orbit SMA (mismatch
        // ⇒ inconsistent frames), and the resulting plan. See glog\Blackbird\rendezvous.log.
        private void LogExecuteDiagnostic(Vessel active, Vessel target, IRendezvousWorld world)
        {
            double mu = world.Mu;
            Vector3d aR = world.ActivePosition, aV = world.ActiveVelocity;
            Vector3d tR = world.TargetPosition, tV = world.TargetVelocity;
            InterceptSolution p = bbState.InterceptSolution;

            double aSmaOrbit = active != null && active.orbit != null ? active.orbit.semiMajorAxis : double.NaN;
            double tSmaOrbit = target != null && target.orbit != null ? target.orbit.semiMajorAxis : double.NaN;

            // Snapshot the do-nothing CA now so POSTBURN-DIAG can report before -> after.
            _caBeforeBurnMeters = LiveClosestApproachMeters;

            _log.Write("EXECUTE-DIAG",
                "mu=" + mu.ToString("E5"),
                string.Format("active |r|={0:F1} |v|={1:F2} SMA_state={2:F1} SMA_orbit={3:F1}",
                    aR.magnitude, aV.magnitude, SmaFromState(aR, aV, mu), aSmaOrbit),
                string.Format("target |r|={0:F1} |v|={1:F2} SMA_state={2:F1} SMA_orbit={3:F1}",
                    tR.magnitude, tV.magnitude, SmaFromState(tR, tV, mu), tSmaOrbit),
                string.Format("plan ok={0} dV={1:F2} tof={2:F0} predCA={3:F1}  CA_before={4:F1}",
                    p.Success, p.DeltaVMagnitude, p.TimeOfFlight, p.PredictedClosestApproach, _caBeforeBurnMeters),
                "activeV=" + aV, "planDV=" + p.DeltaV);
        }

        public void GenerateNewInterceptPlan(Vessel active, Vessel target, InterceptMethod method)
        {
            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);
            _executor.RefreshPlanPreview(world, method);
            _lastPreviewUt = Planetarium.GetUniversalTime();
        }

        // Make one of the previewed Hohmann windows the active plan (all candidates are already valid, so the
        // executor's HasInterceptPlan stays true). Execute then fires the chosen window.
        public void SelectInterceptCandidate(int index)
        {
            if (bbState?.InterceptCandidates == null
                || index < 0 || index >= bbState.InterceptCandidates.Count) return;

            bbState.SelectedInterceptCandidateIndex = index;
            bbState.InterceptSolution = bbState.InterceptCandidates[index];
        }

        // Logs how well the intercept burn matched its plan: planned vs delivered ΔV (shortfall + direction
        // error) and predicted vs re-measured achieved CA. No-op unless the stage was an intercept burn.
        private void LogPostBurnDiagnostic(Vessel active, Vessel target)
        {
            if (!_executor.HasLastInterceptBurnReport || active == null || target == null) return;
            InterceptBurnReport r = _executor.LastInterceptBurnReport;

            // Re-measure the closest approach from the post-burn state so we compare like-for-like.
            ComputeLiveClosestApproach(active, target);

            double deliveredTotal = r.DeliveredVector.magnitude;

            // Direction error between actual velocity change and planned ΔV (how much gravity tilted the burn
            // off-axis); small = the frozen-axis burn tracked the plan well.
            double dirErrorDeg = double.NaN;
            if (r.PlannedDvVector.sqrMagnitude > 0.0 && r.DeliveredVector.sqrMagnitude > 0.0)
            {
                double dot = MathHelpers.Clamp(
                    Vector3d.Dot(r.PlannedDvVector.normalized, r.DeliveredVector.normalized), -1.0, 1.0);
                dirErrorDeg = Math.Acos(dot) * 180.0 / Math.PI;
            }

            // Did the burn actually tighten the closest approach? (before -> after, and the delta.)
            double caDelta = LiveClosestApproachMeters - _caBeforeBurnMeters;

            _log.Write("POSTBURN-DIAG",
                string.Format("planned dV={0:F2}  delivered total={1:F2}  velocity residual={2:F2} m/s",
                    r.PlannedDvMagnitude, deliveredTotal, r.VelocityResidual),
                string.Format("delivered axis={0:F2}  dir error={1:F2} deg  cutoff={2}",
                    r.DeliveredAlongAxis, dirErrorDeg, r.CutoffReason),
                string.Format("CA before={0:F1} m  -> achieved CA={1:F1} m  (delta {2:+0;-0} m)  predicted CA={3:F1} m  in {4:F0}s",
                    _caBeforeBurnMeters, LiveClosestApproachMeters, caDelta,
                    r.PredictedClosestApproach, LiveTimeToClosestApproachSeconds));
        }

        // Semi-major axis implied by a state vector (vis-viva); NaN if unbound. Equals the stock orbit SMA
        // when position and velocity are a consistent pair.
        private static double SmaFromState(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;
            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;
            return -mu / (2.0 * energy);
        }

        // Finds the next true closest approach out to the synodic period (capped) so time-to-CA counts down.
        // Off the draw path, throttled.
        private void ComputeLiveClosestApproach(Vessel active, Vessel target)
        {
            CelestialBody body = active.mainBody;
            if (body == null) return;

            double mu = body.gravParameter;
            Vector3d aPos = TrajectoryProvider.GetPosition(active) - body.position;
            Vector3d aVel = TrajectoryProvider.GetVelocity(active);
            Vector3d tPos = TrajectoryProvider.GetPosition(target) - body.position;
            Vector3d tVel = TrajectoryProvider.GetVelocity(target);

            // J2 so the multi-orbit propagation matches the real (oblate) trajectory under RSS/Principia;
            // ob.J2 == 0 in stock, where the solver falls back to the conic path.
            BodyOblateness.Oblateness ob = BodyOblateness.For(body);
            Vector3d pole = ((Vector3d)body.transform.up).normalized;

            ApproachResult approach = ClosestApproachSolver.FindNextApproach(
                aPos, aVel, tPos, tVel, mu, CaMaxHorizonSeconds, CaSampleCount,
                ob.J2, ob.ReferenceRadiusMeters, pole);

            if (approach.Found)
            {
                LiveClosestApproachMeters = approach.DistanceMeters;
                LiveTimeToClosestApproachSeconds = approach.TimeSeconds;
            }
            else
            {
                // No approach within the horizon: clear, don't leave a stale value the panel keeps showing.
                LiveClosestApproachMeters = double.NaN;
                LiveTimeToClosestApproachSeconds = double.NaN;
            }
        }

        // Fly-by-wire actuation (from BlackBird.OnFlyByWire): steers along the burn vector and sets throttle
        // only while burning; cuts throttle on the frame the burn ends, then releases control.
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null || vessel == null || bbState == null) return;

            bool wantBurn = bbState.ActiveModule == BlackbirdModule.Rendezvous && _hasCommand && _command.HasBurn
                            && _command.ThrustDirection.sqrMagnitude > 0.0;

            if (wantBurn)
            {
                // Soft-enable RCS so torque-poor craft can actually rotate during the engine-off orient hold;
                // capture the player's setting once and restore it on handback.
                if (!_rcsForced)
                {
                    _rcsPriorState = vessel.ActionGroups[KSPActionGroup.RCS];
                    _rcsForced = true;
                }
                vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);

                // let prior burn vector finish before doing a new one
                if (!_settle.Aligned && BlackbirdHelpers.EngineThrustActive(vessel))
                {
                    _attitude.DriveInertial(vessel, state, vessel.ReferenceTransform.up, 0.0); // keep vessel on its current orientation while its engines spool down
                    state.mainThrottle = 0.0f;
                    return;
                }

                // Always steer toward the burn vector; throttle only once the craft is pointed and settled
                _attitude.DriveInertial(vessel, state, _command.ThrustDirection, 0.0);

                double errorDeg = AttitudeErrorDeg(vessel, _command.ThrustDirection);
                // Only pitch/yaw rate moves the nose off the burn vector; roll about the thrust axis doesn't,
                // so it is excluded from the settle gate
                // (KSP vessel angular velocity: x=pitch, y=roll, z=yaw.)
                Vector3d angularVel = vessel.angularVelocityD;
                double pitchYawRateDegPerSec =
                    MathHelpers.Rad2Deg(Math.Sqrt(angularVel.x * angularVel.x + angularVel.z * angularVel.z));
                double now = Planetarium.GetUniversalTime();
                AlignmentErrorDeg = errorDeg;

                // "Still" rate scales with the craft's control authority: heavy/powerful craft must settle to
                // near zero (else they ignite while coasting through alignment), nimble craft keep the legacy bound.
                double stillRate = BurnSettleGate.StillRateThresholdDegPerSec(
                    AttitudeControl.MinControlAngularAccel(vessel), TimeWarp.fixedDeltaTime);
                bool aligned = _settle.Update(errorDeg, pitchYawRateDegPerSec, stillRate, now);

                Orienting = !aligned;
                Stabilizing = !aligned && errorDeg <= BurnSettleGate.AlignStartDeg;   // pointed, settling

                if (aligned)
                {
                    state.mainThrottle = Mathf.Clamp01((float)_command.Throttle);
                }
                else
                {
                    // Holding throttle during orient/stabilize: pin the cutoff baseline to current velocity
                    // so orient-phase gravity isn't counted as delivered ΔV.
                    state.mainThrottle = 0.0f;
                    _executor.HoldBurnBaseline(TrajectoryProvider.GetVelocity(vessel), now);
                }

                _burningLastApply = aligned;
                return;
            }

            // Not burning: hand control back, restoring the player's RCS setting we soft-enabled.
            if (_rcsForced)
            {
                vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, _rcsPriorState);
                _rcsForced = false;
            }

            Orienting = false;
            Stabilizing = false;
            AlignmentErrorDeg = double.NaN;
            _settle.Reset();

            if (_burningLastApply)
            {
                state.mainThrottle = 0.0f;   // cut throttle on the first non-burning frame after a burn
                _burningLastApply = false;
            }
        }

        // Angle (degrees) between the craft's current facing (control-reference nose) and the desired
        // world-frame burn direction. Used to gate throttle until aligned.
        private static double AttitudeErrorDeg(Vessel vessel, Vector3d desiredWorldDirection)
        {
            if (vessel.ReferenceTransform == null) return 180.0;

            Vector3d nose = ((Vector3d)vessel.ReferenceTransform.up).normalized;
            Vector3d desired = desiredWorldDirection.normalized;
            double dot = MathHelpers.Clamp(Vector3d.Dot(nose, desired), -1.0, 1.0);
            return Math.Acos(dot) * 180.0 / Math.PI;
        }
    }
}
