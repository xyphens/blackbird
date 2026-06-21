using System;
using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;
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
        private readonly TerminalRendezvousExecutor _executor = new TerminalRendezvousExecutor();
        private readonly AttitudeControl _attitude = new AttitudeControl();
        private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Rendezvous);

        private RendezvousCommand _command;
        private bool _hasCommand;
        private bool _engaged;
        private bool _burningLastApply;   // were we actuating thrust on the previous fly-by-wire pass?

        // Soft-enable RCS during a maneuver so torque-poor craft can still point, restoring the player's
        // setting on handback. Disabled for now.
        //private bool _rcsForcedOn;
        //private bool _rcsPriorState;

        // Live closest-approach monitor: recomputed off the draw path on a throttle, searching out to the
        // synodic period (capped) so time-to-CA actually counts down rather than pinning at one period.
        private const double CaRecomputeIntervalSeconds = 0.5;
        private const int CaSampleCount = 240;
        private const double CaMaxHorizonSeconds = 6.0 * 3600.0;
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

        // Orient-then-stabilize-then-burn gate: hold throttle until pointed (within AlignStartDeg) AND
        // rotation has settled (below MaxAngularRate) for StabilizeDwell, else the burn fires mid-slew and
        // flings off-axis. Once burning, AlignKeepDeg gives hysteresis.
        private const double AlignStartDeg = 2.0;
        private const double AlignKeepDeg = 20.0;
        private const double MaxAngularRateDegPerSec = 1.0;
        private const double StabilizeDwellSeconds = 1.5;
        private bool _burnAligned;
        private double _steadySinceUt = double.NegativeInfinity;
        private bool _wasExecuting;   // edge-detect entry into a stage burn (for one-shot diagnostics)

        // Warp-to-closest-approach: absolute target UT, auto-stopped a lead short so the craft can pre-orient.
        // The lead is a fixed minimum for arrival-fired stages; for Match Velocity it is the estimated slew to
        // the retro-relative-velocity attitude (+settle/margin).
        private const double WarpLeadMinSeconds = 10.0;    // minimum lead before the event (any stage)
        private const double WarpLeadMaxSeconds = 120.0;   // cap, so a huge slew estimate can't strand the warp
        private const double OrientPaddingSeconds = 3.0;   // safety margin on top of the estimated slew + dwell
        private double _warpTargetUt;
        private double _warpLeadSeconds = WarpLeadMinSeconds;   // lead actually used for the active warp
        public bool Warping { get; private set; }

        //public RendezvousPhase Phase => _executor.Phase;
        //public RendezvousStage Stage => _executor.Stage;
        public bool HasInterceptPlan => _executor.HasInterceptPlan; // todo: replace with sharedstate
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
        public bool Execute() { StopWarp(); return _executor.Execute(); }
        // Execute a specific stage out of order (e.g. Match Velocity any time, to kill a closing rate).
        public bool Execute(RendezvousMethod method) { StopWarp(); return _executor.ForceExecute(method); }

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
        public void RequestPlanRefresh() => _executor.RequestPlanRefresh();
        public const double CloseApproachGainDefault = TerminalRendezvousExecutor.RendezDistanceApproachGainDefault;
        public const double CloseApproachMaxSpeedDefault = TerminalRendezvousExecutor.RendezMaxApproachSpeedDefaultMetersPerSecond;
        public void Abort() { _executor.Abort(); _attitude.Reset(); _burnAligned = false; StopWarp(); }
        public void ResetSequence() { _executor.Reset(); _attitude.Reset(); _burnAligned = false; StopWarp(); }

        public void Init(SharedState s) {
            bbState = s;
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
            Warping = true;
        }

        // Warp lead before transfer ignition = estimated slew (+margin) plus half the burn duration, so the
        // centered burn straddles the planned ignition. Clamped like the other leads.
        private double ComputeIgnitionWarpLeadSeconds()
        {
            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null || !_executor.HasInterceptPlan) return WarpLeadMinSeconds;
            double padding = OrientPaddingSeconds + StabilizeDwellSeconds;
            double slew = AttitudeControl.EstimateSlewTimeSeconds(active, bbState.InterceptSolution.DeltaV, padding);
            double halfBurn = HalfBurnSeconds(active, bbState.InterceptSolution.DeltaVMagnitude);
            return MathHelpers.Clamp(slew + halfBurn, WarpLeadMinSeconds, WarpLeadMaxSeconds);
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
            if (bbState.RendezvousMethod != RendezvousMethod.MatchVelocity || !HasRelative) return WarpLeadMinSeconds;

            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null) return WarpLeadMinSeconds;

            // Burn direction that nulls the relative velocity = (targetVel - activeVel) = RelativeVelocityWorld.
            Vector3d burnDirection = Relative.RelativeVelocityWorld;
            double padding = OrientPaddingSeconds + StabilizeDwellSeconds;
            double slew = AttitudeControl.EstimateSlewTimeSeconds(active, burnDirection, padding);
            return MathHelpers.Clamp(slew, WarpLeadMinSeconds, WarpLeadMaxSeconds);
        }

        public void StopWarp()
        {
            if (Warping) WarpHelper.Stop();
            Warping = false;
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
                double secondsToWarpTarget = _warpTargetUt - now;
                // Stop on a burn or at the lead; let the warp run through the Hohmann coast-to-ignition
                // (Executing but not burning) toward the ignition window.
                bool burning = bbState.InterceptPhase == InterceptPhase.Executing && !CoastingToIgnition;
                if (burning || secondsToWarpTarget <= _warpLeadSeconds)
                    StopWarp();
                else
                    WarpHelper.SetSafeWarpRate(secondsToWarpTarget);
            }
            
            if (!_engaged) return;

            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);

            // Keep a fresh plan preview for the pending stage so the panel shows ΔV/CA before Execute.
            if (bbState.InterceptPhase != InterceptPhase.Executing && now - _lastPreviewUt >= PreviewIntervalSeconds)
            {
                _executor.RefreshPlanPreview(world);
                _lastPreviewUt = now;

                // Feed the ignition-time-drift lead = estimated slew to the burn vector (+settle/margin), so
                // the frozen plan matches the state the engine fires from; refines over successive previews.
                if (bbState.RendezvousMethod == RendezvousMethod.Intercept && _executor.HasInterceptPlan)
                {
                    double padding = OrientPaddingSeconds + StabilizeDwellSeconds;
                    _executor.IgnitionLeadSeconds = AttitudeControl.EstimateSlewTimeSeconds(
                        active, bbState.InterceptSolution.DeltaV, padding);

                    // Feed ship accel so the executor can center the burn (ignite half the burn duration early).
                    VesselState vs = VesselState.FromVessel(active);
                    if (vs != null && MathHelpers.IsFinite(vs.AvailableThrust) && vs.AvailableThrust > 0.0
                        && MathHelpers.IsFinite(vs.TotalMass) && vs.TotalMass > 0.0)
                        _executor.BurnAccelMetersPerSecondSquared = vs.AvailableThrust / vs.TotalMass;
                }
            }

            // Feed the close-approach brake its braking-distance inputs: available decel (thrust/mass) and the
            // slew time to flip retrograde-relative. Throttled, close stage only; a bad reading keeps the last.
            if (bbState.RendezvousMethod == RendezvousMethod.CloseApproach && now - _lastBrakeParamsUt >= BrakeParamsIntervalSeconds)
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

            ApproachResult approach = ClosestApproachSolver.FindNextApproach(
                aPos, aVel, tPos, tVel, mu, CaMaxHorizonSeconds, CaSampleCount);

            if (approach.Found)
            {
                LiveClosestApproachMeters = approach.DistanceMeters;
                LiveTimeToClosestApproachSeconds = approach.TimeSeconds;
            }
        }

        // Fly-by-wire actuation (from BlackBird.OnFlyByWire): steers along the burn vector and sets throttle
        // only while burning; cuts throttle on the frame the burn ends, then releases control.
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null || vessel == null || bbState == null) return;

            bool wantBurn = _engaged && _hasCommand && _command.HasBurn
                            && _command.ThrustDirection.sqrMagnitude > 0.0;

            if (wantBurn)
            {
                // Always steer toward the burn vector; throttle only once the craft is pointed and settled
                _attitude.DriveInertial(vessel, state, _command.ThrustDirection, 0.0);

                double errorDeg = AttitudeErrorDeg(vessel, _command.ThrustDirection);
                // Only pitch/yaw rate moves the nose off the burn vector; roll about the thrust axis doesn't,
                // so it is excluded from the settle gate
                // (KSP vessel angular velocity: x=pitch, y=roll, z=yaw.)
                Vector3d angularVel = vessel.angularVelocityD;
                double pitchYawRateDegPerSec =
                    Math.Sqrt(angularVel.x * angularVel.x + angularVel.z * angularVel.z) * (180.0 / Math.PI);
                double now = Planetarium.GetUniversalTime();
                AlignmentErrorDeg = errorDeg;

                if (!_burnAligned)
                {
                    bool steady = errorDeg <= AlignStartDeg && pitchYawRateDegPerSec <= MaxAngularRateDegPerSec;
                    if (steady)
                    {
                        if (double.IsNegativeInfinity(_steadySinceUt)) _steadySinceUt = now;
                        if (now - _steadySinceUt >= StabilizeDwellSeconds) _burnAligned = true;
                    }
                    else
                    {
                        _steadySinceUt = double.NegativeInfinity;   // moved/rotating: restart the dwell
                    }
                }
                else if (errorDeg > AlignKeepDeg)
                {
                    _burnAligned = false;
                    _steadySinceUt = double.NegativeInfinity;
                }

                Orienting = !_burnAligned;
                Stabilizing = !_burnAligned && errorDeg <= AlignStartDeg;   // pointed, settling

                if (_burnAligned)
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

                _burningLastApply = _burnAligned;
                return;
            }

            Orienting = false;
            Stabilizing = false;
            AlignmentErrorDeg = double.NaN;
            _burnAligned = false;
            _steadySinceUt = double.NegativeInfinity;

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
