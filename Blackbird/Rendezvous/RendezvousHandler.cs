using System;
using Blackbird.Docking;
using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Coordinates the terminal-rendezvous executor with the live vessel — the rendezvous-phase analogue
    // of LaunchHandler. Each frame it builds the world seam, runs the executor, caches the resulting
    // command + relative state, and logs while burning; on the fly-by-wire pass it actuates steering and
    // throttle, but ONLY while a stage is actively burning, so it never fights the player otherwise.
    // Arm/Trigger/Abort/Reset are the user gates surfaced to the UI. A master Engage toggle keeps the
    // whole thing dormant until the user opts in.
    public sealed class RendezvousHandler
    {
        private readonly TerminalRendezvousExecutor _executor = new TerminalRendezvousExecutor();
        private readonly AttitudeControl _attitude = new AttitudeControl();
        private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Rendezvous);

        private RendezvousCommand _command;
        private bool _hasCommand;
        private bool _engaged;
        private bool _burningLastApply;   // were we actuating thrust on the previous fly-by-wire pass?

        // Soft-enable RCS: while we are actively driving a maneuver we turn RCS on so torque-poor craft can
        // still point (reaction wheels aren't guaranteed); when we hand control back we RESTORE the player's
        // prior RCS setting so we don't permanently override their preference.
        // DU: disabled for now
        //private bool _rcsForcedOn;        // are we currently holding RCS on for a maneuver?
        //private bool _rcsPriorState;      // the player's RCS setting captured when we took control
        private bool _principiaProbed;    // ran the one-shot Principia compatibility probe this engage?

        // Live closest-approach monitor: recomputed off the draw path on a throttle so it can be watched
        // collapse during/after a burn, independent of any plan. Searches out to the synodic period
        // (capped) so the time-to-CA is real and decreases, instead of pinning at one orbital period.
        private const double CaRecomputeIntervalSeconds = 0.5;
        private const int CaSampleCount = 240;
        private const double CaMaxHorizonSeconds = 6.0 * 3600.0;   // cap the synodic search at 6 hours
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

        // Orient-then-stabilize-then-burn gate: hold throttle until the craft is BOTH pointed along the
        // burn vector (within AlignStartDeg) AND has stopped rotating (rate below MaxAngularRate), held
        // for StabilizeDwell so it has truly settled — otherwise it fires mid-slew and flings the burn
        // off-axis. Once burning, keep throttle while within the looser AlignKeepDeg (hysteresis).
        private const double AlignStartDeg = 2.0;
        private const double AlignKeepDeg = 20.0;
        private const double MaxAngularRateDegPerSec = 1.0;
        private const double StabilizeDwellSeconds = 1.5;
        private bool _burnAligned;
        private double _steadySinceUt = double.NegativeInfinity;
        private bool _wasExecuting;   // edge-detect entry into a stage burn (for one-shot diagnostics)

        // Docking RCS translation: map the world-frame velocity error onto the control-transform axes and
        // command RCS. Gain converts m/s of error to translation input (saturates fast — RCS is near
        // bang-bang); the deadband stops chatter. The Translate*Sign constants are the FlightCtrlState axis
        // polarity — VERIFY IN-GAME and flip any that come out mirrored (control-frame sign conventions vary).
        private const double DockTranslationGain = 5.0;
        private const float DockTranslationDeadband = 0.05f;
        private const float TranslateRightSign = 1.0f;   // state.X (+ = right)
        private const float TranslateUpSign = 1.0f;      // state.Y (+ = dorsal/up)
        private const float TranslateFwdSign = 1.0f;     // state.Z (+ = toward the controlled port / nose)
        private Vector3d _lastTargetVelocityWorld = Vector3d.zero;   // target velocity, refreshed each engaged tick

        // Warp-to-closest-approach (user convenience). Absolute target UT; auto-stops a lead time short of
        // the event so the craft can pre-orient. The lead is fixed for stages that fire on arrival, but for
        // Match Velocity it is the estimated time to slew to the retro-relative-velocity attitude (plus a
        // settle/safety margin), so the craft is pointed and settled by the time it reaches the approach.
        private const double WarpLeadMinSeconds = 10.0;    // minimum lead before the event (any stage)
        private const double WarpLeadMaxSeconds = 120.0;   // cap, so a huge slew estimate can't strand the warp
        private const double OrientPaddingSeconds = 3.0;   // safety margin on top of the estimated slew + dwell
        private double _warpTargetUt;
        private double _warpLeadSeconds = WarpLeadMinSeconds;   // lead actually used for the active warp
        public bool Warping { get; private set; }

        public bool Engaged => _engaged;
        public RendezvousPhase Phase => _executor.Phase;
        public RendezvousStage Stage => _executor.Stage;
        public bool HasInterceptPlan => _executor.HasInterceptPlan;
        public InterceptSolution InterceptPlan => _executor.InterceptPlan;
        public RendezvousCommand Command => _command;
        public bool HasCommand => _hasCommand;
        public RelativeState Relative { get; private set; }
        public bool HasRelative { get; private set; }
        public Vessel Target { get; private set; }

        // Live (continuously recomputed) closest approach from the CURRENT state, and the time until it.
        public double LiveClosestApproachMeters { get; private set; } = double.NaN;
        public double LiveTimeToClosestApproachSeconds { get; private set; } = double.NaN;

        // Live CA captured at the instant a burn is executed, so POSTBURN-DIAG can report before -> after
        // (whether the burn actually tightened the closest approach). Set in LogExecuteDiagnostic.
        private double _caBeforeBurnMeters = double.NaN;

        // Actuation feedback for the UI: while a burn is commanded, whether we are still orienting (true)
        // or actually thrusting (false), whether we are aligned but waiting to settle (Stabilizing), and
        // the current attitude error to the burn vector.
        public bool Orienting { get; private set; }
        public bool Stabilizing { get; private set; }
        public double AlignmentErrorDeg { get; private set; } = double.NaN;

        // Master enable. Disengaging also drops attitude control so the player regains the craft.
        public void Engage() { _engaged = true; }
        public void Disengage() { _engaged = false; _attitude.Reset(); _principiaProbed = false; }

        // User gates (pass-through to the executor). Executing a stage cancels any warp.
        public bool Execute() { StopWarp(); return _executor.Execute(); }
        // Execute a specific stage out of order (e.g. Match Velocity any time, to kill a closing rate).
        public bool Execute(RendezvousStage stage) { StopWarp(); return _executor.Execute(stage); }
        // Docking gate: start docking (resets to the Approach leg) or resume the next queued leg from Coast.
        public bool ExecuteDocking() { StopWarp(); return _executor.ExecuteDocking(); }
        public DockingLeg DockingLeg => _executor.DockingLeg;

        // Close-approach park distance ("match velocities at X m"). The default is restored when the UI
        // option is off, so a one-off custom distance never silently persists into the next approach.
        public double ParkingDistanceMeters
        {
            get { return _executor.ParkingDistance; }
            set { _executor.ParkingDistance = value; }
        }
        public bool AutoMatchVelocityDistance
        {
            get { return _executor.UseMatchVelocitiesDuringApproach; }
            set { _executor.UseMatchVelocitiesDuringApproach = value;  }
        }
        public const double CloseStandoffDefaultMeters = TerminalRendezvousExecutor.ParkingDistanceDefaultMeters;
        public void Abort() { _executor.Abort(); _attitude.Reset(); _burnAligned = false; StopWarp(); }
        public void ResetSequence() { _executor.Reset(); _attitude.Reset(); _burnAligned = false; StopWarp(); }

        // Warp toward the predicted closest approach using the shared safe-warp ladder; auto-stops a few
        // seconds short (and is cancelled the moment a burn starts). No-op if there is no CA estimate yet.
        public void WarpToClosestApproach()
        {
            if (_executor.Phase == RendezvousPhase.Executing) return;
            double timeToCa = LiveTimeToClosestApproachSeconds;
            double lead = ComputeWarpLeadSeconds();
            if (!MathHelpers.IsFinite(timeToCa) || timeToCa <= lead) return;

            _warpLeadSeconds = lead;
            _warpTargetUt = Planetarium.GetUniversalTime() + timeToCa;
            Warping = true;
        }

        // The lead time to stop the warp short of the predicted closest approach. Stages that fire on
        // arrival need only a small fixed lead; Match Velocity must be pointed retrograde-to-relative
        // before it can burn, so it leaves the estimated slew time (from torque/MOI, via AttitudeControl)
        // plus the settle dwell and a safety margin. Clamped so it always leaves at least the minimum and
        // never an unbounded amount.
        private double ComputeWarpLeadSeconds()
        {
            if (_executor.Stage != RendezvousStage.MatchVelocity || !HasRelative)
                return WarpLeadMinSeconds;

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

        // Per-frame tick (from BlackBird.Update). Computes the command + relative state and logs while
        // executing. Bounded work; does not actuate (that is ApplyFlightControls).
        public void Update(Vessel active, Vessel target)
        {
            Target = target;
            _hasCommand = false;
            HasRelative = false;

            if (active == null || target == null || ReferenceEquals(active, target))
            {
                StopWarp();
                return;
            }

            // Relative state + live closest approach are computed whenever a target exists, so the panel
            // can be monitored before engaging. The CA scan is throttled to keep it cheap.
            Relative = RelativeState.Compute(active, target);
            HasRelative = true;

            double now = Planetarium.GetUniversalTime();
            if (now - _lastCaComputeUt >= CaRecomputeIntervalSeconds)
            {
                ComputeLiveClosestApproach(active, target);
                _lastCaComputeUt = now;
            }

            // Warp-to-CA monitoring: back off the rate as the event nears, stop just short, and bail if
            // a burn starts.
            if (Warping)
            {
                double secondsToWarpTarget = _warpTargetUt - now;
                if (_executor.Phase == RendezvousPhase.Executing || secondsToWarpTarget <= _warpLeadSeconds)
                    StopWarp();
                else
                    WarpHelper.SetSafeWarpRate(secondsToWarpTarget);
            }

            if (!_engaged) return;

            // One-shot Principia compatibility probe (logs to compatibility.log) the first engaged tick we
            // have a target — exploratory, for reviewing whether the n-body CA API is reachable. Never throws.
            if (!_principiaProbed && target != null)
            {
                Compatibility.Principia.Probe(active, target);
                _principiaProbed = true;
            }

            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);

            // Keep a fresh plan preview for the pending stage so the panel shows ΔV/CA before Execute.
            if (_executor.Phase != RendezvousPhase.Executing && now - _lastPreviewUt >= PreviewIntervalSeconds)
            {
                _executor.RefreshPlanPreview(world);
                _lastPreviewUt = now;

                // Feed the executor an ignition-time-drift lead = estimated time to orient to the burn
                // vector (+settle/margin), so the frozen plan matches the state the engine actually fires
                // from. Uses the current preview ΔV direction; refines over successive previews as the plan
                // settles. (See TerminalRendezvousExecutor.PlanIntercept.)
                if (_executor.Stage == RendezvousStage.Intercept && _executor.HasInterceptPlan)
                {
                    double padding = OrientPaddingSeconds + StabilizeDwellSeconds;
                    _executor.IgnitionLeadSeconds = AttitudeControl.EstimateSlewTimeSeconds(
                        active, _executor.InterceptPlan.DeltaV, padding);
                }
            }

            // Feed the close-approach brake its braking-distance inputs from the live craft: available
            // deceleration (thrust / mass) and the time to flip to a retrograde-relative attitude. Throttled,
            // and only while the close stage is current. Guarded so a bad reading just keeps the last/default.
            if (_executor.Stage == RendezvousStage.CloseApproach && now - _lastBrakeParamsUt >= BrakeParamsIntervalSeconds)
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

            // Cache the target velocity for the docking RCS controller (which runs on the fly-by-wire pass
            // where only the active vessel is in hand).
            _lastTargetVelocityWorld = TrajectoryProvider.GetVelocity(target);

            // Feed the docking stage the live port transforms: the target docking port (the operator targets
            // it directly) and the chaser's own port (the part being "Controlled From"). Invalid until both
            // are present, in which case the stage idles with operator guidance.
            if (_executor.Stage == RendezvousStage.Docking)
            {
                bool portsValid = TryGetDockingPorts(active, out PortState chaserPort, out PortState targetPort);
                _executor.SetDockingPorts(portsValid, chaserPort, targetPort);
            }

            // Feed the executor the live predicted CA + time-to-CA so the close-approach stage can decide
            // when to coast (projection reaches the parking band) vs keep closing.
            double closestApproach = MathHelpers.IsFinite(LiveClosestApproachMeters) ? LiveClosestApproachMeters : double.NaN;
            double timeToClosestApproach = MathHelpers.IsFinite(LiveTimeToClosestApproachSeconds)
                ? LiveTimeToClosestApproachSeconds : double.NaN;
            _command = _executor.Update(world, closestApproach, timeToClosestApproach);
            _hasCommand = true;

            // One-shot diagnostic on entering a stage burn: dumps the measured state so we can confirm
            // in-game whether position/velocity are a consistent two-body state (SMA-from-state vs the
            // stock orbit SMA) and whether the plan ΔV is sane. This is the decisive frame check the
            // offline harness can't make.
            bool executingNow = _executor.Phase == RendezvousPhase.Executing;
            if (executingNow && !_wasExecuting) LogExecuteDiagnostic(active, target, world);
            if (!executingNow && _wasExecuting) LogPostBurnDiagnostic(active, target);
            _wasExecuting = executingNow;

            // Throttle the per-frame burn log so a multi-second burn doesn't write megabytes.
            if (executingNow && now - _lastBurnLogUt >= BurnLogIntervalSeconds)
            {
                _log.Write(_executor.Stage.ToString(), _command, Relative);
                _lastBurnLogUt = now;
            }
        }

        // Logs a consistency snapshot at burn start: |r|, |v|, the semi-major axis implied by the state
        // vector vs the stock orbit's SMA (mismatch ⇒ position/velocity are in inconsistent frames), and
        // the resulting plan. Read it from glog\Blackbird\rendezvous.log..
        private void LogExecuteDiagnostic(Vessel active, Vessel target, IRendezvousWorld world)
        {
            double mu = world.Mu;
            Vector3d aR = world.ActivePosition, aV = world.ActiveVelocity;
            Vector3d tR = world.TargetPosition, tV = world.TargetVelocity;
            InterceptSolution p = _executor.InterceptPlan;

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

        // Logs how well the just-finished intercept burn matched its plan: planned vs delivered ΔV
        // (magnitude shortfall + direction error) and the plan's predicted CA vs the freshly re-measured
        // achieved CA. This is the decisive over/under-burn + execution-error data; read it from
        // glog\Blackbird\rendezvous.log. No-op unless the completed stage was an intercept burn.
        private void LogPostBurnDiagnostic(Vessel active, Vessel target)
        {
            if (!_executor.HasLastInterceptBurnReport || active == null || target == null) return;
            InterceptBurnReport r = _executor.LastInterceptBurnReport;

            // Re-measure the closest approach from the post-burn state so we compare like-for-like.
            ComputeLiveClosestApproach(active, target);

            double deliveredTotal = r.DeliveredVector.magnitude;

            // Direction error between the actual velocity change and the planned ΔV (shows how much gravity
            // tilted the burn off the planned axis). Small = the frozen-axis burn tracked the plan well.
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

        // Semi-major axis implied by a state vector (vis-viva); NaN if unbound. Should equal the stock
        // orbit's SMA when position and velocity are a consistent pair.
        private static double SmaFromState(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;
            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;
            return -mu / (2.0 * energy);
        }

        // Finds the next true closest approach (out to the synodic period, capped), so the reported
        // time-to-CA is real and counts down instead of pinning at one orbital period. Called off the
        // draw path on a throttle.
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

        // Fly-by-wire actuation (from BlackBird.OnFlyByWire). Steers along the burn vector and sets
        // throttle only while burning; cuts throttle once on the frame the burn ends, then releases
        // control back to the player.
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null || vessel == null) return;

            // Docking uses 6-DOF RCS translation (hold heading + translate), not the main-engine orient/burn
            // path below. Branch out before any of that logic so the two never interfere.
            if (_engaged && _hasCommand && _command.Stage == RendezvousStage.Docking && _command.HasTranslation
                && _command.ThrustDirection.sqrMagnitude > 0.0)
            {
                ApplyDockingControls(state, vessel);
                return;
            }

            bool wantBurn = _engaged && _hasCommand && _command.HasBurn
                            && _command.ThrustDirection.sqrMagnitude > 0.0;

            if (wantBurn)
            {
                // Soft-enable RCS for the duration of the maneuver (capture the player's setting once, on the
                // rising edge, so we can restore it when we release control below).
                //if (!_rcsForcedOn)
                //{
                //    _rcsPriorState = vessel.ActionGroups[KSPActionGroup.RCS];
                //    vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                //    _rcsForcedOn = true;
                //}

                // Always steer toward the burn vector; throttle only once the craft is pointed AND has
                // settled (low rotation rate) and held that for the dwell — orient, stabilize, then burn.
                _attitude.DriveInertial(vessel, state, _command.ThrustDirection, 0.0);

                double errorDeg = AttitudeErrorDeg(vessel, _command.ThrustDirection);
                // Only the pitch/yaw angular rate moves the nose off the burn vector; roll (about the
                // thrust axis) does not affect a thrust-along-nose maneuver. Excluding roll from the
                // settle gate means a craft that is slow to null roll no longer delays a time-sensitive
                // burn. (KSP vessel angular velocity: x=pitch, y=roll, z=yaw.)
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
                    // Holding throttle while we orient/stabilize: pin the executor's cutoff baseline to
                    // the current velocity so gravity during the orient isn't mistaken for delivered ΔV.
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

            // Maneuver done / control released: restore the player's prior RCS setting.
            //if (_rcsForcedOn)
            //{
            //    vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, _rcsPriorState);
            //    _rcsForcedOn = false;
            //}
        }

        // Docking actuation (6-DOF): hold the mated heading with the attitude controller and drive RCS
        // translation so the chaser's relative velocity tracks the commanded approach velocity. RCS is forced
        // on for the maneuver. No main engine. Validated in-game (the offline harness has no actuation path).
        private void ApplyDockingControls(FlightCtrlState state, Vessel vessel)
        {
            if (vessel.ReferenceTransform == null) return;
            vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);

            // Hold the head-on heading so the chaser port faces the target port; no throttle in docking.
            _attitude.DriveInertial(vessel, state, _command.ThrustDirection, 0.0);
            state.mainThrottle = 0.0f;
            AlignmentErrorDeg = AttitudeErrorDeg(vessel, _command.ThrustDirection);
            Orienting = false;
            Stabilizing = false;
            _burningLastApply = false;

            // Velocity error between the commanded approach velocity and our actual relative velocity.
            Vector3d relVel = TrajectoryProvider.GetVelocity(vessel) - _lastTargetVelocityWorld;
            Vector3d velError = _command.TranslationVelocityWorld - relVel;

            // Map the world-frame error onto the control-transform axes. For the controlled part: up = the
            // outward port/nose axis, right = starboard, forward = the belly ("down"). KSP FlightCtrlState
            // translation: X = right, Y = dorsal/up, Z = forward (toward the nose/port).
            Transform rt = vessel.ReferenceTransform;
            double x = TranslateRightSign * Vector3d.Dot(velError, rt.right) * DockTranslationGain;
            double y = TranslateUpSign * Vector3d.Dot(velError, -(Vector3d)rt.forward) * DockTranslationGain;
            double z = TranslateFwdSign * Vector3d.Dot(velError, rt.up) * DockTranslationGain;

            state.X = TranslationInput((float)x);
            state.Y = TranslationInput((float)y);
            state.Z = TranslationInput((float)z);
        }

        // Clamps an RCS translation command to [-1, 1] and zeroes it inside the deadband (anti-chatter).
        private static float TranslationInput(float value)
        {
            if (value > DockTranslationDeadband) return Mathf.Clamp(value, -1.0f, 1.0f);
            if (value < -DockTranslationDeadband) return Mathf.Clamp(value, -1.0f, 1.0f);
            return 0.0f;
        }

        // Extracts the world-frame docking-port transforms for the docking stage. The TARGET port comes from
        // the current target object (the operator targets the docking port itself — it is an ITargetable); its
        // forward vector is the outward approach axis. The CHASER port is the active vessel's control reference
        // (the operator does "Control From Here" on their port, so ReferenceTransform.up is its outward axis).
        // Returns false if either is unavailable, so the executor can prompt the operator.
        private bool TryGetDockingPorts(Vessel active, out PortState chaserPort, out PortState targetPort)
        {
            chaserPort = default(PortState);
            targetPort = default(PortState);
            if (active == null || active.ReferenceTransform == null) return false;

            ITargetable tgt = FlightGlobals.fetch != null ? FlightGlobals.fetch.VesselTarget : null;
            ModuleDockingNode targetNode = tgt as ModuleDockingNode;
            if (targetNode == null || targetNode.GetTransform() == null) return false;

            targetPort = new PortState(targetNode.GetTransform().position, targetNode.GetFwdVector());
            chaserPort = new PortState(active.ReferenceTransform.position, active.ReferenceTransform.up);
            return true;
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
