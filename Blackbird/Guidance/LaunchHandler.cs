using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Psg;
using Blackbird.Trajectory;
using System;

namespace Blackbird.Guidance
{
    public sealed class LaunchHandler
    {
        private const double WarpStopLeadTimeSeconds = 10.0;
        private double _targetUt;

        SharedState bbState;
        WarpHelper warp;

        private readonly AttitudeControl _attitudeControl = new AttitudeControl();

        // Coast attitude-hold deadband: while coasting (≈zero throttle), once the craft is essentially pointed
        // at the hold direction and barely rotating, STOP actively driving the attitude PID — otherwise it
        // nulls infinitesimal error every frame and fires continuous tiny RCS bursts the whole coast. Latched
        // with hysteresis (enter tight, release only after drifting past the wider exit angle) so it doesn't
        // chatter at the edge. Only suppressed at ≈zero throttle, so a powered burn keeps tight, continuous
        // attitude control. (Scoped here, NOT in the shared AttitudeControl, so it never affects burns/docking.)
        private const double CoastThrottleEpsilon = 0.01;        // ≤ this throttle counts as coasting
        private const double CoastHoldEnterDeg = 0.15;           // enter the hold below this pointing error
        private const double CoastHoldExitDeg = 0.40;            // ...and leave it only past this (hysteresis)
        private const double CoastHoldMaxRateDegPerSec = 0.10;   // ...and only when nearly stopped rotating
        private bool _coastAttitudeHeld;
        // Set once at insertion cutoff: after the engine is cut we stop overriding the throttle so the
        // operator regains manual control (continuously forcing throttle=0 every frame locked them out).
        private bool _completionControlReleased;

        // manual guidance
        public double ManualPitchCommandDeg { get; private set; } = 90.0;
        public double ManualHeadingCommandDeg { get; private set; } = 90.0;
        public double ManualThrottleCommand { get; private set; } = 0.0;
        public double ManualRollCommand { get; private set; } = 0.0;
        // min v-speed to pitch
        private double _minVSpd = 100.0;
        public string MinVSpeedToPitch
        {
            get { return _minVSpd.ToString(); }
            set { if (double.TryParse(value, out double v)) _minVSpd = v; }
        }

        private readonly double _launchTowerClearance = 175.0; // starship's launch tower is ~150 meters

        private double _minAltToPitch = 0.0;
        public string MinAltitudeForPitch
        {
            get { return _minAltToPitch.ToString(); }
            set { if (double.TryParse(value, out double v)) _minAltToPitch = Math.Round(v, 0); }
        }

        // read inputs from Blackbird?
        public GuidanceMode GuidanceMode { get; set; } = GuidanceMode.None;
        public LaunchGuidanceState State { get; private set; }
        public Vessel TargetVessel { get; private set; }

        // Set the rendezvous target independent of plan construction — the LaunchPlanner.Create path builds the
        // plan without going through ConstructLaunchPlan, so the committer sets the target here so the GC plane
        // readout (rel-inc / RAAN vs target) has it.
        public void SetTargetVessel(Vessel target) => TargetVessel = target;
        private readonly AscentGuidance _ascentGuidance = new AscentGuidance();
        
        public void Init(SharedState s)
        {
            if (s == null || FlightGlobals.ActiveVessel == null) return;
            warp = new WarpHelper();
            bbState = s;
            State = s.LaunchPlan != null ? LaunchGuidanceState.PlanReady : LaunchGuidanceState.Idle;

            double lpAlt = FlightGlobals.ActiveVessel.situation == Vessel.Situations.PRELAUNCH
                || FlightGlobals.ActiveVessel.situation == Vessel.Situations.LANDED ?
                FlightGlobals.ActiveVessel.altitude : 0.0;

            _minAltToPitch = _launchTowerClearance + lpAlt;

            RefreshGuidanceComputer();
            _ascentGuidance.Reset();
        }
        
        public AscentGuidanceInfo GuidanceInfo { get; private set; }
        public PsgSolution CurrentSolution => _ascentGuidance.CurrentSolution;

        public readonly AscentRecorder AscentReport = new AscentRecorder();
        public double SecondsUntilLaunch
        {
            get
            {
                if (_targetUt <= 0.0) return 0.0;
                return Math.Max(0.0, _targetUt - Planetarium.GetUniversalTime());
            }
        }
        public void AcceptPlan()
        {
            if (bbState.LaunchPlan == null) return;

            State = LaunchGuidanceState.PlanAccepted;
            // close launch planner and open guidance
            bbState.PlannerVisible = false;
            bbState.GuidanceVisible = true;
            bbState.ActiveModule = BlackbirdModule.LaunchGuidance;
            bbState.GuidanceState = State;
        }
        public void WarpToLaunch()
        {
            if ((State != LaunchGuidanceState.AwaitingLaunch && State != LaunchGuidanceState.PlanAccepted)
                || bbState.LaunchPlan == null) return;

            LaunchCandidate selectedCandidate = bbState.LaunchPlan.SelectedCandidate;
            if (selectedCandidate == null || !selectedCandidate.IsValid) return;

            double liveTimeToLaunch = selectedCandidate.LaunchUt - Planetarium.GetUniversalTime();

            // already close to launch time
            if (liveTimeToLaunch <= WarpStopLeadTimeSeconds)
            {
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            _targetUt = selectedCandidate.LaunchUt;

            State = LaunchGuidanceState.WarpingToLaunch;
        }

        private void RefreshGuidanceComputer()
        {
            _ascentGuidance.Refresh(bbState.IsRSS, bbState.IsPrincipia, _minAltToPitch, _minVSpd);
        }
        //private static void SetSafeWarpRate(double toUt, bool isRss) => WarpHelper.SetSafeWarpRate(secondsRemaining, isRss);
        private void WarpToUT(double UT, Vessel vessel)
        {
            if (warp == null) return;
            warp.BetterWarpToUt(UT, vessel, false);
        }

        // "Arm" the launch: reveals Warp To Launch + the flight-mode selector. No flying yet — choosing a
        // flight mode (BeginAscent) is what starts the ascent.
        public void StartGuidance()
        {
            if (State != LaunchGuidanceState.PlanAccepted) return;
            if (bbState.LaunchPlan == null || bbState.LaunchPlan.SelectedCandidate == null || !bbState.LaunchPlan.SelectedCandidate.IsValid) return;
            _targetUt = bbState.LaunchPlan?.SelectedCandidate?.LaunchUt ?? 0;
            bbState.ActiveModule = BlackbirdModule.LaunchGuidance;
            RefreshGuidanceComputer();
            State = LaunchGuidanceState.AwaitingLaunch;
        }

        // Begin the ascent once a flight mode is chosen while armed (stops any launch warp first).
        public void BeginAscent()
        {
            if (State == LaunchGuidanceState.PlanAccepted) StartGuidance();
            if (State != LaunchGuidanceState.AwaitingLaunch && State != LaunchGuidanceState.WarpingToLaunch) return;
            TimeWarp.SetRate(0, true);
            _ascentGuidance.Reset();
            AscentReport.Reset();
            _completionControlReleased = false;
            State = LaunchGuidanceState.GuidingAscent;
        }

        public void Update(Vessel vessel)
        {
            if (State == LaunchGuidanceState.Idle ||
                State == LaunchGuidanceState.Complete ||
                State == LaunchGuidanceState.Aborted)
            {
                // release module if guidance completed successfully or was aborted
                if (State != LaunchGuidanceState.Idle 
                    && bbState.ActiveModule == BlackbirdModule.LaunchGuidance) bbState.ActiveModule = BlackbirdModule.None;
                return;
            }

            if (State != LaunchGuidanceState.WarpingToLaunch) return;

            // monitor / adjust warp rate
            double nowUt = Planetarium.GetUniversalTime();

            double secondsRemaining = _targetUt - nowUt;

            if (secondsRemaining <= WarpStopLeadTimeSeconds)
            {
                TimeWarp.SetRate(0, true);
                // Keep _targetUt (the launch UT) so the countdown keeps showing the real remaining
                // seconds through the final lead-in — zeroing it made SecondsUntilLaunch jump to 0.
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            WarpToUT(_targetUt - WarpStopLeadTimeSeconds, vessel);
        }

        public void SetGuidanceMode(GuidanceMode gMode, Vessel vessel = null)
        {
            if (GuidanceMode == gMode) return;

            if (gMode == GuidanceMode.None)
            {
                // disengage flight mode
                GuidanceMode = GuidanceMode.None;
                _attitudeControl.Reset();
                _ascentGuidance.Reset();
                return;
            }

            if (vessel != null && bbState.LaunchPlan != null)
            {
                GuidanceInfo =
                    _ascentGuidance.GetGuidance(
                        vessel,
                        bbState.LaunchPlan,
                        ManualPitchCommandDeg,
                        ManualHeadingCommandDeg,
                        ManualThrottleCommand,
                        ManualRollCommand,
                        gMode);
            }

            if (vessel == null || GuidanceInfo == null)
            {
                _attitudeControl.Reset();
                _ascentGuidance.Reset();
                GuidanceMode = gMode;
                return;
            }

            if (gMode == GuidanceMode.Manual)
            {
                ManualPitchCommandDeg = GuidanceInfo.CurrentPitchDeg;
                ManualHeadingCommandDeg = MathHelpers.NormalizeDegrees(GuidanceInfo.CurrentHeadingDeg);
                ManualThrottleCommand = GuidanceInfo.CommandThrottle;
            }

            if (gMode == GuidanceMode.Autopilot)
            {
                ManualPitchCommandDeg = ClampAutopilotPitchCommand(GuidanceInfo.ProfilePitchDeg);
                ManualHeadingCommandDeg = MathHelpers.NormalizeDegrees(GuidanceInfo.ProfileHeadingDeg);
                ManualThrottleCommand = GuidanceInfo.CommandThrottle;
            }

            if (GuidanceMode != gMode) _attitudeControl.Reset();
            if (GuidanceMode != gMode) _ascentGuidance.Reset();

            GuidanceMode = gMode;
        }

        private static double ClampAutopilotPitchCommand(double pitchDeg)
        {
            return Math.Max(-90.0, Math.Min(90.0, pitchDeg));
        }

        // abort launch before liftoff
        public void Abort()
        {
            TimeWarp.SetRate(0, true);
            _targetUt = 0.0;

            State = bbState.LaunchPlan != null ? LaunchGuidanceState.PlanReady : LaunchGuidanceState.Idle;
            bbState.GuidanceEnabled = false;
            if (bbState.ActiveModule == BlackbirdModule.LaunchGuidance) bbState.ActiveModule = BlackbirdModule.None;
            GuidanceInfo = null;   // drop the stale result; a completed flight keeps it for the result panel
            _ascentGuidance.Reset();

            // Return to planning: hide the guidance computer and reopen the flight planner (inverse of AcceptPlan).
            bbState.GuidanceVisible = false;
            bbState.PlannerVisible = true;
            bbState.GuidanceState = State;
        }

        public void ConstructLaunchPlan(Vessel vessel, Vessel target, double apoapsisAlt, double periapsisAlt, double headingDeg, double launchUt = double.NaN)
        {
            if (vessel == null) return;
            GuidanceInfo = null;   // a replaced plan invalidates the previous flight's result
            TargetVessel = target != null ? target : null;

            VesselState vs = VesselState.FromVessel(vessel);
            double lt = double.IsNaN(launchUt) ? Planetarium.GetUniversalTime() : launchUt;
            double secondsUntilLaunch = Math.Max(0.0, lt - vs.UniversalTime);

            double alt = (apoapsisAlt + periapsisAlt) * 0.5;
            double circVel = OrbitMath.GetCircularVelocity(vessel.mainBody, alt);
            double estimatedDv = MathHelpers.IsFinite(circVel) ? circVel : 0.0;
            double remainingDv = vs.RemainingDeltaV - estimatedDv;

            AscentProfile profile = AscentProfileSolver.Create(vs, apoapsisAlt, periapsisAlt, headingDeg, remainingDv);

            LaunchCandidate candidate = new LaunchCandidate
            {
                IsValid = profile.IsValid,
                ReasonUnavailable = string.Empty,
                LaunchUt = lt,
                SecondsUntilLaunch = secondsUntilLaunch,
                InsertionApoapsisAlt = apoapsisAlt,
                InsertionPeriapsisAlt = periapsisAlt,
                LaunchHeadingDeg = headingDeg,
                EstimatedInsertionTimeSeconds = profile.EstimatedTimeToInsertionSeconds,
                EstimatedOrbitsToRendezvous = double.PositiveInfinity,
                EstimatedDeltaVUsed = estimatedDv,
                EstimatedRemainingDeltaV = remainingDv,
                PlaneErrorDeg = double.NaN,
                PhaseErrorDeg = double.NaN,
                RelativeDistanceMeters = double.NaN,
                Score = 0.0,
                AscentProfile = profile,
                PhasingRecommendation = null
            };

            InsertionTarget it = new InsertionTarget { ApoapsisAlt = apoapsisAlt, PeriapsisAlt = periapsisAlt, Heading = headingDeg };
            OrbitInfo oi = OrbitInfo.Create(TargetVessel != null ? TargetVessel.orbit : vessel.orbit);
            PhasingOrbit po = PhasingOrbit.FromInsertionTarget(it, oi, FlightGlobals.currentMainBody, OrbitMath.GetPhaseAngleDeg(vessel, TargetVessel ?? vessel));

            bbState.LaunchPlan = new LaunchPlan
            {
                InsertionTarget = it,
                Candidates = new[] { candidate },
                PhasingOrbit = po,
                SelectedCandidateIndex = 0,
                TargetOrbitNormal = TrajectoryProvider.GetOrbitNormal(target)
            };
        }

        public void Reset()
        {
            if (bbState != null) bbState.LaunchPlan = null;
            if (bbState != null && bbState.ActiveModule == BlackbirdModule.LaunchGuidance) bbState.ActiveModule = BlackbirdModule.None;
            TimeWarp.SetRate(0, true);
            _targetUt = 0.0;
            State = LaunchGuidanceState.Idle;
            _ascentGuidance.Reset();
        }
    
        // pitch command
        public void IncreaseManualPitchCommand() => ManualPitchCommandDeg += 1.0;
        public void DecreaseManualPitchCommand() => ManualPitchCommandDeg -= 1.0;
        public void ResetPitchCommand() => ManualPitchCommandDeg = GuidanceInfo != null ? ClampAutopilotPitchCommand(GuidanceInfo.CurrentPitchDeg) : 90.0;
        public void SetPitchCommand(double pitch) => ManualPitchCommandDeg = MathHelpers.Clamp(pitch, -90.0, 90.0);

        // heading command
        public void IncreaseManualHeadingCommand() => ManualHeadingCommandDeg += 1.0;
        public void DecreaseManualHeadingCommand() => ManualHeadingCommandDeg -= 1.0;
        public void ResetHeadingCommand() => ManualHeadingCommandDeg = GuidanceInfo != null ? MathHelpers.NormalizeDegrees(GuidanceInfo.CurrentHeadingDeg) : 90.0;
        public void SetHeadingCommand(double heading) => ManualHeadingCommandDeg = MathHelpers.Clamp(heading, -180.0, 180.0);

        // roll command
        public void IncreaseManualRollCommand() => ManualRollCommand += 1.0;
        public void DecreaseManualRollCommand() => ManualRollCommand -= 1.0;
        public void ResetRollCommand() => ManualRollCommand = 0;
        public void SetRollCommand(double roll) => ManualRollCommand = MathHelpers.Clamp(roll, -180.0, 180.0);

        // throttle command
        public void IncreaseManualThrottleCommand() => ManualThrottleCommand += 0.10;
        public void DecreaseManualThrottleCommand() => ManualThrottleCommand -= 0.10;
        public void ResetThrottleCommand() => ManualThrottleCommand = GuidanceInfo != null ? GuidanceInfo.CommandThrottle : 0.0;
        public void SetThrottleCommand(double throttle) => ManualThrottleCommand = MathHelpers.Clamp(throttle / 100, 0, 1);
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null) return;
            // Actuate only while we own control; another module taking over (e.g. rendezvous/docking) stops us.
            if (bbState != null && bbState.ActiveModule != BlackbirdModule.LaunchGuidance) return;

            if (State == LaunchGuidanceState.GuidingAscent)
            {
                GuidanceInfo =
                    _ascentGuidance.GetGuidance(
                        vessel,
                        bbState.LaunchPlan,
                        ManualPitchCommandDeg,
                        ManualHeadingCommandDeg,
                        ManualThrottleCommand,
                        ManualRollCommand,
                        GuidanceMode);

                if (GuidanceInfo != null)
                {
                    if (AscentReport.LOG_ENABLED)
                    {
                        double ut = Planetarium.GetUniversalTime();
                        AscentReport.LatchProjected(
                                    AscentPathProvider.Build(_ascentGuidance.CurrentSolution, vessel),
                                    vessel.mainBody.Radius,
                                    GuidanceInfo.TargetApoapsisAlt);
                        AscentReport.SampleActual(vessel, ut);
                    }

                    if (GuidanceInfo.IsGuidanceComplete)
                    {
                        if (AscentReport.HasData) AscentReport.WriteReport();

                        State = LaunchGuidanceState.Complete;
                        bbState.GuidanceState = LaunchGuidanceState.Complete;
                        bbState.ActiveModule = BlackbirdModule.None;
                    }

                }
            }

            // release throttle control back to user
            if (State != LaunchGuidanceState.GuidingAscent || GuidanceMode == GuidanceMode.None || GuidanceInfo == null)
            {
                if (!_completionControlReleased)
                {
                    state.mainThrottle = 0.0f;
                    if (FlightInputHandler.state != null) FlightInputHandler.state.mainThrottle = 0.0f;
                    _completionControlReleased = true;
                }

                return;
            }

            double throttle = GuidanceMode == GuidanceMode.Manual ? ManualThrottleCommand : GuidanceInfo.CommandThrottle;

            if (IsPrelaunchHold(vessel))
            {
                _attitudeControl.Reset();
                state.pitch = state.pitchTrim;
                state.yaw = state.yawTrim;
                state.roll = state.rollTrim;
                ApplyAutopilotThrottle(state, throttle);
                return;
            }

            if (GuidanceMode == GuidanceMode.Autopilot && GuidanceInfo.HasInertialDirection)
            {
                // Coasting and already aligned/settled: hold controls neutral instead of nulling micro-error
                // every frame (which fires continuous tiny RCS bursts). Reset the PID on entry so it doesn't
                // wind up while idle and re-acquires cleanly when the craft drifts past the exit angle.
                if (GuidanceInfo.CommandThrottle <= CoastThrottleEpsilon
                    && CoastAttitudeAligned(vessel, GuidanceInfo.InertialDirection))
                {
                    if (!_coastAttitudeHeld) { _attitudeControl.Reset(); _coastAttitudeHeld = true; }
                    state.pitch = state.pitchTrim;
                    state.yaw = state.yawTrim;
                    state.roll = state.rollTrim;
                }
                else
                {
                    _coastAttitudeHeld = false;
                    _attitudeControl.DriveInertial(vessel, state, GuidanceInfo.InertialDirection, 0.0, bbState.LockRollOnAscent);
                }
            }
            else
            {
                _coastAttitudeHeld = false;
                _attitudeControl.Drive(vessel, state, GuidanceInfo.CommandHeadingDeg, GuidanceInfo.CommandPitchDeg, GuidanceInfo.CommandRoll);
            }

            ApplyAutopilotThrottle(state, throttle);
        }

        private void ApplyAutopilotThrottle(FlightCtrlState state, double throttle)
        {
            if (GuidanceMode != GuidanceMode.Autopilot) return;

            state.mainThrottle = (float)(Math.Max(0.0, Math.Min(1.0, throttle)));
        }

        // Whether the craft is within the coast-hold deadband of the hold direction AND nearly stopped
        // rotating, with hysteresis: it must come within CoastHoldEnterDeg to start holding, and only releases
        // once it has drifted past the wider CoastHoldExitDeg. The rate gate stops it latching mid-slew.
        private bool CoastAttitudeAligned(Vessel vessel, Vector3d holdDirection)
        {
            if (vessel == null || vessel.ReferenceTransform == null || holdDirection.sqrMagnitude <= 0.0)
                return false;

            double errorDeg = Vector3d.Angle(vessel.ReferenceTransform.up, holdDirection);
            // KSP vessel angular velocity: x = pitch, z = yaw (y = roll, which doesn't move the nose).
            Vector3d angularVel = vessel.angularVelocityD;
            double pitchYawRateDegPerSec =
                Math.Sqrt(angularVel.x * angularVel.x + angularVel.z * angularVel.z) * (180.0 / Math.PI);

            double gateDeg = _coastAttitudeHeld ? CoastHoldExitDeg : CoastHoldEnterDeg;
            return errorDeg <= gateDeg && pitchYawRateDegPerSec <= CoastHoldMaxRateDegPerSec;
        }

        private static bool IsPrelaunchHold(Vessel vessel)
        {
            return vessel != null && vessel.situation == Vessel.Situations.PRELAUNCH;
        }
    }
}
