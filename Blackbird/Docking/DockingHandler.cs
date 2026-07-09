using System;
using Blackbird.Guidance;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Docking
{
    // owns docking autopilot and manual RCS controls
    public sealed class DockingHandler
    {
        private readonly RcsController _rcs = new RcsController();
        private readonly AttitudeControl _attitude = new AttitudeControl();
        private readonly DockingAutopilot _autopilot;

        private SharedState bbState;
        public DockingHandler()
        {
            _autopilot = new DockingAutopilot(_rcs, _attitude);
        }

        private bool _keepPointed;
        private bool _alignToPort;
        // latched one-shot: roll/point to a known attitude
        private bool _resetOrientation;
        private bool _lockRoll;
        private bool _efficientTranslation;

        public bool KeepPointed
        {
            get => _keepPointed;
            set { _keepPointed = value; if (value) { _alignToPort = false; _resetOrientation = false; } UpdateAssistClaim(); }
        }

        public bool AlignToPort
        {
            get => _alignToPort;
            set { _alignToPort = value; if (value) { _keepPointed = false; _resetOrientation = false; } UpdateAssistClaim(); }
        }
        public bool ResetOrientation
        {
            get => _resetOrientation;
            set { _resetOrientation = value; if (value) { _keepPointed = false; _alignToPort = false; } UpdateAssistClaim(); }
        }
        public bool LockRoll
        {
            get => _lockRoll;
            set { _lockRoll = value; }
        }
        public bool EfficientTranslation
        {
            get => _efficientTranslation;
            set { _efficientTranslation = value; }
        }

        private const double CenterToleranceMeters = 0.5;   // on-centerline tolerance for dock-readiness
        // AlignToPort dock-readiness: true once the mating axis is aligned AND the chaser sits on the centerline.
        public bool DockReady { get; private set; }
        public string AlignAssistStatus { get; private set; } = "";

        // --- manual input, set by the UI each draw while a button is held; consumed (with a freshness window
        // so it can't stick if the GUI stops drawing) on the fly-by-wire pass ----------------------------
        private Vector3 _manualTranslate;   // craft-local: x = right, y = dorsal-up, z = nose-forward, each -1..1
        private Vector2 _manualRotate;      // x = pitch, y = yaw, each -1..1
        private bool _manualKill;
        private double _manualInputUt = double.NegativeInfinity;
        private const double ManualInputFreshSeconds = 0.2;
        private const double ManualTranslateError = 5.0;    // velocity-error magnitude per held button; large to saturate RCS

        // --- per-tick cache ------------------------------------------------------------------------------
        private Vessel _vessel;
        private VesselState _vs;

        // --- metrics (for the UI) ------------------------------------------------------------------------
        //public bool HasTarget { get; private set; }
        //public string TargetName { get; private set; } = "";
        //public string TargetPortName { get; private set; } = "";
        public double PortDistanceMeters { get; private set; } = double.NaN;
        public double ClosingRateSigned { get; private set; } = double.NaN;   // + = getting closer, - = moving away
        public double RcsFuelPercent { get; private set; } = double.NaN;
        public DockingSteps DockingStep => _autopilot.CurrentStep;
        public string GuidanceStatus => bbState.DockingMode == DockingControlMode.Guidance ? _autopilot.status : "not running";
        public double OrientationErrorDeg => _autopilot.OrientationErrorDeg;
        public double AxialSepMeters => _autopilot.ZSep;
        public double LateralSepMeters => _autopilot.LateralMeters;
        public StepGate CurrentGate => _autopilot.CurrentGate;
        public StepGate GateFor(DockingSteps step) => _autopilot.GateFor(step);

        // UI gates (the panel mirrors the enable/disable logic; these just enact the transitions).
        public void RunDockingGuidance() { 
            bbState.DockingMode = DockingControlMode.Guidance; 
            bbState.DockingEnabled = true;
            bbState.ActiveModule = BlackbirdModule.Docking;
            AlignToPort = false;
            KeepPointed = false;
            ResetOrientation = false;
            _autopilot.Engage(bbState, EfficientTranslation); 
        }
        // Take manual control: claim the authority (Docking) so rendezvous/ascent self-stop, switch to Manual,
        // and stop the docking autopilot. Stock control still passes through when no panel button is held.
        public void AssumeControl() {
            bbState.ActiveModule = BlackbirdModule.Docking;
            bbState.DockingMode = DockingControlMode.Manual;
            bbState.DockingEnabled = true;
            AlignToPort = false;
            KeepPointed = false;
            ResetOrientation = false;
            _autopilot.Disengage();
        }

        public void StopDockingGuidance()
        {
            bbState.ActiveModule = BlackbirdModule.None;
            bbState.DockingMode = DockingControlMode.Off;
            bbState.DockingEnabled = false;
            _autopilot.Disengage();
        }

        private void UpdateAssistClaim()
        {
            bool wantAssist = _alignToPort || _keepPointed || _resetOrientation;
            if (wantAssist && bbState.CanClaimControl(BlackbirdModule.Docking))
            {
                bbState.ActiveModule = BlackbirdModule.Docking;
                bbState.DockingMode = DockingControlMode.Manual;
            }
            else if (!wantAssist && bbState.DockingMode != DockingControlMode.Guidance)
            {
                bbState.ActiveModule = BlackbirdModule.None;
                bbState.DockingMode = DockingControlMode.Off;
            }
        }

        // Combined held-button state from the panel (one call per draw).
        public void SetManualInput(Vector3 translate, Vector2 rotate, bool kill)
        {
            _manualTranslate = translate;
            _manualRotate = rotate;
            _manualKill = kill;
            _manualInputUt = Planetarium.GetUniversalTime();
        }

        public void Init(SharedState s)
        {
            bbState = s;
            bbState.DockingMode = DockingControlMode.Off;
        }

        private bool ManualInputFresh() => Planetarium.GetUniversalTime() - _manualInputUt < ManualInputFreshSeconds;

        // State-machine + metrics tick (from BlackBird.Update). Refreshes the readouts always; builds the
        // VesselState and ticks the autopilot only when something will actuate (so it stays cheap when idle).
        public void Update(Vessel active)
        {
            _vessel = active;
            if (active == null || bbState == null)
            {
                _vs = null;
                return;
            }

            UpdateMetrics();

            // wait until metrics are updated before bailing
            if (bbState.ActiveModule != BlackbirdModule.Docking)
            {
                // flag as off if we had guidance active but are ineligible to continue
                if (bbState.DockingMode == DockingControlMode.Guidance) bbState.DockingMode = DockingControlMode.Off;
                _vs = null;
                return;
            }

            bool actuating = bbState.DockingMode == DockingControlMode.Guidance
                             || (KeepPointed && bbState.HaveTarget)
                             || (AlignToPort && bbState.HaveTarget)
                             || _resetOrientation
                             || ManualInputFresh();
            if (!actuating)
            {
                _vs = null;
                return;
            }

            // RCS must be on before VesselState samples the thrust table and before translation can fire.
            if (!active.ActionGroups[KSPActionGroup.RCS]) active.ActionGroups.SetGroup(KSPActionGroup.RCS, true);

            _vs = VesselState.FromVessel(active);

            if (bbState.DockingMode == DockingControlMode.Guidance)
            {
                _autopilot.OnFixedUpdate(active, _vs, bbState);
                // The autopilot turns Off on capture or target loss; hand control back to the operator.
                if (_autopilot.CurrentStep == DockingSteps.Off)
                {
                    bbState.DockingMode = DockingControlMode.Off;
                    bbState.ActiveModule = BlackbirdModule.None;
                }
            }
        }

        // Actuation (from BlackBird.OnFlyByWire). Guidance flies it; otherwise the operator's manual inputs do.
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null || vessel == null) return;

            if (bbState.DockingMode == DockingControlMode.Guidance)
            {
                _autopilot.Drive(state, LockRoll);
                return;
            }

            // KILL (highest-priority manual action): null the relative velocity and hold the current attitude,
            // letting the attitude controller's rate loop brake all angular rates (incl. roll) to a stop.
            if (_manualKill && ManualInputFresh())
            {
                if (_vs != null)
                {
                    Vector3d baseVel = bbState.HaveTarget ? TrajectoryProvider.GetVelocity(bbState.TargetVessel) : _vs.OrbitalVelocity;
                    _rcs.SetTargetWorldVelocity(baseVel);
                    _rcs.Drive(state, _vs, vessel, bbState, _efficientTranslation);
                }

                _attitude.DriveHoldAttitude(vessel, state);
                return;
            }

            // orient the craft vs target or closest reference body 
            if (_resetOrientation)
            {
                // Vector3d facing = HasTarget ? PointAtTargetDirection() : (Vector3d)vessel.ReferenceTransform.up;
                // use the closest main body as "up" if no target
                Vector3d facing = bbState.HaveTarget ? PointAtTargetDirection() : PointAtWorldDirection();

                if (facing.sqrMagnitude > 0.0)
                {
                    // claude claims the "90.0" will only work if used near equator, otherwise i need to do some kind of vector transform
                    _attitude.DriveInertial(vessel, state, facing, 90.0, LockRoll);
                    if (OrientationAligned(vessel, facing)) _resetOrientation = false;
                }
                else _resetOrientation = false;
            }
            else if (AlignToPort && bbState.HaveTarget)
            {
                // align the mating axis to the docking port
                Vector3d facing = PointAlongDockingAxis();
                if (facing.sqrMagnitude > 0.0)
                {
                    _attitude.DriveInertial(vessel, state, facing, 0.0, LockRoll);

                    // Dock-ready = axis aligned to the mating corridor AND on the centerline. Report the phase so
                    // the operator knows when it's safe to translate straight in.
                    bool aligned = OrientationAligned(vessel, facing);
                    double lateral = LateralOffsetMeters();
                    DockReady = aligned && lateral <= CenterToleranceMeters;
                    AlignAssistStatus = !aligned ? "aligning axis"
                                      : DockReady ? "aligned & centered - translate in to dock"
                                      : $"centering ({lateral:F1} m off-axis)";

                    // center laterally only once the nose is on the mating axis
                    if (aligned && _vs != null && !(_manualTranslate.sqrMagnitude > 0.0 && ManualInputFresh()))
                        DriveLateralCenter(state, vessel);
                }
            }
            else if (KeepPointed && bbState.HaveTarget)
            {
                Vector3d facing = PointAtTargetDirection();
                // had a 90.0 rollDeg lock here but not sure what that was for
                if (facing.sqrMagnitude > 0.0) _attitude.DriveInertial(vessel, state, facing, 0.0, LockRoll);
            }

            if (!ManualInputFresh()) return;   // no buttons held -> release (don't burn RCS idling)

            // Manual translation: RAW RCS thrust in the craft's local axes, like the H/N/I/J/K/L keys — a pure
            // direction to thrust along (no target velocity), so there is NO drift correction on the other
            // axes. Scaled large enough to saturate the thrusters (full deflection while held).
            if (_manualTranslate.sqrMagnitude > 0.0 && _vs != null && vessel.ReferenceTransform != null)
            {
                Transform rt = vessel.ReferenceTransform;
                Vector3d dir = (Vector3d)rt.right * _manualTranslate.x          // right (+) / left (-)
                               + -(Vector3d)rt.forward * _manualTranslate.y     // dorsal up (+) / down (-)
                               + (Vector3d)rt.up * _manualTranslate.z;          // nose-forward (+) / back (-)
                _rcs.SetWorldVelocityError(dir * ManualTranslateError);
                _rcs.Drive(state, _vs, vessel, bbState, _efficientTranslation);
            }

            // Manual rotation (only when nothing else is driving the attitude): bias the nose. Signs verify
            // in-game; flip if mirrored.
            if (!KeepPointed && !_resetOrientation && _manualRotate.sqrMagnitude > 0.0)
            {
                state.pitch = Mathf.Clamp(state.pitch + _manualRotate.x, -1f, 1f);
                state.yaw = Mathf.Clamp(state.yaw + _manualRotate.y, -1f, 1f);
            }
        }

        // True once the nose is within a couple degrees of the target facing AND rotation has settled (so roll
        // has converged too) — used to auto-clear the one-shot orientation reset.
        private static bool OrientationAligned(Vessel vessel, Vector3d facing)
        {
            if (vessel.ReferenceTransform == null) return true;
            double dot = Vector3d.Dot(((Vector3d)vessel.ReferenceTransform.up).normalized, facing.normalized);
            dot = Math.Max(-1.0, Math.Min(1.0, dot));
            double noseErrDeg = Math.Acos(dot) * 180.0 / Math.PI;
            return noseErrDeg < 2.0 && vessel.angularVelocityD.magnitude < 0.05;
        }

        // World-frame LINE OF SIGHT from the chaser's control point to the target port — i.e. the direction
        // to point the nose so it actually aims AT the target (not the port's fixed corridor axis, which only
        // aligns when you're already on the centerline). Zero if no target / transforms unavailable.
        private Vector3d PointAtTargetDirection()
        {
            if (!bbState.HaveTarget || _vessel == null || _vessel.ReferenceTransform == null) return Vector3d.zero;
            Transform tt = bbState.TargetObject.GetTransform();
            if (tt == null) return Vector3d.zero;
            Vector3d los = (Vector3d)tt.position - (Vector3d)_vessel.ReferenceTransform.position;
            return los.sqrMagnitude > 1e-9 ? los.normalized : Vector3d.zero;
        }

        private Vector3d PointAlongDockingAxis()
        {
            if (!bbState.HaveTarget || !(bbState.TargetObject is ModuleDockingNode)) return Vector3d.zero;
            Transform tt = bbState.TargetObject.GetTransform();
            return tt != null ? (-(Vector3d)tt.forward).normalized : Vector3d.zero;
        }

        // Perpendicular offset (m) from the target port's centerline to the chaser control point; +inf if
        // unavailable so an unknown geometry never reads as "centered".
        private double LateralOffsetMeters()
        {
            if (bbState.TargetDockingPort == null || _vessel == null || _vessel.ReferenceTransform == null) return double.PositiveInfinity;
            Transform tt = bbState.TargetDockingPort.GetTransform();
            if (tt == null) return double.PositiveInfinity;
            Vector3d axis = ((Vector3d)tt.forward).normalized;
            Vector3d sep = (Vector3d)_vessel.ReferenceTransform.position - (Vector3d)tt.position;
            return Vector3d.Exclude(axis, sep).magnitude;
        }

        private void DriveLateralCenter(FlightCtrlState state, Vessel vessel)
        {
            if (bbState.TargetDockingPort == null || vessel.ReferenceTransform == null) return;

            Transform tt = bbState.TargetDockingPort.GetTransform();
            if (tt == null) return;

            Vector3d axis = ((Vector3d)tt.forward).normalized;
            Vector3d sep = (Vector3d)vessel.ReferenceTransform.position - (Vector3d)tt.position; // port
            Vector3d lateral = Vector3d.Exclude(axis, sep); // off-centerline

            Vector3d correction = Vector3d.zero;
            if (lateral.magnitude > 0.2)
            {
                //double speed = Math.Min(speedLimit, lateral.magnitude / closeLateralTime);
                //correction = -lateral.normalized * speed; // move towards centerline
                Vector3d toward = -lateral / lateral.magnitude;
                Vector3d localDir = vessel.ReferenceTransform.InverseTransformDirection(toward);
                double accel = _vs.AvailableRcsThrust.GetMagnitude(localDir) * _rcs.rcsAccelerationFactor()
                               / Math.Max(1e-6, _vs.TotalMass);
                correction = toward * DockingSchedule.MaxSpeed(lateral.magnitude, accel, 0.5);  // brakes to a stop at 0.5 m/s speed limit
            }

            Vector3d targetVel = TrajectoryProvider.GetVelocity(bbState.TargetVessel);
            _rcs.SetTargetWorldVelocity(targetVel + correction);
            _rcs.Drive(state, _vs, vessel, bbState, _efficientTranslation);
        }

        private Vector3d PointAtWorldDirection()
        {
            if (_vessel == null || _vessel.mainBody == null) return Vector3d.zero;
            if (_vessel.mainBody.transform == null) return Vector3d.zero;
            Vector3d los = (Vector3d)_vessel.mainBody.transform.position - (Vector3d)_vessel.ReferenceTransform.position;
            return los.sqrMagnitude > 1e-9 ? los.normalized : Vector3d.zero;
        }
        private void UpdateMetrics()
        {
            RcsFuelPercent = RcsMonopropPercent(_vessel);

            if (bbState.TargetVessel == null || bbState.TargetObject == null || _vessel == null || _vessel.ReferenceTransform == null)
            {
                PortDistanceMeters = double.NaN;
                ClosingRateSigned = double.NaN;
                return;
            }

            Transform tt = bbState.TargetObject.GetTransform();

            Vector3d toTarget = (Vector3d)tt.position - (Vector3d)_vessel.ReferenceTransform.position;
            PortDistanceMeters = toTarget.magnitude;

            Vector3d relVel = TrajectoryProvider.GetVelocity(_vessel) - TrajectoryProvider.GetVelocity(bbState.TargetVessel);
            ClosingRateSigned = toTarget.sqrMagnitude > 1e-9
                ? Vector3d.Dot(relVel, toTarget.normalized)   // + = closing, - = receding
                : 0.0;
        }

        // Vessel monopropellant as a percent of capacity (NaN if the craft carries none).
        private static double RcsMonopropPercent(Vessel vessel)
        {
            if (vessel == null || vessel.parts == null) return double.NaN;

            double amount = 0.0, capacity = 0.0;
            for (int i = 0; i < vessel.parts.Count; i++)
            {
                PartResourceList resources = vessel.parts[i].Resources;
                if (resources == null) continue;
                for (int r = 0; r < resources.Count; r++)
                {
                    PartResource res = resources[r];
                    if (res != null && res.resourceName == "MonoPropellant")
                    {
                        amount += res.amount;
                        capacity += res.maxAmount;
                    }
                }
            }

            return capacity > 0.0 ? amount / capacity * 100.0 : double.NaN;
        }
    }
}
