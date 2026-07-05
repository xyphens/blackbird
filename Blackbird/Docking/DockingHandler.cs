using System;
using Blackbird.Guidance;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Docking
{
    // Standalone docking module handler: owns the docking autopilot AND the manual RCS fine-tuning controls,
    // sharing one RcsController + AttitudeControl between them. Ticked from BlackBird (Update + OnFlyByWire),
    // independent of the rendezvous handler. Three control modes:
    //   Neutral  - idle; manual translation available, no autopilot. (No guidance committed yet.)
    //   Manual   - operator drives via the panel's translation/rotation buttons (the operator "assumed control").
    //   Guidance - the docking autopilot flies it.
    // Manual translation reuses the proven RcsController by commanding a small relative-velocity nudge in the
    // craft's local axes, so we never re-derive FlightCtrlState translation signs.
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


        public bool KeepPointed = false;

        // --- manual input, set by the UI each draw while a button is held; consumed (with a freshness window
        // so it can't stick if the GUI stops drawing) on the fly-by-wire pass ----------------------------
        private Vector3 _manualTranslate;   // craft-local: x = right, y = dorsal-up, z = nose-forward, each -1..1
        private Vector2 _manualRotate;      // x = pitch, y = yaw, each -1..1
        private bool _manualKill;
        private bool _resetOrientation;     // latched one-shot: roll/point to a known attitude so the craft frame is predictable
        private double _manualInputUt = double.NegativeInfinity;
        private const double ManualInputFreshSeconds = 0.2;
        private const double ManualTranslateError = 5.0;    // velocity-error magnitude per held button; large to saturate RCS

        // --- per-tick cache ------------------------------------------------------------------------------
        private Vessel _vessel;
        private VesselState _vs;
        private ITargetable _targetObject;
        private Vessel _targetVessel;

        // --- metrics (for the UI) ------------------------------------------------------------------------
        public bool HasTarget { get; private set; }
        public string TargetName { get; private set; } = "";
        public string TargetPortName { get; private set; } = "";
        public double PortDistanceMeters { get; private set; } = double.NaN;
        public double ClosingRateSigned { get; private set; } = double.NaN;   // + = getting closer, - = moving away
        public double RcsFuelPercent { get; private set; } = double.NaN;
        public DockingSteps DockingStep => _autopilot.CurrentStep;
        public string GuidanceStatus => bbState.DockingMode == DockingControlMode.Guidance ? _autopilot.status : "not running";

        public bool ResettingOrientation => _resetOrientation;

        // UI gates (the panel mirrors the enable/disable logic; these just enact the transitions).
        public void RunDockingGuidance() { 
            bbState.DockingMode = DockingControlMode.Guidance; 
            bbState.DockingEnabled = true;
            bbState.ActiveModule = BlackbirdModule.Docking;
            _autopilot.Engage(); 
        }
        // Take manual control: claim the authority (Docking) so rendezvous/ascent self-stop, switch to Manual,
        // and stop the docking autopilot. Stock control still passes through when no panel button is held.
        public void AssumeControl() {
            bbState.ActiveModule = BlackbirdModule.Docking;
            bbState.DockingMode = DockingControlMode.Manual;
            bbState.DockingEnabled = true;
            _autopilot.Disengage();
        }

        public void StopDockingGuidance()
        {
            bbState.ActiveModule = BlackbirdModule.None;
            bbState.DockingMode = DockingControlMode.Off;
            bbState.DockingEnabled = false;
            _autopilot.Disengage();
        }

        // One-shot orientation reset: point at the port (if targeted) and roll to "real up" so the craft's
        // local translation axes become predictable. Latched; auto-clears when aligned. Click again to cancel.
        public void ResetOrientation() { _resetOrientation = !_resetOrientation; }

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

                HasTarget = false;
                _vs = null;
                return;
            }

            _targetObject = active.targetObject;
            _targetVessel = _targetObject != null ? _targetObject.GetVessel() : null;
            HasTarget = _targetObject != null && _targetVessel != null;

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
                             || (KeepPointed && HasTarget)
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
                _autopilot.OnFixedUpdate(active, _vs);
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
                _autopilot.Drive(state);
                return;
            }

            // KILL (highest-priority manual action): null the relative velocity and hold the current attitude,
            // letting the attitude controller's rate loop brake all angular rates (incl. roll) to a stop.
            if (_manualKill && ManualInputFresh())
            {
                if (_vs != null)
                {
                    Vector3d baseVel = HasTarget ? TrajectoryProvider.GetVelocity(_targetVessel) : _vs.OrbitalVelocity;
                    _rcs.SetTargetWorldVelocity(baseVel);
                    _rcs.Drive(state, _vs, vessel);
                }

                _attitude.DriveHoldAttitude(vessel, state);
                return;
            }

            // orient the craft vs target or closest reference body 
            if (_resetOrientation)
            {
                // Vector3d facing = HasTarget ? PointAtTargetDirection() : (Vector3d)vessel.ReferenceTransform.up;
                // use the closest main body as "up" if no target
                Vector3d facing = HasTarget ? PointAtTargetDirection() : PointAtWorldDirection();

                if (facing.sqrMagnitude > 0.0)
                {
                    // claude claims the "90.0" will only work if used near equator, otherwise i need to do some kind of vector transform
                    _attitude.DriveInertial(vessel, state, facing, 90.0);
                    if (OrientationAligned(vessel, facing)) _resetOrientation = false;
                }
                else _resetOrientation = false;
            }
            else if (KeepPointed && HasTarget)
            {
                Vector3d facing = PointAtTargetDirection();
                if (facing.sqrMagnitude > 0.0) _attitude.DriveInertial(vessel, state, facing, 90.0);
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
                _rcs.Drive(state, _vs, vessel);
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
            if (!HasTarget || _vessel == null || _vessel.ReferenceTransform == null) return Vector3d.zero;
            Transform tt = _targetObject.GetTransform();
            if (tt == null) return Vector3d.zero;
            Vector3d los = (Vector3d)tt.position - (Vector3d)_vessel.ReferenceTransform.position;
            return los.sqrMagnitude > 1e-9 ? los.normalized : Vector3d.zero;
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

            if (!HasTarget || _vessel == null || _vessel.ReferenceTransform == null)
            {
                TargetName = "";
                TargetPortName = "";
                PortDistanceMeters = double.NaN;
                ClosingRateSigned = double.NaN;
                return;
            }

            Transform tt = _targetObject.GetTransform();
            TargetName = _targetVessel.vesselName;
            TargetPortName = _targetObject.GetName();

            Vector3d toTarget = (Vector3d)tt.position - (Vector3d)_vessel.ReferenceTransform.position;
            PortDistanceMeters = toTarget.magnitude;

            Vector3d relVel = TrajectoryProvider.GetVelocity(_vessel) - TrajectoryProvider.GetVelocity(_targetVessel);
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
