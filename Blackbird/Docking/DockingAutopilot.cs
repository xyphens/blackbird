using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Trajectory;
using System;
using UnityEngine;

namespace Blackbird.Docking
{
    public class DockingAutopilot
    {
        // --- user-facing tunables --------------------------------------------------------------------
        public double dockSpeedLimit = 1.0;       // cap on every maneuver

        // approaching
        public double approachSpeedLimitMs = 5.0; // cap on the "get closer" sub routine
        public double approachLandingDistance = 150; // where we consider "close enough"

        //public bool forceRoll = false;        // hold a specific roll about the docking axis
        public bool overrideStartDistance = false;
        public bool overrideTargetSize = false;

        private const double DockingCorridorRadius = 1.0;   // lateral tolerance to count as "on the axis"

        // --- derived sizes (refreshed each tick; bounding boxes captured once at Init) ----------------

        public bool OverrideStartDistance = false; // dufixme: not implemented
        public double StartDistanceOverride = 0.0;
        private const double MaxStandoffMeters = 20.0;   // reachable standoff; raw bbox (~57 m Starship) deadlocks BackingUp

        private double startDistance = 10.0; // docking AP will back up to this distance when enabled every time
        private double targetSize = 5.0; // re-calculated unless overridden

        private double acquireRange = 0.25;
        private double vesselBoundingSize = 0.0;
        private double targetBoundingSize = 0.0;
        private Box3d vesselBoundingBox;
        private Box3d targetBoundingBox;

        // --- live geometry (recomputed each tick from transforms) ------------------------------------
        private Vector3d zAxis;        // unit docking axis
        private Vector3d lateralSep;   // perpendicular offset vector (centreline -> chaser)
        private double zSep;           // along-axis separation (>0 in front of the port)

        public DockingSteps Step = DockingSteps.Off;

        private SharedState bbState;

        // --- injected per tick + owned actuators -----------------------------------------------------
        private Vessel v;
        private VesselState vs;
        private readonly RcsController rcs;
        private readonly AttitudeControl attitude;
        private readonly BlackbirdLog bbLogger = new BlackbirdLog(LogContext.Docking);

        // The RCS translation engine and attitude controller are SHARED with the owning DockingHandler, so
        // its manual fine-tuning controls and this guidance drive the same actuators.
        public DockingAutopilot(RcsController rcsController, AttitudeControl attitudeControl)
        {
            rcs = rcsController;
            attitude = attitudeControl;
        }

        // UI accessors.
        public string status = "";
        public DockingSteps CurrentStep => Step;
        public double ZSep => zSep;
        public double LateralMeters => lateralSep.magnitude;
        public bool IsRunning => Step != DockingSteps.Off;

        // angle between chaser's nose and docking axis it aligns to (NaN before geometry exists)
        public double OrientationErrorDeg
        {
            get
            {
                if (v == null || v.ReferenceTransform == null || zAxis.sqrMagnitude < 1e-9) return double.NaN;
                double dot = Math.Max(-1.0, Math.Min(1.0, Vector3d.Dot(((Vector3d)v.ReferenceTransform.up).normalized, zAxis)));
                return Math.Acos(dot) * 180.0 / Math.PI;
            }
        }

        public StepGate GateFor(DockingSteps step) => DockingSchedule.Gate(step, Geom(), Config());
        public StepGate CurrentGate => GateFor(Step);

        // Start a docking run: the next OnFixedUpdate (Step == Starting) captures bounding boxes and picks
        // the entry step. RCS is forced on by the caller before VesselState is built so the thrust table is
        // populated from the first tick.
        public void Engage(SharedState s)
        {
            Step = DockingSteps.Starting;
        }

        public void Disengage()
        {
            EndDocking();
            attitude.Reset();
        }

        // State-machine tick (call from Update/FixedUpdate). Injects the live state, recomputes the geometry,
        // and advances the schedule (Starting -> one-time Init, otherwise DockingSchedule.Advance).
        public void OnFixedUpdate(Vessel vessel, VesselState vesselState, SharedState s)
        {
            v = vessel;
            vs = vesselState;
            bbState = s;

            // not eligible for any docking maneuvers
            if (v == null || vs == null || !bbState.HaveTarget)
            {
                EndDocking();
                return;
            }

            if (bbState.TargetDockingPort == null)
            {
                // no docking port selected, default to approach
                Step = DockingSteps.ClosingRange;
                return;
            }

            if (Step == DockingSteps.ClosingRange) Step = DockingSteps.Starting; // port was selected -> enter docking schedule

            UpdateDistance();
            RefreshSizes();

            if (Step == DockingSteps.Starting)
                InitOnce();
            else
                Step = DockingSchedule.Advance(Step, Geom(), Config());
        }

        // Actuation tick (call from OnFlyByWire). Plans this tick's approach, points at the port (or holds
        // attitude while backing up), and drives RCS translation to match the target velocity plus the
        // approach adjustment. RCS-only — no main engine.
        public void Drive(FlightCtrlState state, bool lockRoll)
        {
            if (state == null || v == null || vs == null || bbState == null || !bbState.HaveTarget) return;
            if (Step == DockingSteps.Off || Step == DockingSteps.Starting) return;
            if (Step == DockingSteps.ClosingRange) { GetToDockingRange(state); return; }

            DockingPlan plan = DockingSchedule.Plan(Step, Geom(), Config(), AccelInDirection);
            status = plan.Status;

            // Attitude: face the port along the docking axis, or hold current attitude while backing up.
            Vector3d facing = plan.Align ? zAxis : (Vector3d)v.ReferenceTransform.up;
            //attitude.DriveInertial(v, state, facing, forceRoll ? roll : 0.0);
            attitude.DriveInertial(v, state, facing, 0.0, lockRoll);
            state.mainThrottle = 0.0f;

            // Translate: command the target's (measured) velocity plus the approach adjustment; the RCS PID
            // nulls the residual. Measured velocity (not orbit.GetVel) keeps it Principia-consistent.
            Vector3d targetVel = TrajectoryProvider.GetVelocity(bbState.TargetVessel);
            rcs.SetTargetWorldVelocity(targetVel + plan.Adjustment);
            rcs.Drive(state, vs, v, bbState);
        }

        // only available/active when we don't have a docking port targeted
        private void GetToDockingRange(FlightCtrlState state)
        {
            
            Vector3d los = (Vector3d)bbState.TargetObject.GetTransform().position - (Vector3d)v.ReferenceTransform.position;
            double range = los.magnitude;
            Vector3d losDir = range > 1e-6 ? los / range : Vector3d.zero;
            Vector3d targetVel = TrajectoryProvider.GetVelocity(bbState.TargetVessel);

            // point at target so forward/after RCS actuators do the closing
            if (losDir.sqrMagnitude > 0.0) attitude.DriveInertial(v, state, losDir, 0.0);
            state.mainThrottle = 0.0f;

            double remaining = range - approachLandingDistance;
            if (remaining <= 0.0) {
                rcs.SetTargetWorldVelocity(targetVel); // match velocity and await port selection
                status = "In docking range - select a docking port to continue";
            } else
            {
                double decel = Math.Max(0.01, AccelInDirection(-losDir));
                double closeSpeed = Math.Min(approachSpeedLimitMs, Math.Sqrt(2.0 * decel * remaining));
                rcs.SetTargetWorldVelocity(targetVel + losDir * closeSpeed);
                status = string.Format("Closing to docking range: {0:F0} m at {1:F2} m/s", range, closeSpeed);
            }

            rcs.Drive(state, vs, v, bbState);
        }

        // Available linear acceleration (m/s^2) in a world-frame travel direction: the RCS thrust available
        // along that axis (in the vessel frame), de-rated by the PID accel factor, over mass. This is the one
        // KSP-coupled query the pure schedule needs, injected as a delegate.
        private double AccelInDirection(Vector3d worldAxis)
        {
            if (worldAxis.sqrMagnitude <= 0.0 || vs == null || v == null || v.ReferenceTransform == null) return 0.0;
            Vector3d localAxis = v.ReferenceTransform.InverseTransformDirection(worldAxis);
            return vs.AvailableRcsThrust.GetMagnitude(localAxis) * rcs.rcsAccelerationFactor()
                   / Math.Max(1e-6, vs.TotalMass);
        }

        // One-time setup on entering a run: capture bounding boxes, derive the port capture range, and pick
        // the entry step. Failures (reflection/transform hiccups) log and fall back to the defaults rather
        // than silently leaving stale sizes.
        private void InitOnce()
        {
            try
            {
                vesselBoundingBox = DockingGeometry.GetBoundingBox(v);
                targetBoundingBox = DockingGeometry.GetBoundingBox(bbState.TargetVessel);
                vesselBoundingSize = vesselBoundingBox.size.magnitude;
                targetBoundingSize = targetBoundingBox.size.magnitude;

                RefreshSizes();
                
                acquireRange = bbState.TargetDockingPort != null ? bbState.TargetDockingPort.acquireRange * 0.5 : 0.25;
            }
            catch (Exception ex)
            {
                bbLogger.Write("docking-init", "bounding-box / acquire-range read failed: " + ex.Message);
            }

            Step = DockingSchedule.PickEntryStep(Geom(), Config());
        }

        private void RefreshSizes()
        {
            double _targetPadding = 0.33 * targetBoundingSize;
            targetSize = Math.Min(MaxStandoffMeters, _targetPadding);
            //startDistance = OverrideStartDistance ? StartDistanceOverride : vesselBoundingSize + targetSize + 0.5;
            startDistance = OverrideStartDistance
                ? StartDistanceOverride
                : Math.Min(targetBoundingSize + 0.5 * vesselBoundingSize , MaxStandoffMeters);
        }

        // Recompute the docking geometry from the live transforms: separation (chaser control-transform to
        // the targeted port), the docking axis, and the along-axis / lateral split.
        private void UpdateDistance()
        {
            Transform targetTransform = bbState.TargetObject.GetTransform();
            Vector3d separation = v.ReferenceTransform.position - targetTransform.position;   // target -> chaser
            Vector3d dockingAxis = CanAlign() ? -targetTransform.forward : -targetTransform.up;

            zAxis = dockingAxis.normalized;
            zSep = -Vector3d.Dot(separation, zAxis);          // >0 in front of the port, <0 behind
            lateralSep = Vector3d.Exclude(zAxis, separation); // perpendicular (off-centreline) component
        }

        // The target exposes a docking orientation (i.e. it's a port we can align to) rather than just a point.
        private bool CanAlign()
        {
            return bbState.TargetObject != null && bbState.TargetObject.GetTargetingMode() == VesselTargetModes.DirectionVelocityAndOrientation;
        }

        private DockingGeom Geom()
        {
            return new DockingGeom
            {
                ZSep = zSep,
                LateralMag = lateralSep.magnitude,
                ZAxis = zAxis,
                LateralDir = lateralSep.sqrMagnitude > 1e-12 ? lateralSep.normalized : Vector3d.zero
            };
        }

        private DockingConfig Config()
        {
            return new DockingConfig
            {
                StartDistance = startDistance,
                TargetSize = targetSize,
                AcquireRange = acquireRange,
                DockingCorridorRadius = DockingCorridorRadius,
                SpeedLimit = dockSpeedLimit,
                VesselBoundingSize = vesselBoundingSize,
                TargetBoundingSize = targetBoundingSize,
                ClipClearance = targetBoundingSize * 0.5
            };
        }

        private void EndDocking()
        {
            Step = DockingSteps.Off;
        }
    }
}
