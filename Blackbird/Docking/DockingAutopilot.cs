using System;
using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Models;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Docking
{
    // KSP-coupled docking autopilot (re-derived from MechJeb's MechJebModuleDockingAutopilot). It is the thin
    // wrapper over the pure DockingSchedule: each tick it reads the live port geometry off the transforms,
    // advances the schedule, and actuates — attitude via AttitudeControl, translation via RcsController.
    //
    // Standalone by design (no rendezvous-stage assumption): it reads the chaser's OWN selected target
    // (vessel.targetObject — a docking port when the operator targets one, which is an ITargetable, NOT a
    // Vessel), derives the target vessel/port transform from it, and runs the full back-up -> side-switch ->
    // move-to-start -> dock sequence. The caller just feeds it the vessel + a fresh VesselState each tick.
    public class DockingAutopilot
    {
        // --- user-facing tunables --------------------------------------------------------------------
        public double speedLimit = 1.0;       // cap on every commanded approach speed (m/s)
        public double roll = 0.0;             // forced roll (deg) when forceRoll is set
        public bool forceRoll = false;        // hold a specific roll about the docking axis
        public bool overrideStartDistance = false;
        public bool overrideTargetSize = false;
        public double OverriddenStartDistance = 5.0;
        public double OverriddenTargetSize = 10.0;

        private const double DockingCorridorRadius = 1.0;   // lateral tolerance to count as "on the axis"

        // --- derived sizes (refreshed each tick; bounding boxes captured once at Init) ----------------
        private double safeDistance = 10.0;
        private double targetSize = 5.0;
        private double acquireRange = 0.25;
        private double vesselBoundingSize = 0.0;
        private Box3d vesselBoundingBox;
        private Box3d targetBoundingBox;

        // --- live geometry (recomputed each tick from transforms) ------------------------------------
        private Vector3d zAxis;        // unit docking axis
        private Vector3d lateralSep;   // perpendicular offset vector (centreline -> chaser)
        private double zSep;           // along-axis separation (>0 in front of the port)

        public DockingSteps Step = DockingSteps.Off;

        // --- injected per tick + owned actuators -----------------------------------------------------
        private Vessel v;
        private Vessel targetVessel;
        private ITargetable targetObject;
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

        // Start a docking run: the next OnFixedUpdate (Step == Starting) captures bounding boxes and picks
        // the entry step. RCS is forced on by the caller before VesselState is built so the thrust table is
        // populated from the first tick.
        public void Engage()
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
        public void OnFixedUpdate(Vessel vessel, VesselState vesselState)
        {
            v = vessel;
            vs = vesselState;
            targetObject = vessel != null ? vessel.targetObject : null;
            targetVessel = targetObject != null ? targetObject.GetVessel() : null;

            if (v == null || vs == null || targetObject == null || targetVessel == null)
            {
                EndDocking();
                return;
            }

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
        public void Drive(FlightCtrlState state)
        {
            if (state == null || v == null || vs == null || targetVessel == null) return;
            if (Step == DockingSteps.Off || Step == DockingSteps.Starting) return;

            DockingPlan plan = DockingSchedule.Plan(Step, Geom(), Config(), AccelInDirection);
            status = plan.Status;

            // Attitude: face the port along the docking axis, or hold current attitude while backing up.
            Vector3d facing = plan.Align ? zAxis : (Vector3d)v.ReferenceTransform.up;
            attitude.DriveInertial(v, state, facing, forceRoll ? roll : 0.0);
            state.mainThrottle = 0.0f;

            // Translate: command the target's (measured) velocity plus the approach adjustment; the RCS PID
            // nulls the residual. Measured velocity (not orbit.GetVel) keeps it Principia-consistent.
            Vector3d targetVel = TrajectoryProvider.GetVelocity(targetVessel);
            rcs.SetTargetWorldVelocity(targetVel + plan.Adjustment);
            rcs.Drive(state, vs, v);
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
                targetBoundingBox = DockingGeometry.GetBoundingBox(targetVessel);
                vesselBoundingSize = vesselBoundingBox.size.magnitude;
                RefreshSizes();
                acquireRange = targetObject is ModuleDockingNode node ? node.acquireRange * 0.5 : 0.25;
            }
            catch (Exception ex)
            {
                bbLogger.Write("docking-init", "bounding-box / acquire-range read failed: " + ex.Message);
            }

            Step = DockingSchedule.PickEntryStep(Geom(), Config());
        }

        private void RefreshSizes()
        {
            targetSize = overrideTargetSize ? OverriddenTargetSize : targetBoundingBox.size.magnitude;
            safeDistance = overrideStartDistance ? OverriddenStartDistance : vesselBoundingSize + targetSize + 0.5;
        }

        // Recompute the docking geometry from the live transforms: separation (chaser control-transform to
        // the targeted port), the docking axis, and the along-axis / lateral split.
        private void UpdateDistance()
        {
            Transform targetTransform = targetObject.GetTransform();
            Vector3d separation = v.ReferenceTransform.position - targetTransform.position;   // target -> chaser
            Vector3d dockingAxis = CanAlign() ? -targetTransform.forward : -targetTransform.up;

            zAxis = dockingAxis.normalized;
            zSep = -Vector3d.Dot(separation, zAxis);          // >0 in front of the port, <0 behind
            lateralSep = Vector3d.Exclude(zAxis, separation); // perpendicular (off-centreline) component
        }

        // The target exposes a docking orientation (i.e. it's a port we can align to) rather than just a point.
        private bool CanAlign()
        {
            return targetObject != null
                && targetObject.GetTargetingMode() == VesselTargetModes.DirectionVelocityAndOrientation;
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
                SafeDistance = safeDistance,
                TargetSize = targetSize,
                AcquireRange = acquireRange,
                DockingCorridorRadius = DockingCorridorRadius,
                SpeedLimit = speedLimit,
                VesselBoundingSize = vesselBoundingSize
            };
        }

        private void EndDocking()
        {
            Step = DockingSteps.Off;
        }
    }
}
