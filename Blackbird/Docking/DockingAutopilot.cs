using System;
using Blackbird.Models;
using Blackbird.RCS;
using Blackbird.Trajectory;
using KSP.Localization;
using UnityEngine;
using static alglib;
using static Blackbird.Docking.DockingAutopilot;

namespace Blackbird.Docking
{
    public class DockingAutopilot
    {
        public string status = "";
        public double speedLimit = 1.0;
        public readonly double roll = 0.0;
        public bool forceRoll = false;

        // i don't whether we actually need this
        public double OverriddenStartDistance = 5.0;
        public double OverriddenTargetSize = 10.0;
        public bool overrideStartDistance = false;
        public bool overrideTargetSize = false;
        public double safeDistance = 10.0; // du todo: minimum distance to start docking routine?
        public double targetSize = 5.0; // i dunno
        public bool drawBoundingBox;

        public enum DockingSteps
        {
            Starting,
            WrongSideBackingUp,
            WrongSideLateral,
            WrongSideSwitchSides,
            BackingUp,
            MovingToStart,
            Docking,
            Off
        }

        public DockingSteps DockingStep = DockingSteps.Off;

        public Vector3d zAxis;
        public double zSep;
        public Vector3d lateralSep;
        public double relZ;
        public double relLat;
        private ITargetable lastTarget;

        private readonly float dockingBoundsRadius = 1;
        private double acquireRange = 0.25;

        public Box3d vesselBoundingBox;
        public Box3d targetBoundingBox;

        Vessel v;
        Vessel targetVessel;
        VesselState vs;
        RcsController rcs;
        public void Init(Vessel vessel, Vessel target, VesselState vesselState, RcsController rcsCtrl)
        {
            if (vessel == null || vs == null || rcs == null || target == null) return;
            v = vessel;
            targetVessel = target;
            vesselState = vs;
            rcs = rcsCtrl;

            lastTarget = target;

            try
            {
                vesselBoundingBox = DockingGeometry.GetBoundingBox(vessel);
                targetBoundingBox = DockingGeometry.GetBoundingBox(target);

                targetSize = overrideTargetSize ? OverriddenTargetSize : targetBoundingBox.size.magnitude;
                safeDistance = overrideStartDistance ? OverriddenStartDistance : vesselBoundingBox.size.magnitude + targetSize + 0.5f;
                acquireRange = vessel.targetObject is ModuleDockingNode ? ((ModuleDockingNode)vessel.targetObject).acquireRange * 0.5 : 0.25;
            }
            catch (Exception ex) { 
                // todo: unhandled / silent fail
            }

            if (zSep < 0)
            {
                // behind target
                // todo: code that compares bounding box positions to determine if we just try to back up or change sides completely.
                DockingStep = Math.Abs(zSep) < vesselBoundingBox.size.magnitude * 0.5f ? DockingSteps.WrongSideBackingUp : DockingSteps.BackingUp;
            } else if (lateralSep.magnitude > dockingBoundsRadius)
            {
                DockingStep = zSep < targetSize ? DockingSteps.BackingUp : DockingSteps.MovingToStart;
            } else
            {
                DockingStep = DockingSteps.Docking;
            }
        }

        private double MaxSpeedByDistance(double distance, Vector3d axis)
        {
            Vector3d localAxis = v.ReferenceTransform.InverseTransformDirection(axis);
            // key formula
            return ClampSpeedLimit(Math.Sqrt(2.0 
                                * Math.Abs(distance) 
                                * vs.AvailableRcsThrust.GetMagnitude(localAxis) * rcs.rcsAccelerationFactor() / vs.TotalMass));
        }

        private double ClampSpeedLimit(double s)
        {
            if (speedLimit != 0)
            {
                if (s > speedLimit) s = speedLimit;
                if (s < -speedLimit) s = -speedLimit;
            }

            return s;
        }

        public void Drive(FlightCtrlState state)
        {
            if (targetVessel == null || DockingStep == DockingSteps.Off || DockingStep == DockingSteps.Starting) return;

            // fixme: might want to pull from principia here?
            Vector3d targetVel = targetVessel.orbit.GetVel();
            double zApproachSpeed = MaxSpeedByDistance(Math.Max(zSep - acquireRange, 0), -zAxis);
            double latApproachSpeed = MaxSpeedByDistance(lateralSep.magnitude, -lateralSep);
            bool align = true;

            double timeToAxis;
            double timeToTargetSize;

            // orchestrate docking sequence speeds
            if (DockingStep == DockingSteps.WrongSideBackingUp) {
                zApproachSpeed = MaxSpeedByDistance(safeDistance + zSep + 2.0, -zAxis);
                if (lateralSep.magnitude < safeDistance)
                    latApproachSpeed *= -1;
                else if (lateralSep.magnitude < safeDistance * 2)
                    latApproachSpeed = 0;

                align = false;
                status = $"Backing up at {zApproachSpeed.ToString("F2")} m/s before lateral movement {latApproachSpeed.ToString()} m/s";
            } else if (DockingStep == DockingSteps.WrongSideLateral)
            {
                zApproachSpeed = 0;
                latApproachSpeed = -MaxSpeedByDistance(safeDistance - lateralSep.magnitude + 2.0, -lateralSep);
                status = $"Moving away from docking axis at {latApproachSpeed.ToString("F2")} m/s";
            }
            else if (DockingStep == DockingSteps.WrongSideSwitchSides)
            {
                zApproachSpeed = -MaxSpeedByDistance(-zSep + targetSize, -zAxis);
                if (lateralSep.magnitude < safeDistance)
                    latApproachSpeed *= -1;
                else if (lateralSep.magnitude < safeDistance * 2)
                    latApproachSpeed = 0;

                status = $"Moving to correct side of target at {latApproachSpeed.ToString("F2")} m/s";
            }
            else if (DockingStep == DockingSteps.BackingUp)
            {
                if (lateralSep.magnitude < safeDistance)
                    latApproachSpeed *= -1;
                else if (lateralSep.magnitude < safeDistance * 2)
                    latApproachSpeed = 0;

                zApproachSpeed = -MaxSpeedByDistance(1 + targetSize - zSep, -zAxis);
                align = false;
                status = $"Backing up at {zApproachSpeed.ToString("F2")} m/s";
            }
            else if (DockingStep == DockingSteps.MovingToStart)
            {
                if (zSep < safeDistance)
                    zApproachSpeed *= -1;
                else
                    zApproachSpeed = 0;

                status = $"Moving to start point at {zApproachSpeed.ToString("F2")} m/s";
            }
            else if (DockingStep == DockingSteps.Docking)
            {
                timeToAxis = Math.Abs(lateralSep.magnitude / latApproachSpeed);
                timeToTargetSize = Math.Abs(zSep / zApproachSpeed);

                if ((zSep <= lateralSep.magnitude * 10 || timeToTargetSize <= timeToAxis * 10) && timeToAxis > 0 && timeToTargetSize > 0)
                {
                    zApproachSpeed *= Math.Min(timeToTargetSize / timeToAxis, 1);
                    latApproachSpeed = ClampSpeedLimit(latApproachSpeed * 2);
                }

                status = $"Finalizing docking sequence at {zApproachSpeed.ToString("F2")} / {latApproachSpeed.ToString("F2")} m/s";
            }

            if (!align)
            {

            }

        }

        // PUBLIC ACCESSOR
        public void OnFixedUpdate(Vessel vessel, Vessel target, VesselState vesselState, RcsController rcsCtrl)
        {
            if (!targetVessel)
            {
                EndDocking();
                return;
            }

            targetSize = overrideTargetSize ?  OverriddenTargetSize : targetBoundingBox.size.magnitude;
            safeDistance = overrideStartDistance ? OverriddenStartDistance : vesselBoundingBox.size.magnitude + targetSize + 0.5;

            UpdateDistance();

            if (DockingStep == DockingSteps.Starting)
            {
                Init(vessel, target, vesselState, rcsCtrl);
            }
            else if (DockingStep == DockingSteps.WrongSideBackingUp)
            {
                if (-zSep > safeDistance) DockingStep = DockingSteps.WrongSideLateral;
            }
            else if (DockingStep == DockingSteps.WrongSideLateral)
            {
                if (lateralSep.magnitude > safeDistance) DockingStep = DockingSteps.WrongSideSwitchSides;
            }
            else if (DockingStep == DockingSteps.WrongSideSwitchSides)
            {
                if (zSep > 0) DockingStep = DockingSteps.BackingUp;
            } else if (DockingStep == DockingSteps.BackingUp)
            {
                if (zSep > targetSize) DockingStep = DockingSteps.MovingToStart;
            } else if (DockingStep == DockingSteps.MovingToStart)
            {
                // within attachment bounding radius
                if (lateralSep.magnitude < dockingBoundsRadius && zSep >= targetSize) DockingStep = DockingSteps.Docking;
            } else if (DockingStep == DockingSteps.Docking)
            {
                if (zSep < acquireRange)
                {
                    // close enough to latch
                    EndDocking();
                } else if (lateralSep.magnitude > dockingBoundsRadius) 
                {
                    // back up if we're behind target or return to start as default
                    DockingStep = zSep < 0 ? DockingSteps.WrongSideBackingUp : DockingSteps.MovingToStart;
                }
            }

        }

        private void EndDocking()
        {
            DockingStep = DockingSteps.Off;
        }

        private void UpdateDistance()
        {
            TrajectoryState trajectory = TrajectoryProvider.GetCurrentState(targetVessel);
            Vector3d separation = trajectory.RelativePosition;
            ITargetable target = targetVessel;
            Vector3d targetDockingAxis = CanAlign() ? -target.GetTransform().forward : -target.GetTransform().up;

            zAxis = targetDockingAxis.normalized;

            zSep = -Vector3d.Dot(separation, zAxis);
            lateralSep = Vector3d.Exclude(zAxis, separation); // inverse of zSep

            relZ = Vector3d.Dot(trajectory.RelativeVelocity, zAxis);
            relLat = Vector3d.Dot(lateralSep, trajectory.RelativeVelocity);
        }

        private bool CanAlign()
        {
            return targetVessel.GetTargetingMode() == VesselTargetModes.DirectionVelocityAndOrientation;
        }
    }
}
