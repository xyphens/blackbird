using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Docking
{
    // The three user-gated legs of a docking run (staged gates, manual between legs):
    //   Approach - translate to a safe standoff waypoint out on the target port axis, aligned and stopped.
    //   Final    - translate in along the axis to a short hold point just clear of the port, aligned/stopped.
    //   Contact  - creep straight in along the axis until the ports touch (magnetic capture finishes the dock).
    public enum DockingLeg
    {
        None,
        Approach,
        Final,
        Contact
    }

    // One tick of docking guidance: the attitude to hold (so the chaser port faces the target port head-on)
    // and the translation VELOCITY to fly (world frame), plus whether the current leg is finished. The
    // actuation layer holds FacingWorld with reaction wheels/RCS torque and drives RCS translation to match
    // TranslationVelocityWorld; this struct itself is frame-independent and pure (offline-testable).
    public struct DockingCommand
    {
        public Vector3d FacingWorld;               // desired chaser-port facing (anti-parallel to target axis)
        public Vector3d TranslationVelocityWorld;  // desired relative velocity of the chaser toward the goal (m/s)
        public bool LegComplete;                   // current leg's arrival/stop (or contact) condition met
        public double AxialDistance;               // chaser offset along the axis (>0 in front of the port)
        public double LateralOffset;               // perpendicular distance off the port centerline (m)
        public double AlignmentErrorDeg;           // heading error from the mated (head-on) attitude
        public string Status;                      // human-readable, for UI/log
    }

    // Pure docking translation/alignment controller. Each leg seeks an on-axis goal point at a slow, tapered,
    // capped closing speed; because the goal is ON the port axis, flying straight to it also nulls the lateral
    // offset. Attitude is always the head-on mated heading (chaser axis anti-parallel to the target axis).
    // Stateless and frame-independent like the match-velocity / close-approach controllers — the executor
    // owns which leg is active; this just turns (ports, relative velocity, leg) into a command.
    public static class DockingController
    {
        // --- leg geometry + speed schedule (slow; RCS authority is small) -----------------------------
        private const double ApproachStandoffMeters = 25.0;   // Approach waypoint distance out along the axis
        private const double FinalStandoffMeters = 5.0;       // Final hold point distance out along the axis
        private const double ContactRangeMeters = 0.6;        // declare contact within this port-to-port range

        private const double ApproachSpeedCapMetersPerSecond = 1.0;  // closing-speed cap on the Approach leg
        private const double FinalSpeedCapMetersPerSecond = 0.4;     // ...on the Final leg
        private const double ContactSpeedCapMetersPerSecond = 0.15;  // ...on the Contact creep
        private const double ApproachGainPerSecond = 0.2;            // commanded speed tapers as goal nears (1/s)

        // --- leg-complete tolerances ------------------------------------------------------------------
        private const double ArrivalToleranceMeters = 1.0;    // within this of an Approach/Final goal counts as there
        private const double AlignToleranceDeg = 5.0;         // ...and pointed within this of the mated heading
        private const double StopSpeedMetersPerSecond = 0.1;  // ...and relative speed below this (arrived AND stopped)

        // relVel = chaser velocity minus target velocity (the chaser's velocity relative to the target).
        public static DockingCommand Compute(PortState chaserPort, PortState targetPort, Vector3d relVel, DockingLeg leg)
        {
            Vector3d axis = targetPort.Axis;
            Vector3d offset = chaserPort.Position - targetPort.Position;   // target port -> chaser port
            double axial = Vector3d.Dot(offset, axis);                     // along the axis (+ = in front)
            double lateral = (offset - axis * axial).magnitude;

            // Head-on mated heading: the chaser port faces opposite the target port's outward axis.
            Vector3d facing = -axis;
            double alignDot = MathHelpers.Clamp(Vector3d.Dot(chaserPort.Axis, facing), -1.0, 1.0);
            double alignDeg = MathHelpers.Rad2Deg(Math.Acos(alignDot));

            // The on-axis goal point and speed cap for this leg.
            Vector3d goal;
            double speedCap;
            switch (leg)
            {
                case DockingLeg.Approach: goal = targetPort.Position + axis * ApproachStandoffMeters; speedCap = ApproachSpeedCapMetersPerSecond; break;
                case DockingLeg.Final:    goal = targetPort.Position + axis * FinalStandoffMeters;    speedCap = FinalSpeedCapMetersPerSecond;    break;
                default:                  goal = targetPort.Position;                                 speedCap = ContactSpeedCapMetersPerSecond;  break;
            }

            Vector3d toGoal = goal - chaserPort.Position;
            double distToGoal = toGoal.magnitude;
            Vector3d dir = distToGoal > 1e-6 ? toGoal / distToGoal : Vector3d.zero;
            double commandedSpeed = MathHelpers.Clamp(distToGoal * ApproachGainPerSecond, 0.0, speedCap);
            Vector3d desiredRelVel = dir * commandedSpeed;

            // Completion: Contact ends when the ports are within capture range; the other legs end on a clean
            // arrival (close to the goal, pointed, and nearly stopped).
            double range = offset.magnitude;
            bool complete = leg == DockingLeg.Contact
                ? range <= ContactRangeMeters
                : distToGoal <= ArrivalToleranceMeters && alignDeg <= AlignToleranceDeg && relVel.magnitude <= StopSpeedMetersPerSecond;

            string status = string.Format("dock {0}: {1:F1} m to go ({2:F2} m/s), axial {3:F1} / lateral {4:F1} m, align {5:F1}°",
                leg, distToGoal, commandedSpeed, axial, lateral, alignDeg);

            return new DockingCommand
            {
                FacingWorld = facing,
                TranslationVelocityWorld = desiredRelVel,
                LegComplete = complete,
                AxialDistance = axial,
                LateralOffset = lateral,
                AlignmentErrorDeg = alignDeg,
                Status = status
            };
        }

        // The leg that follows the given one (Contact is terminal).
        public static DockingLeg NextLeg(DockingLeg leg)
        {
            switch (leg)
            {
                case DockingLeg.Approach: return DockingLeg.Final;
                case DockingLeg.Final:    return DockingLeg.Contact;
                default:                  return DockingLeg.Contact;
            }
        }
    }
}
