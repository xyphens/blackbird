using System;
using UnityEngine;

namespace Blackbird.Docking
{
    // The staged docking sequence, ported from MechJeb's docking autopilot. Each step backs up / slides
    // laterally / closes in along the target's docking axis until the ports are within capture range.
    public enum DockingSteps
    {
        Starting,             // entry: pick the right step from the current geometry
        WrongSideBackingUp,   // we are behind the port: back straight away to clear distance
        WrongSideLateral,     // ...then translate out past the target's width
        WrongSideSwitchSides, // ...then cross over to the correct (front) side
        BackingUp,            // in front but too close / off-axis: back up to the start distance
        MovingToStart,        // centre onto the docking axis at the start distance
        Docking,              // final straight-in approach to contact
        Off
    }

    // Pure geometry the schedule reasons over (all world-frame except the scalar separations). Filled by
    // the KSP wrapper (DockingAutopilot) from the live port transforms; frame-independent so it is
    // offline-testable in the harness.
    public struct DockingGeom
    {
        public double ZSep;        // along the docking axis: >0 in front of the port, <0 behind it
        public double LateralMag;  // perpendicular distance off the centreline (m)
        public Vector3d ZAxis;     // unit docking axis (the direction the chaser approaches along)
        public Vector3d LateralDir;// unit lateral-offset direction (centreline -> chaser); zero if on-axis
    }

    // Tunables + vessel-derived sizes the schedule needs. Constant for a docking run (sizes refreshed by
    // the wrapper); kept pure so the harness can drive the schedule with hand-set values.
    public struct DockingConfig
    {
        public double SafeDistance;            // clearance distance for backing-up / side-switch (bbox-derived)
        public double TargetSize;              // target bounding size; start distance for the final approach
        public double AcquireRange;            // port magnetic-capture range; closing inside this = docked
        public double DockingCorridorRadius;   // lateral tolerance to count as "on the axis"
        public double SpeedLimit;              // user cap on every commanded approach speed (m/s)
        public double VesselBoundingSize;      // chaser bbox size.magnitude (entry-step "behind" threshold)
    }

    // One tick's commanded approach: speeds along the axis / laterally, whether to align to the port
    // (vs. hold attitude while backing up), the resulting world-frame approach velocity, and a status line.
    public struct DockingPlan
    {
        public double ZSpeed;        // commanded speed along the docking axis (signed)
        public double LatSpeed;      // commanded lateral speed (signed)
        public bool Align;           // true = point at the port; false = hold current attitude (backing up)
        public Vector3d Adjustment;  // world-frame approach velocity = -lateralDir*LatSpeed + ZSpeed*zAxis
        public string Status;        // human-readable, for UI/log
    }

    // Pure docking logic: entry-step selection, per-step transitions, and the per-step speed schedule.
    // No KSP types beyond Vector3d, so the whole sequence is harness-testable. The KSP-coupled bits
    // (reading transforms, querying available RCS thrust, actuating) live in DockingAutopilot, which feeds
    // the available-acceleration query in as a delegate.
    public static class DockingSchedule
    {
        // Maximum speed from which we can still brake to a stop over `distance`, given the available linear
        // acceleration in the travel direction: v = sqrt(2 a d), clamped to the user speed limit. (MechJeb's
        // MaxSpeedForDistance.) `accel` is supplied by the caller (thrust-table lookup / mass) so this stays pure.
        public static double MaxSpeed(double distance, double accel, double speedLimit)
        {
            double s = Math.Sqrt(2.0 * Math.Abs(distance) * Math.Max(0.0, accel));
            return ClampSpeed(s, speedLimit);
        }

        // Clamp a signed speed to +/- the user limit (0 = no limit).
        public static double ClampSpeed(double s, double speedLimit)
        {
            if (speedLimit != 0.0)
            {
                if (s > speedLimit) s = speedLimit;
                if (s < -speedLimit) s = -speedLimit;
            }

            return s;
        }

        // Pick the entry step from the starting geometry (MechJeb's InitDocking branch).
        public static DockingSteps PickEntryStep(DockingGeom g, DockingConfig c)
        {
            if (g.ZSep < 0.0)
            {
                // Behind the port: switch sides if we are more than half our own length behind (else we'd try
                // to pass through the target), otherwise just back straight up.
                return Math.Abs(g.ZSep) > c.VesselBoundingSize * 0.5
                    ? DockingSteps.WrongSideBackingUp
                    : DockingSteps.BackingUp;
            }

            if (g.LateralMag > c.DockingCorridorRadius)
            {
                // In front but off the centreline: back up first if too close, else go centre on the axis.
                return g.ZSep < c.TargetSize ? DockingSteps.BackingUp : DockingSteps.MovingToStart;
            }

            return DockingSteps.Docking;
        }

        // Advance the state machine for the current geometry (MechJeb's OnFixedUpdate switch). Returns the
        // (possibly unchanged) next step; DockingSteps.Off means docking is finished (within capture range).
        public static DockingSteps Advance(DockingSteps step, DockingGeom g, DockingConfig c)
        {
            switch (step)
            {
                case DockingSteps.Starting:
                    return PickEntryStep(g, c);
                case DockingSteps.WrongSideBackingUp:
                    return -g.ZSep > c.SafeDistance ? DockingSteps.WrongSideLateral : step;
                case DockingSteps.WrongSideLateral:
                    return g.LateralMag > c.SafeDistance ? DockingSteps.WrongSideSwitchSides : step;
                case DockingSteps.WrongSideSwitchSides:
                    return g.ZSep > 0.0 ? DockingSteps.BackingUp : step;
                case DockingSteps.BackingUp:
                    return g.ZSep > c.TargetSize ? DockingSteps.MovingToStart : step;
                case DockingSteps.MovingToStart:
                    return g.LateralMag < c.DockingCorridorRadius && g.ZSep >= c.TargetSize
                        ? DockingSteps.Docking : step;
                case DockingSteps.Docking:
                    if (g.ZSep < c.AcquireRange) return DockingSteps.Off;   // close enough to latch
                    if (g.LateralMag > c.DockingCorridorRadius)
                    {
                        // Drifted out of the corridor: back up / switch sides if behind, re-centre if barely
                        // in front (MechJeb's zSep<1 guard), else keep closing.
                        if (g.ZSep < 0.0) return DockingSteps.WrongSideBackingUp;
                        if (g.ZSep < 1.0) return DockingSteps.MovingToStart;
                    }
                    return step;
                default:
                    return step;
            }
        }

        // Compute this tick's approach speeds + adjustment for the current step (MechJeb's Drive switch).
        // `accelInDir` returns the available linear acceleration (m/s^2) in a world-frame travel direction.
        public static DockingPlan Plan(DockingSteps step, DockingGeom g, DockingConfig c, Func<Vector3d, double> accelInDir)
        {
            // Base speeds: close along the axis (down to acquire range) and onto the centreline.
            double z = MaxSpeed(Math.Max(g.ZSep - c.AcquireRange, 0.0), accelInDir(-g.ZAxis), c.SpeedLimit);
            double lat = MaxSpeed(g.LateralMag, accelInDir(-g.LateralDir), c.SpeedLimit);
            bool align = true;
            string status;

            switch (step)
            {
                case DockingSteps.WrongSideBackingUp:
                    z = MaxSpeed(c.SafeDistance + g.ZSep + 2.0, accelInDir(-g.ZAxis), c.SpeedLimit);
                    if (g.LateralMag < c.SafeDistance) lat *= -1;
                    else if (g.LateralMag < c.SafeDistance * 2.0) lat = 0;
                    align = false;
                    status = string.Format("Backing up (wrong side) at {0:F2} m/s, lateral {1:F2} m/s", z, lat);
                    break;

                case DockingSteps.WrongSideLateral:
                    z = 0;
                    lat = -MaxSpeed(c.SafeDistance - g.LateralMag + 2.0, accelInDir(-g.LateralDir), c.SpeedLimit);
                    status = string.Format("Moving off the docking axis at {0:F2} m/s", lat);
                    break;

                case DockingSteps.WrongSideSwitchSides:
                    z = -MaxSpeed(-g.ZSep + c.TargetSize, accelInDir(-g.ZAxis), c.SpeedLimit);
                    if (g.LateralMag < c.SafeDistance) lat *= -1;
                    else if (g.LateralMag < c.SafeDistance * 2.0) lat = 0;
                    status = string.Format("Switching to the correct side at {0:F2} m/s, lateral {1:F2} m/s", z, lat);
                    break;

                case DockingSteps.BackingUp:
                    if (g.LateralMag < c.SafeDistance) lat *= -1;
                    else if (g.LateralMag < c.SafeDistance * 2.0) lat = 0;
                    z = -MaxSpeed(1.0 + c.TargetSize - g.ZSep, accelInDir(-g.ZAxis), c.SpeedLimit);
                    align = false;
                    status = string.Format("Backing up at {0:F2} m/s", z);
                    break;

                case DockingSteps.MovingToStart:
                    if (g.ZSep < c.SafeDistance) z *= -1;
                    else z = 0;
                    status = string.Format("Aligning docking ports ({0:F2} m/s)", z);
                    break;

                case DockingSteps.Docking:
                    // Coordinate axial vs lateral so we stay in the corridor: if we'd reach the port before we
                    // are centred, slow the axial speed and speed up the lateral correction. Guard against zero
                    // speeds (e.g. RCS empty => accel 0 => both speeds 0): the time ratios would be inf/inf=NaN
                    // and poison the command, so only coordinate when both speeds are actually nonzero.
                    if (Math.Abs(z) > 1e-9 && Math.Abs(lat) > 1e-9)
                    {
                        double timeToAxis = Math.Abs(g.LateralMag / lat);
                        double timeToTargetSize = Math.Abs(g.ZSep / z);
                        if ((g.ZSep <= g.LateralMag * 10.0 || timeToTargetSize <= timeToAxis * 10.0)
                            && timeToAxis > 0.0 && timeToTargetSize > 0.0)
                        {
                            z *= Math.Min(timeToTargetSize / timeToAxis, 1.0);
                            lat = ClampSpeed(lat * 2.0, c.SpeedLimit);
                        }
                    }

                    status = string.Format("Docking at {0:F2} / {1:F2} m/s", z, lat);
                    break;

                default:
                    status = step.ToString();
                    break;
            }

            // World-frame approach velocity: slide back toward the centreline and move along the axis.
            Vector3d adjustment = -g.LateralDir * lat + z * g.ZAxis;
            return new DockingPlan { ZSpeed = z, LatSpeed = lat, Align = align, Adjustment = adjustment, Status = status };
        }
    }
}
