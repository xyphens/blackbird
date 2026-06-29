using Blackbird.Mathematics;
using System;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class AttitudeControl
    {
        private const double PosKpDefault = 2.03;
        private const double PosTiDefault = 1.97;
        private const double PosTdDefault = 0.0;
        private const double PosNDefault = 1.0;
        private const double PosBDefault = 1.0;
        private const double PosCDefault = 1.0;

        private const double VelKpDefault = 7.98;
        private const double VelTiDefault = 0.0;
        private const double VelTdDefault = 0.0;
        private const double VelNDefault = 1.0;
        private const double VelBDefault = 1.0;
        private const double VelCDefault = 1.0;

        private const double MaxStoppingTime = 2.0;
        private const double MinFlipTime = 120.0;
        private const double LockRollErrorPadding = 0.5; // lock roll without constantly running RCS
        private const double SmoothTorque = 0.10;
        private const double Soften = 0.5;

        private readonly PIDTranslation[] _velPid =
        {
            new PIDTranslation(),
            new PIDTranslation(),
            new PIDTranslation()
        };

        private readonly PIDTranslation[] _posPid =
        {
            new PIDTranslation(),
            new PIDTranslation(),
            new PIDTranslation()
        };

        private readonly DirectionTracking _directionTracking = new DirectionTracking();

        private Vector3d _actuation = Vector3d.zero;
        private Vector3d _controlTorque = Vector3d.zero;

        public void Reset()
        {
            for (int i = 0; i < 3; i++)
            {
                Reset(i);
            }

            _directionTracking.Reset();
            _controlTorque = Vector3d.zero;
            _actuation = Vector3d.zero;
        }

        public void Reset(int index)
        {
            _velPid[index].Reset();
            _posPid[index].Reset();
            _directionTracking.Reset(index);
        }

        public void Drive(
            Vessel vessel,
            FlightCtrlState state,
            double headingDeg,
            double pitchDeg,
            double rollDeg,
            bool lockRoll = false
            )
        {
            if (vessel == null || state == null) return;
            if (vessel.ReferenceTransform == null) return;

            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);

            QuaternionD requestedAttitude = GetSurfaceNorthReferenceRotation(vessel) * BuildSurfaceTargetAttitude(headingDeg, pitchDeg, rollDeg);

            Vector3d torqueAvailable = EstimateTorqueAvailable(vessel);

            Vector3d actuation = UpdatePredictionPI(vessel, requestedAttitude, torqueAvailable, lockRoll);

            ApplyActuation(state, actuation);
        }

        public void DriveInertial(
            Vessel vessel,
            FlightCtrlState state,
            Vector3d inertialDirection,
            double rollDeg,
            bool lockRoll = false
            )
        {
            WorldDirectionToHeadingPitch(vessel, inertialDirection, out double headingDeg, out double pitchDeg);
            Drive(vessel, state, headingDeg, pitchDeg, rollDeg, lockRoll);
        }

        public static void WorldDirectionToHeadingPitch(Vessel vessel, Vector3d worldDir, out double headingDeg, out double pitchDeg)
        {
            headingDeg = 0.0; pitchDeg = 0.0;
            if (vessel == null || vessel.mainBody == null || worldDir.sqrMagnitude <= 0.0) return;

            Vector3d position = vessel.GetWorldPos3D();
            Vector3d up = (position - vessel.mainBody.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vessel.mainBody.transform.up).normalized;
            if (north.sqrMagnitude <= 0.0) north = vessel.north;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            Vector3d direction = worldDir.normalized;
            Vector3d horizontal = Vector3d.Exclude(up, direction);

            pitchDeg = Math.Asin(MathHelpers.Clamp(Vector3d.Dot(direction, up), -1.0, 1.0)) * 180.0 / Math.PI;
            if (horizontal.sqrMagnitude > 0.0)
            {
                Vector3d h = horizontal.normalized;
                headingDeg = Math.Atan2(Vector3d.Dot(h, east), Vector3d.Dot(h, north)) * 180.0 / Math.PI;
                if (headingDeg < 0.0) headingDeg += 360.0;
            }
        }
        // Holds the craft's CURRENT attitude, killing all rotation (incl. roll). This is the analog of
        // MechJeb's attitudeTo(QuaternionD.LookRotation(transform.up, -transform.forward), INERTIAL): the
        // requested attitude is the current one, so the position error is ~0 and the inner rate loop drives
        // the angular velocity to zero. Re-captured every call (like MechJeb), so it brakes a tumble to a stop.
        public void DriveHoldAttitude(Vessel vessel, FlightCtrlState state)
        {
            if (vessel == null || state == null) return;
            if (vessel.ReferenceTransform == null) return;

            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);

            QuaternionD requestedAttitude =
                (QuaternionD)vessel.ReferenceTransform.rotation * Euler(-90.0, 0.0, 0.0);

            Vector3d torqueAvailable = EstimateTorqueAvailable(vessel);
            Vector3d actuation = UpdatePredictionPI(vessel, requestedAttitude, torqueAvailable);
            ApplyActuation(state, actuation);
        }

        private Vector3d UpdatePredictionPI(
            Vessel vessel,
            QuaternionD requestedAttitude,
            Vector3d torqueAvailable,
            bool lockRoll = false
            )
        {
            QuaternionD currentAttitude =
                (QuaternionD)vessel.ReferenceTransform.rotation *
                Euler(-90.0, 0.0, 0.0);

            Vector3d current = _directionTracking.Update(currentAttitude);

            _directionTracking.Desired(
                requestedAttitude,
                out Vector3d desired,
                out Vector3d error,
                out double distance);

            _controlTorque =
                _controlTorque == Vector3d.zero
                    ? torqueAvailable
                    : _controlTorque + SmoothTorque * (torqueAvailable - _controlTorque);

            for (int i = 0; i < 3; i++)
            {
                if (torqueAvailable[i] == 0.0)
                    _controlTorque[i] = 0.0;
            }

            double deltaT = TimeWarp.fixedDeltaTime;
            double warpFactor = deltaT / 0.02;

            Vector3d maxAlpha = Vector3d.zero;
            Vector3d targetOmega = Vector3d.zero;
            Vector3d targetAlpha = Vector3d.zero;
            Vector3d targetTorque = Vector3d.zero;

            for (int i = 0; i < 3; i++)
            {
                maxAlpha[i] = _controlTorque[i] / vessel.MOI[i];

                if (maxAlpha[i] == 0.0 || !MathHelpers.IsFinite(maxAlpha[i])) maxAlpha[i] = 1.0;

                double soften = MathHelpers.Clamp01(Soften);
                double posKp = PosKpDefault / warpFactor;
                double effectiveLinearDistance = soften * soften * maxAlpha[i] / (2.0 * posKp * posKp);

                double maxOmega = maxAlpha[i] * MaxStoppingTime;
                maxOmega = Math.Max(maxOmega, Math.PI / MinFlipTime);

                if (Math.Abs(error[i]) <= 2.0 * effectiveLinearDistance)
                {
                    PIDTranslation posPid = _posPid[i];

                    posPid.Kp = posKp;
                    posPid.Ti = PosTiDefault;
                    posPid.Td = PosTdDefault;
                    posPid.N = PosNDefault;
                    posPid.B = PosBDefault;
                    posPid.C = PosCDefault;
                    posPid.Ts = deltaT;
                    posPid.SmoothIn = 1.0;
                    posPid.SmoothOut = 1.0;
                    posPid.MinOutput = -maxOmega;
                    posPid.MaxOutput = maxOmega;
                    posPid.IntegralDeadband = 0.0;

                    targetOmega[i] = posPid.Update(desired[i], current[i]);
                }
                else
                {
                    _posPid[i].Reset();

                    targetOmega[i] =
                        soften *
                        Math.Sqrt(
                            2.0 *
                            maxAlpha[i] *
                            (Math.Abs(error[i]) - effectiveLinearDistance)) *
                        Math.Sign(error[i]);

                    targetOmega[i] = MathHelpers.Clamp(targetOmega[i], -maxOmega, maxOmega);
                }

                double rollError = distance * Mathf.Rad2Deg;

                if (lockRoll && rollError > LockRollErrorPadding)
                {
                    targetOmega[1] = 0.0;
                    _posPid[1].Reset();
                }

                PIDTranslation velPid = _velPid[i];

                velPid.Kp = VelKpDefault;
                velPid.Ti = VelTiDefault;
                velPid.Td = VelTdDefault;
                velPid.N = VelNDefault;
                velPid.B = VelBDefault;
                velPid.C = VelCDefault;
                velPid.Ts = deltaT;
                velPid.SmoothIn = 1.0;
                velPid.SmoothOut = 1.0;
                velPid.MinOutput = -maxAlpha[i];
                velPid.MaxOutput = maxAlpha[i];
                velPid.IntegralDeadband = 0.0;

                targetAlpha[i] =
                    velPid.Update(targetOmega[i], vessel.angularVelocityD[i]);

                targetTorque[i] =
                    vessel.MOI[i] * targetAlpha[i];

                _actuation[i] =
                    -targetTorque[i] / _controlTorque[i];

                if (_controlTorque[i] == 0.0 || !MathHelpers.IsFinite(_actuation[i]))
                {
                    _actuation[i] = 0.0;
                    Reset(i);
                }

                if (Math.Abs(_actuation[i]) < 2.2204460492503131e-16) _actuation[i] = 0.0;
            }

            return _actuation;
        }
        private static void ApplyActuation(FlightCtrlState state, Vector3d actuation)
        {
            bool userCommandingPitch =
                !Mathf.Approximately(state.pitch, state.pitchTrim);

            bool userCommandingYaw =
                !Mathf.Approximately(state.yaw, state.yawTrim);

            bool userCommandingRoll =
                !Mathf.Approximately(state.roll, state.rollTrim);

            if (!userCommandingRoll)
                state.roll = Mathf.Clamp((float)actuation.y, -1.0f, 1.0f);

            if (!userCommandingPitch && !userCommandingYaw)
            {
                state.pitch = Mathf.Clamp((float)actuation.x, -1.0f, 1.0f);
                state.yaw = Mathf.Clamp((float)actuation.z, -1.0f, 1.0f);
            }
        }
        private static QuaternionD BuildSurfaceTargetAttitude(
            double headingDeg,
            double pitchDeg,
            double rollDeg)
        {
            return
                QuaternionD.AngleAxis((float)headingDeg, Vector3.up) *
                QuaternionD.AngleAxis((float)-pitchDeg, Vector3.right) *
                QuaternionD.AngleAxis((float)-rollDeg, Vector3.forward);
        }

        private static QuaternionD GetSurfaceNorthReferenceRotation(Vessel vessel)
        {
            Vector3d centerOfMass = vessel.CoMD;
            Vector3d orbitalPosition = centerOfMass - vessel.mainBody.position;
            Vector3d surfaceUp = orbitalPosition.normalized;

            return QuaternionD.LookRotation(vessel.north, surfaceUp);
        }
        private static Vector3d EstimateTorqueAvailable(Vessel vessel)
        {
            Vector3d torque = Vector3d.zero;

            foreach (Part part in vessel.parts)
            {
                if (part == null) continue;

                foreach (PartModule module in part.Modules)
                {
                    if (module == null || !module.isEnabled) continue;

                    if (module is ModuleReactionWheel reactionWheel)
                    {
                        reactionWheel.GetPotentialTorque(
                            out Vector3 positive,
                            out Vector3 negative);

                        torque += MaxAbs(positive, negative);
                        continue;
                    }

                    if (module is ModuleControlSurface controlSurface)
                    {
                        controlSurface.GetPotentialTorque(
                            out Vector3 positive,
                            out Vector3 negative);

                        torque += MaxAbs(positive, negative);
                        continue;
                    }

                    if (module is ModuleGimbal gimbal)
                    {
                        gimbal.GetPotentialTorque(
                            out Vector3 positive,
                            out Vector3 negative);

                        torque += MaxAbs(positive, negative);
                        continue;
                    }

                    if (module is ITorqueProvider torqueProvider)
                    {
                        torqueProvider.GetPotentialTorque(
                            out Vector3 positive,
                            out Vector3 negative);

                        torque += MaxAbs(positive, negative);
                    }
                }
            }

            return torque;
        }

        // Estimates how long (seconds) it would take the attitude controller to swing the craft's nose
        // from its current facing to a target world-frame direction, then settle — so callers can size a
        // warp lead / pre-orient window for any maneuver that must fire pointed a particular way.
        //
        // Model (deliberately conservative, so it never under-predicts the lead needed):
        //  - angular acceleration alpha = available control torque / moment of inertia, evaluated on the
        //    WORST of the pitch and yaw axes (the two that swing the nose), so a weak axis dominates;
        //  - the same Soften factor and MaxStoppingTime rate cap the live controller uses, so the estimate
        //    tracks real slew behavior rather than an idealized bang-bang;
        //  - a rate-limited profile: triangular (accel to midpoint, decel) when the peak rate stays under
        //    the cap, otherwise trapezoidal (accel to the cap, cruise, decel);
        //  - plus the caller's padding (settle dwell + safety margin).
        // Returns just paddingSeconds for a negligible rotation, and a torque-free fallback
        // (angleRad + padding) when torque/MOI are unavailable.
        public static double EstimateSlewTimeSeconds(
            Vessel vessel,
            Vector3d targetWorldDirection,
            double paddingSeconds)
        {
            if (vessel == null || vessel.ReferenceTransform == null) return paddingSeconds;

            Vector3d target = targetWorldDirection.normalized;
            if (target.sqrMagnitude <= 0.0) return paddingSeconds;

            Vector3d nose = ((Vector3d)vessel.ReferenceTransform.up).normalized;
            double dot = MathHelpers.Clamp(Vector3d.Dot(nose, target), -1.0, 1.0);
            double angleRad = Math.Acos(dot);
            if (angleRad <= 1e-3) return paddingSeconds;

            Vector3d torque = EstimateTorqueAvailable(vessel);
            Vector3d moi = vessel.MOI;
            double alphaPitch = SafeAlpha(torque.x, moi.x);   // pitch axis
            double alphaYaw = SafeAlpha(torque.z, moi.z);     // yaw axis
            double alpha = Math.Min(alphaPitch, alphaYaw);    // worst nose-swing axis -> conservative
            if (!MathHelpers.IsFinite(alpha) || alpha <= 0.0) return angleRad + paddingSeconds;

            // Conservative accel/rate cap, matching the live controller's softening and stopping time.
            double alphaEff = alpha * Soften;
            double omegaMax = alpha * MaxStoppingTime * Soften;
            if (alphaEff <= 0.0 || omegaMax <= 0.0) return angleRad + paddingSeconds;

            double slew;
            double trianglePeak = Math.Sqrt(alphaEff * angleRad);   // peak rate of a pure accel/decel
            if (trianglePeak <= omegaMax)
            {
                slew = 2.0 * Math.Sqrt(angleRad / alphaEff);
            }
            else
            {
                double tRamp = omegaMax / alphaEff;                          // accel (or decel) duration
                double angleRamp = omegaMax * omegaMax / (2.0 * alphaEff);   // angle covered per ramp
                double angleCruise = angleRad - 2.0 * angleRamp;            // > 0 by the branch condition
                slew = 2.0 * tRamp + angleCruise / omegaMax;
            }

            return slew + paddingSeconds;
        }

        // Min available angular acceleration (rad/s^2) on the nose-swing axes
        public static double MinControlAngularAccel(Vessel vessel)
        {
            if (vessel == null) return 0.0;
            Vector3d torque = EstimateTorqueAvailable(vessel);
            Vector3d moi = vessel.MOI;
            double alpha = Math.Min(SafeAlpha(torque.x, moi.x), SafeAlpha(torque.z, moi.z));
            return MathHelpers.IsFinite(alpha) && alpha > 0.0 ? alpha : 0.0;
        }

        public static double MaxControlRateRadPerSec(Vessel vessel)
        {
            double a = MinControlAngularAccel(vessel);
            return a > 0.0 ? a * MaxStoppingTime * Soften : 0.0;
        }

        // Angular acceleration about one axis (|torque| / MOI). Returns +Infinity for a zero-inertia axis
        // (it never limits the slew) so Math.Min picks the genuinely limiting axis.
        private static double SafeAlpha(double torque, double moi)
        {
            if (moi <= 0.0) return double.PositiveInfinity;
            return Math.Abs(torque) / moi;
        }

        private static Vector3d MaxAbs(Vector3 positive, Vector3 negative)
        {
            return new Vector3d(
                Math.Max(Math.Abs(positive.x), Math.Abs(negative.x)),
                Math.Max(Math.Abs(positive.y), Math.Abs(negative.y)),
                Math.Max(Math.Abs(positive.z), Math.Abs(negative.z)));
        }

        private static QuaternionD Euler(double xDeg, double yDeg, double zDeg)
        {

            double x = MathHelpers.Deg2Rad(xDeg);
            double y = MathHelpers.Deg2Rad(yDeg);
            double z = MathHelpers.Deg2Rad(zDeg);

            double cx = Math.Cos(x * 0.5);
            double sx = Math.Sin(x * 0.5);
            double cy = Math.Cos(y * 0.5);
            double sy = Math.Sin(y * 0.5);
            double cz = Math.Cos(z * 0.5);
            double sz = Math.Sin(z * 0.5);

            return new QuaternionD
            {
                w = cz * cx * cy + sz * sx * sy,
                x = cz * sx * cy - sz * cx * sy,
                y = cz * cx * sy + sz * sx * cy,
                z = sz * cx * cy - cz * sx * sy
            };
        }

    }
}
