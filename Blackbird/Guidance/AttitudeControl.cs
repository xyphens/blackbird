using System;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class AttitudeControl
    {
        /*
         * Portions of this controller are derived from the control architecture
         * used by the MechJeb2 project:
         *
         * https://github.com/MuMech/MechJeb2
         *
         * Specifically:
         * - DirectionTracker-style attitude tracking
         * - BetterController-style cascaded attitude PID control
         * - Surface attitude reference handling
         *
         * Original project licensed under GPL-3.0.
         *
         * BlackBird contains a custom implementation adapted for its own
         * architecture and user interface.
         */
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
        private const double RollControlRangeDeg = 5.0;
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
            double rollDeg)
        {
            if (vessel == null || state == null) return;
            if (vessel.ReferenceTransform == null) return;

            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);

            QuaternionD requestedAttitude = GetSurfaceNorthReferenceRotation(vessel) * BuildSurfaceTargetAttitude(headingDeg, pitchDeg, rollDeg);

            Vector3d torqueAvailable = EstimateTorqueAvailable(vessel);

            Vector3d actuation = UpdatePredictionPI(vessel, requestedAttitude, torqueAvailable);

            ApplyActuation(state, actuation);
        }

        private Vector3d UpdatePredictionPI(
            Vessel vessel,
            QuaternionD requestedAttitude,
            Vector3d torqueAvailable)
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

                if (maxAlpha[i] == 0.0 || !IsFinite(maxAlpha[i])) maxAlpha[i] = 1.0;

                double soften = Clamp01(Soften);
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

                    targetOmega[i] =
                        Clamp(targetOmega[i], -maxOmega, maxOmega);
                }

                if (distance * Mathf.Rad2Deg > RollControlRangeDeg)
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

                if (_controlTorque[i] == 0.0 || !IsFinite(_actuation[i]))
                {
                    _actuation[i] = 0.0;
                    Reset(i);
                }

                if (Math.Abs(_actuation[i]) < 2.2204460492503131e-16)
                    _actuation[i] = 0.0;
            }

            return _actuation;
        }
        private static void ApplyActuation(
            FlightCtrlState state,
            Vector3d actuation)
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

        private static Vector3d MaxAbs(Vector3 positive, Vector3 negative)
        {
            return new Vector3d(
                Math.Max(Math.Abs(positive.x), Math.Abs(negative.x)),
                Math.Max(Math.Abs(positive.y), Math.Abs(negative.y)),
                Math.Max(Math.Abs(positive.z), Math.Abs(negative.z)));
        }

        private static QuaternionD Euler(double xDeg, double yDeg, double zDeg)
        {
            double x = xDeg * Math.PI / 180.0;
            double y = yDeg * Math.PI / 180.0;
            double z = zDeg * Math.PI / 180.0;

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

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        private static double Clamp01(double value)
        {
            return Clamp(value, 0.0, 1.0);
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
