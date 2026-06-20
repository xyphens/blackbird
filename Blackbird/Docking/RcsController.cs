using System;
using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Docking
{
    public class RcsController
    {
        public Vector3d targetVelocity = Vector3d.zero;
        public readonly RcsPID pid;

        private Vector3d lastTranslation = Vector3d.zero;
        private Vector3d worldVelocityDelta = Vector3d.zero;
        private Vector3d lastWorldVelocityDelta = Vector3d.zero;

        private VesselState State;

        private enum TranslationTypes
        {
            TARGET_WORLD_VELOCITY,
            WORLD_VELOCITY_ERROR,
            TARGET_RELATIVE_VELOCITY,
            TARGET_RELATIVE_POSITION
        }

        private TranslationTypes TranslationType;

        private readonly bool efficientTranslation = false; // conserve fuel, should be an input
        private readonly double minRcsTranslationMagnitude = 0.05; // don't use RCS if required thrust is below this
        // Controller time constant (MechJeb's "Tf"): the single knob the PID gains are derived from.
        // MUST default to 1.0 (MechJeb's default) — SetParameters floors it at 0.02, so a 0 default would
        // produce ~300x gains (Kp ∝ 1/Tf²) and slam the RCS. At Tf=1, Kp works out to ~0.125.
        public double timeConstant = 1.0; // tunes the RCS PID rather than hand-tuning PID gains
        // PID gains (K) initialized with defaults
        public double Kp = 0.125; // proportional
        public double Ki = 0.07; // integral
        public double Kd = 0.53; // derivative
        // autopilot enabled/disabled
        public bool rcsManualControl;
        public bool rcsThrottleEnabled = true; // H and N translation allowed
        public bool rcsRotationEnabled = true; // can use RCS for orientation

        // used to determine RCS acceleration and current velocity multiple
        public double rcsAccelerationFactor() => pid.Kp; // todo: possibly rename variable

        public RcsController()
        {
            pid = new RcsPID(Kp, Ki, Kd);
        }

        // [x] SetTargetWorldVelocity(Vector3d)

        // [x] Drive(FlightCtrlState s) as Update()

        // [x] setPIDParameters() as SetParameters()

        // [x] rcsAccelFactor() → pid.Kp

        public void SetParameters()
        {
            if (rcsManualControl)
            {
                pid.Kp = Kp;
                pid.Ki = Ki;
                pid.Kd = Kd;
            } else
            {
                timeConstant = Math.Max(timeConstant, 0.02);
                pid.Kd = 0.53 / timeConstant; // du: where is 0.53 derived?
                pid.Kp = pid.Kd / (3 * Math.Sqrt(2) * timeConstant);
                pid.Ki = pid.Kp / (12 * Math.Sqrt(2) * timeConstant);

                Kp = pid.Kp;
                Ki = pid.Ki;
                Kd = pid.Kd;
            }
        }

        public void SetTargetWorldVelocity(Vector3d velocity)
        {
            targetVelocity = velocity;
            TranslationType = TranslationTypes.TARGET_WORLD_VELOCITY;
        }

        // Raw translation: thrust along a world-frame direction with NO velocity matching (like holding the
        // H/N/I/J/K/L keys). worldVelocityDelta is the error the controller nulls, so a delta of -dv makes it
        // thrust toward +dv; we set it directly here instead of deriving it from a target velocity, so there is
        // no drift correction on the other axes.
        public void SetWorldVelocityError(Vector3d dv)
        {
            worldVelocityDelta = -dv;
            if (TranslationType != TranslationTypes.WORLD_VELOCITY_ERROR)
            {
                lastWorldVelocityDelta = worldVelocityDelta;   // avoid a one-frame derivative spike on the mode switch
                TranslationType = TranslationTypes.WORLD_VELOCITY_ERROR;
            }
        }

        public void Drive(FlightCtrlState ctrlState, VesselState vs, Vessel v)
        {
            SetParameters();
            State = vs;

            ITargetable target = FlightGlobals.fetch != null ? FlightGlobals.fetch.VesselTarget : null;
            Vessel targetVessel = target as Vessel;

            switch (TranslationType)
            {
                case TranslationTypes.TARGET_WORLD_VELOCITY:
                    worldVelocityDelta = State.OrbitalVelocity - targetVelocity; // du: think we want to pull OV from Principia?
                    break;
                case TranslationTypes.TARGET_RELATIVE_VELOCITY:
                    if (FlightGlobals.fetch != null && FlightGlobals.fetch.VesselTarget != null)
                    {
                        Vector3d myVesselVel = TrajectoryProvider.GetOrbitalVelocity(v);
                        Vector3d targetVesselVel = TrajectoryProvider.GetOrbitalVelocity(targetVessel);
                        Vector3d relativeVelocity = myVesselVel - targetVesselVel;
                        // our velocity - our target's vessel - the velocity we want
                        worldVelocityDelta = relativeVelocity - targetVelocity;
                    }
                    
                    break;
            }

            Vector3d velocityDelta = Quaternion.Inverse(v.GetTransform().rotation) * worldVelocityDelta;

            if (!efficientTranslation || velocityDelta.magnitude > minRcsTranslationMagnitude)
            {
                // toggle RCS on
                if (!v.ActionGroups[KSPActionGroup.RCS]) v.ActionGroups.SetGroup(KSPActionGroup.RCS, true);

                Vector3d rcs = new Vector3d();

                for (int i = 0; i < ThrustEnvelope.OrientationValues.Length; i++)
                {
                    ThrustEnvelope.Orientation orientation = ThrustEnvelope.OrientationValues[i];
                    double orDv = Vector3d.Dot(velocityDelta, ThrustEnvelope.Orientations[(int) orientation]);
                    double orAvail = vs.AvailableRcsThrust[orientation]; // du: i'm assuming VesselState is being rebuilt every time it's passed here so this is updated?
                    if (orAvail > 0 && Math.Abs(orDv) > 0.001)
                    {
                        double orAction = orDv / (orAvail * TimeWarp.fixedDeltaTime / vs.TotalMass);
                        if (orAction > 0) {
                            rcs += ThrustEnvelope.Orientations[(int)orientation] * orAction;
                        }
                    }
                }

                Vector3d omega = Vector3d.zero;

                if (TranslationType == TranslationTypes.TARGET_WORLD_VELOCITY)
                {
                    omega = Quaternion.Inverse(v.GetTransform().rotation) * (v.acceleration - vs.GravityForce);
                } else if (TranslationType == TranslationTypes.TARGET_RELATIVE_VELOCITY 
                    || TranslationType == TranslationTypes.WORLD_VELOCITY_ERROR)
                {
                    omega = (worldVelocityDelta - lastWorldVelocityDelta) / TimeWarp.fixedDeltaTime;
                    lastWorldVelocityDelta = worldVelocityDelta;
                }

                rcs = pid.ComputeAction(rcs, omega);
                lastTranslation = rcs;

                // modify the flight control state
                ctrlState.X = Mathf.Clamp((float)rcs.x, -1, 1);
                // z and y are swapped
                ctrlState.Y = Mathf.Clamp((float)rcs.z, -1, 1);
                ctrlState.Z = Mathf.Clamp((float)rcs.y, -1, 1);
            } else if (efficientTranslation && v.ActionGroups[KSPActionGroup.RCS])
            {
                // disable RCS
                v.ActionGroups.SetGroup(KSPActionGroup.RCS, false);
            }
        }
    }
}