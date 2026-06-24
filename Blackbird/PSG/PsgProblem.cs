using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Psg
{
    public sealed class PsgProblem
    {
        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public Vector3d InitialRelativePositionMeters { get; private set; }
        public Vector3d InitialThrustDirection { get; private set; }
        public Vector3d CurrentRelativeVelocityMetersPerSecond { get; private set; }
        public double CurrentMassKg { get; private set; }
        public double CurrentUniversalTime { get; private set; }
        public double CurrentAltitudeMeters { get; private set; }
        public double CurrentVerticalSpeedMetersPerSecond { get; private set; }
        public double CurrentApoapsisAltMeters { get; private set; }
        public double CurrentPeriapsisAltMeters { get; private set; }
        // J2
        public double J2 { get; private set; }
        public double ReferenceRadiusMeters { get; private set; }

        public double BodyGravParameter { get; private set; }
        public double BodyRadiusMeters { get; private set; }
        public Vector3d BodyAngularVelocityRadiansPerSecond { get; private set; }
        public double AtmosphereScaleHeightMeters { get; private set; }

        public PsgPhase[] Phases { get; private set; }
        public PsgTarget Target { get; private set; }

        public static PsgProblem Create(
            PsgInitialState initialState,
            PsgBodyModel bodyModel,
            PsgTarget target,
            PsgPhase[] phases,
            Vector3d initialThrustDirection)
        {
            if (initialState == null || !initialState.IsValid)
            {
                return CreateInvalid(initialState != null ? initialState.ReasonUnavailable : "PSG initial state is unavailable.");
            }

            if (bodyModel == null || !bodyModel.IsValid)
            {
                return CreateInvalid(bodyModel != null ? bodyModel.ReasonUnavailable : "PSG body model is unavailable.");
            }

            if (target == null || !target.IsValid)
            {
                return CreateInvalid(target != null ? target.ReasonUnavailable : "PSG target is unavailable.");
            }

            if (phases == null || phases.Length == 0)
            {
                return CreateInvalid("No powered PSG phases are available.");
            }

            Vector3d thrustDirection = initialThrustDirection.sqrMagnitude > 0.0
                ? initialThrustDirection.normalized
                : initialState.RelativeVelocityMetersPerSecond.normalized;

            if (thrustDirection.sqrMagnitude <= 0.0)
            {
                return CreateInvalid("Initial thrust direction is unavailable.");
            }

            return new PsgProblem
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                InitialRelativePositionMeters = initialState.RelativePositionMeters,
                CurrentRelativeVelocityMetersPerSecond = initialState.RelativeVelocityMetersPerSecond,
                InitialThrustDirection = thrustDirection,
                CurrentMassKg = initialState.MassKg,
                CurrentUniversalTime = initialState.UniversalTime,
                CurrentAltitudeMeters = initialState.RelativePositionMeters.magnitude - bodyModel.RadiusMeters,
                CurrentVerticalSpeedMetersPerSecond = GetRadialVelocity(
                    initialState.RelativePositionMeters,
                    initialState.RelativeVelocityMetersPerSecond),
                CurrentApoapsisAltMeters = double.NaN,
                CurrentPeriapsisAltMeters = double.NaN,
                BodyGravParameter = bodyModel.GravParameter,
                BodyRadiusMeters = bodyModel.RadiusMeters,
                BodyAngularVelocityRadiansPerSecond = bodyModel.AngularVelocityRadiansPerSecond,
                AtmosphereScaleHeightMeters = 0.0,
                Phases = phases,
                Target = target
            };
        }

        public static PsgProblem Create(
            VesselState vesselState,
            PsgTarget target,
            PsgPhase[] phases,
            Vector3d initialThrustDirection)
        {
            if (vesselState == null)
            {
                return CreateInvalid("Vessel state is unavailable.");
            }

            if (target == null || !target.IsValid)
            {
                return CreateInvalid(target != null ? target.ReasonUnavailable : "PSG target is unavailable.");
            }

            if (phases == null || phases.Length == 0)
            {
                return CreateInvalid("No powered PSG phases are available.");
            }

            if (!MathHelpers.IsFinite(vesselState.TotalMass) || vesselState.TotalMass <= 0.0)
            {
                return CreateInvalid("Vessel mass is unavailable.");
            }

            if (vesselState.Body == null ||
                !MathHelpers.IsFinite(vesselState.BodyGravParameter) || vesselState.BodyGravParameter <= 0.0 ||
                !MathHelpers.IsFinite(vesselState.BodyRadius) || vesselState.BodyRadius <= 0.0)
            {
                return CreateInvalid("Reference body constants are unavailable.");
            }

            Vector3d relativePosition = vesselState.Position - vesselState.Body.position;
            Vector3d thrustDirection = initialThrustDirection.sqrMagnitude > 0.0
                ? initialThrustDirection.normalized
                : vesselState.OrbitalVelocity.normalized;

            if (relativePosition.sqrMagnitude <= 0.0 || thrustDirection.sqrMagnitude <= 0.0)
            {
                return CreateInvalid("Initial state vectors are unavailable.");
            }

            BodyOblateness.Oblateness ob = BodyOblateness.For(vesselState.Body);

            return new PsgProblem
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                InitialRelativePositionMeters = relativePosition,
                CurrentRelativeVelocityMetersPerSecond = vesselState.OrbitalVelocity,
                InitialThrustDirection = thrustDirection,
                CurrentMassKg = vesselState.TotalMass * 1000.0,
                CurrentUniversalTime = vesselState.UniversalTime,
                CurrentAltitudeMeters = vesselState.AltitudeMeters,
                CurrentVerticalSpeedMetersPerSecond = vesselState.VerticalSpeed,
                CurrentApoapsisAltMeters = vesselState.CurrentApoapsisAlt,
                CurrentPeriapsisAltMeters = vesselState.CurrentPeriapsisAlt,
                J2=ob.J2,
                ReferenceRadiusMeters = ob.ReferenceRadiusMeters,
                BodyGravParameter = vesselState.BodyGravParameter,
                BodyRadiusMeters = vesselState.BodyRadius,
                BodyAngularVelocityRadiansPerSecond = GetBodyAngularVelocity(vesselState),
                AtmosphereScaleHeightMeters = GetAtmosphereScaleHeight(vesselState.Body),
                Phases = phases,
                Target = target
            };
        }

        private static double GetAtmosphereScaleHeight(CelestialBody body)
        {
            if (body == null || !body.atmosphere || body.atmosphereDepth <= 0.0) return 0.0;

            double sampleAltitude = body.atmosphereDepth * 0.15;
            double rho0 = body.atmDensityASL;
            double rho1 = body.GetDensity(body.GetPressure(sampleAltitude), body.GetTemperature(sampleAltitude));

            if (!MathHelpers.IsFinite(rho0) || !MathHelpers.IsFinite(rho1) || rho0 <= 0.0 || rho1 <= 0.0 || rho0 <= rho1)
            {
                return 0.0;
            }

            double scaleHeight = sampleAltitude / System.Math.Log(rho0 / rho1);
            return MathHelpers.IsFinite(scaleHeight) && scaleHeight > 0.0 ? scaleHeight : 0.0;
        }

        private static Vector3d GetBodyAngularVelocity(VesselState vesselState)
        {
            if (vesselState == null ||
                !MathHelpers.IsFinite(vesselState.BodyRotationPeriod) ||
                vesselState.BodyRotationPeriod <= 0.0)
            {
                return Vector3d.zero;
            }

            Vector3 up = vesselState.Body.transform.up.normalized;
            Vector3d axis = new Vector3d(up.x, up.y, up.z);

            return axis * (2.0 * System.Math.PI / vesselState.BodyRotationPeriod);
        }

        private static double GetRadialVelocity(Vector3d relativePosition, Vector3d relativeVelocity)
        {
            double radius = relativePosition.magnitude;
            if (radius <= 0.0) return double.NaN;

            return Vector3d.Dot(relativePosition, relativeVelocity) / radius;
        }

        private static PsgProblem CreateInvalid(string reason)
        {
            return new PsgProblem
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reason) ? "PSG problem is unavailable." : reason,
                Phases = new PsgPhase[0]
            };
        }
    }
}
