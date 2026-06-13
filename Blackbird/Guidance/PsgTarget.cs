using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PsgTarget
    {
        // TODO(MechJeb parity): split this simple target container into AscentBuilder-style terminal selection.
        // MechJeb chooses FlightPathAngle or Kepler terminal constraints based on fixed burn/coast state,
        // near-circular attachment, and which orbital elements are constrained.
        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public double PeriapsisRadiusMeters { get; private set; }
        public double ApoapsisRadiusMeters { get; private set; }
        public double AttachmentRadiusMeters { get; private set; }
        public double InclinationDeg { get; private set; }
        public double LanDeg { get; private set; }
        public double FlightPathAngleDeg { get; private set; }
        public Vector3d TargetOrbitNormal { get; private set; }
        public double TargetSpecificEnergy { get; private set; }
        public Vector3d TargetAngularMomentumVector { get; private set; }

        public bool UseAttachmentRadius { get; private set; }
        public bool UseLanConstraint { get; private set; }

        public static PsgTarget Create(
            double bodyGravParameter,
            double periapsisRadiusMeters,
            double apoapsisRadiusMeters,
            double attachmentRadiusMeters,
            Vector3d targetOrbitNormal,
            double inclinationDeg,
            double lanDeg,
            bool useLanConstraint)
        {
            if (!OrbitMath.IsFinite(bodyGravParameter) || bodyGravParameter <= 0.0)
            {
                return CreateInvalid("Target body gravitational parameter is invalid.");
            }

            if (!OrbitMath.IsFinite(periapsisRadiusMeters) || periapsisRadiusMeters <= 0.0 ||
                !OrbitMath.IsFinite(apoapsisRadiusMeters) || apoapsisRadiusMeters <= 0.0 ||
                !OrbitMath.IsFinite(attachmentRadiusMeters) || attachmentRadiusMeters <= 0.0)
            {
                return CreateInvalid("Target radii are invalid.");
            }

            if (apoapsisRadiusMeters < periapsisRadiusMeters)
            {
                double temp = apoapsisRadiusMeters;
                apoapsisRadiusMeters = periapsisRadiusMeters;
                periapsisRadiusMeters = temp;
            }

            double semiMajorAxis = (apoapsisRadiusMeters + periapsisRadiusMeters) * 0.5;
            double semiLatusRectum = 2.0 * apoapsisRadiusMeters * periapsisRadiusMeters /
                                     (apoapsisRadiusMeters + periapsisRadiusMeters);
            double specificEnergy = -bodyGravParameter / (2.0 * semiMajorAxis);
            double angularMomentumMagnitude = System.Math.Sqrt(bodyGravParameter * semiLatusRectum);
            Vector3d normal = targetOrbitNormal.sqrMagnitude > 0.0 ? targetOrbitNormal.normalized : Vector3d.zero;

            return new PsgTarget
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                PeriapsisRadiusMeters = periapsisRadiusMeters,
                ApoapsisRadiusMeters = apoapsisRadiusMeters,
                AttachmentRadiusMeters = attachmentRadiusMeters,
                InclinationDeg = inclinationDeg,
                LanDeg = lanDeg,
                FlightPathAngleDeg = 0.0,
                TargetOrbitNormal = normal,
                TargetSpecificEnergy = specificEnergy,
                TargetAngularMomentumVector = normal * angularMomentumMagnitude,
                UseAttachmentRadius = true,
                UseLanConstraint = useLanConstraint
            };
        }

        public static PsgTarget FromPlan(
            VesselState vesselState,
            LaunchPlan launchPlan,
            AscentProfile ascentProfile)
        {
            OrbitInfo targetOrbit = launchPlan != null ? launchPlan.TargetOrbit : null;
            Vector3d orbitNormal = launchPlan != null ? launchPlan.TargetOrbitNormal : Vector3d.zero;
            return FromProfile(vesselState, ascentProfile, targetOrbit, orbitNormal);
        }

        public static PsgTarget FromProfile(
            VesselState vesselState,
            AscentProfile ascentProfile,
            OrbitInfo targetOrbit)
        {
            return FromProfile(vesselState, ascentProfile, targetOrbit, Vector3d.zero);
        }

        public static PsgTarget FromProfile(
            VesselState vesselState,
            AscentProfile ascentProfile,
            OrbitInfo targetOrbit,
            Vector3d targetOrbitNormal)
        {
            if (vesselState == null || ascentProfile == null)
            {
                return CreateInvalid("PSG target inputs are unavailable.");
            }

            if (!ascentProfile.IsValid)
            {
                return CreateInvalid(ascentProfile.ReasonUnavailable);
            }

            if (!OrbitMath.IsFinite(vesselState.BodyRadius) || vesselState.BodyRadius <= 0.0)
            {
                return CreateInvalid("Reference body radius is unavailable.");
            }

            double periapsisRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;
            double apoapsisRadius = vesselState.BodyRadius + ascentProfile.TargetApoapsisAlt;

            if (!OrbitMath.IsFinite(periapsisRadius) || periapsisRadius <= vesselState.BodyRadius ||
                !OrbitMath.IsFinite(apoapsisRadius) || apoapsisRadius <= vesselState.BodyRadius)
            {
                return CreateInvalid("Insertion apsides are unavailable.");
            }

            if (apoapsisRadius < periapsisRadius)
            {
                double temp = apoapsisRadius;
                apoapsisRadius = periapsisRadius;
                periapsisRadius = temp;
            }

            // TODO(MechJeb parity): this always attaches at periapsis. MechJeb only forces periapsis
            // attachment for near-circular free-duration targets, otherwise it selects the terminal
            // family and attachment state from the target flags.
            double attachmentRadius = periapsisRadius;
            double semiMajorAxis = (apoapsisRadius + periapsisRadius) * 0.5;
            double semiLatusRectum = 2.0 * apoapsisRadius * periapsisRadius / (apoapsisRadius + periapsisRadius);
            double specificEnergy = -vesselState.BodyGravParameter / (2.0 * semiMajorAxis);
            double angularMomentumMagnitude = System.Math.Sqrt(vesselState.BodyGravParameter * semiLatusRectum);
            Vector3d normal = targetOrbitNormal.sqrMagnitude > 0.0 ? targetOrbitNormal.normalized : Vector3d.zero;
            double inclination = targetOrbit != null && OrbitMath.IsFinite(targetOrbit.InclinationDeg)
                ? targetOrbit.InclinationDeg
                : vesselState.CurrentInclinationDeg;
            double lan = targetOrbit != null && OrbitMath.IsFinite(targetOrbit.LanDeg)
                ? targetOrbit.LanDeg
                : vesselState.CurrentLanDeg;

            return new PsgTarget
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                PeriapsisRadiusMeters = periapsisRadius,
                ApoapsisRadiusMeters = apoapsisRadius,
                AttachmentRadiusMeters = attachmentRadius,
                InclinationDeg = inclination,
                LanDeg = lan,
                FlightPathAngleDeg = 0.0,
                TargetOrbitNormal = normal,
                TargetSpecificEnergy = specificEnergy,
                TargetAngularMomentumVector = normal * angularMomentumMagnitude,
                UseAttachmentRadius = true,
                UseLanConstraint = OrbitMath.IsFinite(lan)
            };
        }

        private static PsgTarget CreateInvalid(string reason)
        {
            return new PsgTarget
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reason) ? "PSG target is unavailable." : reason,
                PeriapsisRadiusMeters = double.NaN,
                ApoapsisRadiusMeters = double.NaN,
                AttachmentRadiusMeters = double.NaN,
                InclinationDeg = double.NaN,
                LanDeg = double.NaN,
                FlightPathAngleDeg = double.NaN,
                TargetOrbitNormal = Vector3d.zero,
                TargetSpecificEnergy = double.NaN,
                TargetAngularMomentumVector = Vector3d.zero
            };
        }
    }
}
