using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PsgBodyModel
    {
        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public double GravParameter { get; private set; }
        public double RadiusMeters { get; private set; }
        public Vector3d AngularVelocityRadiansPerSecond { get; private set; }

        public static PsgBodyModel Create(
            double gravParameter,
            double radiusMeters,
            Vector3d angularVelocityRadiansPerSecond)
        {
            if (!OrbitMath.IsFinite(gravParameter) || gravParameter <= 0.0)
            {
                return CreateInvalid("Body gravitational parameter is invalid.");
            }

            if (!OrbitMath.IsFinite(radiusMeters) || radiusMeters <= 0.0)
            {
                return CreateInvalid("Body radius is invalid.");
            }

            return new PsgBodyModel
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                GravParameter = gravParameter,
                RadiusMeters = radiusMeters,
                AngularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond
            };
        }

        private static PsgBodyModel CreateInvalid(string reason)
        {
            return new PsgBodyModel
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reason) ? "PSG body model is unavailable." : reason
            };
        }
    }
}
