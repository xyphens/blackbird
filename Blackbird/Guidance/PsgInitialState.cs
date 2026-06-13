using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class PsgInitialState
    {
        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public Vector3d RelativePositionMeters { get; private set; }
        public Vector3d RelativeVelocityMetersPerSecond { get; private set; }
        public double MassKg { get; private set; }
        public double UniversalTime { get; private set; }

        public static PsgInitialState Create(
            Vector3d relativePositionMeters,
            Vector3d relativeVelocityMetersPerSecond,
            double massKg,
            double universalTime)
        {
            if (relativePositionMeters.sqrMagnitude <= 0.0)
            {
                return CreateInvalid("Initial position is invalid.");
            }

            if (!OrbitMath.IsFinite(massKg) || massKg <= 0.0)
            {
                return CreateInvalid("Initial mass is invalid.");
            }

            if (!OrbitMath.IsFinite(universalTime))
            {
                return CreateInvalid("Initial universal time is invalid.");
            }

            return new PsgInitialState
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                RelativePositionMeters = relativePositionMeters,
                RelativeVelocityMetersPerSecond = relativeVelocityMetersPerSecond,
                MassKg = massKg,
                UniversalTime = universalTime
            };
        }

        private static PsgInitialState CreateInvalid(string reason)
        {
            return new PsgInitialState
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reason) ? "PSG initial state is unavailable." : reason
            };
        }
    }
}
