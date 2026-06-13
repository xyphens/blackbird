using UnityEngine;

namespace Blackbird.Models
{
    public sealed class TrajectoryState
    {
        public bool IsValid { get; set; }
        public string Source { get; set; }
        public string ReasonUnavailable { get; set; }

        public Vessel Vessel { get; set; }
        public CelestialBody ReferenceBody { get; set; }
        public double UniversalTime { get; set; }

        public Vector3d WorldPosition { get; set; }
        public Vector3d WorldVelocity { get; set; }
        public Vector3d RelativePosition { get; set; }
        public Vector3d RelativeVelocity { get; set; }

        public double AltitudeMeters { get; set; }
        public double LatitudeDeg { get; set; }
        public double LongitudeDeg { get; set; }
    }
}
