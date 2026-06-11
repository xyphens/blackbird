using Blackbird.Models;

namespace Blackbird.Models
{
    public sealed class LaunchPlan
    {
        public OrbitInfo ActiveOrbit { get; set; }
        public OrbitInfo TargetOrbit { get; set; }
        // expose launch window stats (valid window, time away, etc) 
        public LaunchWindowInfo LaunchWindow { get; set; }

        // compare orbits
        public double RelativeInclinationDeg { get; set; }
        public double RelativeLanDeg { get; set; }
        public double RelativePeriodSeconds { get; set; }

        public double PhaseAngleDeg { get; set; }
        public double DistanceMeters { get; set; }
        public double LaunchAzimuthDeg { get; set; }
        public double RecommendedApAlt { get; set; }
        public double RecommendedPeAlt { get; set; }
        public string ScaleLabel { get; set; }
        public InsertionTarget InsertionTarget { get; set; }
        public PhasingOrbit PhasingOrbit { get; set; }
        public PhasingRecommendation PhasingRecommendation;
    }
}
