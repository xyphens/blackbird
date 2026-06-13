using Blackbird.Guidance;

namespace Blackbird.Models
{
    public sealed class LaunchCandidate
    {
        public bool IsValid { get; set; }
        public string ReasonUnavailable { get; set; }

        public double LaunchUt { get; set; }
        public double SecondsUntilLaunch { get; set; }

        public double InsertionApoapsisAlt { get; set; }
        public double InsertionPeriapsisAlt { get; set; }
        public double LaunchHeadingDeg { get; set; }

        public double EstimatedInsertionTimeSeconds { get; set; }
        public double EstimatedOrbitsToRendezvous { get; set; }

        public double EstimatedDeltaVUsed { get; set; }
        public double EstimatedRemainingDeltaV { get; set; }

        public double PlaneErrorDeg { get; set; }
        public double PhaseErrorDeg { get; set; }
        public double RelativeDistanceMeters { get; set; }
        public double Score { get; set; }

        public AscentProfile AscentProfile { get; set; }
        public PhasingRecommendation PhasingRecommendation { get; set; }
    }
}
