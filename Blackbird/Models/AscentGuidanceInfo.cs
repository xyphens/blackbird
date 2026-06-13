using Blackbird.Enums;

namespace Blackbird.Models
{
    public sealed class AscentGuidanceInfo
    {
        public GuidanceMode GuidanceMode { get; set; }
        public string GuidancePhase { get; set; }

        public double ProfilePitchDeg { get; set; }
        public double ProfileHeadingDeg { get; set; }
        public double ProfileThrottle { get; set; }

        public double CommandPitchDeg { get; set; }
        public double CommandHeadingDeg { get; set; }
        public double CommandThrottle { get; set; }

        public double CurrentPitchDeg { get; set; }
        public double CurrentHeadingDeg { get; set; }

        public double PitchErrorDeg { get; set; }
        public double HeadingErrorDeg { get; set; }

        public string PitchInstruction { get; set; }
        public string HeadingInstruction { get; set; }

        public double TargetApoapsisAlt { get; set; }
        public double TargetPeriapsisAlt { get; set; }
        public double ApoapsisErrorMeters { get; set; }
        public double PeriapsisErrorMeters { get; set; }
        public double GuidanceTimeToGoSeconds { get; set; }
        public double GuidanceVelocityToGoMetersPerSecond { get; set; }

        public double PredictedApoapsisAlt { get; set; }
        public double PredictedPeriapsisAlt { get; set; }

        public double EstimatedDeltaVUsed { get; set; }
        public double EstimatedRemainingDeltaV { get; set; }
        public double EstimatedInsertionTimeSeconds { get; set; }
        public double EstimatedOrbitsToRendezvous { get; set; }

        public double PlaneErrorDeg { get; set; }
        public double PhaseErrorDeg { get; set; }
        public double RelativeDistanceMeters { get; set; }
    }
}
