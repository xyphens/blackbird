using System;

namespace Blackbird.Models
{
    public sealed class AscentPrediction
    {
        public double ProjectedApoapsisAlt { get; set; }
        public double ProjectedPeriapsisAlt { get; set; }

        public double EstimatedTimeToInsertionSeconds { get; set; }
        public double EstimatedRemainingDeltaV { get; set; }

        public double TargetFutureLatitudeDeg { get; set; }
        public double TargetFutureLongitudeDeg { get; set; }
        public double TargetFutureAltitudeMeters { get; set; }

        public double DesiredHeadingDeg { get; set; }
        public double DesiredPitchDeg { get; set; }

        public bool HasDeltaVEstimate { get; set; }
        public bool HasTargetPrediction { get; set; }
    }
}