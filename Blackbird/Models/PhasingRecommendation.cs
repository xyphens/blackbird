using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.Models
{
    public sealed class PhasingRecommendation
    {
        public PhasingRecommendationMode Mode;

        public double ApoapsisAlt;
        public double PeriapsisAlt;

        public double PeriodSeconds;
        public double TargetPeriodSeconds;
        public double PeriodDifferenceSeconds;

        public double PhaseGainDegPerOrbit;

        public double EstimatedOrbitsToRendezvous;
        public double EstimatedTimeToRendezvousSeconds;

        public bool HasRecommendation;
        public string ReasonUnavailable;
    }
}
