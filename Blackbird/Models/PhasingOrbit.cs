using Blackbird.Mathematics;
using Blackbird.Models;
using System;

namespace Blackbird.Models
{
    public sealed class PhasingOrbit
    {
        public double ApoapsisAlt { get; set; }
        public double PeriapsisAlt { get; set; }
        public double PeriodSeconds { get; set; }
        public double TargetPeriodSeconds { get; set; }
        public double PeriodDifferenceSeconds { get; set; }
        public double RelativePhaseGainDegPerOrbit { get; private set; }
        public double EstimatedOrbitsToRendezvous { get; private set; }
        public double EstimatedTimeToRendezvousSeconds { get; private set; }
        public bool HasRendezvousEstimate { get; private set; }
        public double PeriodDifferenceMinutes
        {
            get { return PeriodDifferenceSeconds / 60.0; }
        }
        public double PeriodDifferencePercent
        {
            get
            {
                if (TargetPeriodSeconds == 0.0) return 0.0;
                return PeriodDifferenceSeconds / TargetPeriodSeconds * 100.0;
            }
        }
        public bool IsFasterThanTarget
        {
            get { return PeriodDifferenceSeconds < 0.0; }
        }
        public static PhasingOrbit FromInsertionTarget(
                                    InsertionTarget insertionTarget, 
                                    OrbitInfo targetOrbit, 
                                    CelestialBody body,
                                    double phaseAngleDeg)
        {
            if (insertionTarget == null || targetOrbit == null) return null;

            double period = CalculatePeriodSeconds(insertionTarget.ApoapsisAlt, insertionTarget.PeriapsisAlt, body);

            // positive = insertion orbit gains on target, negative = target pulls away
            double phaseGainPerOrbit = 360.0 * (targetOrbit.PeriodSeconds - period) / targetOrbit.PeriodSeconds;

            bool hasEstimate = Math.Abs(phaseGainPerOrbit) > 0.001;

            double orbitsToRendezvous = 0.0;
            double timeToRendezvous = 0.0;

            double phaseGainAbs =
                Math.Abs(phaseGainPerOrbit);

            if (phaseGainAbs > 0.001)
            {
                double phaseToClose = Math.Abs(OrbitMath.DeltaDegrees(phaseAngleDeg, 0.0));

                hasEstimate = true;

                orbitsToRendezvous = phaseToClose / phaseGainAbs;

                timeToRendezvous = orbitsToRendezvous * period;
            }

            return new PhasingOrbit
            {
                EstimatedOrbitsToRendezvous = orbitsToRendezvous,
                EstimatedTimeToRendezvousSeconds = timeToRendezvous,
                HasRendezvousEstimate = hasEstimate,
                ApoapsisAlt = insertionTarget.ApoapsisAlt,
                PeriapsisAlt = insertionTarget.PeriapsisAlt,
                PeriodSeconds = period,
                TargetPeriodSeconds = targetOrbit.PeriodSeconds,
                PeriodDifferenceSeconds = period - targetOrbit.PeriodSeconds,
                RelativePhaseGainDegPerOrbit = phaseGainPerOrbit
            };
        }

        private static double CalculatePeriodSeconds(double apoapsisAlt, double periapsisAlt, CelestialBody body)
        {
            double radius = body.Radius;
            double mu = body.gravParameter;

            double apoapsisRadius = radius + apoapsisAlt;
            double periapsisRadius = radius + periapsisAlt;

            double semiMajorAxis = (apoapsisRadius + periapsisRadius) / 2.0;

            // 2 x 3.1415... x Sqrt(semiMajorAxis^3 / mu)
            return 2.0 * Math.PI * Math.Sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / mu);
        }
    }
}
