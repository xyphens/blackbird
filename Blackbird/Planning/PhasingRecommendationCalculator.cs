using System;
using Blackbird.Models;
using Blackbird.Enums;
using Blackbird.Mathematics;

namespace Blackbird.Planning
{
    public static class PhasingRecommendationCalculator
    {
        public static PhasingRecommendation Create(
                            CelestialBody body, 
                            OrbitInfo targetOrbit, 
                            double phaseAngleDeg, 
                            PhasingRecommendationMode mode)
        {
            switch (mode)
            {
                case PhasingRecommendationMode.Fastest:
                    return CreateUnavailable(mode, "Fast recommendation not implemented");
                case PhasingRecommendationMode.Efficient:
                    return CreateUnavailable(mode, "Efficient recommendation not implemented");
                case PhasingRecommendationMode.Balanced:
                default:
                    return CreateBalanced(body, targetOrbit, phaseAngleDeg, mode);
            }
        }

        private static PhasingRecommendation CreateBalanced(
                               CelestialBody body,
                               OrbitInfo targetOrbit,
                               double phaseAngleDeg,
                               PhasingRecommendationMode mode)
        {
            // average of ap and pe
            double targetAltitude = (targetOrbit.ApoapsisAlt + targetOrbit.PeriapsisAlt) / 2.0;
            double bestScore = double.MaxValue;
            PhasingRecommendation best = null;

            PlanetScale.PlanetScaleEnum scale = PlanetScale.GetScale();

            double initOffset = scale == PlanetScale.PlanetScaleEnum.RSS ? 150000.0 : 70000.0;
            double offsetScalar = scale == PlanetScale.PlanetScaleEnum.RSS ? 5000.0 : 1500.0;
            double candidateThreshold = scale == PlanetScale.PlanetScaleEnum.RSS ? 145000.0 : 70000.0;

            for (double offset = -initOffset; offset <= initOffset; offset += offsetScalar)
            {
                if (Math.Abs(offset) < 1.0) continue;

                double candidateAltitude = targetAltitude + offset;

                if (candidateAltitude < candidateThreshold) continue;

                PhasingRecommendation candidate = EvaluateCircularCandidate(
                                                        body, 
                                                        targetOrbit, 
                                                        phaseAngleDeg, 
                                                        candidateAltitude, 
                                                        mode);

                if (!candidate.HasRecommendation) continue;

                double altitudePenalty = Math.Abs(offset) / 1000.0;
                // penalize number of orbits to rendezvous
                double orbitPenalty = candidate.EstimatedOrbitsToRendezvous * 10.0;
                double score = altitudePenalty + orbitPenalty;

                // winner is one with lowest with closest altitude to target
                if (score < bestScore)
                {
                    bestScore = score;
                    best = candidate;
                }
            }

            return best ?? CreateUnavailable(mode, "No usable balanced phasing orbit found");
        }

        private static PhasingRecommendation EvaluateCircularCandidate(
                        CelestialBody body,
                        OrbitInfo targetOrbit,
                        double phaseAngleDeg,
                        double altitude,
                        PhasingRecommendationMode mode)
        {
            double period = CalculatePeriodSeconds(body, altitude, altitude);
            double targetPeriod = CalculatePeriodSeconds(body, targetOrbit.ApoapsisAlt, targetOrbit.PeriapsisAlt);

            double periodDiff = period - targetPeriod;
            double phaseGainPerOrbit = -360.0 * periodDiff / targetPeriod;

            if (Math.Abs(phaseGainPerOrbit) < 0.001) return CreateUnavailable(mode, "Candidate phase gain too small.");

            bool targetAhead = phaseAngleDeg > 180.0;
            bool candidateCatchesUp = phaseGainPerOrbit > 0.0;
            bool candidateLetsTargetCatchUp = phaseGainPerOrbit < 0.0;

            if (targetAhead && !candidateCatchesUp) return CreateUnavailable(mode, "Candidate does not catch target");
            if (!targetAhead && !candidateLetsTargetCatchUp) return CreateUnavailable(mode, "Candidate does not let target catch up");
            if (Math.Abs(phaseGainPerOrbit) < 0.001) return CreateUnavailable(mode, "Candidate phase gain too small.");

            double phaseToClose = Math.Abs(OrbitMath.DeltaDegrees(phaseAngleDeg, 0.0));

            double orbitsToRendezvous = phaseToClose / Math.Abs(phaseGainPerOrbit);

            double timeToRendezvous = orbitsToRendezvous * period;

            return new PhasingRecommendation
            {
                Mode = mode,
                ApoapsisAlt = altitude,
                PeriapsisAlt = altitude,
                PeriodSeconds = period,
                TargetPeriodSeconds = targetPeriod,
                PeriodDifferenceSeconds = periodDiff,
                PhaseGainDegPerOrbit = phaseGainPerOrbit,
                EstimatedOrbitsToRendezvous = orbitsToRendezvous,
                EstimatedTimeToRendezvousSeconds = timeToRendezvous,
                HasRecommendation = true,
                ReasonUnavailable = string.Empty
            };
        }

        private static double CalculatePeriodSeconds(
                                CelestialBody body,
                                double apAlt,
                                double peAlt)
        {
            double apRadius = body.Radius + apAlt;
            double peRadius = body.Radius + peAlt;

            double semiMajorAxis = (apRadius + peRadius) / 2.0;

            return 2.0 *
                Math.PI *
                Math.Sqrt(
                    Math.Pow(semiMajorAxis, 3.0) /
                    body.gravParameter);

        }

        private static PhasingRecommendation CreateUnavailable(
            PhasingRecommendationMode mode,
            string reason)
        {
            return new PhasingRecommendation
            {
                Mode = mode,
                HasRecommendation = false,
                ReasonUnavailable = reason
            };
        }
    }
}
