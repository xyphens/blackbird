using System;
using System.Collections.Generic;
using System.Linq;
using Blackbird.Models;
using Blackbird.Mathematics;

namespace Blackbird.Planning
{
    public static class PhasingRecommendationCalculator
    {
        // Cap the phasing-orbit search: as N rises the orbit tends to the target (ΔV->0, time->inf), so the
        // useful candidates live at low-to-moderate N.
        private const int MaxPhasingOrbits = 50;
        // Phasing circle must clear the atmosphere with margin (and stay inside the SOI).
        private const double SafetyMarginMeters = 5000.0;

        // Builds one exact phasing orbit per integer orbit-count N (the period that closes the phase in
        // exactly N revolutions, inverted to a circular altitude), then picks the best for the mode:
        //   Efficient -> least Hohmann ΔV (smallest offset, most orbits)
        //   Fastest   -> least time (largest offset, fewest orbits)
        //   Balanced  -> the knee, a 50/50 blend of normalized ΔV and time.
        public static PhasingRecommendation Create(
                               CelestialBody body,
                               OrbitInfo targetOrbit,
                               double phaseAngleDeg,
                               PhasingRecommendationMode mode)
        {
            if (body == null || targetOrbit == null)
                return CreateUnavailable(mode, "Missing body or target orbit.");

            double targetAltitude = (targetOrbit.ApoapsisAlt + targetOrbit.PeriapsisAlt) / 2.0;
            double targetPeriod = CalculatePeriodSeconds(body, targetOrbit.ApoapsisAlt, targetOrbit.PeriapsisAlt);

            // Signed angle to bring the target's phase to 0 the short way; its sign sets the drift direction
            // (positive -> lower/faster orbit to catch up, negative -> higher orbit to let the target catch up).
            double signedPhaseToClose = MathHelpers.DeltaDegrees(phaseAngleDeg, 0.0);

            double minSafeAltitude = (body.atmosphere ? body.atmosphereDepth : 0.0) + SafetyMarginMeters;
            double maxAltitude = body.sphereOfInfluence > 0.0 && !double.IsInfinity(body.sphereOfInfluence)
                ? body.sphereOfInfluence - body.Radius - SafetyMarginMeters
                : targetAltitude * 5.0;

            // The phase can be closed in N orbits two ways: the short way (drift signedPhaseToClose) or the
            // long way round (drift the opposite direction by 360 - |signedPhaseToClose|). The short way is a
            // lower orbit to catch up; the long way a higher orbit to let the target lap us. Generate both so
            // a sub-atmosphere catch-up orbit doesn't kill an otherwise-viable higher-orbit plan.
            double otherWayPhase = signedPhaseToClose - 360.0 * Math.Sign(signedPhaseToClose);

            List<PhasingRecommendation> candidates = new List<PhasingRecommendation>();
            for (int n = 1; n <= MaxPhasingOrbits; n++)
            {
                AddCandidate(candidates, body, targetAltitude, targetPeriod,
                    signedPhaseToClose / n, n, minSafeAltitude, maxAltitude, mode);

                if (Math.Sign(signedPhaseToClose) != 0)
                    AddCandidate(candidates, body, targetAltitude, targetPeriod,
                        otherWayPhase / n, n, minSafeAltitude, maxAltitude, mode);
            }

            if (candidates.Count == 0)
                return CreateUnavailable(mode, "No phasing orbit closes the phase within a safe altitude band.");

            return SelectByMode(candidates, mode);
        }

        private static void AddCandidate(
            List<PhasingRecommendation> candidates,
            CelestialBody body,
            double targetAltitude,
            double targetPeriod,
            double gainPerOrbit,
            int orbits,
            double minSafeAltitude,
            double maxAltitude,
            PhasingRecommendationMode mode)
        {
            PhasingRecommendation candidate = EvaluateForOrbitCount(
                body, targetAltitude, targetPeriod, gainPerOrbit, orbits, minSafeAltitude, maxAltitude, mode);
            if (candidate.HasRecommendation) candidates.Add(candidate);
        }

        // The exact circular phasing orbit that closes the phase in N revolutions at the given per-orbit phase
        // gain: invert the phase-gain relation for the required period, convert period -> circular altitude,
        // then cost it (Hohmann ΔV to the target + time). Rejected if the altitude leaves the safe band or the
        // gain is numerically tiny.
        private static PhasingRecommendation EvaluateForOrbitCount(
            CelestialBody body,
            double targetAltitude,
            double targetPeriod,
            double gainPerOrbit,
            int orbits,
            double minSafeAltitude,
            double maxAltitude,
            PhasingRecommendationMode mode)
        {
            if (Math.Abs(gainPerOrbit) < 0.001) return CreateUnavailable(mode, "Candidate phase gain too small.");

            // phaseGainPerOrbit = -360 * (period - targetPeriod) / targetPeriod, solved for period.
            double period = targetPeriod * (1.0 - gainPerOrbit / 360.0);
            if (period <= 0.0) return CreateUnavailable(mode, "Non-physical phasing period.");

            double mu = body.gravParameter;
            double semiMajorAxis = Math.Pow(mu * Math.Pow(period / (2.0 * Math.PI), 2.0), 1.0 / 3.0);
            double altitude = semiMajorAxis - body.Radius;
            if (altitude < minSafeAltitude || altitude > maxAltitude)
                return CreateUnavailable(mode, "Phasing altitude outside the safe band.");

            // Hohmann ΔV between the phasing circle (r1) and the target circle (r2): leave then arrive.
            double r1 = body.Radius + altitude;
            double r2 = body.Radius + targetAltitude;
            double v1 = Math.Sqrt(mu / r1);
            double v2 = Math.Sqrt(mu / r2);
            double transferSma = (r1 + r2) / 2.0;
            double vt1 = Math.Sqrt(mu * (2.0 / r1 - 1.0 / transferSma));
            double vt2 = Math.Sqrt(mu * (2.0 / r2 - 1.0 / transferSma));
            double deltaV = Math.Abs(vt1 - v1) + Math.Abs(v2 - vt2);
            // Add the ascent cost to this phasing circle (energy-based, rises with altitude) so cheaper-to-reach
            // LOWER orbits become competitive and surface, instead of always picking the near-target/higher side.
            deltaV += Math.Sqrt(mu * (2.0 / body.Radius - 1.0 / r1));
            double transferTime = Math.PI * Math.Sqrt(transferSma * transferSma * transferSma / mu);

            double periodDiff = period - targetPeriod;
            double phaseGainPerOrbit = -360.0 * periodDiff / targetPeriod;

            return new PhasingRecommendation
            {
                Mode = mode,
                ApoapsisAlt = altitude,
                PeriapsisAlt = altitude,
                PeriodSeconds = period,
                TargetPeriodSeconds = targetPeriod,
                PeriodDifferenceSeconds = periodDiff,
                PhaseGainDegPerOrbit = phaseGainPerOrbit,
                EstimatedOrbitsToRendezvous = orbits,
                EstimatedTimeToRendezvousSeconds = orbits * period + transferTime,
                DeltaVToTargetMetersPerSecond = deltaV,
                HasRecommendation = true,
                ReasonUnavailable = string.Empty
            };
        }

        private static PhasingRecommendation SelectByMode(
            List<PhasingRecommendation> candidates,
            PhasingRecommendationMode mode)
        {
            switch (mode)
            {
                case PhasingRecommendationMode.Efficient:
                    return candidates.OrderBy(c => c.DeltaVToTargetMetersPerSecond).First();

                case PhasingRecommendationMode.Fastest:
                    return candidates.OrderBy(c => c.EstimatedTimeToRendezvousSeconds).First();

                default: // Balanced: minimize a 50/50 blend of normalized ΔV and time (scale-free).
                    double dvMin = candidates.Min(c => c.DeltaVToTargetMetersPerSecond);
                    double dvMax = candidates.Max(c => c.DeltaVToTargetMetersPerSecond);
                    double tMin = candidates.Min(c => c.EstimatedTimeToRendezvousSeconds);
                    double tMax = candidates.Max(c => c.EstimatedTimeToRendezvousSeconds);
                    return candidates
                        .OrderBy(c => 0.5 * Normalize(c.DeltaVToTargetMetersPerSecond, dvMin, dvMax)
                                    + 0.5 * Normalize(c.EstimatedTimeToRendezvousSeconds, tMin, tMax))
                        .First();
            }
        }

        private static double Normalize(double value, double min, double max)
        {
            return max > min ? (value - min) / (max - min) : 0.0;
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
