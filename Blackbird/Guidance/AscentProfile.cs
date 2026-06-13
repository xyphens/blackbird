using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using Blackbird.Models;

namespace Blackbird.Guidance
{
    public sealed class AscentProfile
    {
        public bool IsValid { get; set; }
        public string ReasonUnavailable { get; set; }

        public double TargetApoapsisAlt { get; set; }
        public double TargetPeriapsisAlt { get; set; }

        public double RecommendedHeadingDeg { get; set; }

        public double PredictedApoapsisAlt { get; set; }
        public double PredictedPeriapsisAlt { get; set; }

        public double EstimatedInsertionUt { get; set; }
        public double EstimatedTimeToInsertionSeconds { get; set; }
        public double EstimatedRemainingDeltaV { get; set; }

        public AscentProfilePoint[] Points { get; set; }

        // Returns the commanded pitch for the current altitude using linear interpolation.
        public double GetPitchAtAltitude(double altitudeMeters)
        {
            return InterpolateAtAltitude(altitudeMeters, point => point.PitchDeg);
        }

        // Returns the commanded heading for the current altitude using linear interpolation.
        public double GetHeadingAtAltitude(double altitudeMeters)
        {
            return InterpolateAtAltitude(altitudeMeters, point => point.HeadingDeg);
        }

        // Returns the normalized commanded throttle for the current altitude.
        public double GetThrottleAtAltitude(double altitudeMeters)
        {
            return InterpolateAtAltitude(altitudeMeters, point => point.Throttle);
        }

        // Interpolates a profile value over altitude and clamps outside the known point range.
        private double InterpolateAtAltitude(double altitudeMeters, Func<AscentProfilePoint, double> selector)
        {
            if (Points == null || Points.Length == 0) return double.NaN;
            if (Points.Length == 1) return selector(Points[0]);

            if (altitudeMeters <= Points[0].AltitudeMeters) return selector(Points[0]);

            for (int i = 0; i < Points.Length - 1; i++)
            {
                AscentProfilePoint a = Points[i];
                AscentProfilePoint b = Points[i + 1];

                if (altitudeMeters > b.AltitudeMeters) continue;

                double span = b.AltitudeMeters - a.AltitudeMeters;
                if (Math.Abs(span) < 1e-9) return selector(b);

                double t = (altitudeMeters - a.AltitudeMeters) / span;
                return selector(a) + (selector(b) - selector(a)) * t;
            }

            return selector(Points[Points.Length - 1]);
        }
    }

    public sealed class AscentProfilePoint
    {
        public double AltitudeMeters { get; set; }
        public double PitchDeg { get; set; }
        public double HeadingDeg { get; set; }
        public double Throttle { get; set; }
    }

    public static class AscentProfileSolver
    {
        // Builds the bootstrap/fallback altitude curve. Nominal MechJeb-parity PSG should take over from
        // solver state early in ascent rather than treating this curve as the final guidance law.
        public static AscentProfile Create(
            VesselState vesselState,
            double targetApoapsisAlt,
            double targetPeriapsisAlt,
            double recommendedHeadingDeg,
            double estimatedRemainingDeltaV)
        {
            if (vesselState == null)
            {
                return CreateInvalid("Vessel state is unavailable.");
            }

            if (!OrbitMath.IsFinite(targetApoapsisAlt) ||
                !OrbitMath.IsFinite(targetPeriapsisAlt) ||
                targetApoapsisAlt <= 0.0 ||
                targetPeriapsisAlt <= 0.0)
            {
                return CreateInvalid("Insertion target altitude is unavailable.");
            }

            if (!OrbitMath.IsFinite(recommendedHeadingDeg))
            {
                return CreateInvalid("Launch heading is unavailable.");
            }

            AscentProfilePoint[] points = CreateProfilePoints(
                vesselState,
                targetApoapsisAlt,
                targetPeriapsisAlt,
                recommendedHeadingDeg);

            double estimatedTimeToInsertionSeconds = EstimateTimeToInsertionSeconds(vesselState, points);

            return new AscentProfile
            {
                IsValid = true,
                ReasonUnavailable = null,

                TargetApoapsisAlt = targetApoapsisAlt,
                TargetPeriapsisAlt = targetPeriapsisAlt,
                RecommendedHeadingDeg = recommendedHeadingDeg,

                PredictedApoapsisAlt = targetApoapsisAlt,
                PredictedPeriapsisAlt = targetPeriapsisAlt,
                EstimatedInsertionUt = vesselState.UniversalTime + estimatedTimeToInsertionSeconds,
                EstimatedTimeToInsertionSeconds = estimatedTimeToInsertionSeconds,
                EstimatedRemainingDeltaV = estimatedRemainingDeltaV,

                Points = points
            };
        }

        // Creates an explicit invalid profile so callers can carry a reason through planning and UI.
        public static AscentProfile CreateInvalid(string reasonUnavailable)
        {
            return new AscentProfile
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reasonUnavailable)
                    ? "Ascent profile is unavailable."
                    : reasonUnavailable,
                Points = new AscentProfilePoint[0],
                TargetApoapsisAlt = double.NaN,
                TargetPeriapsisAlt = double.NaN,
                RecommendedHeadingDeg = double.NaN,
                PredictedApoapsisAlt = double.NaN,
                PredictedPeriapsisAlt = double.NaN,
                EstimatedInsertionUt = double.NaN,
                EstimatedTimeToInsertionSeconds = double.NaN,
                EstimatedRemainingDeltaV = double.NaN
            };
        }

        // Creates a smooth altitude-indexed gravity turn for pre-PSG bootstrap commands.
        private static AscentProfilePoint[] CreateProfilePoints(
            VesselState vesselState,
            double targetApoapsisAlt,
            double targetPeriapsisAlt,
            double headingDeg)
        {
            double targetInsertionAlt = Math.Max(100.0, Math.Min(targetApoapsisAlt, targetPeriapsisAlt));
            double atmosphereTop = GetAtmosphereTop(vesselState);
            double turnStartAlt = GetTurnStartAltitude(vesselState, atmosphereTop, targetInsertionAlt);
            double turnEndAlt = GetHorizontalFlightAltitude(atmosphereTop, targetInsertionAlt);
            turnEndAlt = Math.Max(turnStartAlt + 1000.0, turnEndAlt);
            double exitPitchDeg = GetAtmosphereExitPitchDeg(atmosphereTop, turnEndAlt);
            double shapeExponent = SolveTurnShapeExponent(turnStartAlt, turnEndAlt, atmosphereTop, exitPitchDeg);

            double[] controlAltitudes = CreateControlAltitudes(turnStartAlt, turnEndAlt, atmosphereTop);
            var points = new List<AscentProfilePoint>();

            double initialPitch = GetSolvedPitchAtAltitude(vesselState.AltitudeMeters, turnStartAlt, turnEndAlt, shapeExponent);
            double initialThrottle = GetSolvedThrottleAtAltitude(vesselState.AltitudeMeters, turnStartAlt, turnEndAlt);
            AddPoint(points, vesselState.AltitudeMeters, initialPitch, headingDeg, initialThrottle);

            for (int i = 0; i < controlAltitudes.Length; i++)
            {
                double altitude = controlAltitudes[i];
                double pitch = GetSolvedPitchAtAltitude(altitude, turnStartAlt, turnEndAlt, shapeExponent);
                double throttle = GetSolvedThrottleAtAltitude(altitude, turnStartAlt, turnEndAlt);
                AddPoint(points, altitude, pitch, headingDeg, throttle);
            }

            AddPoint(points, turnEndAlt, 0.0, headingDeg, 0.0);

            return points.ToArray();
        }

        // Estimates insertion time from body scale and solved profile altitude span.
        private static double EstimateTimeToInsertionSeconds(VesselState vesselState, AscentProfilePoint[] points)
        {
            if (points == null || points.Length == 0) return double.NaN;

            double finalAltitude = points[points.Length - 1].AltitudeMeters;
            double circularVelocity = OrbitMath.GetCircularVelocity(vesselState.Body, finalAltitude);
            double nominalVerticalRate = OrbitMath.IsFinite(circularVelocity)
                ? OrbitMath.Clamp(circularVelocity / 25.0, 120.0, 450.0)
                : 180.0;

            return Math.Max(60.0, (finalAltitude - vesselState.AltitudeMeters) / nominalVerticalRate);
        }

        // Reads the atmospheric boundary used to decide where the profile should be nearly horizontal.
        private static double GetAtmosphereTop(VesselState vesselState)
        {
            if (vesselState == null || vesselState.Body == null) return 0.0;
            if (!vesselState.Body.atmosphere) return 0.0;

            return Math.Max(0.0, vesselState.Body.atmosphereDepth);
        }

        // Chooses where the bootstrap gravity turn begins before PSG owns the thrust vector.
        private static double GetTurnStartAltitude(VesselState vesselState, double atmosphereTop, double insertionAlt)
        {
            double pressureStart = atmosphereTop > 0.0 ? atmosphereTop * 0.015 : insertionAlt * 0.05;
            double radiusScaledStart = vesselState.BodyRadius * 0.0008;
            double start = Math.Max(250.0, Math.Min(pressureStart, radiusScaledStart));

            return OrbitMath.Clamp(start, vesselState.AltitudeMeters, insertionAlt * 0.25);
        }

        // Computes the desired pitch near atmosphere exit from target altitude and atmospheric depth.
        private static double GetAtmosphereExitPitchDeg(double atmosphereTop, double turnEndAlt)
        {
            if (atmosphereTop <= 0.0) return 0.0;
            if (turnEndAlt <= atmosphereTop) return 0.0;

            double coastFraction = (turnEndAlt - atmosphereTop) / turnEndAlt;
            return OrbitMath.Clamp(5.0 + coastFraction * 15.0, 2.0, 20.0);
        }

        // Chooses where the bootstrap profile becomes horizontal instead of waiting for apoapsis.
        private static double GetHorizontalFlightAltitude(double atmosphereTop, double insertionAlt)
        {
            if (atmosphereTop > 0.0)
            {
                double atmosphereTarget = atmosphereTop * 0.85;
                double insertionTarget = insertionAlt * 0.40;
                return OrbitMath.Clamp(
                    Math.Max(atmosphereTarget, insertionTarget),
                    atmosphereTop * 0.45,
                    Math.Min(insertionAlt, atmosphereTop));
            }

            return insertionAlt * 0.50;
        }

        // Solves the curve exponent so pitch at atmosphere edge matches the desired exit pitch.
        private static double SolveTurnShapeExponent(
            double turnStartAlt,
            double turnEndAlt,
            double atmosphereTop,
            double exitPitchDeg)
        {
            double referenceAlt = atmosphereTop > turnStartAlt && atmosphereTop < turnEndAlt
                ? atmosphereTop
                : turnEndAlt;

            double progress = OrbitMath.Clamp(
                (referenceAlt - turnStartAlt) / (turnEndAlt - turnStartAlt),
                0.001,
                0.999);

            double normalizedPitch = OrbitMath.Clamp(exitPitchDeg / 90.0, 0.001, 0.999);
            double exponent = Math.Log(1.0 - normalizedPitch) / Math.Log(progress);

            return OrbitMath.Clamp(exponent, 0.35, 2.5);
        }

        // Creates altitude samples for stable interpolation before PSG guidance is available.
        private static double[] CreateControlAltitudes(double turnStartAlt, double turnEndAlt, double atmosphereTop)
        {
            // These fractions only shape the bootstrap curve; PSG should command from optimized trajectory state.
            var altitudes = new List<double>
            {
                turnStartAlt,
                Lerp(turnStartAlt, turnEndAlt, 0.18),
                Lerp(turnStartAlt, turnEndAlt, 0.35),
                Lerp(turnStartAlt, turnEndAlt, 0.52),
                Lerp(turnStartAlt, turnEndAlt, 0.70),
                Lerp(turnStartAlt, turnEndAlt, 0.86)
            };

            if (atmosphereTop > turnStartAlt && atmosphereTop < turnEndAlt)
            {
                altitudes.Add(atmosphereTop);
            }

            altitudes.Sort();
            return altitudes.ToArray();
        }

        // Evaluates the solved gravity-turn curve at one altitude.
        private static double GetSolvedPitchAtAltitude(
            double altitude,
            double turnStartAlt,
            double turnEndAlt,
            double shapeExponent)
        {
            if (altitude <= turnStartAlt) return 90.0;
            if (altitude >= turnEndAlt) return 0.0;

            double progress = (altitude - turnStartAlt) / (turnEndAlt - turnStartAlt);
            return OrbitMath.Clamp(90.0 * (1.0 - Math.Pow(progress, shapeExponent)), 0.0, 90.0);
        }

        // Keeps engines burning until the profile reaches its explicit cutoff altitude.
        private static double GetSolvedThrottleAtAltitude(double altitude, double turnStartAlt, double turnEndAlt)
        {
            return altitude >= turnEndAlt ? 0.0 : 1.0;
        }

        private static double Lerp(double a, double b, double t)
        {
            return a + (b - a) * t;
        }

        // Adds one point while preserving strictly increasing altitude order.
        private static void AddPoint(
            List<AscentProfilePoint> points,
            double altitudeMeters,
            double pitchDeg,
            double headingDeg,
            double throttle)
        {
            if (!OrbitMath.IsFinite(altitudeMeters) || !OrbitMath.IsFinite(pitchDeg)) return;
            if (points.Count > 0 && altitudeMeters <= points[points.Count - 1].AltitudeMeters + 1.0) return;

            points.Add(CreatePoint(altitudeMeters, pitchDeg, headingDeg, throttle));
        }

        // Creates one altitude command point with a shared launch heading.
        private static AscentProfilePoint CreatePoint(
            double altitudeMeters,
            double pitchDeg,
            double headingDeg,
            double throttle)
        {
            return new AscentProfilePoint
            {
                AltitudeMeters = altitudeMeters,
                PitchDeg = pitchDeg,
                HeadingDeg = headingDeg,
                Throttle = OrbitMath.Clamp(throttle, 0.0, 1.0)
            };
        }
    }
}
