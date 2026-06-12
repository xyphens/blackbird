using System;
using Blackbird.Mathematics;
using Blackbird.Models;
using KSP.UI.Screens.Flight;

namespace Blackbird.Planning
{
    public static class AscentPredictor
    {
        // used for rocket equation
        private const double StandardGravity = 9.80665;

        public static AscentPrediction Create(
            Vessel active,
            Vessel target,
            LaunchPlan plan,
            double desiredPitchDeg)
        {
            if (active == null || plan == null) return null;

            double estTimeToInsertion = 
        }

        private static double EstimateRemainingDv(Vessel vessel)
        {
            if (vessel == null) return double.NaN;

            try
            {
                double dV = vessel.GetDeltaV();

                if (!double.IsNaN(dV) && dV >= 0.0) return dV;
            } catch
            {
                // predictor is non-fatal

            }

            return double.NaN;
        }

        private static double EstimateTimeToInsertionSeconds(Vessel vessel, LaunchPlan plan) {
            if (vessel == null || plan == null) return double.NaN;

            // note: targetAltitude is our plan's target altitude, not the altitude of our target

            double targetAltitude = plan.RecommendedApAlt > 0.0 ? plan.RecommendedApAlt : plan.TargetOrbit.ApoapsisAlt;
            double altitudeRemaining = Math.Max(0.0, targetAltitude - vessel.altitude);
            double verticalSpeed = Math.Max(100.0, Math.Abs(vessel.verticalSpeed));
            double altitudeTime = altitudeRemaining / verticalSpeed;

            return altitudeTime;
        }
    }
}
