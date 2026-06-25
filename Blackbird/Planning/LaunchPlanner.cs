using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Trajectory;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;

namespace Blackbird.Planning
{
    public static class LaunchPlanner
    {
        public static LaunchPlan Create(Vessel active, Vessel target, InsertionTarget insertionTarget, LaunchLocation launchSite)
        {
            // must be an active vessel, have a target, target not be myself
            if (active == null || target == null || ReferenceEquals(active, target) || active.id == target.id) return null;

            VesselState vesselState = VesselState.FromVessel(active);
            LaunchLocation ls = launchSite ?? LaunchLocation.FromVessel(active);
            InsertionTarget targetInsertion = insertionTarget ?? InsertionTarget.FromTargetOrbit(target);

            OrbitInfo activeOrbit = TrajectoryProvider.GetOrbitInfo(active);
            OrbitInfo targetOrbit = TrajectoryProvider.GetOrbitInfo(target);
            double phaseAngleDeg = OrbitMath.GetPhaseAngleDeg(active, target);

            PhasingOrbit po = PhasingOrbit.FromInsertionTarget(targetInsertion, targetOrbit, target.mainBody, phaseAngleDeg);
            LaunchWindowInfo lwi = LaunchWindowInfo.Create(active, targetOrbit, ls);
            // Use LaunchWindowInfo's inclination-based azimuth; the plane-derived override launched retrograde at one node.
            LaunchCandidate[] candidates = CreateCandidates(vesselState, target, targetOrbit, targetInsertion, lwi, phaseAngleDeg);

            return new LaunchPlan
            {
                TargetVessel = target,
                ActiveOrbit = activeOrbit,
                TargetOrbit = targetOrbit,
                TargetOrbitNormal = TrajectoryProvider.GetOrbitNormal(target),
                LaunchWindow = lwi,
                PhaseAngleDeg = phaseAngleDeg,
                DistanceMeters = Vector3d.Distance(TrajectoryProvider.GetPosition(active), TrajectoryProvider.GetPosition(target)),
                LaunchAzimuthDeg = lwi.SelectedAzimuthDeg,
                RecommendedApAlt = targetInsertion.ApoapsisAlt,
                RecommendedPeAlt = targetInsertion.PeriapsisAlt,
                RelativeInclinationDeg = targetOrbit.InclinationDeg - activeOrbit.InclinationDeg,
                RelativeLanDeg = MathHelpers.DeltaDegrees(activeOrbit.LanDeg, targetOrbit.LanDeg),
                RelativePeriodSeconds = targetOrbit.PeriodSeconds - activeOrbit.PeriodSeconds,
                InsertionTarget = targetInsertion,
                PhasingOrbit = po,
                PhasingRecommendation = candidates.Length > 0 ? candidates[0].PhasingRecommendation : null,
                Candidates = candidates,
                SelectedCandidateIndex = -1
            };
        }


        // Builds selectable launch opportunities from the current plane-crossing windows.
        private static LaunchCandidate[] CreateCandidates(
            VesselState vesselState,
            Vessel target,
            OrbitInfo targetOrbit,
            InsertionTarget insertionTarget,
            LaunchWindowInfo launchWindow,
            double currentPhaseAngleDeg)
        {
            if (vesselState == null || target == null || targetOrbit == null || insertionTarget == null || launchWindow == null)
            {
                return new[] { CreateInvalidCandidate("Planner inputs are incomplete.") };
            }

            List<LaunchCandidate> candidates = new List<LaunchCandidate>();

            foreach (PhasingRecommendationMode mode in Enum.GetValues(typeof(PhasingRecommendationMode)))
            {
                candidates.Add(CreateCandidateForWindow(
                    vesselState, target, targetOrbit, insertionTarget,
                    launchWindow.TimeToAscendingNodeSeconds,
                    launchWindow.AscendingAzimuthDeg,
                    launchWindow.PlaneErrorDeg,
                    currentPhaseAngleDeg,
                    "Ascending launch window is unavailable.",
                    mode));
            }

            foreach (PhasingRecommendationMode mode in Enum.GetValues(typeof(PhasingRecommendationMode)))
            {
                candidates.Add(CreateCandidateForWindow(
                    vesselState, target, targetOrbit, insertionTarget,
                    launchWindow.TimeToDescendingNodeSeconds,
                    launchWindow.DescendingAzimuthDeg,
                    launchWindow.PlaneErrorDeg,
                    currentPhaseAngleDeg,
                    "Descending launch window is unavailable.",
                    mode));
            }

            return candidates
                .OrderBy(candidate => candidate.IsValid ? 0 : 1)
                .ThenBy(candidate => candidate.EstimatedDeltaVUsed)
                .ThenBy(candidate => Math.Abs(candidate.PhaseErrorDeg))
                .ThenBy(candidate => candidate.RelativeDistanceMeters)
                .ThenBy(candidate => candidate.EstimatedOrbitsToRendezvous)
                .ThenBy(candidate => candidate.LaunchUt)
                .ToArray();
        }

        // Creates one candidate by combining a launch window, phasing orbit, and ascent profile.
        private static LaunchCandidate CreateCandidateForWindow(
            VesselState vesselState,
            Vessel target,
            OrbitInfo targetOrbit,
            InsertionTarget insertionTarget,
            double secondsUntilLaunch,
            double launchHeadingDeg,
            double planeErrorDeg,
            double currentPhaseAngleDeg,
            string unavailableReason,
            PhasingRecommendationMode mode
            )
        {
            if (!IsUsableWindow(secondsUntilLaunch, launchHeadingDeg))
            {
                return CreateInvalidCandidate(unavailableReason);
            }

            double launchUt = vesselState.UniversalTime + secondsUntilLaunch;
            double phaseAngleAtLaunch = EstimatePhaseAngleAtLaunch(currentPhaseAngleDeg, targetOrbit, secondsUntilLaunch);

            PhasingRecommendation phasingRecommendation = PhasingRecommendationCalculator.Create(
                vesselState.Body,
                targetOrbit,
                phaseAngleAtLaunch,
                mode);

            double insertionApoapsisAlt = phasingRecommendation.HasRecommendation
                ? phasingRecommendation.ApoapsisAlt
                : insertionTarget.ApoapsisAlt;

            double insertionPeriapsisAlt = phasingRecommendation.HasRecommendation
                ? phasingRecommendation.PeriapsisAlt
                : insertionTarget.PeriapsisAlt;

            double estimatedDeltaVUsed = EstimateLaunchDeltaVUsed(vesselState.Body, insertionApoapsisAlt, insertionPeriapsisAlt, targetOrbit);
            double estimatedRemainingDeltaV = vesselState.RemainingDeltaV - estimatedDeltaVUsed;

            AscentProfile ascentProfile = AscentProfileSolver.Create(
                vesselState,
                insertionApoapsisAlt,
                insertionPeriapsisAlt,
                launchHeadingDeg,
                estimatedRemainingDeltaV);

            Vector3d targetPositionAtLaunch = TrajectoryProvider.GetPositionAtUt(target, launchUt);
            double relativeDistanceMeters = Vector3d.Distance(vesselState.Position, targetPositionAtLaunch);
            double estimatedOrbits = phasingRecommendation.HasRecommendation
                ? phasingRecommendation.EstimatedOrbitsToRendezvous
                : double.PositiveInfinity;

            double phaseErrorDeg = phasingRecommendation.HasRecommendation
                ? 0.0
                : MathHelpers.DeltaDegrees(phaseAngleAtLaunch, 0.0);

            bool isValid =
                phasingRecommendation.HasRecommendation &&
                ascentProfile.IsValid &&
                MathHelpers.IsFinite(estimatedDeltaVUsed);

            double score = ScoreCandidate(
                isValid,
                estimatedDeltaVUsed,
                phaseErrorDeg,
                planeErrorDeg,
                estimatedOrbits,
                secondsUntilLaunch);

            return new LaunchCandidate
            {
                IsValid = isValid,
                ReasonUnavailable = isValid ? string.Empty : phasingRecommendation.ReasonUnavailable,

                LaunchUt = launchUt,
                SecondsUntilLaunch = secondsUntilLaunch,

                InsertionApoapsisAlt = insertionApoapsisAlt,
                InsertionPeriapsisAlt = insertionPeriapsisAlt,
                LaunchHeadingDeg = launchHeadingDeg,

                EstimatedInsertionTimeSeconds = ascentProfile.EstimatedTimeToInsertionSeconds,
                EstimatedOrbitsToRendezvous = estimatedOrbits,

                EstimatedDeltaVUsed = estimatedDeltaVUsed,
                EstimatedRemainingDeltaV = estimatedRemainingDeltaV,

                PlaneErrorDeg = planeErrorDeg,
                PhaseErrorDeg = phaseErrorDeg,
                RelativeDistanceMeters = relativeDistanceMeters,
                Score = score,

                AscentProfile = ascentProfile,
                PhasingRecommendation = phasingRecommendation
            };
        }

        // Returns false when a node cannot produce a usable launch heading or future launch time.
        private static bool IsUsableWindow(double secondsUntilLaunch, double launchHeadingDeg)
        {
            return MathHelpers.IsFinite(secondsUntilLaunch) && MathHelpers.IsFinite(launchHeadingDeg) && secondsUntilLaunch >= 0.0;
        }

        // Advances the current phase estimate by target mean motion during the wait to launch.
        private static double EstimatePhaseAngleAtLaunch(double currentPhaseAngleDeg, OrbitInfo targetOrbit, double secondsUntilLaunch)
        {
            if (targetOrbit == null || targetOrbit.PeriodSeconds <= 0.0) return currentPhaseAngleDeg;

            double targetMotionDeg = secondsUntilLaunch / targetOrbit.PeriodSeconds * 360.0;
            return MathHelpers.NormalizeDegrees(currentPhaseAngleDeg + targetMotionDeg);
        }

        // Estimates ascent plus phasing insertion dV from circular velocity and transfer cost.
        private static double EstimateLaunchDeltaVUsed(
                                CelestialBody body,
                                double insertionApoapsisAlt,
                                double insertionPeriapsisAlt,
                                OrbitInfo targetOrbit)
        {
            double insertionAltitude = (insertionApoapsisAlt + insertionPeriapsisAlt) * 0.5;

            // Energy-based characteristic velocity to reach the insertion orbit. Orbital speed FALLS with
            // altitude, so the old circular-speed term made higher phasing orbits look cheaper; this rises with
            // altitude, so the higher detour is correctly costed.
            double rInsertion = body.Radius + insertionAltitude;
            double ascentDeltaV = Math.Sqrt(body.gravParameter * (2.0 / body.Radius - 1.0 / rInsertion));
            if (!MathHelpers.IsFinite(ascentDeltaV)) return double.NaN;

            double targetAltitude = targetOrbit != null
                ? (targetOrbit.ApoapsisAlt + targetOrbit.PeriapsisAlt) * 0.5
                : insertionAltitude;

            double transferDeltaV = OrbitMath.EstimateHohmannDeltaV(body, insertionAltitude, targetAltitude);
            if (!MathHelpers.IsFinite(transferDeltaV)) transferDeltaV = 0.0;

            return ascentDeltaV + transferDeltaV;
        }

        // Scores candidates so lower dV, lower error, fewer phasing orbits, and earlier launches sort first.
        private static double ScoreCandidate(
            bool isValid,
            double estimatedDeltaVUsed,
            double phaseErrorDeg,
            double planeErrorDeg,
            double estimatedOrbitsToRendezvous,
            double secondsUntilLaunch)
        {
            if (!isValid) return double.PositiveInfinity;

            return estimatedDeltaVUsed +
                   Math.Abs(phaseErrorDeg) * 100.0 +
                   Math.Abs(planeErrorDeg) * 50.0 +
                   estimatedOrbitsToRendezvous * 10.0 +
                   secondsUntilLaunch / 1000.0;
        }

        // Creates an invalid candidate that can still be surfaced by UI.
        private static LaunchCandidate CreateInvalidCandidate(string reasonUnavailable)
        {
            return new LaunchCandidate
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reasonUnavailable)
                    ? "Launch candidate is unavailable."
                    : reasonUnavailable,
                LaunchUt = double.NaN,
                SecondsUntilLaunch = double.NaN,
                InsertionApoapsisAlt = double.NaN,
                InsertionPeriapsisAlt = double.NaN,
                LaunchHeadingDeg = double.NaN,
                EstimatedInsertionTimeSeconds = double.NaN,
                EstimatedOrbitsToRendezvous = double.PositiveInfinity,
                EstimatedDeltaVUsed = double.PositiveInfinity,
                EstimatedRemainingDeltaV = double.NaN,
                PlaneErrorDeg = double.NaN,
                PhaseErrorDeg = double.NaN,
                RelativeDistanceMeters = double.NaN,
                Score = double.PositiveInfinity,
                AscentProfile = AscentProfileSolver.CreateInvalid(reasonUnavailable),
                PhasingRecommendation = null
            };
        }
    }
}
