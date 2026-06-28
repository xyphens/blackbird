using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using Blackbird.Trajectory;
using System.Collections.Generic;
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
            LaunchCandidate[] candidates = CreateCandidates(active, vesselState, target, targetOrbit, ls);

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

        // Adapter: feed the live vessel/target/body into the pure LaunchWindowSolver and map its best ascending
        // and best descending option back to the UI's LaunchCandidate. AscentProfile is used both ways — a
        // nominal profile gives the solver its launch->insertion duration, and each result gets its own profile.
        private static LaunchCandidate[] CreateCandidates(
            Vessel active, VesselState vesselState, Vessel target, OrbitInfo targetOrbit, LaunchLocation launchLocation)
        {
            if (active == null || vesselState == null || target == null || targetOrbit == null || launchLocation == null)
                return new[] { CreateInvalidCandidate("Planner inputs are incomplete.") };

            CelestialBody body = active.mainBody;
            BodyOblateness.Oblateness ob = BodyOblateness.For(body);
            double targetAlt = (targetOrbit.ApoapsisAlt + targetOrbit.PeriapsisAlt) * 0.5;

            // Physical prograde normal: cross(r,v) sign is unreliable in KSP's left-handed frame, so flip it to
            // match KSP's inclination (prograde < 90 deg -> normal on the +pole side). Without this the solver
            // reads prograde orbits as retrograde and recommends a westward launch.
            Vector3d pole = ((Vector3d)body.transform.up).normalized;
            Vector3d targetRelPos = TrajectoryProvider.GetPosition(target) - body.position;
            Vector3d targetVel = TrajectoryProvider.GetVelocity(target);
            Vector3d targetNormal = Vector3d.Cross(targetRelPos, targetVel).normalized;
            if ((targetOrbit.InclinationDeg < 90.0) != (Vector3d.Dot(targetNormal, pole) > 0.0))
                targetNormal = -targetNormal;

            // Actual rotation sense for carrying the pad forward (KSP's own angular-velocity vector); falls back to
            // the geometric pole. NOT the same as Pole: +angle about transform.up sweeps the pad retrograde under
            // KSP's left-handed cross, putting the launch ~half a turn off the real plane crossing on inclined targets.
            Vector3d bodyAngularVelocity = body.angularVelocity;
            Vector3d rotationAxis = bodyAngularVelocity.sqrMagnitude > 0.0 ? bodyAngularVelocity.normalized : pole;

            // Nominal ascent at the target altitude, only to get the launch->insertion duration the solver needs.
            double nominalAzimuth = OrbitMath.GetLaunchAzimuth(targetOrbit.InclinationDeg, launchLocation.LatitudeDeg);
            AscentProfile nominalAscent = AscentProfileSolver.Create(
                vesselState, targetAlt, targetAlt, nominalAzimuth, vesselState.RemainingDeltaV);
            double ascentDuration = nominalAscent.IsValid && MathHelpers.IsFinite(nominalAscent.EstimatedTimeToInsertionSeconds)
                ? nominalAscent.EstimatedTimeToInsertionSeconds : 300.0;

            LaunchWindowSolver.Inputs inputs = new LaunchWindowSolver.Inputs
            {
                Mu = body.gravParameter,
                BodyRadius = body.Radius,
                AtmosphereDepth = body.atmosphere ? body.atmosphereDepth : 0.0,
                RotationPeriodSeconds = body.rotationPeriod,
                J2 = ob.J2,
                J2ReferenceRadius = ob.ReferenceRadiusMeters,
                Pole = pole,
                RotationAxis = rotationAxis,
                CurrentUt = vesselState.UniversalTime,
                LaunchSitePosition = TrajectoryProvider.GetPosition(active) - body.position,
                TargetPosition = targetRelPos,
                TargetVelocity = targetVel,
                TargetOrbitNormal = targetNormal,
                AscentDurationSeconds = ascentDuration,
                RemainingDeltaV = vesselState.RemainingDeltaV
            };

            List<LaunchWindowSolver.Candidate> solved = LaunchWindowSolver.Solve(inputs);
            if (solved.Count == 0) return new[] { CreateInvalidCandidate("No launch window inside the next day.") };

            return solved.Select(c => MapCandidate(vesselState, c)).OrderBy(c => c.Score).ToArray();
        }

        private static LaunchCandidate MapCandidate(VesselState vesselState, LaunchWindowSolver.Candidate c)
        {
            if (!c.IsValid)
                return new LaunchCandidate
                {
                    IsValid = false,
                    ReasonUnavailable = c.Reason,
                    LaunchUt = c.LaunchUt,
                    SecondsUntilLaunch = c.SecondsUntilLaunch,
                    EstimatedOrbitsToRendezvous = double.PositiveInfinity,
                    EstimatedDeltaVUsed = double.PositiveInfinity,
                    Score = c.Score,
                    AscentProfile = AscentProfileSolver.CreateInvalid(c.Reason)
                };

            // Real ascent profile for the chosen phasing orbit, so the guidance panel flies the planned insertion.
            AscentProfile ascent = AscentProfileSolver.Create(
                vesselState, c.PhasingApoapsisAlt, c.PhasingPeriapsisAlt, c.AzimuthDeg, vesselState.RemainingDeltaV);

            return new LaunchCandidate
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                LaunchUt = c.LaunchUt,
                SecondsUntilLaunch = c.SecondsUntilLaunch,
                InsertionApoapsisAlt = c.PhasingApoapsisAlt,
                InsertionPeriapsisAlt = c.PhasingPeriapsisAlt,
                LaunchHeadingDeg = c.AzimuthDeg,
                EstimatedInsertionTimeSeconds = ascent.IsValid ? ascent.EstimatedTimeToInsertionSeconds : double.NaN,
                EstimatedOrbitsToRendezvous = c.OrbitsToRendezvous,
                EstimatedDeltaVUsed = c.EstimatedDeltaVUsed,
                EstimatedRemainingDeltaV = c.RemainingDeltaV,
                PlaneErrorDeg = c.PlaneErrorDeg,
                PhaseErrorDeg = c.PhaseErrorDeg,
                PhasingOrbit = c.PhasingApoapsisAlt,
                RelativeDistanceMeters = c.PredictedClosestApproachMeters,
                Score = c.Score,
                AscentProfile = ascent,
                PhasingRecommendation = null
            };
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
