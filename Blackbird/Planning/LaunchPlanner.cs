using System;
using Blackbird.Models;
using Blackbird.Mathematics;
using Blackbird.Enums;

using UnityEngine;
using KSP;

namespace Blackbird.Planning
{
    public static class LaunchPlanner
    {
        public static double GetPhasingOffset(Vessel active)
        {
            return PlanetScale.GetScale() == PlanetScale.PlanetScaleEnum.RSS ? 50000 : 30000;
        }

        public static LaunchPlan Create(Vessel active, Vessel target, InsertionTarget insertionTarget, LaunchLocation launchSite)
        {
            if (active == null || target == null)
            {
                return null;
            }

            // init basic target orbit info
            // double targetAltitude = target.orbit.altitude;
            // double phasingOffset = GetPhasingOffset(active);

            // launch site init
            LaunchLocation ls = launchSite ?? LaunchLocation.FromVessel(active);

            // decide our insertion target
            InsertionTarget targetInsertion = insertionTarget ?? InsertionTarget.FromTargetOrbit(target);
            OrbitInfo activeOrbit = OrbitInfo.Create(active.orbit);
            OrbitInfo targetOrbit = OrbitInfo.Create(target.orbit);

            double phaseAngleDeg = OrbitMath.GetPhaseAngleDeg(active, target);

            PhasingOrbit po = PhasingOrbit.FromInsertionTarget(targetInsertion, targetOrbit, target.mainBody, phaseAngleDeg);

            LaunchWindowInfo lwi = LaunchWindowInfo.Create(active, targetOrbit, ls);

            PhasingRecommendation pr = PhasingRecommendationCalculator.Create(
                                        active.mainBody,
                                        targetOrbit,
                                        phaseAngleDeg,
                                        PhasingRecommendationMode.Balanced); // todo: make this a user input

            return new LaunchPlan
            {
                ActiveOrbit = activeOrbit,
                TargetOrbit = targetOrbit,
                LaunchWindow = lwi,
                PhaseAngleDeg = OrbitMath.GetPhaseAngleDeg(active, target),
                DistanceMeters = Vector3d.Distance(active.GetWorldPos3D(), target.GetWorldPos3D()),
                LaunchAzimuthDeg = lwi.SelectedAzimuthDeg,
                RecommendedApAlt = targetInsertion.ApoapsisAlt,
                RecommendedPeAlt = targetInsertion.PeriapsisAlt,
                RelativeInclinationDeg = targetOrbit.InclinationDeg - activeOrbit.InclinationDeg,
                RelativeLanDeg = OrbitMath.DeltaDegrees(activeOrbit.LanDeg, targetOrbit.LanDeg),
                RelativePeriodSeconds = targetOrbit.PeriodSeconds - activeOrbit.PeriodSeconds,
                InsertionTarget = targetInsertion,
                PhasingOrbit = po,
                ScaleLabel = PlanetScale.GetScale(),
                PhasingRecommendation = pr
            };
        }
    }
}
