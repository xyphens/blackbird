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
        //private void ShowTargetOrbitDetails(Vessel active, Vessel target)
        //{
        //    double targetInc = target.orbit.inclination;
        //    double targetLAN = target.orbit.LAN;
        //    double targetAp = target.orbit.ApA;
        //    double targetPe = target.orbit.PeA;
        //    double targetPeriod = target.orbit.period;

        //    double phaseAngle = GetPhaseAngleDeg(target, active);

        //    double distance = Vector3d.Distance(active.GetWorldPos3D(), target.GetWorldPos3D());

        //    GUILayout.Space(10);
        //    GUILayout.Label($"[{target.vesselName}] Orbit Details");
        //    GUILayout.Label($"Distance: {distance:N0} m");
        //    GUILayout.Label($"Inclination: {targetInc:F2}°");
        //    GUILayout.Label($"LAN: {targetLAN:F2}°");
        //    GUILayout.Label($"Apoapsis: {targetAp / 1000:F0} km");
        //    GUILayout.Label($"Periapsis: {targetPe / 1000:F0} km");
        //    GUILayout.Label($"Period: {targetPeriod:N0}");
        //    GUILayout.Label($"Phase Angle: {phaseAngle:N0} degrees");
        //}

        //private void RecommendPhasingAltitude(Vessel target, Vessel active)
        //{
        //    GUILayout.Space(10);
        //    GUILayout.Label("Recommended Launch Details");
        //    double targetAltitude = (target.orbit.ApA + target.orbit.PeA) / 2.0;
        //    double phasingOffset = FlightGlobals.currentMainBody.Radius * 0.05;
        //    double recAp = targetAltitude;
        //    double recPe = targetAltitude - phasingOffset;

        //    GUILayout.Space(10);
        //    GUILayout.Label($"Apoapsis: {recAp / 1000:F0} km");
        //    GUILayout.Label($"Periapsis: {recPe / 1000:F0} km");

        //    double launchAzimuth = GetLaunchAzimuth(
        //        target.orbit.inclination,
        //        active.latitude);

        //    GUILayout.Label($"Launch Azimuth: {launchAzimuth:F1}°");
        //    GUILayout.Label($"Scale: {PlanetScale.GetScale()}");
        //}

        //public void ShowRecommendation(Vessel active, Vessel target)
        //{
        //    GUILayout.Space(10);
        //    ShowTargetOrbitDetails((Vessel)target, active);
        //    RecommendPhasingAltitude((Vessel)target, active);
        //}

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
                ScaleLabel = PlanetScale.GetScale().ToString()
            };
        }
    }
}
