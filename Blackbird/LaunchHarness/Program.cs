using System;
using System.Collections.Generic;
using Blackbird.Planning;
using UnityEngine;

namespace Blackbird.LaunchHarness
{
    // Offline launch-window check: builds RSS-Earth body constants + a simulated target orbit + the Cape, runs
    // LaunchWindowSolver, and prints the best ascending / descending candidate. No KSP runtime — it exercises
    // the pure solver only, so we can sanity-check the candidate logic before wiring it into the in-game panel.
    internal static class Program
    {
        // RSS Earth.
        private const double EarthMu = 3.986004418e14;
        private const double EarthRadius = 6378136.3;
        private const double EarthAtmosphere = 140000.0;
        private const double EarthSiderealDay = 86164.0905;
        private const double EarthJ2 = 1.082636e-03;

        // Cape Canaveral.
        private const double CapeLatitude = 28.6;
        private const double CapeLongitude = -80.6;

        private static int Main()
        {
            // Target: 400 km circular at 28.6 deg inclination. Three scenarios vary where the target sits
            // relative to the site's plane-crossing so we see "behind" (chase low) and "ahead" (wait high).
            Scenario("i=51.6, target trailing (insert ahead -> wait high)", 51.6, 0.0, 200.0, EarthJ2, CapeLatitude);
            Scenario("i=51.6, target leading (chase from low)", 51.6, 0.0, 20.0, EarthJ2, CapeLatitude);
            Scenario("i=28.6 == Cape lat (tangent / due-east window)", 28.6, 0.0, 95.0, EarthJ2, CapeLatitude);
            // KSC-on-the-equator case from the in-game test: near-equatorial target, pad at lat ~0. Heading
            // must come out ~90 (east), NOT ~270 (the left-handed-frame retrograde bug).
            Scenario("i=0.18 near-equatorial, pad lat ~0 (heading must be ~90)", 0.18, 0.0, 200.0, EarthJ2, 0.1);
            // Control: same geometry with J2=0 must close to ~0 km (proves the phasing math; isolates J2 as the
            // sole source of the multi-orbit miss above).
            Scenario("CONTROL J2=0 (Keplerian closure, expect predCA ~0)", 51.6, 0.0, 20.0, 0.0, CapeLatitude);
            return 0;
        }

        private static void Scenario(string title, double inclinationDeg, double lanDeg, double argLatDeg, double j2, double launchLatDeg)
        {
            Console.WriteLine();
            Console.WriteLine("=== " + title + " ===");

            double r = EarthRadius + 400000.0;
            StateFromElements(r, inclinationDeg, lanDeg, argLatDeg, EarthMu,
                out Vector3d targetPos, out Vector3d targetVel);

            LaunchWindowSolver.Inputs input = new LaunchWindowSolver.Inputs
            {
                Mu = EarthMu,
                BodyRadius = EarthRadius,
                AtmosphereDepth = EarthAtmosphere,
                RotationPeriodSeconds = EarthSiderealDay,
                J2 = j2,
                J2ReferenceRadius = EarthRadius,
                Pole = new Vector3d(0, 0, 1),

                CurrentUt = 0.0,
                LaunchSitePosition = SiteVector(EarthRadius, launchLatDeg, CapeLongitude),

                TargetPosition = targetPos,
                TargetVelocity = targetVel,
                TargetOrbitNormal = Vector3d.Cross(targetPos, targetVel).normalized,

                AscentDurationSeconds = 500.0,
                RemainingDeltaV = 9500.0
            };

            List<LaunchWindowSolver.Candidate> candidates = LaunchWindowSolver.Solve(input);
            if (candidates.Count == 0) { Console.WriteLine("  (no plane crossings found)"); return; }

            foreach (LaunchWindowSolver.Candidate c in candidates) Print(c);
        }

        private static void Print(LaunchWindowSolver.Candidate c)
        {
            if (!c.IsValid)
            {
                Console.WriteLine(string.Format("  {0,-11}  launch in {1,5:F0} min  -> INVALID: {2}",
                    c.NodeName, c.SecondsUntilLaunch / 60.0, c.Reason));
                return;
            }
            Console.WriteLine(string.Format(
                "  {0,-11}  launch in {1,5:F0} min  hdg {2,5:F1}  phase {3,7:F1} deg  plane {4,4:F1} deg",
                c.NodeName, c.SecondsUntilLaunch / 60.0, c.AzimuthDeg, c.PhaseErrorDeg, c.PlaneErrorDeg));
            Console.WriteLine(string.Format(
                "               phasing {0,4:F0} km x {1,4:F0} orbits  dV {2,5:F0} m/s (rem {3,5:F0})  predCA {4,7:F0} km  score {5:F0}",
                c.PhasingApoapsisAlt / 1000.0, c.OrbitsToRendezvous, c.EstimatedDeltaVUsed,
                c.RemainingDeltaV, c.PredictedClosestApproachMeters / 1000.0, c.Score));
        }

        // Launch-site inertial position (pole = +Z, longitude from +X) at the harness epoch.
        private static Vector3d SiteVector(double radius, double latDeg, double lonDeg)
        {
            double lat = latDeg * Math.PI / 180.0, lon = lonDeg * Math.PI / 180.0;
            double cosLat = Math.Cos(lat);
            return radius * new Vector3d(cosLat * Math.Cos(lon), cosLat * Math.Sin(lon), Math.Sin(lat));
        }

        // Circular state vector from inclination / RAAN / argument-of-latitude (deg).
        private static void StateFromElements(double radius, double incDeg, double lanDeg, double argLatDeg,
            double mu, out Vector3d position, out Vector3d velocity)
        {
            double i = incDeg * Math.PI / 180.0;
            double O = lanDeg * Math.PI / 180.0;
            double u = argLatDeg * Math.PI / 180.0;
            double cosI = Math.Cos(i), sinI = Math.Sin(i);

            Vector3d rHat = new Vector3d(
                Math.Cos(O) * Math.Cos(u) - Math.Sin(O) * Math.Sin(u) * cosI,
                Math.Sin(O) * Math.Cos(u) + Math.Cos(O) * Math.Sin(u) * cosI,
                Math.Sin(u) * sinI);
            Vector3d vHat = new Vector3d(
                -Math.Cos(O) * Math.Sin(u) - Math.Sin(O) * Math.Cos(u) * cosI,
                -Math.Sin(O) * Math.Sin(u) + Math.Cos(O) * Math.Cos(u) * cosI,
                Math.Cos(u) * sinI);

            position = radius * rHat;
            velocity = Math.Sqrt(mu / radius) * vHat;
        }
    }
}
