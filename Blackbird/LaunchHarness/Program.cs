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

            CheckLaunchPlaneError();
            return 0;
        }

        private static readonly Vector3d Pole = new Vector3d(0, 0, 1);

        // Attributes the ~10 deg rel-inc / ~22 deg RAAN launch-plane miss to its real source. The Earth-rotation
        // azimuth correction is sub-degree for an i~lat tangent launch, so this measures rel-inc vs the two real
        // suspects — launch-TIMING slip (huge dRAAN/dt at the tangent) and J2 DIFFERENTIAL nodal regression over
        // the phasing time — plus the azimuth axis as a control. Pure geometry/secular model, no KSP runtime.
        private static void CheckLaunchPlaneError()
        {
            Console.WriteLine();
            Console.WriteLine("=== Launch-plane error attribution (i~lat tangent: inc 28.64, lat 28.6, RSS) ===");

            double mu = EarthMu, R = EarthRadius;
            double omega = 2.0 * Math.PI / EarthSiderealDay;
            double phi = 28.6, iT = 28.64;
            double rIns = R + 200000.0;
            double v = Math.Sqrt(mu / rIns);
            double vRot = omega * R * Math.Cos(phi * Math.PI / 180.0);

            double beta0Rad = Math.Asin(Math.Min(1.0, Math.Cos(iT * Math.PI / 180.0) / Math.Cos(phi * Math.PI / 180.0)));
            double beta0 = beta0Rad * 180.0 / Math.PI;             // uncorrected inertial azimuth, flown as surface hdg

            // Target plane = what a PERFECT, no-rotation launch at the planned heading achieves. Self-consistent
            // with AchievedNormal by construction, so flying beta0 with vRot=0 gives rel-inc 0 (model self-check)
            // and any nonzero result is purely the physical effect under study.
            Vector3d nTarget = AchievedNormal(phi, beta0, 0.0, v, 0.0);

            // Axis A: azimuth. Uncorrected = the planned inertial azimuth flown as a surface heading. Best = the
            // surface heading that actually hits the target plane (the true Earth-rotation correction), found by
            // search so the self-check is exact. The gap is the correctable plane error — tangent geometry blows
            // a sub-degree heading change up into several degrees of rel-inc.
            double riUncorr = RelIncDeg(AchievedNormal(phi, beta0, 0.0, v, vRot), nTarget);
            double bestBeta = beta0, bestRi = double.MaxValue;
            for (double b = beta0 - 10.0; b <= beta0 + 10.0; b += 0.005)
            {
                double ri = RelIncDeg(AchievedNormal(phi, b, 0.0, v, vRot), nTarget);
                if (ri < bestRi) { bestRi = ri; bestBeta = b; }
            }
            Console.WriteLine($"  azimuth:  uncorrected hdg {beta0:F2} deg -> rel-inc {riUncorr:F2} deg" +
                              $"   |   best hdg {bestBeta:F2} deg -> rel-inc {bestRi:F3} deg   (correctable {riUncorr - bestRi:F2} deg)");

            // Axis B: launch-timing slip (fly the planned heading from a pad rotated by omega*dt).
            Console.WriteLine("  timing slip (fly planned hdg):");
            foreach (double dt in new[] { -600.0, -300.0, -120.0, -60.0, 0.0, 60.0, 120.0, 300.0, 600.0 })
            {
                double lonDeg = omega * dt * 180.0 / Math.PI;
                Console.WriteLine($"    dt {dt,6:F0} s ({lonDeg,5:F2} deg) -> rel-inc {RelIncDeg(AchievedNormal(phi, beta0, lonDeg, v, vRot), nTarget),6:F2} deg");
            }

            // Axis C: J2 differential nodal regression (chaser 281 km vs target 310 km) over phasing time.
            Console.WriteLine("  J2 differential regression (chaser 281km vs target 310km, i=28.6):");
            double odChaser = NodalRate(mu, R, EarthJ2, R + 281000.0, phi);
            double odTarget = NodalRate(mu, R, EarthJ2, R + 310000.0, phi);
            foreach (double hours in new[] { 1.0, 3.0, 6.0, 12.0 })
            {
                double dRaan = (odChaser - odTarget) * hours * 3600.0;
                Console.WriteLine($"    {hours,4:F0} h  dRAAN {dRaan * 180.0 / Math.PI,6:F2} deg -> rel-inc {RelIncFromRaan(iT, dRaan),5:F2} deg");
            }
        }

        // Orbital plane normal achieved by flying surface heading betaDeg from a pad at lonDeg, with Earth rotation
        // added to the inertial velocity (the open-loop launch the guidance actually flies).
        private static Vector3d AchievedNormal(double latDeg, double betaDeg, double lonDeg, double v, double vRot)
        {
            Vector3d r = SiteVector(EarthRadius, latDeg, lonDeg);
            Vector3d up = r.normalized;
            Vector3d east = Vector3d.Cross(Pole, up).normalized;
            Vector3d north = Vector3d.Cross(up, east);
            double b = betaDeg * Math.PI / 180.0;
            Vector3d vel = v * (Math.Sin(b) * east + Math.Cos(b) * north) + vRot * east;
            return Vector3d.Cross(r, vel).normalized;
        }

        private static double RelIncDeg(Vector3d a, Vector3d b)
        {
            double ang = Vector3d.Angle(a, b);
            return Math.Min(ang, 180.0 - ang);
        }

        // Secular J2 nodal regression rate (rad/s) for a circular orbit.
        private static double NodalRate(double mu, double R, double j2, double a, double iDeg)
        {
            double n = Math.Sqrt(mu / (a * a * a));
            return -1.5 * n * j2 * (R / a) * (R / a) * Math.Cos(iDeg * Math.PI / 180.0);
        }

        // Relative inclination (deg) between two planes of equal inclination iDeg differing by dRaan (rad).
        private static double RelIncFromRaan(double iDeg, double dRaanRad)
        {
            double i = iDeg * Math.PI / 180.0;
            double c = Math.Cos(i) * Math.Cos(i) + Math.Sin(i) * Math.Sin(i) * Math.Cos(dRaanRad);
            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, c))) * 180.0 / Math.PI;
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
