using System;
using Blackbird.Psg;
using UnityEngine;

namespace Blackbird.Models
{

    public struct AscentPathPoint
    {
        public double UniversalTime;
        public Vector3d RelativePosition;
        public Vector3d RelativeVelocity;
        public double MassKg;
        public double Throttle;
    }
    public sealed class AscentPath
    {
        public double CreatedUniversalTime;
        public AscentPathPoint[] Points;
        public bool IsValid => Points != null && Points.Length >= 2;
    }

    public static class AscentPathProvider
    {
        // psg optimal plan when available, otherwise rough conic approximation or current orbit
        public static AscentPath Build(PsgSolution psg, Vessel vessel)
        {
            if (psg != null && psg.IsValid && psg.Points != null && psg.Points.Length >= 2) return FromPsg(psg);
            return FromOrbit(vessel);
        }

        private static AscentPath FromPsg(PsgSolution psg)
        {
            var pts = new AscentPathPoint[psg.Points.Length];
            for (int i = 0; i < pts.Length; i++)
            {
                PsgSolutionPoint p = psg.Points[i];
                pts[i] = new AscentPathPoint
                {
                    UniversalTime = p.UniversalTime,
                    RelativePosition = p.RelativePosition,
                    RelativeVelocity = p.RelativeVelocity,
                    MassKg = p.MassKg,
                    Throttle = p.Throttle
                };
            }
            return new AscentPath { CreatedUniversalTime = psg.CreatedUniversalTime, Points = pts };
        }

        private static AscentPath FromOrbit(Vessel vessel)
        {
            if (vessel == null || vessel.orbit == null || vessel.mainBody == null) return null;
            Orbit o = vessel.orbit;

            double now = Planetarium.GetUniversalTime();
            double horizon = o.timeToAp;
            if (double.IsNaN(horizon) || double.IsInfinity(horizon) || horizon <= 1.0)
            {
                horizon = o.period > 0.0 ? Math.Min(o.period * 0.5, 1200.0) : 300.0;
            }
            horizon = Math.Min(horizon, 1200.0);

            const int n = 60;
            double massKg = vessel.totalMass * 1000.0;
            double throttle = vessel.ctrlState != null ? vessel.ctrlState.mainThrottle : 0.0;
            Vector3d bodyPos = vessel.mainBody.position;

            var pts = new AscentPathPoint[n + 1];
            for (int i = 0; i < n; i++) {
                double ut = now + horizon * i / n;
                pts[i] = new AscentPathPoint
                {
                    UniversalTime = ut,
                    RelativePosition = o.getPositionAtUT(ut) - bodyPos,
                    RelativeVelocity = o.getOrbitalVelocityAtUT(ut),
                    MassKg = massKg,
                    Throttle = throttle
                };
            }
            return new AscentPath { CreatedUniversalTime = now, Points = pts };
        }
    }
}
