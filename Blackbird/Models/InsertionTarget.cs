using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Models
{
    public sealed class InsertionTarget
    {
        public double ApoapsisAlt { get; set; }
        public double PeriapsisAlt { get; set; }
        public double Heading { get; set; }

        public double SemiMajorAlt
        {
            get
            {
                return (ApoapsisAlt + PeriapsisAlt) / 2.0;
            }
        }

        public bool IsValid
        {
            get
            {
                return ApoapsisAlt > 0.0 && PeriapsisAlt > 0.0 && ApoapsisAlt >= PeriapsisAlt;
            }
        }

        // create a basic circular orbit
        public static InsertionTarget Circular(double altitude, double heading)
        {
            return new InsertionTarget
            {
                ApoapsisAlt = altitude,
                PeriapsisAlt = altitude,
                Heading = heading
            };
        }

        // create an insertion from a given orbit
        public static InsertionTarget FromOrbit(OrbitInfo orbit)
        {
            if (orbit == null) return null;
            return new InsertionTarget
            {
                ApoapsisAlt = orbit.ApoapsisAlt,
                PeriapsisAlt = orbit.PeriapsisAlt,
                Heading = 0 // placeholder
            };
        }

        // Create an insertion target from the active trajectory provider's target orbit summary.
        public static InsertionTarget FromTargetOrbit(Vessel target)
        {
            if (target == null) return null;
            return new InsertionTarget
            {
                ApoapsisAlt = TrajectoryProvider.GetApoapsisAlt(target),
                PeriapsisAlt = TrajectoryProvider.GetPeriapsisAlt(target),
                Heading = 0 // placeholder
            };
        }
    }
}
