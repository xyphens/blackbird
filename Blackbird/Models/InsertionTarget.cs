using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Blackbird.Models
{
    public sealed class InsertionTarget
    {
        public double ApoapsisAlt { get; set; }
        public double PeriapsisAlt { get; set; }

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
        public static InsertionTarget Circular(double altitude)
        {
            return new InsertionTarget
            {
                ApoapsisAlt = altitude,
                PeriapsisAlt = altitude
            };
        }

        // create an insertion from a given orbit
        public static InsertionTarget FromOrbit(OrbitInfo orbit)
        {
            if (orbit == null) return null;
            return new InsertionTarget
            {
                ApoapsisAlt = orbit.ApoapsisAlt,
                PeriapsisAlt = orbit.PeriapsisAlt
            };
        }

        // create an insertion target from a target's orbit
        public static InsertionTarget FromTargetOrbit(Vessel target)
        {
            if (target == null || target.orbit == null) return null;
            return new InsertionTarget
            {
                ApoapsisAlt = target.orbit.ApA,
                PeriapsisAlt = target.orbit.PeA
            };
        }
    }
}
