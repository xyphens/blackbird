using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.Models
{
    public sealed class OrbitInfo
    {
        public double InclinationDeg { get; set; }
        public double LanDeg { get; set; }

        public double ApoapsisAlt { get; set; }
        public double PeriapsisAlt { get; set; }

        public double SemiMajorAxis { get; set; }
        public double PeriodSeconds { get; set; }

        public double Eccentricity { get; set; }

        public static OrbitInfo Create(Orbit orbit)
        {
            if (orbit == null)
            {
                return null;
            }

            return new OrbitInfo
            {
                InclinationDeg = orbit.inclination,
                LanDeg = orbit.LAN,
                ApoapsisAlt = orbit.ApA,
                PeriapsisAlt = orbit.PeA,
                SemiMajorAxis = orbit.semiMajorAxis,
                PeriodSeconds = orbit.period,
                Eccentricity = orbit.eccentricity
            };
        }
    }
}
