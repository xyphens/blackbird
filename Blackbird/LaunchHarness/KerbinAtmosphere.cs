using System;

namespace Blackbird.LaunchHarness
{
    // Stock Kerbin atmospheric density vs altitude (from Kerbin.cfg's model, tabulated). Offline second-body fixture
    // for the cross-body Solve check; the flight code reads the live body. Density is given directly, so no gas law.
    internal static class KerbinAtmosphere
    {
        // altitude(m), density(kg/m^3)
        private static readonly double[][] Density =
        {
            K(0, 1.225), K(2500, 0.898), K(5000, 0.642), K(7500, 0.446), K(10000, 0.288),
            K(15000, 0.108), K(20000, 0.040), K(25000, 0.015), K(30000, 0.006), K(40000, 0.001),
            K(50000, 0.0003), K(60000, 0.00005), K(70000, 0.0)
        };

        public static double DensityAt(double altitudeMeters)
        {
            if (altitudeMeters <= Density[0][0]) return Density[0][1];
            if (altitudeMeters >= Density[Density.Length - 1][0]) return 0.0;
            for (int i = 0; i < Density.Length - 1; i++)
            {
                double a0 = Density[i][0], a1 = Density[i + 1][0];
                if (altitudeMeters < a0 || altitudeMeters > a1) continue;
                double f = (altitudeMeters - a0) / (a1 - a0);
                return Density[i][1] + f * (Density[i + 1][1] - Density[i][1]);
            }
            return 0.0;
        }

        private static double[] K(double a, double d) => new[] { a, d };
    }
}
