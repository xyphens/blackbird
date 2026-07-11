using System;

namespace Blackbird.LaunchHarness
{
    // RSS-Earth atmospheric density from the game's own pressure/temperature FloatCurves (Earth.cfg), evaluated with
    // KSP's cubic-Hermite interpolation. Offline drag fixture only; the flight code reads the live body curve.
    internal static class RssAtmosphere
    {
        private const double RUniversal = 8.31446;
        private const double MolarMass = 0.0289644;   // Earth.cfg atmosphereMolarMass

        // key = altitude(m), value, inTangent, outTangent
        private static readonly double[][] Pressure =   // kPa
        {
            K(0,101.325,0,-0.0119729), K(1000,89.9537,-0.0107923,-0.0107923), K(2000,79.7013,-0.00972759,-0.00972759),
            K(3000,70.4691,-0.00875313,-0.00875313), K(4000,62.1620,-0.00787633,-0.00787633), K(5000,54.6886,-0.00708329,-0.00708329),
            K(6000,47.9719,-0.00636074,-0.00636074), K(7000,41.9470,-0.00569867,-0.00569867), K(8000,36.5555,-0.00509376,-0.00509376),
            K(9000,31.7428,-0.00453892,-0.00453892), K(10000,27.4635,-0.00402664,-0.00402664), K(12000,20.3407,-0.00312205,-0.00312205),
            K(14000,14.8739,-0.00236992,-0.00236992), K(16000,10.7657,-0.00175875,-0.00175875), K(18000,7.76098,-0.00126703,-0.00126703),
            K(20000,5.61289,-0.000901159,-0.000901159), K(22000,4.08419,-0.000643110,-0.000643110), K(24000,2.98894,-0.000462653,-0.000462653),
            K(26000,2.19866,-0.000334849,-0.000334849), K(28000,1.62536,-0.000243495,-0.000243495), K(30000,1.20769,-0.000177736,-0.000177736),
            K(35000,0.588602,-8.25983e-05,-8.25983e-05), K(40000,0.296819,-3.96388e-05,-3.96388e-05), K(45000,0.154692,-1.97099e-05,-1.97099e-05),
            K(50000,0.0825035,-1.03082e-05,-1.03082e-05), K(55000,0.0438832,-5.63677e-06,-5.63677e-06), K(60000,0.0227005,-3.07935e-06,-3.07935e-06),
            K(65000,0.0112807,-1.62592e-06,-1.62592e-06), K(70000,0.00536204,-8.22892e-07,-8.22892e-07), K(75000,0.00243557,-3.94225e-07,-3.94225e-07),
            K(80000,0.00106710,-1.78982e-07,-1.78982e-07), K(85000,0.000456872,-7.82929e-08,-7.82929e-08), K(90000,0.000192739,-3.34218e-08,-3.34218e-08),
            K(95000,8.12137e-05,-1.38889e-08,-1.38889e-08), K(100000,3.52962e-05,-5.69392e-09,-5.69392e-09), K(105000,1.62730e-05,-2.40474e-09,-2.40474e-09),
            K(110000,8.14091e-06,-1.04206e-09,-1.04206e-09), K(115000,4.55287e-06,-4.76718e-10,-4.76718e-10), K(121920,2.40103e-06,-1.98682e-10,-1.98682e-10),
            K(140000,0,0,0)
        };

        private static readonly double[][] Temperature =   // K
        {
            K(0,282.5,0,-0.0025), K(8000,240.5,-0.006,-0.006), K(15000,212,-0.0025,-0.0025), K(21000,214,0.0015,0.0015),
            K(30000,228,0.002,0.002), K(42000,255.5,0.0025,0.0025), K(49750,268,0,0), K(60000,247.5,-0.003,-0.003),
            K(75000,209,-0.002,-0.002), K(91000,191.75,0,0), K(100000,206,0.003,0.003), K(110000,256,0.009,0.009),
            K(120000,375,0.011,0.011), K(140000,560,0.007,0)
        };

        // kg/m^3 from ideal gas: rho = P*M/(R*T).
        public static double Density(double altitudeMeters)
        {
            double p = Evaluate(Pressure, altitudeMeters) * 1000.0;   // kPa -> Pa
            double t = Evaluate(Temperature, altitudeMeters);
            if (p <= 0.0 || t <= 0.0) return 0.0;
            return p * MolarMass / (RUniversal * t);
        }

        private static double[] K(double a, double v, double inT, double outT) => new[] { a, v, inT, outT };

        // KSP FloatCurve cubic-Hermite evaluation, clamped outside the key range.
        private static double Evaluate(double[][] keys, double x)
        {
            if (x <= keys[0][0]) return keys[0][1];
            if (x >= keys[keys.Length - 1][0]) return keys[keys.Length - 1][1];

            for (int i = 0; i < keys.Length - 1; i++)
            {
                double a0 = keys[i][0], a1 = keys[i + 1][0];
                if (x < a0 || x > a1) continue;

                double dt = a1 - a0;
                double t = (x - a0) / dt;
                double m0 = keys[i][3] * dt;       // out-tangent of left key
                double m1 = keys[i + 1][2] * dt;   // in-tangent of right key
                double t2 = t * t, t3 = t2 * t;
                return (2 * t3 - 3 * t2 + 1) * keys[i][1] + (t3 - 2 * t2 + t) * m0
                     + (-2 * t3 + 3 * t2) * keys[i + 1][1] + (t3 - t2) * m1;
            }
            return keys[keys.Length - 1][1];
        }
    }
}
