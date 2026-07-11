using Blackbird.Mathematics;
using System;

namespace Blackbird.Guidance
{
    // Inputs for the 1-D kick-angle shooting solve. Vehicle/atmosphere come from VesselState + the body at solve time.
    public sealed class AscentShootingInputs
    {
        public double Mu;
        public double BodyRadius;
        public double DragAreaCd;                        // Cd*A, m^2
        public AscentStage[] Stages;
        public Func<double, double> DensityAtAltitude;   // altitude m -> kg/m^3
        public AscentState InitialState;                 // launch state (near-vertical, low speed)
        public double TowerClearanceAltitudeMeters;      // hold vertical until cleared, then the kick applies
        public double HandoverAltitudeMeters;            // read FPA here (the shooting target altitude)
        public double TargetFlightPathAngleDeg;          // desired ORBIT-frame (orbit-frame FPA) at handover
        public double MaxKickDeg;                        // upper bracket for the kick
        public double StepSeconds;
        public int MaxSteps;

        // Orbit-frame terms (co-rotation): needed only by the steering law's orbit-prograde phase, not the engine.
        public double RotationRateRadPerSec;             // body omega (2*pi / rotationPeriod)
        public double LaunchLatitudeCos;                 // cos(latitude) of the pad
        public double DynamicPressureShiftPa;            // q below which steering shifts surface- -> orbit-prograde
    }

    public struct AscentShootingResult
    {
        public bool Converged;
        public double KickAngleDeg;
        public double AchievedFlightPathAngleDeg;        // orbit frame, at handover
        public string Status;
    }

    public struct AscentHandoverPrediction
    {
        public double SurfaceFpaDeg;
        public double OrbitFpaDeg;
        public bool ReachedHandover;
    }

    // bisection on the pitch-kick angle so predicted flight-path angle at handover equals the target
    public static class AscentShootingSolver
    {
        private const double FpaToleranceDeg = 0.05;
        private const int MaxBisectionIterations = 60;

        public static AscentShootingResult Solve(AscentShootingInputs io)
        {
            double target = io.TargetFlightPathAngleDeg; // orbit frame
            double fLo = PredictHandover(io, 0.0).OrbitFpaDeg - target;           // no kick: most vertical
            double fHi = PredictHandover(io, io.MaxKickDeg).OrbitFpaDeg - target;   // max kick: most horizontal
            if (fLo < 0.0) return Fail(0.0, fLo + target, "Target is more horizontal than zero-kick ascent");
            if (fHi > 0.0) return Fail(io.MaxKickDeg, fHi + target, "TWR-limited - max kick cannot reach target flight path angle");

            double lo = 0.0;
            double hi = io.MaxKickDeg;
            double mid = 0.0;
            double achieved = double.NaN;

            for (int i = 0; i < MaxBisectionIterations; i++) {
                mid = 0.5 * (lo + hi);
                double f = PredictHandover(io, mid).OrbitFpaDeg - target;
                achieved = f + target;
                if (Math.Abs(f) <= FpaToleranceDeg) break;
                // orbit flight path angle decreases with kick: too vertical = kick harder
                if (f > 0.0)
                {
                    lo = mid;
                } else
                {
                    hi = mid;
                }
            }

            return new AscentShootingResult
            {
                Converged = true,
                KickAngleDeg = mid,
                AchievedFlightPathAngleDeg = achieved,
                Status = "Converged"
            };
        }

        public static AscentHandoverPrediction PredictHandover(AscentShootingInputs io, double kickDeg)
        {
            var cfg = new AscentDynamicsConfig
            {
                Mu = io.Mu,
                BodyRadius = io.BodyRadius,
                DragAreaCd = io.DragAreaCd,
                Stages = io.Stages,
                DensityAtAltitude = io.DensityAtAltitude,
                SteeringPitchFromVerticalDeg = st => PhasedSteeringPitchDeg(st, io, kickDeg)
            };

            var integrator = new AscentIntegrator(cfg);
            AscentState s = io.InitialState;
            for (int i = 0; i < io.MaxSteps; i++)
            {
                if (s.AltitudeMeters(io.BodyRadius) >= io.HandoverAltitudeMeters) return Prediction(s, io, true);
                s = integrator.Step(s, io.StepSeconds);
                if (s.SurfaceSpeed > 1.0 && s.FlightPathAngleDeg <= 0.0) return Prediction(s, io, false); // pitched past horizontal
            }
            return Prediction(s, io, false);
        }

        // phase-structed steering
        public static double PhasedSteeringPitchDeg(AscentState s, AscentShootingInputs io, double kickDeg)
        {
            double alt = s.AltitudeMeters(io.BodyRadius);
            if (alt < io.TowerClearanceAltitudeMeters) return 0.0; // vertical hold

            double surfaceProgradePitch = 90.0 - s.FlightPathAngleDeg;
            if (surfaceProgradePitch < kickDeg) return kickDeg;

            double v = s.SurfaceSpeed;
            double q = 0.5 * io.DensityAtAltitude(alt) * v * v;
            if (q > io.DynamicPressureShiftPa) return surfaceProgradePitch; // keep surface-prograde while aero is significant
            return 90.0 - OrbitFlightPathAngleDeg(s, io.RotationRateRadPerSec, io.LaunchLatitudeCos); // orbit-prograde
        }

        // surface velocity plus body co-rotation
        public static double OrbitFlightPathAngleDeg(AscentState s, double rotationRate, double latitudeCos)
        {
            double r = s.Radius;
            double ex = -s.Y / r, ey = s.X / r;
            double vrot = rotationRate * r * latitudeCos;
            double ox = s.Vx + vrot * ex, oy = s.Vy + vrot * ey;
            double ov = Math.Sqrt(ox * ox + oy * oy);
            if (ov <= 0.0) return 90.0;
            double sinFpa = MathHelpers.Clamp((ox * s.X + oy * s.Y) / (r *  ov), -1.0, 1.0);
            return Math.Asin(sinFpa) * 180.0 / Math.PI;
        }

        private static AscentHandoverPrediction Prediction(AscentState s, AscentShootingInputs io, bool reached)
        {
            return new AscentHandoverPrediction
            {
                SurfaceFpaDeg = s.FlightPathAngleDeg,
                OrbitFpaDeg = OrbitFlightPathAngleDeg(s, io.RotationRateRadPerSec, io.LaunchLatitudeCos),
                ReachedHandover = reached
            };
        }

        private static AscentShootingResult Fail(double kick, double achieved, string status)
        {
            return new AscentShootingResult { Converged = false, KickAngleDeg = kick, AchievedFlightPathAngleDeg = achieved, Status = status };
        }
    }
}
