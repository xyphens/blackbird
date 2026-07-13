using Blackbird.Mathematics;
using System;
using System.Collections.Generic;

namespace Blackbird.Guidance
{
    public struct AscentState
    {
        public double T;   // seconds since integration start
        public double X;    // body-centered position, launch-plane meters
        public double Y;
        public double Vx;   // surface-relative velocity
        public double Vy;
        public double MassKg;

        public double Radius => Math.Sqrt(X * X + Y * Y);
        public double SurfaceSpeed => Math.Sqrt(Vx * Vx + Vy * Vy);
        public double AltitudeMeters(double bodyRadius) => Radius - bodyRadius;

        public double FlightPathAngleDeg
        {
            get
            {
                double r = Radius, v = SurfaceSpeed;
                if (r <= 0.0 || v <= 0.0) return 90.0;
                double s = MathHelpers.Clamp((Vx * X + Vy * Y) / (r * v), -1.0, 1.0);
                return MathHelpers.Rad2Deg(Math.Asin(s));
            }
        }
    }

    public struct AscentStage
    {
        public double ThrustNewtons;
        public double MassFlowKgPerSec;
        public double PropellantKg;
        public double JettisonMassKg;
    }

    public sealed class AscentDynamicsConfig
    {
        public double Mu;
        public double BodyRadius;
        public double DragAreaCd; // Cd*A, m^2
        public AscentStage[] Stages;
        public Func<double, double> DensityAtAltitude; // altitude m -> kg/m^3
        public Func<AscentState, double> SteeringPitchFromVerticalDeg; // commanded attitude vs local vertical
    }
    public sealed class AscentIntegrator
    {
        private readonly AscentDynamicsConfig _cfg;
        private int _stageIndex;
        private double _propellantRemainingKg;

        public bool HasStage => _cfg.Stages != null && _stageIndex < _cfg.Stages.Length;

        // dutodo: wrap this in a try/catch so game doesn't spam errors
        public AscentIntegrator(AscentDynamicsConfig cfg)
        {
            _cfg = cfg ?? throw new ArgumentNullException(nameof(cfg));
            _stageIndex = 0;
            _propellantRemainingKg = HasStage ? _cfg.Stages[0].PropellantKg : 0.0;
        }

        public AscentState Step(AscentState s, double dt)
        {
            double thrust = 0.0;
            double mdot = 0.0;
            if (HasStage)
            {
                thrust = _cfg.Stages[_stageIndex].ThrustNewtons;
                if (_propellantRemainingKg > 0.0)
                {
                    mdot = _cfg.Stages[_stageIndex].MassFlowKgPerSec;
                }
            }

            if (mdot <= 0.0) thrust = 0.0;

            AscentState k1 = Derivative(s, thrust, mdot);
            AscentState k2 = Derivative(Advance(s, k1, dt * 0.5), thrust, mdot);
            AscentState k3 = Derivative(Advance(s, k2, dt * 0.5), thrust, mdot);
            AscentState k4 = Derivative(Advance(s, k3, dt), thrust, mdot);

            s.T += dt;
            s.X += dt / 6.0 * (k1.X + 2.0 * k2.X + 2.0 * k3.X + k4.X);
            s.Y += dt / 6.0 * (k1.Y + 2.0 * k2.Y + 2.0 * k3.Y + k4.Y);
            s.Vx += dt / 6.0 * (k1.Vx + 2.0 * k2.Vx + 2.0 * k3.Vx + k4.Vx);
            s.Vy += dt / 6.0 * (k1.Vy + 2.0 * k2.Vy + 2.0 * k3.Vy + k4.Vy);
            s.MassKg += dt / 6.0 * (k1.MassKg + 2.0 * k2.MassKg + 2.0 * k3.MassKg + k4.MassKg);

            if (mdot > 0.0)
            {
                _propellantRemainingKg -= mdot * dt;
                if (_propellantRemainingKg <= 0.0)
                {
                    s.MassKg -= _cfg.Stages[_stageIndex].JettisonMassKg;
                    _stageIndex++;
                    _propellantRemainingKg = HasStage ? _cfg.Stages[_stageIndex].PropellantKg : 0.0;
                }
            }

            return s;
        }

        public List<AscentState> Integrate(AscentState s0, double dt, Func<AscentState, bool> stop, int maxSteps)
        {
            var path = new List<AscentState>(Math.Min(maxSteps, 4096)) { s0 };
            AscentState s = s0;
            for (int i = 0; i < maxSteps; i++)
            {
                if (stop != null && stop(s)) break;
                s = Step(s, dt);
                path.Add(s);
            }

            return path;
        }
        private AscentState Derivative(AscentState s, double thrust, double mdot)
        {
            double r = s.Radius;
            double rx = s.X / r, ry = s.Y / r;
            double ex = -ry, ey = rx;

            double psi = _cfg.SteeringPitchFromVerticalDeg(s) * Math.PI / 180.0;
            double tx = Math.Cos(psi) * rx + Math.Sin(psi) * ex;
            double ty = Math.Cos(psi) * ry + Math.Sin(psi) * ey;

            double v = s.SurfaceSpeed;
            double rho = _cfg.DensityAtAltitude(r - _cfg.BodyRadius);
            double drag = 0.0, vhx = 0.0, vhy = 0.0, invM = 0.0;

            if (v > 0.0)
            {
                if (rho > 0.0) drag = 0.5 * rho * v * v * _cfg.DragAreaCd;

                vhx = s.Vx / v;
                vhy = s.Vy / v;
            }

            double g = _cfg.Mu / (r * r);
            if (s.MassKg > 0.0) invM = 1.0 / s.MassKg;

            return new AscentState
            {
                T = 1.0,
                X = s.Vx,
                Y = s.Vy,
                Vx = (thrust * tx - drag * vhx) * invM - g * rx,
                Vy = (thrust * ty - drag * vhy) * invM - g * ry,
                MassKg = -mdot
            };
        }

        private static AscentState Advance(AscentState s, AscentState d, double h)
        {
            return new AscentState
            {
                T = s.T + d.T * h,
                X = s.X + d.X * h,
                Y = s.Y + d.Y * h,
                Vx = s.Vx + d.Vx * h,
                Vy = s.Vy + d.Vy * h,
                MassKg = s.MassKg + d.MassKg * h
            };
        }
    }
}
