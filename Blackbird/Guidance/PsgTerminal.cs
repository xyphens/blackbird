using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    internal sealed class PsgScale
    {
        public double Length { get; private set; }
        public double Velocity { get; private set; }
        public double Time { get; private set; }
        public double Mass { get; private set; }

        public static PsgScale FromProblem(PsgProblem problem)
        {
            double length = Math.Max(1.0, problem.InitialRelativePositionMeters.magnitude);
            double velocity = Math.Sqrt(problem.BodyGravParameter / length);
            return new PsgScale
            {
                Length = length,
                Velocity = velocity,
                Time = length / velocity,
                Mass = Math.Max(1.0, problem.InitialMassKg)
            };
        }
    }

    internal enum PsgTerminalKind
    {
        FlightPathAngle4,
        FlightPathAngle5,
        Kepler3,
        Kepler4,
        Kepler5
    }

    internal sealed class PsgTerminal
    {
        private readonly PsgTerminalKind _kind;
        private readonly double _gamma;
        private readonly double _radius;
        private readonly double _speed;
        private readonly double _eccentricity;
        private readonly double _specificEnergy;
        private readonly Vector3d _normal;
        private readonly Vector3d _angularMomentum;
        private readonly Vector3d _eccentricityVector;

        private PsgTerminal(
            PsgTerminalKind kind,
            double gamma,
            double radius,
            double speed,
            double eccentricity,
            double specificEnergy,
            Vector3d normal,
            Vector3d angularMomentum,
            Vector3d eccentricityVector)
        {
            _kind = kind;
            _gamma = gamma;
            _radius = radius;
            _speed = speed;
            _eccentricity = eccentricity;
            _specificEnergy = specificEnergy;
            _normal = normal.sqrMagnitude > 0.0 ? normal.normalized : Vector3d.zero;
            _angularMomentum = angularMomentum;
            _eccentricityVector = eccentricityVector;
        }

        public int ConstraintCount
        {
            get
            {
                switch (_kind)
                {
                    case PsgTerminalKind.FlightPathAngle5:
                        return 5;
                    case PsgTerminalKind.FlightPathAngle4:
                        return 4;
                    case PsgTerminalKind.Kepler5:
                        return 6;
                    case PsgTerminalKind.Kepler4:
                        return 4;
                    default:
                        return 3;
                }
            }
        }

        public double TargetSpecificEnergy
        {
            get { return _specificEnergy; }
        }

        public double TargetAngularMomentum
        {
            get { return _angularMomentum.magnitude; }
        }

        public Vector3d TargetNormal
        {
            get { return _normal; }
        }

        public bool UsesFlightPathAngle
        {
            get { return _kind == PsgTerminalKind.FlightPathAngle4 || _kind == PsgTerminalKind.FlightPathAngle5; }
        }

        public static PsgTerminal Create(PsgProblem problem, PsgScale scale, bool fixedBurnTime)
        {
            PsgTarget target = problem.Target;
            double pe = target.PeriapsisRadiusMeters;
            double ap = target.ApoapsisRadiusMeters;
            if (ap < pe)
            {
                double temp = ap;
                ap = pe;
                pe = temp;
            }

            double sma = (pe + ap) * 0.5;
            double ecc = (ap - pe) / (ap + pe);
            double attachment = target.AttachmentRadiusMeters;
            bool attach = target.UseAttachmentRadius;
            if (!attach && ecc < 1e-4)
            {
                attach = true;
                attachment = pe;
            }

            attachment = ClampAttachmentRadius(attachment, pe, ap);
            Vector3d normal = target.TargetOrbitNormal.sqrMagnitude > 0.0
                ? target.TargetOrbitNormal.normalized
                : Vector3d.Cross(problem.InitialRelativePositionMeters, problem.InitialRelativeVelocityMetersPerSecond).normalized;

            double mu = problem.BodyGravParameter;
            double hMagnitude = Math.Sqrt(mu * sma * Math.Max(0.0, 1.0 - ecc * ecc));
            double speed = Math.Sqrt(Math.Max(0.0, mu * (2.0 / attachment - 1.0 / sma)));
            double gamma = OrbitMath.SafeAcos(OrbitMath.Clamp(hMagnitude / Math.Max(1e-9, attachment * speed), -1.0, 1.0));
            if (!OrbitMath.IsFinite(gamma)) gamma = 0.0;

            double scaledRadius = attachment / scale.Length;
            double scaledSpeed = speed / scale.Velocity;
            double scaledEnergy = -mu / (2.0 * sma) / (scale.Velocity * scale.Velocity);
            Vector3d scaledAngularMomentum = normal * (hMagnitude / (scale.Length * scale.Velocity));

            PsgTerminalKind kind;
            if (attach || fixedBurnTime)
            {
                kind = normal.sqrMagnitude > 0.0 ? PsgTerminalKind.FlightPathAngle5 : PsgTerminalKind.FlightPathAngle4;
            }
            else if (target.UseArgpConstraint)
            {
                kind = PsgTerminalKind.Kepler5;
            }
            else if (target.UseLanConstraint && normal.sqrMagnitude > 0.0)
            {
                kind = PsgTerminalKind.Kepler4;
            }
            else
            {
                kind = PsgTerminalKind.Kepler3;
            }

            Vector3d eccentricityVector = Vector3d.zero;
            if (kind == PsgTerminalKind.Kepler5)
            {
                eccentricityVector = GetEccentricityVector(ecc, normal, target.ArgpDeg);
            }

            return new PsgTerminal(
                kind,
                gamma,
                scaledRadius,
                scaledSpeed,
                ecc,
                scaledEnergy,
                normal,
                scaledAngularMomentum,
                eccentricityVector);
        }

        public PsgTerminal GetFlightPathAngleTerminal()
        {
            if (UsesFlightPathAngle) return this;
            PsgTerminalKind kind = _normal.sqrMagnitude > 0.0
                ? PsgTerminalKind.FlightPathAngle5
                : PsgTerminalKind.FlightPathAngle4;

            return new PsgTerminal(
                kind,
                _gamma,
                _radius,
                _speed,
                _eccentricity,
                _specificEnergy,
                _normal,
                _angularMomentum,
                _eccentricityVector);
        }

        public void Evaluate(Vector3d r, Vector3d v, double[] f, ref int ci)
        {
            switch (_kind)
            {
                case PsgTerminalKind.FlightPathAngle5:
                    AppendVector(f, ref ci, Vector3d.Cross(r, v) - _angularMomentum);
                    f[ci++] = Vector3d.Dot(r, v) - Math.Sin(_gamma);
                    f[ci++] = r.sqrMagnitude - _radius * _radius;
                    return;

                case PsgTerminalKind.FlightPathAngle4:
                    f[ci++] = Vector3d.Dot(r, v) - Math.Sin(_gamma);
                    f[ci++] = r.sqrMagnitude - _radius * _radius;
                    f[ci++] = v.sqrMagnitude - _speed * _speed;
                    f[ci++] = NormalAlignment(r, v) - 1.0;
                    return;

                case PsgTerminalKind.Kepler5:
                    AppendVector(f, ref ci, Vector3d.Cross(r, v) - _angularMomentum);
                    AppendVector(f, ref ci, EccentricityVector(r, v) - _eccentricityVector);
                    return;

                case PsgTerminalKind.Kepler4:
                    Vector3d e = EccentricityVector(r, v);
                    f[ci++] = e.sqrMagnitude - _eccentricity * _eccentricity;
                    AppendVector(f, ref ci, Vector3d.Cross(r, v) - _angularMomentum);
                    return;

                default:
                    Vector3d h = Vector3d.Cross(r, v);
                    f[ci++] = 0.5 * h.sqrMagnitude - 0.5 * _angularMomentum.sqrMagnitude;
                    f[ci++] = 0.5 * v.sqrMagnitude - 1.0 / Math.Max(1e-9, r.magnitude) - _specificEnergy;
                    f[ci++] = NormalAlignment(r, v) - 1.0;
                    return;
            }
        }

        private static double ClampAttachmentRadius(double attachment, double pe, double ap)
        {
            if (!OrbitMath.IsFinite(attachment) || attachment <= 0.0) return pe;
            if (attachment < pe) return pe;
            if (ap > pe && attachment > ap) return ap;
            return attachment;
        }

        private double NormalAlignment(Vector3d r, Vector3d v)
        {
            Vector3d h = Vector3d.Cross(r, v);
            if (h.sqrMagnitude <= 0.0 || _normal.sqrMagnitude <= 0.0) return 0.0;
            return Vector3d.Dot(h.normalized, _normal);
        }

        private static Vector3d EccentricityVector(Vector3d r, Vector3d v)
        {
            double radius = Math.Max(1e-9, r.magnitude);
            return Vector3d.Cross(v, Vector3d.Cross(r, v)) - r / radius;
        }

        private static Vector3d GetEccentricityVector(double eccentricity, Vector3d normal, double argpDeg)
        {
            if (eccentricity <= 0.0 || normal.sqrMagnitude <= 0.0) return Vector3d.zero;
            Vector3d yAxis = new Vector3d(0.0, 1.0, 0.0);
            Vector3d xAxis = new Vector3d(1.0, 0.0, 0.0);
            Vector3d reference = Math.Abs(Vector3d.Dot(normal.normalized, yAxis)) < 0.95
                ? yAxis
                : xAxis;
            Vector3d periapsisBase = Vector3d.Cross(normal, reference).normalized;
            Vector3d periapsisSide = Vector3d.Cross(normal, periapsisBase).normalized;
            double argp = argpDeg * Math.PI / 180.0;
            return (periapsisBase * Math.Cos(argp) + periapsisSide * Math.Sin(argp)).normalized * eccentricity;
        }

        private static void AppendVector(double[] f, ref int ci, Vector3d value)
        {
            f[ci++] = value.x;
            f[ci++] = value.y;
            f[ci++] = value.z;
        }
    }
}
