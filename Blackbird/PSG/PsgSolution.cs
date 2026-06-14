using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Psg
{
    public sealed class PsgGuidanceVector
    {
        public bool IsValid { get; set; }
        public Vector3d InertialDirection { get; set; }
        public double Throttle { get; set; }
    }

    public sealed class PsgSolutionPoint
    {
        public double UniversalTime { get; set; }
        public int PhaseIndex { get; set; }
        public int KspStage { get; set; }
        public Vector3d RelativePosition { get; set; }
        public Vector3d RelativeVelocity { get; set; }
        public double MassKg { get; set; }
        public Vector3d InertialThrustDirection { get; set; }
        public double Throttle { get; set; }
    }

    public sealed class PsgSolutionSegment
    {
        public int PhaseIndex { get; set; }
        public int KspStage { get; set; }
        public double StartUniversalTime { get; set; }
        public double EndUniversalTime { get; set; }
        public bool IsCoast { get; set; }
        public bool AllowShutdown { get; set; }
        public bool PreciseShutdown { get; set; }
        public bool TerminalStage { get; set; }
    }

    public sealed class PsgSolution
    {
        public bool IsValid { get; set; }
        public string Status { get; set; }
        public double CreatedUniversalTime { get; set; }
        public double StartUniversalTime { get; set; }
        public double FinalUniversalTime { get; set; }
        public double TerminalAngularMomentum { get; set; }
        public double TerminalSpecificEnergy { get; set; }
        public int Iterations { get; set; }
        public int TerminationType { get; set; }
        public double ConstraintViolation { get; set; }
        public PsgSolutionPoint[] Points { get; set; }
        public PsgSolutionSegment[] Segments { get; set; }

        public double TimeToGo(double universalTime)
        {
            return Math.Max(0.0, FinalUniversalTime - universalTime);
        }

        public void ShiftStartUniversalTime(double universalTime)
        {
            double delta = universalTime - StartUniversalTime;
            if (Math.Abs(delta) <= 1e-9) return;

            StartUniversalTime += delta;
            CreatedUniversalTime += delta;
            FinalUniversalTime += delta;

            if (Points != null)
            {
                for (int i = 0; i < Points.Length; i++)
                {
                    Points[i].UniversalTime += delta;
                }
            }

            if (Segments != null)
            {
                for (int i = 0; i < Segments.Length; i++)
                {
                    Segments[i].StartUniversalTime += delta;
                    Segments[i].EndUniversalTime += delta;
                }
            }
        }

        public double VelocityToGo(double universalTime)
        {
            if (Points == null || Points.Length < 2) return double.NaN;

            double dv = 0.0;
            PsgSolutionPoint previous = GetPointAt(universalTime);

            for (int i = 0; i < Points.Length; i++)
            {
                if (Points[i].UniversalTime <= universalTime) continue;

                dv += (Points[i].RelativeVelocity - previous.RelativeVelocity).magnitude;
                previous = Points[i];
            }

            return dv;
        }

        public PsgGuidanceVector InertialGuidance(double universalTime)
        {
            PsgSolutionPoint point = GetPointAt(universalTime);
            if (point == null || point.InertialThrustDirection.sqrMagnitude <= 0.0)
            {
                return new PsgGuidanceVector { IsValid = false };
            }

            return new PsgGuidanceVector
            {
                IsValid = true,
                InertialDirection = point.InertialThrustDirection.normalized,
                Throttle = OrbitMath.Clamp(point.Throttle, 0.0, 1.0)
            };
        }

        public bool TerminalGuidanceSatisfied(Vector3d relativePosition, Vector3d relativeVelocity)
        {
            if (!IsValid || TerminalAngularMomentum <= 0.0) return false;

            return TerminalGuidanceSatisfied(relativePosition, relativeVelocity, FinalUniversalTime);
        }

        public bool TerminalGuidanceSatisfied(Vector3d relativePosition, Vector3d relativeVelocity, double universalTime)
        {
            if (!IsValid || TerminalAngularMomentum <= 0.0) return false;

            PsgSolutionPoint terminal = GetTerminalPointForTime(universalTime);
            if (terminal == null) return false;

            double targetAngularMomentum = Vector3d.Cross(terminal.RelativePosition, terminal.RelativeVelocity).magnitude;
            if (targetAngularMomentum <= 0.0) targetAngularMomentum = TerminalAngularMomentum;

            double currentAngularMomentum = Vector3d.Cross(relativePosition, relativeVelocity).magnitude;
            return currentAngularMomentum > targetAngularMomentum;
        }

        public PsgSolutionPoint TerminalState()
        {
            return Points != null && Points.Length > 0 ? Points[Points.Length - 1] : null;
        }

        public PsgSolutionPoint GetPointAtUniversalTime(double universalTime)
        {
            return GetPointAt(universalTime);
        }

        private PsgSolutionPoint GetTerminalPointForTime(double universalTime)
        {
            if (Segments == null || Segments.Length == 0) return TerminalState();

            int index = Segments.Length - 1;
            for (int i = 0; i < Segments.Length; i++)
            {
                if (universalTime < Segments[i].EndUniversalTime)
                {
                    index = i;
                    break;
                }
            }

            while (index > 0 && (!Segments[index].AllowShutdown || Segments[index].IsCoast))
            {
                index--;
            }

            while (index < Segments.Length - 1 &&
                   Segments[index].AllowShutdown &&
                   Segments[index + 1].AllowShutdown &&
                   !Segments[index + 1].IsCoast)
            {
                index++;
            }

            return GetPointAt(Segments[index].EndUniversalTime);
        }

        private PsgSolutionPoint GetPointAt(double universalTime)
        {
            if (Points == null || Points.Length == 0) return null;
            if (Points.Length == 1 || universalTime <= Points[0].UniversalTime) return Points[0];

            for (int i = 0; i < Points.Length - 1; i++)
            {
                PsgSolutionPoint a = Points[i];
                PsgSolutionPoint b = Points[i + 1];
                if (universalTime > b.UniversalTime) continue;

                double span = b.UniversalTime - a.UniversalTime;
                double t = span > 1e-9 ? OrbitMath.Clamp((universalTime - a.UniversalTime) / span, 0.0, 1.0) : 0.0;

                Vector3d direction = Lerp(a.InertialThrustDirection, b.InertialThrustDirection, t);
                return new PsgSolutionPoint
                {
                    UniversalTime = universalTime,
                    PhaseIndex = a.PhaseIndex,
                    KspStage = a.KspStage,
                    RelativePosition = Lerp(a.RelativePosition, b.RelativePosition, t),
                    RelativeVelocity = Lerp(a.RelativeVelocity, b.RelativeVelocity, t),
                    MassKg = a.MassKg + (b.MassKg - a.MassKg) * t,
                    InertialThrustDirection = direction.sqrMagnitude > 0.0 ? direction.normalized : a.InertialThrustDirection,
                    Throttle = a.Throttle + (b.Throttle - a.Throttle) * t
                };
            }

            return Points[Points.Length - 1];
        }

        private static Vector3d Lerp(Vector3d a, Vector3d b, double t)
        {
            return a + (b - a) * t;
        }
    }
}
