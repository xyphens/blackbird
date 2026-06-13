using System;
using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
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
        public Vector3d InertialThrustDirection { get; set; }
        public double Throttle { get; set; }
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

        public double TimeToGo(double universalTime)
        {
            return Math.Max(0.0, FinalUniversalTime - universalTime);
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

            // Matches MechJeb's terminal cutoff shape: stop when the predicted angular momentum
            // crosses the selected terminal target. The parity gap is target construction, not this comparison.
            double currentAngularMomentum = Vector3d.Cross(relativePosition, relativeVelocity).magnitude;
            return currentAngularMomentum >= TerminalAngularMomentum;
        }

        public PsgSolutionPoint TerminalState()
        {
            return Points != null && Points.Length > 0 ? Points[Points.Length - 1] : null;
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
