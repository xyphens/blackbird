using Blackbird.Enums;
using UnityEngine;

namespace Blackbird.Models
{
    public sealed class PoweredGuidanceCommand
    {
        public PoweredGuidancePhase Phase { get; set; }
        public string Status { get; set; }

        public double PitchDeg { get; set; }
        public double HeadingDeg { get; set; }
        public double Throttle { get; set; }
        public bool HasInertialDirection { get; set; }
        public Vector3d InertialDirection { get; set; }

        public double ApoapsisErrorMeters { get; set; }
        public double PeriapsisErrorMeters { get; set; }
        public double TimeToGoSeconds { get; set; }
        public double VelocityToGoMetersPerSecond { get; set; }
        public double SolutionConstraintViolation { get; set; }
        public int OptimizerIterations { get; set; }
        public string OptimizerStatus { get; set; }

        public bool IsComplete { get; set; }
    }
}
