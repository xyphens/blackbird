using Blackbird.Enums;

namespace Blackbird.Models
{
    public sealed class PoweredGuidanceCommand
    {
        public PoweredGuidancePhase Phase { get; set; }
        public string Status { get; set; }

        public double PitchDeg { get; set; }
        public double HeadingDeg { get; set; }
        public double Throttle { get; set; }

        public double ApoapsisErrorMeters { get; set; }
        public double PeriapsisErrorMeters { get; set; }
        public double TimeToGoSeconds { get; set; }
        public double VelocityToGoMetersPerSecond { get; set; }

        public bool IsComplete { get; set; }
    }
}
