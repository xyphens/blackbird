using System.Collections.Generic;

namespace Blackbird.OpenLoop
{
    public struct OpenLoopSample
    {
        public double TimeSec;
        public double AltMeters;
        public double DownrangeMeters;
        public double SurfaceSpeedMps;
        public double PitchDeg;     // above horizon
        public double MassKg;
    }

    public struct OpenLoopCandidate
    {
        public bool Valid;
        public string Reason;
        public double RateDegPerSec;
        public double InjectedMassKg;
        public double TimeToHandoffSeconds;
        public double PsgTimeToGoSeconds;
        public List<OpenLoopSample> Path;
    }
}
