using System.Collections.Generic;
using Blackbird.Psg;

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
        public int PsgIterations;
        public PsgSolution PsgSolution;
        public List<OpenLoopSample> Path;
        public OpenLoopFailure Failure;
    }

    public enum OpenLoopFailure
    {
        None = 0,
        NoStages,
        PitchedPastHorizontal,
        GroundImpact,
        NeverReachedHandoff,
        NoStagesAtHandoff,
        PsgProblemInvalid,
        PsgNotConverged,
        PropellantExceeded,
        Exception
    }

    public static class OpenLoopFailureText
    {
        public static string Describe(OpenLoopFailure failure)
        {
            switch (failure)
            {
                case OpenLoopFailure.None: return string.Empty;
                case OpenLoopFailure.NoStages: return "no stages";
                case OpenLoopFailure.PitchedPastHorizontal: return "pitched past horizontal";
                case OpenLoopFailure.GroundImpact: return "ground impact";
                case OpenLoopFailure.NeverReachedHandoff: return "never reached handoff altitude";
                case OpenLoopFailure.NoStagesAtHandoff: return "no remaining stages at handoff";
                case OpenLoopFailure.PsgProblemInvalid: return "PSG problem invalid";
                case OpenLoopFailure.PsgNotConverged: return "PSG did not converge";
                case OpenLoopFailure.PropellantExceeded: return "PSG solution exceeds available propellant";
                case OpenLoopFailure.Exception: return "exception";
                default: return failure.ToString();
            }
        }
    }
}
