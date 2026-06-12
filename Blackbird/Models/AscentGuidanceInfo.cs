using System;
using Blackbird.Enums;

namespace Blackbird.Models
{
    public sealed class AscentGuidanceInfo
    {
        public double TargetAzimuthDeg { get; set; }
        public double TargetLanDeg { get; set; }
        public double CurrentLanDeg { get; set; }
        public double LanErrorDeg { get; set; }
        public string PitchInstruction { get; set; }
        public string HeadingInstruction { get; set; }
        public double TargetPitchDeg { get; set; }
        public double CurrentPitchDeg { get; set; }
        public double PitchErrorDeg { get; set; }
        public double CommandPitchDeg { get; set; }
        public double CommandHeadingDeg { get; set; }
        public GuidanceMode GuidanceMode { get; set; }
        public double CurrentHeadingDeg { get; set; }
        public double HeadingErrorDeg { get; set; }
    }
}
