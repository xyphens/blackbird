using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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
    }
}
