using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.Guidance
{
    // holds vectors for our ascent gradient
    public sealed class PitchProfilePoint
    {
        public double AltitudeMeters { get; set; }
        public double PitchDegrees { get; set; }
    }
}