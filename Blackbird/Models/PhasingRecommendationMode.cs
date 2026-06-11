using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.Models
{
    public enum PhasingRecommendationMode
    {
        // avoids dumb or extreme orbits
        Balanced,
        // prefer fewer orbits and shorter warps
        // allows more aggressive altitude offsets
        Fastest,
        // prefer orbit closer to target orbit
        // allows longer rendezvous time
        Efficient
    }
}
