using System;
using Blackbird;

namespace Blackbird.Enums
{
    public static class PlanetScale
    {
        public enum PlanetScaleEnum
        {
            Stock,
            RSS
        }
        public static PlanetScaleEnum GetScale()
        {
            return FlightGlobals.currentMainBody.Radius > 1_000_000
                ? PlanetScaleEnum.RSS
                : PlanetScaleEnum.Stock;
        }
    }
}
