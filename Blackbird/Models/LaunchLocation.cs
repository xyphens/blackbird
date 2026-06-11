using Blackbird;

namespace Blackbird.Models
{
    public sealed class LaunchLocation
    {
        public double LatitudeDeg { get; set; }
        public double LongitudeDeg { get; set; }
        public double BodyRadius { get; set; }
        public double RotationPeriod { get; set; }
        public string Name { get; set; }
        public bool IsValid
        {
            get
            {
                return LatitudeDeg >= -90.0 && LatitudeDeg <= 90.0 && LongitudeDeg >= -180.0 && LongitudeDeg <= 180.0;
            }
        }

        public static LaunchLocation FromVessel(Vessel vessel)
        {
            if (vessel == null) return null;

            return new LaunchLocation
            {
                Name = vessel.mainBody.bodyName + " Current Position",
                LatitudeDeg = vessel.latitude,
                LongitudeDeg = vessel.longitude,
                RotationPeriod = vessel.mainBody.rotationPeriod,
                BodyRadius = vessel.mainBody.Radius,
            };
        }
    }
}
