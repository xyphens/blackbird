using System;
using Blackbird.Mathematics;

namespace Blackbird.Models
{
    public sealed class LaunchWindowInfo
    {
        public double PlaneErrorDeg { get; set; }

        public double TimeToPlaneCrossingSeconds { get; set; }

        public string NodeName { get; set; }

        public double AscendingNodeLongitudeDeg { get; set; }
        public double DescendingNodeLongitudeDeg { get; set; }

        public double TimeToAscendingNodeSeconds { get; set; }
        public double TimeToDescendingNodeSeconds { get; set; }

        public double PlaneOffsetDeg { get; set; }
        public bool UseAscendingNode { get; set; }
        public bool IsLaunchWindowValid { get; set; }
        public string LaunchCountdownText { get; set; }
        public double AscendingAzimuthDeg { get; set; }
        public double DescendingAzimuthDeg { get; set; }
        public double SelectedAzimuthDeg { get; set; }

        public static LaunchWindowInfo Create(
            Vessel active,
            OrbitInfo targetOrbit,
            LaunchLocation launchLocation)
        {
            if (active == null || targetOrbit == null || launchLocation == null)
            {
                return new LaunchWindowInfo
                {
                    PlaneErrorDeg = 0.0,
                    TimeToPlaneCrossingSeconds = 0.0,
                    NodeName = "Unknown",
                    IsLaunchWindowValid = false
                };
            }

            double currentUt = Planetarium.GetUniversalTime();

            double targetAscInertialLong = targetOrbit.LanDeg;
            double targetDescInertialLong = targetOrbit.LanDeg + 180.0;

            double bodyRotationPeriod = active.mainBody.rotationPeriod;

            double targetAscBodyFixedLong =

                OrbitMath.GetBodyFixedLongitudeAtTime(
                    targetAscInertialLong,
                    currentUt,
                    bodyRotationPeriod);

            double targetDescBodyFixedLong =
                OrbitMath.GetBodyFixedLongitudeAtTime(
                    targetDescInertialLong,
                    currentUt,
                    bodyRotationPeriod);

            double launchLongitude =
                OrbitMath.NormalizeDegrees(launchLocation.LongitudeDeg);

            double timeToAsc =
                OrbitMath.TimeToLongitudeSeconds(
                    launchLongitude,
                    targetAscBodyFixedLong,
                    bodyRotationPeriod);

            double timeToDesc =
                OrbitMath.TimeToLongitudeSeconds(
                    launchLongitude,
                    targetDescBodyFixedLong,
                    bodyRotationPeriod);

            double ascAzimuth =
                OrbitMath.GetLaunchAzimuth(
                    targetOrbit.InclinationDeg,
                    launchLocation.LatitudeDeg);

            double descAzimuth = double.IsNaN(ascAzimuth) ? double.NaN : OrbitMath.NormalizeDegrees(360.0 - ascAzimuth);

            bool useAscending = timeToAsc <= timeToDesc;

            return new LaunchWindowInfo
            {
                AscendingAzimuthDeg = ascAzimuth,
                DescendingAzimuthDeg = descAzimuth,
                SelectedAzimuthDeg = useAscending ? ascAzimuth : descAzimuth,

                AscendingNodeLongitudeDeg = targetAscBodyFixedLong,
                DescendingNodeLongitudeDeg = targetDescBodyFixedLong,

                TimeToAscendingNodeSeconds = timeToAsc,
                TimeToDescendingNodeSeconds = timeToDesc,

                PlaneOffsetDeg = useAscending
                    ? OrbitMath.DeltaDegrees(launchLongitude, targetAscBodyFixedLong)
                    : OrbitMath.DeltaDegrees(launchLongitude, targetDescBodyFixedLong),

                TimeToPlaneCrossingSeconds = useAscending
                    ? timeToAsc
                    : timeToDesc,

                NodeName = useAscending ? "Ascending" : "Descending",
                UseAscendingNode = useAscending,
                IsLaunchWindowValid = true
            };
        }
    }
}
