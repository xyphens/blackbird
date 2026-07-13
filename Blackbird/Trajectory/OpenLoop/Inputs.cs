using Blackbird.Models;
using Blackbird.Psg;
using System;

namespace Blackbird.OpenLoop
{
    // Everything the builder needs, as plain values/delegates so it runs identically in-game and offline.
    // In-game the caller fills this from VesselState + LaunchPlan; the harness fills it synthetically.
    public sealed class OpenLoopInputs
    {
        public double Mu;
        public double BodyRadiusMeters;
        public double DragAreaCd;                        // Cd*A m^2
        public Func<double, double> DensityAtAltitude;   // altitude m -> kg/m^3
        public PoweredStageInfo[] Stages;                // burn order; feeds BOTH the integrator and PSG
        public double LiftoffMassKg;
        public double PadAltitudeMeters;
        public double UniversalTime;

        public Vector3d PadRelativePosition;             // body-centered pad position (world axes)
        public Vector3d DownrangeDirection;              // launch-heading horizontal unit vector
        public Vector3d BodyAngularVelocity;             // rad/s vector (pole * 2pi/period)

        public double HandoffAltitudeMeters;             // from the pressure-fraction sampler (kPa Fraction input)
        public double PitchOverSpeedMps;                 // existing _minVrfSpeedToPitch (100)
        public double HoldVerticalUntilAltMeters;        // existing _holdPitchUntilAlt

        public PsgTarget Target;                         // built by caller exactly as flight PSG builds it
    }
}
