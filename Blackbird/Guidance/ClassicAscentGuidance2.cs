using System;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using Blackbird.Modules;

namespace Blackbird.Guidance
{
    public sealed class ClassicAscentGuidance2
    {
        private enum Phase { FollowProfile, Coast, Circularize, Complete }

        private Phase _phase = Phase.FollowProfile;

        // ThrottleToRaiseApoapsis state (mirrors MechJeb's rate-based proportional approach)
        private double _apLastAlt = double.NaN;
        private double _apLastUT = double.NegativeInfinity;
        private double _apLastThrottle;
        private readonly double[] _apRateSamples = new double[3];
        private int _apRateCount;

        // Live insertion target (replaces the old frozen circularize node), set each frame from the profile.
        private Vector3d _targetNormal = Vector3d.zero;
        private double _apoapsisRadius;
        private double _periapsisRadius;
        private double _circAtUt;                       // next-apoapsis UT, for centered ignition
        private Vector3d _insertionSteerDir = Vector3d.zero;
        private double _minVelToGo = double.MaxValue;   // for the overshoot cutoff
        private double _lastProgressUt = double.NegativeInfinity;

        private const double InsertionCutoffMps = 1.0; // "on the orbit" — cut here, don't chase to zero
        private const double InsertionOvershootMps = 0.5; // velToGo climbing back up = overshot
        private const double InsertionSteerLockMps = 5.0; // freeze steer dir below this (no end-game pivot)
        private const double InsertionProgressDeadband = 0.1;
        private const double InsertionStallSeconds = 2.0; // no progress this long (out of fuel) = cut

        public void Reset()
        {
            _phase = Phase.FollowProfile;
            _apLastAlt = double.NaN;
            _apLastUT = double.NegativeInfinity;
            _apLastThrottle = 0.0;
            _apRateCount = 0;
            _targetNormal = Vector3d.zero;
            _apoapsisRadius = 0.0;
            _periapsisRadius = 0.0;
            _circAtUt = 0.0;
            _insertionSteerDir = Vector3d.zero;
            _minVelToGo = double.MaxValue;
            _lastProgressUt = double.NegativeInfinity;
            for (int i = 0; i < _apRateSamples.Length; i++) _apRateSamples[i] = 0.0;
        }

        public PoweredGuidanceCommand GetCommand(
            VesselState vesselState,
            AscentProfile ascentProfile,
            double profilePitchDeg,
            double profileHeadingDeg,
            Vector3d targetOrbitNormal)
        {
            if (vesselState == null || ascentProfile == null || !ascentProfile.IsValid) return Unavailable();

            double targetApAlt = ascentProfile.TargetApoapsisAlt;
            _apoapsisRadius = vesselState.BodyRadius + ascentProfile.TargetApoapsisAlt;
            _periapsisRadius = vesselState.BodyRadius + ascentProfile.TargetPeriapsisAlt;
            if (_apoapsisRadius < _periapsisRadius)
            {
                double t = _apoapsisRadius; _apoapsisRadius = _periapsisRadius; _periapsisRadius = t;
            }
            _targetNormal = targetOrbitNormal;

            AdvancePhase(vesselState, targetApAlt);

            switch (_phase)
            {
                case Phase.Coast:
                    double coastThrottle = 0.0;
                    if (MathHelpers.IsFinite(vesselState.CurrentApoapsisAlt) && vesselState.CurrentApoapsisAlt < targetApAlt)
                        coastThrottle = ThrottleToRaiseApoapsis(vesselState, targetApAlt);
                    return Build(
                        PoweredGuidancePhase.Coast,
                        "Classic: coasting to apoapsis",
                        0.0, 0.0, coastThrottle,
                        true, HorizontalPrograde(vesselState), false);

                case Phase.Circularize:
                    // Closed-loop insertion: steer the live velocity-to-go to the target orbit's velocity at
                    // our current radius, full throttle. AdvancePhase cuts when we're on the orbit.
                    return Build(
                        PoweredGuidancePhase.Circularize,
                        "Classic: insertion burn",
                        0.0, 0.0, 1.0,
                        true, _insertionSteerDir, false);

                case Phase.Complete:
                    return Build(
                        PoweredGuidancePhase.Complete,
                        "Classic: orbit achieved",
                        profilePitchDeg, profileHeadingDeg, 0.0,
                        false, Vector3d.zero, true);

                default: // FollowProfile
                    {
                        double throttle = ThrottleToRaiseApoapsis(vesselState, targetApAlt);
                        bool isVertical = profilePitchDeg >= 80.0;
                        return Build(
                            isVertical ? PoweredGuidancePhase.VerticalAscent : PoweredGuidancePhase.PitchProgram,
                            isVertical ? "Classic: vertical ascent" : "Classic: pitch program",
                            profilePitchDeg, profileHeadingDeg, throttle,
                            false, Vector3d.zero, false);
                    }
            }
        }

        private void AdvancePhase(VesselState vs, double targetApAlt)
        {
            if (_phase == Phase.Complete) return;

            if (_phase == Phase.FollowProfile)
            {
                if (MathHelpers.IsFinite(vs.CurrentApoapsisAlt) && vs.CurrentApoapsisAlt >= targetApAlt)
                    _phase = Phase.Coast;
                return;
            }

            if (_phase == Phase.Coast)
            {
                Orbit orbit = vs.Vessel != null ? vs.Vessel.orbit : null;
                _circAtUt = orbit != null ? OrbitMath.TimeToNextApoapsis(orbit, Planetarium.GetUniversalTime()) : double.NaN;

                // Centered ignition: start half a burn before apoapsis so it straddles the apsis. Burn size
                // estimated from the live velocity-to-go (covers eccentric + plane-trim insertions).
                Vector3d targetVel = ComputeTargetVelocity(vs);
                Vector3d velToGo = targetVel - vs.OrbitalVelocity;
                double halfBurn = HalfBurnTime(vs, velToGo.magnitude);

                if (MathHelpers.IsFinite(_circAtUt) &&
                    Planetarium.GetUniversalTime() >= _circAtUt - halfBurn)
                {
                    _phase = Phase.Circularize;
                    _minVelToGo = double.MaxValue;
                    _lastProgressUt = Planetarium.GetUniversalTime();
                    _insertionSteerDir = velToGo.sqrMagnitude > 1e-12 ? velToGo.normalized : HorizontalPrograde(vs);
                    TimeWarp.SetRate(0, true); // fires once; we're now in Circularize
                }

                return;
            }

            if (_phase == Phase.Circularize)
            {
                Vector3d velToGo = ComputeTargetVelocity(vs) - vs.OrbitalVelocity;
                double mag = velToGo.magnitude;
                double now = Planetarium.GetUniversalTime();

                // Re-aim along velToGo while well-defined; freeze the direction for the final approach so the
                // craft can't pivot chasing a noise-dominated near-zero vector (the velToGo oscillation).
                if (mag > InsertionSteerLockMps || _insertionSteerDir.sqrMagnitude <= 0.0)
                    _insertionSteerDir = mag > 1e-6 ? velToGo / mag : _insertionSteerDir;

                if (mag < _minVelToGo - InsertionProgressDeadband)
                {
                    _minVelToGo = mag;
                    _lastProgressUt = now;
                }

                bool reached = mag <= InsertionCutoffMps;                     // on the target orbit
                bool overshot = mag >= _minVelToGo + InsertionOvershootMps;    // shot past it
                bool stalled = now - _lastProgressUt > InsertionStallSeconds; // no progress (out of fuel)
                if (reached || overshot || stalled) _phase = Phase.Complete;
            }
        }

        // The target orbit's inertial velocity at our CURRENT radius (world frame). Match it and we're on the
        // target orbit: vis-viva sets speed (energy/SMA), flight-path angle sets the radial split (angular
        // momentum -> eccentricity), target normal sets the plane. Circular is just the phi -> 0 case.
        private Vector3d ComputeTargetVelocity(VesselState vs)
        {
            double mu = vs.BodyGravParameter;
            if (vs.Body == null || !MathHelpers.IsFinite(mu) || mu <= 0.0) return vs.OrbitalVelocity;

            Vector3d rVec = vs.Position - vs.Body.position;
            double r = rVec.magnitude;
            if (r <= 0.0 || _apoapsisRadius <= 0.0 || _periapsisRadius <= 0.0) return vs.OrbitalVelocity;

            Vector3d up = rVec / r;
            Vector3d v = vs.OrbitalVelocity;

            // Target plane: the target orbit normal, or the current plane when none is given (hold plane).
            Vector3d normal = _targetNormal.sqrMagnitude > 0.0 ? _targetNormal : Vector3d.Cross(rVec, v);
            if (normal.sqrMagnitude <= 0.0) return v;
            normal = normal.normalized;

            double a = 0.5 * (_apoapsisRadius + _periapsisRadius);
            double p = 2.0 * _apoapsisRadius * _periapsisRadius / (_apoapsisRadius + _periapsisRadius);
            double speed = Math.Sqrt(Math.Max(0.0, mu * (2.0 / r - 1.0 / a)));
            double h = Math.Sqrt(Math.Max(0.0, mu * p));
            if (speed <= 0.0) return v;

            double cosPhi = MathHelpers.Clamp(h / (r * speed), -1.0, 1.0);
            double phi = Math.Acos(cosPhi);
            double radialSign = Vector3d.Dot(up, v) >= 0.0 ? 1.0 : -1.0; // climbing toward Ap = +, ~0 at apsis

            Vector3d horizontal = Vector3d.Cross(normal, up); // prograde-horizontal, in the target plane
            if (horizontal.sqrMagnitude <= 0.0) return v;     // at the pole; nothing sane to do
            horizontal = horizontal.normalized;

            return speed * (Math.Cos(phi) * horizontal + Math.Sin(phi) * radialSign * up);
        }

        // Rate-based proportional throttle that soft-lands apoapsis onto the target (MechJeb-parity).
        private double ThrottleToRaiseApoapsis(VesselState vs, double targetAp)
        {
            double currentAp = vs.CurrentApoapsisAlt;
            double now = vs.UniversalTime;

            if (!MathHelpers.IsFinite(currentAp))
            {
                RecordApSample(currentAp, now, 1.0);
                return 1.0;
            }

            if (currentAp > targetAp + 5.0)
            {
                RecordApSample(currentAp, now, 0.0);
                return 0.0;
            }

            double throttle;
            double elapsed = now - _apLastUT;
            if (MathHelpers.IsFinite(_apLastAlt) && elapsed > 0.0 && _apLastThrottle > 0.0)
            {
                double instantRate = (currentAp - _apLastAlt) / (elapsed * _apLastThrottle);
                instantRate = Math.Max(1.0, instantRate);

                _apRateSamples[_apRateCount % 3] = instantRate;
                _apRateCount++;
                double avgRate = AverageApRate();

                double desiredRate = targetAp - currentAp;
                throttle = MathHelpers.Clamp(desiredRate / avgRate, 0.05, 1.0);
            }
            else
            {
                throttle = 1.0;
            }

            RecordApSample(currentAp, now, throttle);
            return throttle;
        }

        private void RecordApSample(double alt, double ut, double throttle)
        {
            _apLastAlt = alt;
            _apLastUT = ut;
            _apLastThrottle = throttle;
        }

        private double AverageApRate()
        {
            int count = Math.Min(_apRateCount, 3);
            if (count == 0) return 1.0;
            double sum = 0.0;
            for (int i = 0; i < count; i++) sum += _apRateSamples[i];
            return sum / count;
        }

        // Horizontal (level prograde) component of orbital velocity, world frame.
        private static Vector3d HorizontalPrograde(VesselState vs)
        {
            if (vs == null || vs.Body == null) return Vector3d.zero;
            Vector3d up = (vs.Position - vs.Body.position).normalized;
            Vector3d horiz = Vector3d.Exclude(up, vs.OrbitalVelocity);
            return horiz.sqrMagnitude > 1e-6 ? horiz.normalized : Vector3d.zero;
        }

        // Tsiolkovsky time to deliver the first half of a burn's dV (rising TWR over the half). Centers the burn.
        private static double HalfBurnTime(VesselState vs, double dvTotal)
        {
            if (vs.TotalMass <= 0.0 || vs.AvailableThrust <= 0.0 || vs.VacuumSpecificImpulse <= 0.0)
                return 0.0;
            if (!MathHelpers.IsFinite(dvTotal) || dvTotal <= 0.0) return 0.0;

            double vE = vs.VacuumSpecificImpulse * PsgPhase.StandardGravity;
            double halfDv = dvTotal * 0.5;
            return (vE * vs.TotalMass / vs.AvailableThrust) * (1.0 - Math.Exp(-halfDv / vE));
        }

        private static PoweredGuidanceCommand Build(
            PoweredGuidancePhase phase,
            string status,
            double pitchDeg,
            double headingDeg,
            double throttle,
            bool hasInertialDirection,
            Vector3d inertialDirection,
            bool isComplete)
        {
            Vector3d direction = hasInertialDirection && inertialDirection.sqrMagnitude > 0.0
                ? inertialDirection.normalized
                : Vector3d.zero;

            return new PoweredGuidanceCommand
            {
                Phase = phase,
                Status = status,
                PitchDeg = pitchDeg,
                HeadingDeg = MathHelpers.NormalizeDegrees(headingDeg),
                Throttle = MathHelpers.Clamp(throttle, 0.0, 1.0),
                HasInertialDirection = hasInertialDirection && direction.sqrMagnitude > 0.0,
                InertialDirection = direction,
                ApoapsisErrorMeters = double.NaN,
                PeriapsisErrorMeters = double.NaN,
                TimeToGoSeconds = double.NaN,
                VelocityToGoMetersPerSecond = double.NaN,
                OptimizerStatus = string.Empty,
                OptimizerIterations = 0,
                SolutionConstraintViolation = double.NaN,
                IsComplete = isComplete
            };
        }

        private static PoweredGuidanceCommand Unavailable() =>
            Build(PoweredGuidancePhase.Unavailable, "Classic: unavailable",
                  90.0, 90.0, 0.0, false, Vector3d.zero, false);
    }
}