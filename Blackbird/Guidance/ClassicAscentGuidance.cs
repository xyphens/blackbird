using System;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class ClassicAscentGuidance
    {
        private enum Phase { FollowProfile, Coast, Circularize, Complete }

        private Phase _phase = Phase.FollowProfile;

        // ThrottleToRaiseApoapsis state (mirrors MechJeb's rate-based proportional approach)
        private double _apLastAlt    = double.NaN;
        private double _apLastUT     = double.NegativeInfinity;
        private double _apLastThrottle;
        private readonly double[] _apRateSamples = new double[3];
        private int    _apRateCount;

        // Circularization "node" planned during Coast and then executed like a maneuver node: world-frame
        // dV (MechJeb convention — already a world-space steering vector) at the next-apoapsis UT.
        private Vector3d _dvToCircularize = Vector3d.zero;   // world frame
        private double _circAtUt = 0.0;                      // absolute UT of the node (apoapsis)
        private Vector3d _burnStartVelocity = Vector3d.zero; // orbital velocity at ignition, for delivered-dV cutoff

        public void Reset()
        {
            _phase             = Phase.FollowProfile;
            _apLastAlt         = double.NaN;
            _apLastUT          = double.NegativeInfinity;
            _apLastThrottle    = 0.0;
            _apRateCount       = 0;
            _dvToCircularize   = Vector3d.zero;
            _circAtUt          = 0.0;
            _burnStartVelocity = Vector3d.zero;
            for (int i = 0; i < _apRateSamples.Length; i++) _apRateSamples[i] = 0.0;
        }

        public PoweredGuidanceCommand GetCommand(
            VesselState vesselState,
            AscentProfile ascentProfile,
            double profilePitchDeg,
            double profileHeadingDeg,
            double profileInclination
            )
        {
            if (vesselState == null || ascentProfile == null || !ascentProfile.IsValid) return Unavailable();

            double targetAp = ascentProfile.TargetApoapsisAlt;
            double targetInc = profileInclination;

            AdvancePhase(vesselState, targetAp, targetInc);

            //double peRemaining = targetPe - vesselState.CurrentPeriapsisAlt;

            switch (_phase)
            {
                case Phase.Coast:
                    // Re-ignite if atmosphere drag has pulled Ap below target.
                    // Pre-orient to horizontal prograde so the circularize burn starts already aligned
                    // (no slew at the critical apoapsis-crossing moment).
                    double coastThrottle = 0.0;
                    if (OrbitMath.IsFinite(vesselState.CurrentApoapsisAlt) && vesselState.CurrentApoapsisAlt < targetAp)
                        coastThrottle = ThrottleToRaiseApoapsis(vesselState, targetAp);
                    return Build(
                        PoweredGuidancePhase.Coast,
                        "Classic: coasting to apoapsis",
                        0.0, 0.0, coastThrottle,
                        true, HorizontalPrograde(vesselState), false);

                case Phase.Circularize:
                    // Execute the planned node: hold attitude along its world-frame dV vector at full
                    // throttle. AdvancePhase cuts us off once the dV delivered along this vector reaches
                    // the planned magnitude.
                    return Build(
                        PoweredGuidancePhase.Circularize,
                        "Classic: circularizing",
                        0.0, 0.0, 1.0,
                        true, _dvToCircularize, false);

                case Phase.Complete:
                    return Build(
                        PoweredGuidancePhase.Complete,
                        "Classic: orbit achieved",
                        profilePitchDeg, profileHeadingDeg, 0.0,
                        false, Vector3d.zero, true);

                default: // FollowProfile
                {
                    double throttle = ThrottleToRaiseApoapsis(vesselState, targetAp);
                    bool isVertical = profilePitchDeg >= 80.0;
                    return Build(
                        isVertical ? PoweredGuidancePhase.VerticalAscent : PoweredGuidancePhase.PitchProgram,
                        isVertical ? "Classic: vertical ascent" : "Classic: pitch program",
                        profilePitchDeg, profileHeadingDeg, throttle,
                        false, Vector3d.zero, false);
                }
            }
        }

        private void AdvancePhase(VesselState vs, double targetAp, double targetInclination)
        {
            if (_phase == Phase.Complete) return;

            if (_phase == Phase.FollowProfile)
            {
                if (OrbitMath.IsFinite(vs.CurrentApoapsisAlt) && vs.CurrentApoapsisAlt >= targetAp)
                    _phase = Phase.Coast;
                return;
            }

            if (_phase == Phase.Coast)
            {
                // Re-plan the node every frame while coasting (cheap, keeps it current as Ap is trimmed).
                var node = PlanCircularizationNode(vs, targetInclination);
                _circAtUt        = node.ut;
                _dvToCircularize = node.dvWorld;

                // Ignite half a burn-time before the node so the burn straddles apoapsis (node-executor
                // centering: ignitionUT = node.UT - halfBurnTime), instead of starting the whole burn at
                // the node and overshooting.
                double halfBurn = HalfBurnTime(vs, _dvToCircularize.magnitude);
                if (OrbitMath.IsFinite(_circAtUt) &&
                    Planetarium.GetUniversalTime() >= _circAtUt - halfBurn)
                {
                    _phase = Phase.Circularize;
                    _burnStartVelocity = vs.OrbitalVelocity; // baseline for delivered-dV cutoff
                    // Cancel warp so we don't overshoot (fires once; we're now in Circularize).
                    TimeWarp.SetRate(0, true);
                }

                return;
            }

            // Terminate when the velocity change delivered ALONG the node's burn vector reaches the
            // planned magnitude — i.e. we've executed the node. Monotonic (thrust along the burn axis
            // only adds to it) so it can't stall, and it handles a combined plane-change + circularize
            // burn, where horizontal speed alone wouldn't (a plane change rotates velocity without
            // raising its horizontal magnitude).
            if (_phase == Phase.Circularize)
            {
                double planned = _dvToCircularize.magnitude;
                if (planned <= 1e-3)
                {
                    _phase = Phase.Complete;
                    return;
                }
                double delivered = Vector3d.Dot(vs.OrbitalVelocity - _burnStartVelocity,
                                                _dvToCircularize / planned);
                if (delivered >= planned)
                    _phase = Phase.Complete;
            }
        }

        // Rate-based proportional throttle that soft-lands apoapsis onto the target.
        // Mirrors MechJeb's ThrottleToRaiseApoapsis: tracks dAp/dt per unit throttle
        // using a 3-sample moving average, then solves for the throttle that closes
        // the remaining gap in 1 second.
        private double ThrottleToRaiseApoapsis(VesselState vs, double targetAp)
        {
            double currentAp = vs.CurrentApoapsisAlt;
            double now       = vs.UniversalTime;

            if (!OrbitMath.IsFinite(currentAp))
            {
                RecordApSample(currentAp, now, 1.0);
                return 1.0;
            }

            // Hard cutoff — already past target.
            if (currentAp > targetAp + 5.0)
            {
                RecordApSample(currentAp, now, 0.0);
                return 0.0;
            }

            double throttle;

            double elapsed = now - _apLastUT;
            if (OrbitMath.IsFinite(_apLastAlt) && elapsed > 0.0 && _apLastThrottle > 0.0)
            {
                double instantRate = (currentAp - _apLastAlt) / (elapsed * _apLastThrottle);
                instantRate = Math.Max(1.0, instantRate); // guard against negative/zero

                // 3-sample ring-buffer average
                _apRateSamples[_apRateCount % 3] = instantRate;
                _apRateCount++;
                double avgRate = AverageApRate();

                double desiredRate = targetAp - currentAp; // close the gap in 1 second
                throttle = OrbitMath.Clamp(desiredRate / avgRate, 0.05, 1.0);
            }
            else
            {
                throttle = 1.0; // no recent data — full throttle
            }

            RecordApSample(currentAp, now, throttle);
            return throttle;
        }

        private void RecordApSample(double alt, double ut, double throttle)
        {
            _apLastAlt      = alt;
            _apLastUT       = ut;
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

        // Horizontal component of orbital velocity (radial/up component removed), in world frame.
        // This is the level prograde direction used for the circularization burn.
        private static Vector3d HorizontalPrograde(VesselState vs)
        {
            if (vs == null || vs.Body == null) return Vector3d.zero;
            Vector3d up = (vs.Position - vs.Body.position).normalized;
            Vector3d horiz = Vector3d.Exclude(up, vs.OrbitalVelocity);
            return horiz.sqrMagnitude > 1e-6 ? horiz.normalized : Vector3d.zero;
        }

        // Plans the circularization "node": absolute UT of the next apoapsis and the dV to burn there,
        // computed MechJeb-style (OrbitalManeuverCalculator). All dV come back in the SwapYZ world frame,
        // which is already a world-space steering vector — no further conversion. We execute it directly
        // (no patched-conic node), so it works under Principia too.
        private static (double ut, Vector3d dvWorld) PlanCircularizationNode(VesselState vs, double targetInclination)
        {
            Orbit orbit = vs.Vessel != null ? vs.Vessel.orbit : null;
            if (orbit == null) return (double.NaN, Vector3d.zero);

            double ut = OrbitMath.TimeToNextApoapsis(orbit, Planetarium.GetUniversalTime()); // absolute UT
            if (!OrbitMath.IsFinite(ut) || ut <= 0.0) return (double.NaN, Vector3d.zero);

            // Plane change DISABLED for now. The frame fix made the circularization dV correct (the burn
            // is horizontal), but DeltaVToChangeInclination is still rotating the orbit to polar — even
            // when asked to hold the current plane. So do a pure prograde circularization, which preserves
            // whatever plane we launched into. The machinery stays in OrbitMath; re-enable the two lines
            // below (guarded on a real targetInclination) once it's validated against a known plane change.

            //   Vector3d incCorrection = OrbitMath.DeltaVToChangeInclination(orbit, ut, OrbitMath.Deg2Rad(targetInclination));
            //   circCorrection = OrbitMath.DeltaVToCircularize(OrbitMath.PerturbedOrbit(orbit, ut, incCorrection), ut);

            Vector3d incCorrection = Vector3d.zero;
            Vector3d circCorrection = OrbitMath.DeltaVToCircularize(orbit, ut);
            Vector3d dvWorld = incCorrection + circCorrection;

            // VALIDATION: at apoapsis a coplanar circularization dV is purely prograde, so its angle to
            // the velocity AT the node should be ~0 (independent of coast progress — a measurement-frame
            // artifact would vary, a frame bug would be a large constant). incCorrection should be ~0
            // with no target.
            Vector3d velAtAp = orbit.getOrbitalVelocityAtUT(ut).xzy;
            // Debug.Log($"[CIRC NODE] dv|{dvWorld.magnitude:F1}| inc|{incCorrection.magnitude:F1}| " +
            //           $"angleFromPrograde={Vector3d.Angle(dvWorld, velAtAp):F1}deg");

            return (ut, dvWorld);
        }

        // Tsiolkovsky time to deliver the first half of a burn's dV, accounting for the mass the
        // vessel sheds (rising TWR) over that half. Used to center the burn on the node.
        private static double HalfBurnTime(VesselState vs, double dvTotal)
        {
            if (vs.TotalMass <= 0.0 || vs.AvailableThrust <= 0.0 || vs.VacuumSpecificImpulse <= 0.0)
                return 0.0;
            if (!OrbitMath.IsFinite(dvTotal) || dvTotal <= 0.0) return 0.0;

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
                Phase                    = phase,
                Status                   = status,
                PitchDeg                 = pitchDeg,
                HeadingDeg               = OrbitMath.NormalizeDegrees(headingDeg),
                Throttle                 = OrbitMath.Clamp(throttle, 0.0, 1.0),
                HasInertialDirection     = hasInertialDirection && direction.sqrMagnitude > 0.0,
                InertialDirection        = direction,
                ApoapsisErrorMeters      = double.NaN,
                PeriapsisErrorMeters     = double.NaN,
                TimeToGoSeconds          = double.NaN,
                VelocityToGoMetersPerSecond = double.NaN,
                OptimizerStatus          = string.Empty,
                OptimizerIterations      = 0,
                SolutionConstraintViolation = double.NaN,
                IsComplete               = isComplete
            };
        }

        private static PoweredGuidanceCommand Unavailable() =>
            Build(PoweredGuidancePhase.Unavailable, "Classic: unavailable",
                  90.0, 90.0, 0.0, false, Vector3d.zero, false);
    }
}
