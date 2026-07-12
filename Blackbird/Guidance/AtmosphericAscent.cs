using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;
using Blackbird.Logging;
using UnityEngine;

namespace Blackbird.Guidance
{
    // pre-PSG guidance ascent law
    // kick is solved once at launch to a target read from PSG's own solution and a conservative margin
    // then a 3-phase law flies it (vertical hold -> kick + surface pg -> slew to PSG in vac
    // fallback to caller's ramp until kick solves so we never regress guidance
    public sealed class AtmosphericAscent
    {
        public enum Phase { ProgradeAtmospheric, ClosedLoopVacuum };

        private const double ConservatismMarginDeg = 10.0; // hold handover this many degrees more vertical than PSG
        private const double HandoverPressureFraction = 0.001; // vacuum boundary
        private const double DynamicPressureShiftPa = 2000.0; // surface- -> orbit-prograde shift used by solve
        private const double DragThrustRatioTrigger = 0.001; // aero force negligible vs thrust -> hand to PSG
        private const double HandoverQKpa = 1.0;  // safe kPa to had over to PSG
        private const double MaxSlewDegPerSec = 5.0;
        private const double MaxKickDeg = 30.0;
        private const double SolveStepSeconds = 0.5;
        private const int SolveMaxSteps = 4000;

        private static readonly BlackbirdLog Log = new BlackbirdLog(LogContext.Trajectory);

        private Phase _phase = Phase.ProgradeAtmospheric;
        private bool _kickSolved;
        private double _kickDeg;
        private double _lastQKpa = double.NaN;
        private double _lastUt = double.NaN;
        private Vector3d _slewDir = Vector3d.zero;

        public bool KickSolved => _kickSolved;
        public double KickAngleDeg => _kickDeg;
        public Phase CurrentPhase => _phase;

        public void Reset()
        {
            _phase = Phase.ProgradeAtmospheric;
            _kickSolved = false;
            _kickDeg = 0.0;
            _lastQKpa = double.NaN;
            _lastUt = double.NaN;
            _slewDir = Vector3d.zero;
        }

        public struct Command
        {
            public double PitchDeg; // used when !HasIntertialDirection
            public bool HasInertialDirection;
            public Vector3d InertialDirection; // slew-limited PSG vector (vacuum)
        }

        // solve the kick one PSG has a solution
        public bool TrySolveKick(VesselState vs, PsgSolution psg)
        {
            if (_kickSolved) return true;
            if (vs == null || vs.Body == null || !vs.Body.atmosphere) return false;
            if (psg == null || !psg.IsValid || psg.Points == null || psg.Points.Length < 2) return false;

            try
            {
                double handover = HandoverAltitude(vs.Body);
                if (!MathHelpers.IsFinite(handover) || handover <= vs.AltitudeMeters) return false;

                double psgFpa = PsgExpectedOrbitFpaDeg(psg, vs.BodyRadius, handover);
                if (!MathHelpers.IsFinite(psgFpa)) return false;
                double targetFpa = psgFpa + ConservatismMarginDeg; // hold more vertical than PSG's flat optimum

                AscentShootingInputs io = BuildInputs(vs, handover, targetFpa);
                if (io == null) return false;

                AscentShootingResult r = AscentShootingSolver.Solve(io);
                if (!r.Converged) return false;

                _kickDeg = r.KickAngleDeg;
                _kickSolved = true;
                Log.Write(string.Format("[kick-ascent] kick={0:F2} deg | PSG FPA {1:F1} + {2:F0} margin = {3:F1} | handover {4:F1} km",
                        _kickDeg, psgFpa, ConservatismMarginDeg, targetFpa, handover / 1000.0));
                return true;
            }
            catch (Exception ex) {
                Log.Write("[kick-ascent] solve failed, using ramp: " + ex.Message);
                return false;
            }
        }

        public Command Update(VesselState vs, bool psgReady, Vector3d psgIntertialDirection)
        {
            double dt = MathHelpers.IsFinite(_lastUt) ? vs.UniversalTime - _lastUt : 0.0;
            _lastUt = vs.UniversalTime;

            double qKpa = vs.DynamicPressureKpa;
            double dQdt = MathHelpers.IsFinite(_lastQKpa) && dt > 0.0 ? (qKpa - _lastQKpa) / dt : 0.0;
            _lastQKpa = qKpa;

            if (_phase == Phase.ProgradeAtmospheric)
            {
                if (psgReady && AeroNegligible(vs, qKpa, dQdt))
                {
                    _phase = Phase.ClosedLoopVacuum;
                    _slewDir = NoseDirection(vs); // begin slew from current altitude
                    Log.Write(string.Format("[kick-ascent] PSG handoff at {0:F1} km", vs.AltitudeMeters / 1000.0));
                } else
                {
                    double surfaceFpa = FlightPathAngleDeg(vs.VerticalSpeed, vs.SurfaceSpeed);
                    double pitch = Math.Min(90.0 - _kickDeg, surfaceFpa); // kick held until velocity catches up, then prograde
                    return new Command
                    {
                        PitchDeg = pitch,
                        HasInertialDirection = false
                    };
                }
            }

            // close loop vacuum - rate-limited slew toward's PSG commanded vector
            if (psgIntertialDirection.sqrMagnitude > 0.0)
            {
                if (_slewDir.sqrMagnitude <= 0.0)
                {
                    _slewDir = psgIntertialDirection.normalized;
                } else if (dt > 0.0)
                {
                    _slewDir = SlewToward(_slewDir, psgIntertialDirection.normalized, MaxSlewDegPerSec * dt);
                }
            }

            return new Command
            {
                HasInertialDirection = _slewDir.sqrMagnitude > 0.0,
                InertialDirection = _slewDir
            };
        }

        private static double FlightPathAngleDeg(double vSpeed, double speed)
        {
            if (speed <= 0) return 90.0;
            return Math.Asin(MathHelpers.Clamp(vSpeed / speed, -1.0, 1.0)) * 180.0 / Math.PI;
        }

        private static bool AeroNegligible(VesselState vs, double qKpa, double dQdt)
        {
            double thrustN = (vs.CurrentThrust > 0.0 ? vs.CurrentThrust : vs.AvailableThrust) * 1000.0; // kN -> N
            if (thrustN <= 0.0) return false;
            double dragN = 0.5 * vs.AtmosphericDensity * vs.SurfaceSpeed * vs.SurfaceSpeed * EstimateDragAreaCd(vs.Vessel);
            return qKpa < HandoverQKpa && dQdt < 0.0 && dragN / thrustN < DragThrustRatioTrigger;
        }

        private static Vector3d NoseDirection(VesselState vs)
        {
            if (vs.Vessel == null || vs.Vessel.ReferenceTransform == null) return Vector3d.zero;
            Vector3 up = vs.Vessel.ReferenceTransform.up;
            return new Vector3d(up.x, up.y, up.z).normalized;
        }

        private static Vector3d SlewToward(Vector3d from, Vector3d to, double maxDeg)
        {
            from = from.normalized;
            to = to.normalized;
            double ang = Vector3d.Angle(from, to);
            if (ang <= maxDeg || ang < 1e-9) return to;
            double t = maxDeg / ang;
            return (from + (to - from) * t).normalized;
        }

        private static double HandoverAltitude(CelestialBody body)
        {
            double seaLevel = body.GetPressure(0.0);
            if (seaLevel <= 0.0) return double.NaN;
            double threshold = HandoverPressureFraction * seaLevel;
            double top = body.atmosphereDepth;
            for (double alt =  0.0; alt <= top; alt += 500.0)
            {
                if (body.GetPressure(alt) <= threshold) return alt;
            }
            return top;
        }

        private static double PsgExpectedOrbitFpaDeg(PsgSolution psg, double bodyRadius, double altitude)
        {
            PsgSolutionPoint[] pts = psg.Points;
            double targetRadius = bodyRadius + altitude;
            for (int i = 0; i < pts.Length - 1; i++) {
                double ra = pts[i].RelativePosition.magnitude;
                double rb = pts[i + 1].RelativePosition.magnitude;
                if ((ra <= targetRadius && targetRadius <= rb) || (rb <= targetRadius && targetRadius <= ra)) {
                    double span = rb - ra;
                    double t = Math.Abs(span) > 1e-6 ? MathHelpers.Clamp((targetRadius - ra) / span, 0.0, 1.0) : 0.0;
                    return PointOrbitFpaDeg(pts[i]) + (PointOrbitFpaDeg(pts[i + 1]) - PointOrbitFpaDeg(pts[i])) * t;
                }
            }

            return double.NaN;
        }

        private static double PointOrbitFpaDeg(PsgSolutionPoint p)
        {
            double r = p.RelativePosition.magnitude;
            double v = p.RelativeVelocity.magnitude;
            if (r <= 0.0 || v <= 0.0) return double.NaN;
            double sin = Vector3d.Dot(p.RelativeVelocity, p.RelativePosition) / (r * v);
            return Math.Asin(MathHelpers.Clamp(sin, -1.0, 1.0)) * 180 / Math.PI;
        }

        private static AscentShootingInputs BuildInputs(VesselState vs, double handover, double targetFpa)
        {
            AscentStage[] stages = BuildStages(vs);
            if (stages.Length == 0) return null;

            return new AscentShootingInputs
            {
                Mu = vs.BodyGravParameter,
                BodyRadius = vs.BodyRadius,
                DragAreaCd = EstimateDragAreaCd(vs.Vessel),
                Stages = stages,
                DensityAtAltitude = alt => DensityAt(vs.Body, alt),
                InitialState = new AscentState
                {
                    T = 0.0,
                    X = vs.BodyRadius + vs.AltitudeMeters,
                    Y = 0.0,
                    Vx = vs.VerticalSpeed,
                    Vy = vs.HorizontalSpeed,
                    MassKg = vs.TotalMass * 1000.0
                },
                TowerClearanceAltitudeMeters = vs.AltitudeMeters, // kick from current cleared state
                HandoverAltitudeMeters = handover,
                TargetFlightPathAngleDeg = targetFpa,
                MaxKickDeg = MaxKickDeg,
                StepSeconds = SolveStepSeconds,
                MaxSteps = SolveMaxSteps,
                RotationRateRadPerSec = vs.BodyRotationPeriod > 0.0 ? 2.0 * Math.PI / vs.BodyRotationPeriod : 0.0,
                LaunchLatitudeCos = Math.Cos(vs.LatitudeDeg *  Math.PI / 180.0),
                DynamicPressureShiftPa = DynamicPressureShiftPa
            };
        }

        private static double DensityAt(CelestialBody body, double altitude)
        {
            if (body == null || altitude >= body.atmosphereDepth) return 0.0;
            return body.GetDensity(body.GetPressure(altitude), body.GetTemperature(altitude));
        }

        private static AscentStage[] BuildStages(VesselState vs)
        {
            var list = new List<AscentStage>();
            if (vs.PoweredStages == null) return list.ToArray();
            foreach (PoweredStageInfo s in vs.PoweredStages)
            {
                if (s == null || !s.IsValid || !s.IsCurrentOrFutureStage) continue;
                double thrustN = s.VacuumThrust * 1000.0;
                double isp = s.VacuumSpecificImpulse;
                if (thrustN <= 0.0 || !MathHelpers.IsFinite(isp) || isp <= 0.0) continue;
                double mdot = thrustN / (isp * MathHelpers.StandardGravity);
                double propKg = (s.StartMass - s.EndMass) * 1000.0;
                if (mdot <= 0.0 || propKg <= 0.0) continue;
                list.Add(new AscentStage { ThrustNewtons = thrustN, MassFlowKgPerSec = mdot, PropellantKg = propKg, JettisonMassKg = 0.0 });
            }

            return list.ToArray();
        }

        private static double EstimateDragAreaCd(Vessel vessel)
        {
            if (vessel == null) return 10.0;
            Vector3 size = vessel.vesselSize;
            double a = size.x, b = size.y, c = size.z;
            double largest = Math.Max(a, Math.Max(b, c));
            double d1, d2;
            if (largest == a)
            {
                d1 = b; 
                d2 = c;
            }
            else if (largest == b) {
                d1 = a;
                d2 = c;
            } else
            {
                d1 = a;
                d2 = b;
            }
            
            return 0.5 * (0.25 * Math.PI * d1 *  d2);
        }
    }
}
