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

        //private const double DefaultConservatismMarginDeg = 10.0; // hold handover this many degrees more vertical than PSG (should be ~55 degrees at 34km)
        //private const double DefaultHandoverPressureFraction = 0.001; // vacuum boundary
        private const double DynamicPressureShiftPa = 2000.0; // surface- -> orbit-prograde shift used by solve
        private const double DragThrustRatioTrigger = 0.001; // aero force negligible vs thrust -> hand to PSG
        private const double HandoverQKpa = 1.0;  // safe kPa to had over to PSG
        private const double MaxSlewDegPerSec = 5.0;
        private const double MaxKickDeg = 30.0;
        private const double SolveStepSeconds = 0.5;
        private const int SolveMaxSteps = 4000;
        private const double ProfileLogIntervalSeconds = 1.0; // per-step pitch-profile sampling cadence

        private static readonly BlackbirdLog Log = new BlackbirdLog(LogContext.Trajectory);

        // in-flight tunables (GuidanceComputer "PSG Smoothing Margin" and "kPa Fraction")
        private double _conservatismMarginDeg = 10.0;
        private double _handoverPressureFraction = 0.001;

        private Phase _phase = Phase.ProgradeAtmospheric;
        private bool _kickSolved;
        private double _kickDeg;
        private double _lastQKpa = double.NaN;
        private double _lastUt = double.NaN;
        private Vector3d _slewDir = Vector3d.zero;
        private double _lastDiagUt = double.NegativeInfinity;
        private double _lastProfileUt = double.NegativeInfinity;

        private double _handoverAltMeters = double.NaN;

        public bool KickSolved => _kickSolved;
        public double KickAngleDeg => _kickDeg;
        public Phase CurrentPhase => _phase;

        // live tunables from GuidanceComputer; ignore non-finite/non-positive to keep the defaults safe.
        // takes effect on the next kick solve (has no effect once _kickSolved latches).
        public void Configure(double conservatismMarginDeg, double handoverPressureFraction)
        {
            if (MathHelpers.IsFinite(conservatismMarginDeg)) _conservatismMarginDeg = conservatismMarginDeg;
            if (MathHelpers.IsFinite(handoverPressureFraction) && handoverPressureFraction > 0.0) _handoverPressureFraction = handoverPressureFraction;
        }

        public void Reset()
        {
            _phase = Phase.ProgradeAtmospheric;
            _kickSolved = false;
            _kickDeg = 0.0;
            _lastQKpa = double.NaN;
            _lastUt = double.NaN;
            _slewDir = Vector3d.zero;
            _handoverAltMeters = double.NaN;
        }

        public struct Command
        {
            public double PitchDeg; // used when !HasIntertialDirection
            public bool HasInertialDirection;
            public Vector3d InertialDirection; // slew-limited PSG vector (vacuum)
        }

        // solve the kick once PSG has a solution
        // only ran once
        public bool TrySolveKick(VesselState vs, PsgSolution psg, double vrfSpeed)
        {
            if (_kickSolved) return true;
            if (vs == null || vs.Body == null || !vs.Body.atmosphere) return Diag(vs, "no atmosphere");
            if (psg == null || !psg.IsValid || psg.Points == null || psg.Points.Length < 2) return Diag(vs, "PSG not ready");

            // 100^2 / (2*a0)
            double a0 = Math.Max(0.5, vs.AvailableThrust / vs.TotalMass - vs.BodySurfaceGravity);
            _handoverAltMeters = (vrfSpeed * vrfSpeed) / (2.0 * a0);

            try
            {
                double handover = HandoverAltitude(vs.Body);
                if (!MathHelpers.IsFinite(handover) || handover <= vs.AltitudeMeters) return Diag(vs, "handover invalid: " + handover);

                double psgFpa = PsgExpectedOrbitFpaDeg(psg, vs.BodyRadius, handover);
                if (!MathHelpers.IsFinite(psgFpa)) return Diag(vs, string.Format("psgFpa NaN (points don't bracket {0:F0} m)", handover));
                double targetFpa = psgFpa + _conservatismMarginDeg;

                AscentShootingInputs io = BuildInputs(vs, handover, targetFpa, _handoverAltMeters);
                if (io == null) return Diag(vs, "no stages");

                AscentShootingResult r = AscentShootingSolver.Solve(io);
                if (!r.Converged)
                {
                    double zk = AscentShootingSolver.PredictHandover(io, 0.0).OrbitFpaDeg;
                    double mk = AscentShootingSolver.PredictHandover(io, MaxKickDeg).OrbitFpaDeg;
                    return Diag(vs, string.Format("target {0:F1} vs window [{1:F1}..{2:F1}] (maxkick..0kick), surfFPA {3:F1}",
                        targetFpa, mk, zk, FlightPathAngleDeg(vs.VerticalSpeed, vs.SurfaceSpeed)));
                }

                _kickDeg = r.KickAngleDeg;
                _handoverAltMeters = handover;
                _kickSolved = true;
                Log.Write(string.Format("[kick-ascent] kick={0:F2} deg | PSG FPA {1:F1} + {2:F1} margin = {3:F1} | handover {4:F1} m (kPa frac {5:G})",
                        _kickDeg, psgFpa, _conservatismMarginDeg, targetFpa, handover, _handoverPressureFraction));
                return true;
            }
            catch (Exception ex)
            {
                Log.Write("[kick-ascent] exception: " + ex.Message);
                return false;
            }
        }

        private bool Diag(VesselState vs, string reason)
        {
            if (vs != null && vs.UniversalTime - _lastDiagUt > 2.0)
            {
                _lastDiagUt = vs.UniversalTime;
                Log.Write(string.Format("[kick-ascent] waiting @ {0:F1} km: {1}", vs.AltitudeMeters / 1000.0, reason));
            }
            return false;
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
                if (psgReady && (AeroNegligible(vs, qKpa, dQdt) || vs.AltitudeMeters >= _handoverAltMeters))
                {
                    _phase = Phase.ClosedLoopVacuum;
                    _slewDir = NoseDirection(vs); // begin slew from current altitude
                    Log.Write(string.Format("[kick-ascent] PSG handoff at {0:F1} km", vs.AltitudeMeters / 1000.0));
                } else
                {
                    double surfaceFpa = FlightPathAngleDeg(vs.VerticalSpeed, vs.SurfaceSpeed);
                    double pitch = Math.Min(90.0 - _kickDeg, surfaceFpa); // kick held until velocity catches up, then prograde
                    LogProfile(vs, pitch, surfaceFpa);
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

            LogProfile(vs, PitchAboveHorizonDeg(vs, _slewDir), FlightPathAngleDeg(vs.VerticalSpeed, vs.SurfaceSpeed));
            return new Command
            {
                HasInertialDirection = _slewDir.sqrMagnitude > 0.0,
                InertialDirection = _slewDir
            };
        }

        // per-step pitch-profile sample: the atmospheric climb FPA/pitch that the act/proj streams don't capture.
        // cmdPitchDeg is the commanded pitch above horizon (surface-prograde law in atmosphere, slewed PSG vector in vacuum).
        private void LogProfile(VesselState vs, double cmdPitchDeg, double surfaceFpaDeg)
        {
            if (vs == null || vs.UniversalTime - _lastProfileUt < ProfileLogIntervalSeconds) return;
            _lastProfileUt = vs.UniversalTime;
            Log.Write("kick", new KickProfileSample
            {
                Phase = _phase.ToString(),
                AltKm = vs.AltitudeMeters / 1000.0,
                CmdPitchDeg = cmdPitchDeg,
                SurfaceFpaDeg = surfaceFpaDeg,
                AoADeg = surfaceFpaDeg - cmdPitchDeg,
                KickDeg = _kickDeg,
                Qkpa = vs.DynamicPressureKpa,
                Mach = vs.Mach,
                SurfaceSpeed = vs.SurfaceSpeed
            });
        }

        private struct KickProfileSample
        {
            public string Phase;
            public double AltKm;
            public double CmdPitchDeg;   // commanded pitch above horizon
            public double SurfaceFpaDeg; // flown surface-frame flight-path angle
            public double AoADeg;        // flown minus commanded (velocity lag)
            public double KickDeg;
            public double Qkpa;
            public double Mach;
            public double SurfaceSpeed;
        }

        // Pitch of a world-frame direction above the local horizon (90 = straight up, 0 = horizontal).
        private static double PitchAboveHorizonDeg(VesselState vs, Vector3d dir)
        {
            if (vs == null || vs.Body == null || dir.sqrMagnitude <= 0.0) return double.NaN;
            Vector3d up = (vs.Position - vs.Body.position).normalized;
            return 90.0 - Vector3d.Angle(dir, up);
        }

        private static double FlightPathAngleDeg(double vSpeed, double speed)
        {
            if (speed <= 0) return 90.0;
            return MathHelpers.Rad2Deg(Math.Asin(MathHelpers.Clamp(vSpeed / speed, -1.0, 1.0)));
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

        private double HandoverAltitude(CelestialBody body) => HandoverAltitude(body, _handoverPressureFraction);

        // hand control to PSG when there's no atmosphere/drag left
        public static double HandoverAltitude(CelestialBody body, double pressureFraction)
        {
            double seaLevel = body.GetPressure(0.0);
            if (seaLevel <= 0.0) return double.NaN;
            double threshold = pressureFraction * seaLevel;
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

        private static AscentShootingInputs BuildInputs(VesselState vs, double handover, double targetFpa, double towerClearance)
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
                TowerClearanceAltitudeMeters = towerClearance,
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

        public static double EstimateDragAreaCd(Vessel vessel)
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
