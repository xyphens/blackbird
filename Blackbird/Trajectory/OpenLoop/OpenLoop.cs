using System;
using System.Collections.Generic;
using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Psg;


namespace Blackbird.OpenLoop
{
    // Pre-launch open-loop ascent plan (I-load / chi table), derived from "ATMOSPHERIC ASCENT GUIDANCE FOR ROCKET-POWERED LAUNCH VEHICLES (Dukeman*)"
    // one free in-plane parameter (pitch rate after vertical rise), optimized to MAXIMIZE INJECTED MASS.
    // Each candidate: integrate first stage (vertical -> constant-rate ramp -> surface-prograde AoA=0)
    // to the pressure-fraction handoff altitude, hand the terminal state to PSG, read delivered mass.
    // The winning trajectory is stored as pitch-vs-SURFACE-SPEED (never time) and flown open-loop.
    public sealed class OpenLoopTrajectory
    {
        // search band for pitch rate (Ares I is 0.5-1.5 deg/s)
        // widened for TWR range we fly
        private const double RateMinDegPerSec = 0.2;
        private const double RateMaxDegPerSec = 2.2;
        private const int CoarseSamples = 9;
        private const double RateToleranceDegPerSec = 0.005;
        private const double GoldenRatio = 0.6180339887498949;

        private const double StepSeconds = 0.5; // matches AtmosphericAscent.SolveStepSeconds
        private const int MaxStepsPerCandidate = 4000; // 2000 s cap
        private const double RampCapFromVerticalDeg = 90.0;
        private const double RampMaxAoaDeg = 2.0; // ramp attitude may lead the velocity vector by at most this alpha


        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public double PitchRateDegPerSecond { get; private set; }
        public double HandoffAltitudeMeters {  get; private set; }
        public double PredictedInjectedMassKg { get; private set; }
        public double PredictedTimeToHandoffSeconds { get; private set; }
        public double PredictedTimeToOrbitSeconds { get; private set; }

        // chi table: pitch above horizon vs surface speed, strictly ascending
        public double[] TableSpeedMps { get; private set; }
        public double[] TablePitchDeg {  get; private set; }

        public OpenLoopSample[] Trace {  get; private set; }
        private OpenLoopTrajectory() { }

        private static OpenLoopTrajectory Fail(string reason)
        {
            return new OpenLoopTrajectory { IsValid = false, ReasonUnavailable = reason };
        }

        // flight lookup;  below first table = vertical hold, beyond = last
        public double PitchDeg(double surfaceSpeedMps)
        {
            double[] s = TableSpeedMps;
            double[] p = TablePitchDeg;
            if (s == null || s.Length == 0) return 90.0;
            if (surfaceSpeedMps <= s[0]) return 90.0;
            if (surfaceSpeedMps >= s[s.Length - 1]) return p[p.Length - 1];
            for (int i = 0; i < s.Length - 1; i++) {
                if (surfaceSpeedMps > s[i + 1]) continue;
                double f = (surfaceSpeedMps - s[i]) / (s[i + 1] - s[i]);
                return p[i] + (p[i + 1] - p[i]) * f;
            }

            return p[p.Length - 1];
        }

        public static OpenLoopTrajectory Build(OpenLoopInputs io)
        {
            string bad = Validate(io);
            if (bad != null) return Fail(bad);

            // coarse scan, then golden-section refine around the best sample
            // warm-start PSG with the previous candidate's solution
            PsgSolution warm = null;
            var coarse = new OpenLoopCandidate[CoarseSamples];
            int best = -1;
            for (int i = 0; i < CoarseSamples; i++)
            {
                double rate = RateMinDegPerSec + (RateMaxDegPerSec - RateMinDegPerSec) * i / (CoarseSamples - 1);
                coarse[i] = Evaluate(io, rate, ref warm);
                if (coarse[i].Valid && (best < 0 || coarse[i].InjectedMassKg > coarse[best].InjectedMassKg)) best = i;
            }

            if (best < 0)
            {
                var sb = new System.Text.StringBuilder("no pitch rate produced a PSG-convergent trajectory:");
                for (int i = 0; i < CoarseSamples; i++)
                    sb.Append(string.Format(" {0:F2}->{1};", coarse[i].RateDegPerSec, coarse[i].Reason));
                return Fail(sb.ToString());
            }

            double lo = best > 0 ? RateAt(best - 1) : RateAt(best);
            double hi = best < CoarseSamples - 1 ? RateAt(best + 1) : RateAt(best);
            OpenLoopCandidate winner = coarse[best];

            double a = lo;
            double b = hi;
            double x1 = b - GoldenRatio * (b - a);
            double x2 = a + GoldenRatio * (b - a);
            OpenLoopCandidate c1 = Evaluate(io, x1, ref warm);
            OpenLoopCandidate c2 = Evaluate(io, x2, ref warm);
            while (b - a > RateToleranceDegPerSec)
            {
                if (Score(c1) >= Score(c2)) {
                    b = x2;
                    x2 = x1;
                    c2 = c1;
                    x1 = b - GoldenRatio * (b - a);
                    c1 = Evaluate(io, x1, ref warm);
                } else
                {
                    a = x1;
                    x1 = x2;
                    c1 = c2;
                    x2 = a + GoldenRatio * (b - a);
                    c2 = Evaluate(io, x2,ref warm);
                }
            }

            if (Score(c1) > Score(winner)) winner = c1;
            if (Score(c2) > Score(winner)) winner = c2;
            if (!winner.Valid) return Fail($"refinement lost convergence: {winner.Reason}");

            return FromCandidate(io, winner);
        }
        private static double RateAt(int i) => RateMinDegPerSec + (RateMaxDegPerSec - RateMinDegPerSec) * i / (CoarseSamples - 1);
        private static double Score(OpenLoopCandidate c) => c.Valid ? c.InjectedMassKg : double.NegativeInfinity;
        public static void EmbedState(AscentState st, OpenLoopInputs io, out Vector3d r3, out Vector3d v3)
        {
            Vector3d er = io.PadRelativePosition.normalized;
            // dot product of a vector tells you angle between the two
            // (positive = point in similar direction, zero = perpendicular, negative = opposite directions)
            // removes unwanted directional component while maining aligned with downrange path
            Vector3d et = (io.DownrangeDirection - Vector3d.Dot(io.DownrangeDirection, er) * er).normalized;
            r3 = st.X * er + st.Y * et;
            v3 = st.Vx * er + st.Vy * et + Vector3d.Cross(r3, io.BodyAngularVelocity);
        }

        public static OpenLoopCandidate EvaluateCandidate(OpenLoopInputs io, double rateDegPerSec, PsgSolution warm = null)
        {
            string bad = Validate(io);
            if (bad != null) return new OpenLoopCandidate { RateDegPerSec = rateDegPerSec, Reason = bad };
            return Evaluate(io, rateDegPerSec, ref warm);
        }
        // candidate evaluation: integrate -> embed -> PSG
        private static OpenLoopCandidate Evaluate(OpenLoopInputs io, double rateDegPerSec, ref PsgSolution warm)
        {
            var c = new OpenLoopCandidate { RateDegPerSec = rateDegPerSec, Reason = string.Empty };

            try
            {
                AscentStage[] stages = BuildIntegratorStages(io.Stages);
                if (stages.Length == 0) { c.Reason = "no stages"; return c; }

                // net liftoff accel; clearance where speed hits the pitch-over threshold
                double a0 = Math.Max(0.5, stages[0].ThrustNewtons / io.LiftoffMassKg - io.Mu / (io.BodyRadiusMeters * io.BodyRadiusMeters));
                double clearanceAlt = io.PadAltitudeMeters + (io.PitchOverSpeedMps * io.PitchOverSpeedMps) / (2.0 * a0);
                double holdAlt = Math.Max(clearanceAlt, io.HoldVerticalUntilAltMeters);

                int seg = 0; // 0 vertical, 1 ramp, 2 prograde; flipped only between committed steps
                double tRampStart = io.PitchOverSpeedMps / a0;
                double law(AscentState st)
                {
                    if (seg == 0) return 0.0;
                    double prograde = 90.0 - st.FlightPathAngleDeg; // surface-prograde, AoA = 0
                    if (seg == 1)
                    {
                        double ramp = Math.Min(rateDegPerSec * Math.Max(0.0, st.T - tRampStart), RampCapFromVerticalDeg);
                        return Math.Min(ramp, prograde + RampMaxAoaDeg);
                    }
                    return prograde;
                }

                var cfg = new AscentDynamicsConfig
                {
                    Mu = io.Mu,
                    BodyRadius = io.BodyRadiusMeters,
                    DragAreaCd = io.DragAreaCd,
                    Stages = stages,
                    DensityAtAltitude = io.DensityAtAltitude,
                    SteeringPitchFromVerticalDeg = law
                };

                var integrator = new AscentIntegrator(cfg);

                var s = new AscentState
                {
                    T = 0.0,
                    X = io.BodyRadiusMeters + io.PadAltitudeMeters,
                    Y = 0.0,
                    Vx = 0.0,
                    Vy = 0.0,
                    MassKg = io.LiftoffMassKg
                };

                var path = new List<OpenLoopSample>(512) { ToSample(s, io, 90.0) };
                bool reached = false;

                for (int i = 0; i < MaxStepsPerCandidate; i++)
                {
                    double alt = s.AltitudeMeters(io.BodyRadiusMeters);
                    if (seg == 0 && s.SurfaceSpeed >= io.PitchOverSpeedMps && alt >= holdAlt)
                    {
                        seg = 1;
                        tRampStart = s.T;
                    }
                    if (seg == 1 && s.T > tRampStart && 90.0 - s.FlightPathAngleDeg >= rateDegPerSec * (s.T - tRampStart)) seg = 2;
                    if (alt >= io.HandoffAltitudeMeters)
                    {
                        reached = true;
                        break;
                    }
                    if (s.SurfaceSpeed > 1.0 && s.FlightPathAngleDeg <= 0.0)
                    {
                        c.Reason = "pitched past horizontal";
                        return c;
                    }
                    if (alt < -1.0)
                    {
                        c.Reason = "ground impact";
                        return c;
                    }

                    s = integrator.Step(s, StepSeconds);
                    path.Add(ToSample(s, io, 90.0 - law(s)));
                }
                if (!reached)
                {
                    c.Reason = "never reached handoff altitude";
                    return c;
                }

                // embed 2d terminal state into 3d: PSg takes body-centered position and inertial velocity
                // co-rotation is exactly: v = v_srf + omega x r
                EmbedState(s, io, out Vector3d r3, out Vector3d v3);

                PoweredStageInfo[] handoffStages = BuildHandoffStages(io.Stages, s.MassKg);
                PsgPhase[] phases = PsgPhase.FromPoweredStages(handoffStages);
                if (phases == null || phases.Length == 0)
                {
                    c.Reason = "no remaining stages at handoff";
                    return c;
                }

                double handoffUt = io.UniversalTime + s.T;
                PsgInitialState initial = PsgInitialState.Create(r3, v3, s.MassKg, handoffUt);
                PsgBodyModel body = PsgBodyModel.Create(io.Mu, io.BodyRadiusMeters, io.BodyAngularVelocity);
                PsgProblem problem = PsgProblem.Create(initial, body, io.Target, phases, v3.normalized);
                if (problem == null || !problem.IsValid)
                {
                    c.Reason = $"PSG problem invalid: {(problem == null ? "null" : problem.ReasonUnavailable)}";
                    return c;
                }
                PsgOptimizationResult result = new PsgOptimizer().Solve(problem, warm);
                if (result == null || !result.Success || result.Solution == null)
                {
                    c.Reason = "PSG did not converge";
                    return c;
                }

                warm = result.Solution;

                double burnoutFloorKg = handoffStages[handoffStages.Length - 1].EndMass * 1000.0;
                if (result.Solution.TerminalState().MassKg < burnoutFloorKg)
                {
                    c.Reason = "PSG solution exceeds available propellant";
                    return c;
                }

                c.Valid = true;
                c.PsgIterations = result.Iterations;
                c.PsgSolution = result.Solution;
                c.InjectedMassKg = result.Solution.TerminalState().MassKg;
                c.TimeToHandoffSeconds = s.T;
                c.PsgTimeToGoSeconds = result.Solution.TimeToGo(handoffUt);
                c.Path = path;
                return c;
            }
            catch (Exception ex)
            {
                c.Reason = $"exception: {ex.Message}";
                return c;
            }
        }

        private static OpenLoopSample ToSample(AscentState s, OpenLoopInputs io, double pitchAboveHorizonDeg)
        {
            return new OpenLoopSample
            {
                TimeSec = s.T,
                AltMeters = s.AltitudeMeters(io.BodyRadiusMeters),
                DownrangeMeters = io.BodyRadiusMeters * Math.Atan2(s.Y, s.X),
                SurfaceSpeedMps = s.SurfaceSpeed,
                PitchDeg = pitchAboveHorizonDeg,
                MassKg = s.MassKg
            };
        }

        // same source and formulas as AtmosphericAscent.BuildStages, plus inter-stage jettison mass
        private static AscentStage[] BuildIntegratorStages(PoweredStageInfo[] stages)
        {
            // usable stage
            var usable = new List<PoweredStageInfo>();
            if (stages != null) {
                foreach (PoweredStageInfo st in stages)
                {
                    if (st != null && st.IsValid && st.IsCurrentOrFutureStage && st.VacuumThrust > 0.0
                        && MathHelpers.IsFinite(st.VacuumSpecificImpulse) && st.VacuumSpecificImpulse > 0.0
                        && st.StartMass > st.EndMass) usable.Add(st);
                }
            }

            var list = new List<AscentStage>(usable.Count);
            for (int i = 0; i < usable.Count; i++)
            {
                double thrustN = usable[i].VacuumThrust * 1000.0;
                double mdot = thrustN / (usable[i].VacuumSpecificImpulse * MathHelpers.StandardGravity);
                double jettisonKg = i + 1 < usable.Count
                                    ? Math.Max(0.0, (usable[i].EndMass - usable[i + 1].StartMass) * 1000.0)
                                    : 0.0;
                list.Add(new AscentStage
                {
                    ThrustNewtons = thrustN,
                    MassFlowKgPerSec = mdot,
                    PropellantKg = (usable[i].StartMass - usable[i].EndMass) * 1000.0,
                    JettisonMassKg = jettisonKg
                });
            }

            return list.ToArray();
        }

        // Stage list as it will exist at handoff: exhausted stages dropped, the active stage's
        // StartMass replaced by the integrated vehicle mass. Masses in PoweredStageInfo are tons.
        private static PoweredStageInfo[] BuildHandoffStages(PoweredStageInfo[] stages, double vehicleMassKg)
        {
            var outList = new List<PoweredStageInfo>();
            if (stages == null) return outList.ToArray();
            double massTons = vehicleMassKg / 1000.0;
            bool activeFound = false;

            foreach (PoweredStageInfo st in stages)
            {
                if (st == null || !st.IsValid || !st.IsCurrentOrFutureStage) continue;
                if (!activeFound)
                {
                    if (massTons <=  st.EndMass) continue;
                    activeFound = true;
                    double start = Math.Min(massTons, st.StartMass);
                    outList.Add(Clone(st, start));
                } else
                {
                    outList.Add(Clone(st, st.StartMass));
                }
            }

            return outList.ToArray();
        }

        private static PoweredStageInfo Clone(PoweredStageInfo st, double startMassTons)
        {
            double mdotTons = st.VacuumSpecificImpulse > 0.0 ? st.VacuumThrust / (st.VacuumSpecificImpulse * MathHelpers.StandardGravity) : 0.0;
            return new PoweredStageInfo
            {
                IsValid = st.IsValid,
                ReasonUnavailable = st.ReasonUnavailable,
                Stage = st.Stage,
                PhaseIndex = st.PhaseIndex,
                IsCurrentOrFutureStage = true,
                StartMass = startMassTons,
                EndMass = st.EndMass,
                VacuumSpecificImpulse = st.VacuumSpecificImpulse,
                CurrentSpecificImpulse = st.CurrentSpecificImpulse,
                VacuumThrust = st.VacuumThrust,
                CurrentThrust = st.CurrentThrust,
                MinimumThrust = st.MinimumThrust,
                MinimumThrottle = st.MinimumThrottle,
                BurnTimeSeconds = mdotTons > 0.0 ? (startMassTons - st.EndMass) / mdotTons : st.BurnTimeSeconds,
                VacuumDeltaV = st.VacuumDeltaV,
                CurrentDeltaV = st.CurrentDeltaV
            };
        }

        // assemble the plan
        private static OpenLoopTrajectory FromCandidate(OpenLoopInputs io, OpenLoopCandidate w)
        {
            var speeds = new List<double>(w.Path.Count);
            var pitches = new List<double>(w.Path.Count);
            double lastSpeed = double.NegativeInfinity;
            foreach (OpenLoopSample p in w.Path) {
                if (p.SurfaceSpeedMps < io.PitchOverSpeedMps) continue; // vhold handled by lookup
                if (p.SurfaceSpeedMps <= lastSpeed + 0.1) continue; // enforce by ascending index
                lastSpeed = p.SurfaceSpeedMps;
                speeds.Add(p.SurfaceSpeedMps);
                pitches.Add(p.PitchDeg);
            }

            if (speeds.Count < 2) return Fail("winning trajectory produced a degenerate table");

            return new OpenLoopTrajectory
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                PitchRateDegPerSecond = w.RateDegPerSec,
                HandoffAltitudeMeters = io.HandoffAltitudeMeters,
                PredictedInjectedMassKg = w.InjectedMassKg,
                PredictedTimeToHandoffSeconds = w.TimeToHandoffSeconds,
                PredictedTimeToOrbitSeconds = w.TimeToHandoffSeconds + w.PsgTimeToGoSeconds,
                TableSpeedMps = speeds.ToArray(),
                TablePitchDeg = pitches.ToArray(),
                Trace = w.Path.ToArray()
            };
        }

        private static string Validate(OpenLoopInputs io)
        {
            if (io == null) return "null inputs";
            if (!MathHelpers.IsFinite(io.Mu) || io.Mu <= 0.0) return "bad mu";
            if (!MathHelpers.IsFinite(io.BodyRadiusMeters) || io.BodyRadiusMeters <= 0.0) return "bad body radius";
            if (!MathHelpers.IsFinite(io.LiftoffMassKg) || io.LiftoffMassKg <= 0.0) return "bad liftoff mass";
            if (!MathHelpers.IsFinite(io.HandoffAltitudeMeters) || io.HandoffAltitudeMeters <= io.PadAltitudeMeters) return "bad handoff altitude";
            if (io.DensityAtAltitude == null) return "no atmosphere model";
            if (io.Target == null || !io.Target.IsValid) return "no PSG target";
            if (io.Stages == null || io.Stages.Length == 0) return "no stages";
            if (io.PadRelativePosition.sqrMagnitude <= 0.0) return "no pad position";
            if (io.DownrangeDirection.sqrMagnitude <= 0.0) return "no downrange direction";
            return null;
        }
    }
}
