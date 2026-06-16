using System;
using Blackbird.Mathematics;
using Blackbird.Rendezvous;
using UnityEngine;

namespace Blackbird.RendezvousHarness
{
    // Offline verification vehicle for the rendezvous math (no KSP runtime required, only the
    // Vector3d struct from UnityEngine.dll). Mirrors PsgHarness. Run directly; exit code 0 = all
    // checks passed. Add a new CheckXxx() per rendezvous build step as it lands.
    internal static class Program
    {
        private const double KerbinMu = 3.5316e12;
        private const double KerbinRadius = 600000.0;

        private static int _failures;

        private static int Main()
        {
            Console.WriteLine("BlackBird Rendezvous Harness");
            Console.WriteLine();

            CheckRelativeStateEquatorial();
            CheckRelativeStateRotatedPlane();
            CheckKeplerPropagation();
            CheckLambertRecoversCircularOrbit();
            CheckLambertRecoversEllipticalOrbit();
            CheckInterceptCoplanarCatchup();
            CheckExecutorStateMachine();

            Console.WriteLine();
            if (_failures == 0)
            {
                Console.WriteLine("ALL CHECKS PASSED");
                return 0;
            }

            Console.WriteLine(_failures + " CHECK(S) FAILED");
            return 1;
        }

        // Known geometry: target on a circular equatorial orbit (plane = XY, normal = +Z), so the
        // LVLH frame lands on the world axes and every expected value is hand-verifiable.
        // Chaser sits 200 m radially below, 500 m behind (along-track), 30 m to +H, and is closing
        // along-track at 5 m/s. Therefore target relative to chaser is +200 R / +500 V / -30 H,
        // and relative velocity is -5 along V (gap closing).
        private static void CheckRelativeStateEquatorial()
        {
            Console.WriteLine("Case 1: circular equatorial target (axis-aligned LVLH)");

            double r = KerbinRadius + 100000.0;
            double v = Math.Sqrt(KerbinMu / r);

            Vector3d body = Vector3d.zero;
            Vector3d targetPos = new Vector3d(r, 0.0, 0.0);
            Vector3d targetVel = new Vector3d(0.0, v, 0.0);
            Vector3d activePos = new Vector3d(r - 200.0, -500.0, 30.0);
            Vector3d activeVel = new Vector3d(0.0, v + 5.0, 0.0);

            RelativeState state = RelativeState.Compute(activePos, activeVel, targetPos, targetVel, body);

            AssertVec("RBar", state.Frame.RBar, new Vector3d(1, 0, 0));
            AssertVec("VBar", state.Frame.VBar, new Vector3d(0, 1, 0));
            AssertVec("HBar", state.Frame.HBar, new Vector3d(0, 0, 1));
            AssertVec("RelPosWorld", state.RelativePositionWorld, new Vector3d(200, 500, -30));
            AssertVec("RelPosLvlh", state.RelativePositionLvlh, new Vector3d(200, 500, -30));
            AssertVec("RelVelWorld", state.RelativeVelocityWorld, new Vector3d(0, -5, 0));
            AssertVec("RelVelLvlh", state.RelativeVelocityLvlh, new Vector3d(0, -5, 0));

            double expRange = Math.Sqrt(200.0 * 200.0 + 500.0 * 500.0 + 30.0 * 30.0);
            AssertScalar("Range", state.Range, expRange);
            AssertScalar("RangeRate", state.RangeRate, -2500.0 / expRange);
        }

        // Same relative geometry but the orbit plane is rotated (target at +Y moving toward -X,
        // normal still +Z). The world vectors differ, but the LVLH components MUST be identical to
        // Case 1 - a rotation-invariance check that the frame projection is real, not axis luck.
        private static void CheckRelativeStateRotatedPlane()
        {
            Console.WriteLine("Case 2: rotated orbit plane (LVLH must match Case 1)");

            double r = KerbinRadius + 100000.0;
            double v = Math.Sqrt(KerbinMu / r);

            Vector3d body = Vector3d.zero;
            Vector3d targetPos = new Vector3d(0.0, r, 0.0);
            Vector3d targetVel = new Vector3d(-v, 0.0, 0.0);
            // 200 below (-RBar=(0,-1,0)), 500 behind (-VBar=(1,0,0)), 30 to +H (0,0,1).
            Vector3d activePos = new Vector3d(500.0, r - 200.0, 30.0);
            Vector3d activeVel = new Vector3d(-(v + 5.0), 0.0, 0.0);

            RelativeState state = RelativeState.Compute(activePos, activeVel, targetPos, targetVel, body);

            AssertVec("RBar", state.Frame.RBar, new Vector3d(0, 1, 0));
            AssertVec("VBar", state.Frame.VBar, new Vector3d(-1, 0, 0));
            AssertVec("HBar", state.Frame.HBar, new Vector3d(0, 0, 1));
            AssertVec("RelPosWorld", state.RelativePositionWorld, new Vector3d(-500, 200, -30));
            AssertVec("RelPosLvlh", state.RelativePositionLvlh, new Vector3d(200, 500, -30));
            AssertVec("RelVelLvlh", state.RelativeVelocityLvlh, new Vector3d(0, -5, 0));

            double expRange = Math.Sqrt(200.0 * 200.0 + 500.0 * 500.0 + 30.0 * 30.0);
            AssertScalar("Range", state.Range, expRange);
            AssertScalar("RangeRate", state.RangeRate, -2500.0 / expRange);
        }

        // Two-body propagation: a circular orbit advanced a quarter period must rotate 90 degrees;
        // a full period (circular and elliptical) must return exactly to the start. Validates the
        // universal-variable propagator independently of Lambert.
        private static void CheckKeplerPropagation()
        {
            Console.WriteLine("Case 3: two-body Kepler propagation (universal variable)");

            double r = KerbinRadius + 200000.0;
            double vc = Math.Sqrt(KerbinMu / r);
            Vector3d r0 = new Vector3d(r, 0.0, 0.0);
            Vector3d v0 = new Vector3d(0.0, vc, 0.0);
            double period = CircularPeriod(r);

            TwoBody.Propagate(r0, v0, KerbinMu, period / 4.0, out Vector3d rQ, out Vector3d vQ);
            AssertVecRel("circ T/4 pos", rQ, new Vector3d(0.0, r, 0.0), 1e-6);
            AssertVecRel("circ T/4 vel", vQ, new Vector3d(-vc, 0.0, 0.0), 1e-6);

            TwoBody.Propagate(r0, v0, KerbinMu, period, out Vector3d rF, out Vector3d vF);
            AssertVecRel("circ T pos", rF, r0, 1e-6);
            AssertVecRel("circ T vel", vF, v0, 1e-6);

            // Elliptical orbit (faster than circular at this apse): full period returns to start.
            Vector3d ev0 = new Vector3d(0.0, vc * 1.2, 0.0);
            double a = 1.0 / (2.0 / r - ev0.sqrMagnitude / KerbinMu);
            double ePeriod = 2.0 * Math.PI * Math.Sqrt(a * a * a / KerbinMu);
            TwoBody.Propagate(r0, ev0, KerbinMu, ePeriod, out Vector3d erF, out Vector3d evF);
            AssertVecRel("ellip T pos", erF, r0, 1e-6);
            AssertVecRel("ellip T vel", evF, ev0, 1e-6);
        }

        // Lambert self-consistency: take r1 and r2 from a known circular orbit separated by a known
        // time of flight; the solved transfer velocities must equal the orbit's actual velocities.
        // If Lambert recovers the generating orbit, the solver is correct.
        private static void CheckLambertRecoversCircularOrbit()
        {
            Console.WriteLine("Case 4: Lambert recovers the generating circular orbit");

            double r = KerbinRadius + 300000.0;
            double vc = Math.Sqrt(KerbinMu / r);
            Vector3d r1 = new Vector3d(r, 0.0, 0.0);
            Vector3d v1True = new Vector3d(0.0, vc, 0.0);
            double tof = CircularPeriod(r) / 4.0;   // quarter orbit -> short way

            TwoBody.Propagate(r1, v1True, KerbinMu, tof, out Vector3d r2, out Vector3d v2True);

            LambertResult res = LambertSolver.Solve(r1, r2, tof, KerbinMu, true, new Vector3d(0, 0, 1));
            AssertTrue("solver success", res.Success);
            AssertVecRel("departure V1", res.V1, v1True, 1e-5);
            AssertVecRel("arrival V2", res.V2, v2True, 1e-5);
        }

        // Same self-consistency check on an eccentric orbit, with a time of flight under half a
        // period so the short-way solution is the correct one.
        private static void CheckLambertRecoversEllipticalOrbit()
        {
            Console.WriteLine("Case 5: Lambert recovers the generating elliptical orbit");

            double r = KerbinRadius + 150000.0;
            double vc = Math.Sqrt(KerbinMu / r);
            Vector3d r1 = new Vector3d(r, 0.0, 0.0);
            Vector3d v1True = new Vector3d(0.0, vc * 1.15, 0.0);   // eccentric, starting at periapsis

            double a = 1.0 / (2.0 / r - v1True.sqrMagnitude / KerbinMu);
            double period = 2.0 * Math.PI * Math.Sqrt(a * a * a / KerbinMu);
            double tof = period * 0.2;                              // < half period -> short way

            TwoBody.Propagate(r1, v1True, KerbinMu, tof, out Vector3d r2, out Vector3d v2True);

            LambertResult res = LambertSolver.Solve(r1, r2, tof, KerbinMu, true, new Vector3d(0, 0, 1));
            AssertTrue("solver success", res.Success);
            AssertVecRel("departure V1", res.V1, v1True, 1e-5);
            AssertVecRel("arrival V2", res.V2, v2True, 1e-5);
        }

        // Coplanar catch-up: chaser and target share a circular orbit with the target 15 degrees
        // ahead. The planner must find a feasible, sane-ΔV intercept whose transfer actually reaches
        // the target (closest approach ~0), within the iteration/time budget. Target prediction is the
        // pure two-body propagator, so the conic transfer lands exactly on the predicted point.
        private static void CheckInterceptCoplanarCatchup()
        {
            Console.WriteLine("Case 6: coplanar catch-up intercept (Lambert sweep)");

            double R = KerbinRadius + 250000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double period = CircularPeriod(R);

            Vector3d chaserPos = new Vector3d(R, 0.0, 0.0);
            Vector3d chaserVel = new Vector3d(0.0, vc, 0.0);

            double lead = 15.0 * Math.PI / 180.0;   // target ahead along-track
            Vector3d targetPos0 = new Vector3d(R * Math.Cos(lead), R * Math.Sin(lead), 0.0);
            Vector3d targetVel0 = new Vector3d(-vc * Math.Sin(lead), vc * Math.Cos(lead), 0.0);

            // Target prediction = pure two-body propagation from its t0 state (ignition UT = 0).
            Func<double, Vector3d> targetAt = ut =>
            {
                TwoBody.Propagate(targetPos0, targetVel0, KerbinMu, ut, out Vector3d rt, out _);
                return rt;
            };

            InterceptSolution sol = InterceptSolver.Solve(
                chaserPos, chaserVel, KerbinMu, new Vector3d(0, 0, 1), 0.0, targetAt,
                tofMin: period * 0.1, tofMax: period * 0.9, arrivalSamples: 60,
                prograde: true, budgetMilliseconds: 50.0);

            AssertTrue("intercept success", sol.Success);
            AssertTrue("status Ok", sol.Status == InterceptStatus.Ok);
            AssertTrue("dV positive", sol.DeltaVMagnitude > 0.0);
            AssertTrue("dV sane (<1500)", sol.DeltaVMagnitude < 1500.0);
            AssertTrue("CA near zero (<1 m)", sol.PredictedClosestApproach < 1.0);

            // Independent check: fly the planned transfer and confirm it lands on the target.
            TwoBody.Propagate(chaserPos, sol.TransferDepartureVelocity, KerbinMu, sol.TimeOfFlight,
                              out Vector3d landed, out _);
            AssertVecRel("transfer lands on target", landed, targetAt(sol.ArrivalUt), 1e-6);

            Console.WriteLine(string.Format(
                "    chose tof={0:F1}s dV={1:F2} m/s CA={2:E2} m samples={3}",
                sol.TimeOfFlight, sol.DeltaVMagnitude, sol.PredictedClosestApproach, sol.SamplesEvaluated));
        }

        // Drives the executor skeleton through the full user-gated sequence and asserts the phase/stage
        // transitions: Idle -> arm/trigger Intercept -> Coast -> arm/trigger MatchVelocity -> Coast ->
        // arm/trigger CloseApproach -> Complete. Also checks the gates reject out-of-order actions and
        // that no command carries a burn (none wired yet).
        private static void CheckExecutorStateMachine()
        {
            Console.WriteLine("Case 7: terminal executor skeleton (staged state machine)");

            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            FakeWorld world = new FakeWorld
            {
                UniversalTime = 0.0,
                Mu = KerbinMu,
                ActivePosition = new Vector3d(KerbinRadius + 200000.0, 0, 0),
                ActiveVelocity = new Vector3d(0, 2000, 0),
                TargetPosition = new Vector3d(KerbinRadius + 200000.0, 1000, 0),
                TargetVelocity = new Vector3d(0, 2000, 0),
                ReferenceNormal = new Vector3d(0, 0, 1)
            };

            AssertTrue("starts Idle", ex.Phase == RendezvousPhase.Idle);
            AssertTrue("starts at Intercept", ex.Stage == RendezvousStage.Intercept);
            AssertTrue("idle update has no burn", !ex.Update(world).HasBurn);
            AssertTrue("trigger blocked before arm", !ex.Trigger());

            // Stage 1: Intercept
            AssertTrue("arm intercept", ex.Arm());
            AssertTrue("armed phase", ex.Phase == RendezvousPhase.Armed);
            AssertTrue("re-arm blocked while armed", !ex.Arm());
            AssertTrue("armed update no burn", !ex.Update(world).HasBurn);
            AssertTrue("trigger intercept", ex.Trigger());
            AssertTrue("executing phase", ex.Phase == RendezvousPhase.Executing);
            AssertTrue("intercept step no burn", !ex.Update(world).HasBurn);
            AssertTrue("coast after intercept", ex.Phase == RendezvousPhase.Coast);
            AssertTrue("stage still intercept", ex.Stage == RendezvousStage.Intercept);

            // Stage 2: MatchVelocity
            AssertTrue("arm advances to match", ex.Arm());
            AssertTrue("stage is match", ex.Stage == RendezvousStage.MatchVelocity);
            ex.Trigger();
            ex.Update(world);
            AssertTrue("coast after match", ex.Phase == RendezvousPhase.Coast);

            // Stage 3: CloseApproach -> Complete
            AssertTrue("arm advances to close", ex.Arm());
            AssertTrue("stage is close", ex.Stage == RendezvousStage.CloseApproach);
            ex.Trigger();
            ex.Update(world);
            AssertTrue("complete after close", ex.Phase == RendezvousPhase.Complete);
            AssertTrue("IsComplete", ex.IsComplete);
            AssertTrue("arm blocked when complete", !ex.Arm());

            // Abort then reset
            ex.Abort();
            AssertTrue("aborted phase", ex.Phase == RendezvousPhase.Aborted);
            ex.Reset();
            AssertTrue("reset to idle", ex.Phase == RendezvousPhase.Idle && ex.Stage == RendezvousStage.Intercept);
        }

        // Minimal in-memory IRendezvousWorld for driving the executor offline.
        private sealed class FakeWorld : IRendezvousWorld
        {
            public double UniversalTime { get; set; }
            public double Mu { get; set; }
            public Vector3d ActivePosition { get; set; }
            public Vector3d ActiveVelocity { get; set; }
            public Vector3d TargetPosition { get; set; }
            public Vector3d TargetVelocity { get; set; }
            public Vector3d ReferenceNormal { get; set; }
        }

        private static double CircularPeriod(double radius)
        {
            return 2.0 * Math.PI * Math.Sqrt(radius * radius * radius / KerbinMu);
        }

        private static void AssertTrue(string label, bool condition)
        {
            Report(label, condition, "true", condition ? "true" : "false", "-");
        }

        // Relative-tolerance vector compare, for large-magnitude (orbital) quantities where an
        // absolute 1e-6 m would be unreasonably strict.
        private static void AssertVecRel(string label, Vector3d actual, Vector3d expected, double relTol)
        {
            double scale = Math.Max(expected.magnitude, 1.0);
            double err = (actual - expected).magnitude / scale;
            Report(label, err <= relTol, Fmt(expected), Fmt(actual), err.ToString("E2"));
        }

        private static void AssertVec(string label, Vector3d actual, Vector3d expected)
        {
            double err = (actual - expected).magnitude;
            bool ok = err <= 1e-6;
            Report(label, ok, Fmt(expected), Fmt(actual), err.ToString("E2"));
        }

        private static void AssertScalar(string label, double actual, double expected)
        {
            double err = Math.Abs(actual - expected);
            bool ok = err <= 1e-6;
            Report(label, ok, expected.ToString("F6"), actual.ToString("F6"), err.ToString("E2"));
        }

        private static void Report(string label, bool ok, string expected, string actual, string err)
        {
            if (ok)
            {
                Console.WriteLine("  [PASS] " + label.PadRight(12) + " = " + actual);
            }
            else
            {
                _failures++;
                Console.WriteLine("  [FAIL] " + label.PadRight(12) +
                                  " expected " + expected + " got " + actual + " (err " + err + ")");
            }
        }

        private static string Fmt(Vector3d v)
        {
            return "(" + v.x.ToString("F3") + ", " + v.y.ToString("F3") + ", " + v.z.ToString("F3") + ")";
        }
    }
}
