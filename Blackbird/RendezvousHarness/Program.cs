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
            CheckInterceptNearClosestApproach();
            CheckExecutorGating();
            CheckInterceptBurnClosedLoop();
            CheckClosestApproachCountsDown();
            CheckInterceptLargeCoOrbitalGap();
            CheckInterceptBurnTerminatesUnderWeakThrust();
            CheckMatchVelocityNullsRelativeVelocity();
            CheckInterceptBurnTerminatesUnderMinThrottleCreep();
            CheckCloseApproachParksAtStandoff();

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

        // Validates the executor's user gates and abort/reset, and that arming the intercept produces a
        // plan and triggering it starts a real burn command. Does not run the burn to completion (that
        // is Case 8) — a static-ish step here only confirms the gate logic and burn entry.
        private static void CheckExecutorGating()
        {
            Console.WriteLine("Case 7: terminal executor gating + abort/reset");

            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            SimWorld world = MakeCatchupSim(250000.0, 15.0);

            AssertTrue("starts Idle", ex.Phase == RendezvousPhase.Idle);
            AssertTrue("starts at Intercept", ex.Stage == RendezvousStage.Intercept);
            AssertTrue("idle update has no burn", !ex.Update(world).HasBurn);

            ex.RefreshPlanPreview(world);
            AssertTrue("preview plan available", ex.HasInterceptPlan);

            AssertTrue("execute ok", ex.Execute());
            AssertTrue("executing phase", ex.Phase == RendezvousPhase.Executing);
            AssertTrue("execute blocked while executing", !ex.Execute());
            RendezvousCommand first = ex.Update(world);
            AssertTrue("intercept burn started", first.HasBurn && ex.Phase == RendezvousPhase.Executing);

            ex.Abort();
            AssertTrue("aborted phase", ex.Phase == RendezvousPhase.Aborted);
            AssertTrue("aborted update has no burn", !ex.Update(world).HasBurn);
            AssertTrue("execute blocked when aborted", !ex.Execute());

            ex.Reset();
            AssertTrue("reset to idle", ex.Phase == RendezvousPhase.Idle && ex.Stage == RendezvousStage.Intercept);
        }

        // Drives the intercept burn to completion against a stepping two-body sim: arm -> trigger ->
        // apply the commanded thrust each tick -> propagate both vessels. Asserts the burn terminates
        // (delivered ΔV reaches the plan), the burn dramatically reduces closest approach vs coasting,
        // and the remaining stub stages still traverse to Complete on user triggers.
        private static void CheckInterceptBurnClosedLoop()
        {
            Console.WriteLine("Case 8: intercept burn closed loop (stepping sim)");

            double altitude = 250000.0;
            double R = KerbinRadius + altitude;
            SimWorld sim = MakeCatchupSim(altitude, 15.0);

            // Baseline: closest approach over one orbit if we never burn (stays the constant catch-up gap).
            double coastCA = MinSeparation(
                sim.ActivePosition, sim.ActiveVelocity, sim.TargetPosition, sim.TargetVelocity,
                KerbinMu, CircularPeriod(R), 720);

            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            ex.Execute();       // start the intercept stage; plan is frozen on the first executing tick

            const double maxAccel = 200.0;   // m/s^2 sim engine; high enough that the burn is near-impulsive
            const double dt = 0.01;
            InterceptSolution plan = default(InterceptSolution);
            bool planCaptured = false;
            Vector3d dvUnit = Vector3d.zero;
            double appliedAlongPlan = 0.0;
            int ticks = 0;

            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 200000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!planCaptured && ex.HasInterceptPlan)
                {
                    plan = ex.InterceptPlan;
                    dvUnit = plan.DeltaV.normalized;
                    planCaptured = true;
                }
                if (ex.Phase != RendezvousPhase.Executing) break;   // burn completed inside Update

                if (cmd.HasBurn)
                {
                    Vector3d dv = cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt);
                    sim.ApplyDeltaV(dv);
                    appliedAlongPlan += Vector3d.Dot(dv, dvUnit);
                }
                sim.Advance(dt);
            }

            AssertTrue("plan captured", planCaptured);
            AssertTrue("burn completed -> coast", ex.Phase == RendezvousPhase.Coast);
            AssertTrue("applied ΔV ~ planned", Math.Abs(appliedAlongPlan - plan.DeltaVMagnitude) < 2.0);

            // Closest approach from the post-burn state to the planned arrival.
            double remaining = Math.Max(plan.ArrivalUt - sim.UniversalTime, 1.0);
            double postCA = MinSeparation(
                sim.ActivePosition, sim.ActiveVelocity, sim.TargetPosition, sim.TargetVelocity,
                KerbinMu, remaining, 720);

            AssertTrue("burn collapses CA vs coast", postCA < coastCA * 0.05);
            AssertTrue("post-burn CA small (<3 km)", postCA < 3000.0);

            // After intercept the stage auto-advances to Match Velocity (queued for Execute). The full
            // match -> close traversal is exercised end-to-end in Case 13.
            AssertTrue("stage advanced to match", ex.Stage == RendezvousStage.MatchVelocity);
            AssertTrue("coast awaiting match", ex.Phase == RendezvousPhase.Coast);

            Console.WriteLine(string.Format(
                "    plan ΔV={0:F2} applied={1:F2} m/s  coastCA={2:F0} m  postCA={3:F1} m  ticks={4}",
                plan.DeltaVMagnitude, appliedAlongPlan, coastCA, postCA, ticks));
        }

        // DIAGNOSTIC: reproduces the user's "loaded in orbit" geometry — target ~60 deg ahead on the SAME
        // circular orbit (range ~700 km, rel speed ~vc, constant separation / never approaches). Shows
        // what the single-rev intercept returns here (is it a sane catch-up ΔV, or a degenerate ~0 that
        // the executor wrongly treats as "done"?).
        private static void CheckInterceptLargeCoOrbitalGap()
        {
            Console.WriteLine("Case 11: intercept across a large co-orbital gap [diagnostic]");

            double R = KerbinRadius + 100000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double period = CircularPeriod(R);
            double lead = 60.0 * Math.PI / 180.0;

            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(R * Math.Cos(lead), R * Math.Sin(lead), 0.0);
            Vector3d tVel = new Vector3d(-vc * Math.Sin(lead), vc * Math.Cos(lead), 0.0);

            Func<double, Vector3d> targetAt = ut =>
            {
                TwoBody.Propagate(tPos, tVel, KerbinMu, ut, out Vector3d rt, out _);
                return rt;
            };

            InterceptSolution sol = InterceptSolver.Solve(
                aPos, aVel, KerbinMu, new Vector3d(0, 0, 1), 0.0, targetAt,
                period * 0.05, period * 0.95, 60, true, 50.0);

            Console.WriteLine(string.Format(
                "    sep={0:F0} m  relspeed={1:F0} m/s  ->  success={2} dV={3:F2} tof={4:F0} predCA={5:F1}",
                (tPos - aPos).magnitude, (tVel - aVel).magnitude,
                sol.Success, sol.DeltaVMagnitude, sol.TimeOfFlight, sol.PredictedClosestApproach));
        }

        // Reproduces Bug 1: chaser and target on slightly different circular orbits (different periods)
        // so the true closest approach is a SYNODIC-scale event many orbits away. The old one-period scan
        // pinned the reported time at ~one period; the solver must instead report a real time that COUNTS
        // DOWN as the state advances. Asserts the time decreases by ~the elapsed time between samples.
        private static void CheckClosestApproachCountsDown()
        {
            Console.WriteLine("Case 10: closest-approach time counts down (synodic search)");

            double R = KerbinRadius + 150000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double period = CircularPeriod(R);

            // Target ~5 km higher (slower) and AHEAD; the faster lower chaser catches up to a close pass
            // a few hours out (synodic scale), well beyond one orbital period.
            double R2 = R + 5000.0;
            double vc2 = Math.Sqrt(KerbinMu / R2);
            double lead = 25.0 * Math.PI / 180.0;
            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(R2 * Math.Cos(lead), R2 * Math.Sin(lead), 0.0);
            Vector3d tVel = new Vector3d(-vc2 * Math.Sin(lead), vc2 * Math.Cos(lead), 0.0);

            double maxHorizon = 6.0 * 3600.0;
            ApproachResult first = ClosestApproachSolver.FindNextApproach(aPos, aVel, tPos, tVel, KerbinMu, maxHorizon, 240);

            // Advance both vessels 600 s and re-evaluate; the time-to-CA should drop by ~600 s.
            double advance = 600.0;
            TwoBody.Propagate(aPos, aVel, KerbinMu, advance, out Vector3d aPos2, out Vector3d aVel2);
            TwoBody.Propagate(tPos, tVel, KerbinMu, advance, out Vector3d tPos2, out Vector3d tVel2);
            ApproachResult second = ClosestApproachSolver.FindNextApproach(aPos2, aVel2, tPos2, tVel2, KerbinMu, maxHorizon, 240);

            AssertTrue("first approach found", first.Found);
            AssertTrue("time is beyond one period (synodic)", first.TimeSeconds > period);
            AssertTrue("time counts down ~600s after advancing", Math.Abs((first.TimeSeconds - advance) - second.TimeSeconds) < 60.0);
            AssertTrue("approach distance is small (real pass)", first.DistanceMeters < 8000.0);

            Console.WriteLine(string.Format(
                "    period={0:F0}s  CA#1={1:F0}m in {2:F0}s  CA#2 in {3:F0}s (expected ~{4:F0})",
                period, first.DistanceMeters, first.TimeSeconds, second.TimeSeconds, first.TimeSeconds - advance));
        }

        // DIAGNOSTIC: reproduces the in-game "accurate launch" geometry — target ~20 km radially out on a
        // slightly higher circular orbit, so the pair is at closest approach NOW and sliding past at a low
        // relative speed. Shows what the single-rev intercept solver does in this regime (vs the easy
        // 15-degree catch-up). No assertions; this is for diagnosis.
        private static void CheckInterceptNearClosestApproach()
        {
            Console.WriteLine("Case 9: intercept near closest approach (sliding past) [diagnostic]");

            double R = KerbinRadius + 100000.0;
            double dr = 19570.0;
            double R2 = R + dr;
            double vc = Math.Sqrt(KerbinMu / R);
            double vc2 = Math.Sqrt(KerbinMu / R2);

            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(R2, 0.0, 0.0);          // 19.57 km radially out
            Vector3d tVel = new Vector3d(0.0, vc2, 0.0);          // circular -> relvel perpendicular -> CA now

            double period = CircularPeriod(R);
            Func<double, Vector3d> targetAt = ut =>
            {
                TwoBody.Propagate(tPos, tVel, KerbinMu, ut, out Vector3d rt, out _);
                return rt;
            };

            InterceptSolution sol = InterceptSolver.Solve(
                aPos, aVel, KerbinMu, new Vector3d(0, 0, 1), 0.0, targetAt,
                period * 0.05, period * 0.95, 60, true, 50.0);

            double coastCA = MinSeparation(aPos, aVel, tPos, tVel, KerbinMu, period, 720);

            Console.WriteLine(string.Format(
                "    separation={0:F2} km  rel speed={1:F1} m/s  coast CA={2:F0} m",
                dr / 1000.0, Math.Abs(vc2 - vc), coastCA));
            Console.WriteLine(string.Format(
                "    intercept: success={0} dV={1:F1} m/s  tof={2:F0}s  predicted CA={3:F1} m",
                sol.Success, sol.DeltaVMagnitude, sol.TimeOfFlight, sol.PredictedClosestApproach));
        }

        // Reproduces the "stuck at 5% forever" bug: a weak engine on a long burn, where the delivered
        // component along the fixed axis plateaus below the planned ΔV (gravity rotates the velocity). The
        // burn must still TERMINATE (via the stall cutoff) instead of flooring at min throttle forever.
        private static void CheckInterceptBurnTerminatesUnderWeakThrust()
        {
            Console.WriteLine("Case 12: intercept burn terminates under weak thrust (stall cutoff)");

            SimWorld sim = MakeCatchupSim(250000.0, 15.0);
            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            ex.Execute();

            const double maxAccel = 0.5;   // weak engine -> long burn -> axis saturates before reaching planned
            const double dt = 0.2;
            int ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            AssertTrue("weak-thrust burn terminates", ex.Phase == RendezvousPhase.Coast);
            AssertTrue("terminated within tick budget", ticks < 500000);
            Console.WriteLine(string.Format("    terminated after {0} ticks ({1:F0}s sim)", ticks, ticks * dt));
        }

        // End-to-end Step 6: run the intercept burn to completion, coast to the closest approach the
        // intercept aimed at, then run the MATCH-VELOCITY stage and confirm it drives the relative speed
        // to ~0 and advances to the close stage. Both vessels propagate on their conics; the harness
        // applies the commanded thrust each tick.
        private static void CheckMatchVelocityNullsRelativeVelocity()
        {
            Console.WriteLine("Case 13: match-velocity nulls relative velocity at closest approach");

            SimWorld sim = MakeCatchupSim(250000.0, 15.0);
            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();

            const double maxAccel = 50.0;   // near-impulsive engine so both burns finish quickly
            double dt = 0.02;

            // --- intercept burn to completion (as in Case 8) ---
            ex.Execute();
            InterceptSolution plan = default(InterceptSolution);
            bool planCaptured = false;
            int ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!planCaptured && ex.HasInterceptPlan) { plan = ex.InterceptPlan; planCaptured = true; }
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }
            AssertTrue("plan captured", planCaptured);
            AssertTrue("intercept -> match stage",
                ex.Stage == RendezvousStage.MatchVelocity && ex.Phase == RendezvousPhase.Coast);

            // --- coast to the closest approach the intercept aimed at ---
            while (sim.UniversalTime < plan.ArrivalUt - dt) sim.Advance(dt);
            double relBefore = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            // --- match-velocity burn to completion ---
            AssertTrue("execute match", ex.Execute());
            ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }
            double relAfter = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            AssertTrue("match completed -> coast",
                ex.Phase == RendezvousPhase.Coast && ex.Stage == RendezvousStage.CloseApproach);
            AssertTrue("relative velocity nulled (<0.3 m/s)", relAfter < 0.3);
            AssertTrue("match reduced relative speed", relAfter < relBefore);

            Console.WriteLine(string.Format(
                "    relSpeed before={0:F2} after={1:F3} m/s  ticks={2}", relBefore, relAfter, ticks));
        }

        // End-to-end Step 7: with the executor advanced to the CloseApproach stage, drive the closing-
        // velocity controller against a fresh clean geometry — chaser 800 m behind a target on a circular
        // orbit, already matched — and confirm it parks within the standoff band, matched, and ends the
        // sequence (Phase.Complete = control handed back). The close stage is stateless, so feeding it a
        // different world than the one used to reach the stage is valid.
        private static void CheckCloseApproachParksAtStandoff()
        {
            Console.WriteLine("Case 15: close-approach parks at standoff and matches (stepping sim)");

            TerminalRendezvousExecutor ex = AdvanceToCloseStage();
            AssertTrue("at close stage",
                ex.Stage == RendezvousStage.CloseApproach && ex.Phase == RendezvousPhase.Coast);

            double R = KerbinRadius + 200000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            SimWorld sim = new SimWorld(KerbinMu,
                new Vector3d(R, -800.0, 0.0), new Vector3d(0.0, vc, 0.0),   // chaser 800 m behind, matched
                new Vector3d(R, 0.0, 0.0), new Vector3d(0.0, vc, 0.0),
                new Vector3d(0, 0, 1));

            double rangeBefore = (sim.TargetPosition - sim.ActivePosition).magnitude;

            AssertTrue("execute close", ex.Execute());
            const double maxAccel = 20.0;
            double dt = 0.05;
            int ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 1000000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            double rangeAfter = (sim.TargetPosition - sim.ActivePosition).magnitude;
            double relAfter = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            AssertTrue("close completed (rendezvous done)", ex.Phase == RendezvousPhase.Complete);
            AssertTrue("parked near standoff (<=150 m)", rangeAfter <= 150.0);
            AssertTrue("parked not collided (>=50 m)", rangeAfter >= 50.0);
            AssertTrue("matched at standoff (<0.6 m/s)", relAfter < 0.6);
            AssertTrue("range reduced", rangeAfter < rangeBefore);

            Console.WriteLine(string.Format(
                "    range {0:F0} -> {1:F0} m  relSpeed {2:F2} m/s  ticks={3} ({4:F0}s)",
                rangeBefore, rangeAfter, relAfter, ticks, ticks * dt));
        }

        // Drives a fresh executor through Intercept + Match Velocity to completion against a coplanar
        // catch-up sim, leaving it queued (Coast) at the CloseApproach stage. Used to set up Case 15
        // legitimately (the only way to reach the close stage is to complete the earlier stages).
        private static TerminalRendezvousExecutor AdvanceToCloseStage()
        {
            SimWorld sim = MakeCatchupSim(250000.0, 15.0);
            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            const double maxAccel = 50.0;
            double dt = 0.02;

            ex.Execute();   // intercept
            InterceptSolution plan = default(InterceptSolution);
            bool captured = false;
            int ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!captured && ex.HasInterceptPlan) { plan = ex.InterceptPlan; captured = true; }
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            while (sim.UniversalTime < plan.ArrivalUt - dt) sim.Advance(dt);   // coast to CA

            ex.Execute();   // match velocity
            ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            return ex;
        }

        // Regression for the in-game "intercept burn 6.8/7.6 m/s then stuck at min throttle forever" bug.
        // Delivered ΔV is driven to creep ASYMPTOTICALLY toward a ceiling BELOW the planned magnitude:
        // each frame it sets a microscopic new maximum but never reaches planned and never plateaus
        // exactly. The OLD stall logic reset its timer on every new maximum, so it never tripped (the burn
        // would floor forever); the deadband progress check must now stall out and complete the stage.
        // Uses InjectWorld so delivered ΔV is controlled directly (independent of commanded throttle).
        private static void CheckInterceptBurnTerminatesUnderMinThrottleCreep()
        {
            Console.WriteLine("Case 14: intercept burn terminates under min-throttle creep (deadband stall)");

            InjectWorld w = new InjectWorld(MakeCatchupSim(250000.0, 15.0));
            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            ex.Execute();

            // First update arms the burn and freezes the plan; the baseline is the active velocity now.
            ex.Update(w);
            Vector3d baseline = w.ActiveVelocity;
            Vector3d dvUnit = ex.InterceptPlan.DeltaV.normalized;
            double planned = ex.InterceptPlan.DeltaVMagnitude;
            AssertTrue("plan armed", ex.HasInterceptPlan && planned > 2.0);

            // Deliver a fast chunk to 1 m/s short of planned (well past the stall-arm threshold), then
            // creep asymptotically toward a ceiling 0.5 m/s short of planned — never reaching cutoff.
            w.AddVelocity(dvUnit * (planned - 1.0));
            double ceiling = planned - 0.5;

            double dt = 0.1;
            int ticks = 0;
            while (ex.Phase == RendezvousPhase.Executing && ticks++ < 100000)
            {
                ex.Update(w);
                if (ex.Phase != RendezvousPhase.Executing) break;
                if (ticks % 10 == 0)   // a tiny, shrinking new maximum roughly once per second
                {
                    double delivered = Vector3d.Dot(w.ActiveVelocity - baseline, dvUnit);
                    double gap = ceiling - delivered;
                    if (gap > 0.0) w.AddVelocity(dvUnit * (gap * 0.5));
                }
                w.Advance(dt);
            }

            double finalDelivered = Vector3d.Dot(w.ActiveVelocity - baseline, dvUnit);
            AssertTrue("creep burn terminates", ex.Phase == RendezvousPhase.Coast);
            AssertTrue("terminated promptly (<2000 ticks)", ticks < 2000);
            AssertTrue("stalled below planned (not 'reached')", finalDelivered < planned - CutoffMargin);

            Console.WriteLine(string.Format(
                "    planned={0:F2} delivered={1:F2} m/s  ticks={2} ({3:F0}s sim)",
                planned, finalDelivered, ticks, ticks * dt));
        }

        // The reached-cutoff epsilon used in the executor (kept in sync for the assertion above).
        private const double CutoffMargin = 0.15;

        // Builds a stepping sim where chaser and target share a circular orbit with the target leadDeg
        // ahead along-track (the classic coplanar catch-up).
        private static SimWorld MakeCatchupSim(double altitude, double leadDeg)
        {
            double R = KerbinRadius + altitude;
            double vc = Math.Sqrt(KerbinMu / R);
            double lead = leadDeg * Math.PI / 180.0;

            return new SimWorld(
                KerbinMu,
                new Vector3d(R, 0.0, 0.0),
                new Vector3d(0.0, vc, 0.0),
                new Vector3d(R * Math.Cos(lead), R * Math.Sin(lead), 0.0),
                new Vector3d(-vc * Math.Sin(lead), vc * Math.Cos(lead), 0.0),
                new Vector3d(0, 0, 1));
        }

        // Minimum separation between two two-body trajectories over a duration, sampled uniformly.
        private static double MinSeparation(
            Vector3d aPos, Vector3d aVel, Vector3d tPos, Vector3d tVel,
            double mu, double duration, int samples)
        {
            double min = double.PositiveInfinity;
            for (int i = 0; i <= samples; i++)
            {
                double dt = duration * i / samples;
                TwoBody.Propagate(aPos, aVel, mu, dt, out Vector3d ra, out _);
                TwoBody.Propagate(tPos, tVel, mu, dt, out Vector3d rt, out _);
                double d = (ra - rt).magnitude;
                if (d < min) min = d;
            }
            return min;
        }

        // Stepping in-memory IRendezvousWorld: advances both vessels along their two-body conics and lets
        // the harness apply impulses to the active vessel, so the executor's closed loop can be driven.
        private sealed class SimWorld : IRendezvousWorld
        {
            private readonly double _mu;
            private Vector3d _aPos, _aVel, _tPos, _tVel;
            private readonly Vector3d _refN;
            private double _ut;

            public SimWorld(double mu, Vector3d aPos, Vector3d aVel, Vector3d tPos, Vector3d tVel, Vector3d refN)
            {
                _mu = mu;
                _aPos = aPos; _aVel = aVel;
                _tPos = tPos; _tVel = tVel;
                _refN = refN;
                _ut = 0.0;
            }

            public double UniversalTime => _ut;
            public double Mu => _mu;
            public Vector3d ActivePosition => _aPos;
            public Vector3d ActiveVelocity => _aVel;
            public Vector3d TargetPosition => _tPos;
            public Vector3d TargetVelocity => _tVel;
            public Vector3d ReferenceNormal => _refN;

            public void ApplyDeltaV(Vector3d dv) { _aVel += dv; }

            public void Advance(double dt)
            {
                TwoBody.Propagate(_aPos, _aVel, _mu, dt, out _aPos, out _aVel);
                TwoBody.Propagate(_tPos, _tVel, _mu, dt, out _tPos, out _tVel);
                _ut += dt;
            }
        }

        // An IRendezvousWorld whose active velocity (and only that) is driven directly by the test, with a
        // manually advanced clock. Positions/target are frozen from a seed sim. Lets a test feed an exact
        // delivered-ΔV profile to the executor's cutoff logic, decoupled from any thrust model.
        private sealed class InjectWorld : IRendezvousWorld
        {
            private readonly SimWorld _seed;
            private Vector3d _activeVel;
            private double _ut;

            public InjectWorld(SimWorld seed)
            {
                _seed = seed;
                _activeVel = seed.ActiveVelocity;
                _ut = 0.0;
            }

            public double UniversalTime => _ut;
            public double Mu => _seed.Mu;
            public Vector3d ActivePosition => _seed.ActivePosition;
            public Vector3d ActiveVelocity => _activeVel;
            public Vector3d TargetPosition => _seed.TargetPosition;
            public Vector3d TargetVelocity => _seed.TargetVelocity;
            public Vector3d ReferenceNormal => _seed.ReferenceNormal;

            public void AddVelocity(Vector3d dv) { _activeVel += dv; }
            public void Advance(double dt) { _ut += dt; }
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
