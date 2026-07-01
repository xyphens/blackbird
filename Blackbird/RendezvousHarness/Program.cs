using System;
using Blackbird.Docking;
using Blackbird.Mathematics;
using Blackbird.Modules;
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
            CheckClosestApproachJ2ShiftsResult();
            CheckClosestApproachStableUnderTimeAdvance();
            CheckClosestApproachSeparatingNowFindsNext();
            CheckClosestApproachFastPassMatchesBruteForce();
            CheckClosestApproachNoApproachReturnsNotFound();
            CheckHonestPredictedCaUnderJ2();
            CheckJ2AimReducesMiss();
            CheckReSolveAtIgnitionBeatsFrozen();
            CheckInterceptLargeCoOrbitalGap();
            CheckInterceptBurnTerminatesUnderWeakThrust();
            CheckMatchVelocityNullsRelativeVelocity();
            CheckInterceptBurnTerminatesUnderMinThrottleCreep();
            CheckCloseApproachParksAtStandoff();
            CheckInterceptIgnitionLead();
            CheckInterceptFiniteBurnAchievesCloseApproach();
            CheckDockingGeometry();
            CheckDockingController();
            CheckMatchVelocityAtDistanceHoldsWhileFinalApproachChases();
            CheckMatchVelocityAtDistanceBrakesAtTrigger();
            CheckCloseApproachDeadbandRelaxesWithRange();
            CheckMatchVelocityReaimsEachTick();
            CheckMatchVelocityStopsOnOvershootNoSecondBurn();
            CheckThrustEnvelope();
            CheckDockingSchedule();
            CheckBurnSettleGate();

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

            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            SimWorld world = MakeCatchupSim(250000.0, 15.0);

            AssertTrue("starts Idle", state.InterceptPhase == InterceptPhase.Idle);
            AssertTrue("no method until chosen", state.RendezvousMethod == RendezvousMethod.None);
            AssertTrue("idle update has no burn", !ex.Update(world).HasBurn);

            // Choose Intercept, then preview its plan while still idle.
            state.RendezvousMethod = RendezvousMethod.Intercept;
            ex.RefreshPlanPreview(world, InterceptMethod.SinglePhase);
            AssertTrue("preview plan available", ex.HasInterceptPlan);

            AssertTrue("execute ok", ex.Execute());
            AssertTrue("executing phase", state.InterceptPhase == InterceptPhase.Executing);
            AssertTrue("execute blocked while executing", !ex.Execute());
            RendezvousCommand first = ex.Update(world);
            AssertTrue("intercept burn started", first.HasBurn && state.InterceptPhase == InterceptPhase.Executing);

            ex.Abort();
            AssertTrue("aborted phase", state.InterceptPhase == InterceptPhase.Aborted);
            AssertTrue("aborted update has no burn", !ex.Update(world).HasBurn);
            AssertTrue("execute blocked when aborted", !ex.Execute());

            ex.Reset();
            AssertTrue("reset to idle", state.InterceptPhase == InterceptPhase.Idle && state.RendezvousMethod == RendezvousMethod.None);

            // Out-of-order execution: jump straight to Match Velocity from Idle (the on-demand safety gate).
            AssertTrue("execute match directly", ex.ForceExecute(RendezvousMethod.MatchVelocity));
            AssertTrue("jumped to match method",
                state.RendezvousMethod == RendezvousMethod.MatchVelocity && state.InterceptPhase == InterceptPhase.Executing);
            // ForceExecute PREEMPTS a running stage (the match-velocity-anytime safety valve), so it succeeds
            // mid-burn and switches the active method instead of being blocked.
            AssertTrue("execute(method) preempts mid-burn", ex.ForceExecute(RendezvousMethod.Intercept));
            AssertTrue("preempted to intercept",
                state.RendezvousMethod == RendezvousMethod.Intercept && state.InterceptPhase == InterceptPhase.Executing);
            ex.Reset();
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

            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.ForceExecute(RendezvousMethod.Intercept);   // plan is frozen on the first executing tick

            const double maxAccel = 200.0;   // m/s^2 sim engine; high enough that the burn is near-impulsive
            const double dt = 0.01;
            InterceptSolution plan = default(InterceptSolution);
            bool planCaptured = false;
            Vector3d dvUnit = Vector3d.zero;
            double appliedAlongPlan = 0.0;
            int ticks = 0;

            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 200000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!planCaptured && ex.HasInterceptPlan)
                {
                    plan = state.InterceptSolution;
                    dvUnit = plan.DeltaV.normalized;
                    planCaptured = true;
                }
                if (state.InterceptPhase != InterceptPhase.Executing) break;   // burn completed inside Update

                if (cmd.HasBurn)
                {
                    Vector3d dv = cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt);
                    sim.ApplyDeltaV(dv);
                    appliedAlongPlan += Vector3d.Dot(dv, dvUnit);
                }
                sim.Advance(dt);
            }

            AssertTrue("plan captured", planCaptured);
            AssertTrue("burn completed -> coast", state.InterceptPhase == InterceptPhase.Coast);
            AssertTrue("applied ΔV ~ planned", Math.Abs(appliedAlongPlan - plan.DeltaVMagnitude) < 2.0);

            // Closest approach from the post-burn state to the planned arrival.
            double remaining = Math.Max(plan.ArrivalUt - sim.UniversalTime, 1.0);
            double postCA = MinSeparation(
                sim.ActivePosition, sim.ActiveVelocity, sim.TargetPosition, sim.TargetVelocity,
                KerbinMu, remaining, 720);

            AssertTrue("burn collapses CA vs coast", postCA < coastCA * 0.05);
            AssertTrue("post-burn CA small (<3 km)", postCA < 3000.0);

            // Methods are independent now: intercept finishing drops to Coast with the method unchanged
            // (no auto-advance). The user picks the next method explicitly (exercised in Case 13).
            AssertTrue("intercept done -> coast", state.InterceptPhase == InterceptPhase.Coast);
            AssertTrue("method stays intercept (no auto-advance)", state.RendezvousMethod == RendezvousMethod.Intercept);

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

        // J2 materially shifts the predicted closest approach at RSS scale. Two coplanar inclined circular
        // orbits (250 / 400 km, i=51.6 deg) with the lower chaser catching the leading target over ~hours:
        // conic and J2 disagree on WHEN they line up (differential draconic period) and HOW close they pass
        // (differential nodal regression opens cross-track). Verifies the J2 path is wired and changes the
        // answer; the absolute accuracy vs Principia is an in-game check, not provable offline.
        private static void CheckClosestApproachJ2ShiftsResult()
        {
            Console.WriteLine("Case 10b: J2 shifts closest approach at RSS scale");

            const double earthMu = 3.986004418e14;
            const double earthRe = 6378136.3;
            const double earthJ2 = 1.082636e-03;
            double inc = 51.6 * Math.PI / 180.0;

            // Inclined plane whose line of nodes is +x; in-plane basis (u along node, p perpendicular).
            Vector3d u = new Vector3d(1.0, 0.0, 0.0);
            Vector3d p = new Vector3d(0.0, Math.Cos(inc), Math.Sin(inc));
            Vector3d pole = new Vector3d(0.0, 0.0, 1.0);   // body spin axis = +z (equator is XY)

            double R1 = earthRe + 250000.0, vc1 = Math.Sqrt(earthMu / R1);
            double R2 = earthRe + 400000.0, vc2 = Math.Sqrt(earthMu / R2);
            double lead = 40.0 * Math.PI / 180.0;          // target ahead; lower/faster chaser catches up

            Vector3d aPos = R1 * u;
            Vector3d aVel = vc1 * p;
            Vector3d tPos = R2 * (Math.Cos(lead) * u + Math.Sin(lead) * p);
            Vector3d tVel = vc2 * (-Math.Sin(lead) * u + Math.Cos(lead) * p);

            double horizon = 6.0 * 3600.0;
            int samples = 360;

            ApproachResult conic = ClosestApproachSolver.FindNextApproach(
                aPos, aVel, tPos, tVel, earthMu, horizon, samples);
            ApproachResult conicExplicitZero = ClosestApproachSolver.FindNextApproach(
                aPos, aVel, tPos, tVel, earthMu, horizon, samples, 0.0, earthRe, pole);
            ApproachResult j2 = ClosestApproachSolver.FindNextApproach(
                aPos, aVel, tPos, tVel, earthMu, horizon, samples, earthJ2, earthRe, pole);

            double timeShift = Math.Abs(j2.TimeSeconds - conic.TimeSeconds);
            double distShift = Math.Abs(j2.DistanceMeters - conic.DistanceMeters);

            Console.WriteLine(string.Format(
                "    conic: {0:F0}m in {1:F0}s   J2: {2:F0}m in {3:F0}s   dt={4:F0}s  dd={5:F0}m",
                conic.DistanceMeters, conic.TimeSeconds, j2.DistanceMeters, j2.TimeSeconds, timeShift, distShift));

            AssertTrue("conic and J2 both find an approach", conic.Found && j2.Found);
            AssertTrue("j2=0 reproduces the conic path exactly",
                Math.Abs(conicExplicitZero.TimeSeconds - conic.TimeSeconds) < 1e-6
                && Math.Abs(conicExplicitZero.DistanceMeters - conic.DistanceMeters) < 1e-6);
            AssertTrue("J2 materially shifts CA time (> 20 s)", timeShift > 20.0);
        }

        // The core stability guarantee: a closest approach is a fixed absolute event, so re-solving from a
        // LATER measured state must return the SAME event — distance unchanged, time counted down 1:1 by the
        // elapsed time. This is what the old grid-quantized/horizon-capped search failed (the in-game readout
        // that jumped and never converged to zero). Target 5 km higher and ahead; the lower/faster chaser
        // closes over a few hours. Advance the start by several amounts and assert the absolute CA UT holds.
        private static void CheckClosestApproachStableUnderTimeAdvance()
        {
            Console.WriteLine("Case 26: closest approach is a stable absolute event (distance fixed, time counts down 1:1)");

            double R = KerbinRadius + 150000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double R2 = R + 5000.0;
            double vc2 = Math.Sqrt(KerbinMu / R2);
            double lead = 25.0 * Math.PI / 180.0;

            Vector3d aPos0 = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel0 = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos0 = new Vector3d(R2 * Math.Cos(lead), R2 * Math.Sin(lead), 0.0);
            Vector3d tVel0 = new Vector3d(-vc2 * Math.Sin(lead), vc2 * Math.Cos(lead), 0.0);

            double horizon = 24.0 * 3600.0;
            ApproachResult baseRes = ClosestApproachSolver.FindNextApproach(aPos0, aVel0, tPos0, tVel0, KerbinMu, horizon, 240);
            AssertTrue("base approach found", baseRes.Found);

            double[] advances = { 300.0, 900.0, 1800.0, 3600.0 };
            foreach (double adv in advances)
            {
                TwoBody.Propagate(aPos0, aVel0, KerbinMu, adv, out Vector3d aP, out Vector3d aV);
                TwoBody.Propagate(tPos0, tVel0, KerbinMu, adv, out Vector3d tP, out Vector3d tV);
                ApproachResult r = ClosestApproachSolver.FindNextApproach(aP, aV, tP, tV, KerbinMu, horizon, 240);

                AssertTrue("approach found after +" + adv.ToString("F0") + "s", r.Found);
                AssertTrue("absolute CA UT stable (+" + adv.ToString("F0") + "s, <5 s)",
                    Math.Abs((adv + r.TimeSeconds) - baseRes.TimeSeconds) < 5.0);
                AssertTrue("CA distance stable (+" + adv.ToString("F0") + "s, <50 m)",
                    Math.Abs(r.DistanceMeters - baseRes.DistanceMeters) < 50.0);
                AssertTrue("time-to-CA counted down (+" + adv.ToString("F0") + "s)", r.TimeSeconds < baseRes.TimeSeconds);
            }

            Console.WriteLine(string.Format(
                "    CA {0:F0} m at absolute T+{1:F0}s; invariant across advances", baseRes.DistanceMeters, baseRes.TimeSeconds));
        }

        // Reproduces the "closest approach reads as 'now' while separating" symptom: the pair is in-track
        // aligned NOW (a local minimum of range) and immediately separating. The solver MUST NOT report that
        // t=0 point — it returns the NEXT real approach (one synodic period out), not the degenerate "in 0 s".
        private static void CheckClosestApproachSeparatingNowFindsNext()
        {
            Console.WriteLine("Case 27: separating-now is skipped; the NEXT approach is returned (no 'in 0 s')");

            double R = KerbinRadius + 100000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double R2 = R + 80000.0;                 // 80 km higher -> synodic ~ a few hours (within horizon)
            double vc2 = Math.Sqrt(KerbinMu / R2);
            double periodA = CircularPeriod(R);

            // Both at angle 0: in-track aligned, so range = radial gap NOW and grows as they de-phase.
            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(R2, 0.0, 0.0);
            Vector3d tVel = new Vector3d(0.0, vc2, 0.0);

            double horizon = 24.0 * 3600.0;
            ApproachResult res = ClosestApproachSolver.FindNextApproach(aPos, aVel, tPos, tVel, KerbinMu, horizon, 240);

            AssertTrue("approach found", res.Found);
            AssertTrue("skips the closest-now point (t >> 0, next pass)", res.TimeSeconds > periodA * 0.5);
            AssertTrue("within horizon", res.TimeSeconds < horizon);
            AssertTrue("next pass distance ~ radial gap (<2 km)", Math.Abs(res.DistanceMeters - (R2 - R)) < 2000.0);

            Console.WriteLine(string.Format(
                "    next approach {0:F0} m at T+{1:F0}s (period {2:F0}s)", res.DistanceMeters, res.TimeSeconds, periodA));
        }

        // Accuracy: a fast, sub-km pass (two near-equal orbits in slightly tilted planes cross near the node at
        // high relative speed) must be resolved to brute-force precision — not grid-quantized. Compares the
        // solver's distance/time against a dense uniform sweep over the first orbit (skipping the t~0 node).
        private static void CheckClosestApproachFastPassMatchesBruteForce()
        {
            Console.WriteLine("Case 28: fast sub-km pass resolved to brute-force accuracy");

            double R = KerbinRadius + 200000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            double R2 = R + 300.0;
            double vc2 = Math.Sqrt(KerbinMu / R2);
            double inc = 3.0 * Math.PI / 180.0;
            double period = CircularPeriod(R);

            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(R2, 0.0, 0.0);
            Vector3d tVel = new Vector3d(0.0, vc2 * Math.Cos(inc), vc2 * Math.Sin(inc));   // tilted plane

            double horizon = 6.0 * 3600.0;
            ApproachResult res = ClosestApproachSolver.FindNextApproach(aPos, aVel, tPos, tVel, KerbinMu, horizon, 240);

            BruteApproach(aPos, aVel, tPos, tVel, KerbinMu, 0.05 * period, 0.95 * period, 400000,
                out double bruteDist, out double bruteTime);

            AssertTrue("approach found", res.Found);
            AssertTrue("distance matches brute force (<5 m)", Math.Abs(res.DistanceMeters - bruteDist) < 5.0);
            AssertTrue("time matches brute force (<2 s)", Math.Abs(res.TimeSeconds - bruteTime) < 2.0);

            Console.WriteLine(string.Format(
                "    solver {0:F1} m @ {1:F1}s   brute {2:F1} m @ {3:F1}s", res.DistanceMeters, res.TimeSeconds, bruteDist, bruteTime));
        }

        // The solver must report NO approach (not a fabricated one) when none exists in the horizon: (a) two
        // co-orbital craft 180 deg apart hold a constant separation forever; (b) a real approach exists but
        // lies beyond a deliberately short horizon while the pair is still closing.
        private static void CheckClosestApproachNoApproachReturnsNotFound()
        {
            Console.WriteLine("Case 29: no approach within horizon returns NotFound (no fabricated minimum)");

            double R = KerbinRadius + 200000.0;
            double vc = Math.Sqrt(KerbinMu / R);

            // (a) Same orbit, 180 deg apart: range is constant (2R), so there is no local minimum at all.
            Vector3d aPos = new Vector3d(R, 0.0, 0.0);
            Vector3d aVel = new Vector3d(0.0, vc, 0.0);
            Vector3d tPos = new Vector3d(-R, 0.0, 0.0);
            Vector3d tVel = new Vector3d(0.0, -vc, 0.0);
            ApproachResult flat = ClosestApproachSolver.FindNextApproach(aPos, aVel, tPos, tVel, KerbinMu, 6.0 * 3600.0, 240);
            AssertTrue("constant-separation co-orbital: not found", !flat.Found);

            // (b) Hours-away approach, but a 600 s horizon: still closing, no minimum reached -> not fabricated.
            double R2 = R + 5000.0;
            double vc2 = Math.Sqrt(KerbinMu / R2);
            double lead = 25.0 * Math.PI / 180.0;
            Vector3d cPos = new Vector3d(R, 0.0, 0.0);
            Vector3d cVel = new Vector3d(0.0, vc, 0.0);
            Vector3d dPos = new Vector3d(R2 * Math.Cos(lead), R2 * Math.Sin(lead), 0.0);
            Vector3d dVel = new Vector3d(-vc2 * Math.Sin(lead), vc2 * Math.Cos(lead), 0.0);
            ApproachResult shortHorizon = ClosestApproachSolver.FindNextApproach(cPos, cVel, dPos, dVel, KerbinMu, 600.0, 240);
            AssertTrue("approach beyond horizon: not fabricated", !shortHorizon.Found);

            Console.WriteLine("    flat co-orbital + short-horizon both correctly NotFound");
        }

        // Phase 2A: the honest predicted-CA helper. A conic Lambert transfer that hits the target under conic
        // propagation must read ~0 from MinSeparationOverWindow with j2=0, and a material miss with Earth J2 —
        // proving the helper surfaces the real oblate miss a conic plan will fly.
        private static void CheckHonestPredictedCaUnderJ2()
        {
            Console.WriteLine("Case 33: honest predicted CA — conic plan hits under conic, misses under J2");

            const double earthMu = 3.986004418e14, earthRe = 6378136.3, earthJ2 = 1.082636e-3;
            double inc = 51.6 * Math.PI / 180.0;
            Vector3d u = new Vector3d(1, 0, 0);
            Vector3d p = new Vector3d(0, Math.Cos(inc), Math.Sin(inc));
            Vector3d pole = new Vector3d(0, 0, 1);
            Vector3d planeNormal = Vector3d.Cross(u, p).normalized;

            double R1 = earthRe + 250000.0, vc1 = Math.Sqrt(earthMu / R1);
            double R2 = earthRe + 400000.0, vc2 = Math.Sqrt(earthMu / R2);
            double lead = 40.0 * Math.PI / 180.0;
            Vector3d aPos = R1 * u;
            Vector3d tPos = R2 * (Math.Cos(lead) * u + Math.Sin(lead) * p);
            Vector3d tVel = vc2 * (-Math.Sin(lead) * u + Math.Cos(lead) * p);

            double tof = 3000.0;
            TwoBody.Propagate(tPos, tVel, earthMu, tof, out Vector3d tArrConic, out _);
            LambertResult lam = LambertSolver.Solve(aPos, tArrConic, tof, earthMu, true, planeNormal);
            AssertTrue("lambert success", lam.Success);

            double conicCa = ClosestApproachSolver.MinSeparationOverWindow(
                aPos, lam.V1, 0.0, tof, tPos, tVel, 0.0, earthMu, 200, 0.0, 0.0, Vector3d.zero);
            double j2Ca = ClosestApproachSolver.MinSeparationOverWindow(
                aPos, lam.V1, 0.0, tof, tPos, tVel, 0.0, earthMu, 200, earthJ2, earthRe, pole);

            AssertTrue("conic plan hits under conic (<5 m)", conicCa < 5.0);
            AssertTrue("same plan misses under J2 (>10 m)", j2Ca > 10.0);
            Console.WriteLine(string.Format("    conic CA={0:F1} m   J2 CA={1:F1} m", conicCa, j2Ca));
        }

        // Phase 2A: aiming the transfer at the J2-propagated target reduces the achieved miss — for the case it
        // actually matters: a FAR-FUTURE departure (the Hohmann window), where the target has coasted hours so
        // its conic-vs-J2 position error is large. (Harness finding: for a near-term transfer the chaser's own
        // transfer-arc J2 deviation dominates and aiming alone barely helps — that residual is for 2B re-solve /
        // 2C shooting / the closed loop.) Here the long target coast makes the J2 aim the dominant correction.
        private static void CheckJ2AimReducesMiss()
        {
            Console.WriteLine("Case 34: J2 aim reduces the miss for a far-future (Hohmann-style) departure");

            const double earthMu = 3.986004418e14, earthRe = 6378136.3, earthJ2 = 1.082636e-3;
            double inc = 51.6 * Math.PI / 180.0;
            Vector3d u = new Vector3d(1, 0, 0);
            Vector3d p = new Vector3d(0, Math.Cos(inc), Math.Sin(inc));
            Vector3d pole = new Vector3d(0, 0, 1);
            Vector3d planeNormal = Vector3d.Cross(u, p).normalized;

            double R1 = earthRe + 250000.0, vc1 = Math.Sqrt(earthMu / R1);
            double R2 = earthRe + 400000.0, vc2 = Math.Sqrt(earthMu / R2);
            double lead = 40.0 * Math.PI / 180.0;
            Vector3d aPos = R1 * u, aVel = vc1 * p;
            Vector3d tPos = R2 * (Math.Cos(lead) * u + Math.Sin(lead) * p);
            Vector3d tVel = vc2 * (-Math.Sin(lead) * u + Math.Cos(lead) * p);

            double ignitionUt = 18000.0;   // ~5 h out: a Hohmann window, long target coast
            double tof = 2000.0;           // short transfer arc, so target-position error dominates
            double arrivalUt = ignitionUt + tof;

            // Chaser's real (J2) state at the future ignition.
            ClosestApproachSolver.Propagate(aPos, aVel, ignitionUt, earthMu, earthJ2, earthRe, pole,
                out Vector3d aIg, out _);

            // Target position at arrival: conic prediction vs the real J2 position (differ by the long coast).
            TwoBody.Propagate(tPos, tVel, earthMu, arrivalUt, out Vector3d tArrConic, out _);
            ClosestApproachSolver.Propagate(tPos, tVel, arrivalUt, earthMu, earthJ2, earthRe, pole,
                out Vector3d tArrJ2, out _);

            LambertResult conicAim = LambertSolver.Solve(aIg, tArrConic, tof, earthMu, true, planeNormal);
            LambertResult j2Aim = LambertSolver.Solve(aIg, tArrJ2, tof, earthMu, true, planeNormal);
            AssertTrue("both lambert solves succeed", conicAim.Success && j2Aim.Success);

            double missConicAim = ClosestApproachSolver.MinSeparationOverWindow(
                aIg, conicAim.V1, ignitionUt, arrivalUt, tPos, tVel, 0.0, earthMu, 200, earthJ2, earthRe, pole);
            double missJ2Aim = ClosestApproachSolver.MinSeparationOverWindow(
                aIg, j2Aim.V1, ignitionUt, arrivalUt, tPos, tVel, 0.0, earthMu, 200, earthJ2, earthRe, pole);

            AssertTrue("J2 aim beats conic aim under J2", missJ2Aim < missConicAim);
            AssertTrue("J2 aim at least halves the miss", missJ2Aim < 0.5 * missConicAim);
            Console.WriteLine(string.Format("    conic-aim miss={0:F0} m   J2-aim miss={1:F0} m", missConicAim, missJ2Aim));
        }

        // Phase 2B: re-solving the burn at ignition (from the real measured/J2 state, J2 target, fixed UTs)
        // beats flying the hours-old frozen conic vector. Models a Hohmann: a conic plan made at t=0 is flown
        // from the chaser's REAL (J2) state at a far ignition; re-solving there cuts the miss sharply.
        private static void CheckReSolveAtIgnitionBeatsFrozen()
        {
            Console.WriteLine("Case 35: re-solve at ignition beats the stale frozen conic vector (under J2)");

            const double earthMu = 3.986004418e14, earthRe = 6378136.3, earthJ2 = 1.082636e-3;
            double inc = 51.6 * Math.PI / 180.0;
            Vector3d u = new Vector3d(1, 0, 0);
            Vector3d p = new Vector3d(0, Math.Cos(inc), Math.Sin(inc));
            Vector3d pole = new Vector3d(0, 0, 1);
            Vector3d planeNormal = Vector3d.Cross(u, p).normalized;

            double R1 = earthRe + 250000.0, vc1 = Math.Sqrt(earthMu / R1);
            double R2 = earthRe + 400000.0, vc2 = Math.Sqrt(earthMu / R2);
            double lead = 40.0 * Math.PI / 180.0;
            Vector3d aPos = R1 * u, aVel = vc1 * p;
            Vector3d tPos = R2 * (Math.Cos(lead) * u + Math.Sin(lead) * p);
            Vector3d tVel = vc2 * (-Math.Sin(lead) * u + Math.Cos(lead) * p);

            double ignitionUt = 18000.0, tof = 2000.0, arrivalUt = ignitionUt + tof;

            // Stale plan made at t=0 on CONIC states (what the frozen vector assumed).
            TwoBody.Propagate(aPos, aVel, earthMu, ignitionUt, out Vector3d aIgConic, out Vector3d aIgConicVel);
            TwoBody.Propagate(tPos, tVel, earthMu, arrivalUt, out Vector3d tArrConic, out _);
            LambertResult frozen = LambertSolver.Solve(aIgConic, tArrConic, tof, earthMu, true, planeNormal);

            // Reality at ignition: the chaser is at its J2 state.
            ClosestApproachSolver.Propagate(aPos, aVel, ignitionUt, earthMu, earthJ2, earthRe, pole,
                out Vector3d aIgJ2, out Vector3d aIgJ2Vel);
            ClosestApproachSolver.Propagate(tPos, tVel, arrivalUt, earthMu, earthJ2, earthRe, pole,
                out Vector3d tArrJ2, out _);
            LambertResult resolved = LambertSolver.Solve(aIgJ2, tArrJ2, tof, earthMu, true, planeNormal);
            AssertTrue("both solves succeed", frozen.Success && resolved.Success);

            // Frozen: deliver the stale world-frame dv1 onto the real velocity; fly under J2.
            Vector3d dv1Frozen = frozen.V1 - aIgConicVel;
            double frozenMiss = ClosestApproachSolver.MinSeparationOverWindow(
                aIgJ2, aIgJ2Vel + dv1Frozen, ignitionUt, arrivalUt, tPos, tVel, 0.0, earthMu, 200, earthJ2, earthRe, pole);
            // Re-solve: fresh Lambert from the real state to the J2 target.
            double resolvedMiss = ClosestApproachSolver.MinSeparationOverWindow(
                aIgJ2, resolved.V1, ignitionUt, arrivalUt, tPos, tVel, 0.0, earthMu, 200, earthJ2, earthRe, pole);

            AssertTrue("re-solve beats frozen", resolvedMiss < frozenMiss);
            AssertTrue("re-solve much better (< 0.3x frozen)", resolvedMiss < 0.3 * frozenMiss);
            Console.WriteLine(string.Format("    frozen miss={0:F0} m   re-solved miss={1:F0} m", frozenMiss, resolvedMiss));
        }

        // Dense uniform-sweep ground truth for the closest approach over [t0, t1]: returns the minimum
        // separation and the time of it. Independent of the solver (brute force) for accuracy assertions.
        private static void BruteApproach(
            Vector3d aPos, Vector3d aVel, Vector3d tPos, Vector3d tVel, double mu,
            double t0, double t1, int samples, out double minDistance, out double timeAtMin)
        {
            minDistance = double.PositiveInfinity;
            timeAtMin = t0;
            for (int i = 0; i <= samples; i++)
            {
                double t = t0 + (t1 - t0) * i / samples;
                TwoBody.Propagate(aPos, aVel, mu, t, out Vector3d ra, out _);
                TwoBody.Propagate(tPos, tVel, mu, t, out Vector3d rt, out _);
                double d = (ra - rt).magnitude;
                if (d < minDistance) { minDistance = d; timeAtMin = t; }
            }
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
            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.ForceExecute(RendezvousMethod.Intercept);

            const double maxAccel = 0.5;   // weak engine -> long burn -> axis saturates before reaching planned
            const double dt = 0.2;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            AssertTrue("weak-thrust burn terminates", state.InterceptPhase == InterceptPhase.Coast);
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
            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.UseDistanceForMatchVelocities = false;   // test the immediate null-relVel match (not distance-gated braking)

            const double maxAccel = 50.0;   // near-impulsive engine so both burns finish quickly
            double dt = 0.02;

            // --- intercept burn to completion (as in Case 8) ---
            ex.ForceExecute(RendezvousMethod.Intercept);
            InterceptSolution plan = default(InterceptSolution);
            bool planCaptured = false;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!planCaptured && ex.HasInterceptPlan) { plan = state.InterceptSolution; planCaptured = true; }
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }
            AssertTrue("plan captured", planCaptured);
            AssertTrue("intercept done -> coast", state.InterceptPhase == InterceptPhase.Coast);

            // --- coast to the closest approach the intercept aimed at ---
            while (sim.UniversalTime < plan.ArrivalUt - dt) sim.Advance(dt);
            double relBefore = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            // --- match-velocity burn to completion (explicitly chosen now that stages are independent) ---
            AssertTrue("execute match", ex.ForceExecute(RendezvousMethod.MatchVelocity));
            ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }
            double relAfter = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            AssertTrue("match completed -> coast",
                state.InterceptPhase == InterceptPhase.Coast && state.RendezvousMethod == RendezvousMethod.MatchVelocity);
            AssertTrue("relative velocity nulled (<0.3 m/s)", relAfter < 0.3);
            AssertTrue("match reduced relative speed", relAfter < relBefore);

            Console.WriteLine(string.Format(
                "    relSpeed before={0:F2} after={1:F3} m/s  ticks={2}", relBefore, relAfter, ticks));
        }

        // Velocity-to-go guidance under a FINITE burn: with a weak-ish engine the burn takes several
        // seconds, during which gravity meaningfully rotates the velocity. A frozen-axis cutoff would end
        // at V1 + a large perpendicular (gravity) component → km-scale miss (the in-game 7-18° dir error).
        // Velocity-to-go steers to null (V1 - currentVel), so the burn ends ON V1 and the achieved closest
        // approach stays small. Asserts the post-burn CA collapses vs coasting and is small in absolute m.
        private static void CheckInterceptFiniteBurnAchievesCloseApproach()
        {
            Console.WriteLine("Case 17: finite-burn intercept collapses CA and terminates (frozen-axis; gravity leaves a bounded perpendicular residual)");

            double altitude = 250000.0;
            double R = KerbinRadius + altitude;
            SimWorld sim = MakeCatchupSim(altitude, 15.0);

            double coastCA = MinSeparation(
                sim.ActivePosition, sim.ActiveVelocity, sim.TargetPosition, sim.TargetVelocity,
                KerbinMu, CircularPeriod(R), 720);

            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.ForceExecute(RendezvousMethod.Intercept);

            const double maxAccel = 8.0;   // a ~30 m/s burn takes ~4 s -> gravity acts appreciably during it
            const double dt = 0.05;
            InterceptSolution plan = default(InterceptSolution);
            bool captured = false;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 1000000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!captured && ex.HasInterceptPlan) { plan = state.InterceptSolution; captured = true; }
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            AssertTrue("plan captured", captured);
            AssertTrue("burn completed -> coast", state.InterceptPhase == InterceptPhase.Coast);

            double remaining = Math.Max(plan.ArrivalUt - sim.UniversalTime, 1.0);
            double postCA = MinSeparation(
                sim.ActivePosition, sim.ActiveVelocity, sim.TargetPosition, sim.TargetVelocity,
                KerbinMu, remaining, 1440);

            AssertTrue("finite burn collapses CA vs coast", postCA < coastCA * 0.05);
            // Frozen-axis steering does not null the perpendicular velocity gravity adds over a multi-second
            // burn, so a single intercept on this gravity-heavy geometry leaves a few-km residual (match
            // velocity / a re-plan trims it). The point of this case is that the burn TERMINATES cleanly and
            // collapses CA by >95% — not sub-km precision, which only the (unstable, removed) re-aim gave.
            AssertTrue("finite-burn CA bounded (<8 km; residual for match/re-plan)", postCA < 8000.0);

            Console.WriteLine(string.Format(
                "    maxAccel={0} m/s^2  coastCA={1:F0} m  postCA={2:F0} m  ticks={3} ({4:F0}s)",
                maxAccel, coastCA, postCA, ticks, ticks * dt));
        }

        // Validates the ignition-time-drift correction: with a lead set, the executor must plan from the
        // active state COASTED FORWARD by the lead, and reference the arrival to that ignition. Verified by
        // (1) the plan's ignition UT = now + lead, and (2) flying the planned transfer FROM the coasted
        // ignition state lands exactly on the target's predicted arrival position. With lead applied, the
        // ΔV is measured against the ignition velocity (not the earlier measured velocity).
        private static void CheckInterceptIgnitionLead()
        {
            Console.WriteLine("Case 16: intercept ignition-lead drift correction");

            SimWorld sim = MakeCatchupSim(250000.0, 15.0);
            double lead = 120.0;

            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.IgnitionLeadSeconds = lead;
            ex.ForceExecute(RendezvousMethod.Intercept);
            ex.Update(sim);   // arms + freezes the plan using the lead
            InterceptSolution plan = state.InterceptSolution;

            AssertTrue("plan success", plan.Success);
            AssertScalar("ignition UT = now + lead", plan.IgnitionUt, sim.UniversalTime + lead);

            // Coast the chaser to the ignition state, then fly the planned departure for the time of flight.
            TwoBody.Propagate(sim.ActivePosition, sim.ActiveVelocity, KerbinMu, lead,
                out Vector3d ignPos, out Vector3d ignVel);
            TwoBody.Propagate(ignPos, plan.TransferDepartureVelocity, KerbinMu, plan.TimeOfFlight,
                out Vector3d landed, out _);

            // Target predicted position at arrival (2-body from its measured state, as the executor uses).
            TwoBody.Propagate(sim.TargetPosition, sim.TargetVelocity, KerbinMu, plan.ArrivalUt - sim.UniversalTime,
                out Vector3d targetArrival, out _);

            AssertVecRel("transfer (from ignition) lands on target", landed, targetArrival, 1e-6);

            // ΔV is referenced to the ignition velocity, not the measured velocity.
            AssertVecRel("plan ΔV = departureV - ignitionV", plan.DeltaV,
                plan.TransferDepartureVelocity - ignVel, 1e-6);

            Console.WriteLine(string.Format(
                "    lead={0:F0}s  ignUT={1:F0}  dV={2:F2} m/s  tof={3:F0}s  predCA={4:E2} m",
                lead, plan.IgnitionUt, plan.DeltaVMagnitude, plan.TimeOfFlight, plan.PredictedClosestApproach));
        }

        // End-to-end Step 7: with the executor advanced to the CloseApproach stage, drive the closing-
        // velocity controller against a fresh clean geometry — chaser 800 m behind a target on a circular
        // orbit, already matched — and confirm it parks within the standoff band, matched, and ends the
        // sequence (Phase.Complete = control handed back). The close stage is stateless, so feeding it a
        // different world than the one used to reach the stage is valid.
        private static void CheckCloseApproachParksAtStandoff()
        {
            Console.WriteLine("Case 15: close-approach parks at standoff and matches (stepping sim)");

            TerminalRendezvousExecutor ex = AdvanceToCloseStage(out SharedState state);
            AssertTrue("ready after match (coast)", state.InterceptPhase == InterceptPhase.Coast);

            double R = KerbinRadius + 200000.0;
            double vc = Math.Sqrt(KerbinMu / R);
            SimWorld sim = new SimWorld(KerbinMu,
                new Vector3d(R, -800.0, 0.0), new Vector3d(0.0, vc, 0.0),   // chaser 800 m behind, matched
                new Vector3d(R, 0.0, 0.0), new Vector3d(0.0, vc, 0.0),
                new Vector3d(0, 0, 1));

            double rangeBefore = (sim.TargetPosition - sim.ActivePosition).magnitude;

            // Park at 100 m for this test (independent of the default), so the standoff-band asserts are meaningful.
            ex.ParkingDistance = 100.0;

            AssertTrue("execute close", ex.ForceExecute(RendezvousMethod.FinalApproach));
            const double maxAccel = 20.0;
            double dt = 0.05;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 1000000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            double rangeAfter = (sim.TargetPosition - sim.ActivePosition).magnitude;
            double relAfter = (sim.ActiveVelocity - sim.TargetVelocity).magnitude;

            AssertTrue("close completed (rendezvous done)", state.InterceptPhase == InterceptPhase.Complete);
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
        // Build an executor wired to a fresh SharedState (Init + Reset), the way BlackBird does in-game.
        private static TerminalRendezvousExecutor NewExecutor(out SharedState state)
        {
            state = new SharedState();
            state.InterceptMethod = InterceptMethod.SinglePhase;   // these cases exercise the single-rev intercept
            TerminalRendezvousExecutor ex = new TerminalRendezvousExecutor();
            ex.Init(state);
            ex.Reset();
            return ex;
        }

        private static TerminalRendezvousExecutor AdvanceToCloseStage(out SharedState state)
        {
            SimWorld sim = MakeCatchupSim(250000.0, 15.0);
            TerminalRendezvousExecutor ex = NewExecutor(out state);
            const double maxAccel = 50.0;
            double dt = 0.02;

            ex.ForceExecute(RendezvousMethod.Intercept);
            InterceptSolution plan = default(InterceptSolution);
            bool captured = false;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (!captured && ex.HasInterceptPlan) { plan = state.InterceptSolution; captured = true; }
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }

            while (sim.UniversalTime < plan.ArrivalUt - dt) sim.Advance(dt);   // coast to CA

            // Immediate null-relVel match to reach the close stage; restore the default afterwards so the
            // caller's close-approach run still uses the distance-gated behavior.
            ex.UseDistanceForMatchVelocities = false;
            ex.ForceExecute(RendezvousMethod.MatchVelocity);
            ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 500000)
            {
                RendezvousCommand cmd = ex.Update(sim);
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (cmd.HasBurn)
                    sim.ApplyDeltaV(cmd.ThrustDirection.normalized * (cmd.Throttle * maxAccel * dt));
                sim.Advance(dt);
            }
            ex.UseDistanceForMatchVelocities = true;

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
            TerminalRendezvousExecutor ex = NewExecutor(out SharedState state);
            ex.ForceExecute(RendezvousMethod.Intercept);

            // First update arms the burn and freezes the plan; the baseline is the active velocity now.
            ex.Update(w);
            Vector3d baseline = w.ActiveVelocity;
            Vector3d dvUnit = state.InterceptSolution.DeltaV.normalized;
            double planned = state.InterceptSolution.DeltaVMagnitude;
            AssertTrue("plan armed", ex.HasInterceptPlan && planned > 2.0);

            // Deliver a fast chunk to 1 m/s short of planned (well past the stall-arm threshold), then
            // creep asymptotically toward a ceiling 0.5 m/s short of planned — never reaching cutoff.
            w.AddVelocity(dvUnit * (planned - 1.0));
            double ceiling = planned - 0.5;

            double dt = 0.1;
            int ticks = 0;
            while (state.InterceptPhase == InterceptPhase.Executing && ticks++ < 100000)
            {
                ex.Update(w);
                if (state.InterceptPhase != InterceptPhase.Executing) break;
                if (ticks % 10 == 0)   // a tiny, shrinking new maximum roughly once per second
                {
                    double delivered = Vector3d.Dot(w.ActiveVelocity - baseline, dvUnit);
                    double gap = ceiling - delivered;
                    if (gap > 0.0) w.AddVelocity(dvUnit * (gap * 0.5));
                }
                w.Advance(dt);
            }

            double finalDelivered = Vector3d.Dot(w.ActiveVelocity - baseline, dvUnit);
            AssertTrue("creep burn terminates", state.InterceptPhase == InterceptPhase.Coast);
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
            public double BodyRadius => 0.0;
            public double AtmosphereDepth => 0.0;
            public double J2 => 0.0;
            public double J2ReferenceRadius => 0.0;
            public Vector3d Pole => Vector3d.zero;

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
            public double BodyRadius => 0.0;
            public double AtmosphereDepth => 0.0;
            public double J2 => 0.0;
            public double J2ReferenceRadius => 0.0;
            public Vector3d Pole => Vector3d.zero;

            public void AddVelocity(Vector3d dv) { _activeVel += dv; }
            public void Advance(double dt) { _ut += dt; }
        }

        // Fixed-state world for single-tick controller checks (no propagation) — set the vectors directly.
        private sealed class StaticWorld : IRendezvousWorld
        {
            public double UniversalTime { get; set; }
            public double Mu { get; set; }
            public Vector3d ActivePosition { get; set; }
            public Vector3d ActiveVelocity { get; set; }
            public Vector3d TargetPosition { get; set; }
            public Vector3d TargetVelocity { get; set; }
            public Vector3d ReferenceNormal { get; set; }
            public double BodyRadius { get; set; }
            public double AtmosphereDepth { get; set; }
            public double J2 { get; set; }
            public double J2ReferenceRadius { get; set; }
            public Vector3d Pole { get; set; }
        }

        // Case 18: docking-port geometry decomposition. Known geometry, then a rotation-invariance check
        // that the along-axis / lateral split is real, not axis luck (mirrors Cases 1/2). Target port faces
        // +X; the chaser sits 10 m in front of the face and 2 m off the centerline, its own port facing back
        // down the axis (the mated heading), with a 50 m standoff waypoint.
        private static void CheckDockingGeometry()
        {
            Console.WriteLine("Case 18: docking-port geometry (axial/lateral split + alignment)");

            const double standoff = 50.0;

            // Axis-aligned: target port at origin facing +X, chaser 10 m ahead / 2 m to +Y, facing -X.
            PortState targetA = new PortState(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0));
            PortState chaserA = new PortState(new Vector3d(10, 2, 0), new Vector3d(-1, 0, 0));
            DockingGeometry a = DockingGeometry.Compute(chaserA, targetA, standoff);

            AssertScalar("axial", a.AxialDistance, 10.0);
            AssertScalar("lateral", a.LateralOffset, 2.0);
            AssertVec("lateral dir", a.LateralDirection, new Vector3d(0, 1, 0));
            AssertVec("waypoint", a.ApproachWaypoint, new Vector3d(50, 0, 0));
            AssertScalar("aligned err", a.AlignmentErrorDeg, 0.0);
            AssertScalar("range", a.Range, Math.Sqrt(104.0));

            // Misaligned heading: chaser port rotated 90 deg off the mated (-axis) heading.
            PortState chaserMis = new PortState(new Vector3d(10, 2, 0), new Vector3d(0, -1, 0));
            DockingGeometry mis = DockingGeometry.Compute(chaserMis, targetA, standoff);
            AssertScalar("misaligned err", mis.AlignmentErrorDeg, 90.0);

            // Rotation invariance: rotate the axis-aligned setup +90 deg about +Z ((x,y)->(-y,x)). Scalars
            // MUST be unchanged; the world vectors rotate with the frame.
            PortState targetR = new PortState(new Vector3d(0, 0, 0), new Vector3d(0, 1, 0));
            PortState chaserR = new PortState(new Vector3d(-2, 10, 0), new Vector3d(0, -1, 0));
            DockingGeometry r = DockingGeometry.Compute(chaserR, targetR, standoff);

            AssertScalar("rot axial", r.AxialDistance, 10.0);
            AssertScalar("rot lateral", r.LateralOffset, 2.0);
            AssertVec("rot lateral dir", r.LateralDirection, new Vector3d(-1, 0, 0));
            AssertVec("rot waypoint", r.ApproachWaypoint, new Vector3d(0, 50, 0));
            AssertScalar("rot aligned err", r.AlignmentErrorDeg, 0.0);
        }

        // Case 19: docking controller. Target port at origin facing +X; the chaser port faces -X (the mated
        // head-on heading). Checks that translation steers toward the on-axis goal, that a leg completes only
        // when arrived AND aligned AND stopped, that Contact completes within capture range, and leg order.
        private static void CheckDockingController()
        {
            Console.WriteLine("Case 19: docking controller (translation + leg gates)");

            PortState target = new PortState(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0));

            // Approach leg, chaser off-axis and beyond the 25 m waypoint: translation points toward (25,0,0).
            PortState chaserFar = new PortState(new Vector3d(40, 5, 0), new Vector3d(-1, 0, 0));
            DockingCommand approach = DockingController.Compute(chaserFar, target, Vector3d.zero, DockingLeg.Approach);
            Vector3d toWaypoint = new Vector3d(25, 0, 0) - new Vector3d(40, 5, 0);
            AssertTrue("approach steers to waypoint", Vector3d.Dot(approach.TranslationVelocityWorld, toWaypoint) > 0.0);
            AssertTrue("approach not complete (far)", !approach.LegComplete);
            AssertVec("mated facing", approach.FacingWorld, new Vector3d(-1, 0, 0));
            AssertScalar("axial", approach.AxialDistance, 40.0);
            AssertScalar("lateral", approach.LateralOffset, 5.0);

            // Arrived at the waypoint, aligned, stopped -> Approach complete.
            PortState chaserAtWp = new PortState(new Vector3d(25.0, 0.3, 0), new Vector3d(-1, 0, 0));
            AssertTrue("approach complete at waypoint",
                DockingController.Compute(chaserAtWp, target, Vector3d.zero, DockingLeg.Approach).LegComplete);

            // Same spot but mis-pointed, or still moving -> NOT complete (must be aligned AND nearly stopped).
            PortState chaserMis = new PortState(new Vector3d(25.0, 0.3, 0), new Vector3d(0, 1, 0));
            AssertTrue("misaligned blocks completion",
                !DockingController.Compute(chaserMis, target, Vector3d.zero, DockingLeg.Approach).LegComplete);
            AssertTrue("motion blocks completion",
                !DockingController.Compute(chaserAtWp, target, new Vector3d(0, 0, 0.5), DockingLeg.Approach).LegComplete);

            // Contact leg: within capture range -> complete.
            PortState chaserContact = new PortState(new Vector3d(0.4, 0, 0), new Vector3d(-1, 0, 0));
            AssertTrue("contact at capture range",
                DockingController.Compute(chaserContact, target, Vector3d.zero, DockingLeg.Contact).LegComplete);

            AssertTrue("Approach -> Final", DockingController.NextLeg(DockingLeg.Approach) == DockingLeg.Final);
            AssertTrue("Final -> Contact", DockingController.NextLeg(DockingLeg.Final) == DockingLeg.Contact);
        }

        // Case 20: the Match-Velocity-vs-Final-Approach split (regression guard for the bug where "Match
        // velocities at X" ran Final Approach and burned TOWARD the target). MV-at-distance must HOLD
        // retrograde and never chase; Final Approach on the same geometry must actively command thrust (close).
        private static void CheckMatchVelocityAtDistanceHoldsWhileFinalApproachChases()
        {
            Console.WriteLine("Case 20: match-velocity-at-distance holds retrograde; final approach acts/closes");

            // 11 km out, closing straight at the target at 148 m/s (the in-game geometry of the bug).
            StaticWorld world = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                BodyRadius = KerbinRadius, AtmosphereDepth = 70000.0,
                TargetPosition = new Vector3d(11000.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = new Vector3d(0, 100.0, 0), ActiveVelocity = new Vector3d(148.0, 100.0, 0)
            };
            Vector3d bearing = new Vector3d(1, 0, 0);

            // Match Velocity + "at X" checked: far from the brake point -> HOLD retrograde, zero throttle, and
            // crucially NOT toward the target (CA fed in band; MV must ignore it, not chase).
            TerminalRendezvousExecutor mv = NewExecutor(out _);
            mv.UseDistanceForMatchVelocities = true;
            mv.BrakingDecelMetersPerSecondSquared = 5.0;
            mv.ForceExecute(RendezvousMethod.MatchVelocity);
            RendezvousCommand hold = mv.Update(world, 50.0, 60.0);
            AssertScalar("MV holds (zero throttle) far from brake point", hold.Throttle, 0.0);
            AssertTrue("MV oriented retrograde, NOT toward target", Vector3d.Dot(hold.ThrustDirection, bearing) < 0.0);

            // Final Approach (box unchecked): same geometry must actively control the approach (nonzero throttle).
            TerminalRendezvousExecutor fa = NewExecutor(out _);
            fa.UseDistanceForMatchVelocities = false;
            fa.BrakingDecelMetersPerSecondSquared = 5.0;
            fa.ForceExecute(RendezvousMethod.FinalApproach);
            RendezvousCommand chase = fa.Update(world, 50.0, 60.0);
            AssertTrue("FA acts (nonzero throttle)", chase.Throttle > 0.0);
        }

        // Case 23: Match-Velocity-at-distance fires the kill-velocity burn at the brake point and inside the
        // band. The brake point comes from the measured closing speed; it self-latches once reached.
        private static void CheckMatchVelocityAtDistanceBrakesAtTrigger()
        {
            Console.WriteLine("Case 23: match-velocity-at-distance holds until the physical brake point (no early fire)");

            // decel 5, closing 30 -> stoppingDistance = 30^2/(2*5) = 90; brake point = ParkingDistance(100)+90 = 190 m.
            // The brake point is physics-only (D + v^2/2a) with NO flip-DISTANCE budget; the flip-to-retro happens
            // for free during the pre-oriented hold. So FAR OUT (1000 m) it must HOLD (throttle 0), not fire early
            // (the old D + v^2/2a + v*slewLead froze a cold 180-degree slew estimate in and braked minutes early).
            Vector3d bearing = new Vector3d(1, 0, 0);
            StaticWorld far = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                BodyRadius = KerbinRadius, AtmosphereDepth = 70000.0,
                TargetPosition = new Vector3d(1000.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = new Vector3d(0, 100.0, 0), ActiveVelocity = new Vector3d(30.0, 100.0, 0)
            };
            TerminalRendezvousExecutor hold = NewExecutor(out _);
            hold.UseDistanceForMatchVelocities = true;
            hold.ParkingDistance = 100.0;
            hold.BrakingDecelMetersPerSecondSquared = 5.0;
            hold.ForceExecute(RendezvousMethod.MatchVelocity);
            hold.Update(far, 900.0, 30.0);                               // tick 1: set the brake point
            RendezvousCommand held = hold.Update(far, 900.0, 30.0);      // tick 2: 1000 m > 190 m brake point
            AssertTrue("holds far out (no early brake)", held.Throttle == 0.0);
            AssertTrue("holds retrograde while waiting", Vector3d.Dot(held.ThrustDirection, bearing) < 0.0);

            // At the brake point (180 m <= 190 m): fire retrograde.
            StaticWorld near = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                BodyRadius = KerbinRadius, AtmosphereDepth = 70000.0,
                TargetPosition = new Vector3d(180.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = new Vector3d(0, 100.0, 0), ActiveVelocity = new Vector3d(30.0, 100.0, 0)
            };
            TerminalRendezvousExecutor mv = NewExecutor(out _);
            mv.UseDistanceForMatchVelocities = true;
            mv.ParkingDistance = 100.0;
            mv.BrakingDecelMetersPerSecondSquared = 5.0;
            mv.ForceExecute(RendezvousMethod.MatchVelocity);
            mv.Update(near, 80.0, 6.0);                            // tick 1: set the brake point, hold
            RendezvousCommand brake = mv.Update(near, 80.0, 6.0);  // tick 2: inside the brake point -> fire
            AssertTrue("brakes at the trigger (nonzero throttle)", brake.Throttle > 0.0);
            AssertTrue("brake is retrograde", Vector3d.Dot(brake.ThrustDirection, bearing) < 0.0);

            // Inside the parking band (8 m): fire immediately.
            StaticWorld haven = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                BodyRadius = KerbinRadius, AtmosphereDepth = 70000.0,
                TargetPosition = new Vector3d(8.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = new Vector3d(0, 100.0, 0), ActiveVelocity = new Vector3d(2.0, 100.0, 0)
            };
            TerminalRendezvousExecutor hav = NewExecutor(out _);
            hav.UseDistanceForMatchVelocities = true;
            hav.ForceExecute(RendezvousMethod.MatchVelocity);
            RendezvousCommand burn = hav.Update(haven, 8.0, 60.0);
            AssertTrue("fires inside the parking band", burn.Throttle > 0.0);
        }

        // Case 24: ThrustEnvelope bins POSITIVE authority into each of the 6 directions and reads it back.
        // This guards the sign convention: a thruster pushing +x must register as Right (not negative Left),
        // and GetMagnitude in a direction must equal the authority pushing that way. (The original < 0 binning
        // made every bin negative, so the RcsController's `orAvail > 0` gate saw zero authority and never
        // commanded translation — this case fails loudly on that.)
        private static void CheckThrustEnvelope()
        {
            Console.WriteLine("Case 24: thrust envelope bins positive authority + GetMagnitude");

            ThrustEnvelope e = new ThrustEnvelope();
            e.Add(new Vector3d(10, 0, 0));   // thrust pushing +x (right)
            e.Add(new Vector3d(0, 0, -4));   // pushing -z (back)
            e.Add(new Vector3d(0, 6, 0));    // pushing +y (up)

            AssertScalar("right bin = 10", e[ThrustEnvelope.Orientation.RIGHT], 10.0);
            AssertScalar("back bin = 4", e[ThrustEnvelope.Orientation.BACK], 4.0);
            AssertScalar("up bin = 6", e[ThrustEnvelope.Orientation.UP], 6.0);
            AssertScalar("left bin = 0 (no -x thrust)", e[ThrustEnvelope.Orientation.LEFT], 0.0);

            AssertScalar("GetMagnitude(+x) = 10", e.GetMagnitude(new Vector3d(1, 0, 0)), 10.0);
            AssertScalar("GetMagnitude(back) = 4", e.GetMagnitude(new Vector3d(0, 0, -1)), 4.0);
            AssertScalar("GetMagnitude(-x) = 0 (no left authority)", e.GetMagnitude(new Vector3d(-1, 0, 0)), 0.0);
        }

        // Case 25: DockingSchedule entry-step selection, transitions, and the approach speed schedule (pure).
        // Covers the "behind" threshold fix (|zSep| > halfBox => switch sides, not back straight up) and that
        // the final Docking step actually closes toward the port at a capped speed.
        private static void CheckDockingSchedule()
        {
            Console.WriteLine("Case 25: docking schedule entry-step, transitions, and approach speeds");

            DockingConfig c = new DockingConfig
            {
                SafeDistance = 20.0, TargetSize = 5.0, AcquireRange = 0.3,
                DockingCorridorRadius = 1.0, SpeedLimit = 1.0, VesselBoundingSize = 4.0
            };
            Vector3d axis = new Vector3d(1, 0, 0);

            // Entry-step selection.
            DockingGeom front = new DockingGeom { ZSep = 50, LateralMag = 0.2, ZAxis = axis, LateralDir = Vector3d.zero };
            AssertTrue("on-axis in front -> Docking", DockingSchedule.PickEntryStep(front, c) == DockingSteps.Docking);

            DockingGeom offAxis = new DockingGeom { ZSep = 50, LateralMag = 5, ZAxis = axis, LateralDir = new Vector3d(0, 1, 0) };
            AssertTrue("off-axis far in front -> MovingToStart", DockingSchedule.PickEntryStep(offAxis, c) == DockingSteps.MovingToStart);

            DockingGeom behindFar = new DockingGeom { ZSep = -10, LateralMag = 1, ZAxis = axis, LateralDir = new Vector3d(0, 1, 0) };
            AssertTrue("far behind (>halfBox) -> WrongSideBackingUp", DockingSchedule.PickEntryStep(behindFar, c) == DockingSteps.WrongSideBackingUp);

            DockingGeom behindClose = new DockingGeom { ZSep = -1, LateralMag = 0.2, ZAxis = axis, LateralDir = Vector3d.zero };
            AssertTrue("just behind (<halfBox) -> BackingUp", DockingSchedule.PickEntryStep(behindClose, c) == DockingSteps.BackingUp);

            // Transitions.
            DockingGeom backedUp = new DockingGeom { ZSep = 6, LateralMag = 0.5, ZAxis = axis, LateralDir = Vector3d.zero };
            AssertTrue("BackingUp past targetSize -> MovingToStart",
                DockingSchedule.Advance(DockingSteps.BackingUp, backedUp, c) == DockingSteps.MovingToStart);

            DockingGeom contact = new DockingGeom { ZSep = 0.2, LateralMag = 0.1, ZAxis = axis, LateralDir = Vector3d.zero };
            AssertTrue("Docking within acquire range -> Off",
                DockingSchedule.Advance(DockingSteps.Docking, contact, c) == DockingSteps.Off);

            // Approach speeds: the Docking step closes toward the port at a positive, capped speed.
            Func<Vector3d, double> accel = _ => 1.0;   // 1 m/s^2 available in every direction
            DockingGeom closing = new DockingGeom { ZSep = 20, LateralMag = 0.0, ZAxis = axis, LateralDir = Vector3d.zero };
            DockingPlan plan = DockingSchedule.Plan(DockingSteps.Docking, closing, c, accel);
            AssertTrue("docking closes (z speed > 0)", plan.ZSpeed > 0.0);
            AssertTrue("docking z speed capped at limit", plan.ZSpeed <= c.SpeedLimit + 1e-9);
            AssertTrue("adjustment points toward the port (+zAxis)", Vector3d.Dot(plan.Adjustment, axis) > 0.0);
            AssertTrue("docking aligns to the port", plan.Align);
        }

        // Case 21: the close-approach velocity deadband relaxes with range. The SAME small closing-velocity
        // error (0.4 m/s off the commanded 5 m/s) must be tolerated (hold, no burn) when 4 km out, but still
        // corrected when 200 m out — so we stop micro-burning to hold an exact velocity at km distance.
        private static void CheckCloseApproachDeadbandRelaxesWithRange()
        {
            Console.WriteLine("Case 21: close-approach deadband relaxes with range");

            // Closing along +X at 4.6 m/s vs a commanded 5.0 m/s cap => a 0.4 m/s gap (under the 4 km deadband
            // ~2.0, over the 200 m deadband ~0.1). CA fed out of band so the closing controller is the one used.
            Vector3d targetVel = new Vector3d(0, 100.0, 0);
            Vector3d activeVel = new Vector3d(4.6, 100.0, 0);

            StaticWorld far = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                TargetPosition = new Vector3d(4000.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = targetVel, ActiveVelocity = activeVel
            };
            TerminalRendezvousExecutor farExec = NewExecutor(out _);
            farExec.ForceExecute(RendezvousMethod.FinalApproach);
            RendezvousCommand farCmd = farExec.Update(far, 2000.0, 600.0);
            AssertScalar("4 km: holds (no micro-burn)", farCmd.Throttle, 0.0);

            StaticWorld near = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                TargetPosition = new Vector3d(200.0, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = targetVel, ActiveVelocity = activeVel
            };
            TerminalRendezvousExecutor nearExec = NewExecutor(out _);
            nearExec.ForceExecute(RendezvousMethod.FinalApproach);
            RendezvousCommand nearCmd = nearExec.Update(near, 2000.0, 600.0);
            AssertTrue("200 m: still corrects", nearCmd.Throttle > 0.0);
        }

        // Case 22: immediate match velocity (distance gating off) steers opposite the CURRENT relative velocity
        // every tick, so the commanded thrust direction tracks relVel as it changes. (The earlier direction-LOCK
        // when slow was removed; StepMatchVelocity re-aims each tick now.)
        private static void CheckMatchVelocityReaimsEachTick()
        {
            Console.WriteLine("Case 22: match velocity re-aims opposite relative velocity each tick");

            StaticWorld world = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                TargetPosition = new Vector3d(1000, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = Vector3d.zero, ActiveVelocity = new Vector3d(0, 2.0, 0)   // relVel (0,2,0): 2 m/s
            };

            TerminalRendezvousExecutor exec = NewExecutor(out _);
            exec.UseDistanceForMatchVelocities = false;   // immediate null, not distance-gated braking
            exec.ForceExecute(RendezvousMethod.MatchVelocity);

            // Thrust opposes relVel: (0,2,0) -> (0,-1,0).
            RendezvousCommand fast = exec.Update(world);
            AssertVec("aims opposite relVel (fast)", fast.ThrustDirection, new Vector3d(0, -1, 0));

            // relVel now points a different way: thrust re-aims to oppose it, (0.3,0,0) -> (-1,0,0).
            world.ActiveVelocity = new Vector3d(0.3, 0, 0);   // relVel (0.3,0,0): 0.3 m/s
            RendezvousCommand slow = exec.Update(world);
            AssertVec("re-aims opposite relVel (slow)", slow.ThrustDirection, new Vector3d(-1, 0, 0));
        }

        // Case 22b: the ONLY MV change — once near the null, if the burn OVERSHOOTS (relVel reverses so the null
        // direction would flip past 90°), the stage completes instead of turning around and firing a second burn
        // back the other way (which added speed). Re-aiming for smaller corrections is unchanged (Case 22).
        private static void CheckMatchVelocityStopsOnOvershootNoSecondBurn()
        {
            Console.WriteLine("Case 22b: match velocity stops on overshoot (no 180 flip / second burn)");

            StaticWorld world = new StaticWorld
            {
                Mu = KerbinMu, ReferenceNormal = new Vector3d(0, 0, 1),
                TargetPosition = new Vector3d(1000, 0, 0), ActivePosition = Vector3d.zero,
                TargetVelocity = Vector3d.zero, ActiveVelocity = new Vector3d(0, 0.5, 0)   // near null (0.5 m/s)
            };

            TerminalRendezvousExecutor exec = NewExecutor(out SharedState state);
            exec.UseDistanceForMatchVelocities = false;
            exec.ForceExecute(RendezvousMethod.MatchVelocity);

            // Ignition captures the null axis (0,-1,0) and burns (0.5 m/s > the 0.15 completion tolerance).
            RendezvousCommand fire = exec.Update(world);
            AssertVec("burns to null (captures axis)", fire.ThrustDirection, new Vector3d(0, -1, 0));
            AssertTrue("still executing before overshoot", state.InterceptPhase == InterceptPhase.Executing);

            // Overshoot: the burn pushed relVel past zero so it now points the OTHER way. The re-aim would flip
            // ~180° — instead the stage must COMPLETE (drop to Coast) and issue no burn, not chase it back.
            world.ActiveVelocity = new Vector3d(0, -0.4, 0);   // relVel reversed (0.4 m/s the other way)
            RendezvousCommand after = exec.Update(world);
            AssertTrue("completes on overshoot (-> Coast)", state.InterceptPhase == InterceptPhase.Coast);
            AssertTrue("no second (flipped) burn", !after.HasBurn);
        }

        private static double CircularPeriod(double radius)
        {
            return 2.0 * Math.PI * Math.Sqrt(radius * radius * radius / KerbinMu);
        }

        // The "still before burn" gate now waits for the rotation rate to PLATEAU (stop dropping) while pointed,
        // rather than clearing a fixed rate floor. This self-calibrates to each craft's real limit-cycle floor:
        // the weak craft that used to hang in "Stabilizing" forever (its jitter never cleared the 0.1 floor) now
        // arms, while nothing ignites mid-swing (above the loose ceiling) or while still slewing to the vector.
        private static void CheckBurnSettleGate()
        {
            Console.WriteLine("Case 36: burn settle gate (rate-plateau ignition)");

            const double dt = 0.02;   // physics step

            // The "improvement" deadband scales with control authority (finer for a weak craft) and falls back to
            // the default noise floor when torque authority is unknown.
            double dbNimble = BurnSettleGate.RateImproveDeadbandDegPerSec(5.0, dt);    // alpha 5 rad/s^2
            double dbWeak = BurnSettleGate.RateImproveDeadbandDegPerSec(0.02, dt);     // alpha 0.02 rad/s^2
            double dbUnknown = BurnSettleGate.RateImproveDeadbandDegPerSec(0.0, dt);   // no torque data
            AssertTrue("weak-craft deadband finer than nimble", dbWeak < dbNimble);
            AssertScalar("unknown-authority deadband = default noise floor", dbUnknown, BurnSettleGate.DefaultRateImproveDeadbandDegPerSec);

            // THE BUG FIX: a weak craft limit-cycling at ~0.2 deg/s (which the old 0.1 floor rejected forever) must
            // now arm, because ignition waits for the rate to plateau, not to clear a floor it can never reach.
            {
                var t = new BurnSettleTracker();
                bool armed = false;
                double rate = 0.20;
                for (double now = 0.0; now <= 3.0 && !armed; now += dt)
                {
                    rate = rate >= 0.205 ? 0.20 : 0.21;   // limit-cycle jitter around 0.2 deg/s, no net improvement
                    armed = t.Update(0.5, rate, dbWeak, now);
                }
                AssertTrue("weak craft (~0.2 deg/s jitter) eventually arms", armed);
            }

            // Never arms while still slewing to the burn vector (pointing error above AlignStartDeg), however
            // steady the rotation looks — the plateau only counts once we are ON the target.
            {
                var t = new BurnSettleTracker();
                bool armed = false;
                for (double now = 0.0; now <= 3.0; now += dt)
                    armed = t.Update(5.0, 0.01, dbWeak, now);   // 5 deg off axis, near-zero rate
                AssertTrue("does not arm while slewing (error > 1 deg)", !armed);
            }

            // Dwell: a freshly-plateaued craft must hold the plateau for StabilizeDwellSeconds before it arms.
            {
                var t = new BurnSettleTracker();
                bool early = false, late = false;
                for (double now = 0.0; now < BurnSettleGate.StabilizeDwellSeconds - 0.1; now += dt)
                    early = t.Update(0.5, 0.03, dbWeak, now);
                for (double now = BurnSettleGate.StabilizeDwellSeconds - 0.1; now <= BurnSettleGate.StabilizeDwellSeconds + 0.2; now += dt)
                    late = late || t.Update(0.5, 0.03, dbWeak, now);
                AssertTrue("not armed before the dwell elapses", !early);
                AssertTrue("arms after a continuous plateau dwell", late);
            }

            // Never fires mid-swing: even a perfectly steady (plateaued) rate above the ceiling must NOT arm.
            {
                var t = new BurnSettleTracker();
                bool armed = false;
                for (double now = 0.0; now <= 3.0; now += dt)
                    armed = armed || t.Update(0.5, 3.0, dbWeak, now);   // constant 3 deg/s: plateaus, but > 1 deg/s ceiling
                AssertTrue("does not arm above the mid-swing ceiling", !armed);
            }

            // A transient rotation spike does not prematurely arm inside the dwell window.
            {
                var t = new BurnSettleTracker();
                t.Update(0.5, 0.03, dbWeak, 0.0);       // pointed and momentarily steady
                t.Update(0.5, 5.0, dbWeak, 0.02);       // brief spike (still pointed)
                bool armedSoon = t.Update(0.5, 0.03, dbWeak, 0.04);
                AssertTrue("transient spike does not arm within the dwell", !armedSoon);
            }

            // Hysteresis: once armed, small drift (< AlignKeepDeg) holds the burn; a large excursion re-orients.
            {
                var t = new BurnSettleTracker();
                bool armed = false;
                for (double now = 0.0; now <= 2.0 && !armed; now += dt)
                    armed = t.Update(0.5, 0.03, dbWeak, now);
                AssertTrue("armed after plateau", armed);
                bool holdsThroughDrift = t.Update(5.0, 0.03, dbWeak, 2.1);    // 5 deg < keep band
                bool dropsOnExcursion = t.Update(25.0, 0.03, dbWeak, 2.2);    // 25 deg > keep band
                AssertTrue("armed burn holds through small drift", holdsThroughDrift);
                AssertTrue("large excursion forces re-orient", !dropsOnExcursion);
            }
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
