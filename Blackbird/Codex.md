BlackBird Codex Handoff
=======================

Last updated: 2026-06-13

Session Context
---------------

Codex does not have direct access to other chatgpt.com or VS Code chat sessions.

This file is the durable handoff. A future Codex session should treat this file plus
the repository state as the source of truth, not assume hidden chat history exists.


Project Summary
---------------

BlackBird is a Kerbal Space Program launch planning and ascent rendezvous addon.

Development testbed:
- Stock KSP.

Primary target environment:
- RSS + Principia.

User workflow goal:
1. User selects a target vessel already in orbit.
2. BlackBird computes launch windows and selectable launch/rendezvous candidates.
3. User chooses a candidate.
4. BlackBird guides or flies ascent into the planned insertion orbit.
5. Result should be near rendezvous with minimal correction burns.

The planner should know the intended rendezvous strategy before launch. It should
not invent the strategy only after reaching orbit.


Hard Rules
----------

Do not violate these:

- No placeholder implementations.
- No temporary algorithmic substitutes presented as real implementation.
- No "make do with what we have" when the correct move is to expand a contract or state model.
- If required data is missing, expand the appropriate model/interface instead of building around the absence.
- Do not patch around failures caused by an incomplete/non-working implementation.
- Do not keep tuning heuristic ascent logic as if it were PVG/PSG.
- Do not import MechJeb or copy MechJeb source.
- It is acceptable to study MechJeb architecture and derive our own implementation from the same algorithms.
- Universal math/algorithm ideas may be reimplemented, but no direct MechJeb code transplant.
- Agree on contracts before broad implementation.
- Keep responsibilities clear:
  - `LaunchPlanner` is mission planning.
  - `AscentGuidance` is the flight-control computer.
  - `PoweredAscentGuidance` coordinates powered solved guidance.
  - `LaunchHandler` coordinates user actions, state, warp, and control application.
  - `AttitudeControl` actuates steering.
  - `VesselState` is a current vessel snapshot, including solver inputs.
  - `LaunchPlan` remains the selected mission plan source of truth.
  - `ITrajectoryProvider` is the boundary for Stock vs Principia trajectory reads.


Current Build And Harness State
-------------------------------

Main build command:

`C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe Blackbird.csproj /p:Configuration=Debug`

Latest result:
- Build succeeded.
- 0 warnings.
- 0 errors.
- Output: `bin\Debug\Blackbird.dll`

PSG harness build command:

`C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe PsgHarness\PsgHarness.csproj /p:Configuration=Debug`

Latest result:
- Build succeeded.
- 0 warnings.
- 0 errors.
- Output: `PsgHarness\bin\Debug\Blackbird.PsgHarness.exe`

PSG harness run:

`PsgHarness\bin\Debug\Blackbird.PsgHarness.exe`

Latest result:
- Scenario: stock Kerbin, equatorial 81 km insertion.
- Success: true.
- Iterations: 46.
- Termination: 2.
- Constraint violation: about `2.0E-014`.
- Elapsed solve time: about `0.43 s`.
- Terminal AP: `81000.0 m`.
- Terminal PE: `81000.0 m`.
- Eccentricity: `0.000000`.


Important Current Reality
-------------------------

The project has moved past the older heuristic-only `PoweredAscentGuidance` state.

Current repo state includes a BlackBird-owned PSG implementation:

- `Guidance/PsgOptimizer.cs`
- `Guidance/PsgPhase.cs`
- `Guidance/PsgTarget.cs`
- `Guidance/PsgProblem.cs`
- `Guidance/PsgSolution.cs`
- `Guidance/PsgTerminal.cs`
- `Guidance/PsgInitialState.cs`
- `Guidance/PsgBodyModel.cs`
- `Guidance/PsgOptimizationResult.cs`
- `Guidance/PsgSnapshotLogger.cs`
- `ThirdParty/alglib/*.cs`
- `PsgHarness/Program.cs`
- `PsgHarness/PsgHarness.csproj`

`Blackbird.csproj` explicitly compiles:
- all `Guidance\Psg*.cs` files;
- `Models\PoweredStageInfo.cs`;
- `Models\PoweredGuidanceCommand.cs`;
- `ThirdParty\alglib\*.cs`.

This is no longer just a contract sketch. It is a compiling PSG solver path with a
standalone harness that converges on a simple stock Kerbin case.


PSG Implementation State
------------------------

`PsgOptimizer`:
- uses ALGLIB nonlinear constrained optimization via `minnlc`;
- uses SQP (`minlcsetalgosqp`);
- uses finite-difference objective/constraints;
- uses `MaxIterations = 1200`;
- uses `SimpsonNodesPerPhase = 8`, producing 15 knots per phase;
- supports bootstrapping and warm-started converged updates;
- bootstraps with a flight-path-angle terminal pass, then attempts terminal relaxation when appropriate;
- emits `PsgOptimizationResult` with success/status/iterations/termination/constraint violation.

`PsgProblem`:
- can be created from `VesselState`, or from explicit `PsgInitialState` + `PsgBodyModel`;
- captures initial relative position, velocity, thrust direction, mass, UT, altitude, AP/PE, body mu/radius/rotation, atmosphere scale height, phases, and target.

`PsgPhase`:
- is built from `PoweredStageInfo`;
- converts KSP tons/kN into kg/N;
- derives mass flow from vacuum thrust and vacuum Isp;
- carries start/end mass, thrust, Isp, burn time bounds, min throttle, KSP stage, shutdown, coast, unguided, and mass-continuity flags;
- supports coast phases, though normal current extraction builds powered phases.

`PsgTarget`:
- is built from selected `LaunchPlan` / `AscentProfile`;
- captures periapsis radius, apoapsis radius, attachment radius, inclination, LAN, target orbit normal, specific energy, and angular momentum vector.

`PsgTerminal`:
- supports terminal families analogous to the MechJeb PSG terminal ideas:
  - `FlightPathAngle4`
  - `FlightPathAngle5`
  - `Kepler3`
  - `Kepler4`
  - `Kepler5`
- evaluates angular momentum, energy/eccentricity, radius/speed/FPA, and normal alignment constraints depending on target kind.

`PsgSolution`:
- stores solved points and segments;
- exposes `InertialGuidance(universalTime)`;
- exposes `TimeToGo(universalTime)`;
- exposes `VelocityToGo(universalTime)`;
- exposes `TerminalGuidanceSatisfied(...)`;
- supports shifting the solution start UT while the vessel is still prelaunch/landed.

`PsgSnapshotLogger`:
- writes problem/result snapshots to:
  - `D:\SteamLibrary\steamapps\common\Kerbal Space Program Development\glog`
  - or `<KSP root>\Blackbird`
- logs body, initial state, target, phase inputs, optimizer status, terminal state, and solution points.


Current Guidance Flow
---------------------

Planning:

`LaunchPlanner.Create(...)`
-> builds `LaunchPlan`
-> generates `LaunchCandidate[]`
-> each candidate owns insertion AP/PE, launch heading, phasing recommendation, ascent profile
-> `LaunchPlan.SelectedCandidateIndex` chooses active candidate

Guidance:

`LaunchHandler.Update(vessel)`
-> `AscentGuidance.GetGuidance(...)`
-> `VesselState.FromVessel(vessel)`
-> selected candidate/profile
-> `PoweredAscentGuidance.GetCommand(...)`
-> `AscentGuidanceInfo`

Powered solved guidance:

`PoweredAscentGuidance.GetCommand(...)`
-> builds `PsgTarget.FromPlan(...)`
-> builds `PsgPhase[]` from `VesselState.PoweredStages`
-> builds `PsgProblem.Create(...)`
-> asynchronously runs `PsgOptimizer.Solve(problem, warmStart)`
-> on success stores `PsgSolution`
-> reads `PsgSolution.InertialGuidance(vesselState.UniversalTime)`
-> converts inertial guidance to pitch/heading for display
-> returns `PoweredGuidanceCommand` with inertial vector and throttle

Control:

`LaunchHandler.ApplyFlightControls(...)`
-> if `GuidanceInfo.HasInertialDirection`, calls `AttitudeControl.DriveInertial(...)`
-> otherwise calls `AttitudeControl.Drive(...)`
-> sets `FlightCtrlState.mainThrottle` from `GuidanceInfo.CommandThrottle`

This preserves the rule that PSG should not duplicate existing steering/throttle plumbing.


Stage Data State
----------------

`Models/PoweredStageInfo.cs` and `VesselState.PoweredStages` exist.

`VesselState.PoweredStages` is populated from KSP's `VesselDeltaV` stage simulation using reflection over:
- `Vessel.VesselDeltaV`
- `OperatingStageInfo`
- `WorkingStageInfo`
- `DeltaVStageInfo`
- `DeltaVEngineInfo`

Each `PoweredStageInfo` exposes:
- `IsValid`
- `ReasonUnavailable`
- `KspStage`
- `PhaseIndex`
- `IsCurrentOrFutureStage`
- `StartMass`
- `EndMass`
- `DryMass`
- `FuelMass`
- `DecoupledMass`
- `VacuumSpecificImpulse`
- `SeaLevelSpecificImpulse`
- `CurrentSpecificImpulse`
- `VacuumThrust`
- `SeaLevelThrust`
- `CurrentThrust`
- `MinimumThrust`
- `MinimumThrottle`
- `VacuumDeltaV`
- `SeaLevelDeltaV`
- `CurrentDeltaV`
- `BurnTimeSeconds`

Why this matters:
- MechJeb-style PSG consumes explicit phase/stage definitions.
- Aggregated vessel thrust/mass/Isp is insufficient.
- If more stage data is needed, expand `PoweredStageInfo` / `VesselState`, do not fake it inside the solver.


Trajectory Provider Boundary
----------------------------

The app is wired through `ITrajectoryProvider` for fundamental reads where practical.

Current provider methods:
- `GetCurrentState`
- `GetOrbitInfo`
- `GetPosition`
- `GetPositionAtUt`
- `GetVelocity`
- `GetSurfaceVelocity`
- `GetOrbitNormal`
- `GetApoapsisAlt`
- `GetPeriapsisAlt`

Stock provider:
- Uses KSP patched conics and vessel state.

Principia provider:
- Uses reflection into `ksp_plugin_adapter.dll` for current vessel state where available.
- Still falls back to stock for future propagation/orbit summaries in places.

Future PSG/RSS/Principia work should consume `VesselState` and `ITrajectoryProvider`
rather than reading geometry directly from scattered KSP APIs.


UI / Debug State
----------------

The UI currently shows:
- active vessel;
- altitude;
- current apoapsis/periapsis;
- launch plan/candidates;
- candidate AP/PE/heading/orbits/dV/remaining/error;
- ascent profile altitude/pitch/heading/throttle;
- guidance mode;
- guidance phase;
- pitch/heading profile/input/current values;
- profile/command throttle;
- target AP/PE;
- AP/PE error;
- guidance vgo/tgo;
- PSG optimizer status;
- PSG iterations;
- PSG constraint violation;
- predicted AP/PE;
- remaining dV;
- phase/plane error;
- phasing recommendation;
- launch recommendation/window;
- advanced orbit details.

This makes KSP test feedback much easier. Keep these fields until PSG is stable.


MechJeb Reference Context
-------------------------

Local MechJeb repo path provided by the user:

`C:\Users\David\Downloads\MechJeb2-dev`

Important files to study, not copy:
- `MechJeb2/MechJebModulePSGGlueBall.cs`
- `MechJeb2/MechJebModuleGuidanceController.cs`
- `MechJeb2/MechJebModuleAscentPSGAutopilot.cs`
- `MechJebLib/PSG/AscentBuilder.cs`
- `MechJebLib/PSG/Ascent.cs`
- `MechJebLib/PSG/Optimizer.cs`
- `MechJebLib/PSG/AscentProblem.cs`
- `MechJebLib/PSG/Solution.cs`
- `MechJebLib/PSG/Phase.cs`
- `MechJebLib/PSG/Terminal/*`

Critical conclusion:
- MechJeb's "PVG" path is effectively PSG/direct optimized powered guidance.
- It is not a small AP/PE feedback formula.
- The autopilot follows a solved inertial thrust-vector schedule.
- Shutdown is based on solved terminal angular momentum/constraints, not "AP close" or "PE reached".


Known Historical Test Notes
---------------------------

Historical issues fixed or investigated:
- Launch heading initially went polar/incorrect due to heading/orbit-normal issues; stock heading now behaves better.
- UI throttle display was corrected to normalized 0-1 internally and 0-100% for display.
- Launch countdown now decreases from accepted plan rather than staying locked.
- Current AP/PE added to main UI.
- Stage data added to `VesselState`.
- Existing control plumbing now accepts solved inertial guidance vectors.

Older heuristic powered guidance failed testing:
- early versions cut throttle before circularization;
- later patched versions overburned;
- that failure is why PSG implementation is now the correct path.

Do not revive AP/PE feedback patches as the primary solution.


Open Risks And Remaining Work
-----------------------------

The main remaining work is no longer "create PSG contracts"; those files exist.
The remaining work is to validate and mature the PSG implementation in real KSP
flight conditions, especially multi-stage vehicles and RSS/Principia.

Immediate next work:

1. Test current PSG flight behavior in Stock KSP.
   - Watch PSG status, iterations, and constraint violation.
   - Confirm it uses inertial guidance, not profile fallback, after solving.
   - Compare solved terminal AP/PE to actual achieved AP/PE.
   - Save/review `blackbird-psg.log` snapshots after failed flights.

2. Validate stage extraction.
   - Confirm `VesselState.PoweredStages` has correct current/future stages.
   - Confirm units after conversion: KSP tons -> kg, kN -> N.
   - Confirm `MinimumThrottle` is meaningful for engines that cannot throttle deeply.
   - Confirm multi-stage mass continuity/staging behavior.

3. Validate PSG dynamics.
   - Current optimizer dynamics use two-body gravity plus thrust.
   - Thrust varies with a simple atmosphere/Isp interpolation.
   - Real aero drag/lift losses are not clearly modeled in the optimizer path.
   - This likely matters for RSS ascent and may matter for stock low-altitude guidance.

4. Validate terminal completion.
   - `PsgSolution.TerminalGuidanceSatisfied(...)` currently checks angular momentum against a time-selected terminal point.
   - Confirm it cuts off at the right time in KSP.
   - Do not add arbitrary AP/PE cutoff guards unless they correspond to the solved terminal constraints.

5. Validate async solve cadence.
   - `PoweredAscentGuidance` requests solves every 1s before a solution, every 5s with a solution, and every 0.5s near terminal.
   - Confirm solver latency does not stall flight.
   - Confirm stale/expired solution behavior is appropriate.

6. Validate target construction.
   - `PsgTarget.FromPlan(...)` uses selected ascent profile target AP/PE and launch plan target normal.
   - Confirm target normal/LAN behavior for prograde, retrograde, and inclined rendezvous launches.

7. Validate Principia boundary.
   - Keep one PSG guidance method for Stock and RSS/Principia.
   - Improve provider-backed state reads as needed.
   - Do not split into separate "stock ascent" and "RSS ascent" brains.

8. Keep `PsgHarness` useful.
   - Add deterministic harness cases for multi-stage ascent.
   - Add an off-equator/inclined target case.
   - Add an RSS-like high-energy case.
   - Use the harness to distinguish optimizer math failures from KSP control/telemetry failures.


Recommended Next Debug Loop
---------------------------

Start here next session:

1. Re-read this file.
2. Run:
   - `MSBuild Blackbird.csproj /p:Configuration=Debug`
   - `MSBuild PsgHarness\PsgHarness.csproj /p:Configuration=Debug`
   - `PsgHarness\bin\Debug\Blackbird.PsgHarness.exe`
3. In KSP, run one simple stock Kerbin launch with:
   - single-stage or simple two-stage vessel;
   - target orbit near 80-100 km;
   - target vessel in low-inclination orbit.
4. Capture:
   - PSG status;
   - PSG iterations;
   - PSG violation;
   - guidance tgo/vgo;
   - target AP/PE;
   - current AP/PE near cutoff;
   - `blackbird-psg.log`.
5. If the harness succeeds but KSP flight fails:
   - first inspect `VesselState.PoweredStages`;
   - then inspect `PsgProblem` snapshots;
   - then inspect attitude/throttle application.

Do not tune random constants first. Find the mismatch between:
- solver problem inputs;
- solved trajectory;
- real vessel telemetry;
- control actuation.


Most Relevant Files
-------------------

Models:
- `Models/LaunchCandidate.cs`
- `Models/LaunchPlan.cs`
- `Models/VesselState.cs`
- `Models/PoweredStageInfo.cs`
- `Models/PoweredGuidanceCommand.cs`
- `Models/AscentGuidanceInfo.cs`
- `Models/OrbitInfo.cs`
- `Models/TrajectoryState.cs`

Guidance:
- `Guidance/AscentProfile.cs`
- `Guidance/AscentGuidance.cs`
- `Guidance/PoweredAscentGuidance.cs`
- `Guidance/PsgOptimizer.cs`
- `Guidance/PsgPhase.cs`
- `Guidance/PsgTarget.cs`
- `Guidance/PsgProblem.cs`
- `Guidance/PsgSolution.cs`
- `Guidance/PsgTerminal.cs`
- `Guidance/PsgSnapshotLogger.cs`
- `Guidance/LaunchHandler.cs`
- `Guidance/AttitudeControl.cs`

Planning:
- `Planning/LaunchPlanner.cs`
- `Planning/PhasingRecommendationCalculator.cs`

Trajectory:
- `Trajectory/ITrajectoryProvider.cs`
- `Trajectory/TrajectoryProvider.cs`
- `Trajectory/StockTrajectoryProvider.cs`
- `Trajectory/PrincipiaTrajectoryProvider.cs`

Harness:
- `PsgHarness/Program.cs`
- `PsgHarness/PsgHarness.csproj`

Math:
- `Math/OrbitMath.cs`

UI:
- `BlackBird.cs`
