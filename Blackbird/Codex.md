BlackBird Codex Handoff
=======================

Last updated: 2026-06-13

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

The planner should know the intended rendezvous strategy before launch. It should not invent the strategy only after reaching orbit.


Hard Rules
----------

Do not violate these:

- No placeholder implementations.
- No temporary algorithmic substitutes presented as real implementation.
- No "make do with what we have" when the correct move is to expand a contract or state model.
- If required data is missing, expand the appropriate model/interface instead of building around the absence.
- Do not patch around failures caused by an incomplete/non-working implementation.
- Do not keep tuning the current heuristic ascent controller as if it were PVG.
- Do not import MechJeb or copy MechJeb source.
- It is acceptable to study MechJeb architecture and derive our own implementation from the same algorithms.
- Universal math/algorithm ideas may be reimplemented, but no direct MechJeb code transplant.
- Agree on contracts before broad implementation.
- Keep responsibilities clear:
  - `LaunchPlanner` is mission planning.
  - `AscentGuidance` is the flight-control computer.
  - `PoweredAscentGuidance` should become the powered guidance brain.
  - `LaunchHandler` coordinates user actions, state, warp, and control application.
  - `AttitudeControl` actuates steering.
  - `VesselState` is a current vessel snapshot, including solver inputs.
  - `LaunchPlan` remains the selected mission plan source of truth.
  - `ITrajectoryProvider` is the boundary for Stock vs Principia trajectory reads.


Current Build State
-------------------

Last known build command:

`C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe Blackbird.csproj /p:Configuration=Debug`

Last known result:
- Build succeeded.
- 0 warnings.
- 0 errors.
- Output: `bin\Debug\Blackbird.dll`


Important Current Reality
-------------------------

The current `PoweredAscentGuidance` is not PVG.

It is a heuristic feedback scaffold that:
- follows the altitude-indexed ascent profile early,
- enters "Powered guidance",
- commands mostly 100% throttle,
- computes pitch from AP/PE/velocity heuristics,
- cuts off when PE reaches insertion tolerance.

This scaffold has failed testing:
- earlier version cut throttle before circularization;
- patched version stopped cutting early but overburned;
- behavior confirms it is not solving terminal powered ascent.

Do not continue patching or tuning the scaffold.

Correct next direction:
- replace heuristic internals with a real PSG/PVG-style optimizer and solved inertial guidance output.


What Changed Recently
---------------------

Added stage-phase input surface:

- `Models/PoweredStageInfo.cs`
- `VesselState.PoweredStages`

`VesselState.PoweredStages` is populated from KSP's own `VesselDeltaV` stage simulation using reflection over:
- `Vessel.VesselDeltaV`
- `OperatingStageInfo`
- `WorkingStageInfo`
- `DeltaVStageInfo`
- `DeltaVEngineInfo`

Each `PoweredStageInfo` exposes:
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
- MechJeb's PSG/PVG path consumes explicit phase/stage definitions.
- Aggregated vessel thrust/mass/Isp is insufficient.
- This was corrected after explicitly rejecting the "make do with aggregate values" approach.

Also note:
- A premature aggregate `PvgAscentSolution.cs` file was started and then removed.
- Do not resurrect aggregate-only solver work.


Current Relevant Files
----------------------

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

Math:
- `Math/OrbitMath.cs`

Enums:
- `Enums/PoweredGuidancePhase.cs`
- `Enums/AssistantMode.cs`
- `Enums/PlanetScale.cs`

UI:
- `BlackBird.cs`


Current Architecture Flow
-------------------------

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

Control:

`LaunchHandler.ApplyFlightControls(...)`
-> `AttitudeControl.Drive(vessel, state, heading, pitch, roll)`
-> sets `FlightCtrlState.mainThrottle` from `GuidanceInfo.CommandThrottle`


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

Future PSG/PVG work should consume `VesselState` and `ITrajectoryProvider` rather than reading geometry directly from scattered KSP APIs.


MechJeb Reference Context
-------------------------

Local MechJeb repo:

`C:\Users\David\Downloads\MechJeb2-dev\MechJeb2-dev`

Important files inspected:

- `MechJeb2/MechJebModulePSGGlueBall.cs`
  - Builds the PSG ascent problem.
  - Feeds initial state, target, aero constants, stage phases, old solution.
  - Uses `Core.StageStats.VacStats` and `Core.StageStats.AtmoStats`.

- `MechJeb2/MechJebModuleGuidanceController.cs`
  - Owns guidance status.
  - Follows `Solution.InertialGuidance(time)`.
  - Updates `Tgo` and `Vgo`.
  - Ends terminal guidance via `Solution.TerminalGuidanceSatisfied(...)`.

- `MechJeb2/MechJebModuleAscentPSGAutopilot.cs`
  - Handles ascent phase transitions: vertical ascent, pitch program, zero-lift, guidance.
  - In guidance mode, points at the inertial vector from the guidance controller.

- `MechJebLib/PSG/AscentBuilder.cs`
  - Builder for problem initial state, aerodynamic constants, stages/coasts, and terminal target.

- `MechJebLib/PSG/Ascent.cs`
  - Bootstrapping and converged optimization flow.

- `MechJebLib/PSG/Optimizer.cs`
  - Nonlinear constrained optimizer.
  - Uses ALGLIB.
  - Transcribes previous solution.
  - Builds solution from optimized variables.

- `MechJebLib/PSG/AscentProblem.cs`
  - Direct collocation constraints.
  - Dynamic constraints.
  - Staging constraints.
  - Control norm constraints.
  - Terminal constraints.

- `MechJebLib/PSG/Solution.cs`
  - Stores solved trajectory.
  - `InertialGuidance(time)` returns inertial direction and throttle.
  - `TerminalGuidanceSatisfied(pos, vel, time)` checks angular momentum against solved terminal state.

- `MechJebLib/PSG/Phase.cs`
  - Defines powered/coast phase parameters:
    - `M0`
    - `Mf`
    - thrust
    - Isp
    - burn time
    - min throttle
    - KSP stage
    - allow shutdown
    - mass continuity

- `MechJebLib/PSG/Terminal/*`
  - Terminal constraints:
    - `FlightPathAngle4`
    - `FlightPathAngle5`
    - `Kepler3`
    - `Kepler4`
    - `Kepler5`

Critical conclusion:
- MechJeb's "PVG" path is really PSG/direct optimized powered guidance.
- It is not a small AP/PE feedback formula.
- The autopilot follows a solved inertial thrust-vector schedule.
- Shutdown is based on solved terminal angular momentum/constraints, not "AP close" or "PE reached".


Optimizer Decision Still Needed
-------------------------------

To match the MechJeb-style implementation, BlackBird needs a nonlinear constrained optimizer.

MechJeb uses ALGLIB.

Options:
1. Add ALGLIB as a dependency and implement BlackBird's own PSG optimizer using it.
2. Implement an equivalent nonlinear constrained optimizer ourselves.

Do not:
- keep heuristic AP/PE feedback and call it PVG;
- add a "temporary" optimizer substitute;
- use aggregate vessel values instead of `PoweredStages`;
- copy/import MechJebLib.

If ALGLIB is used:
- confirm licensing/dependency approach;
- add it as an optimizer dependency, not as a MechJeb import;
- write BlackBird-owned problem/phase/solution classes.


Required Next Contract
----------------------

Before implementing the real guidance brain, define contracts for these BlackBird-owned types:

1. `PsgPhase`
   - Built from `PoweredStageInfo`.
   - Fields should mirror required solver inputs:
     - start mass
     - end mass
     - vacuum thrust
     - vacuum Isp
     - current/sea-level Isp if atmosphere is modeled
     - burn time bounds
     - min throttle
     - KSP stage
     - allow shutdown
     - coast/mass continuity flags if supported

2. `PsgTarget`
   - Built from selected `LaunchCandidate` / `AscentProfile`.
   - Represents terminal orbit constraints:
     - periapsis radius
     - apoapsis radius
     - attachment radius/FPA or Kepler constraints
     - inclination / plane normal
     - LAN if required

3. `PsgProblem`
   - Initial state from `VesselState`:
     - relative position
     - relative velocity
     - initial thrust direction
     - mass
     - universal time
     - body mu
     - body radius
     - body rotation vector
     - atmosphere/aero constants if modeled
   - Phase list from `VesselState.PoweredStages`.
   - Terminal target from `PsgTarget`.

4. `PsgSolution`
   - Solved trajectory/control output.
   - Must expose:
     - `InertialGuidance(double universalTime)`
     - `TimeToGo(double universalTime)`
     - `VelocityToGo(double universalTime)` or equivalent
     - `TerminalGuidanceSatisfied(...)`
     - terminal state summary for debugging/UI

5. `PsgOptimizer`
   - Owns actual solve.
   - Should be async or throttled similarly to MechJeb so it does not stall flight.
   - Supports warm-start/old solution later.

6. `PoweredAscentGuidance`
   - Should become a coordinator around `PsgOptimizer` and `PsgSolution`.
   - It should not contain AP/PE heuristic steering.
   - It should output a `PoweredGuidanceCommand` generated from solved inertial guidance.


Current `PoweredAscentGuidance` Warning
---------------------------------------

`Guidance/PoweredAscentGuidance.cs` currently contains many `FIXME` comments.

These are not accepted final behavior.

Current temporary items include:
- terminal AP/PE tolerances;
- pitch-program handoff margin;
- terminal handoff bands;
- vertical-ascent gate;
- pitch blend/clamp;
- radial-speed schedule;
- AP/PE proportional shaping gains;
- descent guard;
- percentage-based terminal tolerance;
- PE-based insertion cutoff.

Do not spend more time tuning these unless the explicit task is to make a temporary test flight safer.

For real work, replace these internals with solved guidance.


Expected Behavior Right Now
---------------------------

If tested in KSP right now:

- Launch planning UI should function.
- Candidate selection should function.
- Guidance should start.
- The vessel should follow the early ascent profile.
- It should enter "Powered guidance".
- It may overburn or miss the insertion target.

This is expected because real PSG/PVG is not implemented yet.

Do not interpret overburn/miss as a small bug in the final algorithm.
It is evidence that the current scaffold must be replaced.


Known UI / Debug State
----------------------

The UI currently shows:
- active vessel;
- altitude;
- current apoapsis/periapsis;
- launch plan/candidates;
- ascent profile;
- guidance mode;
- guidance phase;
- pitch/heading profile/input/current values;
- profile/command throttle;
- target AP/PE;
- AP/PE error;
- guidance vgo/tgo;
- predicted AP/PE;
- remaining dV;
- phase/plane error;
- phasing recommendation;
- launch recommendation/window;
- advanced orbit details.

`Guidance vgo` currently comes from the heuristic scaffold and is not a solved PVG value.


Important Bugs / Design Notes From Testing
------------------------------------------

Historical issues fixed or investigated:
- Launch heading initially went polar/incorrect due to heading/orbit-normal issues; stock heading now behaves better.
- UI throttle display was corrected to normalized 0-1 internally and 0-100% for display.
- Launch countdown now decreases from accepted plan rather than staying locked.
- Current AP/PE added to main UI.
- Stage data added to `VesselState`.

Current unresolved core issue:
- Powered guidance is not a real PSG/PVG solve and overburns/misses target.

Ascent profile notes:
- `AscentProfileSolver` still has heuristic gravity-turn values and `FIXME` comments.
- User noted profile points should eventually be calculated from an optimal altitude -> pitch -> target altitude relationship.
- For RSS/Principia, expected behavior is continuous powered ascent, not coast-to-apoapsis circularization.


Principia/RSS Direction
-----------------------

Long-term design:
- All fundamental geometry/vector/orbit reads should go through `ITrajectoryProvider`.
- Stock provider uses KSP patched conics.
- Principia provider should fulfill the same API with true Principia state where possible.
- The PSG/PVG solver should consume provider-backed `VesselState`.

Do not build separate "stock ascent" and "RSS ascent" brains.
There should be one powered guidance method whose inputs come from the provider/state layer.


Immediate Next Steps For Tomorrow
----------------------------------

Start here:

1. Re-read this file.
2. Re-read:
   - `Models/VesselState.cs`
   - `Models/PoweredStageInfo.cs`
   - `Guidance/PoweredAscentGuidance.cs`
   - `Models/PoweredGuidanceCommand.cs`
   - `Enums/PoweredGuidancePhase.cs`
   - `Guidance/AscentGuidance.cs`
   - `Guidance/LaunchHandler.cs`
3. Do not tune the heuristic guidance.
4. Define the PSG/PVG contracts listed above.
5. Decide optimizer dependency path:
   - ALGLIB dependency vs in-house nonlinear constrained optimizer.
6. Implement `PsgPhase` from `PoweredStageInfo`.
7. Implement `PsgTarget` from selected candidate/profile.
8. Implement `PsgProblem` from `VesselState + PsgPhase[] + PsgTarget`.
9. Only then implement solver/solution and wire `PoweredAscentGuidance` to solved inertial output.

Suggested first concrete file tomorrow:

`Guidance/PsgPhase.cs`

Reason:
- It codifies the MechJeb-equivalent phase contract using our newly exposed `PoweredStageInfo`.
- It prevents backsliding into aggregate-vessel guidance.


Do Not Forget
-------------

The user explicitly corrected the approach:

Instead of saying:
"I'll make do with what we have."

The correct response is:
"We need to expand our model/interface to expose the needed items."

That correction is now part of the project contract.
