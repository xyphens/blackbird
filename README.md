# BlackBird Context

## Acknowledgements

BlackBird's attitude control system is based on concepts and control architecture derived from the MechJeb2 project. MechJeb2 is licensed under GPL-3.0 and can be found at:

https://github.com/MuMech/MechJeb2

---

## Overview

BlackBird is a KSP launch planning and rendezvous assistance addon.

The long-term goal is:

1. Select a target vessel already in orbit.
2. Generate a launch plan.
3. Launch at the optimal time.
4. Follow launch guidance or autopilot.
5. Reach a near-rendezvous with minimal correction burns.

BlackBird is intended to be a focused rendezvous launch tool for RSS + Principia.  It does provide passive support for Stock KSP.

---

## Supported Environments

### Current

* KSP 1.12.5
* Stock

### Primary Target

* RSS
* RO

### Future

* Principia

RSS + Principia are the design targets.

Stock is currently used as the test environment.

---

## Design Philosophy

### Production First

Avoid temporary implementations.

Prefer implementing the final architecture directly rather than creating placeholder systems that will immediately be replaced.

### Focused Scope

BlackBird is responsible for:

* Launch planning
* Launch windows
* Launch guidance
* Attitude control
* Rendezvous planning
* Optional ascent autopilot

BlackBird is NOT responsible for:

* Autostaging
* Solar panels
* Vessel babysitting
* Abort systems
* Full mission automation
* Fine-grained autopilot tuning

The user should remain in control unless autopilot is explicitly enabled.

---

# Project Structure

## Enums

### AssistantMode.cs

Contains:

```csharp
GuidanceMode
{
    None,
    Guidance,
    Autopilot
}
```

### PlanetScale.cs

Planet scale support.

---

## Guidance

### AscentGuidance.cs

Responsible for:

* Current vessel attitude
* Target attitude
* Pitch profiles
* Guidance calculations
* Command heading/pitch selection

### AscentProfile.cs

Pitch profile definitions.

### LaunchGuidanceState.cs

Launch guidance state machine.

### LaunchHandler.cs

Primary launch guidance coordinator.

Responsibilities:

* State transitions
* Warp management
* Guidance updates
* Manual command state
* Fly-by-wire handoff
* Attitude controller integration

### AttitudeController.cs

Responsible for:

* Surface-reference target attitude construction
* Requested attitude calculation
* Cascaded position/velocity PID control
* Torque-based actuation calculation
* Writing pitch/roll/yaw controls through `FlightCtrlState`

### DirectionTracker.cs

Tracks current and desired attitude deltas in pitch/roll/yaw order.

### PidLoop.cs

PID controller used by `AttitudeController`.

---

## Helpers

### Helpers.cs

General utility functions.

---

## Math

### OrbitMath.cs

Contains orbital and shared math utilities.

Important functions:

```csharp
NormalizeDegrees(...)
DeltaDegrees(...)
GetPhaseAngleDeg(...)
GetLaunchAzimuth(...)
TimeToLongitudeSeconds(...)
GetBodyFixedLongitudeAtTime(...)
```

### Important

GetLaunchAzimuth signature:

```csharp
GetLaunchAzimuth(
    double targetInclination,
    double activeVesselLatitude)
```

Parameter order matters.

---

## Models

### AscentGuidanceInfo

Current ascent guidance state.

### InsertionTarget

Orbit insertion target.

### LaunchLocation

Launch site information.

### LaunchPlan

Full launch recommendation.

### LaunchWindowInfo

Launch timing information.

### OrbitInfo

Orbit model.

### PhasingOrbit

Recommended phasing orbit.

### PhasingRecommendation

Recommended rendezvous solution.

### PitchProfilePoint

Pitch profile node.

---

## Planning

### LaunchPlanner

Creates `LaunchPlan` objects.

### PhasingRecommendationCalculator

Computes rendezvous/phasing recommendations.

---

## Root

### BlackBird.cs / RendezvousAssistant.cs

Main addon entry point.

Owns:

* UI
* Update loop
* Active vessel tracking
* Fly-by-wire subscription
* Launch handler calls

---

# Guidance Modes

## None

BlackBird does nothing.

User flies manually using vessel controls.

## Guidance

BlackBird maintains user-commanded attitude.

User commands:

* Heading
* Pitch

through the BlackBird UI.

Current roll command is fixed internally and should eventually become user-controllable.

## Autopilot

BlackBird follows:

* Launch heading
* Pitch profile

automatically.

Autopilot uses the same attitude controller as Guidance mode.

---

# Planning Status

## Complete

### Launch Planning

* OrbitInfo
* LaunchWindowInfo
* InsertionTarget
* PhasingOrbit
* PhasingRecommendation

### Phasing

Implemented.

Supports:

* Balanced mode
* Period calculations
* Relative phase gain
* Rendezvous estimates

### Launch Azimuth

Implemented.

Returns:

```csharp
double.NaN
```

when launch geometry is impossible.

This is intentional.

---

# Guidance Status

## Implemented

### Pitch Profiles

* Stock profile
* RSS profile
* Altitude-based interpolation

### Guidance UI

Supports:

* None
* Guidance
* Autopilot

### Manual Commands

Current architecture:

```csharp
ManualPitchCommandDeg
ManualHeadingCommandDeg
```

Older offset-based architecture has been removed.

### Warp

Warp-to-launch is implemented and currently working.

### Attitude Control

BlackBird now uses a Euler/Quaternion-based attitude controller with error correction instead of independent scalar pitch/yaw control.

Old model:

```text
PitchErrorDeg -> state.pitch
HeadingErrorDeg -> state.yaw
```

Current model:

```text
Command heading/pitch/roll
    ->
Requested attitude quaternion
    ->
DirectionTracker
    ->
Position PID
    ->
Velocity PID
    ->
Torque demand
    ->
FlightCtrlState pitch/roll/yaw
```

This fixed the prior guidance-mode tumble caused by applying surface heading/pitch errors directly to vessel-local controls.

---

# Current Architecture

## Guidance Calculation

`LaunchHandler.Update()` computes:

```csharp
GuidanceInfo
```

No steering occurs inside `Update()`.

## Steering

Steering occurs only through fly-by-wire:

```text
Update()
    ->
AscentGuidance.GetGuidance()
    ->
GuidanceInfo
    ->
OnFlyByWire
    ->
LaunchHandler.ApplyFlightControls()
    ->
AttitudeController.Drive()
    ->
FlightCtrlState
```

The old SAS steering path has been removed.

### Fly-By-Wire

Relevant Vessel API:

```csharp
public FlightInputCallback OnFlyByWire;
public FlightCtrlState ctrlState;
public void SetControlState(FlightCtrlState state);
```

BlackBird subscribes to the active vessel's `OnFlyByWire` callback and unsubscribes when the active vessel changes.

SAS should be disabled while BlackBird attitude control is active so stock SAS does not fight the controller.

---

# Current Issues

## P0

### Autopilot Validation

Guidance attitude control is now mechanically working.

Autopilot still needs validation for:

* Launch azimuth correctness
* Pitch profile correctness
* Roll behavior
* Ascent stability across different rockets

### Roll Command

Roll is currently not exposed as a user command.

Eventually Guidance mode should support:

```csharp
ManualRollCommandDeg
```

and initialize roll to current vessel roll when Guidance is enabled.

---

# Roadmap

## P0

* Stabilize fly-by-wire controller [Done]
* Replace scalar pitch/yaw steering with attitude control [Done]
* Fix warp system [Done]
* Validate Guidance mode attitude holding
* Validate Autopilot ascent behavior

## P1

* Validate launch azimuth
* Validate pitch profiles
* Add manual roll command support
* Validate RSS pitch profiles
* Improve ascent guidance

## P2

* Advanced rendezvous guidance
* Principia support

---

# LLM Handoff Notes

Important project decisions:

* No temporary code.
* Build toward final architecture.
* RSS + Principia are the design targets.
* Stock is only the test environment.
* BlackBird is not intended to become another MechJeb.
* BlackBird borrows MechJeb-derived attitude control concepts, but its mission scope is focused rendezvous launch planning in RSS + Principia.
* User selects a target vessel and BlackBird should produce a near-rendezvous launch solution.
* Warp-to-launch is fixed.
* Offset-based manual guidance has been removed.
* Guidance mode now uses quaternion/PID-based attitude control through fly-by-wire.
* Next major work:
1.  Refine ascent calculations (especially heading estimates)
    - Projections point at the target vessel's current location rather than a heading we should intercept.  We need to factor in our rocket's deltaV, ideal pitch + heading, TWR into our rendezvous projection
2.  Incorporate deltaV in orbit recommendations
3.  Add yaw control and raw number inputs + buttons
4.  (Re-)move debug level metrics labels and keep UI cleanly focused on need-to-know
5.  Add + show the following:  projected apoapsis and periapsis, estimated remaining deltaV, target info (current heading, altitude)
