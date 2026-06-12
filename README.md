# BlackBird Context

## Overview

BlackBird is a KSP launch planning and rendezvous assistance addon.

The long-term goal is:

1. Select a target vessel already in orbit.
2. Generate a launch plan.
3. Launch at the optimal time.
4. Follow launch guidance or autopilot.
5. Reach a near-rendezvous with minimal correction burns.

BlackBird is intended to be a focused rendezvous tool, not a full automation suite like MechJeb.

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

### AscentProfile.cs

Pitch profile definitions.

### LaunchGuidanceState.cs

Launch guidance state machine.

### LaunchHandler.cs

Primary launch guidance controller.

Responsibilities:

* State transitions
* Warp management
* Guidance updates
* Fly-by-wire integration

---

## Helpers

### Helpers.cs

General utility functions.

---

## Math

### OrbitMath.cs

Contains orbital math utilities.

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

Creates LaunchPlan objects.

### PhasingRecommendationCalculator

Computes rendezvous recommendations.

---

## Root

### BlackBird.cs

Main addon entry point.

Owns:

* UI
* Update loop
* Fly-by-wire integration

---

# Guidance Modes

## None

BlackBird does nothing.

User flies manually using vessel controls.

## Guidance

BlackBird provides steering guidance.

User commands:

* Heading
* Pitch

through the BlackBird UI.

BlackBird should maintain the user-commanded attitude.

## Autopilot

BlackBird follows:

* Launch heading
* Pitch profile

automatically.

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

Stock profile

RSS profile

Altitude-based interpolation.

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

Older offset-based architecture is being removed.

---

# Current Architecture

## Guidance Calculation

LaunchHandler.Update()

Only computes:

```csharp
GuidanceInfo
```

No steering should occur inside Update().

## Steering

Migrating away from:

```csharp
ApplyAscentGuidance(...)
Quaternion.LookRotation(...)
SAS.LockRotation(...)
```

Old SAS steering path should eventually be removed.

### Fly-By-Wire

Relevant Vessel API:

```csharp
public FlightInputCallback OnFlyByWire;
public FlightCtrlState ctrlState;
public void SetControlState(FlightCtrlState state);
```

Target architecture:

```text
Update()
    ->
GetGuidance()
    ->
GuidanceInfo
    ->
OnFlyByWire
    ->
ApplyFlightControls()
```

---

# Current Issues

## P0

### Guidance Mode Tumbles

Autopilot mode appears functional.

Guidance mode causes unstable flight.

Likely causes:

* Incorrect pitch sign
* Incorrect yaw sign
* Surface-heading errors being applied directly to vessel-local controls
* Controller gain tuning

### Warp Logic

Warp system currently does not behave correctly.

Known issues:

* Overshoots launch windows
* Stops early
* Does not scale rates correctly

Warp system requires redesign.

---

# Roadmap

## P0

* Stabilize fly-by-wire controller
* Fix Guidance mode
* Fix warp system

## P1

* Validate launch guidance
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
* User selects a target vessel and BlackBird should produce a near-rendezvous launch solution.

Current focus:

Stabilizing the new fly-by-wire guidance controller.
