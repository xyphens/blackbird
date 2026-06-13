using System;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class LaunchHandler
    {
        private const double WarpStopLeadTimeSeconds = 10.0;
        private double _targetUt;

        private readonly AttitudeControl _attitudeControl = new AttitudeControl();

        // manual guidance
        public double ManualPitchCommandDeg { get; private set; } = 90.0;
        public double ManualHeadingCommandDeg { get; private set; } = 90.0;
        // read inputs from Blackbird?
        public GuidanceMode GuidanceMode { get; set; } = GuidanceMode.None;
        public LaunchGuidanceState State { get; private set; }
        public LaunchPlan CurrentPlan { get; private set; }

        public void SetPlan(LaunchPlan plan)
        {
            CurrentPlan = plan;
            State = plan != null ? LaunchGuidanceState.PlanReady : LaunchGuidanceState.Idle;
            _ascentGuidance.Reset();
        }
        private readonly AscentGuidance _ascentGuidance = new AscentGuidance();
        public AscentGuidanceInfo GuidanceInfo { get; private set; }
        public double SecondsUntilLaunch
        {
            get
            {
                if (_targetUt <= 0.0) return 0.0;
                return Math.Max(0.0, _targetUt - Planetarium.GetUniversalTime());
            }
        }
        public void AcceptPlan()
        {
            if (CurrentPlan == null) return;

            State = LaunchGuidanceState.PlanAccepted;
        }

        public void WarpToLaunch()
        {
            if (State != LaunchGuidanceState.PlanAccepted || CurrentPlan == null) return;

            LaunchCandidate selectedCandidate = CurrentPlan.SelectedCandidate;
            if (selectedCandidate == null || !selectedCandidate.IsValid) return;

            double timeToLaunch = selectedCandidate.SecondsUntilLaunch;

            // already close to launch time
            if (timeToLaunch <= WarpStopLeadTimeSeconds)
            {
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            _targetUt = selectedCandidate.LaunchUt;

            State = LaunchGuidanceState.WarpingToLaunch;
        }
        private static void SetSafeWarpRate(double secondsRemaining)
        {
            int rateIndex;

            if (secondsRemaining <= 15.0)
                rateIndex = 1;
            else if (secondsRemaining <= 60.0)
                rateIndex = 2;
            else if (secondsRemaining <= 180.0)
                rateIndex = 3;
            else if (secondsRemaining <= 600.0)
                rateIndex = 4;
            else if (secondsRemaining <= 1800.0)
                rateIndex = 5;
            else
                rateIndex = 6;

            TimeWarp.SetRate(rateIndex, true);

            Debug.Log(
                $"[BlackBird] Warp: T-{secondsRemaining:F1}s");
        }

        public void StartGuidance()
        {
            if (State != LaunchGuidanceState.AwaitingLaunch &&
                State != LaunchGuidanceState.PlanAccepted) return;
            if (CurrentPlan == null || CurrentPlan.SelectedCandidate == null || !CurrentPlan.SelectedCandidate.IsValid) return;

            _ascentGuidance.Reset();
            State = LaunchGuidanceState.GuidingAscent;
        }

        public void Update(Vessel vessel)
        {
            // handle guidance
            if (State == LaunchGuidanceState.GuidingAscent)
            {
                GuidanceInfo =
                    _ascentGuidance.GetGuidance(
                        vessel,
                        CurrentPlan,
                        ManualPitchCommandDeg,
                        ManualHeadingCommandDeg,
                        GuidanceMode);
                return;
            }

            if (State != LaunchGuidanceState.WarpingToLaunch) return;

            // monitor / adjust warp rate
            double nowUt = Planetarium.GetUniversalTime();

            double secondsRemaining = _targetUt - nowUt;

            if (secondsRemaining <= WarpStopLeadTimeSeconds)
            {
                TimeWarp.SetRate(0, true);
                _targetUt = 0.0;
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            SetSafeWarpRate(secondsRemaining);
        }
        public void SetGuidanceMode(GuidanceMode gMode, Vessel vessel = null)
        {
            if (GuidanceMode == gMode) return;

            if (vessel != null && CurrentPlan != null)
            {
                GuidanceInfo =
                    _ascentGuidance.GetGuidance(
                        vessel,
                        CurrentPlan,
                        ManualPitchCommandDeg,
                        ManualHeadingCommandDeg,
                        gMode);
            }

            if (vessel == null || GuidanceInfo == null)
            {
                GuidanceMode = gMode;
                return;
            }

            if (gMode == GuidanceMode.Guidance)
            {
                ManualPitchCommandDeg = GuidanceInfo.CurrentPitchDeg;
                ManualHeadingCommandDeg = OrbitMath.NormalizeDegrees(GuidanceInfo.CurrentHeadingDeg);
                ScreenMessages.PostScreenMessage(
                    $"BlackBird Guidance: pitch={ManualPitchCommandDeg:F1}, heading={ManualHeadingCommandDeg:F1}",
                    5.0f,
                    ScreenMessageStyle.UPPER_CENTER);
            }

            if (gMode == GuidanceMode.Autopilot)
            {
                ManualPitchCommandDeg = ClampAutopilotPitchCommand(GuidanceInfo.ProfilePitchDeg);
                ManualHeadingCommandDeg = OrbitMath.NormalizeDegrees(GuidanceInfo.ProfileHeadingDeg);
            }

            if (GuidanceMode != gMode) _attitudeControl.Reset();
            if (GuidanceMode != gMode) _ascentGuidance.Reset();

            GuidanceMode = gMode;
        }

        private static double ClampAutopilotPitchCommand(double pitchDeg)
        {
            return Math.Max(-30.0, Math.Min(90.0, pitchDeg));
        }

        // abort launch before liftoff
        public void Abort()
        {
            TimeWarp.SetRate(0, true);
            _targetUt = 0.0;

            if (CurrentPlan != null)
            {
                State = LaunchGuidanceState.PlanReady;
            }
            else
            {
                State = LaunchGuidanceState.Idle;
            }

            _ascentGuidance.Reset();
        }

        public void Reset()
        {
            TimeWarp.SetRate(0, true);
            CurrentPlan = null;
            _targetUt = 0.0;
            State = LaunchGuidanceState.Idle;
            _ascentGuidance.Reset();
        }
    
        // pitch command
        public void IncreaseManualPitchCommand()
        {
            ManualPitchCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreaseManualPitchCommand -> {ManualPitchCommandDeg:F1}");
        }
        public void DecreaseManualPitchCommand()
        {
            ManualPitchCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreaseManualPitchCommand -> {ManualPitchCommandDeg:F1}");
        }
        public void ResetPitchCommand()
        {
            if (GuidanceInfo != null)
                ManualPitchCommandDeg = ClampAutopilotPitchCommand(GuidanceInfo.CurrentPitchDeg);
            else
                ManualPitchCommandDeg = 90.0;
        }
        public void IncreaseManualHeadingCommand()
        {
            ManualHeadingCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreaseManualHeadingCommand -> {ManualHeadingCommandDeg:F1}");
        }
        public void DecreaseManualHeadingCommand()
        {
            ManualHeadingCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreaseManualHeadingCommand -> {ManualHeadingCommandDeg:F1}");
        }
        public void ResetHeadingCommand()
        {
            ManualHeadingCommandDeg = GuidanceInfo != null
                                    ? OrbitMath.NormalizeDegrees(GuidanceInfo.CurrentHeadingDeg)
                                    : 90.0;
            Debug.Log($"[BlackBird] ResetHeadingCommand -> {ManualHeadingCommandDeg:F1}");
        }

        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null) return;
            if (State != LaunchGuidanceState.GuidingAscent) return;
            if (GuidanceMode == GuidanceMode.None || GuidanceInfo == null) return;

            if (IsPrelaunchHold(vessel))
            {
                _attitudeControl.Reset();
                state.pitch = state.pitchTrim;
                state.yaw = state.yawTrim;
                state.roll = state.rollTrim;
                ApplyAutopilotThrottle(state);
                return;
            }

            if (GuidanceInfo.HasInertialDirection)
            {
                _attitudeControl.DriveInertial(vessel, state, GuidanceInfo.InertialDirection, 0.0);
            }
            else
            {
                _attitudeControl.Drive(vessel, state, GuidanceInfo.CommandHeadingDeg, GuidanceInfo.CommandPitchDeg, 0.0);
            }

            ApplyAutopilotThrottle(state);
        }

        private void ApplyAutopilotThrottle(FlightCtrlState state)
        {
            if (GuidanceMode != GuidanceMode.Autopilot) return;

            state.mainThrottle = (float)(Math.Max(0.0, Math.Min(1.0, GuidanceInfo.CommandThrottle)));
        }

        private static bool IsPrelaunchHold(Vessel vessel)
        {
            return vessel != null && vessel.situation == Vessel.Situations.PRELAUNCH;
        }
    }
}
