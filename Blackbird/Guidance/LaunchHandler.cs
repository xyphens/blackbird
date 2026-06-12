using System;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class LaunchHandler
    {
        // TODO: lower the lead time or make it an input
        private const double WarpStopLeadTimeSeconds = 10.0;
        private double _targetUt;

        private const double PitchGain = 0.015;
        private const double YawGain = 0.015;
        private const float MaxControlInput = 0.25f;

        // pitch guidance
        public double PitchOffsetDeg { get; private set; }
        public double GuidanceBasePitchDeg { get; private set; }
        public double ManualPitchCommandDeg { get; private set; } = 90.0;
        // heading guidance
        public double HeadingOffsetDeg { get; private set; }
        public double GuidanceBaseHeadingDeg { get; private set; }
  
        public double ManualHeadingCommandDeg { get; private set; } = 90.0;
        // read inputs from Blackbird?
        public GuidanceMode GuidanceMode { get; set; } = GuidanceMode.None;
        private GuidanceMode _previousGuidanceMode = GuidanceMode.None;
        public LaunchGuidanceState State { get; private set; }
        public LaunchPlan CurrentPlan { get; private set; }
        public bool HasPlan => CurrentPlan != null;

        public void SetPlan(LaunchPlan plan)
        {
            CurrentPlan = plan;
            State = plan != null ? LaunchGuidanceState.PlanReady : LaunchGuidanceState.Idle;
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

            double timeToLaunch = CurrentPlan.LaunchWindow.TimeToPlaneCrossingSeconds;

            // already close to launch time
            if (timeToLaunch <= WarpStopLeadTimeSeconds)
            {
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            _targetUt =
                Planetarium.GetUniversalTime() +
                timeToLaunch;

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

            // switch to manual guidance
            if (vessel == null || GuidanceInfo == null) return;

            if (gMode == GuidanceMode.Guidance)
            {
                // prefill manual guidance vectors w/ ships default heading
                ManualPitchCommandDeg = GuidanceInfo.CurrentPitchDeg;
                ManualHeadingCommandDeg = GuidanceInfo.CurrentHeadingDeg;

                if (GuidanceMode == GuidanceMode.None)
                {
                    // log our base heading + pitch at the pad
                    GuidanceBasePitchDeg = GuidanceInfo.CurrentPitchDeg;
                    GuidanceBaseHeadingDeg = GuidanceInfo.CurrentHeadingDeg;
                    // point to default if pre-launch
                    //PitchOffsetDeg = 90.0;
                    //HeadingOffsetDeg = 0.0;
                }
            }

            // auto-pilot enabled, point to target vectors
            if (gMode == GuidanceMode.Autopilot)
            {
                PitchOffsetDeg =
                    GuidanceInfo.CurrentPitchDeg - GuidanceInfo.TargetPitchDeg;

                HeadingOffsetDeg =
                    OrbitMath.DeltaDegrees(
                        GuidanceInfo.TargetAzimuthDeg,
                        GuidanceInfo.CurrentHeadingDeg);

                ManualPitchCommandDeg = GuidanceInfo.TargetPitchDeg;

                ManualHeadingCommandDeg = GuidanceInfo.TargetAzimuthDeg;
            }

            GuidanceMode = gMode;
        }

        // abort launch before liftoff
        public void Abort()
        {
            TimeWarp.SetRate(0, true);
            _targetUt = 0.0;

            PitchOffsetDeg = 0;
            HeadingOffsetDeg = 0;

            if (CurrentPlan != null)
            {
                State = LaunchGuidanceState.PlanReady;
            }
            else
            {
                State = LaunchGuidanceState.Idle;
            }
        }

        public void Reset()
        {
            TimeWarp.SetRate(0, true);
            CurrentPlan = null;
            _targetUt = 0.0;
            State = LaunchGuidanceState.Idle;
        }
    
        // pitch command
        public void IncreaseManualPitchCommand()
        {
            ManualPitchCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreasePitchOffset -> {ManualPitchCommandDeg:F1}");
        }
        public void DecreaseManualPitchCommand()
        {
            ManualPitchCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreasePitchOffset -> {ManualPitchCommandDeg:F1}");
        }
        public void ResetPitchCommand()
        {
            PitchOffsetDeg = 0.0;
            GuidanceBasePitchDeg = 90.0;
            ManualPitchCommandDeg = 90.0;
            Debug.Log($"[BlackBird] Pitch Reset -> {GuidanceBasePitchDeg:F1}");
        }
        public void IncreaseManualHeadingCommand()
        {
            ManualHeadingCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreaseHeadingOffset -> {HeadingOffsetDeg:F1}");
        }
        public void DecreaseManualHeadingCommand()
        {
            ManualHeadingCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreaseHeadingOffset -> {HeadingOffsetDeg:F1}");
        }
        public void ResetHeadingCommand()
        {
            GuidanceBaseHeadingDeg = 90.0;
            HeadingOffsetDeg = 0.0;
            ManualHeadingCommandDeg = 90.0;
            Debug.Log($"[BlackBird] Heading Reset -> {HeadingOffsetDeg:F1}");
        }

        public void ApplyFlightControls(FlightCtrlState state)
        {
            if (state == null) return;
            if (State != LaunchGuidanceState.GuidingAscent) return;
            if (GuidanceMode == GuidanceMode.None || GuidanceInfo == null) return;

            float pitchInput =
                Mathf.Clamp(
                    (float)(GuidanceInfo.PitchErrorDeg * PitchGain),
                    -MaxControlInput,
                    MaxControlInput);

            float yawInput =
                Mathf.Clamp(
                    (float)(GuidanceInfo.HeadingErrorDeg * YawGain),
                    -MaxControlInput,
                    MaxControlInput);

            state.pitch = pitchInput;
            state.yaw = yawInput;

            // Don't force roll yet.
            // state.roll = 0.0f;
        }
    }
}
