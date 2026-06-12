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
        private const double WarpStopLeadTimeSeconds = 300.0;
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
            TimeWarp.SetRate(4, false);
        }
        private static void SetSafeWarpRate(double secondsRemaining)
        {
            int rateIndex;

            if (secondsRemaining <= 300.0)
            {
                rateIndex = 0;
            }
            else if (secondsRemaining <= 420.0)
            {
                rateIndex = 1;
            }
            else if (secondsRemaining <= 600.0)
            {
                rateIndex = 2;
            }
            else if (secondsRemaining <= 900.0)
            {
                rateIndex = 3;
            }
            else if (secondsRemaining <= 1200.0)
            {
                rateIndex = 4;
            }
            else if (secondsRemaining <= 1800.0)
            {
                rateIndex = 5;
            }
            else if (secondsRemaining <= 3600.0)
            {
                rateIndex = 6;
            }
            else
            {
                rateIndex = 7;
            }

            TimeWarp.SetRate(rateIndex, false);

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

                //if ((GuidanceMode == GuidanceMode.Guidance ||
                //     GuidanceMode == GuidanceMode.Autopilot) &&
                //    GuidanceInfo != null)
                //{
                //    Debug.Log($"Pitch set to {GuidanceInfo.CommandPitchDeg:F1}, heading set to: {GuidanceInfo.CommandHeadingDeg:F1}");
                //    ApplyAscentGuidance(vessel, GuidanceInfo);
                //}
                return;
            }

            if (State != LaunchGuidanceState.WarpingToLaunch) return;

            // monitor / adjust warp rate
            double nowUt = Planetarium.GetUniversalTime();

            double secondsRemaining = _targetUt - nowUt;

            if (secondsRemaining <= 0.0)
            {
                TimeWarp.SetRate(0, true);
                _targetUt = 0.0;
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

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
        public void IncreasePitchOffset()
        {
            ManualPitchCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreasePitchOffset -> {ManualPitchCommandDeg:F1}");
        }
        public void DecreasePitchOffset()
        {
            ManualPitchCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreasePitchOffset -> {ManualPitchCommandDeg:F1}");
        }
        public void SetPitchOffset(float pitchOffset) 
        {
            PitchOffsetDeg = pitchOffset;
        }
        public void ResetPitchOffset()
        {
            PitchOffsetDeg = 0.0;
            GuidanceBasePitchDeg = 90.0;
            ManualPitchCommandDeg = 90.0;
            Debug.Log($"[BlackBird] Pitch Reset -> {GuidanceBasePitchDeg:F1}");
        }
        public void IncreaseHeadingOffset()
        {
            ManualHeadingCommandDeg += 1.0;
            Debug.Log($"[BlackBird] IncreaseHeadingOffset -> {HeadingOffsetDeg:F1}");
        }
        public void DecreaseHeadingOffset()
        {
            ManualHeadingCommandDeg -= 1.0;
            Debug.Log($"[BlackBird] DecreaseHeadingOffset -> {HeadingOffsetDeg:F1}");
        }
        public void SetHeadingOffset(float headingOffset)
        {
            HeadingOffsetDeg = headingOffset;
            
        }
        public void ResetHeadingOffset()
        {
            GuidanceBaseHeadingDeg = 90.0;
            HeadingOffsetDeg = 0.0;
            ManualHeadingCommandDeg = 90.0;
            Debug.Log($"[BlackBird] Heading Reset -> {HeadingOffsetDeg:F1}");
        }
        // apply either manual or autopilot pitch + heading inputs
        //public static void ApplyAscentGuidance(Vessel vessel, AscentGuidanceInfo gi)
        //{

        //    if (vessel == null || gi == null || double.IsNaN(gi.CommandPitchDeg) || double.IsNaN(gi.CommandHeadingDeg)) return;

        //    if (!vessel.Autopilot.Enabled) vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.StabilityAssist);

        //    Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
        //    Vector3d north = Vector3d.Exclude(up, vessel.mainBody.transform.up).normalized;
        //    Vector3d east = Vector3d.Cross(up, north).normalized;

        //    double headingRad = gi.CommandHeadingDeg * Math.PI / 180.0;

        //    Vector3d horizontalDirection = (north * Math.Cos(headingRad)) + (east * Math.Sin(headingRad));
        //    double pitchRad = gi.CommandPitchDeg * Math.PI / 180.0;

        //    Vector3d desiredForward = (horizontalDirection * Math.Cos(pitchRad)) + (up * Math.Sin(pitchRad));

        //    //Quaternion desiredRotation = Quaternion.LookRotation(desiredForward, up);
        //    Quaternion desiredRotation = Quaternion.LookRotation(desiredForward, up) * Quaternion.Euler(90.0f, 0.0f, 0.0f);

        //    //Debug.Log(
        //    //    $"CommandHeadingDeg={gi.CommandHeadingDeg:F1} " +
        //    //    $"CommandPitchDeg={gi.CommandPitchDeg:F1} ");

        //    //vessel.Autopilot.SAS.LockRotation(desiredRotation);
        //}

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
