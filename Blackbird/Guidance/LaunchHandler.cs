using System;
using Blackbird.Enums;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Guidance
{
    public sealed class LaunchHandler
    {
        // TODO: lower the lead time or make it an input
        private const double WarpStopLeadTimeSeconds = 300.0;
        private double _targetUt;

        // pitch guidance
        public double PitchOffsetDeg { get; private set; }
        public double HeadingOffsetDeg { get; private set; }
        // read inputs from Blackbird?
        public GuidanceMode GuidanceMode { get; set; } = GuidanceMode.None;
        public bool EnableAutopilot { get; set; }
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
                        PitchOffsetDeg,
                        HeadingOffsetDeg,
                        GuidanceMode
                        );

                if ((GuidanceMode == GuidanceMode.Guidance ||
                     GuidanceMode == GuidanceMode.Autopilot) &&
                    GuidanceInfo != null)
                {
                    ApplyAscentGuidance(vessel, GuidanceInfo);
                }
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

        public void SetGuidanceMode(GuidanceMode mode)
        {
            GuidanceMode = mode;
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
            PitchOffsetDeg += 1.0;
        }
        public void DecreasePitchOffset()
        {
            PitchOffsetDeg -= 1.0;
        }
        public void SetPitchOffset(float pitchOffset) 
        {
            PitchOffsetDeg = pitchOffset;
        }
        public void ResetPitchOffset()
        {
            PitchOffsetDeg = 0.0;
        }
        public void IncreaseHeadingOffset()
        {
            HeadingOffsetDeg += 1.0;
        }
        public void DecreaseHeadingOffset()
        {
            HeadingOffsetDeg -= 1.0;
        }
        public void SetHeadingOffset(float headingOffset)
        {
            HeadingOffsetDeg = headingOffset;
        }
        public void ResetHeadingOffset()
        {
            HeadingOffsetDeg = 0.0;
        }
        // apply either manual or autopilot pitch + heading inputs
        public static void ApplyAscentGuidance(Vessel vessel, AscentGuidanceInfo gi)
        {
            if (vessel == null || gi == null || double.IsNaN(gi.CommandPitchDeg) || double.IsNaN(gi.CommandHeadingDeg)) return;

            if (!vessel.Autopilot.Enabled) vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.StabilityAssist);

            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vessel.mainBody.transform.up).normalized;
            Vector3d east = Vector3d.Cross(up, north).normalized;

            double headingRad = gi.CommandHeadingDeg * Math.PI / 180.0;

            Vector3d horizontalDirection = (north * Math.Cos(headingRad)) + (east * Math.Sin(headingRad));
            double pitchRad = gi.CommandPitchDeg * Math.PI / 180.0;

            Vector3d desiredForward = (horizontalDirection * Math.Cos(pitchRad)) + (up * Math.Sin(pitchRad));

            Quaternion desiredRotation = Quaternion.LookRotation(desiredForward, up);

            vessel.Autopilot.SAS.LockRotation(desiredRotation);
        }
    }
}
