using System;
using Blackbird.Models;

namespace Blackbird.Guidance
{
    public sealed class LaunchHandler
    {
        // TODO: lower the lead time or make it an input
        private const double WarpStopLeadTimeSeconds = 20.0;
        private double _targetUt;
        public LaunchGuidanceState State { get; private set; }
        public LaunchPlan CurrentPlan { get; private set; }
        public bool HasPlan => CurrentPlan != null;
        public void SetPlan(LaunchPlan plan)
        {
            CurrentPlan = plan;
            State = plan != null ? LaunchGuidanceState.PlanReady : LaunchGuidanceState.Idle;
        }
        public double SecondsUntilLaunch
        {
            get
            {
                if (_targetUt <= 0.0) return 0.0;
                return _targetUt - Planetarium.GetUniversalTime();
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

            SetSafeWarpRate(timeToLaunch);
        }
        // ramp the warp rate down so we don't overshoot our target
        private static void SetSafeWarpRate(double secondsRemaining)
        {
            int rateIndex;

            if (secondsRemaining > 7200.0)
            {
                rateIndex = 7;
            }
            else if (secondsRemaining > 3600.0)
            {
                rateIndex = 6;
            }
            else if (secondsRemaining > 1200.0)
            {
                rateIndex = 5;
            }
            else if (secondsRemaining > 600.0)
            {
                rateIndex = 4;
            }
            else if (secondsRemaining > 240.0)
            {
                rateIndex = 3;
            }
            else if (secondsRemaining > 90.0)
            {
                rateIndex = 2;
            }
            else if (secondsRemaining > WarpStopLeadTimeSeconds)
            {
                rateIndex = 1;
            }
            else
            {
                rateIndex = 0;
            }

            TimeWarp.SetRate(rateIndex, false);
        }

        public void StartGuidance()
        {
            if (State != LaunchGuidanceState.AwaitingLaunch &&
                State != LaunchGuidanceState.PlanAccepted) return;

            State = LaunchGuidanceState.GuidingAscent;
        }

        public void Update()
        {
            if (State != LaunchGuidanceState.WarpingToLaunch) return;

            double secondsRemaining = _targetUt - Planetarium.GetUniversalTime();

            if (secondsRemaining <= WarpStopLeadTimeSeconds)
            {
                TimeWarp.SetRate(0, true);
                State = LaunchGuidanceState.AwaitingLaunch;
                return;
            }

            SetSafeWarpRate(secondsRemaining);
        }

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
        }

        public void Reset()
        {
            TimeWarp.SetRate(0, true);
            CurrentPlan = null;
            _targetUt = 0.0;
            State = LaunchGuidanceState.Idle;
        }
    }
}
