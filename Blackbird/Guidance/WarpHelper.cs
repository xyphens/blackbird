using UnityEngine;
using System;
using Blackbird.Mathematics;

namespace Blackbird.Guidance
{
    
    public sealed class WarpHelper
    {
        //private int lastRequestWarpIdx;
        private double lastWarpIncrease;
        private const double WarpIncreaseIntervalSeconds = 2.0; // min real seconds between rate step-ups
        // RSS boosts the warp rates way up, so we need to slash those > 15 seconds
        // do not exceed level 5 if in RSS (todo: might even restrict to 4 in RSS)
        public void SetSafeWarpRate(double secondsRemaining, bool isRss)
        {
            //int rateIndex;

            //if (secondsRemaining <= 15.0)
            //    rateIndex = 1;
            //else if (secondsRemaining <= 60.0)
            //    rateIndex = 2;
            //else if (secondsRemaining <= 180.0)
            //    rateIndex = 3;
            //else if (secondsRemaining <= 600.0)
            //    rateIndex = 4;
            //else if (secondsRemaining <= 1800.0)
            //    rateIndex = 5;
            //else
            //    rateIndex = isRss ? 5 : 6;

            //TimeWarp.SetRate(rateIndex, true);
        }

        private void SetRate(int index, bool instant)
        {
            if (index != TimeWarp.CurrentRateIndex) TimeWarp.SetRate(index, instant);
        }

        // should use ignitionTime - lead time
        public void BetterWarpToUt(double UT, Vessel vessel, bool fastWarp = false, double _maxRate = -1)
        {
            double targetRate = 1;
            double endUt = vessel.orbit.patchEndTransition != Orbit.PatchTransitionType.FINAL 
                            && vessel.orbit.EndUT < UT 
                                ? vessel.orbit.EndUT 
                                : UT;

            double maxRate = _maxRate < 0 ? TimeWarp.fetch.warpRates[TimeWarp.fetch.warpRates.Length - 1] : _maxRate;
            if (fastWarp)
            {
                for (int i = 0; i < TimeWarp.fetch.warpRates.Length; i++)
                {
                    // get the next rate that doesn't throw us beyond our target UT
                    if (Time.fixedDeltaTime * TimeWarp.fetch.warpRates[i] <= endUt - Planetarium.GetUniversalTime())
                    {
                        targetRate = TimeWarp.fetch.warpRates[i] + 0.1;
                    }
                    else
                    {
                        break;
                    }
                }
            } else
            {
                // rate proportional to time-to-go (capped at the SOI/patch end); decelerates as we approach
                targetRate = endUt - (Planetarium.GetUniversalTime() + Time.fixedDeltaTime * TimeWarp.CurrentRateIndex);
            }

            targetRate = MathHelpers.Clamp(targetRate, 1, maxRate);
            if (!vessel.LandedOrSplashed && vessel.mainBody.GetAltitude(vessel.CoMD) < TimeWarp.fetch.GetAltitudeLimit(1, vessel.mainBody)) {
                // physics warp only
                PhysicsWarp((float)Math.Min(targetRate, maxRate), vessel);
            } else
            {
                // step-down is immediate; the rate increase is throttled inside NormalWarp
                NormalWarp((float)Math.Min(targetRate, maxRate), vessel);
            }
        }

        // smooth increase, instant decrease
        private bool SwitchToPhysicsWarp()
        {
            if (TimeWarp.WarpMode == TimeWarp.Modes.LOW) return true;
            TimeWarp.fetch.Mode = TimeWarp.Modes.LOW;
            SetRate(0, true);
            return false;
        }

        // returns true if we're eligible for fast warp
        private bool SwitchToNormalWarp(Vessel vessel)
        {
            if (TimeWarp.WarpMode != TimeWarp.Modes.HIGH)
            {
                double altitude = (vessel.CoMD - vessel.mainBody.position).magnitude - vessel.mainBody.Radius;
                double atmosphere = !vessel.mainBody.atmosphere ? 0 : vessel.mainBody.atmosphereDepth;
                // above atmosphere or landed -> on-rails (HIGH) warp is allowed
                if (altitude > atmosphere || vessel.LandedOrSplashed)
                {
                    TimeWarp.fetch.Mode = TimeWarp.Modes.HIGH;
                    SetRate(0, true);
                    return false;
                }
            }

            return true;
        }

        private void NormalWarp(float maxRate, Vessel vessel)
        {
            if (!SwitchToNormalWarp(vessel)) return; // ineligible to switch to fast warp
            if (TimeWarp.CurrentRateIndex > 0 && TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex] > maxRate)
            {
                SetRate(TimeWarp.CurrentRateIndex - 1, true);
            }
            else if (TimeWarp.CurrentRateIndex + 1 < TimeWarp.fetch.warpRates.Length
                     && TimeWarp.fetch.warpRates[TimeWarp.CurrentRateIndex + 1] <= maxRate
                     && Planetarium.GetUniversalTime() - lastWarpIncrease >= WarpIncreaseIntervalSeconds)
            {
                lastWarpIncrease = Planetarium.GetUniversalTime();
                SetRate(TimeWarp.CurrentRateIndex + 1, false);
            }
        }

        private void PhysicsWarp(float maxRate, Vessel vessel)
        {
            if (!SwitchToPhysicsWarp()) return;
            if (TimeWarp.CurrentRateIndex > 0 && TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex] > maxRate)
            {
                // reduce to desired warp rate instantly
                SetRate(TimeWarp.CurrentRateIndex - 1, true);
            } else if (TimeWarp.CurrentRateIndex + 1 < TimeWarp.fetch.physicsWarpRates.Length
                && TimeWarp.fetch.physicsWarpRates[TimeWarp.CurrentRateIndex + 1] <= maxRate
                && Planetarium.GetUniversalTime() - lastWarpIncrease >= WarpIncreaseIntervalSeconds)
            {
                lastWarpIncrease = Planetarium.GetUniversalTime();
                // incrementally increase warp rate
                SetRate(TimeWarp.CurrentRateIndex + 1, false);
            }
        }

        public static void Stop() => TimeWarp.SetRate(0, true);
    }
}
