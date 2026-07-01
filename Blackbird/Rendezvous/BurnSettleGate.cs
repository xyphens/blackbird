using System;
using Blackbird.Mathematics;

namespace Blackbird.Rendezvous
{
    // Decides when a craft is pointed and rotationally STILL enough to begin a burn. Firing mid-swing flings
    // the delivered dV off-axis, ruining the maneuver
    public static class BurnSettleGate
    {
        public const double AlignStartDeg = 1.0;            // pointing tolerance to begin settling
        public const double AlignKeepDeg = 20.0;            // hysteresis: re-orient only if error exceeds this mid-burn
        public const double StabilizeDwellSeconds = 1.5;    // the rotation rate must be plateaued this long before igniting
        public const double MaxStillRateDegPerSec = 1.0;    // loose ceiling: never ignite above this (mid-swing guard)

        // Rate improvement smaller than this is noise, not settling. Derived from the craft's per-step rate
        // granularity (one control step changes the rate by ~alpha*dt) so there is no hand-tuned floor to clear —
        // a weak craft that limit-cycles at 0.2 deg/s plateaus and arms, instead of chasing an unreachable floor.
        public const double DefaultRateImproveDeadbandDegPerSec = 0.05;   // fallback when torque authority is unknown

        public static double RateImproveDeadbandDegPerSec(double minAngularAccelRadPerS2, double physicsDt)
        {
            if (!(minAngularAccelRadPerS2 > 0.0) || !(physicsDt > 0.0)) return DefaultRateImproveDeadbandDegPerSec;
            return MathHelpers.Rad2Deg(minAngularAccelRadPerS2 * physicsDt);
        }
    }

    // Tracks the orient -> settle -> armed transition across frames. Ignition waits for the rotation rate to stop
    // dropping (plateau) while pointed, rather than clearing a fixed rate floor: it self-calibrates to whatever
    // limit-cycle floor the craft actually holds.
    public struct BurnSettleTracker
    {
        private bool _tracking;         // pointed and tracking the rate plateau (false while still slewing)
        private double _minRate;        // smallest pitch/yaw rate seen since we started pointing
        private double _lastImproveUt;  // last time _minRate dropped by more than the deadband
        private bool _aligned;

        public bool Aligned => _aligned;

        public void Reset()
        {
            _tracking = false;
            _minRate = 0.0;
            _lastImproveUt = 0.0;
            _aligned = false;
        }

        // Advance one frame. now = monotonic seconds. Returns whether the burn may fire this frame.
        public bool Update(double errorDeg, double rateDegPerSec, double rateImproveDeadbandDegPerSec, double now)
        {
            if (_aligned)
            {
                if (errorDeg > BurnSettleGate.AlignKeepDeg)   // knocked far off axis mid-burn: re-orient
                {
                    _aligned = false;
                    _tracking = false;
                }
                return _aligned;
            }

            // The rate plateau only means "settled" once we are pointed; while still slewing to the burn vector,
            // keep resetting so the dwell measures steadiness ON the target, not on the way to it.
            if (errorDeg > BurnSettleGate.AlignStartDeg)
            {
                _tracking = false;
                return false;
            }

            if (!_tracking)
            {
                _tracking = true;
                _minRate = rateDegPerSec;
                _lastImproveUt = now;
            }
            else if (rateDegPerSec < _minRate - rateImproveDeadbandDegPerSec)
            {
                _minRate = rateDegPerSec;      // rate still meaningfully dropping: not plateaued yet
                _lastImproveUt = now;
            }
            else if (rateDegPerSec < _minRate)
            {
                _minRate = rateDegPerSec;      // track the true minimum without resetting the plateau clock
            }

            bool plateaued = now - _lastImproveUt >= BurnSettleGate.StabilizeDwellSeconds;
            if (plateaued && rateDegPerSec <= BurnSettleGate.MaxStillRateDegPerSec)
                _aligned = true;

            return _aligned;
        }
    }
}
