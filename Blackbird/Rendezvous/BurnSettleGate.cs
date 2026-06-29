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
        public const double StabilizeDwellSeconds = 1.5;    // steady condition must hold this long before igniting
        private const double MaxStillRateDegPerSec = 1.0;   // legacy bound; a high-authority craft never needs stricter
        private const double MinStillRateDegPerSec = 0.1;   // achievable settle floor; below this no controller holds

        private const double SettleStepFactor = 2.0;        // multiples of one control step's rate granularity (margin)

        // Max angular rate (deg/s) treated as "still" for this craft
        public static double StillRateThresholdDegPerSec(double minAngularAccelRadPerS2, double physicsDt)
        {
            if (!(minAngularAccelRadPerS2 > 0.0) || !(physicsDt > 0.0)) return MaxStillRateDegPerSec;
            double floor = MathHelpers.Rad2Deg(SettleStepFactor * minAngularAccelRadPerS2 * physicsDt);
            return MathHelpers.Clamp(floor, MinStillRateDegPerSec, MaxStillRateDegPerSec);
        }

        // Pointed and rotationally settled this instant (pre-dwell).
        public static bool IsSteady(double errorDeg, double rateDegPerSec, double stillRateThresholdDegPerSec)
        {
            return errorDeg <= AlignStartDeg && rateDegPerSec <= stillRateThresholdDegPerSec;
        }
    }

    // Tracks the orient -> settle -> armed transition across frames
    public struct BurnSettleTracker
    {
        private bool _hasSteady;
        private double _steadySinceUt;
        private bool _aligned;

        public bool Aligned => _aligned;

        public void Reset()
        {
            _hasSteady = false;
            _steadySinceUt = 0.0;
            _aligned = false;
        }

        // Advance one frame. now = monotonic seconds. Returns whether the burn may fire this frame.
        public bool Update(double errorDeg, double rateDegPerSec, double stillRateThresholdDegPerSec, double now)
        {
            if (!_aligned)
            {
                if (BurnSettleGate.IsSteady(errorDeg, rateDegPerSec, stillRateThresholdDegPerSec))
                {
                    if (!_hasSteady) { _hasSteady = true; _steadySinceUt = now; }
                    if (now - _steadySinceUt >= BurnSettleGate.StabilizeDwellSeconds) _aligned = true;
                }
                else
                {
                    _hasSteady = false;   // moving / off-target: restart the dwell
                }
            }
            else if (errorDeg > BurnSettleGate.AlignKeepDeg)
            {
                _aligned = false;
                _hasSteady = false;
            }
            return _aligned;
        }
    }
}
