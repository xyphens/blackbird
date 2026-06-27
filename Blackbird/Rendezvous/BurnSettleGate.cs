using System;
using Blackbird.Mathematics;

namespace Blackbird.Rendezvous
{
    // Decides when a craft is pointed and rotationally STILL enough to begin a burn. Firing mid-swing flings
    // the delivered dV off-axis, ruining the maneuver; a powerful engine amplifies any residual rotation. The
    // "still" rate threshold scales with control authority so it holds for every craft: a heavy/low-authority
    // vessel must settle to near zero (it coasts through alignment, so a fixed bound lets it fire while still
    // turning), while a nimble one keeps the looser legacy bound and isn't slowed. Pure + harness-tested.
    public static class BurnSettleGate
    {
        public const double AlignStartDeg = 1.0;            // pointing tolerance to begin settling
        public const double AlignKeepDeg = 20.0;            // hysteresis: re-orient only if error exceeds this mid-burn
        public const double StabilizeDwellSeconds = 1.5;    // steady condition must hold this long before igniting
        private const double LooseRateCapDegPerSec = 1.0;   // legacy bound; a high-authority craft never needs stricter
        private const double SettleStepFactor = 2.0;        // multiples of one control step's rate granularity (margin)

        // Max angular rate (deg/s) treated as "still" for this craft. One control step changes the rate by
        // alpha*dt, the tightest the controller can reliably hold, so require a small multiple of that, capped
        // at the legacy bound. Heavy/low-alpha craft -> near zero; nimble high-alpha craft -> the cap.
        public static double StillRateThresholdDegPerSec(double minAngularAccelRadPerS2, double physicsDt)
        {
            if (!(minAngularAccelRadPerS2 > 0.0) || !(physicsDt > 0.0)) return LooseRateCapDegPerSec;
            double floor = MathHelpers.Rad2Deg(SettleStepFactor * minAngularAccelRadPerS2 * physicsDt);
            return Math.Min(LooseRateCapDegPerSec, floor);
        }

        // Pointed and rotationally settled this instant (pre-dwell).
        public static bool IsSteady(double errorDeg, double rateDegPerSec, double stillRateThresholdDegPerSec)
        {
            return errorDeg <= AlignStartDeg && rateDegPerSec <= stillRateThresholdDegPerSec;
        }
    }

    // Tracks the orient -> settle -> armed transition across frames. Arms only after the steady condition holds
    // CONTINUOUSLY for the dwell (so a transient zero-crossing during a coast-through can't arm it); disarms and
    // forces a re-orient if pointing drifts past the keep band. Value type: the default state equals Reset().
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
