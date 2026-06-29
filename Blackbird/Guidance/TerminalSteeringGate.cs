using Blackbird.Mathematics;
using UnityEngine;

namespace Blackbird.Guidance
{
    // Holds the last followable ascent steering vector. As the terminal orbit error -> 0 the optimal thrust
    // direction goes ill-conditioned and a solve can command a turn the craft can't fly; that command is rejected
    // and the last accepted direction is held. Pure + harness-tested; the KSP layer supplies the craft's slew cap.
    public struct TerminalSteeringGate
    {
        private Vector3d _held;
        private double _lastEvalUt;
        private bool _hasHeld;

        public bool HasHeld => _hasHeld;
        public Vector3d Held => _held;

        public void Reset()
        {
            _held = Vector3d.zero;
            _lastEvalUt = 0.0;
            _hasHeld = false;
        }

        // Feed the freshly-solved direction; returns the direction to fly
        public Vector3d Update(Vector3d solvedDirection, double ut, double maxRateRadPerSec)
        {
            Vector3d dir = solvedDirection.normalized;
            if (dir.sqrMagnitude <= 0.0) return _hasHeld ? _held : dir;

            double dt = ut - _lastEvalUt;
            _lastEvalUt = ut;

            if (_hasHeld && maxRateRadPerSec > 0.0 && dt > 0.0)
            {
                double cmdRate = MathHelpers.Deg2Rad(Vector3d.Angle(_held, dir)) / dt;
                if (cmdRate > maxRateRadPerSec) return _held;   // unfollowable this tick -> hold last clean direction
            }

            _held = dir;
            _hasHeld = true;
            return dir;
        }
    }
}
