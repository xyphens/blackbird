using System;

namespace Blackbird.Guidance
{
    public class RcsPID
    {
        private Vector3d integralAccum;
        public double Ki;
        private Vector3d derivAction;
        public double Kd;
        private Vector3d propAction;
        public double Kp;
        private readonly double _max;
        private readonly double _min;

        public void Reset() => integralAccum = Vector3d.zero;

        public RcsPID(double _Kp = 0, double _Ki = 0, double _Kd = 0, double max = double.MaxValue, double min = double.MinValue)
        {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
            _max = max;
            _min = min;
        }

        public void Load(double _Kp, double _Ki, double _Kd)
        {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
        }

        public Vector3d ComputeAction(Vector3d error, Vector3d omega, double dt)
        {
            derivAction = omega * Kd;
            propAction = error * Kp;

            // Clamping anti-windup: integrate per axis only when the accumulated output would not sit past a
            // rail while the error keeps pushing it there — so a steady error can't ramp the integral without
            // bound. (The old guard keyed off the derivative, which is ~0 during a steady hold, and the live
            // controller had _max = +inf, so it never engaged.) With real ±limits set at construction, the
            // integral is bounded by the actuation limit.
            integralAccum.x = Integrate(integralAccum.x, error.x, propAction.x + derivAction.x, dt);
            integralAccum.y = Integrate(integralAccum.y, error.y, propAction.y + derivAction.y, dt);
            integralAccum.z = Integrate(integralAccum.z, error.z, propAction.z + derivAction.z, dt);

            return new Vector3d(
                Math.Max(_min, Math.Min(_max, propAction.x + derivAction.x + integralAccum.x)),
                Math.Max(_min, Math.Min(_max, propAction.y + derivAction.y + integralAccum.y)),
                Math.Max(_min, Math.Min(_max, propAction.z + derivAction.z + integralAccum.z)));
        }

        private double Integrate(double accum, double error, double pdOut, double dt)
        {
            double next = accum + error * Ki * dt;
            if (pdOut + next >= _max && error > 0.0) return accum;   // would saturate high & error pushes higher -> hold
            if (pdOut + next <= _min && error < 0.0) return accum;   // would saturate low & error pushes lower -> hold
            return next;
        }
    }
}
