using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

        public Vector3d ComputeAction(Vector3d error, Vector3d omega)
        {
            derivAction = omega * Kd;

            integralAccum.x = Math.Abs(derivAction.x) < 0.6 * _max ? integralAccum.x + error.x * Ki * TimeWarp.fixedDeltaTime : 0.9 * integralAccum.x;
            integralAccum.y = Math.Abs(derivAction.y) < 0.6 * _max ? integralAccum.y + error.y * Ki * TimeWarp.fixedDeltaTime : 0.9 * integralAccum.y;
            integralAccum.z = Math.Abs(derivAction.z) < 0.6 * _max ? integralAccum.z + error.z * Ki * TimeWarp.fixedDeltaTime : 0.9 * integralAccum.z;

            propAction = error * Kp;

            Vector3d action = propAction + derivAction + integralAccum;

            // action clamp
            action = new Vector3d(Math.Max(_min, Math.Min(_max, action.x)),
                Math.Max(_min, Math.Min(_max, action.y)),
                Math.Max(_min, Math.Min(_max, action.z)));
            return action;
        }
    }
}
