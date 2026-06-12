using System;

namespace Blackbird.Guidance
{
    public sealed class PidHandler
    {
        private double _ei1;
        private double _ed1;
        private double _u1 = double.NaN;
        private double _y1 = double.NaN;

        public double PTerm { get; private set; }
        public double ITerm { get; private set; }
        public double DTerm { get; private set; }

        public double K { get; set; } = 1.0;
        public double Ti { get; set; }
        public double Td { get; set; }
        public double N { get; set; } = 50.0;
        public double Ts { get; set; } = 0.02;

        public double Kp { get => K; set => K = value; }

        public double B { get; set; } = 1.0;
        public double C { get; set; } = 1.0;

        public double SmoothIn { get; set; } = 1.0;
        public double SmoothOut { get; set; } = 1.0;
        public double ProportionalDeadband { get; set; }
        public double IntegralDeadband { get; set; }
        public double DerivativeDeadband { get; set; }
        public double OutputDeadband { get; set; }

        public double MinOutput { get; set; } = double.MinValue;
        public double MaxOutput { get; set; } = double.MaxValue;

        public bool Clegg { get; set; }

        /**
         * Outputs a correction value using the proportional term, integral term and derivative term.  Used to:
         * 1.  translate position error to target angular velocity
         * 2.  translate angular velocity error to target angular acceleration
         * double r: requested value
         * double y = measured/current value
        **/
        public double Update(double r, double y)
        {
            y = IsFinite(_y1) ? _y1 + SmoothIn * (y - _y1) : y;

            double ep = ApplyDeadband(B * r - y, ProportionalDeadband);
            double ei = ApplyDeadband(r - y, IntegralDeadband);
            double ed = ApplyDeadband(C * r - y, DerivativeDeadband);

            PTerm = K * ep;

            if (Clegg && ei *ITerm < 0.0) ITerm = 0.0;

            double k = K == 0.0 ? 1.0 : K;
            ITerm += 0.5 * k * Ts * (ei + _ei1) / Ti;

            double den = 2.0 * Td + N * Ts;
            DTerm = (2.0 * Td - N * Ts) / den * DTerm + 2.0 * N * k * Td / den * (ed - _ed1);

            if (!IsFinite(ITerm)) ITerm = 0.0;
            if (!IsFinite(ITerm)) DTerm = 0.0;

            double z = ApplyDeadband(PTerm + ITerm + DTerm, OutputDeadband);
            double u = Clamp(z, MinOutput, MaxOutput);

            if (Ti != 0.0)
            {
                double tr = Td == 0.0 ? Ti : Math.Sqrt(Ti * Td);
                ITerm += Ts / tr * (u - z);
            }

            _u1 = IsFinite(_u1) ? _u1 + SmoothOut * (u - _u1) : u;

            _y1 = y;
            _ei1 = ei;
            _ed1 = ed;

            return _u1;
        }

        public void Reset()
        {
            ITerm = 0.0;
            DTerm = 0.0;
            _ei1 = 0.0;
            _y1 = double.NaN;
            _u1 = double.NaN;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static double ApplyDeadband(double value, double deadband)
        {
            return Math.Abs(value) < deadband ? 0.0 : value - Math.Sign(value) * deadband;
        }

        private static double Clamp(double value, double min, double max)
        {
            return value < min ? min : value > max ? max : value;
        }
    }
}
