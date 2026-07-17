namespace Blackbird.Planning
{
    public enum FeasibilityLevel { Green, Yellow, Red };
    public struct FeasibilityResult
    {
        public FeasibilityLevel Level;
        public double Penalty; // added to candidate score, lower is better
        public string Note;
    }
    public static class Feasibility
    {
        // dufixme: probably want to make this inputs.  violates the no-magic-number rule.
        // vacuum dV thresholds
        private const double TwrComfort             = 0.50; // >= this: is clean insertion
        private const double TwrFloor               = 0.30; // < this: heavy loft, PSG won't converge
        private const double ReserveDvMps           = 200.0; // dV headroom above the estimate
        private const double TwrPenaltyScale        = 1000.0;
        private const double DvPenaltyPerMps        = 5.0;
        private const double InfeasiblePenalty      = 1e6; // matches existing fuel cliff in ScoreCandidate

        public static FeasibilityResult Evaluate(
                        double upperStageVacThrustN, double upperStageMassKg, double targetRadiusM,
                        double mu, double availDv, double reqDv)
        {
            if (!(upperStageVacThrustN > 0.0) || !(upperStageMassKg > 0.0) || !(targetRadiusM > 0.0) || !(mu > 0.0))
            {
                return new FeasibilityResult
                {
                    Level = FeasibilityLevel.Red,
                    Penalty = InfeasiblePenalty,
                    Note = "insufficient vehicle/plan data"
                };
            }


            double gAtTarget = mu / (targetRadiusM * targetRadiusM);
            double upperTwr = upperStageVacThrustN / (upperStageMassKg * gAtTarget);
            double dvMargin = availDv - reqDv;

            bool twrHardFail = upperTwr < TwrFloor;
            bool dvHardFail = dvMargin < 0.0;

            double twrPenalty = upperTwr >= TwrComfort ? 0.0
                                : (TwrComfort - upperTwr) / TwrComfort * TwrPenaltyScale;
            double dvPenalty = dvMargin >= ReserveDvMps ? 0.0
                                : (ReserveDvMps - dvMargin) * DvPenaltyPerMps;
            double penalty = twrPenalty + dvPenalty + (twrHardFail || dvHardFail ? InfeasiblePenalty : 0.0);

            FeasibilityLevel level = twrHardFail || dvHardFail ? FeasibilityLevel.Red :
                                        twrHardFail ? FeasibilityLevel.Yellow :
                                        FeasibilityLevel.Green;

            string note =
                        level == FeasibilityLevel.Green ? string.Empty :
                        twrHardFail ? $"upper-stage TWR {upperTwr:F2} too low to reach this altitude" :
                        dvHardFail ? $"insufficient dV: {-dvMargin:F0} m/s short" :
                        upperTwr < TwrComfort ? $"upper-stage TWR {upperTwr:F2} marginal (loft risk)" :
                                                 $"dV margin thin: {dvMargin:F0} m/s";

            return new FeasibilityResult { Level = level, Penalty = penalty, Note = note };
        }
    }
}
