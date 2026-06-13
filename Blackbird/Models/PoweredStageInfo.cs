namespace Blackbird.Models
{
    public sealed class PoweredStageInfo
    {
        public bool IsValid { get; set; }
        public string ReasonUnavailable { get; set; }

        public int KspStage { get; set; }
        public int PhaseIndex { get; set; }
        public bool IsCurrentOrFutureStage { get; set; }

        public double StartMass { get; set; }
        public double EndMass { get; set; }
        public double DryMass { get; set; }
        public double FuelMass { get; set; }
        public double DecoupledMass { get; set; }

        public double VacuumSpecificImpulse { get; set; }
        public double SeaLevelSpecificImpulse { get; set; }
        public double CurrentSpecificImpulse { get; set; }

        public double VacuumThrust { get; set; }
        public double SeaLevelThrust { get; set; }
        public double CurrentThrust { get; set; }
        public double MinimumThrust { get; set; }
        public double MinimumThrottle { get; set; }

        public double VacuumDeltaV { get; set; }
        public double SeaLevelDeltaV { get; set; }
        public double CurrentDeltaV { get; set; }
        public double BurnTimeSeconds { get; set; }
    }
}
