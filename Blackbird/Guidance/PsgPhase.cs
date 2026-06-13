using System;
using System.Collections.Generic;
using Blackbird.Mathematics;
using Blackbird.Models;

namespace Blackbird.Guidance
{
    public sealed class PsgPhase
    {
        private const double StandardGravity = 9.80665;
        private const double KilogramsPerKspTon = 1000.0;
        private const double NewtonsPerKilonewton = 1000.0;
        private const double MinimumUsablePropellantMassKg = 1.0;
        private const double MinimumUsableBurnTimeSeconds = 0.05;

        public bool IsValid { get; private set; }
        public string ReasonUnavailable { get; private set; }

        public int KspStage { get; private set; }
        public int PhaseIndex { get; private set; }

        public double StartMassKg { get; private set; }
        public double EndMassKg { get; private set; }
        public double VacuumThrustNewtons { get; private set; }
        public double VacuumSpecificImpulseSeconds { get; private set; }
        public double CurrentSpecificImpulseSeconds { get; private set; }
        public double MassFlowKgPerSecond { get; private set; }
        public double NominalBurnTimeSeconds { get; private set; }
        public double MinimumBurnTimeSeconds { get; private set; }
        public double MaximumBurnTimeSeconds { get; private set; }
        public double MinimumThrottle { get; private set; }

        public bool IsCoast { get; private set; }
        public bool IsUnguided { get; private set; }
        public bool AllowShutdown { get; private set; }
        public bool EnforceMassContinuity { get; private set; }

        public double ExhaustVelocityVacuumMetersPerSecond
        {
            get { return VacuumSpecificImpulseSeconds * StandardGravity; }
        }

        public double ExhaustVelocityCurrentMetersPerSecond
        {
            get { return CurrentSpecificImpulseSeconds * StandardGravity; }
        }

        public double PropellantMassKg
        {
            get { return Math.Max(0.0, StartMassKg - EndMassKg); }
        }

        public static PsgPhase[] FromPoweredStages(PoweredStageInfo[] poweredStages)
        {
            if (poweredStages == null || poweredStages.Length == 0) return new PsgPhase[0];

            var phases = new List<PsgPhase>();

            for (int i = 0; i < poweredStages.Length; i++)
            {
                PsgPhase phase = FromPoweredStage(poweredStages[i], false);
                if (phase.IsValid) phases.Add(phase);
            }

            return phases.ToArray();
        }

        public static PsgPhase FromPoweredStage(PoweredStageInfo stage, bool enforceMassContinuity)
        {
            if (stage == null)
            {
                return CreateInvalid(-1, -1, "Powered stage is unavailable.");
            }

            if (!stage.IsValid)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, stage.ReasonUnavailable);
            }

            if (!stage.IsCurrentOrFutureStage)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage is not current or future.");
            }

            double startMassKg = stage.StartMass * KilogramsPerKspTon;
            double endMassKg = stage.EndMass * KilogramsPerKspTon;
            double vacuumThrustNewtons = stage.VacuumThrust * NewtonsPerKilonewton;
            double vacuumIsp = stage.VacuumSpecificImpulse;
            double currentIsp = OrbitMath.IsFinite(stage.CurrentSpecificImpulse) && stage.CurrentSpecificImpulse > 0.0
                ? stage.CurrentSpecificImpulse
                : vacuumIsp;

            if (!OrbitMath.IsFinite(startMassKg) || startMassKg <= 0.0 ||
                !OrbitMath.IsFinite(endMassKg) || endMassKg <= 0.0 ||
                endMassKg > startMassKg)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage mass bounds are invalid.");
            }

            if (startMassKg - endMassKg <= MinimumUsablePropellantMassKg)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage has no usable propellant.");
            }

            if (!OrbitMath.IsFinite(vacuumThrustNewtons) || vacuumThrustNewtons <= 0.0)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage vacuum thrust is unavailable.");
            }

            if (!OrbitMath.IsFinite(vacuumIsp) || vacuumIsp <= 0.0)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage vacuum specific impulse is unavailable.");
            }

            double massFlow = vacuumThrustNewtons / (vacuumIsp * StandardGravity);
            if (!OrbitMath.IsFinite(massFlow) || massFlow <= 0.0)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage mass flow cannot be derived.");
            }

            double nominalBurnTime = OrbitMath.IsFinite(stage.BurnTimeSeconds) && stage.BurnTimeSeconds > 0.0
                ? stage.BurnTimeSeconds
                : (startMassKg - endMassKg) / massFlow;

            if (!OrbitMath.IsFinite(nominalBurnTime) || nominalBurnTime <= 0.0)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage burn time cannot be derived.");
            }

            if (nominalBurnTime <= MinimumUsableBurnTimeSeconds)
            {
                return CreateInvalid(stage.KspStage, stage.PhaseIndex, "Stage burn time is too short to guide.");
            }

            double minimumThrottle = OrbitMath.Clamp(stage.MinimumThrottle, 0.0, 1.0);
            double maximumBurnTime = minimumThrottle > 0.0
                ? nominalBurnTime / minimumThrottle
                : double.PositiveInfinity;

            return new PsgPhase
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                KspStage = stage.KspStage,
                PhaseIndex = stage.PhaseIndex,
                StartMassKg = startMassKg,
                EndMassKg = endMassKg,
                VacuumThrustNewtons = vacuumThrustNewtons,
                VacuumSpecificImpulseSeconds = vacuumIsp,
                CurrentSpecificImpulseSeconds = currentIsp,
                MassFlowKgPerSecond = massFlow,
                NominalBurnTimeSeconds = nominalBurnTime,
                MinimumBurnTimeSeconds = 0.0,
                MaximumBurnTimeSeconds = maximumBurnTime,
                MinimumThrottle = minimumThrottle,
                IsCoast = false,
                IsUnguided = false,
                AllowShutdown = true,
                EnforceMassContinuity = enforceMassContinuity
            };
        }

        public static PsgPhase CreateCoast(
            double massKg,
            double minimumTimeSeconds,
            double maximumTimeSeconds,
            int kspStage,
            int phaseIndex,
            bool isUnguided,
            bool enforceMassContinuity)
        {
            if (!OrbitMath.IsFinite(massKg) || massKg <= 0.0)
            {
                return CreateInvalid(kspStage, phaseIndex, "Coast mass is invalid.");
            }

            if (!OrbitMath.IsFinite(minimumTimeSeconds) || minimumTimeSeconds < 0.0 ||
                !OrbitMath.IsFinite(maximumTimeSeconds) || maximumTimeSeconds < minimumTimeSeconds)
            {
                return CreateInvalid(kspStage, phaseIndex, "Coast time bounds are invalid.");
            }

            return new PsgPhase
            {
                IsValid = true,
                ReasonUnavailable = string.Empty,
                KspStage = kspStage,
                PhaseIndex = phaseIndex,
                StartMassKg = massKg,
                EndMassKg = massKg,
                VacuumThrustNewtons = 0.0,
                VacuumSpecificImpulseSeconds = 0.0,
                CurrentSpecificImpulseSeconds = 0.0,
                MassFlowKgPerSecond = 0.0,
                NominalBurnTimeSeconds = minimumTimeSeconds,
                MinimumBurnTimeSeconds = minimumTimeSeconds,
                MaximumBurnTimeSeconds = maximumTimeSeconds,
                MinimumThrottle = 0.0,
                IsCoast = true,
                IsUnguided = isUnguided,
                AllowShutdown = false,
                EnforceMassContinuity = enforceMassContinuity
            };
        }

        private static PsgPhase CreateInvalid(int kspStage, int phaseIndex, string reason)
        {
            return new PsgPhase
            {
                IsValid = false,
                ReasonUnavailable = string.IsNullOrEmpty(reason) ? "PSG phase is unavailable." : reason,
                KspStage = kspStage,
                PhaseIndex = phaseIndex
            };
        }
    }
}
