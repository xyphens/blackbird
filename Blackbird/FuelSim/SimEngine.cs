using System.Collections.Generic;

namespace Blackbird.FuelSim
{
    // Resource routing rules, same member order as KSP's global ResourceFlowMode.
    public enum FlowMode
    {
        NoFlow,
        AllVessel,
        StagePriorityFlow,
        StackPrioritySearch,
        AllVesselBalance,
        StagePriorityFlowBalance,
        StageStackFlow,
        StageStackFlowBalance,
        Null
    }

    public struct EnginePropellant
    {
        public int ResourceId;
        public double Ratio;
        public double DensityTonsPerUnit;
        public FlowMode FlowMode;
        public bool IgnoreForIsp;
    }

    // One engine module at fixed vacuum conditions and full commanded throttle.
    public sealed class SimEngine
    {
        public SimPart Part;

        public readonly List<EnginePropellant> Propellants = new List<EnginePropellant>();
        public readonly Dictionary<int, FlowMode> PropellantFlowModes = new Dictionary<int, FlowMode>();
        public readonly Dictionary<int, double> ResourceConsumptions = new Dictionary<int, double>(); // units/s at full throttle

        public bool IsEnabled;
        public bool IsOperational;
        public bool ThrottleLocked;
        public bool NoPropellants;      // engine is already flamed out with empty tanks in the live vessel
        public double ThrustLimiter;    // 0..1 from the tweakable thrust percentage
        public double MinFuelFlowTons;  // tons/s
        public double MaxFuelFlowTons;  // tons/s
        public double MinThrustKn;
        public double MaxThrustKn;
        public double VacuumIsp;        // seconds, multIsp folded in
        public double MultFlow = 1.0;
        public double GravityIsp = 9.80665;
        public double ModuleResiduals;  // RealFuels unusable-propellant fraction

        public double MassFlowTons { get; private set; }         // full-throttle vacuum flow
        public double ThrustKn { get; private set; }             // full-throttle vacuum thrust after limiter
        public double MinThrottleThrustKn { get; private set; }  // lowest thrust the engine can hold

        // Derives the fixed vacuum flow, thrust and per-resource consumption rates. Call once after loading fields.
        public void Initialize()
        {
            double minFlow = MinFuelFlowTons;
            double maxFlow = MaxFuelFlowTons;
            double ispG = VacuumIsp * GravityIsp;

            if (minFlow == 0.0 && MinThrustKn > 0.0 && ispG > 0.0) minFlow = MinThrustKn / ispG;
            if (maxFlow == 0.0 && MaxThrustKn > 0.0 && ispG > 0.0) maxFlow = MaxThrustKn / ispG;

            MassFlowTons = minFlow + (maxFlow - minFlow) * ThrustLimiter;
            ThrustKn = MassFlowTons * ispG * MultFlow;
            MinThrottleThrustKn = ThrottleLocked ? ThrustKn : minFlow * ispG * MultFlow;

            SetConsumptionRates();
        }

        // Splits the mass flow into per-resource volume rates by propellant ratio and density.
        private void SetConsumptionRates()
        {
            ResourceConsumptions.Clear();
            PropellantFlowModes.Clear();

            double totalDensity = 0.0;

            for (int i = 0; i < Propellants.Count; i++)
            {
                EnginePropellant p = Propellants[i];

                // zero-density propellants (EC, intake air) are treated as available and infinite
                if (p.DensityTonsPerUnit <= 0.0) continue;

                if (!PropellantFlowModes.ContainsKey(p.ResourceId))
                {
                    PropellantFlowModes.Add(p.ResourceId, p.FlowMode);
                }

                if (p.IgnoreForIsp) continue;

                totalDensity += p.Ratio * p.DensityTonsPerUnit;
            }

            if (totalDensity <= 0.0) return;

            double volumeFlowRate = MassFlowTons / totalDensity;

            for (int i = 0; i < Propellants.Count; i++)
            {
                EnginePropellant p = Propellants[i];
                if (p.DensityTonsPerUnit <= 0.0) continue;

                double propVolumeRate = p.Ratio * volumeFlowRate;

                double existing;
                if (ResourceConsumptions.TryGetValue(p.ResourceId, out existing))
                {
                    ResourceConsumptions[p.ResourceId] = existing + propVolumeRate;
                }
                else
                {
                    ResourceConsumptions.Add(p.ResourceId, propVolumeRate);
                }
            }
        }

        // Flames the engine out when any propellant is unreachable through its flow mode.
        public void UpdateEngineStatus()
        {
            if (CanDrawResources()) return;
            IsOperational = false;
        }

        private bool CanDrawResources()
        {
            if (NoPropellants) return false;

            foreach (KeyValuePair<int, FlowMode> entry in PropellantFlowModes)
            {
                int resourceId = entry.Key;

                switch (entry.Value)
                {
                    case FlowMode.NoFlow:
                        if (!PartHasResource(Part, resourceId)) return false;
                        break;
                    case FlowMode.AllVessel:
                    case FlowMode.AllVesselBalance:
                    case FlowMode.StagePriorityFlow:
                    case FlowMode.StagePriorityFlowBalance:
                        if (!PartsHaveResource(Part.Vessel.PartsRemainingInStage(Part.Vessel.CurrentStage), resourceId)) return false;
                        break;
                    case FlowMode.StageStackFlow:
                    case FlowMode.StageStackFlowBalance:
                    case FlowMode.StackPrioritySearch:
                        if (!PartsHaveResource(Part.CrossFeedPartSet, resourceId)) return false;
                        break;
                    default:
                        return false;
                }
            }

            return true;
        }

        private bool PartHasResource(SimPart part, int resourceId)
        {
            Resource resource;
            if (!part.TryGetResource(resourceId, out resource)) return false;

            return resource.Amount > resource.MaxAmount * ModuleResiduals + part.ResourceRequestRemainingThreshold;
        }

        private bool PartsHaveResource(IReadOnlyList<SimPart> parts, int resourceId)
        {
            for (int i = 0; i < parts.Count; i++)
            {
                if (PartHasResource(parts[i], resourceId)) return true;
            }

            return false;
        }

        // True when staging the given stage would jettison a tank this engine can still drain.
        public bool WouldDropAccessibleFuelTank(int stageNum)
        {
            foreach (KeyValuePair<int, FlowMode> entry in PropellantFlowModes)
            {
                int resourceId = entry.Key;

                switch (entry.Value)
                {
                    case FlowMode.NoFlow:
                        if (PartHasResource(Part, resourceId) && Part.DecoupledInStage == stageNum) return true;
                        break;
                    case FlowMode.AllVessel:
                    case FlowMode.AllVesselBalance:
                    case FlowMode.StagePriorityFlow:
                    case FlowMode.StagePriorityFlowBalance:
                        if (DrawsFromPartsDroppedInStage(Part.Vessel.PartsRemainingInStage(Part.Vessel.CurrentStage), resourceId, stageNum)) return true;
                        break;
                    case FlowMode.StageStackFlow:
                    case FlowMode.StageStackFlowBalance:
                    case FlowMode.StackPrioritySearch:
                        if (DrawsFromPartsDroppedInStage(Part.CrossFeedPartSet, resourceId, stageNum)) return true;
                        break;
                }
            }

            return false;
        }

        private bool DrawsFromPartsDroppedInStage(IReadOnlyList<SimPart> parts, int resourceId, int stageNum)
        {
            for (int i = 0; i < parts.Count; i++)
            {
                if (PartHasResource(parts[i], resourceId) && parts[i].DecoupledInStage == stageNum) return true;
            }

            return false;
        }
    }
}
