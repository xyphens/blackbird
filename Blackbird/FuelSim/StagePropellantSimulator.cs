using System;
using System.Collections.Generic;
using System.Reflection;
using Blackbird.Mathematics;
using Blackbird.Psg;

namespace Blackbird.FuelSim
{
    public struct SimStageStats
    {
        public int Stage;
        public double StartMassKg;
        public double EndMassKg;            // at flameout of reachable, compatible propellant
        public double BurnablePropellantKg; // Start − End, the honest number
        public double VacuumThrustNewtons;  // burn-time-weighted sum of active engines
        public double VacuumIspSeconds;     // mass-flow-weighted
        public double MinimumThrottle;
        public double FullThrottleBurnTimeSeconds;
    }

    // Simulates the vessel's staged burn at full throttle in vacuum by draining tanks through
    // each engine's propellant flow rules, yielding honest per-stage mass and burn-time stats.
    public static class StagePropellantSimulator
    {
        // The drain loop is event-driven (each iteration ends at a tank-empty or flameout event), so a
        // stage needs roughly one iteration per tank group. 100 is a runaway guard, not a resolution knob.
        private const int MaxStepsPerStage = 100;

        // Builds the simulation snapshot from the live vessel and runs it. Flight scene only.
        public static SimStageStats[] Simulate(Vessel vessel)
        {
            if (vessel == null || vessel.parts == null) return new SimStageStats[0];

            // KSP rebuilds crossfeed part sets lazily; reading a stale set routes fuel across closed
            // crossfeed boundaries (e.g. through the tanker's SLA adapter). MechJeb performs the same
            // refresh before every simulation build.
            vessel.UpdateResourceSetsIfDirty();

            SimVessel simVessel = BuildVessel(vessel);
            return Run(simVessel);
        }

        // Routing diagnostic: the sim's view of every fuel-relevant part (staging, priority, crossfeed
        // reach, resources) and each engine's propellant flow modes. One line; log on stage change.
        public static string DescribeParts(Vessel kspVessel)
        {
            if (kspVessel == null || kspVessel.parts == null) return "no vessel";

            kspVessel.UpdateResourceSetsIfDirty();
            SimVessel vessel = BuildVessel(kspVessel);

            var sb = new System.Text.StringBuilder();
            sb.Append("stage=").Append(vessel.CurrentStage).Append(" parts=").Append(vessel.Parts.Count);

            foreach (SimPart part in vessel.Parts)
            {
                if (part.Resources.Count == 0 && part.Engines.Count == 0 && part.DisabledResourcesMassTons <= 0.0) continue;

                sb.Append(" || ").Append(part.Name)
                  .Append(" inv=").Append(part.InverseStage)
                  .Append(" dec=").Append(part.DecoupledInStage)
                  .Append(" pri=").Append(part.ResourcePriority)
                  .Append(" xf=").Append(part.CrossFeedPartSet.Count);

                foreach (Resource resource in part.Resources.Values)
                {
                    sb.Append(' ').Append(ResourceName(resource.Id)).Append('=').Append(resource.Amount.ToString("F0"));
                }

                if (part.DisabledResourcesMassTons > 0.0)
                {
                    sb.Append(" lockedMass=").Append(part.DisabledResourcesMassTons.ToString("F1")).Append('t');
                }

                foreach (SimEngine engine in part.Engines)
                {
                    sb.Append(" ENG[");
                    foreach (EnginePropellant propellant in engine.Propellants)
                    {
                        sb.Append(ResourceName(propellant.ResourceId)).Append(':').Append(propellant.FlowMode).Append(' ');
                    }
                    sb.Append(']');
                }
            }

            return sb.ToString();
        }

        private static string ResourceName(int resourceId)
        {
            PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(resourceId);
            return definition != null ? definition.name : resourceId.ToString();
        }

        // Runs the staged drain simulation. Public so harnesses can drive a hand-built vessel.
        // Results are in firing order: the currently burning stage first, stage 0 last.
        public static SimStageStats[] Run(SimVessel vessel)
        {
            var stats = new List<SimStageStats>();
            var partsWithDrains = new HashSet<SimPart>();

            vessel.ActivateEngines();

            while (vessel.CurrentStage >= 0)
            {
                stats.Add(SimulateStage(vessel, partsWithDrains));
                ClearDrainsAndResiduals(partsWithDrains);
                vessel.Stage();
            }

            return stats.ToArray();
        }

        private static SimStageStats SimulateStage(SimVessel vessel, HashSet<SimPart> partsWithDrains)
        {
            vessel.UpdateMass();
            vessel.UpdateActiveEngines();

            double startMassTons = vessel.MassTons;
            double startThrustKn = vessel.ThrustKn;
            double startMinThrustKn = vessel.MinThrottleThrustKn;
            double burnTime = 0.0;
            double thrustTimeIntegral = 0.0; // kN·s

            UpdateResourceDrainsAndResiduals(vessel, partsWithDrains);

            for (int steps = MaxStepsPerStage; steps > 0; steps--)
            {
                if (AllowedToStage(vessel))
                {
                    return FinishStage(vessel, startMassTons, startThrustKn, startMinThrustKn, burnTime, thrustTimeIntegral);
                }

                double dt = MinimumTimeStep(partsWithDrains);

                thrustTimeIntegral += vessel.ThrustKn * dt;
                burnTime += dt;

                foreach (SimPart part in partsWithDrains)
                {
                    part.ApplyResourceDrains(dt);
                }

                vessel.UpdateMass();
                vessel.UpdateActiveEngines();
                UpdateResourceDrainsAndResiduals(vessel, partsWithDrains);
            }

            throw new InvalidOperationException("FuelSim exceeded " + MaxStepsPerStage + " steps in stage " + vessel.CurrentStage);
        }

        private static SimStageStats FinishStage(
            SimVessel vessel, double startMassTons, double startThrustKn, double startMinThrustKn,
            double burnTime, double thrustTimeIntegral)
        {
            double endMassTons = vessel.MassTons;
            double burnedTons = Math.Max(0.0, startMassTons - endMassTons);

            return new SimStageStats
            {
                Stage = vessel.CurrentStage,
                StartMassKg = startMassTons * 1000.0,
                EndMassKg = endMassTons * MathHelpers.KilogramsPerTon,
                BurnablePropellantKg = burnedTons * MathHelpers.KilogramsPerTon,
                FullThrottleBurnTimeSeconds = burnTime,
                VacuumThrustNewtons = burnTime > 0.0
                    ? thrustTimeIntegral / burnTime * MathHelpers.NewtonsPerKilonewton
                    : startThrustKn * MathHelpers.NewtonsPerKilonewton,
                VacuumIspSeconds = burnedTons > 0.0
                    ? thrustTimeIntegral * MathHelpers.NewtonsPerKilonewton / (MathHelpers.StandardGravity * burnedTons * MathHelpers.KilogramsPerTon)
                    : 0.0,
                MinimumThrottle = startThrustKn > 0.0
                    ? Math.Min(1.0, Math.Max(0.0, startMinThrustKn / startThrustKn))
                    : 0.0
            };
        }

        // Staging rules: never drop a burning engine or a tank a burning engine can still reach,
        // and only fire a stage that actually separates something (unless everything is burned out).
        private static bool AllowedToStage(SimVessel vessel)
        {
            List<SimEngine> active = vessel.ActiveEngines;

            if (active.Count == 0) return true;

            for (int i = 0; i < active.Count; i++)
            {
                SimEngine e = active[i];

                if (e.Part.IsSepratron) continue;
                if (e.Part.DecoupledInStage >= vessel.CurrentStage - 1) return false;
                if (e.WouldDropAccessibleFuelTank(vessel.CurrentStage - 1)) return false;
            }

            if (vessel.PartsRemainingInStage(vessel.CurrentStage - 1).Count == vessel.PartsRemainingInStage(vessel.CurrentStage).Count)
            {
                return false;
            }

            return vessel.CurrentStage > 0;
        }

        private static double MinimumTimeStep(HashSet<SimPart> partsWithDrains)
        {
            double maxTime = double.MaxValue;

            foreach (SimPart part in partsWithDrains)
            {
                maxTime = Math.Min(maxTime, part.ResourceMaxTime());
            }

            return maxTime < double.MaxValue && maxTime >= 0.0 ? maxTime : 0.0;
        }

        private static void ClearDrainsAndResiduals(HashSet<SimPart> partsWithDrains)
        {
            foreach (SimPart part in partsWithDrains)
            {
                part.ClearResourceDrains();
                part.ClearResiduals();
            }

            partsWithDrains.Clear();
        }

        // Routes every active engine's consumption onto source tanks per KSP flow rules.
        private static void UpdateResourceDrainsAndResiduals(SimVessel vessel, HashSet<SimPart> partsWithDrains)
        {
            ClearDrainsAndResiduals(partsWithDrains);

            for (int i = 0; i < vessel.ActiveEngines.Count; i++)
            {
                SimEngine e = vessel.ActiveEngines[i];

                foreach (KeyValuePair<int, FlowMode> entry in e.PropellantFlowModes)
                {
                    int resourceId = entry.Key;
                    double consumption = e.ResourceConsumptions[resourceId];

                    switch (entry.Value)
                    {
                        case FlowMode.NoFlow:
                            AddDrain(partsWithDrains, e.Part, consumption, resourceId, e.ModuleResiduals);
                            break;
                        case FlowMode.AllVessel:
                        case FlowMode.AllVesselBalance:
                            AddDrains(partsWithDrains, vessel.PartsRemainingInStage(vessel.CurrentStage), consumption, resourceId, false, e.ModuleResiduals);
                            break;
                        case FlowMode.StagePriorityFlow:
                        case FlowMode.StagePriorityFlowBalance:
                            AddDrains(partsWithDrains, vessel.PartsRemainingInStage(vessel.CurrentStage), consumption, resourceId, true, e.ModuleResiduals);
                            break;
                        case FlowMode.StageStackFlow:
                        case FlowMode.StageStackFlowBalance:
                        case FlowMode.StackPrioritySearch:
                            AddDrains(partsWithDrains, e.Part.CrossFeedPartSet, consumption, resourceId, true, e.ModuleResiduals);
                            break;
                    }
                }
            }
        }

        private static readonly List<SimPart> _sources = new List<SimPart>();

        private static void AddDrains(
            HashSet<SimPart> partsWithDrains, IReadOnlyList<SimPart> parts, double consumption, int resourceId,
            bool usePriority, double residual)
        {
            int maxPriority = int.MinValue;

            _sources.Clear();

            for (int i = 0; i < parts.Count; i++)
            {
                SimPart p = parts[i];

                Resource resource;
                if (!p.TryGetResource(resourceId, out resource)) continue;
                if (resource.Free) continue;
                if (resource.Amount <= residual * resource.MaxAmount + p.ResourceRequestRemainingThreshold) continue;

                if (usePriority)
                {
                    if (p.ResourcePriority < maxPriority) continue;

                    if (p.ResourcePriority > maxPriority)
                    {
                        _sources.Clear();
                        maxPriority = p.ResourcePriority;
                    }
                }

                _sources.Add(p);
            }

            for (int i = 0; i < _sources.Count; i++)
            {
                AddDrain(partsWithDrains, _sources[i], consumption / _sources.Count, resourceId, residual);
            }
        }

        private static void AddDrain(HashSet<SimPart> partsWithDrains, SimPart p, double consumption, int resourceId, double residual)
        {
            partsWithDrains.Add(p);
            p.AddResourceDrain(resourceId, consumption);
            p.UpdateResourceResidual(residual, resourceId);
        }

        // ---- KSP vessel ingestion (flight scene) ----

        private static SimVessel BuildVessel(Vessel kspVessel)
        {
            var vessel = new SimVessel();
            vessel.SetCurrentStage(kspVessel.currentStage);

            var partMap = new Dictionary<Part, SimPart>();
            var pendingAttachments = new List<KeyValuePair<Decoupler, Part>>();

            foreach (Part part in kspVessel.parts)
            {
                SimPart simPart = BuildPart(vessel, part, pendingAttachments);
                vessel.Parts.Add(simPart);
                partMap.Add(part, simPart);
            }

            // second pass: cross-part references need the completed map
            foreach (Part kspPart in kspVessel.parts)
            {
                SimPart part = partMap[kspPart];

                if (kspPart.parent != null)
                {
                    part.Links.Add(partMap[kspPart.parent]);
                }

                foreach (Part child in kspPart.children)
                {
                    SimPart mapped;
                    if (partMap.TryGetValue(child, out mapped))
                    {
                        part.Links.Add(mapped);
                    }
                }

                if (kspPart.crossfeedPartSet != null)
                {
                    foreach (Part member in kspPart.crossfeedPartSet.GetParts())
                    {
                        SimPart mapped;
                        if (partMap.TryGetValue(member, out mapped))
                        {
                            part.CrossFeedPartSet.Add(mapped);
                        }
                    }
                }
            }

            foreach (KeyValuePair<Decoupler, Part> pending in pendingAttachments)
            {
                SimPart mapped;
                if (pending.Value != null && partMap.TryGetValue(pending.Value, out mapped))
                {
                    pending.Key.AttachedPart = mapped;
                }
            }

            vessel.FinalizeBuild();
            return vessel;
        }

        private static SimPart BuildPart(SimVessel vessel, Part part, List<KeyValuePair<Decoupler, Part>> pendingAttachments)
        {
            var simPart = new SimPart(vessel, part.partInfo != null ? part.partInfo.name : part.name)
            {
                InverseStage = part.inverseStage,
                StagingOn = part.stagingOn,
                ActivatesEvenIfDisconnected = part.ActivatesEvenIfDisconnected,
                ResourcePriority = part.GetResourcePriority(),
                ResourceRequestRemainingThreshold = part.resourceRequestRemainingThreshold,
                IsRoot = part.parent == null,
                DryMassTons = part.mass,
                CrewMassTons = GetCrewMassTons(part)
            };

            foreach (PartResource resource in part.Resources)
            {
                if (resource.info == null) continue;

                if (!resource.flowState)
                {
                    simPart.DisabledResourcesMassTons += resource.amount * resource.info.density;
                    continue;
                }

                simPart.Resources[resource.info.id] = new Resource
                {
                    Id = resource.info.id,
                    Amount = resource.amount,
                    MaxAmount = resource.maxAmount,
                    Density = resource.info.density,
                    Free = resource.info.density == 0.0,
                    Residual = 0.0
                };
            }

            foreach (PartModule module in part.Modules)
            {
                BuildModule(simPart, part, module, pendingAttachments);
            }

            return simPart;
        }

        private static void BuildModule(SimPart simPart, Part part, PartModule module, List<KeyValuePair<Decoupler, Part>> pendingAttachments)
        {
            if (part.FindModuleImplementing<LaunchClamp>() != null 
                || module is global::LaunchClamp) {
                simPart.IsLaunchClamp = true;
                return;
            }

            var kspEngine = module as ModuleEngines;
            if (kspEngine != null)
            {
                // air-breathing engines produce nothing in vacuum and are irrelevant to orbital stats
                if (!kspEngine.atmChangeFlow)
                {
                    BuildEngine(simPart, kspEngine);
                }

                simPart.IsThrottleLocked = kspEngine.throttleLocked;
                return;
            }

            var kspDecoupler = module as ModuleDecouplerBase;
            if (kspDecoupler != null)
            {
                var decoupler = new Decoupler
                {
                    IsOmniDecoupler = kspDecoupler.isOmniDecoupler,
                    IsDecoupled = kspDecoupler.isDecoupled,
                    Staged = kspDecoupler.staged,
                    StagingEnabled = kspDecoupler.stagingEnabled
                };
                simPart.Decoupler = decoupler;

                AttachNode node = kspDecoupler.ExplosiveNode;
                pendingAttachments.Add(new KeyValuePair<Decoupler, Part>(decoupler, node != null ? node.attachedPart : null));
                return;
            }

            var kspDockingNode = module as ModuleDockingNode;
            if (kspDockingNode != null)
            {
                var decoupler = new Decoupler
                {
                    Staged = kspDockingNode.staged,
                    StagingEnabled = kspDockingNode.stagingEnabled
                };
                simPart.Decoupler = decoupler;

                // referenceNode.attachedPart only covers editor-attached ports; a DOCKED pair
                // (ship stacked on booster) links via otherNode instead
                Part attached = kspDockingNode.referenceNode != null ? kspDockingNode.referenceNode.attachedPart : null;
                if (attached == null && kspDockingNode.otherNode != null) attached = kspDockingNode.otherNode.part;
                pendingAttachments.Add(new KeyValuePair<Decoupler, Part>(decoupler, attached));
                return;
            }

            if (module.moduleName.Contains("ProceduralFairingDecoupler"))
            {
                simPart.Decoupler = new Decoupler
                {
                    DecouplesSelfFromParent = true,
                    IsDecoupled = GetBoolField(module, "decoupled"),
                    StagingEnabled = module.stagingEnabled
                };
            }
        }
        public static string DescribeSimStaging(Vessel vessel)
        {
            if (vessel == null || vessel.parts == null) return "no vessel";
            SimVessel sim = BuildVessel(vessel);   // same builder Simulate uses
            var sb = new System.Text.StringBuilder();
            foreach (SimPart p in sim.Parts)
            {
                sb.Append(" || ").Append(p.Name)
                  .Append(" inv=").Append(p.InverseStage)
                  .Append(" simDec=").Append(p.DecoupledInStage)
                  .Append(" dec?").Append(p.Decoupler != null)
                  .Append(" en=").Append(p.Decoupler != null && p.Decoupler.StagingEnabled)
                  .Append(" att=").Append(p.Decoupler != null && p.Decoupler.AttachedPart != null ? p.Decoupler.AttachedPart.Name : "-");
            }
            return sb.ToString();
        }

        private static void BuildEngine(SimPart part, ModuleEngines kspEngine)
        {
            double vacIsp = kspEngine.atmosphereCurve != null ? kspEngine.atmosphereCurve.Evaluate(0f) : 0.0;
            if (vacIsp <= 0.0) return;

            var engine = new SimEngine
            {
                Part = part,
                IsEnabled = kspEngine.isEnabled,
                IsOperational = kspEngine.isOperational,
                ThrottleLocked = kspEngine.throttleLocked,
                ThrustLimiter = Math.Min(100.0, Math.Max(0.0, kspEngine.thrustPercentage)) / 100.0,
                MinFuelFlowTons = kspEngine.minFuelFlow,
                MaxFuelFlowTons = kspEngine.maxFuelFlow,
                MinThrustKn = kspEngine.minThrust,
                MaxThrustKn = kspEngine.maxThrust,
                GravityIsp = kspEngine.g,
                MultFlow = kspEngine.multFlow,
                VacuumIsp = vacIsp * kspEngine.multIsp,
                NoPropellants = kspEngine.flameout && kspEngine.statusL2 == "No propellants",
                ModuleResiduals = GetDoubleField(kspEngine, "predictedMaximumResiduals") // RealFuels; 0 on stock
            };

            foreach (global::Propellant kspPropellant in kspEngine.propellants)
            {
                PartResourceDefinition definition = PartResourceLibrary.Instance.GetDefinition(kspPropellant.id);

                engine.Propellants.Add(new EnginePropellant
                {
                    ResourceId = kspPropellant.id,
                    Ratio = kspPropellant.ratio,
                    DensityTonsPerUnit = definition != null ? definition.density : 0.0,
                    FlowMode = ConvertFlowMode(kspPropellant.GetFlowMode()),
                    IgnoreForIsp = kspPropellant.ignoreForIsp
                });
            }

            engine.Initialize();
            part.Engines.Add(engine);
        }

        private static FlowMode ConvertFlowMode(ResourceFlowMode mode)
        {
            switch (mode)
            {
                case ResourceFlowMode.NO_FLOW: return FlowMode.NoFlow;
                case ResourceFlowMode.ALL_VESSEL: return FlowMode.AllVessel;
                case ResourceFlowMode.STAGE_PRIORITY_FLOW: return FlowMode.StagePriorityFlow;
                case ResourceFlowMode.STACK_PRIORITY_SEARCH: return FlowMode.StackPrioritySearch;
                case ResourceFlowMode.ALL_VESSEL_BALANCE: return FlowMode.AllVesselBalance;
                case ResourceFlowMode.STAGE_PRIORITY_FLOW_BALANCE: return FlowMode.StagePriorityFlowBalance;
                case ResourceFlowMode.STAGE_STACK_FLOW: return FlowMode.StageStackFlow;
                case ResourceFlowMode.STAGE_STACK_FLOW_BALANCE: return FlowMode.StageStackFlowBalance;
                default: return FlowMode.Null;
            }
        }

        private static double GetCrewMassTons(Part kspPart)
        {
            if (kspPart.protoModuleCrew == null) return 0.0;

            double mass = 0.0;
            foreach (ProtoCrewMember crew in kspPart.protoModuleCrew)
            {
                mass += PhysicsGlobals.KerbalCrewMass + crew.ResourceMass() + crew.InventoryMass();
            }

            return mass;
        }

        // Mod fields (RealFuels, ProceduralFairings) read by name so the sim has no hard mod dependency.
        private static readonly Dictionary<string, FieldInfo> _fieldCache = new Dictionary<string, FieldInfo>();

        private static FieldInfo GetCachedField(object instance, string fieldName)
        {
            Type type = instance.GetType();
            string key = type.FullName + "." + fieldName;

            FieldInfo field;
            if (!_fieldCache.TryGetValue(key, out field))
            {
                field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                _fieldCache[key] = field;
            }

            return field;
        }

        private static double GetDoubleField(object instance, string fieldName)
        {
            FieldInfo field = GetCachedField(instance, fieldName);
            if (field == null) return 0.0;

            object value = field.GetValue(instance);
            if (value is double) return (double)value;
            if (value is float) return (float)value;
            return 0.0;
        }

        private static bool GetBoolField(object instance, string fieldName)
        {
            FieldInfo field = GetCachedField(instance, fieldName);
            if (field == null) return false;

            object value = field.GetValue(instance);
            return value is bool && (bool)value;
        }
    }
}
