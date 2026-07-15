using System.Collections.Generic;

namespace Blackbird.FuelSim
{
    // Simulated vessel: the part graph plus per-stage bookkeeping of what is attached and what is burning.
    public sealed class SimVessel
    {
        public readonly List<SimPart> Parts = new List<SimPart>();
        public readonly List<SimEngine> ActiveEngines = new List<SimEngine>();

        private readonly Dictionary<int, List<SimPart>> _partsRemainingInStage = new Dictionary<int, List<SimPart>>();
        private readonly Dictionary<int, List<SimEngine>> _enginesDroppedInStage = new Dictionary<int, List<SimEngine>>();
        private readonly Dictionary<int, List<SimEngine>> _enginesActivatedInStage = new Dictionary<int, List<SimEngine>>();

        public int CurrentStage { get; private set; }
        public double MassTons { get; private set; }
        public double ThrustKn { get; private set; }
        public double MinThrottleThrustKn { get; private set; }
        public double MassFlowTons { get; private set; }

        public void SetCurrentStage(int stage) => CurrentStage = stage;
        public List<SimPart> PartsRemainingInStage(int stage) => GetList(_partsRemainingInStage, stage);
        public List<SimEngine> EnginesDroppedInStage(int stage) => GetList(_enginesDroppedInStage, stage);
        public List<SimEngine> EnginesActivatedInStage(int stage) => GetList(_enginesActivatedInStage, stage);

        private static List<T> GetList<T>(Dictionary<int, List<T>> buckets, int stage)
        {
            List<T> list;
            if (!buckets.TryGetValue(stage, out list))
            {
                list = new List<T>();
                buckets.Add(stage, list);
            }

            return list;
        }

        // Runs the decoupling analysis and buckets engines by activation/decouple stage.
        // Call once after the part graph (links, decouplers, engines) is fully loaded.
        public void FinalizeBuild()
        {
            AnalyzeDecoupling();

            foreach (SimPart part in Parts)
            {
                foreach (SimEngine engine in part.Engines)
                {
                    EnginesDroppedInStage(part.DecoupledInStage).Add(engine);
                    EnginesActivatedInStage(part.InverseStage).Add(engine);
                }
            }
        }

        public void UpdateMass()
        {
            double mass = 0.0;

            List<SimPart> parts = PartsRemainingInStage(CurrentStage);
            for (int i = 0; i < parts.Count; i++)
            {
                parts[i].UpdateMass();
                mass += parts[i].MassTons;
            }

            MassTons = mass;
        }

        public void Stage()
        {
            if (CurrentStage < 0) return;

            CurrentStage--;
            ActivateEngines();
            UpdateMass();
        }

        public void ActivateEngines()
        {
            List<SimEngine> engines = EnginesActivatedInStage(CurrentStage);
            for (int i = 0; i < engines.Count; i++)
            {
                if (engines[i].IsEnabled) engines[i].IsOperational = true;
            }
        }

        // Refreshes the burning-engine set (flameouts included) and the vessel thrust/flow aggregates.
        public void UpdateActiveEngines()
        {
            ActiveEngines.Clear();

            for (int stage = -1; stage < CurrentStage; stage++)
            {
                List<SimEngine> engines = EnginesDroppedInStage(stage);
                for (int i = 0; i < engines.Count; i++)
                {
                    SimEngine e = engines[i];
                    // fixme: there are almost an infinite number of reasons we may want to exclude an engine:
                    // i.e., user has a real solid rocket motor on some ultra-massive rocket that's being used as separator
                    if (e.Part.IsSepratron) continue;
                    if (e.MassFlowTons <= 0.0) continue;

                    e.UpdateEngineStatus();
                    if (!e.IsOperational) continue;

                    ActiveEngines.Add(e);
                }
            }

            double thrust = 0.0;
            double minThrust = 0.0;
            double massFlow = 0.0;

            for (int i = 0; i < ActiveEngines.Count; i++)
            {
                thrust += ActiveEngines[i].ThrustKn;
                minThrust += ActiveEngines[i].MinThrottleThrustKn;
                massFlow += ActiveEngines[i].MassFlowTons;
            }

            ThrustKn = thrust;
            MinThrottleThrustKn = minThrust;
            MassFlowTons = massFlow;
        }

        // Walks the tree from the root assigning each part the stage in which it leaves the vessel,
        // and fills PartsRemainingInStage for every stage the part is still attached.
        private void AnalyzeDecoupling()
        {
            SimPart rootPart = null;

            for (int i = 0; i < Parts.Count; i++)
            {
                Parts[i].DecoupledInStage = int.MinValue;
                if (rootPart == null && Parts[i].IsRoot) rootPart = Parts[i];
            }

            if (rootPart == null && Parts.Count > 0) rootPart = Parts[0];

            if (rootPart == null) return;

            CalculateDecoupledInStageRecursively(rootPart, null, -1);
        }

        private void CalculateDecoupledInStageRecursively(SimPart p, SimPart parent, int inheritedDecoupledInStage)
        {
            int childDecoupledInStage = CalculateDecoupledInStage(p, parent, inheritedDecoupledInStage);

            for (int i = 0; i < p.Links.Count; i++)
            {
                if (p.Links[i] == parent) continue;

                CalculateDecoupledInStageRecursively(p.Links[i], p, childDecoupledInStage);
            }
        }

        private int CalculateDecoupledInStage(SimPart p, SimPart parent, int parentDecoupledInStage)
        {
            // already visited (a decoupler's attached side is traversed ahead of the normal walk)
            if (p.DecoupledInStage != int.MinValue) return p.DecoupledInStage;

            // separators already fired before this snapshot cannot fire again
            if (p.InverseStage >= parentDecoupledInStage)
            {
                if (p.IsLaunchClamp)
                {
                    p.DecoupledInStage = p.InverseStage > parentDecoupledInStage ? p.InverseStage : parentDecoupledInStage;
                    TrackPartDecoupledInStage(p, p.DecoupledInStage);
                    return p.DecoupledInStage;
                }

                Decoupler d = p.Decoupler;
                if (d != null && d.StagingEnabled && p.StagingOn && !d.IsDecoupled)
                {
                    if (d.DecouplesSelfFromParent || d.IsOmniDecoupler)
                    {
                        // this part and its subtree leave the vessel when the separator fires
                        p.DecoupledInStage = p.InverseStage;
                        TrackPartDecoupledInStage(p, p.DecoupledInStage);
                        return p.DecoupledInStage;
                    }

                    if (d.AttachedPart != null)
                    {
                        if (d.AttachedPart == parent && d.Staged)
                        {
                            // the separator points back at our traversal parent: we are the dropped side
                            p.DecoupledInStage = p.InverseStage;
                            TrackPartDecoupledInStage(p, p.DecoupledInStage);
                            return p.DecoupledInStage;
                        }

                        // we stay with the parent; the attached side is dropped when the separator fires
                        p.DecoupledInStage = parentDecoupledInStage;
                        TrackPartDecoupledInStage(p, p.DecoupledInStage);
                        CalculateDecoupledInStageRecursively(d.AttachedPart, p, p.InverseStage);
                        return p.DecoupledInStage;
                    }
                }
            }

            p.DecoupledInStage = parentDecoupledInStage;
            TrackPartDecoupledInStage(p, p.DecoupledInStage);
            return p.DecoupledInStage;
        }

        private void TrackPartDecoupledInStage(SimPart part, int stage)
        {
            // a part staged above the recorded current stage means the snapshot's stage count is low
            if (stage + 1 > CurrentStage) SetCurrentStage(stage + 1);

            for (int i = stage + 1; i <= CurrentStage; i++)
            {
                PartsRemainingInStage(i).Add(part);
            }
        }
    }
}
