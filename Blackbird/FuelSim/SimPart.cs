using System;
using System.Collections.Generic;

namespace Blackbird.FuelSim
{
    // Snapshot of one resource inside a simulated part. Amounts are KSP units, density is tons/unit.
    public struct Resource
    {
        public int Id;
        public double Amount;
        public double MaxAmount;
        public double Density;
        public bool Free;        // zero-density resources (EC, intake air) are never drained or weighed
        public double Residual;  // fraction of MaxAmount unusable by the draining engine (RealFuels)

        public double ResidualThreshold
        {
            get { return Residual * MaxAmount; }
        }

        public double MassTons
        {
            get { return Free ? 0.0 : Amount * Density; }
        }
    }

    // Staging separator carried by a part: decoupler, docking port or fairing base.
    // A fairing-style separator always drops its own subtree; the others drop the attached side.
    public sealed class Decoupler
    {
        public bool IsOmniDecoupler;
        public bool IsDecoupled;
        public bool Staged;
        public bool StagingEnabled;
        public bool DecouplesSelfFromParent;
        public SimPart AttachedPart;
    }

    public sealed class SimPart
    {
        public SimVessel Vessel;
        public string Name;

        public readonly List<SimEngine> Engines = new List<SimEngine>();
        public readonly List<SimPart> Links = new List<SimPart>();
        public readonly List<SimPart> CrossFeedPartSet = new List<SimPart>();
        public readonly Dictionary<int, Resource> Resources = new Dictionary<int, Resource>();

        private readonly Dictionary<int, double> _resourceDrains = new Dictionary<int, double>();
        private readonly List<int> _scratchKeys = new List<int>();

        public Decoupler Decoupler;
        public bool IsLaunchClamp;
        public bool IsRoot;
        public bool StagingOn;
        public bool ActivatesEvenIfDisconnected;
        public bool IsThrottleLocked;
        public int InverseStage;
        public int DecoupledInStage;
        public int ResourcePriority;
        public double ResourceRequestRemainingThreshold;

        public double DryMassTons;               // in flight KSP folds module masses into part.mass; excludes resources
        public double CrewMassTons;
        public double DisabledResourcesMassTons; // flow-locked tanks are dead weight that staging cannot enable
        public double MassTons;

        public bool IsEngine => Engines.Count > 0;

        // Sepratrons burn while being dropped, so they never block staging.
        public bool IsSepratron => IsEngine && IsThrottleLocked && ActivatesEvenIfDisconnected && InverseStage == DecoupledInStage;
        public bool TryGetResource(int resourceId, out Resource resource) => Resources.TryGetValue(resourceId, out resource);
        public void ClearResourceDrains() => _resourceDrains.Clear();
        public bool HasResourceDrains => _resourceDrains.Count > 0;

        public SimPart(SimVessel vessel, string name)
        {
            Vessel = vessel;
            Name = name;
            DecoupledInStage = int.MinValue;
        }

        public void UpdateMass()
        {
            if (IsLaunchClamp)
            {
                MassTons = 0.0;
                return;
            }

            double mass = DryMassTons + CrewMassTons + DisabledResourcesMassTons;
            foreach (Resource resource in Resources.Values)
            {
                mass += resource.MassTons;
            }

            MassTons = mass;
        }

        public void AddResourceDrain(int resourceId, double unitsPerSecond)
        {
            double existing;
            if (_resourceDrains.TryGetValue(resourceId, out existing))
            {
                _resourceDrains[resourceId] = existing + unitsPerSecond;
            }
            else
            {
                _resourceDrains.Add(resourceId, unitsPerSecond);
            }
        }

        public void ApplyResourceDrains(double dt)
        {
            foreach (KeyValuePair<int, double> drain in _resourceDrains)
            {
                Resource resource;
                if (!Resources.TryGetValue(drain.Key, out resource)) continue;

                resource.Amount = Math.Max(0.0, resource.Amount - drain.Value * dt);
                Resources[drain.Key] = resource;
            }
        }

        public void UpdateResourceResidual(double residual, int resourceId)
        {
            Resource resource;
            if (!Resources.TryGetValue(resourceId, out resource)) return;

            resource.Residual = Math.Max(resource.Residual, residual);
            Resources[resourceId] = resource;
        }

        public void ClearResiduals()
        {
            _scratchKeys.Clear();
            foreach (int id in Resources.Keys)
            {
                _scratchKeys.Add(id);
            }

            foreach (int id in _scratchKeys)
            {
                Resource resource = Resources[id];
                resource.Residual = 0.0;
                Resources[id] = resource;
            }
        }

        // Shortest time until any drained resource in this part hits its unusable floor.
        public double ResourceMaxTime()
        {
            double maxTime = double.MaxValue;

            foreach (Resource resource in Resources.Values)
            {
                if (resource.Free) continue;
                if (resource.Amount <= ResourceRequestRemainingThreshold) continue;

                double drain;
                if (!_resourceDrains.TryGetValue(resource.Id, out drain) || drain <= 0.0) continue;

                maxTime = Math.Min(maxTime, (resource.Amount - resource.ResidualThreshold) / drain);
            }

            return maxTime;
        }
    }
}
