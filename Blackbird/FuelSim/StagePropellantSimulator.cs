using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Blackbird.FuelSim
{
    public struct SimStageStats
    {
        public int KspStage;
        public double StartMassKg;
        public double EndMassKg;            // at flameout of reachable, compatible propellant
        public double BurnablePropellantKg; // Start − End, the honest number
        public double VacuumThrustNewtons;  // sum of active engines
        public double VacuumIspSeconds;     // mass-flow-weighted
        public double MinimumThrottle;
        public double FullThrottleBurnTimeSeconds; // BurnableProp / vacuum mass flow
    }

    public static class StagePropellantSimulator
    {
        // dufixme: implement
        //public static SimStageStats[] Simulate(Vessel vessel)
        //{
        //    return new SimStageStats();
        //}
    }
}
