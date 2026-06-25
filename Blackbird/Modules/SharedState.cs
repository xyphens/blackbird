using Blackbird.Guidance;
using Blackbird.Models;
using Blackbird.Modules;
using Blackbird.Rendezvous;
using System;
using System.Collections.Generic;

namespace Blackbird.Modules
{
    public enum BlackbirdModule { 
        None, 
        Planner, 
        Rendezvous,
        LaunchGuidance,
        Docking
    }

    // environment
    public enum PlanetScaleEnum { Stock, RSS }
    public enum LaunchGuidanceState { Idle, PlanReady, PlanAccepted, WarpingToLaunch, AwaitingLaunch, GuidingAscent, Complete, Aborted };
    public enum GuidanceMode { None, Manual, Autopilot };
    public enum PoweredGuidancePhase { None, VerticalAscent, PitchProgram, Coast, Circularize, PoweredGuidance, Terminal, Complete, InsertionCutoff, Unavailable };

    // RENDEZVOUS
    // execution methods within Rendezvous module
    // important: was RendezvousStage 
    public enum RendezvousMethod { None, Intercept, MatchVelocity, CloseApproach }; // note: had Docking before, now is separate
    public enum InterceptMethod { SinglePhase, Hohmann, Phasing };
    // Result of an intercept plan: the impulsive burn (world-frame ΔV at the ignition UT) that puts the
    // active vessel onto a conic transfer reaching the target's predicted position, plus the arrival
    // timing and a predicted closest approach. Planning only — execution happens in the executor.
    public struct InterceptSolution
    {
        public bool Success;
        public InterceptStatus Status;

        public Vector3d DeltaV;                       // world-frame burn applied at IgnitionUt
        public double DeltaVMagnitude;                // |DeltaV| (m/s)
        public double IgnitionUt;                     // when the burn is applied (= "now")
        public double ArrivalUt;                      // when the transfer reaches the target point
        public double TimeOfFlight;                   // ArrivalUt - IgnitionUt (s)
        public double PredictedClosestApproach;       // min transfer-to-target separation over the arc (m)

        public Vector3d TransferDepartureVelocity;    // Lambert V1 (post-burn velocity at ignition)
        public Vector3d TransferArrivalVelocity;      // Lambert V2 (velocity at arrival, used by match velocity)
        public int SamplesEvaluated;                  // Lambert solves actually attempted
    }

    // status
    // important: was RendezvousPhase
    public enum InterceptPhase { Idle, Executing, Coast, Complete, Aborted };
    // DOCKING
    public enum DockingMethod { Automatic, Manual };
    public enum DockingControlMode { Off, Manual, Guidance }
    public static class Universe
    {
        // deprecated - use SharedState.IsRSS
        //public static PlanetScaleEnum PlanetScale => FlightGlobals.currentMainBody.Radius > 1_000_000 ? PlanetScaleEnum.RSS : PlanetScaleEnum.Stock;
        // returns true if the provided altitude is at or above the current planet/moon's atmosphere
        public static bool IsInSpace(double altitude) => altitude >= FlightGlobals.currentMainBody.atmosphereDepth;
    }
    
    public sealed class SharedState
    {
        // general
        public BlackbirdModule ActiveModule { get; set; }

        public bool PlannerVisible = false;
        public bool RendezvousVisible = false;
        public bool GuidanceVisible = false;
        public bool DockingVisible = false;
        // planner
        public bool PlannerEnabled { get; set; } = false;
        public bool IsRO { get; set; } = false;
        public bool IsRSS { get; set; } = false;
        public bool IsPrincipia { get; set; } = false;
        public void Init()
        {
            CheckAssembliesLoaded();
        }

        public bool CheckAssembliesLoaded()
        {
            foreach (AssemblyLoader.LoadedAssembly asm in AssemblyLoader.loadedAssemblies)
            {
                try
                {
                    if (asm.assembly.GetName().Name.ToLower() == "principia.ksp_plugin_adapter") IsPrincipia = true;
                    if (asm.assembly.GetName().Name.ToLower() == "realismoverhaul") IsRO = true;
                    if (asm.assembly.GetName().Name.ToLower() == "realsolarsystem") IsRSS = true; // i think this is the right lookup?
                }
                catch (InvalidOperationException) { }
            }

            return false;
        }

        // guidance
        public bool LockRollOnAscent { get; set; } = false;
        public LaunchGuidanceState GuidanceState { get; set; } = LaunchGuidanceState.Idle;
        public GuidanceMode GuidanceMode { get; set; } = GuidanceMode.None;
        public PoweredGuidancePhase LaunchPhase { get; set; } = PoweredGuidancePhase.None;
        public bool GuidanceEnabled { get; set; } = false;
        public LaunchCandidate SelectedLaunchCandidate { get; set; } // activate launch candidate
        public LaunchPlan LaunchPlan { get; set; } // latest launch plan (contains array of candidates)

        // rendezvous
        public RendezvousMethod RendezvousMethod { get; set; } = RendezvousMethod.None;
        public bool RendezvousEnabled { get; set; } = false;
        public InterceptPhase InterceptPhase { get; set; } = InterceptPhase.Idle;
        public string[] InterceptMethods = { "Single Phase", "Hohmann", "Phasing" };
        public InterceptSolution InterceptSolution { get; set; }
        // Eligible Hohmann transfer windows for the user to choose from; selection copies one into InterceptSolution.
        public List<InterceptSolution> InterceptCandidates { get; set; } = new List<InterceptSolution>();
        public int SelectedInterceptCandidateIndex { get; set; } = -1;

        public InterceptMethod InterceptMethod = InterceptMethod.Hohmann;
        // used by index-based dropdown
        public int _interceptMethod
        {
            get => (int)InterceptMethod;
            set { if (Enum.IsDefined(typeof(InterceptMethod), value)) InterceptMethod = (InterceptMethod)value; }

        }
        // docking
        public DockingMethod DockingMethod { get; set; } = DockingMethod.Automatic;
        public DockingControlMode DockingMode { get; set; } = DockingControlMode.Off;
        public bool DockingEnabled { get; set; } = false;

        public void Reset()
        {
            LockRollOnAscent = false;
            LaunchPlan = null;
            SelectedLaunchCandidate = null;
            PlannerEnabled = false;
            ActiveModule = BlackbirdModule.None;
            GuidanceState = LaunchGuidanceState.Idle;
            GuidanceMode = GuidanceMode.None;
            LaunchPhase = PoweredGuidancePhase.None;
            GuidanceEnabled = false;
            RendezvousMethod = RendezvousMethod.None;
            InterceptPhase = InterceptPhase.Idle;
            InterceptMethod = InterceptMethod.Hohmann;
            RendezvousEnabled = false;
            DockingMethod = DockingMethod.Automatic;
            DockingMode = DockingControlMode.Off;
            DockingEnabled = false;
            InterceptSolution = new InterceptSolution();
            InterceptCandidates = new List<InterceptSolution>();
            SelectedInterceptCandidateIndex = -1;
        }
    }
}
