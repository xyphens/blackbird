using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Blackbird.Mathematics;
using Blackbird.Trajectory;
using UnityEngine;
using Blackbird.Helpers;
using Blackbird.Psg;
using Blackbird.Logging;
using System.Text;

namespace Blackbird.Models
{
    public sealed class VesselState
    {
        private VesselState() { }

        public Vessel Vessel { get; private set; }
        public CelestialBody Body { get; private set; }
        public double UniversalTime { get; private set; }

        public double LatitudeDeg { get; private set; }
        public double LongitudeDeg { get; private set; }
        public double AltitudeMeters { get; private set; }

        public Vector3d Position { get; private set; }
        public Vector3d SurfaceVelocity { get; private set; }
        public Vector3d OrbitalVelocity { get; private set; }

        public double VerticalSpeed { get; private set; }
        public double SurfaceSpeed { get; private set; }
        public double HorizontalSpeed { get; private set; }

        public double CurrentApoapsisAlt { get; private set; }
        public double CurrentPeriapsisAlt { get; private set; }
        public double TimeToApoapsisSeconds { get; private set; }
        public double TimeToPeriapsisSeconds { get; private set; }
        public double CurrentInclinationDeg { get; private set; }
        public double CurrentLanDeg { get; private set; }

        public double TotalMass { get; private set; }
        public double RemainingDeltaV { get; private set; }
        public double AvailableThrust { get; private set; }
        public ThrustEnvelope AvailableRcsThrust = new ThrustEnvelope();
        public ThrustEnvelope AvailableRcsTorque = new ThrustEnvelope();
        public double CurrentThrust { get; private set; }
        public double ThrustToWeight { get; private set; }
        public double VacuumSpecificImpulse { get; private set; }
        public double AtmosphericSpecificImpulse { get; private set; }
        public int CurrentStage { get; private set; }
        public PoweredStageInfo[] PoweredStages { get; private set; }

        public double DynamicPressureKpa { get; private set; }
        public double StaticPressureKpa { get; private set; }
        public double AtmosphericDensity { get; private set; }
        public double Mach { get; private set; }

        public double BodyRadius { get; private set; }
        public double BodyGravParameter { get; private set; }
        public double BodySurfaceGravity { get; private set; }
        public double BodyRotationPeriod { get; private set; }
        public Vector3d GravityForce { get; private set; }

        // PSG re-solves at 0.5-5 s; re-running the fuel sim per frame buys nothing.
        private const double FuelSimRefreshSeconds = 0.5;
        private const double FuelSimDivergenceTolerance = 0.01;

        private static readonly BlackbirdLog FuelSimLog = new BlackbirdLog(LogContext.Psg);
        private static Guid _fuelSimVesselId;
        private static int _fuelSimStage;
        private static double _fuelSimUt;
        private static PoweredStageInfo[] _fuelSimStages;
        private static string _fuelSimDivergenceSignature = string.Empty;

        public static VesselState FromVessel(Vessel vessel)
        {
            if (vessel == null) return null;

            CelestialBody body = vessel.mainBody;
            TrajectoryState trajectoryState = TrajectoryProvider.GetCurrentState(vessel);
            OrbitInfo orbitInfo = TrajectoryProvider.GetOrbitInfo(vessel);
            double surfaceSpeed = vessel.srfSpeed;
            double verticalSpeed = vessel.verticalSpeed;
            double totalMass = vessel.totalMass; // du: was GetDouble("totalMass") but no idea why?
            PropulsionInfo propulsion = GetPropulsionInfo(vessel, totalMass, body);

            (ThrustEnvelope rcsThrust, ThrustEnvelope rcsTorque) = GetRCS(vessel, false);

            return new VesselState
            {
                Vessel = vessel,
                Body = body,
                UniversalTime = Planetarium.GetUniversalTime(),

                LatitudeDeg = trajectoryState != null && trajectoryState.IsValid ? trajectoryState.LatitudeDeg : vessel.latitude,
                LongitudeDeg = trajectoryState != null && trajectoryState.IsValid ? trajectoryState.LongitudeDeg : vessel.longitude,
                AltitudeMeters = trajectoryState != null && trajectoryState.IsValid ? trajectoryState.AltitudeMeters : vessel.altitude,

                Position = trajectoryState != null && trajectoryState.IsValid ? trajectoryState.WorldPosition : TrajectoryProvider.GetPosition(vessel),
                SurfaceVelocity = TrajectoryProvider.GetSurfaceVelocity(vessel),
                OrbitalVelocity = trajectoryState != null && trajectoryState.IsValid ? trajectoryState.WorldVelocity : TrajectoryProvider.GetVelocity(vessel),

                VerticalSpeed = verticalSpeed,
                SurfaceSpeed = surfaceSpeed,
                HorizontalSpeed = Math.Sqrt(Math.Max(0.0, surfaceSpeed * surfaceSpeed - verticalSpeed * verticalSpeed)),

                CurrentApoapsisAlt = orbitInfo != null ? orbitInfo.ApoapsisAlt : double.NaN,
                CurrentPeriapsisAlt = orbitInfo != null ? orbitInfo.PeriapsisAlt : double.NaN,
                TimeToApoapsisSeconds = orbitInfo != null ? orbitInfo.TimeToApoapsisSeconds : double.NaN,
                TimeToPeriapsisSeconds = orbitInfo != null ? orbitInfo.TimeToPeriapsisSeconds : double.NaN,
                CurrentInclinationDeg = orbitInfo != null ? orbitInfo.InclinationDeg : double.NaN,
                CurrentLanDeg = orbitInfo != null ? orbitInfo.LanDeg : double.NaN,

                AvailableRcsThrust = rcsThrust,
                AvailableRcsTorque = rcsTorque,

                TotalMass = totalMass,
                RemainingDeltaV = GetRemainingDeltaV(vessel),
                AvailableThrust = propulsion.AvailableThrust,
                CurrentThrust = propulsion.CurrentThrust,
                ThrustToWeight = propulsion.ThrustToWeight,
                VacuumSpecificImpulse = propulsion.VacuumSpecificImpulse,
                AtmosphericSpecificImpulse = propulsion.AtmosphericSpecificImpulse,
                CurrentStage = vessel.currentStage,
                PoweredStages = GetPoweredStages(vessel),
                GravityForce = FlightGlobals.getGeeForceAtPosition(vessel.CoMD), // there's a CoM and CoMD -> CoMD maps to a Vector3d so i'm using that


                DynamicPressureKpa = vessel.dynamicPressurekPa,
                StaticPressureKpa = vessel.staticPressurekPa,
                AtmosphericDensity = vessel.atmDensity,
                Mach = vessel.mach,

                BodyRadius = body != null ? body.Radius : double.NaN,
                BodyGravParameter = body != null ? body.gravParameter : double.NaN,
                BodySurfaceGravity = GetBodySurfaceGravity(body),
                BodyRotationPeriod = body != null ? body.rotationPeriod : double.NaN
            };
        }

        private static PoweredStageInfo[] GetPoweredStages(Vessel vessel)
        {
            double now = Planetarium.GetUniversalTime();

            if (_fuelSimStages != null 
                && vessel.id == _fuelSimVesselId 
                && vessel.currentStage == _fuelSimStage 
                && now - _fuelSimUt < FuelSimRefreshSeconds) return _fuelSimStages;

            PoweredStageInfo[] stockStages = GetStockPoweredStages(vessel);
            PoweredStageInfo[] stages;

            try
            {
                stages = BuildStagesFromFuelSim(vessel, stockStages);
                if (stages.Length == 0 && stockStages.Length > 0)
                {
                    FuelSimLog.Write("[fuelsim-fallback] sim produced no powered stages; using stock VesselDeltaV");
                    stages = stockStages;
                }
            }
            catch (Exception ex) {
                FuelSimLog.Write("[fuelsim-fallback] " + ex.Message);
                stages = stockStages;
            }

            _fuelSimVesselId = vessel.id;
            _fuelSimStage = vessel.currentStage;
            _fuelSimUt = now;
            _fuelSimStages = stages;
            return stages;
        }

        private static PoweredStageInfo[] BuildStagesFromFuelSim(Vessel vessel, PoweredStageInfo[] stockStages)
        {
            FuelSim.SimStageStats[] simStats = FuelSim.StagePropellantSimulator.Simulate(vessel);

            var stages = new List<PoweredStageInfo>();
            int phaseIndex = 0;

            for (int i = 0; i < simStats.Length; i++)
            {
                FuelSim.SimStageStats s = simStats[i];

                // decouple-only stages carry no burn and are not phases
                if (s.VacuumThrustNewtons <= 0.0 || s.BurnablePropellantKg <= 0.0) continue;

                double startMassTons = s.StartMassKg / 1000.0;
                double endMassTons = s.EndMassKg / 1000.0;
                double vacuumThrustKn = s.VacuumThrustNewtons / 1000.0;
                PoweredStageInfo stock = FindStockStage(stockStages, s.Stage);

                stages.Add(new PoweredStageInfo
                {
                    IsValid = true,
                    ReasonUnavailable = null,
                    Stage = s.Stage,
                    PhaseIndex = phaseIndex,
                    IsCurrentOrFutureStage = true,

                    StartMass = startMassTons,
                    EndMass = endMassTons,
                    DryMass = endMassTons,
                    FuelMass = s.BurnablePropellantKg / 1000.0,
                    DecoupledMass = double.NaN,

                    VacuumSpecificImpulse = s.VacuumIspSeconds,
                    SeaLevelSpecificImpulse = stock != null ? stock.SeaLevelSpecificImpulse : double.NaN,
                    CurrentSpecificImpulse = stock != null && MathHelpers.IsFinite(stock.CurrentSpecificImpulse) && stock.CurrentSpecificImpulse > 0.0
                        ? stock.CurrentSpecificImpulse
                        : s.VacuumIspSeconds,

                    VacuumThrust = vacuumThrustKn,
                    SeaLevelThrust = stock != null ? stock.SeaLevelThrust : double.NaN,
                    CurrentThrust = stock != null ? stock.CurrentThrust : double.NaN,
                    MinimumThrust = s.MinimumThrottle * vacuumThrustKn,
                    MinimumThrottle = s.MinimumThrottle,

                    VacuumDeltaV = endMassTons > 0.0
                        ? s.VacuumIspSeconds * MathHelpers.StandardGravity * Math.Log(startMassTons / endMassTons)
                        : double.NaN,
                    SeaLevelDeltaV = double.NaN,
                    CurrentDeltaV = double.NaN,
                    BurnTimeSeconds = s.FullThrottleBurnTimeSeconds
                });
                phaseIndex++;
            }

            LogDivergence(stockStages, stages);
            return stages.ToArray();
        }

        private static PoweredStageInfo FindStockStage(PoweredStageInfo[] stockStages, int stage)
        {
            for (int i = 0; i < stockStages.Length; i++) {
                if (stockStages[i].Stage == stage) return stockStages[i];
            }

            return null;
        }

        private static void LogDivergence(PoweredStageInfo[] stockStages, List<PoweredStageInfo> simStages)
        {
            var sb = new StringBuilder();

            for (int i = 0; i < simStages.Count; i++) {
                PoweredStageInfo sim = simStages[i];
                PoweredStageInfo stock = FindStockStage(stockStages, sim.Stage);

                if (stock == null)
                {
                    sb.Append($"{sim.Stage}: no stock stage; ");
                    continue;
                }

                double simProp = sim.StartMass - sim.EndMass;
                double stockProp = stock.StartMass - stock.EndMass;

                if (Math.Abs(simProp - stockProp) / Math.Max(Math.Abs(simProp), 1e-9) > FuelSimDivergenceTolerance)
                {
                    sb.Append($"S{sim.Stage}: prop sim={simProp:F2}t stock={stockProp:F2}t, burnT sim={sim.BurnTimeSeconds:F1}s stock={stock.BurnTimeSeconds:F1}s; ");
                }
            }

            string signature = sb.ToString();
            if (signature.Length > 0 && signature != _fuelSimDivergenceSignature) {
                FuelSimLog.Write("[fuelsim-divergence] " + signature);
            }

            _fuelSimDivergenceSignature = signature;
        }

        // Reads KSP's stage-resolved delta-v model so powered guidance can build PSG phases.
        // fallback for our own fuel sim
        private static PoweredStageInfo[] GetStockPoweredStages(Vessel vessel)
        {
            if (vessel.VesselDeltaV == null) return new PoweredStageInfo[0];

            IEnumerable stageInfos = vessel.VesselDeltaV.OperatingStageInfo;

            if (stageInfos == null) return new PoweredStageInfo[0];

            var stages = new List<PoweredStageInfo>();
            int phaseIndex = 0;

            foreach (DeltaVStageInfo stageInfo in stageInfos)
            {
                PoweredStageInfo stage = CreatePoweredStageInfo(vessel, stageInfo, phaseIndex);
                phaseIndex++;

                if (stage != null && stage.IsValid)
                {
                    stages.Add(stage);
                }
            }

            return stages
                .OrderByDescending(stage => stage.Stage)
                .ThenBy(stage => stage.PhaseIndex)
                .ToArray();
        }

        // Converts one KSP DeltaVStageInfo object into Blackbird's solver-facing stage contract.
        private static PoweredStageInfo CreatePoweredStageInfo(Vessel vessel, DeltaVStageInfo stageInfo, int phaseIndex)
        {
            if (stageInfo == null) return null;

            //double rawKspStage = GetDouble(stageInfo, "stage", double.NaN);
            //double startMass = GetDouble(stageInfo, "startMass", double.NaN);
            //double endMass = GetDouble(stageInfo, "endMass", double.NaN);
            //double thrustVac = GetDouble(stageInfo, "thrustVac", double.NaN);
            //double thrustAsl = GetDouble(stageInfo, "thrustASL", double.NaN);
            //double thrustActual = GetDouble(stageInfo, "thrustActual", double.NaN);
            //double deltaVVac = GetDouble(stageInfo, "deltaVinVac", double.NaN);
            //double deltaVAsl = GetDouble(stageInfo, "deltaVatASL", double.NaN);
            //double deltaVActual = GetDouble(stageInfo, "deltaVActual", double.NaN);
            //double burnTime = GetDouble(stageInfo, "stageBurnTime", double.NaN);

            if (!MathHelpers.IsFinite(stageInfo.stage) ||
                !MathHelpers.IsFinite(stageInfo.startMass) ||
                !MathHelpers.IsFinite(stageInfo.endMass) ||
                stageInfo.startMass <= 0.0 ||
                stageInfo.endMass <= 0.0)
            {
                return CreateInvalidStage(-1, phaseIndex, "KSP stage mass data is unavailable.");
            }

            int stage = Convert.ToInt32(stageInfo.stage);

            double minimumThrust = GetStageMinimumThrust(stageInfo);
            double maximumThrust = MathHelpers.IsFinite(stageInfo.thrustVac) && stageInfo.thrustVac > 0.0 ? stageInfo.thrustVac : stageInfo.thrustActual;

            //double ispVac = stageInfo.ispVac;
            double burnTime = MathHelpers.IsFinite(stageInfo.ispVac) && stageInfo.ispVac > 0.0 && maximumThrust > 0.0
                                ? (stageInfo.startMass - stageInfo.endMass) * stageInfo.ispVac * MathHelpers.StandardGravity / maximumThrust
                                : double.NaN;

            double minimumThrottle = maximumThrust > 0.0 && MathHelpers.IsFinite(minimumThrust)
                ? MathHelpers.Clamp(minimumThrust / maximumThrust, 0.0, 1.0)
                : 0.0;

            bool hasPoweredCapability =
                (MathHelpers.IsFinite(stageInfo.thrustVac) && stageInfo.thrustVac > 0.0) ||
                (MathHelpers.IsFinite(stageInfo.thrustActual) && stageInfo.thrustActual > 0.0) ||
                (MathHelpers.IsFinite(stageInfo.deltaVinVac) && stageInfo.deltaVinVac > 0.0) ||
                (MathHelpers.IsFinite(stageInfo.deltaVActual) && stageInfo.deltaVActual > 0.0);

            if (!hasPoweredCapability)
            {
                return CreateInvalidStage(stage, phaseIndex, "Stage has no powered delta-v.");
            }

            return new PoweredStageInfo
            {
                IsValid = true,
                ReasonUnavailable = null,
                Stage = stage,
                PhaseIndex = phaseIndex,
                IsCurrentOrFutureStage = vessel != null && stage <= vessel.currentStage,

                StartMass = stageInfo.startMass,
                EndMass = stageInfo.endMass,
                DryMass = stageInfo.dryMass,
                FuelMass = stageInfo.fuelMass,
                DecoupledMass = stageInfo.decoupledMass,

                VacuumSpecificImpulse = stageInfo.ispVac,
                SeaLevelSpecificImpulse = stageInfo.ispASL,
                CurrentSpecificImpulse = stageInfo.ispActual,

                VacuumThrust = stageInfo.thrustVac,
                SeaLevelThrust = stageInfo.thrustASL,
                CurrentThrust = stageInfo.thrustActual,
                MinimumThrust = minimumThrust,
                MinimumThrottle = minimumThrottle,

                VacuumDeltaV = stageInfo.deltaVinVac,
                SeaLevelDeltaV = stageInfo.deltaVatASL,
                CurrentDeltaV = stageInfo.deltaVActual,
                BurnTimeSeconds = burnTime
            };
        }

        // Aggregates engine min-thrust for the stage so PSG can model throttle bounds.
        private static double GetStageMinimumThrust(DeltaVStageInfo stageInfo)
        {
            IEnumerable engines = stageInfo.enginesInStage ?? stageInfo.enginesActiveInStage;

            if (engines == null) return 0.0;

            double minimumThrust = 0.0;

            foreach (DeltaVEngineInfo engineInfo in engines)
            {
                ModuleEngines engine = engineInfo?.engine;
                if (engine == null) continue;

                double tp = MathHelpers.IsFinite(engine.thrustPercentage) ? engine.thrustPercentage : 100.0;
                double minThrust = MathHelpers.IsFinite(engine.minThrust) ? Math.Max(0.0, engine.minThrust) : 0.0;
                double thrustLimiter = MathHelpers.Clamp(tp, 0.0, 100.0) / 100.0;
                minimumThrust += minThrust * thrustLimiter;
            }

            return minimumThrust;
        }

        private static PoweredStageInfo CreateInvalidStage(int stage, int phaseIndex, string reason)
        {
            return new PoweredStageInfo
            {
                IsValid = false,
                ReasonUnavailable = reason,
                Stage = stage,
                PhaseIndex = phaseIndex
            };
        }

        private sealed class PropulsionInfo
        {
            public double AvailableThrust { get; set; }
            public double CurrentThrust { get; set; }
            public double ThrustToWeight { get; set; }
            public double VacuumSpecificImpulse { get; set; }
            public double AtmosphericSpecificImpulse { get; set; }
        }
        // Re-calculates the remaining thrust and torque for our RCS
        public static (ThrustEnvelope thrust, ThrustEnvelope torque) GetRCS(Vessel vessel, bool useBalancer)
        {
            ThrustEnvelope _thrust = new ThrustEnvelope();
            ThrustEnvelope _torque = new ThrustEnvelope();

            // RCS not enabled
            if (!vessel.ActionGroups[KSPActionGroup.RCS]) return (_thrust, _torque);

            //Guid vesselId = vessel == null ? Guid.Empty : vessel.id;
            if (useBalancer)
            {
                // todo: not implemented
            }

            Vector3d movingCoM = vessel.CurrentCoM;
            for (int i = 0; i < vessel.Parts.Count; i++)
            {
                Part part = vessel.Parts[i];
                for (int m = 0; m < part.Modules.Count; m++)
                {
                    ModuleRCS rcs = part.Modules[m] as ModuleRCS;
                    // not an RCS part
                    if (rcs == null) continue;

                    if (!part.ShieldedFromAirstream
                        && rcs.rcsEnabled
                        && rcs.isEnabled
                        && !rcs.isJustForShow
                        && !rcs.flameout && rcs.rcs_active)
                    {
                        Vector3 attitude = new Vector3(rcs.enablePitch ? 1 : 0, rcs.enableRoll ? 1 : 0, rcs.enableYaw ? 1 : 0);
                        Vector3 translation = new Vector3(rcs.enableX ? 1 : 0f, rcs.enableZ ? 1 : 0, rcs.enableY ? 1 : 0);
                        for (int j = 0; j < rcs.thrusterTransforms.Count; j++)
                        {
                            Transform t = rcs.thrusterTransforms[j];

                            // remove inactive RCS part variants
                            if (!t.gameObject.activeInHierarchy) continue;

                            Vector3d thrusterPos = t.position - movingCoM;
                            Vector3d thrustDir = rcs.useZaxis ? -t.forward : -t.up;
                            float power = rcs.thrusterPower * rcs.thrustPercentage * 0.01f;

                            if (FlightInputHandler.fetch.precisionMode)
                            {
                                if (rcs.useLever)
                                {
                                    // move the RCS thrust lever if enabled
                                    float lever = rcs.GetLeverDistance(t, thrustDir, movingCoM);
                                    if (lever > 1)
                                    {
                                        power = power / lever;
                                    }
                                }
                                else
                                {
                                    power *= rcs.precisionFactor;
                                }
                            }

                            // update thrust
                            Vector3d thrusterLevel = thrustDir * power;
                            _thrust.Add(Vector3.Scale(vessel.GetTransform().InverseTransformDirection(thrusterLevel), translation));

                            // update torque
                            Vector3d thrusterTorque = Vector3d.Cross(thrusterPos, thrusterLevel);
                            _torque.Add(Vector3d.Scale(vessel.GetTransform().InverseTransformDirection(thrusterTorque), attitude));

                        }
                    }
                }
            }

            return (_thrust, _torque);
        }

        // Measures active engine capability so guidance can scale throttle to the current vehicle.
        private static PropulsionInfo GetPropulsionInfo(Vessel vessel, double totalMass, CelestialBody body)
        {
            var info = new PropulsionInfo
            {
                AvailableThrust = 0.0,
                CurrentThrust = 0.0,
                ThrustToWeight = double.NaN,
                VacuumSpecificImpulse = double.NaN,
                AtmosphericSpecificImpulse = double.NaN
            };

            if (vessel == null || vessel.parts == null) return info;

            double weightedVacuumIsp = 0.0;
            double weightedAtmosphericIsp = 0.0;
            double ispWeight = 0.0;
            
            double pressureAtm = MathHelpers.IsFinite(vessel.staticPressurekPa) ? vessel.staticPressurekPa / 101.325 : 0.0;

            foreach (Part part in vessel.parts)
            {
                if (part == null || part.Modules == null) continue;

                foreach (PartModule module in part.Modules)
                {
                    if (!BlackbirdHelpers.IsUsableEngine(module, false)) continue;

                    ModuleEngines e = (ModuleEngines) module;

                    double maxThrust = Math.Max(0.0, e.maxThrust);
                    double thrustLimiter = MathHelpers.Clamp(e.thrustPercentage, 0.0, 100.0) / 100.0;
                    double availableThrust = maxThrust * thrustLimiter;
                    double currentThrust = Math.Max(0.0, e.finalThrust);

                    info.AvailableThrust += availableThrust;
                    info.CurrentThrust += currentThrust;

                    double vacuumIsp = GetEngineIsp(e, 0.0);
                    double atmosphericIsp = GetEngineIsp(e, pressureAtm);

                    if (availableThrust > 0.0 && MathHelpers.IsFinite(vacuumIsp))
                    {
                        weightedVacuumIsp += vacuumIsp * availableThrust;
                        ispWeight += availableThrust;
                    }

                    if (availableThrust > 0.0 && MathHelpers.IsFinite(atmosphericIsp))
                    {
                        weightedAtmosphericIsp += atmosphericIsp * availableThrust;
                    }
                }
            }

            if (ispWeight > 0.0)
            {
                info.VacuumSpecificImpulse = weightedVacuumIsp / ispWeight;
                info.AtmosphericSpecificImpulse = weightedAtmosphericIsp / ispWeight;
            }

            double surfaceGravity = GetBodySurfaceGravity(body);
            if (totalMass > 0.0 && MathHelpers.IsFinite(surfaceGravity) && surfaceGravity > 0.0)
            {
                info.ThrustToWeight = info.AvailableThrust / (totalMass * surfaceGravity);
            }

            return info;
        }

        private static double GetEngineIsp(ModuleEngines module, double pressureAtm)
        {
            if (module.atmosphereCurve == null) return double.NaN;
            return module.atmosphereCurve.Evaluate((float)Math.Max(0.0, pressureAtm));
        }

        // Stock ΔV system (vessel.VesselDeltaV). TotalDeltaVVac is the stable, situation-independent ship
        // figure (use TotalDeltaVActual for the current-atmosphere value). Returns NaN until the stock calc is
        // ready, nudging it dirty so it computes within a frame or two; once ready it stays ready until mass /
        // staging changes, so this doesn't thrash. Replaces vessel.GetDeltaV() which read 0.
        private static double GetRemainingDeltaV(Vessel vessel)
        {
            try
            {
                VesselDeltaV dv = vessel?.VesselDeltaV;
                if (dv == null) return double.NaN;
                if (!dv.IsReady)
                {
                    dv.SetCalcsDirty(false, false);
                    return double.NaN;
                }
                double total = dv.TotalDeltaVVac;
                return total > 0.0 ? total : double.NaN;
            }
            catch
            {
                return double.NaN;
            }
        }

        private static double GetBodySurfaceGravity(CelestialBody body)
        {
            if (body == null || body.Radius <= 0.0) return double.NaN;

            return body.gravParameter / (body.Radius * body.Radius);
        }

        //private static double GetDouble(object instance, string memberName, double fallback)
        //{
        //    return ConvertToDouble(GetMember(instance, memberName), fallback);
        //}

        private static object GetMember(object instance, string memberName)
        {
            if (instance == null) return null;

            Type type = instance.GetType();
            var property = type.GetProperty(memberName);
            if (property != null)
            {
                return property.GetValue(instance, null);
            }

            var field = type.GetField(memberName);
            return field != null ? field.GetValue(instance) : null;
        }

        private static double ConvertToDouble(object value, double fallback)
        {
            if (value == null) return fallback;

            try
            {
                return Convert.ToDouble(value);
            }
            catch
            {
                return fallback;
            }
        }
    }
}
