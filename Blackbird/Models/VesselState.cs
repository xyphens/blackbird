using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Blackbird.Enums;
using Blackbird.Mathematics;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Models
{
    public sealed class VesselState
    {
        private VesselState()
        {
        }

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

        public PlanetScale.PlanetScaleEnum Scale { get; private set; }

        public static VesselState FromVessel(Vessel vessel)
        {
            if (vessel == null) return null;

            CelestialBody body = vessel.mainBody;
            TrajectoryState trajectoryState = TrajectoryProvider.GetCurrentState(vessel);
            OrbitInfo orbitInfo = TrajectoryProvider.GetOrbitInfo(vessel);
            double surfaceSpeed = vessel.srfSpeed;
            double verticalSpeed = vessel.verticalSpeed;
            double totalMass = GetDouble(vessel, "totalMass", double.NaN);
            PropulsionInfo propulsion = GetPropulsionInfo(vessel, totalMass, body);

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

                TotalMass = totalMass,
                RemainingDeltaV = GetRemainingDeltaV(vessel),
                AvailableThrust = propulsion.AvailableThrust,
                CurrentThrust = propulsion.CurrentThrust,
                ThrustToWeight = propulsion.ThrustToWeight,
                VacuumSpecificImpulse = propulsion.VacuumSpecificImpulse,
                AtmosphericSpecificImpulse = propulsion.AtmosphericSpecificImpulse,
                CurrentStage = vessel.currentStage,
                PoweredStages = GetPoweredStages(vessel),

                DynamicPressureKpa = GetDouble(vessel, "dynamicPressurekPa", double.NaN),
                StaticPressureKpa = GetDouble(vessel, "staticPressurekPa", double.NaN),
                AtmosphericDensity = GetDouble(vessel, "atmDensity", double.NaN),
                Mach = GetDouble(vessel, "mach", double.NaN),

                BodyRadius = body != null ? body.Radius : double.NaN,
                BodyGravParameter = body != null ? body.gravParameter : double.NaN,
                BodySurfaceGravity = GetBodySurfaceGravity(body),
                BodyRotationPeriod = body != null ? body.rotationPeriod : double.NaN,

                Scale = body != null && body.Radius > 1000000.0
                    ? PlanetScale.PlanetScaleEnum.RSS
                    : PlanetScale.PlanetScaleEnum.Stock
            };
        }

        // Reads KSP's stage-resolved delta-v model so powered guidance can build PSG phases.
        private static PoweredStageInfo[] GetPoweredStages(Vessel vessel)
        {
            object vesselDeltaV = GetMember(vessel, "VesselDeltaV");
            if (vesselDeltaV == null) return new PoweredStageInfo[0];

            IEnumerable stageInfos =
                GetMember(vesselDeltaV, "OperatingStageInfo") as IEnumerable ??
                GetMember(vesselDeltaV, "WorkingStageInfo") as IEnumerable;

            if (stageInfos == null) return new PoweredStageInfo[0];

            var stages = new List<PoweredStageInfo>();
            int phaseIndex = 0;

            foreach (object stageInfo in stageInfos)
            {
                PoweredStageInfo stage = CreatePoweredStageInfo(vessel, stageInfo, phaseIndex);
                phaseIndex++;

                if (stage != null && stage.IsValid)
                {
                    stages.Add(stage);
                }
            }

            return stages
                .OrderByDescending(stage => stage.KspStage)
                .ThenBy(stage => stage.PhaseIndex)
                .ToArray();
        }

        // Converts one KSP DeltaVStageInfo object into Blackbird's solver-facing stage contract.
        private static PoweredStageInfo CreatePoweredStageInfo(Vessel vessel, object stageInfo, int phaseIndex)
        {
            if (stageInfo == null) return null;

            double rawKspStage = GetDouble(stageInfo, "stage", double.NaN);
            double startMass = GetDouble(stageInfo, "startMass", double.NaN);
            double endMass = GetDouble(stageInfo, "endMass", double.NaN);
            double thrustVac = GetDouble(stageInfo, "thrustVac", double.NaN);
            double thrustAsl = GetDouble(stageInfo, "thrustASL", double.NaN);
            double thrustActual = GetDouble(stageInfo, "thrustActual", double.NaN);
            double deltaVVac = GetDouble(stageInfo, "deltaVinVac", double.NaN);
            double deltaVAsl = GetDouble(stageInfo, "deltaVatASL", double.NaN);
            double deltaVActual = GetDouble(stageInfo, "deltaVActual", double.NaN);
            double burnTime = GetDouble(stageInfo, "stageBurnTime", double.NaN);

            if (!OrbitMath.IsFinite(rawKspStage) ||
                !OrbitMath.IsFinite(startMass) ||
                !OrbitMath.IsFinite(endMass) ||
                startMass <= 0.0 ||
                endMass <= 0.0)
            {
                return CreateInvalidStage(-1, phaseIndex, "KSP stage mass data is unavailable.");
            }

            int kspStage = Convert.ToInt32(rawKspStage);

            double minimumThrust = GetStageMinimumThrust(stageInfo);
            double maximumThrust = OrbitMath.IsFinite(thrustVac) && thrustVac > 0.0 ? thrustVac : thrustActual;
            double minimumThrottle = maximumThrust > 0.0 && OrbitMath.IsFinite(minimumThrust)
                ? OrbitMath.Clamp(minimumThrust / maximumThrust, 0.0, 1.0)
                : 0.0;

            bool hasPoweredCapability =
                (OrbitMath.IsFinite(thrustVac) && thrustVac > 0.0) ||
                (OrbitMath.IsFinite(thrustActual) && thrustActual > 0.0) ||
                (OrbitMath.IsFinite(deltaVVac) && deltaVVac > 0.0) ||
                (OrbitMath.IsFinite(deltaVActual) && deltaVActual > 0.0);

            if (!hasPoweredCapability)
            {
                return CreateInvalidStage(kspStage, phaseIndex, "Stage has no powered delta-v.");
            }

            return new PoweredStageInfo
            {
                IsValid = true,
                ReasonUnavailable = null,
                KspStage = kspStage,
                PhaseIndex = phaseIndex,
                IsCurrentOrFutureStage = vessel != null && kspStage <= vessel.currentStage,

                StartMass = startMass,
                EndMass = endMass,
                DryMass = GetDouble(stageInfo, "dryMass", double.NaN),
                FuelMass = GetDouble(stageInfo, "fuelMass", double.NaN),
                DecoupledMass = GetDouble(stageInfo, "decoupledMass", double.NaN),

                VacuumSpecificImpulse = GetDouble(stageInfo, "ispVac", double.NaN),
                SeaLevelSpecificImpulse = GetDouble(stageInfo, "ispASL", double.NaN),
                CurrentSpecificImpulse = GetDouble(stageInfo, "ispActual", double.NaN),

                VacuumThrust = thrustVac,
                SeaLevelThrust = thrustAsl,
                CurrentThrust = thrustActual,
                MinimumThrust = minimumThrust,
                MinimumThrottle = minimumThrottle,

                VacuumDeltaV = deltaVVac,
                SeaLevelDeltaV = deltaVAsl,
                CurrentDeltaV = deltaVActual,
                BurnTimeSeconds = burnTime
            };
        }

        // Aggregates engine min-thrust for the stage so PSG can model throttle bounds.
        private static double GetStageMinimumThrust(object stageInfo)
        {
            IEnumerable engines =
                GetMember(stageInfo, "enginesInStage") as IEnumerable ??
                GetMember(stageInfo, "enginesActiveInStage") as IEnumerable;

            if (engines == null) return 0.0;

            double minimumThrust = 0.0;

            foreach (object engineInfo in engines)
            {
                object engine = GetMember(engineInfo, "engine");
                if (engine == null) continue;

                double minThrust = Math.Max(0.0, GetDouble(engine, "minThrust", 0.0));
                double thrustLimiter = OrbitMath.Clamp(GetDouble(engine, "thrustPercentage", 100.0), 0.0, 100.0) / 100.0;
                minimumThrust += minThrust * thrustLimiter;
            }

            return minimumThrust;
        }

        private static PoweredStageInfo CreateInvalidStage(int kspStage, int phaseIndex, string reason)
        {
            return new PoweredStageInfo
            {
                IsValid = false,
                ReasonUnavailable = reason,
                KspStage = kspStage,
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
            double pressureAtm = GetDouble(vessel, "staticPressurekPa", 0.0) / 101.325;

            foreach (Part part in vessel.parts)
            {
                if (part == null || part.Modules == null) continue;

                foreach (PartModule module in part.Modules)
                {
                    if (module == null || !IsEngineModule(module) || !IsUsableEngine(module)) continue;

                    double maxThrust = Math.Max(0.0, GetDouble(module, "maxThrust", 0.0));
                    double thrustLimiter = OrbitMath.Clamp(GetDouble(module, "thrustPercentage", 100.0), 0.0, 100.0) / 100.0;
                    double availableThrust = maxThrust * thrustLimiter;
                    double currentThrust = Math.Max(0.0, GetDouble(module, "finalThrust", 0.0));

                    info.AvailableThrust += availableThrust;
                    info.CurrentThrust += currentThrust;

                    double vacuumIsp = GetEngineIsp(module, 0.0);
                    double atmosphericIsp = GetEngineIsp(module, pressureAtm);

                    if (availableThrust > 0.0 && OrbitMath.IsFinite(vacuumIsp))
                    {
                        weightedVacuumIsp += vacuumIsp * availableThrust;
                        ispWeight += availableThrust;
                    }

                    if (availableThrust > 0.0 && OrbitMath.IsFinite(atmosphericIsp))
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
            if (totalMass > 0.0 && OrbitMath.IsFinite(surfaceGravity) && surfaceGravity > 0.0)
            {
                info.ThrustToWeight = info.AvailableThrust / (totalMass * surfaceGravity);
            }

            return info;
        }

        private static bool IsEngineModule(PartModule module)
        {
            Type type = module.GetType();
            return type.Name == "ModuleEngines" || type.Name == "ModuleEnginesFX";
        }

        private static bool IsUsableEngine(PartModule module)
        {
            if (!module.isEnabled) return false;
            if (!GetBool(module, "EngineIgnited", true)) return false;
            if (GetBool(module, "flameout", false)) return false;
            if (InvokeBool(module, "getFlameoutState", false)) return false;

            return true;
        }

        private static double GetEngineIsp(PartModule module, double pressureAtm)
        {
            object curve = GetMember(module, "atmosphereCurve");
            if (curve == null) return double.NaN;

            var evaluate = curve.GetType().GetMethod("Evaluate", new[] { typeof(float) });
            if (evaluate == null) return double.NaN;

            try
            {
                object value = evaluate.Invoke(curve, new object[] { (float)Math.Max(0.0, pressureAtm) });
                return ConvertToDouble(value, double.NaN);
            }
            catch
            {
                return double.NaN;
            }
        }

        private static double GetRemainingDeltaV(Vessel vessel)
        {
            try
            {
                return vessel.GetDeltaV();
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

        private static double GetDouble(object instance, string memberName, double fallback)
        {
            return ConvertToDouble(GetMember(instance, memberName), fallback);
        }

        private static bool GetBool(object instance, string memberName, bool fallback)
        {
            object value = GetMember(instance, memberName);
            if (value == null) return fallback;

            try
            {
                return Convert.ToBoolean(value);
            }
            catch
            {
                return fallback;
            }
        }

        private static bool InvokeBool(object instance, string methodName, bool fallback)
        {
            if (instance == null) return fallback;

            var method = instance.GetType().GetMethod(methodName, Type.EmptyTypes);
            if (method == null) return fallback;

            try
            {
                return Convert.ToBoolean(method.Invoke(instance, null));
            }
            catch
            {
                return fallback;
            }
        }

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
