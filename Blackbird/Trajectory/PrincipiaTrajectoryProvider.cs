using System;
using System.Reflection;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Trajectory
{
    public sealed class PrincipiaTrajectoryProvider : ITrajectoryProvider
    {
        private const string AdapterTypeName = "principia.ksp_plugin_adapter.PrincipiaPluginAdapter, ksp_plugin_adapter";
        private const string InterfaceTypeName = "principia.ksp_plugin_adapter.Interface, ksp_plugin_adapter";

        private readonly StockTrajectoryProvider _stockFallback = new StockTrajectoryProvider();

        private Type _adapterType;
        private Type _interfaceType;
        private MethodInfo _pluginRunningMethod;
        private MethodInfo _pluginMethod;
        private MethodInfo _hasVesselMethod;
        private MethodInfo _vesselFromParentMethod;
        private MethodInfo _vesselVelocityMethod;

        public string SourceName { get { return "Principia"; } }

        public bool IsAvailable
        {
            get
            {
                object adapter = FindAdapter();
                return adapter != null && IsPluginRunning(adapter) && GetPlugin(adapter) != IntPtr.Zero;
            }
        }

        // Captures the current Principia-managed state when available, otherwise returns stock state.
        public TrajectoryState GetCurrentState(Vessel vessel)
        {
            if (vessel == null || vessel.mainBody == null) return _stockFallback.GetCurrentState(vessel);

            object adapter = FindAdapter();
            if (adapter == null || !IsPluginRunning(adapter)) return _stockFallback.GetCurrentState(vessel);

            IntPtr plugin = GetPlugin(adapter);
            string vesselGuid = GetVesselGuid(vessel);

            if (plugin == IntPtr.Zero || string.IsNullOrEmpty(vesselGuid) || !HasVessel(plugin, vesselGuid))
            {
                return _stockFallback.GetCurrentState(vessel);
            }

            try
            {
                object qp = _vesselFromParentMethod.Invoke(
                    null,
                    new object[] { plugin, vessel.mainBody.flightGlobalsIndex, vesselGuid });

                Vector3d relativePosition;
                Vector3d relativeVelocity;
                if (!TryReadQp(qp, out relativePosition, out relativeVelocity))
                {
                    return _stockFallback.GetCurrentState(vessel);
                }

                Vector3d velocity = GetPrincipiaVelocity(plugin, vesselGuid, relativeVelocity);
                Vector3d worldPosition = vessel.mainBody.position + relativePosition;

                return new TrajectoryState
                {
                    IsValid = true,
                    Source = SourceName,
                    Vessel = vessel,
                    ReferenceBody = vessel.mainBody,
                    UniversalTime = Planetarium.GetUniversalTime(),
                    WorldPosition = worldPosition,
                    WorldVelocity = velocity,
                    RelativePosition = relativePosition,
                    RelativeVelocity = velocity,
                    AltitudeMeters = OrbitMath.GetAltitudeAtPosition(vessel.mainBody, worldPosition),
                    LatitudeDeg = vessel.mainBody.GetLatitude(worldPosition),
                    LongitudeDeg = vessel.mainBody.GetLongitude(worldPosition)
                };
            }
            catch
            {
                return _stockFallback.GetCurrentState(vessel);
            }
        }

        // Returns orbit summary data through the provider boundary; currently stock osculating elements.
        public OrbitInfo GetOrbitInfo(Vessel vessel)
        {
            return _stockFallback.GetOrbitInfo(vessel);
        }

        // Returns current Principia position when available, otherwise stock world position.
        public Vector3d GetPosition(Vessel vessel)
        {
            TrajectoryState state = GetCurrentState(vessel);
            return state != null && state.IsValid ? state.WorldPosition : _stockFallback.GetPosition(vessel);
        }

        // Uses stock propagation for future samples until Principia prediction iterators are wired in.
        public Vector3d GetPositionAtUt(Vessel vessel, double universalTime)
        {
            return _stockFallback.GetPositionAtUt(vessel, universalTime);
        }

        // Returns the current Principia velocity when available, otherwise stock orbital velocity.
        public Vector3d GetVelocity(Vessel vessel)
        {
            TrajectoryState state = GetCurrentState(vessel);
            return state != null && state.IsValid ? state.WorldVelocity : _stockFallback.GetVelocity(vessel);
        }

        // Returns surface-relative velocity through the provider boundary.
        public Vector3d GetSurfaceVelocity(Vessel vessel)
        {
            return _stockFallback.GetSurfaceVelocity(vessel);
        }

        // Returns orbit normal through the provider boundary; currently stock osculating normal.
        public Vector3d GetOrbitNormal(Vessel vessel)
        {
            return _stockFallback.GetOrbitNormal(vessel);
        }

        // Returns apoapsis through the provider boundary; currently stock osculating apoapsis.
        public double GetApoapsisAlt(Vessel vessel)
        {
            return _stockFallback.GetApoapsisAlt(vessel);
        }

        // Returns periapsis through the provider boundary; currently stock osculating periapsis.
        public double GetPeriapsisAlt(Vessel vessel)
        {
            return _stockFallback.GetPeriapsisAlt(vessel);
        }


        // Resolves Principia adapter methods lazily so Blackbird can load without Principia installed.
        private bool EnsureReflection()
        {
            if (_adapterType != null && _interfaceType != null) return true;

            _adapterType = Type.GetType(AdapterTypeName);
            _interfaceType = Type.GetType(InterfaceTypeName);

            if (_adapterType == null || _interfaceType == null) return false;

            _pluginRunningMethod = _adapterType.GetMethod(
                "PluginRunning",
                BindingFlags.Public | BindingFlags.Instance);

            _pluginMethod = _adapterType.GetMethod(
                "Plugin",
                BindingFlags.NonPublic | BindingFlags.Instance);

            _hasVesselMethod = _interfaceType.GetMethod(
                "HasVessel",
                BindingFlags.NonPublic | BindingFlags.Static);

            _vesselFromParentMethod = _interfaceType.GetMethod(
                "VesselFromParent",
                BindingFlags.NonPublic | BindingFlags.Static);

            _vesselVelocityMethod = _interfaceType.GetMethod(
                "VesselVelocity",
                BindingFlags.NonPublic | BindingFlags.Static);

            return _pluginRunningMethod != null &&
                   _pluginMethod != null &&
                   _hasVesselMethod != null &&
                   _vesselFromParentMethod != null;
        }

        // Finds the live Principia plugin adapter MonoBehaviour.
        private object FindAdapter()
        {
            if (!EnsureReflection()) return null;

            UnityEngine.Object adapter = UnityEngine.Object.FindObjectOfType(_adapterType);
            return adapter;
        }

        private bool IsPluginRunning(object adapter)
        {
            if (adapter == null || _pluginRunningMethod == null) return false;

            object result = _pluginRunningMethod.Invoke(adapter, null);
            return result is bool && (bool)result;
        }

        private IntPtr GetPlugin(object adapter)
        {
            if (adapter == null || _pluginMethod == null) return IntPtr.Zero;

            object result = _pluginMethod.Invoke(adapter, null);
            return result is IntPtr ? (IntPtr)result : IntPtr.Zero;
        }

        private bool HasVessel(IntPtr plugin, string vesselGuid)
        {
            object result = _hasVesselMethod.Invoke(null, new object[] { plugin, vesselGuid });
            return result is bool && (bool)result;
        }

        private Vector3d GetPrincipiaVelocity(IntPtr plugin, string vesselGuid, Vector3d fallback)
        {
            if (_vesselVelocityMethod == null) return fallback;

            object xyz = _vesselVelocityMethod.Invoke(null, new object[] { plugin, vesselGuid });
            Vector3d velocity;
            return TryReadXyz(xyz, out velocity) ? velocity : fallback;
        }

        private static string GetVesselGuid(Vessel vessel)
        {
            return vessel != null && vessel.id != null ? vessel.id.ToString() : null;
        }

        private static bool TryReadQp(object qp, out Vector3d q, out Vector3d p)
        {
            q = Vector3d.zero;
            p = Vector3d.zero;
            if (qp == null) return false;

            Type type = qp.GetType();
            object qValue = GetMemberValue(type, qp, "q");
            object pValue = GetMemberValue(type, qp, "p");

            return TryReadXyz(qValue, out q) && TryReadXyz(pValue, out p);
        }

        private static bool TryReadXyz(object xyz, out Vector3d value)
        {
            value = Vector3d.zero;
            if (xyz == null) return false;

            Type type = xyz.GetType();
            object x = GetMemberValue(type, xyz, "x");
            object y = GetMemberValue(type, xyz, "y");
            object z = GetMemberValue(type, xyz, "z");

            if (x == null || y == null || z == null) return false;

            value = new Vector3d(
                Convert.ToDouble(x),
                Convert.ToDouble(y),
                Convert.ToDouble(z));
            return true;
        }

        private static object GetMemberValue(Type type, object instance, string name)
        {
            FieldInfo field = type.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (field != null) return field.GetValue(instance);

            PropertyInfo property = type.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            return property != null ? property.GetValue(instance, null) : null;
        }
    }
}
