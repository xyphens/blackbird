using System;
using System.Linq;
using System.Reflection;
using Blackbird.Logging;
using UnityEngine;

namespace Blackbird.Compatibility
{
    // First, exploratory step of the Principia interop (see project memory reference-principia-ca-api).
    // It does NOT yet feed Principia's n-body CA into guidance. It PROBES the live Principia install via
    // reflection (no compile-time dependency on Principia, so it is version-robust and degrades cleanly)
    // and LOGS what we can reach to glog\Blackbird\compatibility.log:
    //   - is Principia loaded, and its version,
    //   - the live PrincipiaPluginAdapter instance + the native `plugin` IntPtr handle,
    //   - whether each Interface method we will need binds, and with what signature,
    //   - the active/target vessel GUIDs Principia identifies craft by.
    // We review that log after an in-game test, then decide whether the full RenderedPredictionClosest-
    // Approaches integration is safe (and whether the signatures still match this Principia release — the
    // signature-probe is the compat gate, NOT the version string). Everything is wrapped so it can never
    // throw into the flight path; on any failure it logs and reports Available = false.
    public static class Principia
    {
        private static readonly BlackbirdLog Log = new BlackbirdLog(LogContext.Compatibility);

        // True only if the adapter, a non-zero plugin handle, and ALL needed methods were found. The CA
        // integration (next step) gates on this; if false we fall back to our two-body ClosestApproachSolver.
        public static bool Available { get; private set; }

        private const string AdapterTypeName = "principia.ksp_plugin_adapter.PrincipiaPluginAdapter";
        private const string InterfaceTypeName = "principia.ksp_plugin_adapter.Interface";

        // Interface methods we will need for the n-body CA read (probed for existence + signature).
        private static readonly string[] NeededMethods =
        {
            "RenderedPredictionClosestApproaches",
            "UpdatePrediction",
            "IteratorGetDiscreteTrajectoryQP",
            "IteratorGetDiscreteTrajectoryTime",
        };

        // Probe the live Principia install and log everything reachable. Safe to call when rendezvous is
        // engaged (the plugin handle is only valid in flight). Never throws.
        public static void Probe(Vessel active, Vessel target)
        {
            try { ProbeInner(active, target); }
            catch (Exception e)
            {
                Available = false;
                Log.Write("PRINCIPIA-PROBE", "FAILED (exception)", e.GetType().Name, e.Message);
            }
        }

        private static void ProbeInner(Vessel active, Vessel target)
        {
            Log.Write("PRINCIPIA-PROBE", "=== probe start ===",
                "active=" + (active != null ? active.id.ToString() : "null"),
                "target=" + (target != null ? target.id.ToString() : "null"));

            Type adapterType = FindType(AdapterTypeName);
            Type interfaceType = FindType(InterfaceTypeName);
            if (adapterType == null || interfaceType == null)
            {
                Available = false;
                Log.Write("PRINCIPIA-PROBE", "Principia NOT present (adapter types not loaded)",
                    "adapterType=" + (adapterType != null), "interfaceType=" + (interfaceType != null));
                return;
            }

            UnityEngine.Object adapterObj = UnityEngine.Object.FindObjectOfType(adapterType);
            Log.Write("PRINCIPIA-PROBE", "adapter loaded",
                "assembly=" + adapterType.Assembly.GetName().Name, "instanceFound=" + (adapterObj != null));

            LogVersion(interfaceType);

            // Native plugin handle: a private IntPtr field on the adapter instance (Principia names it
            // `plugin_`). Take the first non-zero IntPtr field and log which one.
            IntPtr plugin = IntPtr.Zero;
            string pluginField = "<none>";
            if (adapterObj != null)
            {
                FieldInfo[] fields = adapterType.GetFields(
                    BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                for (int i = 0; i < fields.Length; i++)
                {
                    if (fields[i].FieldType != typeof(IntPtr)) continue;
                    object v = fields[i].GetValue(adapterObj);
                    if (v != null && ((IntPtr)v) != IntPtr.Zero)
                    {
                        plugin = (IntPtr)v;
                        pluginField = fields[i].Name;
                        break;
                    }
                }
            }
            Log.Write("PRINCIPIA-PROBE", "plugin handle", "field=" + pluginField,
                "nonZero=" + (plugin != IntPtr.Zero));

            int bound = 0;
            for (int i = 0; i < NeededMethods.Length; i++)
            {
                MethodInfo m = interfaceType.GetMethod(NeededMethods[i],
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                if (m == null) { Log.Write("PRINCIPIA-PROBE", "method MISSING: " + NeededMethods[i]); continue; }
                bound++;
                string sig = string.Join(", ",
                    m.GetParameters().Select(p => p.ParameterType.Name + " " + p.Name).ToArray());
                Log.Write("PRINCIPIA-PROBE", "method OK: " + NeededMethods[i],
                    m.ReturnType.Name + "(" + sig + ")");
            }

            Available = adapterObj != null && plugin != IntPtr.Zero && bound == NeededMethods.Length;
            Log.Write("PRINCIPIA-PROBE", "=== SUMMARY ===",
                "available=" + Available,
                "methods=" + bound + "/" + NeededMethods.Length,
                "handle=" + (plugin != IntPtr.Zero));
        }

        // Principia's Interface.GetVersion(out ...) — invoke with a fresh args array and log whatever the
        // out parameters come back with (build date / version string). Best effort.
        private static void LogVersion(Type interfaceType)
        {
            try
            {
                MethodInfo getVersion = interfaceType.GetMethod("GetVersion",
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                if (getVersion == null) { Log.Write("PRINCIPIA-PROBE", "GetVersion not found"); return; }
                object[] args = new object[getVersion.GetParameters().Length];
                getVersion.Invoke(null, args);
                Log.Write("PRINCIPIA-PROBE", "version",
                    string.Join(" | ", args.Select(a => a == null ? "null" : a.ToString()).ToArray()));
            }
            catch (Exception e) { Log.Write("PRINCIPIA-PROBE", "GetVersion failed", e.GetType().Name, e.Message); }
        }

        // First loaded type matching the full name, across all loaded assemblies (Principia's adapter
        // assembly is loaded by KSP at startup if the mod is installed).
        private static Type FindType(string fullName)
        {
            Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
            for (int i = 0; i < assemblies.Length; i++)
            {
                try
                {
                    Type t = assemblies[i].GetType(fullName, false);
                    if (t != null) return t;
                }
                catch { /* dynamic/secured assemblies: skip */ }
            }
            return null;
        }
    }
}
