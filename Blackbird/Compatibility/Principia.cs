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
    //
    // This is READ-ONLY: it reflects/logs and makes only harmless native reads (GetVersion). The live
    // closest-approach read was removed (see the note where it used to live) because the required
    // SetTargetVessel call mutates Principia's global renderer state and crashed the map view.
    public static class Principia
    {
        private static readonly BlackbirdLog Log = new BlackbirdLog(LogContext.Compatibility);

        // True only if the adapter, a non-zero plugin handle, and ALL needed methods were found. The CA
        // integration (next step) gates on this; if false we fall back to our two-body ClosestApproachSolver.
        public static bool Available { get; private set; }

        private const string AdapterTypeName = "principia.ksp_plugin_adapter.PrincipiaPluginAdapter";
        private const string InterfaceTypeName = "principia.ksp_plugin_adapter.Interface";

        // Interface methods the n-body CA read needs (probed for existence + signature). Gates Available.
        // The first 4 were confirmed by the initial probe; the iterator-nav + dispose methods are needed to
        // actually walk the returned DisposableIterator and were NOT covered before.
        private static readonly string[] NeededMethods =
        {
            "RenderedPredictionClosestApproaches",
            "UpdatePrediction",
            "IteratorGetDiscreteTrajectoryQP",
            "IteratorGetDiscreteTrajectoryTime",
            "IteratorAtEnd",
            "IteratorIncrement",
        };

        // Optional methods (logged but not gating): iterator size, and the prediction-length parameters so we
        // can READ (never override) the user's horizon and warn when it's too short to reach the true CA.
        private static readonly string[] OptionalMethods =
        {
            "IteratorSize",
            "VesselGetPredictionAdaptiveStepParameters",
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
                if (LogMethod(interfaceType, NeededMethods[i])) bound++;
            }

            // Optional methods + the struct layouts we must mirror for P/Invoke (XYZ / QP / DisposableIterator /
            // the prediction-step params). These are the previously-unverified pieces the real read needs.
            for (int i = 0; i < OptionalMethods.Length; i++) LogMethod(interfaceType, OptionalMethods[i]);
            LogStructLayouts(interfaceType);
            LogIteratorDisposal(interfaceType);
            LogMethodSurface(interfaceType);

            Available = adapterObj != null && plugin != IntPtr.Zero && bound == NeededMethods.Length;
            Log.Write("PRINCIPIA-PROBE", "=== SUMMARY ===",
                "available=" + Available,
                "methods=" + bound + "/" + NeededMethods.Length,
                "handle=" + (plugin != IntPtr.Zero));
        }

        // NOTE: the live closest-approach READ (TestClosestApproach) was REMOVED 2026-06-19. The reflection
        // call sequence worked (UpdatePrediction → SetTargetVessel → RenderedPredictionClosestApproaches →
        // iterator walk → Dispose), but SetTargetVessel MUTATES Principia's global renderer target+frame and
        // there is no harmless way to undo it — leaving it set crashed the game when the map view rendered the
        // mutated frame (confirmed in-game). Principia's renderer target is independent of KSP's target
        // selection (proven: RCA aborted while a KSP target was selected), so we cannot rely on auto-resync.
        // A live readout would have to call SetTargetVessel every refresh = continuously hijack the user's
        // target/frame, which is unacceptable. The full working recipe is preserved in memory
        // (reference-principia-ca-api) should we revisit with a save-target/restore-target design. Only the
        // read-only probe below remains. See the two-body ClosestApproachSolver for the side-effect-free path.

        // Resolve + log one Interface method's return/parameter signature. Returns whether it bound.
        private static bool LogMethod(Type interfaceType, string name)
        {
            MethodInfo m = interfaceType.GetMethod(name,
                BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
            if (m == null) { Log.Write("PRINCIPIA-PROBE", "method MISSING: " + name); return false; }

            string sig = string.Join(", ",
                m.GetParameters().Select(p =>
                    (p.ParameterType.IsByRef ? "out " : "") + p.ParameterType.Name + " " + p.Name).ToArray());
            Log.Write("PRINCIPIA-PROBE", "method OK: " + name, m.ReturnType.Name + "(" + sig + ")");
            return true;
        }

        // Log the field layout of the marshalling structs we must mirror exactly for the P/Invoke read. We
        // derive the types from the bound method signatures rather than guessing names: QP is the return of
        // IteratorGetDiscreteTrajectoryQP, XYZ is QP's first field, DisposableIterator is the out-iterator of
        // RenderedPredictionClosestApproaches, and the prediction params are the return of the Vessel getter.
        private static void LogStructLayouts(Type interfaceType)
        {
            try
            {
                MethodInfo qpGet = interfaceType.GetMethod("IteratorGetDiscreteTrajectoryQP",
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                Type qpType = qpGet != null ? qpGet.ReturnType : null;
                LogTypeLayout("QP", qpType);

                if (qpType != null)
                {
                    FieldInfo[] qpFields = qpType.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    if (qpFields.Length > 0) LogTypeLayout("XYZ (QP field)", qpFields[0].FieldType);
                }

                MethodInfo rca = interfaceType.GetMethod("RenderedPredictionClosestApproaches",
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                if (rca != null)
                {
                    ParameterInfo[] ps = rca.GetParameters();
                    if (ps.Length > 0)
                    {
                        Type iterType = ps[ps.Length - 1].ParameterType;
                        if (iterType.IsByRef) iterType = iterType.GetElementType();
                        LogTypeLayout("DisposableIterator", iterType);
                    }
                }

                MethodInfo predParams = interfaceType.GetMethod("VesselGetPredictionAdaptiveStepParameters",
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                if (predParams != null) LogTypeLayout("AdaptiveStepParameters", predParams.ReturnType);
            }
            catch (Exception e)
            {
                Log.Write("PRINCIPIA-PROBE", "struct-layout probe failed", e.GetType().Name, e.Message);
            }
        }

        // How do we free the iterator the CA call returns? `IteratorDispose` (Interface static) does NOT
        // exist — DisposableIterator is Principia's IDisposable handle, freed via the returned object. Log the
        // mechanism so we use it correctly (a wrong guess leaks a native iterator every CA poll): is the type
        // a value type, does it implement IDisposable, what Dispose/Delete methods does it expose, and what
        // Iterator* static methods exist on Interface (in case there is an IteratorDelete instead).
        private static void LogIteratorDisposal(Type interfaceType)
        {
            try
            {
                MethodInfo rca = interfaceType.GetMethod("RenderedPredictionClosestApproaches",
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                Type iterType = null;
                if (rca != null)
                {
                    ParameterInfo[] ps = rca.GetParameters();
                    if (ps.Length > 0)
                    {
                        iterType = ps[ps.Length - 1].ParameterType;
                        if (iterType.IsByRef) iterType = iterType.GetElementType();
                    }
                }

                if (iterType != null)
                {
                    bool disposable = typeof(IDisposable).IsAssignableFrom(iterType);
                    string instanceMethods = string.Join(", ", iterType
                        .GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
                        .Where(m => m.Name == "Dispose" || m.Name.IndexOf("Delete", StringComparison.OrdinalIgnoreCase) >= 0)
                        .Select(m => m.Name).Distinct().ToArray());
                    Log.Write("PRINCIPIA-PROBE", "DisposableIterator disposal",
                        "isValueType=" + iterType.IsValueType, "IDisposable=" + disposable,
                        "methods={ " + instanceMethods + " }");
                }

                string iterStatics = string.Join(", ", interfaceType
                    .GetMethods(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic)
                    .Where(m => m.Name.IndexOf("Iterator", StringComparison.OrdinalIgnoreCase) >= 0)
                    .Select(m => m.Name).Distinct().ToArray());
                Log.Write("PRINCIPIA-PROBE", "Interface Iterator* statics", iterStatics);
            }
            catch (Exception e)
            {
                Log.Write("PRINCIPIA-PROBE", "iterator-disposal probe failed", e.GetType().Name, e.Message);
            }
        }

        // SAFE (pure reflection, NO native calls) enumeration of the Interface method surface relevant to the
        // CA work: any method whose name mentions Target / Prediction / Apsides / Nodes or starts with Rendered.
        // RCA crashes natively (likely an unmet precondition like "no target set in the renderer"); this reveals
        // the exact setter/getter names (e.g. SetTargetVessel, HasTargetVessel, the real RenderedPrediction
        // name) so we can satisfy that precondition without guessing. Logged with signatures.
        private static void LogMethodSurface(Type interfaceType)
        {
            try
            {
                MethodInfo[] methods = interfaceType.GetMethods(
                    BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
                foreach (MethodInfo m in methods)
                {
                    string n = m.Name;
                    bool relevant =
                        n.IndexOf("Target", StringComparison.OrdinalIgnoreCase) >= 0
                        || n.IndexOf("Prediction", StringComparison.OrdinalIgnoreCase) >= 0
                        || n.IndexOf("Apsides", StringComparison.OrdinalIgnoreCase) >= 0
                        || n.IndexOf("Nodes", StringComparison.OrdinalIgnoreCase) >= 0
                        || n.StartsWith("Rendered", StringComparison.OrdinalIgnoreCase);
                    if (!relevant) continue;

                    string sig = string.Join(", ", m.GetParameters().Select(p =>
                        (p.ParameterType.IsByRef ? "out " : "") + p.ParameterType.Name + " " + p.Name).ToArray());
                    Log.Write("PRINCIPIA-SURFACE", n, m.ReturnType.Name + "(" + sig + ")");
                }
            }
            catch (Exception e)
            {
                Log.Write("PRINCIPIA-SURFACE", "enumeration failed", e.GetType().Name, e.Message);
            }
        }

        // Log a value type's full name + its fields (type + name), so we can mirror it for marshalling.
        private static void LogTypeLayout(string label, Type t)
        {
            if (t == null) { Log.Write("PRINCIPIA-PROBE", "layout " + label + ": <null>"); return; }
            FieldInfo[] fields = t.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            string fieldStr = string.Join(", ", fields.Select(f => f.FieldType.Name + " " + f.Name).ToArray());
            Log.Write("PRINCIPIA-PROBE", "layout " + label + " = " + t.FullName, "{ " + fieldStr + " }");
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
