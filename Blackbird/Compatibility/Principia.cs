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

            Available = adapterObj != null && plugin != IntPtr.Zero && bound == NeededMethods.Length;
            Log.Write("PRINCIPIA-PROBE", "=== SUMMARY ===",
                "available=" + Available,
                "methods=" + bound + "/" + NeededMethods.Length,
                "handle=" + (plugin != IntPtr.Zero));
        }

        // Make the real closest-approach call and log the raw iterator output, so we can interpret the point
        // semantics before committing to a distance computation. Logs: point count, each point's UT (+dt from
        // now) and world-frame q with several candidate distances (|q|, |q-sun|, |q-targetNow|), plus the
        // reference current positions. From this we learn whether q is the vessel trajectory point (pair with
        // the target), the separation, or vessel/target pairs. Frozen iterator is disposed via IDisposable.
        private static void LogClosestApproachRaw(Type interfaceType, IntPtr plugin, Vessel active, Vessel target)
        {
            try
            {
                if (plugin == IntPtr.Zero || active == null || target == null)
                { Log.Write("PRINCIPIA-CA", "skip (no plugin/vessels)"); return; }

                MethodInfo update = M(interfaceType, "UpdatePrediction");
                MethodInfo rca = M(interfaceType, "RenderedPredictionClosestApproaches");
                MethodInfo atEnd = M(interfaceType, "IteratorAtEnd");
                MethodInfo incr = M(interfaceType, "IteratorIncrement");
                MethodInfo size = M(interfaceType, "IteratorSize");
                MethodInfo getQp = M(interfaceType, "IteratorGetDiscreteTrajectoryQP");
                MethodInfo getTime = M(interfaceType, "IteratorGetDiscreteTrajectoryTime");
                if (update == null || rca == null || atEnd == null || incr == null || getQp == null || getTime == null)
                { Log.Write("PRINCIPIA-CA", "skip (method resolve failed)"); return; }

                Type qpType = getQp.ReturnType;
                FieldInfo qpQField = qpType.GetField("q", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                Type xyzType = qpQField != null ? qpQField.FieldType : null;
                if (xyzType == null) { Log.Write("PRINCIPIA-CA", "skip (no XYZ type)"); return; }

                string guid = active.id.ToString();
                Vector3d sun = Planetarium.fetch.Sun.position;
                object sunXyz = MakeXyz(xyzType, sun.x, sun.y, sun.z);

                // Refresh the active vessel's prediction, then fetch its closest approaches to the current target.
                update.Invoke(null, new object[] { plugin, new[] { guid } });
                object[] args = { plugin, guid, sunXyz, 8, null };
                rca.Invoke(null, args);
                object iter = args[4];
                if (iter == null) { Log.Write("PRINCIPIA-CA", "iterator null"); return; }

                double nowUt = Planetarium.GetUniversalTime();
                Vector3d aw = active.GetWorldPos3D();
                Vector3d tw = target.GetWorldPos3D();
                string targetGuid = target.id.ToString();

                // (1) Prediction horizon knob (max_steps + tolerances) for the active vessel.
                MethodInfo predParams = M(interfaceType, "VesselGetPredictionAdaptiveStepParameters");
                if (predParams != null)
                {
                    object ap = predParams.Invoke(null, new object[] { plugin, guid });
                    Type apType = ap.GetType();
                    Log.Write("PRINCIPIA-CA", "prediction params",
                        "max_steps=" + ReadField(apType, ap, "max_steps"),
                        "len_tol=" + ReadField(apType, ap, "length_integration_tolerance"),
                        "spd_tol=" + ReadField(apType, ap, "speed_integration_tolerance"));
                }

                // (2) Predicted trajectory EXTENTS for both vessels (the real horizon span) + a few sample
                // points, so if the CA points turn out to be vessel-only we already have the target's track to
                // interpolate, and we know how far the prediction reaches.
                MethodInfo renderedPrediction = M(interfaceType, "RenderedPrediction");
                LogPredictionExtent("active", renderedPrediction, atEnd, incr, size, getQp, getTime,
                    qpType, qpQField, xyzType, plugin, guid, sunXyz, nowUt, tw);
                LogPredictionExtent("target", renderedPrediction, atEnd, incr, size, getQp, getTime,
                    qpType, qpQField, xyzType, plugin, targetGuid, sunXyz, nowUt, aw);

                // (3) The closest-approach iterator itself.
                int n = size != null ? Convert.ToInt32(size.Invoke(null, new[] { iter })) : -1;
                Log.Write("PRINCIPIA-CA", "CA points=" + n,
                    "activeWorld=" + aw, "targetWorld=" + tw,
                    "curSep=" + (aw - tw).magnitude.ToString("F1"), "sun=" + sun);

                MethodInfo dGetQp = M(interfaceType, "IteratorGetDistinguishedPointsQP");
                MethodInfo dGetTime = M(interfaceType, "IteratorGetDistinguishedPointsTime");
                MethodInfo getXyz = M(interfaceType, "IteratorGetDiscreteTrajectoryXYZ");
                FieldInfo qpPField = qpType.GetField("p", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);

                int i = 0;
                while (!Convert.ToBoolean(atEnd.Invoke(null, new[] { iter })) && i < 12)
                {
                    object qp = getQp.Invoke(null, new[] { iter });
                    double t = Convert.ToDouble(getTime.Invoke(null, new[] { iter }));
                    Vector3d q = ReadXyz(xyzType, qpQField.GetValue(qp));
                    Log.Write("PRINCIPIA-CA", "pt " + i,
                        "dt=" + (t - nowUt).ToString("F1") + "s",
                        "q=" + q, "|q|=" + q.magnitude.ToString("E3"),
                        "|q-sun|=" + (q - sun).magnitude.ToString("F1"),
                        "|q-targetNow|=" + (q - tw).magnitude.ToString("F1"));

                    // Distinguished points (the likely vessel/target endpoint pair) — try/catch since this
                    // getter may not apply to this iterator kind; the failure is itself informative.
                    if (dGetQp != null)
                    {
                        try
                        {
                            object dqp = dGetQp.Invoke(null, new[] { iter });
                            Vector3d dq = ReadXyz(xyzType, qpQField.GetValue(dqp));
                            Vector3d dp = qpPField != null ? ReadXyz(xyzType, qpPField.GetValue(dqp)) : Vector3d.zero;
                            Log.Write("PRINCIPIA-CA", "  pt " + i + " distinguished",
                                "q=" + dq, "p=" + dp, "|q-p|=" + (dq - dp).magnitude.ToString("F1"));
                        }
                        catch (Exception ex) { Log.Write("PRINCIPIA-CA", "  pt " + i + " distinguished N/A", ex.GetType().Name); }
                    }

                    if (getXyz != null)
                    {
                        try
                        {
                            Vector3d xyz = ReadXyz(xyzType, getXyz.Invoke(null, new[] { iter }));
                            Log.Write("PRINCIPIA-CA", "  pt " + i + " XYZ", "v=" + xyz, "|v|=" + xyz.magnitude.ToString("E3"));
                        }
                        catch (Exception ex) { Log.Write("PRINCIPIA-CA", "  pt " + i + " XYZ N/A", ex.GetType().Name); }
                    }

                    incr.Invoke(null, new[] { iter });
                    i++;
                }

                (iter as IDisposable)?.Dispose();
                Log.Write("PRINCIPIA-CA", "iterator disposed");
            }
            catch (Exception e)
            {
                Log.Write("PRINCIPIA-CA", "FAILED", e.GetType().Name, e.Message);
            }
        }

        // Walk a vessel's rendered prediction: log point count, the time span (= the real horizon reach), and
        // first/mid/last sample points (dt + world q + distance to the supplied reference position `refPos`).
        // For the target this gives us its predicted track to interpolate against if the CA points are
        // vessel-only; for either it tells us how far the prediction actually reaches.
        private static void LogPredictionExtent(
            string label, MethodInfo renderedPrediction, MethodInfo atEnd, MethodInfo incr, MethodInfo size,
            MethodInfo getQp, MethodInfo getTime, Type qpType, FieldInfo qpQField, Type xyzType,
            IntPtr plugin, string guid, object sunXyz, double nowUt, Vector3d refPos)
        {
            if (renderedPrediction == null) { Log.Write("PRINCIPIA-CA", "RenderedPrediction not found"); return; }
            try
            {
                object[] args = { plugin, guid, sunXyz, null };
                renderedPrediction.Invoke(null, args);
                object iter = args[3];
                if (iter == null) { Log.Write("PRINCIPIA-CA", "prediction[" + label + "] iterator null"); return; }

                int n = size != null ? Convert.ToInt32(size.Invoke(null, new[] { iter })) : -1;
                double firstT = double.NaN, lastT = double.NaN;
                int i = 0;
                while (!Convert.ToBoolean(atEnd.Invoke(null, new[] { iter })))
                {
                    double t = Convert.ToDouble(getTime.Invoke(null, new[] { iter }));
                    if (i == 0) firstT = t;
                    lastT = t;
                    // Sample the first, and roughly every quarter, to characterize the track without spamming.
                    if (i == 0 || (n > 0 && i % Math.Max(1, n / 4) == 0))
                    {
                        Vector3d q = ReadXyz(xyzType, qpQField.GetValue(getQp.Invoke(null, new[] { iter })));
                        Log.Write("PRINCIPIA-CA", "  pred[" + label + "] pt " + i,
                            "dt=" + (t - nowUt).ToString("F1") + "s",
                            "|q-refNow|=" + (q - refPos).magnitude.ToString("F1"));
                    }
                    incr.Invoke(null, new[] { iter });
                    i++;
                }

                Log.Write("PRINCIPIA-CA", "prediction[" + label + "]",
                    "points=" + n, "spanSeconds=" + (lastT - firstT).ToString("F0"),
                    "reachDt=" + (lastT - nowUt).ToString("F0") + "s");
                (iter as IDisposable)?.Dispose();
            }
            catch (Exception e)
            {
                Log.Write("PRINCIPIA-CA", "prediction[" + label + "] FAILED", e.GetType().Name, e.Message);
            }
        }

        private static string ReadField(Type type, object instance, string name)
        {
            FieldInfo f = type.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            object v = f != null ? f.GetValue(instance) : null;
            return v != null ? v.ToString() : "<none>";
        }

        private static MethodInfo M(Type t, string name) =>
            t.GetMethod(name, BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

        // Build a Principia XYZ struct (boxed) with the given components, via reflection.
        private static object MakeXyz(Type xyzType, double x, double y, double z)
        {
            object o = Activator.CreateInstance(xyzType);
            SetXyzField(xyzType, o, "x", x);
            SetXyzField(xyzType, o, "y", y);
            SetXyzField(xyzType, o, "z", z);
            return o;
        }

        private static void SetXyzField(Type xyzType, object boxed, string name, double value)
        {
            FieldInfo f = xyzType.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            if (f != null) f.SetValue(boxed, value);
        }

        private static Vector3d ReadXyz(Type xyzType, object xyz)
        {
            double x = Convert.ToDouble(xyzType.GetField("x", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).GetValue(xyz));
            double y = Convert.ToDouble(xyzType.GetField("y", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).GetValue(xyz));
            double z = Convert.ToDouble(xyzType.GetField("z", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).GetValue(xyz));
            return new Vector3d(x, y, z);
        }

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
