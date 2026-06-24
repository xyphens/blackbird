using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace Blackbird.Logging
{
    // Identifies which file a log instance writes to; one file per context under the glog folder.
    public enum LogContext
    {
        Debug,
        Psg,
        Rendezvous,
        Compatibility,
        Docking
    }

    // BlackbirdLog: lightweight, fire-and-forget file logger for offline diagnosis.
    //
    // Usage is two lines total at the call site:
    //   private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Psg);   // once
    //   _log.Write(problem, result, someVector, 42, "note");                     // anywhere, per-frame OK
    //
    // Write() accepts any number of objects of ANY type and reflects each into a compact JSON-ish
    // string. Unlike UnityEngine.JsonUtility (fields-only, [Serializable]-only, and silently emits
    // "{}" for properties/primitives/strings/collections), this serializer walks public PROPERTIES
    // and FIELDS and renders scalars/vectors/collections inline. So existing property-based DTOs
    // (PsgProblem, PsgOptimizationResult, RelativeState, ...) log their real contents with NO changes
    // to the type or the call site.
    //
    // Performance: the file handle is opened ONCE per file and kept open (see GetWriter) — there is
    // no open/close per call, so this is safe to attach to per-frame code. Reflection metadata is
    // cached per type, and the line is built before the write lock is taken. AutoFlush is on, so each
    // line reaches the OS immediately and survives a KSP process crash (the case you most want logged).
    // Write() NEVER throws: a debug logger must not be able to break the guidance/flight path.
    public class BlackbirdLog
    {
        // Master on/off switch. Lets a build disable all logging cost without touching call sites.
        public static bool Enabled = true;

        // context -> filename. Each context maps to a distinct file.
        private static readonly Dictionary<LogContext, string> ContextToFileName = new Dictionary<LogContext, string>()
        {
            { LogContext.Psg, "psg.log" },
            { LogContext.Rendezvous, "rendezvous.log" },
            { LogContext.Compatibility, "compatibility.log" },
            { LogContext.Docking, "docking.log" },
            { LogContext.Debug, "debug.log" },
        };

        // Output location; mirrors PsgSnapshotLogger's preferred glog path.
        private const string PreferredRoot = @"D:\SteamLibrary\steamapps\common\Kerbal Space Program Development\glog";
        private const string SubFolder = "Blackbird";

        // Guards against runaway work on deep or cyclic object graphs.
        private const int MaxDepth = 6;
        private const int MaxCollectionItems = 64;

        // Per-type member list cache so GetProperties/GetFields runs once per type, not per call.
        // Concurrent so cache reads need no lock (PSG solves can finish off the main thread).
        private static readonly ConcurrentDictionary<Type, MemberInfo[]> MemberCache =
            new ConcurrentDictionary<Type, MemberInfo[]>();

        // One persistent StreamWriter per file path, shared by every BlackbirdLog that targets it.
        // Lazy<T> guarantees the file is opened exactly once even under concurrent first use.
        private static readonly ConcurrentDictionary<string, Lazy<StreamWriter>> Writers =
            new ConcurrentDictionary<string, Lazy<StreamWriter>>();

        private readonly string _fullFilePath;

        // Flush and close any open writers on shutdown. AutoFlush already protects the data; this just
        // releases the file handles cleanly. Not guaranteed on a hard crash, which is fine.
        static BlackbirdLog()
        {
            AppDomain.CurrentDomain.ProcessExit += (s, e) => CloseAll();
            AppDomain.CurrentDomain.DomainUnload += (s, e) => CloseAll();
        }

        // Resolves the target file from the context. The file is not opened until the first Write.
        public BlackbirdLog(LogContext ctx)
        {
            string fileName = ContextToFileName.TryGetValue(ctx, out string mapped) ? mapped : "debug.log";
            _fullFilePath = Path.Combine(Path.Combine(PreferredRoot, SubFolder), fileName);
        }

        // Logs any number of objects of any type as a single timestamped line, using the persistent
        // writer for this context. The whole method is wrapped so it can never throw into the caller;
        // on any failure the line is simply dropped.
        public void Write(params object[] data)
        {
            if (!Enabled || data == null || data.Length == 0) return;

            try
            {
                // Reflect each argument to a string (the slow part) before touching the writer lock.
                StringBuilder payload = new StringBuilder();
                for (int i = 0; i < data.Length; i++)
                {
                    if (i > 0) payload.Append(", ");
                    payload.Append(Serialize(data[i], 0));
                }

                string logLine = string.Format("[{0}] DATA: {1}",
                    DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
                    payload);

                // StreamWriter is not thread-safe; one writer at a time. AutoFlush pushes it to disk.
                StreamWriter writer = GetWriter(_fullFilePath);
                lock (writer)
                {
                    writer.WriteLine(logLine);
                }
            }
            catch
            {
                // Intentionally swallowed — logging must never disrupt guidance/flight code.
            }
        }

        // Flushes and closes every open writer. Safe to call manually (e.g. from an addon OnDestroy);
        // also runs automatically on process/domain exit.
        public static void CloseAll()
        {
            foreach (KeyValuePair<string, Lazy<StreamWriter>> kv in Writers)
            {
                try
                {
                    if (!kv.Value.IsValueCreated) continue;
                    StreamWriter w = kv.Value.Value;
                    lock (w) { w.Flush(); w.Dispose(); }
                }
                catch { /* best-effort cleanup */ }
            }
            Writers.Clear();
        }

        // --- persistent writer management --------------------------------------------------------

        // Returns the single, kept-open writer for a file, creating it (and the directory) on first use.
        private static StreamWriter GetWriter(string path)
        {
            return Writers.GetOrAdd(path, p => new Lazy<StreamWriter>(() => OpenWriter(p))).Value;
        }

        // Opens a file for appending and keeps it open. FileShare.ReadWrite lets you tail the log live
        // while KSP runs. AutoFlush makes every line durable without an explicit flush per call.
        private static StreamWriter OpenWriter(string path)
        {
            string dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            FileStream stream = new FileStream(path, FileMode.Append, FileAccess.Write, FileShare.ReadWrite);
            StreamWriter writer = new StreamWriter(stream) { AutoFlush = true };
            writer.WriteLine(string.Format("=== session start {0} ===", DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")));
            return writer;
        }

        // --- reflection serializer ---------------------------------------------------------------

        // Renders an arbitrary value to a compact JSON-ish string. Handles null, primitives/strings/
        // enums/DateTime, Vector3d/Vector3, IEnumerable collections, and complex objects (public
        // properties + fields). 'depth' bounds recursion against deep or cyclic graphs.
        private static string Serialize(object value, int depth)
        {
            if (value == null) return "null";

            // Scalars rendered inline.
            if (value is string str) return Quote(str);
            if (value is bool bln) return bln ? "true" : "false";
            if (value is char chr) return Quote(chr.ToString());
            if (value is DateTime dtm) return Quote(dtm.ToString("o", CultureInfo.InvariantCulture));
            if (value.GetType().IsEnum) return Quote(value.ToString());
            if (IsNumeric(value)) return Convert.ToString(value, CultureInfo.InvariantCulture);

            // KSP/Unity vectors: readable [x, y, z] rather than nested member noise.
            if (value is Vector3d v3d)
                return string.Format(CultureInfo.InvariantCulture, "[{0}, {1}, {2}]", v3d.x, v3d.y, v3d.z);
            if (value is Vector3 v3)
                return string.Format(CultureInfo.InvariantCulture, "[{0}, {1}, {2}]", v3.x, v3.y, v3.z);

            // Past the depth cap, fall back to ToString() instead of recursing further.
            if (depth >= MaxDepth) return Quote(value.ToString());

            // Collections -> [a, b, c] (capped to keep lines bounded).
            if (value is IEnumerable enumerable)
                return SerializeEnumerable(enumerable, depth);

            // Everything else: object graph via cached public members.
            return SerializeObject(value, depth);
        }

        // Serializes a collection as a bracketed, comma-separated list, capped at MaxCollectionItems.
        private static string SerializeEnumerable(IEnumerable enumerable, int depth)
        {
            StringBuilder sb = new StringBuilder("[");
            int count = 0;
            foreach (object item in enumerable)
            {
                if (count >= MaxCollectionItems) { sb.Append(", ..."); break; }
                if (count > 0) sb.Append(", ");
                sb.Append(Serialize(item, depth + 1));
                count++;
            }
            sb.Append("]");
            return sb.ToString();
        }

        // Serializes a complex object as {"Name":value, ...} over its public properties and fields.
        // A member whose getter throws is skipped rather than failing the whole line.
        private static string SerializeObject(object value, int depth)
        {
            MemberInfo[] members = GetMembers(value.GetType());
            StringBuilder sb = new StringBuilder("{");
            bool first = true;

            foreach (MemberInfo m in members)
            {
                object memberValue;
                try
                {
                    memberValue = m is PropertyInfo p
                        ? p.GetValue(value, null)
                        : ((FieldInfo)m).GetValue(value);
                }
                catch
                {
                    continue;
                }

                if (!first) sb.Append(", ");
                first = false;
                sb.Append(Quote(m.Name)).Append(":").Append(Serialize(memberValue, depth + 1));
            }

            sb.Append("}");
            return sb.ToString();
        }

        // Returns (and caches) the public, readable, non-indexer properties plus public fields of a type.
        private static MemberInfo[] GetMembers(Type type)
        {
            return MemberCache.GetOrAdd(type, BuildMembers);
        }

        private static MemberInfo[] BuildMembers(Type type)
        {
            List<MemberInfo> members = new List<MemberInfo>();

            foreach (PropertyInfo p in type.GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                if (p.CanRead && p.GetIndexParameters().Length == 0) members.Add(p);
            }
            foreach (FieldInfo f in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
            {
                members.Add(f);
            }

            return members.ToArray();
        }

        // True for the built-in numeric types (rendered without quotes).
        private static bool IsNumeric(object value)
        {
            return value is byte || value is sbyte || value is short || value is ushort
                || value is int || value is uint || value is long || value is ulong
                || value is float || value is double || value is decimal;
        }

        // Wraps a string in quotes and escapes backslashes/quotes so the line stays parseable.
        private static string Quote(string s)
        {
            return "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }

        // --- original JsonUtility implementation, superseded by the reflection serializer above ----
        // Kept for reference. JsonUtility serializes public FIELDS of [Serializable] types only and
        // emits "{}" for properties/primitives/strings/collections, so it produced empty output for
        // the property-based DTOs at the call sites. It also opened and closed the file on every call.
        //
        // public void Write(params object[] data)
        // {
        //     if (data == null || data.Length == 0) return;
        //     List<string> serialized = new List<string>();
        //
        //     foreach (object obj in data)
        //     {
        //         if (obj == null) continue;
        //         serialized.Add(JsonUtility.ToJson(obj, false));
        //     }
        //
        //     string combined = string.Join(", ", serialized.ToArray());
        //
        //     string logLine = string.Format("[{0}] DATA: {1}{2}",
        //         DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
        //         combined,
        //         Environment.NewLine);
        //
        //     File.AppendAllText(FullFilePath, logLine);
        // }
    }
}
