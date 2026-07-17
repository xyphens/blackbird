using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace Blackbird.Helpers
{
    // Persists window positions across scene changes (the addon is rebuilt every scene) and game restarts.
    // Only x/y are stored; width/height stay owned by GUILayout auto-sizing.
    public static class WindowPositions
    {
        private const string RootNodeName = "BLACKBIRD_WINDOWS";
        private const string FileName = "windows.cfg";
        private const string DataFolderName = "PluginData";

        // How much of a restored window must stay on screen to remain grabbable after a resolution change.
        private const float MinVisibleWidth = 120f;
        private const float MinVisibleHeight = 30f;

        private static readonly Dictionary<string, Vector2> _positions = new Dictionary<string, Vector2>();
        private static bool _loaded;
        private static bool _dirty;

        // Returns the stored position for `key` at the fallback's size, or the fallback when nothing is stored.
        public static Rect Restore(string key, Rect fallback)
        {
            Load();
            if (!_positions.TryGetValue(key, out Vector2 p)) return fallback;
            return ClampToScreen(new Rect(p.x, p.y, fallback.width, fallback.height), Screen.width, Screen.height);
        }

        // Cheap enough to call every OnGUI pass; only a moved window marks the store dirty.
        public static void Record(string key, Rect rect)
        {
            if (_positions.TryGetValue(key, out Vector2 p) && p.x == rect.x && p.y == rect.y) return;
            _positions[key] = new Vector2(rect.x, rect.y);
            _dirty = true;
        }

        public static void Save()
        {
            if (!_dirty) return;
            try
            {
                string path = ConfigFilePath();
                if (path == null) return;

                Directory.CreateDirectory(Path.GetDirectoryName(path));
                ConfigNode root = new ConfigNode();
                ConfigNode node = root.AddNode(RootNodeName);
                foreach (KeyValuePair<string, Vector2> entry in _positions)
                    node.AddValue(entry.Key, FormatPosition(entry.Value));
                root.Save(path);
                _dirty = false;
            }
            catch (Exception e)
            {
                Debug.LogWarning("[BlackBird] Window positions not saved: " + e.Message);
            }
        }

        public static Rect ClampToScreen(Rect rect, float screenWidth, float screenHeight)
        {
            rect.x = Mathf.Clamp(rect.x, MinVisibleWidth - rect.width, screenWidth - MinVisibleWidth);
            rect.y = Mathf.Clamp(rect.y, 0f, screenHeight - MinVisibleHeight);
            return rect;
        }

        public static bool TryParsePosition(string s, out Vector2 p)
        {
            p = Vector2.zero;
            if (string.IsNullOrEmpty(s)) return false;

            string[] parts = s.Split(',');
            if (parts.Length != 2) return false;
            if (!float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x)) return false;
            if (!float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y)) return false;
            if (float.IsNaN(x) || float.IsInfinity(x) || float.IsNaN(y) || float.IsInfinity(y)) return false;

            p = new Vector2(x, y);
            return true;
        }

        private static void Load()
        {
            if (_loaded) return;
            _loaded = true;
            try
            {
                string path = ConfigFilePath();
                if (path == null || !File.Exists(path)) return;

                ConfigNode root = ConfigNode.Load(path);
                ConfigNode node = root?.GetNode(RootNodeName);
                if (node == null) return;

                foreach (ConfigNode.Value v in node.values)
                    if (TryParsePosition(v.value, out Vector2 p)) _positions[v.name] = p;
            }
            catch (Exception e)
            {
                Debug.LogWarning("[BlackBird] Window positions not loaded: " + e.Message);
            }
        }

        public static string FormatPosition(Vector2 p)
        {
            return p.x.ToString("R", CultureInfo.InvariantCulture) + "," + p.y.ToString("R", CultureInfo.InvariantCulture);
        }

        // Beside the DLL, so the file follows the install rather than a hard-coded GameData path.
        private static string ConfigFilePath()
        {
            try
            {
                string dir = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
                if (string.IsNullOrEmpty(dir)) return null;
                return Path.Combine(Path.Combine(dir, DataFolderName), FileName);
            }
            catch (Exception)
            {
                return null;
            }
        }
    }
}
