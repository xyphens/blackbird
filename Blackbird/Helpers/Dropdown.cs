using System;
using UnityEngine;

namespace Blackbird.Helpers
{
    public class Dropdown
    {
        private static Rect rect;
        private static object owner;
        private static string[] items;
        private static bool _isActive;
        private static int _selectedIndex;

        private static float Scale = Mathf.Clamp((float)1.0, 0.2f, 5f);
        private static int ScreenWidth = Mathf.RoundToInt(Screen.width / Scale);
        private static int ScreenHeight = Mathf.RoundToInt(Screen.height / Scale);

        private static readonly int windowId = GUIUtility.GetControlID(FocusType.Passive);
        private static readonly GUIStyle style;
        private static bool IsActive => owner != null && rect.height > 0 && _isActive;
        public static Rect PopupRect => rect;

        static Dropdown()
        {
            style = new GUIStyle(GUI.skin.window) { normal = { background = null }, onNormal = { background = null } };
            style.border.top = style.border.bottom;
            style.padding.top = style.padding.bottom;
        }

        private static Texture2D Container()
        {
            Texture2D bg = new Texture2D(16, 16, TextureFormat.RGBA32, false);

            bg.wrapMode = TextureWrapMode.Clamp;
            for (int x = 0; x < bg.width; x++)
                for (int y = 0; y < bg.height; y++)
                {
                    if (x == 0 || x == bg.width - 1 || y == 0 || y == bg.height - 1)
                        bg.SetPixel(x, y, new Color(0, 0, 0, 1));
                    else
                        bg.SetPixel(x, y, new Color(0.05f, 0.05f, 0.05f, 0.95f));
                }

            bg.Apply();

            return bg;
        }

        public static void DrawGUI()
        {
            if (owner == null || rect.height <= 0 || !_isActive) return;

            if (style.normal.background == null) style.normal.background = Container();

            rect.x = Math.Max(0, Math.Min(rect.x, ScreenWidth - rect.width));
            rect.y = Math.Max(0, Math.Min(rect.y, ScreenHeight - rect.height));

            rect = GUILayout.Window(windowId, rect, identifier =>
            {
                _selectedIndex = GUILayout.SelectionGrid(-1, items, 1, UiHover.YellowOnHover);
                if (GUI.changed) _isActive = false;

            }, "", style);

            if (Event.current.type == EventType.MouseDown && !rect.Contains(Event.current.mousePosition)) owner = null;
        }

        public static int SelectBox(int selectedIdx, string[] entries, object caller, bool expandWidth = true)
        {
            if (entries.Length == 0) return 0;
            if (entries.Length == 1)
            {
                GUILayout.Label(entries[0]);
                return 0;
            }

            if (selectedIdx >= entries.Length) selectedIdx = entries.Length - 1;

            if (owner == caller && !_isActive)
            {
                owner = null;
                selectedIdx = _selectedIndex;
                GUI.changed = true;
            }

            bool guiChanged = GUI.changed;
            if (GUILayout.Button("↓ " + entries[selectedIdx] + " ↓", GUILayout.ExpandWidth(expandWidth)))
            {
                GUI.changed = guiChanged;
                owner = caller;
                _isActive = true;
                items = entries;
                rect = new Rect(0, 0, 0, 0);
            }

            if (Event.current.type == EventType.Repaint && owner == caller && rect.height == 0)
            {
                rect = GUILayoutUtility.GetLastRect();
                Vector2 mousePos = Input.mousePosition;
                mousePos.y = Screen.height - mousePos.y;
                Vector2 clippedMousePos = Event.current.mousePosition;
                rect.x = (rect.x + mousePos.x) / Scale - clippedMousePos.x;
                rect.y = (rect.y + mousePos.y) / Scale - clippedMousePos.y;
            }

            return selectedIdx;
        }
    }

    public class UiHover {
        private static GUIStyle _onHover;
        public static GUIStyle YellowOnHover
        {
            get
            {
                if (_onHover != null) return _onHover;

                _onHover = new GUIStyle(GUI.skin.label) { hover = { textColor = Color.yellow } };
                var t = new Texture2D(1, 1);
                t.SetPixel(0, 0, new Color(0, 0, 0, 0));
                t.Apply();
                _onHover.hover.background = t;

                return _onHover;
            }
        }
    }
}
