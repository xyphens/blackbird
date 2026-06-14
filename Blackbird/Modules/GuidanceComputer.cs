using UnityEngine;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private static readonly int WindowId = "Blackbird.GuidanceComputer".GetHashCode();
        private Rect _windowRect = new Rect(560, 620, 380, 300);
        public bool IsVisible { get; set; }

        public void Toggle() => IsVisible = !IsVisible;

        public void Draw()
        {
            if (!IsVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
        }

        private void DrawContents(int _windowId)
        {
            GUILayout.Label("[Guidance Computer]");
            GUI.DragWindow();
        }
    }
}
