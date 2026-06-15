using UnityEngine;
using System;
using Blackbird;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;
using Blackbird.Guidance;
using Blackbird.Enums;
using Blackbird.Trajectory;
using KSP.UI.Screens;
using Blackbird.Modules;

namespace Blackbird
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class BlackBird : MonoBehaviour
    {
        private Rect _windowRect = new Rect(200, 200, 350, 800);
        private Vessel _flyByWireVessel;

        // launch plan and guidance
        private readonly LaunchHandler _launchHandler = new LaunchHandler();

        private bool _showWindow = false;
        private ApplicationLauncherButton _toolbarButton;
        private Texture2D _toolbarIcon;
        private bool _toolbarIconOwned;

        private readonly Planner _planner = new Planner();
        private readonly GuidanceComputer _guidanceComputer = new GuidanceComputer();

        public void Start()
        {
            Debug.Log("[BlackBird] Loaded");
            _planner.Initialize(_launchHandler);
            GameEvents.onGUIApplicationLauncherReady.Add(AddToolbarButton);
            GameEvents.onGUIApplicationLauncherDestroyed.Add(RemoveToolbarButton);
        }

        public void Update()
        {
            Vessel activeVessel = FlightGlobals.ActiveVessel;

            if (_flyByWireVessel != activeVessel)
            {
                if (_flyByWireVessel != null) _flyByWireVessel.OnFlyByWire -= OnFlyByWire;

                _flyByWireVessel = activeVessel;

                if (_flyByWireVessel != null) _flyByWireVessel.OnFlyByWire += OnFlyByWire;
            }

            _launchHandler.Update(activeVessel);
        }
        private void OnFlyByWire(FlightCtrlState state)
        {
            if (_launchHandler == null || _flyByWireVessel == null) return;

            _launchHandler.ApplyFlightControls(state, _flyByWireVessel);
        }

        public void OnDestroy()
        {
            if (_flyByWireVessel != null)
            {
                _flyByWireVessel.OnFlyByWire -= OnFlyByWire;
                _flyByWireVessel = null;
            }
            GameEvents.onGUIApplicationLauncherReady.Remove(AddToolbarButton);
            GameEvents.onGUIApplicationLauncherDestroyed.Remove(RemoveToolbarButton);
            RemoveToolbarButton();
        }

        private void OnGUI()
        {
            if (!_showWindow) return;
            _windowRect = GUILayout.Window(
                12345,
                _windowRect,
                DrawMainMenu,
                "BlackBird");
            _planner.Draw();
            _guidanceComputer.Draw();
        }

        private void DrawMainMenu(int _windowId)
        {
            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), "x"))
            {
                _showWindow = false;
                _toolbarButton?.SetFalse(false);
            }

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null) return;

            DrawModuleToggles();
            GUI.DragWindow();
        }

        private void DrawModuleToggles()
        {
            GUILayout.BeginHorizontal();
            _planner.IsVisible = GUILayout.Toggle(_planner.IsVisible, "Planner");
            _guidanceComputer.IsVisible = GUILayout.Toggle(_guidanceComputer.IsVisible, "Guidance Computer");
            GUILayout.EndHorizontal();
        }

        private void AddToolbarButton()
        {
            if (_toolbarButton != null) return;

            Texture2D dbIcon = GameDatabase.Instance.GetTexture("BlackBird/Textures/toolbar_icon", false);
            if (dbIcon != null)
            {
                _toolbarIcon = dbIcon;
                _toolbarIconOwned = false;
            }
            else
            {
                _toolbarIcon = CreateToolbarIcon();
                _toolbarIconOwned = true;
            }

            _toolbarButton = ApplicationLauncher.Instance.AddModApplication(
                () => _showWindow = true,
                () => _showWindow = false,
                null, null, null, null,
                ApplicationLauncher.AppScenes.FLIGHT,
                _toolbarIcon);
        }

        private void RemoveToolbarButton()
        {
            if (_toolbarButton != null)
            {
                if (ApplicationLauncher.Instance != null)
                    ApplicationLauncher.Instance.RemoveModApplication(_toolbarButton);
                _toolbarButton = null;
            }
            if (_toolbarIconOwned && _toolbarIcon != null)
            {
                Destroy(_toolbarIcon);
            }
            _toolbarIcon = null;
            _toolbarIconOwned = false;
        }

        private static Texture2D CreateToolbarIcon()
        {
            var tex = new Texture2D(38, 38, TextureFormat.RGBA32, false);
            var pixels = new Color[38 * 38];
            for (int i = 0; i < pixels.Length; i++)
                pixels[i] = new Color(0.18f, 0.48f, 0.87f, 1f);
            tex.SetPixels(pixels);
            tex.Apply();
            return tex;
        }
    }
}
