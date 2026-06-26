using UnityEngine;
using Blackbird.Guidance;
using KSP.UI.Screens;
using Blackbird.Modules;
using Blackbird.Rendezvous;
using Blackbird.Docking;

namespace Blackbird
{
    [KSPAddon(KSPAddon.Startup.FlightAndEditor, false)]
    public sealed class BlackBird : MonoBehaviour
    {
        private Rect _windowRect = new Rect(200, 200, 350, 200);
        private Vessel _flyByWireVessel;

        // launch plan and guidance
        private readonly LaunchHandler _launchHandler = new LaunchHandler();

        private bool _showWindow = false;
        private ApplicationLauncherButton _toolbarButton;
        private Texture2D _toolbarIcon;
        private bool _toolbarIconOwned;

        private SharedState _bbState = new SharedState();

        private readonly RendezvousHandler _rendezvousHandler = new RendezvousHandler();
        private readonly DockingHandler _dockingHandler = new DockingHandler();

        private readonly Planner _planner = new Planner();
        private readonly GuidanceComputer _guidanceComputer = new GuidanceComputer();
        private readonly RendezvousComputer _rendezvousComputer = new RendezvousComputer();
        private readonly DockingComputer _dockingComputer = new DockingComputer();

        public void Start()
        {
            Debug.Log("[BlackBird] Loaded");
            // loading bb state globals
            _bbState.Init();

            _planner.Init(_launchHandler, _bbState);
            _guidanceComputer.Init(_launchHandler, _bbState);
            _rendezvousComputer.Init(_rendezvousHandler, _bbState);
            _dockingComputer.Init(_dockingHandler, _bbState);
            GameEvents.onGUIApplicationLauncherReady.Add(AddToolbarButton);
            GameEvents.onGUIApplicationLauncherDestroyed.Add(RemoveToolbarButton);

            // On revert-to-launch the launcher is often already ready, so onGUIApplicationLauncherReady
            // never fires for this fresh instance and the button goes missing. Add it directly when the
            // launcher is already up (AddToolbarButton is idempotent).
            if (ApplicationLauncher.Ready) AddToolbarButton();

            // pass state to all sub-modules
            _launchHandler.Init(_bbState);
            _rendezvousHandler.Init(_bbState);
            _dockingHandler.Init(_bbState);
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
            // "Guidance running" mirror for the UI; control arbitration is via ActiveModule, not this flag.
            _bbState.GuidanceEnabled = _launchHandler.State == LaunchGuidanceState.GuidingAscent;

            ITargetable target = FlightGlobals.fetch != null ? FlightGlobals.fetch.VesselTarget : null;
            // A targeted docking port is a ModuleDockingNode, not a Vessel; resolve to its vessel so the
            // rendezvous stages still get a target (the docking stage reads the port itself from the target).
            Vessel targetVessel = target as Vessel;
            if (targetVessel == null && target is ModuleDockingNode targetNode) targetVessel = targetNode.vessel;
            _rendezvousHandler.Update(activeVessel, targetVessel);
            _dockingHandler.Update(activeVessel);
        }
        private void OnFlyByWire(FlightCtrlState state)
        {
            if (_flyByWireVessel == null) return;

            _launchHandler.ApplyFlightControls(state, _flyByWireVessel);
            _rendezvousHandler.ApplyFlightControls(state, _flyByWireVessel);
            _dockingHandler.ApplyFlightControls(state, _flyByWireVessel);
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
            _rendezvousComputer.Draw();
            _dockingComputer.Draw();

            // Deferred dropdown popups (e.g. the rendezvous planner select) draw their own window, so they
            // must be rendered at the top level — after the panels — not nested inside a panel window.
            Blackbird.Helpers.Dropdown.DrawGUI();
        }

        private void DrawMainMenu(int _windowId)
        {
            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), " "))
            {
                _showWindow = false;
                _toolbarButton?.SetFalse(false);
            }

            if (FlightGlobals.ActiveVessel == null) return;

            DrawModuleToggles();
            GUI.DragWindow();
        }

        private void DrawModuleToggles()
        {
            GUILayout.Space(10);
            GUIStyle normal = GUI.skin.toggle;
            GUIStyle selected = new GUIStyle(normal) { normal = { textColor = Color.green }, onNormal = { textColor = Color.green } };
            _bbState.PlannerVisible = GUILayout.Toggle(_bbState.PlannerVisible, "Flight Planner", _bbState.ActiveModule == BlackbirdModule.Planner ? selected : normal);
            GUILayout.Space(5);
            _bbState.GuidanceVisible = GUILayout.Toggle(_bbState.GuidanceVisible, "Guidance Computer", _bbState.ActiveModule == BlackbirdModule.LaunchGuidance ? selected : normal);
            GUILayout.Space(5);
            _bbState.RendezvousVisible = GUILayout.Toggle(_bbState.RendezvousVisible, "Rendezvous Computer", _bbState.ActiveModule == BlackbirdModule.Rendezvous ? selected : normal);
            GUILayout.Space(5);
            _bbState.DockingVisible = GUILayout.Toggle(_bbState.DockingVisible, "Docking Computer", _bbState.ActiveModule == BlackbirdModule.Docking ? selected : normal);
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
