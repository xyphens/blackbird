using Blackbird.Docking;
using UnityEngine;

namespace Blackbird.Modules
{
    // Operator panel for the docking module: live readouts (target, distance, closing rate, RCS fuel,
    // guidance status) and a cross of RCS fine-tuning buttons (translate / slew / rotate / kill), plus the
    // Run-guidance / Assume-control toggle. Manual buttons are RepeatButtons: held = active. Each draw rebuilds
    // the combined held-button state and hands it to the handler, which actuates it on the fly-by-wire pass.
    public sealed class DockingComputer
    {
        private SharedState bbState;
        private static readonly int WindowId = "Blackbird.DockingComputer".GetHashCode();
        private Rect _windowRect = new Rect(600, 200, 320, 430);

        private const float BtnW = 60f;
        private const float BtnH = 34f;

        private DockingHandler _handler;

        private static readonly DockingSteps[] _dockingStepDisplay =
        {
            DockingSteps.WrongSideBackingUp, DockingSteps.WrongSideLateral, DockingSteps.WrongSideSwitchSides,
            DockingSteps.BackingUp, DockingSteps.MovingToStart, DockingSteps.Docking
        };

        public void Init(DockingHandler handler, SharedState s)
        {
            _handler = handler;
            bbState = s;
        }

        public void Draw()
        {
            if (bbState == null || !bbState.DockingVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Docking Computer");
        }
        private void DrawContents(int _)
        {
            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), " ")) bbState.DockingVisible = false;

            if (_handler == null)
            {
                GUILayout.Label("Docking unavailable");
                GUI.DragWindow();
                return;
            }

            // --- readouts ---
            GUILayout.Label(_handler.HasTarget
                ? $"Target: {_handler.TargetName} ({_handler.TargetPortName})"
                : "Target: No target selected");
            GUILayout.Label($"Docking status: {DockingStatusText()}");
            GUILayout.Label($"Docking port distance: {FormatDistance(_handler.PortDistanceMeters)}");
            GUILayout.Label($"Rel velocity: {FormatClosing(_handler.ClosingRateSigned)}");
            GUILayout.Label($"RCS fuel available: {FormatPercent(_handler.RcsFuelPercent)}");
            GUILayout.Label($"Guidance status: {_handler.GuidanceStatus}");

            if (bbState.DockingMode == DockingControlMode.Guidance && _handler.DockingStep != DockingSteps.Off && _handler.DockingStep != DockingSteps.ClosingRange)
            {
                StepGate gate = _handler.CurrentGate;
                double oe = _handler.OrientationErrorDeg;
                GUILayout.Label($"  waiting on {gate.Label}: {gate.Current:F1} {(gate.Rising ? "→ > " : "→ < ")}{gate.Target:F1} m");
                GUILayout.Label($"  axial {_handler.AxialSepMeters:F1} m   lateral {_handler.LateralSepMeters:F1} m   facing {(double.IsNaN(oe) ? "--" : oe.ToString("F1") + "°")} off");

                // full step table: each step's gate against the current geometry; ▶ marks the active step.
                foreach (DockingSteps s in _dockingStepDisplay)
                {
                    StepGate g = _handler.GateFor(s);
                    GUILayout.Label($"    {(s == _handler.DockingStep ? "▶" : " ")} {s}: {g.Current:F1} {(g.Rising ? ">" : "<")} {g.Target:F1} {(g.Met ? "OK" : "")}");
                }

            }

            GUILayout.Space(8);

            DrawControlGrid();

            GUILayout.Space(6);

            // --- autopilot toggle buttons (see the state logic in the header of DockingHandler) ---
            bool alreadyRunning = bbState.DockingMode == DockingControlMode.Guidance;

            bool canRun = _handler.HasTarget && !alreadyRunning && bbState.CanClaimControl(BlackbirdModule.Docking);

            if (!canRun && !alreadyRunning)
            {
                string cantRunReason = "unavailable";
                if (!_handler.HasTarget)
                    cantRunReason = "no target selected";
                else if (bbState.ActiveModule == BlackbirdModule.LaunchGuidance)
                    cantRunReason = "ascent guidance is running";
                else if (bbState.ActiveModule == BlackbirdModule.Rendezvous)
                    cantRunReason = "rendezvous in progress";

                GUILayout.Label($"Docking unavailable: {cantRunReason}");
            }

            GUILayout.BeginHorizontal();
            GUI.enabled = canRun;
            if (GUILayout.Button("Run Docking Guidance", GUILayout.Height(30))) _handler.RunDockingGuidance();
            // allow canceling/stopping if docking is enabled at all
            GUI.enabled = bbState.DockingEnabled == true;
            if (GUILayout.Button("Stop Docking Guidance", GUILayout.Height(30))) _handler.StopDockingGuidance();
            if (GUILayout.Button("Assume Control", GUILayout.Height(30))) _handler.AssumeControl();
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }

        // The RCS fine-tuning cross. Rows 1 & 3 are indented one button so their three buttons sit over the
        // middle three of row 2. Translation/rotation/kill are RepeatButtons (held = active); nose-rotation
        // buttons are disabled while "keep pointed at target" is on; everything is disabled while guidance runs.
        private void DrawControlGrid()
        {
            bool manualEnabled = bbState.DockingMode != DockingControlMode.Guidance;
            bool noseEnabled = manualEnabled && !_handler.KeepPointed;

            var translate = Vector3.zero;   // x = right, y = dorsal-up, z = nose-forward
            var rotate = Vector2.zero;      // x = pitch, y = yaw
            bool kill = false;

            // Row 1: [nose left] [slew up] [nose right]
            GUILayout.BeginHorizontal();
            GUILayout.Space(BtnW);
            GUI.enabled = noseEnabled;
            if (Held("Nose ◄")) rotate.y -= 1f;     // yaw left
            GUI.enabled = manualEnabled;
            if (Held("Up")) translate.y += 1f;            // slew up
            GUI.enabled = noseEnabled;
            if (Held("Nose ►")) rotate.y += 1f;      // yaw right
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            // Row 2: [slew left] [throttle fwd] [kill] [throttle back] [slew right]
            GUILayout.BeginHorizontal();
            GUI.enabled = manualEnabled;
            if (Held("Left")) translate.x -= 1f;          // slew left
            if (Held("+")) translate.z += 1f;             // throttle forward (toward port)
            if (Held("KILL")) kill = true;                // null velocity + roll
            if (Held("−")) translate.z -= 1f;        // throttle back
            if (Held("Right")) translate.x += 1f;         // slew right
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            // Row 3: [nose up] [slew down] [nose down]
            GUILayout.BeginHorizontal();
            GUILayout.Space(BtnW);
            GUI.enabled = noseEnabled;
            if (Held("Nose ▲")) rotate.x += 1f;      // pitch up
            GUI.enabled = manualEnabled;
            if (Held("Down")) translate.y -= 1f;          // slew down
            GUI.enabled = noseEnabled;
            if (Held("Nose ▼")) rotate.x -= 1f;      // pitch down
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            _handler.KeepPointed = GUILayout.Toggle(_handler.KeepPointed, " Keep pointed at target");

            // Reset Orientation: roll/point the craft to a known attitude ("real up") so the craft-local
            // translation buttons become predictable. Latches until aligned; click again to cancel.
            GUI.enabled = manualEnabled;
            if (GUILayout.Button(_handler.ResettingOrientation ? "Resetting orientation..." : "Reset Orientation"))
                _handler.ResetOrientation();
            GUI.enabled = true;

            // Hand the combined held-button state to the handler (only meaningful in manual modes).
            if (manualEnabled) _handler.SetManualInput(translate, rotate, kill);
        }

        private static bool Held(string label) =>
            GUILayout.RepeatButton(label, GUILayout.Width(BtnW), GUILayout.Height(BtnH));

        private string DockingStatusText() => bbState.DockingMode == DockingControlMode.Guidance ? _handler.DockingStep.ToString() : bbState.DockingMode.ToString();

        private static string FormatDistance(double meters)
        {
            if (double.IsNaN(meters)) return "n/a";
            return meters >= 1000.0 ? $"{meters / 1000.0:F2} km" : $"{meters:F1} m";
        }

        private static string FormatClosing(double mps)
        {
            if (double.IsNaN(mps)) return "n/a";
            return $"{mps:+0.00;-0.00;0.00} m/s";   // + getting closer, - moving away
        }

        private static string FormatPercent(double pct)
        {
            return double.IsNaN(pct) ? "n/a" : $"{pct:F0}%";
        }
    }
}
