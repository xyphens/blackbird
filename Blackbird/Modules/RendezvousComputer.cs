using System;
using Blackbird.Docking;
using Blackbird.Rendezvous;
using UnityEngine;

namespace Blackbird.Modules
{
    // Operator panel for the terminal-rendezvous executor. Lays the flow out as an explicit checklist
    // (Intercept -> Match Velocity -> Close Approach) with a description of the active step and a status
    // line that always states whether the autopilot is waiting on the USER (Execute) or on an EVENT
    // (coast to closest approach). One button per stage: Execute. Step 9 will refine; this is the
    // drivable operator UI for Steps 4-7.
    public sealed class RendezvousComputer
    {
        private static readonly int WindowId = "Blackbird.RendezvousComputer".GetHashCode();
        private Rect _windowRect = new Rect(950, 200, 360, 380);

        // Above this, a single-rev intercept is almost certainly the wrong tool (too far / wrong phase).
        private const double HighDeltaVWarnMetersPerSecond = 300.0;

        private RendezvousHandler _handler;
        public bool IsVisible { get; set; }

        // Close-approach "match velocities at X m" option (see ApplyCloseStandoff).
        private bool _matchAtEnabled;
        private string _matchAtMetersText = "100";

        public void Initialize(RendezvousHandler handler) => _handler = handler;
        public void Toggle() => IsVisible = !IsVisible;

        public void Draw()
        {
            if (!IsVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Rendezvous");
        }

        private void DrawContents(int _)
        {
            if (_handler == null)
            {
                GUILayout.Label("Rendezvous unavailable");
                GUI.DragWindow();
                return;
            }

            bool engaged = GUILayout.Toggle(_handler.Engaged, "Enable rendezvous autopilot");
            if (engaged != _handler.Engaged)
            {
                if (engaged) _handler.Engage(); else _handler.Disengage();
            }

            if (_handler.Target == null)
            {
                GUILayout.Label("Select a target vessel to begin.");
                GUI.DragWindow();
                return;
            }

            // --- live relative state (shown whether or not engaged) ---
            GUILayout.Label($"Target: {_handler.Target.vesselName}");
            if (_handler.HasRelative)
            {
                GUILayout.Label($"Range: {FormatDistance(_handler.Relative.Range)}   "
                              + $"rel speed: {FormatSpeed(_handler.Relative.RelativeVelocityWorld.magnitude)}");
            }
            if (IsFinite(_handler.LiveClosestApproachMeters))
            {
                GUILayout.Label($"Closest approach: {FormatDistance(_handler.LiveClosestApproachMeters)} "
                              + $"in {FormatTime(_handler.LiveTimeToClosestApproachSeconds)}");

                // Closest approach is now AND we're separating ⇒ no natural approach within an orbit;
                // the pair won't close on its own. Tell the user a corrective burn is needed.
                bool separating = _handler.HasRelative && _handler.Relative.RangeRate > 0.0;
                if (separating && _handler.LiveTimeToClosestApproachSeconds < 1.0)
                {
                    GUILayout.Label("  (separating - no approach this orbit; Execute Intercept to set one up)");
                }
            }

            GUILayout.Space(8);

            // --- stage checklist ---
            int stageIndex = (int)_handler.Stage;
            bool complete = _handler.Phase == RendezvousPhase.Complete;
            DrawStageRow(0, "1. Intercept", "Burn that puts your closest approach on the target.", stageIndex, complete);
            DrawStageRow(1, "2. Match Velocity", "At closest approach, cancel the relative velocity.", stageIndex, complete);
            DrawStageRow(2, "3. Close Approach", "Close to ~100 m, then hand control back.", stageIndex, complete);

            GUILayout.Space(6);

            // --- plan preview for the pending stage (only meaningful while Intercept is the next stage) ---
            if (_handler.HasInterceptPlan && _handler.Stage == RendezvousStage.Intercept
                && (_handler.Phase == RendezvousPhase.Idle || _handler.Phase == RendezvousPhase.Coast))
            {
                InterceptSolution plan = _handler.InterceptPlan;
                GUILayout.Label($"Plan: ΔV {plan.DeltaVMagnitude:F1} m/s  ->  CA {plan.PredictedClosestApproach:F0} m  "
                              + $"(arc {FormatTime(plan.TimeOfFlight)})");

                // A single-rev intercept across a big phase gap is legitimately huge (can even escape).
                // Warn before the user commits — the cheap move from a close flyby is Match Velocity.
                if (plan.DeltaVMagnitude > HighDeltaVWarnMetersPerSecond)
                {
                    GUILayout.Label("  WARNING: very large ΔV - you are likely too far / wrong phase for a");
                    GUILayout.Label("  direct intercept. Wait for a closer pass, or match velocity instead.");
                }
            }

            // --- instruction / status: is it on the user or on an event? ---
            GUILayout.Label(GetInstruction());

            GUILayout.Space(8);

            // --- action buttons: any stage can be executed at any time (out of order) ---
            // Match Velocity in particular must be reachable on demand to kill a dangerous closing rate.
            bool canExecute = _handler.Engaged
                              && _handler.Phase != RendezvousPhase.Executing
                              && _handler.Phase != RendezvousPhase.Aborted;
            if (_handler.Phase == RendezvousPhase.Executing)
            {
                GUILayout.Label($"Executing: {StageName(_handler.Stage)}...");
            }
            GUI.enabled = canExecute;
            if (GUILayout.Button("Execute: Intercept")) _handler.Execute(RendezvousStage.Intercept);
            if (GUILayout.Button("Execute: Match Velocity")) _handler.Execute(RendezvousStage.MatchVelocity);
            if (GUILayout.Button("Execute: Close Approach"))
            {
                ApplyCloseStandoff();
                _handler.Execute(RendezvousStage.CloseApproach);
            }

            // Docking: staged RCS gates (Approach -> Final -> Contact). One click runs the current leg; on
            // completion it coasts awaiting the next gate. Needs a targeted docking port + "Control From Here".
            DockingLeg nextLeg = _handler.Stage == RendezvousStage.Docking ? _handler.DockingLeg : DockingLeg.Approach;
            if (GUILayout.Button($"Execute: Dock ({nextLeg})")) _handler.ExecuteDocking();
            GUI.enabled = true;

            // Close-approach park distance: when checked, close in to (and velocity-match at) the input
            // distance instead of the default ~100 m. Always editable so it can be set before executing.
            GUILayout.BeginHorizontal();
            _matchAtEnabled = GUILayout.Toggle(_matchAtEnabled, " Match velocities at:", GUILayout.Width(160));
            _matchAtMetersText = GUILayout.TextField(_matchAtMetersText, GUILayout.Width(60));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            // Warp to closest approach (auto-stops short; cancelled if a burn starts).
            if (_handler.Warping)
            {
                if (GUILayout.Button("Stop Warp")) _handler.StopWarp();
            }
            else
            {
                GUI.enabled = _handler.Phase != RendezvousPhase.Executing
                              && IsFinite(_handler.LiveTimeToClosestApproachSeconds)
                              && _handler.LiveTimeToClosestApproachSeconds > 10.0;
                if (GUILayout.Button("Warp to Next Closest Approach")) _handler.WarpToClosestApproach();
                GUI.enabled = true;
            }

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Abort")) _handler.Abort();
            if (GUILayout.Button("Reset")) _handler.ResetSequence();
            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }

        private void DrawStageRow(int index, string title, string description, int currentIndex, bool complete)
        {
            bool done = complete || index < currentIndex;
            bool active = !complete && index == currentIndex;
            string mark = done ? "[x]" : active ? "[>]" : "[ ]";

            GUILayout.Label($"{mark} {title}");
            if (active) GUILayout.Label("        " + description);
        }

        // The one line that tells the user what is happening and what to do next.
        private string GetInstruction()
        {
            if (!_handler.Engaged)
                return "Enable the autopilot above to begin.";

            switch (_handler.Phase)
            {
                case RendezvousPhase.Idle:
                    return "Ready. Execute Intercept any time — the plan re-solves from your "
                         + "current position, so there is no window to miss.";

                case RendezvousPhase.Executing:
                    if (_handler.Stabilizing)
                        return $"Aligned ({_handler.AlignmentErrorDeg:F1} deg) - settling before burn...";
                    if (_handler.Orienting)
                        return $"Orienting to burn attitude ({_handler.AlignmentErrorDeg:F0} deg off)...";
                    return _handler.HasCommand ? _handler.Command.Status : "Executing...";

                case RendezvousPhase.Coast:
                    if (_handler.Stage == RendezvousStage.MatchVelocity)
                        return "Intercept done. Coasting toward closest approach "
                             + $"(in {FormatTime(_handler.LiveTimeToClosestApproachSeconds)}). "
                             + "Execute Match Velocity as you near it.";
                    if (_handler.Stage == RendezvousStage.CloseApproach)
                        return "Matched. Execute Close Approach to close in to ~100 m and hand back control.";
                    return $"Stage done. Execute {StageName(_handler.Stage)} when ready.";

                case RendezvousPhase.Complete:
                    return "Rendezvous complete - control returned to you.";

                case RendezvousPhase.Aborted:
                    return "Aborted. Reset to start over.";

                default:
                    return string.Empty;
            }
        }

        // Apply the "match velocities at X m" option to the executor before a close-approach run: when
        // enabled with a valid positive number, park/match at that distance; otherwise restore the default.
        private void ApplyCloseStandoff()
        {
            double meters;
            if (_matchAtEnabled
                && double.TryParse(_matchAtMetersText, out meters)
                && !double.IsNaN(meters) && meters > 0.0)
            {
                _handler.ParkingDistanceMeters = meters;
                _handler.AutoMatchVelocityDistance = true;
            }
            else
            {
                _handler.ParkingDistanceMeters = RendezvousHandler.CloseStandoffDefaultMeters;
                _handler.AutoMatchVelocityDistance = false;
            }
        }

        private static string StageName(RendezvousStage stage)
        {
            switch (stage)
            {
                case RendezvousStage.Intercept:     return "Intercept";
                case RendezvousStage.MatchVelocity: return "Match Velocity";
                case RendezvousStage.Docking:       return "Docking";
                default:                            return "Close Approach";
            }
        }

        private static string FormatTime(double seconds)
        {
            if (!IsFinite(seconds)) return "--";
            if (seconds < 60.0) return $"{seconds:F0}s";
            if (seconds < 3600.0) return $"{seconds / 60.0:F1} min";
            return $"{seconds / 3600.0:F1} h";
        }

        // Distance with a unit that suits the magnitude: mm / m / km (so 900 m doesn't read "0.90 km").
        private static string FormatDistance(double meters)
        {
            if (!IsFinite(meters)) return "--";
            double a = Math.Abs(meters);
            if (a < 1.0) return $"{meters * 1000.0:F0} mm";
            if (a < 1000.0) return $"{meters:F0} m";
            return $"{meters / 1000.0:F2} km";
        }

        // Speed with a unit that suits the magnitude: mm/s / m/s / km/s (so 0.1 m/s reads "100 mm/s").
        private static string FormatSpeed(double metersPerSecond)
        {
            if (!IsFinite(metersPerSecond)) return "--";
            double a = Math.Abs(metersPerSecond);
            if (a < 1.0) return $"{metersPerSecond * 1000.0:F0} mm/s";
            if (a < 1000.0) return $"{metersPerSecond:F1} m/s";
            return $"{metersPerSecond / 1000.0:F2} km/s";
        }

        private static bool IsFinite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);
    }
}
