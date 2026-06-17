using Blackbird.Rendezvous;
using UnityEngine;

namespace Blackbird.Modules
{
    // Minimal operator panel for the terminal-rendezvous executor (Step 4b). Shows the target, the
    // current phase/stage, the measured relative state, and the cached intercept plan, and exposes the
    // user gates (Engage, Arm, Trigger, Abort, Reset). Step 9 expands this into the full staged panel;
    // for now it is enough to drive and observe an in-game intercept burn.
    public sealed class RendezvousComputer
    {
        private static readonly int WindowId = "Blackbird.RendezvousComputer".GetHashCode();
        private Rect _windowRect = new Rect(950, 200, 340, 340);

        private RendezvousHandler _handler;
        public bool IsVisible { get; set; }

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

            bool engaged = GUILayout.Toggle(_handler.Engaged, "Engage rendezvous autopilot");
            if (engaged != _handler.Engaged)
            {
                if (engaged) _handler.Engage(); else _handler.Disengage();
            }

            if (_handler.Target == null)
            {
                GUILayout.Label("Select a target vessel.");
                GUI.DragWindow();
                return;
            }

            GUILayout.Label($"Target: {_handler.Target.vesselName}");
            GUILayout.Label($"Phase: {_handler.Phase}    Stage: {_handler.Stage}");

            if (_handler.HasRelative)
            {
                GUILayout.Label($"Range: {_handler.Relative.Range / 1000.0:F2} km");
                GUILayout.Label($"Rel speed: {_handler.Relative.RelativeVelocityWorld.magnitude:F1} m/s   "
                              + $"range rate: {_handler.Relative.RangeRate:F1} m/s");
            }

            // Live closest approach (CA) = the minimum predicted chaser-target separation from the
            // current state, recomputed continuously so it can be watched collapse during a burn.
            if (!double.IsNaN(_handler.LiveClosestApproachMeters)
                && !double.IsInfinity(_handler.LiveClosestApproachMeters))
            {
                GUILayout.Label($"Closest approach: {_handler.LiveClosestApproachMeters / 1000.0:F2} km "
                              + $"in {_handler.LiveTimeToClosestApproachSeconds:F0} s");
            }

            if (_handler.HasInterceptPlan)
            {
                InterceptSolution plan = _handler.InterceptPlan;
                GUILayout.Label($"Plan ΔV: {plan.DeltaVMagnitude:F1} m/s   tof: {plan.TimeOfFlight:F0} s");
                GUILayout.Label($"Plan predicted CA: {plan.PredictedClosestApproach:F0} m");
            }

            if (_handler.HasCommand)
                GUILayout.Label($"Status: {_handler.Command.Status}");

            GUILayout.Space(8);
            GUILayout.BeginHorizontal();

            GUI.enabled = _handler.Phase == RendezvousPhase.Idle || _handler.Phase == RendezvousPhase.Coast;
            if (GUILayout.Button("Arm")) _handler.Arm();

            GUI.enabled = _handler.Phase == RendezvousPhase.Armed;
            if (GUILayout.Button("Trigger")) _handler.Trigger();

            GUI.enabled = true;
            if (GUILayout.Button("Abort")) _handler.Abort();
            if (GUILayout.Button("Reset")) _handler.ResetSequence();

            GUILayout.EndHorizontal();
            GUI.DragWindow();
        }
    }
}
