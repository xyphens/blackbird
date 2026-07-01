using Blackbird.Docking;
using Blackbird.Guidance;
using Blackbird.Mathematics;
using Blackbird.Rendezvous;
using System;
using UnityEngine;

namespace Blackbird.Modules
{
    // Operator panel for the terminal-rendezvous executor: a checklist (Intercept -> Match Velocity ->
    // Close Approach) with the active step's description and a status line stating whether the autopilot
    // is waiting on the user (Execute) or on an event (coast to closest approach).
    public sealed class RendezvousComputer
    {
        private SharedState bbState;
        private static readonly int WindowId = "Blackbird.RendezvousComputer".GetHashCode();
        private Rect _windowRect = new Rect(950, 200, 360, 380);

        // Above this, a single-rev intercept is almost certainly the wrong tool (too far / wrong phase).
        private const double HighDeltaVWarnMetersPerSecond = 300.0;

        private RendezvousHandler _handler;

        // Close-approach "match velocities at X m" option (see ApplyCloseStandoff).
        private bool _matchAtEnabled;
        private string _matchAtMetersText = "100";

        // Close-approach closing-speed tuning (applied live; see ApplyCloseApproachParams). Raise the max
        // speed to close a long-range gap as a few large burns instead of a slow capped crawl.
        private string _closeGainText = "0.2";
        private string _closeMaxSpeedText = "5";

        // Extra warp-stop lead (seconds) before the closest approach; floors the auto (slew + half the burn).
        private string _warpLeadText = "30";

        private InterceptMethod _lastAlgorithm;

        // Lazily-built red label style for warnings (e.g. a burn that can't settle to ignite).
        private GUIStyle _warnStyle;

        public void Init(RendezvousHandler handler, SharedState s)
        {
            _handler = handler;
            bbState = s;
        }

        public void Draw()
        {
            if (bbState == null || !bbState.RendezvousVisible)
            {
                // Window closed: stop preview/monitor only. A running stage keeps control (it owns ActiveModule).
                _handler?.ToggleEngage(false);
                return;
            }

            // Engaged (preview + monitor) while the window is open with a target; control authority is separate.
            _handler.ToggleEngage(_handler.Target != null);
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Rendezvous Computer");
        }

        private void DrawContents(int _)
        {

            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), " ")) bbState.RendezvousVisible = false;

            if (FlightGlobals.ActiveVessel == null) return;

            if (_handler == null || _handler.Target == null || !Universe.IsInSpace(FlightGlobals.ActiveVessel?.altitude ?? 0))
            {
                string strTarget = _handler?.Target?.name ?? "no target";

                GUILayout.Label($"Rendezvous computer unavailable (target: {strTarget}, altitude: {FormatDistance(FlightGlobals.ActiveVessel.altitude)} m / {FormatDistance(FlightGlobals.currentMainBody.atmosphereDepth)} m)");
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

            Orbit v = FlightGlobals.ActiveVessel.orbit, t = _handler.Target.orbit;

            if (v.referenceBody == t.referenceBody)
            {
                double relInc = OrbitMath.GetRelativeInclination(FlightGlobals.ActiveVessel, _handler.Target);
                GUILayout.Label($"Rel. Inclination: {relInc:F2}°");
                GUILayout.Label($"Inclination: {v.inclination:F2}° vs {t.inclination:F2}°");
                GUILayout.Label($"RAAN (LAN): {v.LAN:F2}° vs {t.LAN:F2}°");
            } else
            {
                GUILayout.Label("Target is in different reference body");
            }
 
            if (MathHelpers.IsFinite(_handler.LiveClosestApproachMeters))
            {
                GUILayout.Label($"Closest approach: {FormatDistance(_handler.LiveClosestApproachMeters)} "
                              + $"in {FormatTime(_handler.LiveTimeToClosestApproachSeconds)}");

                // Closest approach is now AND we're separating ⇒ no natural approach within an orbit;
                // the pair won't close on its own. Tell the user a corrective burn is needed.
                bool separating = _handler.HasRelative && _handler.Relative.RangeRate > 0.0;
                if (separating && _handler.LiveTimeToClosestApproachSeconds < 1.0)
                {
                    GUILayout.Label("Status: separating");
                }
            }
            else if (_handler.HasRelative)
            {
                // No upcoming pass within the horizon (holding / velocity-matched, or a single monotonic close):
                // the closest approach is the current range, now. Keep the line visible regardless of state.
                GUILayout.Label($"Closest approach: {FormatDistance(_handler.Relative.Range)} now (no upcoming pass)");
            }

            // Intercept plan preview (computed whenever idle/coast, before any method is chosen).
            if (_handler.HasInterceptPlan
                && (bbState.InterceptPhase == InterceptPhase.Idle || bbState.InterceptPhase == InterceptPhase.Coast))
            {
                
                GUILayout.Label($"Plan: ΔV {bbState.InterceptSolution.DeltaVMagnitude:F1} m/s for {bbState.InterceptSolution.PredictedClosestApproach:F0} m encounter (arriving in {FormatTime(bbState.InterceptSolution.TimeOfFlight)})");

                // A single-rev intercept across a big phase gap is legitimately huge (can even escape).
                // Warn before the user commits — the cheap move from a close flyby is Match Velocity.
                if (bbState.InterceptSolution.DeltaVMagnitude > HighDeltaVWarnMetersPerSecond)
                {
                    GUILayout.Label("  WARNING: very large ΔV - you are likely too far / wrong phase for a direct intercept.  Wait for a closer pass, or match velocity instead.");
                }
            }

            bbState._interceptMethod = Helpers.Dropdown.SelectBox(bbState._interceptMethod, bbState.InterceptMethods, this);
            // clear prior plan and calc the targeted plan type
            if (bbState.InterceptMethod != _lastAlgorithm)
            {
                _handler.ResetSequence();
                _lastAlgorithm = bbState.InterceptMethod;
                _handler.GenerateNewInterceptPlan(FlightGlobals.ActiveVessel, _handler.Target, bbState.InterceptMethod);
            }

            if (GUILayout.Button($"Compute {(bbState.InterceptMethod == InterceptMethod.Hohmann ? "Hohmann transfer" : "plan")}"))
            {
                _handler.GenerateNewInterceptPlan(FlightGlobals.ActiveVessel, _handler.Target, bbState.InterceptMethod);
            }

            DrawInterceptCandidates();

            GUILayout.Space(4);

            // --- instruction / status: is it on the user or on an event? ---
            GUILayout.Label($"Status: {GetInstruction()}");

            GUILayout.Space(8);
            

            // --- action buttons: any stage can be executed when rendezvous can hold control and isn't mid-burn ---
            bool canExecute = bbState.CanClaimControl(BlackbirdModule.Rendezvous)
                              && bbState.InterceptPhase != InterceptPhase.Executing
                              && bbState.InterceptPhase != InterceptPhase.Aborted;
            if (bbState.InterceptPhase == InterceptPhase.Executing)
            {
                GUILayout.Label($"Executing: {StageName(bbState.RendezvousMethod)}...");

                // Burn can't ignite because the craft won't hold the vector: warn in red (no force-fire).
                if (_handler.SettleStalled)
                {
                    if (_warnStyle == null)
                        _warnStyle = new GUIStyle(GUI.skin.label) { normal = { textColor = Color.red }, wordWrap = true };
                    GUILayout.Label($"Error: Burn stalled — {_handler.SettleStallReason}", _warnStyle);
                }
            }

            GUI.enabled = canExecute;
            if (GUILayout.Button("Execute: Intercept"))
            {
                bbState.RendezvousMethod = RendezvousMethod.Intercept;
                _handler.Execute();
            }

            // Warp to closest approach (auto-stops short; cancelled if a burn starts).

            if (_handler.Warping)
            {
                if (GUILayout.Button("Stop Warp")) _handler.StopWarp();
            }
            else if (bbState.InterceptMethod == InterceptMethod.Hohmann
                     && bbState.RendezvousMethod == RendezvousMethod.Intercept
                     && (_handler.CoastingToIgnition
                         || (bbState.InterceptPhase == InterceptPhase.Idle && _handler.HasInterceptPlan)))
            {
                // Hohmann ignites at a future departure UT, so warp to that window: the frozen ignition while
                // coasting (post-Execute), otherwise the previewed plan's.

                double ignitionUt = _handler.CoastingToIgnition
                    ? _handler.PlannedIgnitionUt
                    : bbState.InterceptSolution.IgnitionUt;
                double dtToIgnition = ignitionUt - Planetarium.GetUniversalTime();

                GUI.enabled = (bbState.InterceptPhase != InterceptPhase.Executing || _handler.CoastingToIgnition)
                              && dtToIgnition > 10.0;
                if (GUILayout.Button($"Warp to transfer ignition ({FormatTime(dtToIgnition)})"))
                    _handler.WarpToIgnition(ignitionUt);
                GUI.enabled = true;
            }
            else
            {
                GUI.enabled = bbState.InterceptPhase != InterceptPhase.Executing
                              && MathHelpers.IsFinite(_handler.LiveTimeToClosestApproachSeconds)
                              && _handler.LiveTimeToClosestApproachSeconds > 10.0;
                if (GUILayout.Button("Warp to Next Closest Approach")) _handler.WarpToClosestApproach();
                GUI.enabled = true;
            }

            GUI.enabled = canExecute;
            if (GUILayout.Button("Execute: Match Velocity"))
            {
                bbState.RendezvousMethod = RendezvousMethod.MatchVelocity;
                ApplyCloseStandoff();
                _handler.Execute(RendezvousMethod.MatchVelocity);
            }

            if (GUILayout.Button("Execute: Final Approach"))
            {
                bbState.RendezvousMethod = RendezvousMethod.FinalApproach;
                ApplyCloseStandoff();
                _handler.Execute();
            }

            GUI.enabled = true;

            // Close-approach park distance: when checked, close in to (and velocity-match at) the input
            // distance instead of the default ~100 m. Always editable so it can be set before executing.
            GUILayout.BeginHorizontal();
            _matchAtEnabled = GUILayout.Toggle(_matchAtEnabled, " Match velocities at:", GUILayout.Width(160));
            _matchAtMetersText = GUILayout.TextField(_matchAtMetersText, GUILayout.Width(60));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            // Close-approach closing-speed levers. Max speed is the big one for long-range gaps: raise it so
            // the stage accelerates, coasts, then auto-brakes instead of crawling at the cap. Applied live.
            GUILayout.BeginHorizontal();
            GUILayout.Label("Close gain (speed/m):", GUILayout.Width(160));
            _closeGainText = GUILayout.TextField(_closeGainText, GUILayout.Width(60));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Max closing speed:", GUILayout.Width(160));
            _closeMaxSpeedText = GUILayout.TextField(_closeMaxSpeedText, GUILayout.Width(60));
            GUILayout.Label("m/s");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Warp lead (0=auto):", GUILayout.Width(160));
            _warpLeadText = GUILayout.TextField(_warpLeadText, GUILayout.Width(60));
            GUILayout.Label("s");
            GUILayout.EndHorizontal();
            ApplyCloseApproachParams();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Abort")) _handler.Abort();
            if (GUILayout.Button("Reset")) _handler.ResetSequence();
            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }

        // Hohmann candidate windows: depart-time / ΔV / time-of-flight, one Choose button each. Picking copies
        // the window into bbState.InterceptSolution so Execute fires it. Empty for the single-phase method.
        private void DrawInterceptCandidates()
        {
            System.Collections.Generic.List<InterceptSolution> candidates = bbState.InterceptCandidates;
            if (candidates == null || candidates.Count == 0) return;

            GUILayout.Space(4);
            GUILayout.Label("Transfer windows (pick one):");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Depart in", GUILayout.Width(90));
            GUILayout.Label("ΔV", GUILayout.Width(75));
            GUILayout.Label("Arrive in", GUILayout.Width(90));
            GUILayout.Label("", GUILayout.Width(60));
            GUILayout.EndHorizontal();

            double now = Planetarium.GetUniversalTime();
            bool canChoose = bbState.InterceptPhase != InterceptPhase.Executing
                             && bbState.InterceptPhase != InterceptPhase.Aborted;

            for (int i = 0; i < candidates.Count; i++)
            {
                InterceptSolution c = candidates[i];
                GUILayout.BeginHorizontal();
                // Show the FULL transfer cost (dv1 + dv2), not just the departure burn — the arrival burn carries
                // any plane change, so a window can depart cheap (dv1) but cost far more total when off-plane.
                double shownDv = c.TotalDeltaVMagnitude > 0.0 ? c.TotalDeltaVMagnitude : c.DeltaVMagnitude;
                GUILayout.Label(FormatTime(c.IgnitionUt - now), GUILayout.Width(90));
                GUILayout.Label($"{shownDv:F1} m/s", GUILayout.Width(75));
                GUILayout.Label(FormatTime(c.TimeOfFlight), GUILayout.Width(90));

                bool isSelected = bbState.SelectedInterceptCandidateIndex == i;
                GUI.enabled = canChoose && !isSelected;
                if (GUILayout.Button(isSelected ? "Active" : "Choose", GUILayout.Width(60)))
                    _handler.SelectInterceptCandidate(i);
                GUI.enabled = true;
                GUILayout.EndHorizontal();
            }
        }

        // The one line that tells the user what is happening and what to do next.
        private string GetInstruction()
        {
            if (!bbState.RendezvousEnabled) return "inactive";

            switch (bbState.InterceptPhase)
            {
                case InterceptPhase.Idle: return "awaiting instructions";
                case InterceptPhase.Executing:
                    if (_handler.HasCommand &&
                        (_handler.Stabilizing || _handler.Orienting))
                    {
                        AttitudeControl.WorldDirectionToHeadingPitch(
                            FlightGlobals.ActiveVessel, _handler.Command.ThrustDirection,
                            out double hdg, out double pit);
                        string verb = _handler.Stabilizing ? "Stabilizing" : "Orienting";
                        return $"{verb}: point to {hdg:F0}° / {pit:F0}° "
                             + $"({_handler.AlignmentErrorDeg:F3}° remaining)...";
                    }
                    return _handler.HasCommand ? _handler.Command.Status : "Executing...";

                case InterceptPhase.Coast:
                    if (bbState.RendezvousMethod == RendezvousMethod.MatchVelocity)
                        return "Intercept done. Coasting toward closest approach "
                             + $"in {FormatTime(_handler.LiveTimeToClosestApproachSeconds)}. "
                             + "Execute Match Velocity as you near it.";
                    if (bbState.RendezvousMethod == RendezvousMethod.FinalApproach)
                        return "Velocities matched.";
                    return $"Stage done";

                case InterceptPhase.Complete: return "completed";
                case InterceptPhase.Aborted: return "aborted";
                default: return string.Empty;
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

        // Apply the close-approach closing-speed levers live (each draw). Invalid/empty text keeps the last
        // good value. Raising the max speed turns a long-range crawl into a few large burns + coast + brake.
        private void ApplyCloseApproachParams()
        {
            if (double.TryParse(_closeGainText, out double gain) && !double.IsNaN(gain) && gain > 0.0)
                _handler.CloseApproachGain = gain;
            if (double.TryParse(_closeMaxSpeedText, out double maxSpeed) && !double.IsNaN(maxSpeed) && maxSpeed > 0.0)
                _handler.CloseApproachMaxSpeedMetersPerSecond = maxSpeed;
            if (double.TryParse(_warpLeadText, out double warpLead) && !double.IsNaN(warpLead) && warpLead >= 0.0)
                _handler.WarpLeadInputSeconds = warpLead;
        }

        private static string StageName(RendezvousMethod stage)
        {
            switch (stage)
            {
                case RendezvousMethod.Intercept:     return "Intercept";
                case RendezvousMethod.MatchVelocity: return "Match Velocity";
                default:                             return "Close Approach";
            }
        }

        private static string FormatTime(double seconds)
        {
            if (!MathHelpers.IsFinite(seconds)) return "--";
            if (seconds < 60.0) return $"{seconds:F0}s";
            if (seconds < 3600.0) return $"{seconds / 60.0:F1} min";
            return $"{seconds / 3600.0:F1} h";
        }

        // Distance with a unit that suits the magnitude: mm / m / km (so 900 m doesn't read "0.90 km").
        private static string FormatDistance(double meters)
        {
            if (!MathHelpers.IsFinite(meters)) return "--";
            double a = Math.Abs(meters);
            if (a < 1.0) return $"{meters * 1000.0:F0} mm";
            if (a < 1000.0) return $"{meters:F0} m";
            return $"{meters / 1000.0:F2} km";
        }

        // Speed with a unit that suits the magnitude: mm/s / m/s / km/s (so 0.1 m/s reads "100 mm/s").
        private static string FormatSpeed(double metersPerSecond)
        {
            if (!MathHelpers.IsFinite(metersPerSecond)) return "--";
            double a = Math.Abs(metersPerSecond);
            if (a < 1.0) return $"{metersPerSecond * 1000.0:F0} mm/s";
            if (a < 1000.0) return $"{metersPerSecond:F1} m/s";
            return $"{metersPerSecond / 1000.0:F2} km/s";
        }
    }
}
