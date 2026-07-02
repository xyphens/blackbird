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
        private RendezvousHandler _handler;
        private InterceptMethod _lastAlgorithm;

        // Lazily-built red label style for warnings (e.g. a burn that can't settle to ignite).
        private GUIStyle _warnStyle, _errorStyle;

        private static readonly int WindowId = "Blackbird.RendezvousComputer".GetHashCode();
        private Rect _windowRect = new Rect(950, 200, 360, 380);

        // Above this, a single-rev intercept is almost certainly the wrong tool (too far / wrong phase).
        // todo: remove - pointless warning (user can read dV)
        private const double HighDeltaVWarnMetersPerSecond = 300.0;

        private string _faError;

        public bool _approachParamsSet = false;

        // Final Approach - lock axes
        public bool LockedAxes = false;

        private double _warpLead = 30.0;
        private string WarpLead
        {
            get { return _warpLead.ToString("F0"); }
            set { if (double.TryParse(value, out double v)) _warpLead = v; }
        }

        // tells executor if it should auto-MV when finished
        private bool ParkingDistanceEnabled;
        private double _parkingDistance = 100.0;
        private double DefaultParkingDistance = 100.0;
        private string ParkingDistance
        {
            get { return _parkingDistance.ToString("F0"); }
            set { if (double.TryParse(value, out double v)) _parkingDistance = v; }
        }

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

            if (_warnStyle == null) _warnStyle = new GUIStyle(GUI.skin.label) { normal = { textColor = Color.yellow }, wordWrap = true };
            if (_errorStyle == null) _errorStyle = new GUIStyle(GUI.skin.label) { normal = { textColor = Color.red }, wordWrap = true };

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
            if (_handler.HasInterceptPlan && bbState.InterceptPhase == InterceptPhase.Idle)
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
                    GUILayout.Label($"Error: Burn stalled — {_handler.SettleStallReason}", _warnStyle);
                }
            }

            GUI.enabled = true;

            GUILayout.BeginHorizontal();
            GUILayout.Label("Warp lead (0=auto):", GUILayout.Width(160));
            WarpLead = GUILayout.TextField(WarpLead, GUILayout.Width(60));
            GUILayout.Label("s");
            GUILayout.EndHorizontal();

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
                if (GUILayout.Button($"Warp to transfer ignition ({FormatTime(dtToIgnition)})")) _handler.WarpToIgnition(ignitionUt);
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
                SetParkingDistance();
                _handler.Execute(RendezvousMethod.MatchVelocity);
            }

            // ---- FINAL APPROACH ----
            GUI.enabled = canExecute && bbState.RendezvousMethod != RendezvousMethod.FinalApproach;

            if (GUILayout.Button("Final Approach")) bbState.RendezvousMethod = RendezvousMethod.FinalApproach;

            GUI.enabled = true;

            // Close-approach park distance: when checked, close in to (and velocity-match at) the input
            // distance instead of the default ~100 m. Always editable so it can be set before executing.
            GUILayout.BeginHorizontal();
            GUILayout.Label("Park at distance:", GUILayout.Width(160));
            ParkingDistance = GUILayout.TextField(ParkingDistance, GUILayout.Width(60));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            // enabled = FA will run MV when done; MV will wait for parking distance
            ParkingDistanceEnabled = GUILayout.Toggle(ParkingDistanceEnabled, " Match velocities at CA", GUILayout.Width(200));

            if (bbState.RendezvousMethod == RendezvousMethod.FinalApproach)
            {
                LockedAxes = GUILayout.Toggle(LockedAxes, " Keep axes frozen", GUILayout.Width(160));

                // show errors/warnings
                if (_faError != null) GUILayout.Label(_faError, _errorStyle);   // reuse the red style at line 38

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply")) ApplyCloseApproachParams();

                GUI.enabled = _approachParamsSet;
                if (GUILayout.Button("Execute"))
                {
                    if (MathHelpers.IsFinite(_handler.LiveClosestApproachMeters)
                        && _handler.LiveClosestApproachMeters < _parkingDistance)
                        _faError = "Closest approach is already inside the park distance — lower it or use Match Velocity.";
                    else { _faError = null; _handler.Execute(); }
                }
                GUILayout.EndHorizontal();
            }

            GUI.enabled = true;

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Abort")) _handler.Abort();
            if (GUILayout.Button("Reset"))
            {
                _approachParamsSet = false;

                // reset parking distance
                _parkingDistance = DefaultParkingDistance;
                ParkingDistance = DefaultParkingDistance.ToString("F0");
                ParkingDistanceEnabled = false;
                _faError = null;
                _handler.ResetSequence();
            }
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
            bool canChoose = bbState.InterceptPhase != InterceptPhase.Executing && bbState.InterceptPhase != InterceptPhase.Aborted;

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
                case InterceptPhase.Complete: return "completed";
                case InterceptPhase.Aborted: return "aborted";
                default: return string.Empty;
            }
        }

        // Apply the close-approach closing-speed levers live (each draw). Invalid/empty text keeps the last
        // good value. Raising the max speed turns a long-range crawl into a few large burns + coast + brake.
        private void ApplyCloseApproachParams()
        {
            _handler.WarpLeadInputSeconds = !double.IsNaN(_warpLead) && _warpLead >= 0.0 ? _warpLead : 0.0;
            _handler.KeepFaAxesLocked = LockedAxes;
            SetParkingDistance();
            _approachParamsSet = true;
        }

        private void SetParkingDistance()
        {
            _handler.ParkingDistanceEnabled = ParkingDistanceEnabled;
            _handler.ParkingDistanceMeters = !double.IsNaN(_parkingDistance) && _parkingDistance >= 0.0
                                ? _parkingDistance
                                : DefaultParkingDistance;
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
