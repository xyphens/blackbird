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
        private string _insertionApText = "";
        private string _insertionPeText = "";
        private bool _useTargetOrbitInsertion = true;
        private bool _showAdvancedDetails;
        private Vessel _flyByWireVessel;

        private readonly string[] _guidanceModeLabels =
        {
            "None",
            "Guidance",
            "Autopilot"
        };

        // launch plan and guidance
        private readonly LaunchHandler _launchHandler = new LaunchHandler();
        private LaunchPlan _currentPlan;
        private LaunchPlan _selectedPlan;
        private LaunchPlan _cachedLaunchPlan;
        private double _lastPlanComputeTime = double.NegativeInfinity;
        private const double PlanRecomputeIntervalSeconds = 2.0;

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
                "Rendezvous Assistant");
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
            GUILayout.Space(5);

            GUILayout.Label($"Active: {vessel.vesselName}"); // remove
            GUILayout.Label($"Altitude: {vessel.altitude:N0} m"); // guidance computer
            GUILayout.Label($"Apoapsis: {FormatKm(TrajectoryProvider.GetApoapsisAlt(vessel))} km"); // guidance computer
            GUILayout.Label($"Periapsis: {FormatKm(TrajectoryProvider.GetPeriapsisAlt(vessel))} km"); // guidance computer

            ITargetable target = FlightGlobals.fetch.VesselTarget;

            if (target is Vessel targetVessel)
            {
                if (ReferenceEquals(vessel, targetVessel) || vessel.id == targetVessel.id)
                {
                    GUILayout.Space(10);
                    GUILayout.Label("Target cannot be the active vessel.");
                    GUI.DragWindow();
                    return;
                }

                InsertionTarget insertionTarget = DrawInsertionTargetInputs(targetVessel);

                bool planActive =
                    _launchHandler.State == LaunchGuidanceState.Idle ||
                    _launchHandler.State == LaunchGuidanceState.PlanReady;

                double now = Planetarium.GetUniversalTime();
                if (planActive && (now - _lastPlanComputeTime >= PlanRecomputeIntervalSeconds || _cachedLaunchPlan == null))
                {
                    _lastPlanComputeTime = now;
                    LaunchLocation launchLocation = LaunchLocation.FromVessel(vessel);
                    _cachedLaunchPlan = LaunchPlanner.Create(vessel, targetVessel, insertionTarget, launchLocation);
                    SyncLaunchPlan(_cachedLaunchPlan);
                }

                LaunchPlan displayPlan = GetDisplayPlan(_cachedLaunchPlan);

                DrawPlanSelector();
                DrawLaunchPlanSummary(displayPlan, targetVessel);
                DrawCandidateOptions(displayPlan);
                DrawAscentProfileSummary(displayPlan);
                DrawLaunchHandlerButtons();
                DrawAscentGuidance();
                ShowPhasingRecommendation(displayPlan.PhasingRecommendation, displayPlan);
                DrawLaunchRecommendation(displayPlan);
                DrawLaunchWindowSummary(displayPlan);

                GUILayout.Space(10);
                _showAdvancedDetails = GUILayout.Toggle(
                    _showAdvancedDetails,
                    "Show Advanced Details");

                if (_showAdvancedDetails)
                {
                    DrawAdvancedDetails(displayPlan, targetVessel);
                }
            }
            else
            {
                GUILayout.Space(10);
                GUILayout.Label("No Target");
            }

            GUI.DragWindow();
        }

        private void DrawModuleToggles()
        {
            GUILayout.BeginHorizontal();
            _planner.IsVisible = GUILayout.Toggle(_planner.IsVisible, "Planner");
            _guidanceComputer.IsVisible = GUILayout.Toggle(_guidanceComputer.IsVisible, "Guidance Computer");
            GUILayout.EndHorizontal();
        }

        private void SyncLaunchPlan(LaunchPlan launchPlan)
        {
            PreserveSelectedCandidate(launchPlan);

            if (_launchHandler.State == LaunchGuidanceState.Idle)
            {
                SetCurrentPlan(launchPlan);
            }
            else if (_launchHandler.State == LaunchGuidanceState.PlanReady)
            {
                _currentPlan = launchPlan;
                _selectedPlan = launchPlan;
                _launchHandler.SetPlan(launchPlan);
            }
        }

        // Displays the active handler plan after acceptance so UI matches what guidance is flying.
        private LaunchPlan GetDisplayPlan(LaunchPlan computedPlan)
        {
            if (_launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                _launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch ||
                _launchHandler.State == LaunchGuidanceState.GuidingAscent)
            {
                return _launchHandler.CurrentPlan ?? computedPlan;
            }

            return computedPlan;
        }

        // Carries the user's selected candidate across fresh per-frame plan calculations.
        private void PreserveSelectedCandidate(LaunchPlan launchPlan)
        {
            if (launchPlan == null || _selectedPlan == null) return;
            if (launchPlan.Candidates == null || launchPlan.Candidates.Length == 0) return;

            int selectedIndex = _selectedPlan.SelectedCandidateIndex;
            if (selectedIndex < 0 || selectedIndex >= launchPlan.Candidates.Length) return;

            launchPlan.SelectedCandidateIndex = selectedIndex;
        }

        private InsertionTarget DrawInsertionTargetInputs(Vessel targetVessel)
        {
            GUILayout.Space(10);
            GUILayout.Label("-- Insertion Target --");
            _useTargetOrbitInsertion = GUILayout.Toggle(_useTargetOrbitInsertion, "Use Target Orbit");

            if (_useTargetOrbitInsertion)
            {
                _insertionApText = TrajectoryProvider.GetApoapsisAlt(targetVessel).ToString("F0");
                _insertionPeText = TrajectoryProvider.GetPeriapsisAlt(targetVessel).ToString("F0");
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Ap:", GUILayout.Width(40));
            _insertionApText = GUILayout.TextField(_insertionApText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Pe:", GUILayout.Width(40));
            _insertionPeText = GUILayout.TextField(_insertionPeText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            return CreateInsertionTargetFromUi(targetVessel);
        }

        private void SetCurrentPlan(LaunchPlan plan)
        {
            _currentPlan = plan;
            _selectedPlan = plan;
            _launchHandler.SetPlan(plan);
        }

        private InsertionTarget CreateInsertionTargetFromUi(Vessel targetVessel)
        {
            if (_useTargetOrbitInsertion)
            {
                return InsertionTarget.FromTargetOrbit(targetVessel);
            }

            double ap;
            double pe;

            bool validAp = double.TryParse(_insertionApText, out ap);
            bool validPe = double.TryParse(_insertionPeText, out pe);

            if (!validPe || !validAp || ap < 0.0 || pe < 0.0)
            {
                return InsertionTarget.FromTargetOrbit(targetVessel);
            }

            if (pe > ap)
            {
                double temp = ap;
                ap = pe;
                pe = temp;
            }

            return new InsertionTarget
            {
                ApoapsisAlt = ap,
                PeriapsisAlt = pe
            };
        }

        private void DrawPlanSelector()
        {
            GUILayout.Space(8);
            GUILayout.Label("Launch Plan");

            if (_currentPlan == null)
            {
                GUILayout.Label("No valid launch plan.");
                return;
            }

            bool selected = ReferenceEquals(_selectedPlan, _currentPlan);
            bool newSelected = GUILayout.Toggle(selected, "Current computed plan");

            if (newSelected && !selected)
            {
                _selectedPlan = _currentPlan;
                _launchHandler.SetPlan(_selectedPlan);
            }

            GUILayout.Label($"Guidance State: {_launchHandler.State}");
        }

        private void DrawLaunchHandlerButtons()
        {
            if (_selectedPlan == null)
            {
                return;
            }

            GUILayout.Space(8);

            GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanReady;
            if (GUILayout.Button("Accept Plan"))
            {
                _launchHandler.AcceptPlan();
            }

            GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanAccepted;
            if (GUILayout.Button("Warp To Launch"))
            {
                _launchHandler.WarpToLaunch();
            }

            GUI.enabled =
                _launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch;
            if (GUILayout.Button("Start Guidance"))
            {
                _launchHandler.StartGuidance();
            }

            GUI.enabled =
                _launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                _launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch ||
                _launchHandler.State == LaunchGuidanceState.GuidingAscent;
            if (GUILayout.Button("Abort Guidance"))
            {
                _launchHandler.Abort();
            }

            GUI.enabled = true;
        }

        private void DrawCandidateOptions(LaunchPlan launchPlan)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Candidate Options]");

            if (launchPlan == null || launchPlan.Candidates == null || launchPlan.Candidates.Length == 0)
            {
                GUILayout.Label("No launch candidates.");
                return;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Choose", GUILayout.Width(60));
            GUILayout.Label("Launch In", GUILayout.Width(80));
            GUILayout.Label("AP", GUILayout.Width(45));
            GUILayout.Label("PE", GUILayout.Width(45));
            GUILayout.Label("Head", GUILayout.Width(50));
            GUILayout.Label("Orb", GUILayout.Width(40));
            GUILayout.Label("dV", GUILayout.Width(45));
            GUILayout.Label("Remain", GUILayout.Width(60));
            GUILayout.Label("Err", GUILayout.Width(45));
            GUILayout.EndHorizontal();

            for (int i = 0; i < launchPlan.Candidates.Length; i++)
            {
                DrawCandidateRow(launchPlan, i);
            }
        }

        private void DrawCandidateRow(LaunchPlan launchPlan, int candidateIndex)
        {
            LaunchCandidate candidate = launchPlan.Candidates[candidateIndex];
            bool isSelected = launchPlan.SelectedCandidateIndex == candidateIndex;
            bool canChoose =
                candidate.IsValid &&
                (_launchHandler.State == LaunchGuidanceState.Idle ||
                 _launchHandler.State == LaunchGuidanceState.PlanReady);

            GUILayout.BeginHorizontal();

            GUI.enabled = canChoose && !isSelected;
            if (GUILayout.Button(isSelected ? "Chosen" : "Choose", GUILayout.Width(60)))
            {
                SelectCandidate(launchPlan, candidateIndex);
            }

            GUI.enabled = true;

            GUILayout.Label(
                candidate.IsValid
                    ? BlackbirdHelpers.FormatDuration(candidate.SecondsUntilLaunch)
                    : "N/A",
                GUILayout.Width(80));
            GUILayout.Label(FormatKm(candidate.InsertionApoapsisAlt), GUILayout.Width(45));
            GUILayout.Label(FormatKm(candidate.InsertionPeriapsisAlt), GUILayout.Width(45));
            GUILayout.Label(FormatValue(candidate.LaunchHeadingDeg, "F1"), GUILayout.Width(50));
            GUILayout.Label(FormatValue(candidate.EstimatedOrbitsToRendezvous, "F1"), GUILayout.Width(40));
            GUILayout.Label(FormatValue(candidate.EstimatedDeltaVUsed, "F0"), GUILayout.Width(45));
            GUILayout.Label(FormatValue(candidate.EstimatedRemainingDeltaV, "F0"), GUILayout.Width(60));
            GUILayout.Label(FormatValue(Math.Abs(candidate.PhaseErrorDeg), "F1"), GUILayout.Width(45));

            GUILayout.EndHorizontal();

            if (!candidate.IsValid && !string.IsNullOrEmpty(candidate.ReasonUnavailable))
            {
                GUILayout.Label(candidate.ReasonUnavailable);
            }
        }

        // Applies a candidate choice without duplicating selected candidate fields on LaunchPlan.
        private void SelectCandidate(LaunchPlan launchPlan, int candidateIndex)
        {
            if (launchPlan == null || launchPlan.Candidates == null) return;
            if (candidateIndex < 0 || candidateIndex >= launchPlan.Candidates.Length) return;

            launchPlan.SelectedCandidateIndex = candidateIndex;
            _currentPlan = launchPlan;
            _selectedPlan = launchPlan;

            if (_launchHandler.State == LaunchGuidanceState.Idle ||
                _launchHandler.State == LaunchGuidanceState.PlanReady)
            {
                _launchHandler.SetPlan(launchPlan);
            }
        }

        private static string FormatKm(double meters)
        {
            return double.IsNaN(meters) || double.IsInfinity(meters)
                ? "N/A"
                : (meters / 1000.0).ToString("F0");
        }

        private static string FormatValue(double value, string format)
        {
            return double.IsNaN(value) || double.IsInfinity(value)
                ? "N/A"
                : value.ToString(format);
        }

        private void DrawAscentProfileSummary(LaunchPlan launchPlan)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Ascent Profile]");

            AscentProfile profile = launchPlan != null ? launchPlan.AscentProfile : null;
            if (profile == null || profile.Points == null || profile.Points.Length == 0)
            {
                GUILayout.Label("Unavailable");
                return;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Alt", GUILayout.Width(60));
            GUILayout.Label("Pitch", GUILayout.Width(55));
            GUILayout.Label("Head", GUILayout.Width(55));
            GUILayout.Label("Throttle", GUILayout.Width(70));
            GUILayout.EndHorizontal();

            for (int i = 0; i < profile.Points.Length; i++)
            {
                AscentProfilePoint point = profile.Points[i];

                GUILayout.BeginHorizontal();
                GUILayout.Label(FormatKm(point.AltitudeMeters) + " km", GUILayout.Width(60));
                GUILayout.Label(point.PitchDeg.ToString("F1") + "°", GUILayout.Width(55));
                GUILayout.Label(point.HeadingDeg.ToString("F1") + "°", GUILayout.Width(55));
                GUILayout.Label(FormatThrottle(point.Throttle), GUILayout.Width(70));
                GUILayout.EndHorizontal();
            }
        }

        private static string FormatThrottle(double throttlePercent)
        {
            if (double.IsNaN(throttlePercent) || double.IsInfinity(throttlePercent)) return "N/A";
            return throttlePercent <= 0.0 ? "cutoff" : (throttlePercent * 100).ToString("F0") + "%";
        }

        private void DrawLaunchPlanSummary(
            LaunchPlan launchPlan,
            Vessel targetVessel)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Plan]");
            GUILayout.Label($"Target: {targetVessel.vesselName}");
            GUILayout.Label($"Scale: {launchPlan.ScaleLabel}");
            GUILayout.Label($"Trajectory: {TrajectoryProvider.ActiveSourceName}");
            GUILayout.Label($"Active Heading: {launchPlan.LaunchAzimuthDeg:F1}°");
            GUILayout.Label($"Target Inc: {launchPlan.TargetOrbit.InclinationDeg:F2}°");

            if (_launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch)
            {
                GUILayout.Label(
                    "Launch In: " +
                    BlackbirdHelpers.FormatDuration(
                        Math.Max(0.0, _launchHandler.SecondsUntilLaunch)));
            }
        }

        private void DrawAscentGuidanceMethod()
        {
            GUILayout.Space(10);
            GUILayout.Label("Flight Mode");

            int selectedIndex =
                _launchHandler.GuidanceMode == GuidanceMode.Guidance ? 1 :
                _launchHandler.GuidanceMode == GuidanceMode.Autopilot ? 2 :
                0;

            int newSelectedIndex =
                GUILayout.SelectionGrid(
                    selectedIndex,
                    _guidanceModeLabels,
                    3);

            GuidanceMode newMode =
                newSelectedIndex == 1 ? GuidanceMode.Guidance :
                newSelectedIndex == 2 ? GuidanceMode.Autopilot :
                GuidanceMode.None;

            if (newMode != _launchHandler.GuidanceMode) _launchHandler.SetGuidanceMode(newMode, FlightGlobals.ActiveVessel);
        }

        private void DrawAscentGuidance()
        {
            if (_launchHandler.State != LaunchGuidanceState.GuidingAscent) return;

            AscentGuidanceInfo guidanceInfo = _launchHandler.GuidanceInfo;

            GUILayout.Space(10);
            GUILayout.Label("[Ascent Guidance]");

            // show guidance method dropdowns
            DrawAscentGuidanceMethod();

            if (guidanceInfo == null)
            {
                GUILayout.Label("Guidance unavailable");
                return;
            }

            string gMode = _launchHandler.GuidanceMode == GuidanceMode.Autopilot 
                            ? "Autopilot" :
                            _launchHandler.GuidanceMode == GuidanceMode.Guidance ? "Guidance" 
                            : "None";

            bool canAdjustGuidance = _launchHandler.GuidanceMode == GuidanceMode.Guidance;

            GUILayout.Label($"Guidance mode: {gMode}");
            GUILayout.Label($"Guidance phase: {guidanceInfo.GuidancePhase}");

            GUILayout.Label(guidanceInfo.PitchInstruction);
            GUILayout.Label(guidanceInfo.HeadingInstruction);

            // PITCH
            GUILayout.Label($"Pitch Profile");
            GUILayout.Label($"Profile Pitch: {guidanceInfo.ProfilePitchDeg:F1}°");
            GUILayout.Label($"Pitch Input: {guidanceInfo.CommandPitchDeg:F1}°");
            GUILayout.Label($"Current Pitch: {guidanceInfo.CurrentPitchDeg:F1}°");
            //GUILayout.Label($"Pitch Error: {guidanceInfo.PitchErrorDeg:F1}°");

            // pitch inputs
            if (canAdjustGuidance)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("- Pitch")) _launchHandler.DecreaseManualPitchCommand();
                if (GUILayout.Button("+ Pitch")) _launchHandler.IncreaseManualPitchCommand();
                if (GUILayout.Button("Reset Pitch")) _launchHandler.ResetPitchCommand();
                GUILayout.EndHorizontal();
            }

            // HEADING
            GUILayout.Label($"Heading Profile");
            GUILayout.Label(
                double.IsNaN(guidanceInfo.ProfileHeadingDeg)
                    ? "Profile Heading: unavailable"
                    : $"Profile Heading: {guidanceInfo.ProfileHeadingDeg:F1}°");

            GUILayout.Label($"Heading Input: {guidanceInfo.CommandHeadingDeg:F1}°");
            GUILayout.Label($"Current Heading: {guidanceInfo.CurrentHeadingDeg:F1}°");
            //GUILayout.Label($"Heading Error: {guidanceInfo.HeadingErrorDeg:F1}°");

            GUILayout.Label($"Profile Throttle: {FormatThrottle(guidanceInfo.ProfileThrottle)}");
            GUILayout.Label($"Command Throttle: {FormatThrottle(guidanceInfo.CommandThrottle)}");

            // heading inputs
            if (canAdjustGuidance)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("- Heading")) _launchHandler.DecreaseManualHeadingCommand();
                if (GUILayout.Button("+ Heading")) _launchHandler.IncreaseManualHeadingCommand();
                if (GUILayout.Button("Reset Heading")) _launchHandler.ResetHeadingCommand();
                GUILayout.EndHorizontal();
            }

            GUILayout.Label($"Target AP: {guidanceInfo.TargetApoapsisAlt / 1000.0:F0} km");
            GUILayout.Label($"Target PE: {guidanceInfo.TargetPeriapsisAlt / 1000.0:F0} km");
            GUILayout.Label($"AP Error: {guidanceInfo.ApoapsisErrorMeters / 1000.0:F1} km");
            GUILayout.Label($"PE Error: {guidanceInfo.PeriapsisErrorMeters / 1000.0:F1} km");
            GUILayout.Label($"Guidance vgo: {guidanceInfo.GuidanceVelocityToGoMetersPerSecond:F0} m/s");
            GUILayout.Label($"Guidance tgo: {guidanceInfo.GuidanceTimeToGoSeconds:F1} s");
            GUILayout.Label($"PSG: {guidanceInfo.GuidanceOptimizerStatus}");
            GUILayout.Label($"PSG iters: {guidanceInfo.GuidanceOptimizerIterations}");
            GUILayout.Label($"PSG violation: {guidanceInfo.GuidanceConstraintViolation:E2}");
            GUILayout.Label($"Predicted AP: {guidanceInfo.PredictedApoapsisAlt / 1000.0:F0} km");
            GUILayout.Label($"Predicted PE: {guidanceInfo.PredictedPeriapsisAlt / 1000.0:F0} km");
            GUILayout.Label($"Remaining dV: {guidanceInfo.EstimatedRemainingDeltaV:F0} m/s");
            GUILayout.Label($"Phase Error: {guidanceInfo.PhaseErrorDeg:F2}°");
            GUILayout.Label($"Plane Error: {guidanceInfo.PlaneErrorDeg:F2}°");
        }

        //private void ShowPhasingRecommendation(
        //    PhasingRecommendation recommendation,
        //    LaunchPlan launchPlan)
        //{
        //    GUILayout.Space(10);
        //    GUILayout.Label("[Phasing Recommendation]");

        //    if (recommendation == null)
        //    {
        //        GUILayout.Label("Unavailable");
        //        return;
        //    }

        //    if (!recommendation.HasRecommendation)
        //    {
        //        GUILayout.Label("Unavailable");
        //        GUILayout.Label(recommendation.ReasonUnavailable);
        //        return;
        //    }

        //    GUILayout.Label($"Mode: {recommendation.Mode}");
        //    GUILayout.Label($"Apoapsis: {recommendation.ApoapsisAlt / 1000.0:N0} km");
        //    GUILayout.Label($"Periapsis: {recommendation.PeriapsisAlt / 1000.0:N0} km");
        //    GUILayout.Label($"Rendezvous: {BlackbirdHelpers.FormatDuration(recommendation.EstimatedTimeToRendezvousSeconds)}");
        //    GUILayout.Label($"Rendezvous Orbits: {recommendation.EstimatedOrbitsToRendezvous:N1}");
        //}

        //private void DrawLaunchRecommendation(LaunchPlan launchPlan)
        //{
        //    GUILayout.Space(10);
        //    GUILayout.Label("[Launch Recommendation]");
        //    GUILayout.Label(
        //        double.IsNaN(launchPlan.LaunchAzimuthDeg)
        //            ? "Azimuth: unavailable"
        //            : $"Azimuth: {launchPlan.LaunchAzimuthDeg:F1}°");
        //    GUILayout.Label($"Apoapsis: {launchPlan.RecommendedApAlt / 1000:F0} km");
        //    GUILayout.Label($"Periapsis: {launchPlan.RecommendedPeAlt / 1000:F0} km");
        //}

        //private void DrawLaunchWindowSummary(LaunchPlan launchPlan)
        //{
        //    GUILayout.Space(10);
        //    GUILayout.Label("[Launch Window]");
        //    GUILayout.Label($"Current Node: {launchPlan.LaunchWindow.NodeName}");
        //    GUILayout.Label(
        //        "Launch In: " +
        //        BlackbirdHelpers.FormatDuration(GetDisplayedLaunchCountdownSeconds(launchPlan)));
        //}

        // Uses live handler countdown after a plan is accepted, otherwise shows the computed window.
        private double GetDisplayedLaunchCountdownSeconds(LaunchPlan launchPlan)
        {
            if (_launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                _launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch ||
                _launchHandler.State == LaunchGuidanceState.GuidingAscent)
            {
                return Math.Max(0.0, _launchHandler.SecondsUntilLaunch);
            }

            return launchPlan != null && launchPlan.LaunchWindow != null
                ? launchPlan.LaunchWindow.TimeToPlaneCrossingSeconds
                : double.NaN;
        }

        private void DrawAdvancedDetails(
            LaunchPlan launchPlan,
            Vessel targetVessel)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Advanced Details]");

            GUILayout.Space(10);
            GUILayout.Label("-- Active Orbit --");
            GUILayout.Label($"Inclination: {launchPlan.ActiveOrbit.InclinationDeg:F2}°");
            GUILayout.Label($"LAN: {launchPlan.ActiveOrbit.LanDeg:F2}°");
            GUILayout.Label($"Apoapsis: {launchPlan.ActiveOrbit.ApoapsisAlt / 1000:F0} km");
            GUILayout.Label($"Periapsis: {launchPlan.ActiveOrbit.PeriapsisAlt / 1000:F0} km");
            GUILayout.Label($"Period: {launchPlan.ActiveOrbit.PeriodSeconds:F1}s");

            GUILayout.Space(10);
            GUILayout.Label("-- Target Orbit --");
            GUILayout.Label($"Name: {targetVessel.vesselName}");
            GUILayout.Label($"Distance: {launchPlan.DistanceMeters / 1000:F1} km");
            GUILayout.Label($"Inclination: {launchPlan.TargetOrbit.InclinationDeg:F2}°");
            GUILayout.Label($"LAN: {launchPlan.TargetOrbit.LanDeg:F2}°");
            GUILayout.Label($"Apoapsis: {launchPlan.TargetOrbit.ApoapsisAlt / 1000:F0} km");
            GUILayout.Label($"Periapsis: {launchPlan.TargetOrbit.PeriapsisAlt / 1000:F0} km");
            GUILayout.Label($"Phase Angle: {launchPlan.PhaseAngleDeg:F1}°");

            GUILayout.Space(10);
            GUILayout.Label("-- Orbit Comparison --");
            GUILayout.Label($"Inc Delta: {launchPlan.RelativeInclinationDeg:F2}°");
            GUILayout.Label($"LAN Delta: {launchPlan.RelativeLanDeg:F2}°");
            GUILayout.Label($"Period Delta: {launchPlan.RelativePeriodSeconds:F1}s");

            GUILayout.Space(10);
            GUILayout.Label("-- Phasing Period --");
            GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferenceSeconds:F1}s");
            GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferenceMinutes:F2} min");
            GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferencePercent:F3}%");
            GUILayout.Label($"Phase Gain: {launchPlan.PhasingOrbit.RelativePhaseGainDegPerOrbit:F2}°/orbit");

            if (launchPlan.PhasingOrbit.HasRendezvousEstimate)
            {
                GUILayout.Label($"Rendezvous Orbits: {launchPlan.PhasingOrbit.EstimatedOrbitsToRendezvous:F1}");
                GUILayout.Label(
                    $"Rendezvous Time: {BlackbirdHelpers.FormatDuration(launchPlan.PhasingOrbit.EstimatedTimeToRendezvousSeconds)}");
            }
            else
            {
                GUILayout.Label("Rendezvous Estimate: unavailable");
            }

            GUILayout.Label(
                launchPlan.PhasingOrbit.IsFasterThanTarget
                    ? "Phasing: insertion orbit is faster than target"
                    : "Phasing: insertion orbit is slower than target");

            GUILayout.Space(10);
            GUILayout.Label("-- Phasing Recommendation Details --");
            PhasingRecommendation recommendation = launchPlan.PhasingRecommendation;
            if (recommendation != null && recommendation.HasRecommendation)
            {
                GUILayout.Label($"Period Diff: {recommendation.PeriodDifferenceSeconds:N1}s");
                GUILayout.Label($"Phase Gain: {recommendation.PhaseGainDegPerOrbit:N2}°/orbit");
                GUILayout.Label(
                    "Offset: " +
                    ((recommendation.ApoapsisAlt - launchPlan.TargetOrbit.ApoapsisAlt) / 1000.0).ToString("N0") +
                    " km");
            }
            else
            {
                GUILayout.Label("Unavailable");
            }

            GUILayout.Space(10);
            GUILayout.Label("-- Launch Window Details --");
            GUILayout.Label($"Asc Node Lon: {launchPlan.LaunchWindow.AscendingNodeLongitudeDeg:F2}°");
            GUILayout.Label($"Desc Node Lon: {launchPlan.LaunchWindow.DescendingNodeLongitudeDeg:F2}°");
            GUILayout.Label($"Time to Asc: {launchPlan.LaunchWindow.TimeToAscendingNodeSeconds:F0}s");
            GUILayout.Label($"Time to Desc: {launchPlan.LaunchWindow.TimeToDescendingNodeSeconds:F0}s");
            GUILayout.Label($"Selected Offset: {launchPlan.LaunchWindow.PlaneOffsetDeg:F2}°");
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
