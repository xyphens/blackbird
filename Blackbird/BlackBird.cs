using UnityEngine;
using System;
using Blackbird;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;
using Blackbird.Guidance;
using Blackbird.Enums;

namespace Blackbird
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class RendezvousAssistant : MonoBehaviour
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

        public void Start()
        {
            Debug.Log("[RendezvousAssistant] Loaded");
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
            _launchHandler.ApplyFlightControls(state);
        }

        public void OnDestroy()
        {
            if (_flyByWireVessel != null)
            {
                _flyByWireVessel.OnFlyByWire -= OnFlyByWire;
                _flyByWireVessel = null;
            }
        }

        private void OnGUI()
        {
            _windowRect = GUILayout.Window(
                12345,
                _windowRect,
                DrawWindow,
                "Rendezvous Assistant");
        }

        private void DrawWindow(int windowId)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
            {
                return;
            }

            GUILayout.Label($"Active: {vessel.vesselName}");
            GUILayout.Label($"Altitude: {vessel.altitude:N0} m");

            ITargetable target = FlightGlobals.fetch.VesselTarget;

            if (target is Vessel targetVessel)
            {
                InsertionTarget insertionTarget = DrawInsertionTargetInputs(targetVessel);
                LaunchLocation launchLocation = LaunchLocation.FromVessel(vessel);
                LaunchPlan launchPlan = LaunchPlanner.Create(
                    vessel,
                    targetVessel,
                    insertionTarget,
                    launchLocation);

                SyncLaunchPlan(launchPlan);

                DrawPlanSelector();
                DrawLaunchHandlerButtons();
                DrawLaunchPlanSummary(launchPlan, targetVessel);
                DrawAscentGuidance();
                ShowPhasingRecommendation(launchPlan.PhasingRecommendation, launchPlan);
                DrawLaunchRecommendation(launchPlan);
                DrawLaunchWindowSummary(launchPlan);

                GUILayout.Space(10);
                _showAdvancedDetails = GUILayout.Toggle(
                    _showAdvancedDetails,
                    "Show Advanced Details");

                if (_showAdvancedDetails)
                {
                    DrawAdvancedDetails(launchPlan, targetVessel);
                }
            }
            else
            {
                GUILayout.Space(10);
                GUILayout.Label("No Target");
            }

            GUI.DragWindow();
        }

        private void SyncLaunchPlan(LaunchPlan launchPlan)
        {
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

        private InsertionTarget DrawInsertionTargetInputs(Vessel targetVessel)
        {
            GUILayout.Space(10);
            GUILayout.Label("-- Insertion Target --");
            _useTargetOrbitInsertion = GUILayout.Toggle(_useTargetOrbitInsertion, "Use Target Orbit");

            if (_useTargetOrbitInsertion)
            {
                _insertionApText = targetVessel.orbit.ApA.ToString("F0");
                _insertionPeText = targetVessel.orbit.PeA.ToString("F0");
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

        private void DrawLaunchPlanSummary(
            LaunchPlan launchPlan,
            Vessel targetVessel)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Plan]");
            GUILayout.Label($"Target: {targetVessel.vesselName}");
            GUILayout.Label($"Scale: {launchPlan.ScaleLabel}");

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
            if (_launchHandler.State != LaunchGuidanceState.GuidingAscent)
            {
                return;
            }

            AscentGuidanceInfo guidanceInfo = _launchHandler.GuidanceInfo;

            GUILayout.Space(10);
            GUILayout.Label("[Ascent Guidance]");

            if (guidanceInfo == null)
            {
                GUILayout.Label("Guidance unavailable");
                return;
            }

            // show guidance method dropdowns
            DrawAscentGuidanceMethod();

            string gMode = _launchHandler.GuidanceMode == GuidanceMode.Autopilot 
                            ? "Autopilot" :
                            _launchHandler.GuidanceMode == GuidanceMode.Guidance ? "Guidance" 
                            : "None";

            bool canAdjustGuidance = _launchHandler.GuidanceMode == GuidanceMode.Guidance;

            GUILayout.Label($"Guidance mode: {gMode}");

            GUILayout.Label(guidanceInfo.PitchInstruction);
            GUILayout.Label(guidanceInfo.HeadingInstruction);

            // PITCH
            GUILayout.Label($"Pitch Profile");
            GUILayout.Label($"Target Pitch: {guidanceInfo.TargetPitchDeg:F1}°");
            //GUILayout.Label($"Pitch Offset: {guidanceInfo.PitchOffsetDeg:+0.0;-0.0;0.0}°");
            GUILayout.Label($"Pitching Towards: {guidanceInfo.CommandPitchDeg:F1}°");
            GUILayout.Label($"Current Pitch: {guidanceInfo.CurrentPitchDeg:F1}°");
            GUILayout.Label($"Pitch Error: {guidanceInfo.PitchErrorDeg:F1}°");

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
                double.IsNaN(guidanceInfo.TargetAzimuthDeg)
                    ? "Target Heading: unavailable"
                    : $"Target Heading: {guidanceInfo.TargetAzimuthDeg:F1}°");

            GUILayout.Label($"Heading Towards: {guidanceInfo.CommandHeadingDeg:F1}°");
            GUILayout.Label($"Current Heading: {guidanceInfo.CurrentHeadingDeg:F1}°");
            GUILayout.Label($"Heading Error: {guidanceInfo.HeadingErrorDeg:F1}°");

            // heading inputs
            if (canAdjustGuidance)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("- Heading")) _launchHandler.DecreaseManualHeadingCommand();
                if (GUILayout.Button("+ Heading")) _launchHandler.IncreaseManualHeadingCommand();
                if (GUILayout.Button("Reset Heading")) _launchHandler.ResetHeadingCommand();
                GUILayout.EndHorizontal();
            }

            GUILayout.Label($"Target LAN: {guidanceInfo.TargetLanDeg:F2}°");
            GUILayout.Label($"Current LAN: {guidanceInfo.CurrentLanDeg:F2}°");
            GUILayout.Label($"LAN Error: {guidanceInfo.LanErrorDeg:F2}°");
        }

        private void ShowPhasingRecommendation(
            PhasingRecommendation recommendation,
            LaunchPlan launchPlan)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Phasing Recommendation]");

            if (recommendation == null)
            {
                GUILayout.Label("Unavailable");
                return;
            }

            if (!recommendation.HasRecommendation)
            {
                GUILayout.Label("Unavailable");
                GUILayout.Label(recommendation.ReasonUnavailable);
                return;
            }

            GUILayout.Label($"Mode: {recommendation.Mode}");
            GUILayout.Label($"Apoapsis: {recommendation.ApoapsisAlt / 1000.0:N0} km");
            GUILayout.Label($"Periapsis: {recommendation.PeriapsisAlt / 1000.0:N0} km");
            GUILayout.Label($"Rendezvous: {BlackbirdHelpers.FormatDuration(recommendation.EstimatedTimeToRendezvousSeconds)}");
            GUILayout.Label($"Rendezvous Orbits: {recommendation.EstimatedOrbitsToRendezvous:N1}");
        }

        private void DrawLaunchRecommendation(LaunchPlan launchPlan)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Recommendation]");
            GUILayout.Label(
                double.IsNaN(launchPlan.LaunchAzimuthDeg)
                    ? "Azimuth: unavailable"
                    : $"Azimuth: {launchPlan.LaunchAzimuthDeg:F1}°");
            GUILayout.Label($"Apoapsis: {launchPlan.RecommendedApAlt / 1000:F0} km");
            GUILayout.Label($"Periapsis: {launchPlan.RecommendedPeAlt / 1000:F0} km");
        }

        private void DrawLaunchWindowSummary(LaunchPlan launchPlan)
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Window]");
            GUILayout.Label($"Current Node: {launchPlan.LaunchWindow.NodeName}");
            GUILayout.Label(
                "Launch In: " +
                BlackbirdHelpers.FormatDuration(
                    launchPlan.LaunchWindow.TimeToPlaneCrossingSeconds));
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
    }
}
