using System;
using Blackbird.Enums;
using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Models;
using UnityEngine;
using static EdyCommonTools.Spline;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private static readonly int WindowId = "Blackbird.GuidanceComputer".GetHashCode();
        private Rect _windowRect = new Rect(560, 620, 380, 300);
        private string _pitchInputText = "";
        private string _headingInputText = "";
        private string _rollInputText = "";
        private string _throttleInputText = "";
        private bool _showAdvancedDetails;
        private double MinSecondsToUseWarp = 10;
        private readonly string[] _guidanceModeLabels =
        {
            "None",
            "Manual",
            "Autopilot"
        };

        private LaunchHandler _launchHandler;
        private GuidanceComputer _guidanceComputer;
        public bool IsVisible { get; set; }

        public void Toggle() => IsVisible = !IsVisible;
        public void Initialize(LaunchHandler handler) => _launchHandler = handler;

        public void Draw()
        {
            if (!IsVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
        }

        private void DrawContents(int _windowId)
        {
            if (_launchHandler.State != LaunchGuidanceState.GuidingAscent)
            {
                if (_launchHandler == null || _launchHandler.CurrentPlan == null)
                {
                    GUILayout.Label("Guidance unavailable");
                    return;
                } else
                {
                    GUILayout.Label("[Launch Window]");
                    GUILayout.Label($"Asc Node Lon: {_launchHandler.CurrentPlan.LaunchWindow.AscendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Desc Node Lon: {_launchHandler.CurrentPlan.LaunchWindow.DescendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Time to Asc: {_launchHandler.CurrentPlan.LaunchWindow.TimeToAscendingNodeSeconds:F0}s");
                    GUILayout.Label($"Time to Desc: {_launchHandler.CurrentPlan.LaunchWindow.TimeToDescendingNodeSeconds:F0}s");
                    GUILayout.Label($"Selected Offset: {_launchHandler.CurrentPlan.LaunchWindow.PlaneOffsetDeg:F2}°");

                    double _countdownSeconds = GetDisplayedLaunchCountdownSeconds(_launchHandler.CurrentPlan);
                    GUILayout.Label($"T- {_countdownSeconds:F0} seconds");

                    GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanAccepted && _countdownSeconds >= MinSecondsToUseWarp;
                    if (GUILayout.Button("Warp To Launch")) _launchHandler.WarpToLaunch();

                    GUI.enabled =
                        _launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                        _launchHandler.State == LaunchGuidanceState.AwaitingLaunch;
                    if (GUILayout.Button("Start Guidance")) _launchHandler.StartGuidance();

                    GUI.enabled =
                        _launchHandler.State == LaunchGuidanceState.PlanAccepted ||
                        _launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                        _launchHandler.State == LaunchGuidanceState.AwaitingLaunch ||
                        _launchHandler.State == LaunchGuidanceState.GuidingAscent;
                    if (GUILayout.Button("Abort Guidance")) _launchHandler.Abort();

                    GUI.enabled = true;
                }
                return;
            }

            AscentGuidanceInfo guidanceInfo = _launchHandler.GuidanceInfo;

            // select Autopilot / Guidance / None
            DrawSelectGuidanceMethod();

            GUILayout.Space(10);

            string gMode = _launchHandler.GuidanceMode == GuidanceMode.Autopilot
                            ? "Autopilot" :
                            _launchHandler.GuidanceMode == GuidanceMode.Manual ? "Manual"
                            : "None";

            bool canAdjustGuidance = _launchHandler.GuidanceMode == GuidanceMode.Manual;

            GUILayout.Label($"Guidance mode: {gMode}");
            
            // pitch inputs
            if (canAdjustGuidance)
            {
                GUILayout.Label("[Manual Control]");
                GUILayout.Space(10);
                // PITCH
                GUILayout.BeginHorizontal();
                GUILayout.Label("[Pitch]", GUILayout.Width(70));
                GUILayout.Label($"Command Pitch: {guidanceInfo.CommandPitchDeg:F2}°");
                // input
                _pitchInputText = GUILayout.TextField(_pitchInputText, GUILayout.Width(100));
                double.TryParse(_headingInputText, out double pitch);
                if (GUILayout.Button("Exct.")) _launchHandler.SetPitchCommand(pitch);

                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualPitchCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualPitchCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetPitchCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(5);

                // HEADING
                GUILayout.BeginHorizontal();
                GUILayout.Label("[Heading]", GUILayout.Width(70));
                GUILayout.Label($"Command Heading: {guidanceInfo.CommandHeadingDeg:F2}°");
                // input
                _headingInputText = GUILayout.TextField(_headingInputText, GUILayout.Width(100));
                double.TryParse(_headingInputText, out double hdg);
                if (GUILayout.Button("Exct.")) _launchHandler.SetHeadingCommand(hdg);

                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualHeadingCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualHeadingCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetHeadingCommand();
                GUILayout.EndHorizontal();

                // ROLL
                GUILayout.BeginHorizontal();
                GUILayout.Label("[Roll]", GUILayout.Width(70));
                GUILayout.Label($"Command Roll: {guidanceInfo.CommandRoll:F2}°");
                // input
                _rollInputText = GUILayout.TextField(_rollInputText, GUILayout.Width(100));
                double.TryParse(_headingInputText, out double roll);
                if (GUILayout.Button("Exct.")) _launchHandler.SetRollCommand(roll);

                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualRollCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualRollCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetRollCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(5);

                // THROTTLE
                GUILayout.BeginHorizontal();
                GUILayout.Label("[Throttle]", GUILayout.Width(70));
                GUILayout.Label($"Command Throttle: {BlackbirdHelpers.FormatThrottle(guidanceInfo.CommandThrottle)}%");

                _throttleInputText = GUILayout.TextField(_throttleInputText, GUILayout.Width(100));
                double.TryParse(_throttleInputText, out double thtl);

                if (GUILayout.Button("Exct.")) _launchHandler.SetThrottleCommand(thtl);

                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualThrottleCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualThrottleCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetThrottleCommand();
                GUILayout.EndHorizontal();
            } else if (_launchHandler.GuidanceMode == GuidanceMode.Autopilot) { 
                GUILayout.Label("[Flight]", GUILayout.Width(70));
                // table headers
                GUILayout.BeginHorizontal();
                GUILayout.Label("Command", GUILayout.Width(80));
                GUILayout.Label("Guidance", GUILayout.Width(45));
                GUILayout.Label("Vessel", GUILayout.Width(45));
                GUILayout.EndHorizontal();

                // pitch
                GUILayout.BeginHorizontal();
                GUILayout.Label("Pitch", GUILayout.Width(80));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.ProfilePitchDeg)
                            ? "Unavailable"
                            : $"{guidanceInfo.ProfilePitchDeg:F1}°", GUILayout.Width(45));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.CommandPitchDeg)
                            ? "Unavailable"
                            : $"{guidanceInfo.CommandPitchDeg:F1}° ({guidanceInfo.CurrentPitchDeg:F1}°)", GUILayout.Width(45));
                GUILayout.EndHorizontal();

                // heading
                GUILayout.BeginHorizontal();
                GUILayout.Label("Heading", GUILayout.Width(80));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.ProfileHeadingDeg)
                            ? "Unavailable"
                            : $"{guidanceInfo.ProfileHeadingDeg:F1}°", GUILayout.Width(45));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.CommandHeadingDeg)
                            ? "Unavailable"
                            : $"{guidanceInfo.CommandHeadingDeg:F1}° ({guidanceInfo.CurrentHeadingDeg:F1}°)", GUILayout.Width(45));
                GUILayout.EndHorizontal();

                // throttle
                GUILayout.BeginHorizontal();
                GUILayout.Label("Throttle", GUILayout.Width(80));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.ProfileThrottle)
                            ? "Unavailable"
                            : $"{guidanceInfo.ProfileThrottle:F0}%", GUILayout.Width(45));
                GUILayout.Label(
                        double.IsNaN(guidanceInfo.CommandThrottle)
                            ? "Unavailable"
                            : $"{guidanceInfo.CommandThrottle:F0}%", GUILayout.Width(45));
                GUILayout.EndHorizontal();

                GUILayout.Space(10);

                GUILayout.Label("[Guidance]", GUILayout.Width(70));
                GUILayout.Label($"Status: {guidanceInfo.GuidanceOptimizerStatus}");
                GUILayout.Label($"Target AP: {guidanceInfo.TargetApoapsisAlt / 1000.0:F0} km");
                GUILayout.Label($"Predicted AP: {guidanceInfo.PredictedApoapsisAlt / 1000.0:F0} km");
                GUILayout.Label($"Target PE: {guidanceInfo.TargetPeriapsisAlt / 1000.0:F0} km");
                GUILayout.Label($"Predicted PE: {guidanceInfo.PredictedPeriapsisAlt / 1000.0:F0} km");
                GUILayout.Label($"AP Error: {guidanceInfo.ApoapsisErrorMeters / 1000.0:F1} km");
                GUILayout.Label($"PE Error: {guidanceInfo.PeriapsisErrorMeters / 1000.0:F1} km");
                GUILayout.Label(
                    _launchHandler.CurrentPlan.PhasingOrbit.IsFasterThanTarget
                        ? "Phasing: insertion orbit is faster than target"
                        : "Phasing: insertion orbit is slower than target");
                    if (_launchHandler.CurrentPlan.PhasingOrbit.HasRendezvousEstimate)
                    {
                        GUILayout.Label($"Rendezvous Orbits: {_launchHandler.CurrentPlan.PhasingOrbit.EstimatedOrbitsToRendezvous:F1}");
                        GUILayout.Label(
                            $"Rendezvous Time: {BlackbirdHelpers.FormatDuration(_launchHandler.CurrentPlan.PhasingOrbit.EstimatedTimeToRendezvousSeconds)}");
                    }
                    else
                    {
                        GUILayout.Label("Rendezvous Estimate: unavailable");
                    }
                // remaining stats
                GUILayout.Label($"Rem. Velocity: {guidanceInfo.GuidanceVelocityToGoMetersPerSecond:F0} m/s");
                GUILayout.Label($"Rem. Time: {guidanceInfo.GuidanceTimeToGoSeconds:F1} s");
                GUILayout.Label($"Rem. dV: {guidanceInfo.EstimatedRemainingDeltaV:F0} m/s");
                GUILayout.Label($"Phase Error: {guidanceInfo.PhaseErrorDeg:F2}°");
                GUILayout.Label($"Plane Error: {guidanceInfo.PlaneErrorDeg:F2}°");

                _showAdvancedDetails = GUILayout.Toggle(_showAdvancedDetails, "Show Advanced Details");
                
                if (_showAdvancedDetails) DrawAdvancedDetails(_launchHandler.CurrentPlan, _launchHandler.TargetVessel);
            }
        }
        private void DrawAdvancedDetails(LaunchPlan launchPlan, Vessel targetVessel)
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

        private void DrawSelectGuidanceMethod()
        {
            GUILayout.Space(10);
            GUILayout.Label("Flight Mode");

            int selectedIndex =
                _launchHandler.GuidanceMode == GuidanceMode.Manual ? 1 :
                _launchHandler.GuidanceMode == GuidanceMode.Autopilot ? 2 :
                0;

            int newSelectedIndex =
                GUILayout.SelectionGrid(
                    selectedIndex,
                    _guidanceModeLabels,
                    3);

            GuidanceMode newMode =
                newSelectedIndex == 1 ? GuidanceMode.Manual :
                newSelectedIndex == 2 ? GuidanceMode.Autopilot :
                GuidanceMode.None;

            if (newMode != _launchHandler.GuidanceMode) _launchHandler.SetGuidanceMode(newMode, FlightGlobals.ActiveVessel);
        }

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
    }
}
