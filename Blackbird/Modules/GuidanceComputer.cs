using System;
using Blackbird.Enums;
using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private static readonly int WindowId = "Blackbird.GuidanceComputer".GetHashCode();
        private Rect _windowRect = new Rect(560, 620, 380, 300);
        private string _pitchInputText = "";
        private string _headingInputText = "90";
        private string _rollInputText = "90";
        private string _throttleInputText = "0";
        private bool _showAdvancedDetails;
        private const double MinSecondsToUseWarp = 10.0;
        private readonly string[] _guidanceModeLabels = { "None", "Manual", "Autopilot" };

        private LaunchHandler _launchHandler;
        public bool IsVisible { get; set; }

        public void Toggle() => IsVisible = !IsVisible;
        public void Initialize(LaunchHandler handler) => _launchHandler = handler;

        public void Draw()
        {
            if (!IsVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
        }

        private void DrawContents(int _)
        {
            // null guard before any State access
            if (_launchHandler == null)
            {
                GUILayout.Label("Guidance unavailable");
                GUI.DragWindow();
                return;
            }

            if (_launchHandler.State != LaunchGuidanceState.GuidingAscent)
            {
                if (_launchHandler.CurrentPlan == null)
                {
                    GUILayout.Label("No plan loaded");
                    GUI.DragWindow();
                    return;
                }

                LaunchWindowInfo lw = _launchHandler.CurrentPlan.LaunchWindow;
                if (lw != null)
                {
                    GUILayout.Label("[Launch Window]");
                    GUILayout.Label($"Asc Node Lon: {lw.AscendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Desc Node Lon: {lw.DescendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Time to Asc: {lw.TimeToAscendingNodeSeconds:F0}s");
                    GUILayout.Label($"Time to Desc: {lw.TimeToDescendingNodeSeconds:F0}s");
                    GUILayout.Label($"Selected Offset: {lw.PlaneOffsetDeg:F2}°");
                }

                double countdown = GetDisplayedLaunchCountdownSeconds(_launchHandler.CurrentPlan);
                GUILayout.Label(double.IsNaN(countdown) ? "T- -- seconds" : $"T- {countdown:F0} seconds");

                GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanAccepted && countdown >= MinSecondsToUseWarp;
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
                GUI.DragWindow();
                return;
            }

            AscentGuidanceInfo guidanceInfo = _launchHandler.GuidanceInfo;

            DrawSelectGuidanceMethod();
            GUILayout.Space(10);

            string gMode = _launchHandler.GuidanceMode == GuidanceMode.Autopilot ? "Autopilot" :
                           _launchHandler.GuidanceMode == GuidanceMode.Manual ? "Manual" : "None";
            GUILayout.Label($"Mode: {gMode}");
            GUILayout.Label($"Flight Status: {guidanceInfo.GuidancePhase}");

            if (_launchHandler.GuidanceMode == GuidanceMode.Manual)
            {
                GUILayout.Label("[Manual Control]");
                GUILayout.Space(10);

                // PITCH
                GUILayout.Label($"[Pitch - {guidanceInfo.CommandPitchDeg:F2}°]", GUILayout.Width(70));
                GUILayout.BeginHorizontal();
                _pitchInputText = GUILayout.TextField(_pitchInputText, GUILayout.Width(100));
                double.TryParse(_pitchInputText, out double pitch);
                if (GUILayout.Button("Exct.")) _launchHandler.SetPitchCommand(pitch);
                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualPitchCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualPitchCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetPitchCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(5);

                // HEADING
                GUILayout.Label($"[Heading - {guidanceInfo.CommandHeadingDeg:F2}°]", GUILayout.Width(70));
                GUILayout.BeginHorizontal();
                _headingInputText = GUILayout.TextField(_headingInputText, GUILayout.Width(100));
                double.TryParse(_headingInputText, out double hdg);
                if (GUILayout.Button("Exct.")) _launchHandler.SetHeadingCommand(hdg);
                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualHeadingCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualHeadingCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetHeadingCommand();
                GUILayout.EndHorizontal();

                // ROLL
                GUILayout.BeginHorizontal();
                GUILayout.Label($"[Roll - {guidanceInfo.CommandRoll:F2}°]", GUILayout.Width(70));
                _rollInputText = GUILayout.TextField(_rollInputText, GUILayout.Width(100));
                double.TryParse(_rollInputText, out double roll);
                if (GUILayout.Button("Exct.")) _launchHandler.SetRollCommand(roll);
                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualRollCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualRollCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetRollCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(5);

                // THROTTLE
                GUILayout.BeginHorizontal();
                GUILayout.Label($"[Throttle - {BlackbirdHelpers.FormatThrottle(guidanceInfo.CommandThrottle)}%]", GUILayout.Width(70));
                _throttleInputText = GUILayout.TextField(_throttleInputText, GUILayout.Width(100));
                double.TryParse(_throttleInputText, out double thtl);
                if (GUILayout.Button("Exct.")) _launchHandler.SetThrottleCommand(thtl);
                if (GUILayout.Button(" - ")) _launchHandler.DecreaseManualThrottleCommand();
                if (GUILayout.Button(" + ")) _launchHandler.IncreaseManualThrottleCommand();
                if (GUILayout.Button("Reset")) _launchHandler.ResetThrottleCommand();
                GUILayout.EndHorizontal();
            }
            else if (_launchHandler.GuidanceMode == GuidanceMode.Autopilot)
            {
                GUILayout.Label("[Guidance]");
                GUILayout.Label($"Status: {guidanceInfo.GuidanceOptimizerStatus}");

                GUILayout.Space(10);

                GUILayout.Label("[Flight]");
                GUILayout.BeginHorizontal();
                GUILayout.Label("", GUILayout.Width(80));
                GUILayout.Label("Profile", GUILayout.Width(60));
                GUILayout.Label("PSG cmd (actual)", GUILayout.Width(110));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Pitch", GUILayout.Width(80));
                GUILayout.Label(double.IsNaN(guidanceInfo.ProfilePitchDeg) ? "N/A" : $"{guidanceInfo.ProfilePitchDeg:F1}°", GUILayout.Width(60));
                GUILayout.Label(double.IsNaN(guidanceInfo.CommandPitchDeg) ? "N/A" : $"{guidanceInfo.CommandPitchDeg:F1}° ({guidanceInfo.CurrentPitchDeg:F1}°)", GUILayout.Width(110));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Heading", GUILayout.Width(80));
                GUILayout.Label(double.IsNaN(guidanceInfo.ProfileHeadingDeg) ? "N/A" : $"{guidanceInfo.ProfileHeadingDeg:F1}°", GUILayout.Width(60));
                GUILayout.Label(double.IsNaN(guidanceInfo.CommandHeadingDeg) ? "N/A" : $"{guidanceInfo.CommandHeadingDeg:F1}° ({guidanceInfo.CurrentHeadingDeg:F1}°)", GUILayout.Width(110));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Throttle", GUILayout.Width(80));
                GUILayout.Label(double.IsNaN(guidanceInfo.ProfileThrottle) ? "N/A" : $"{guidanceInfo.ProfileThrottle * 100:F0}%", GUILayout.Width(60));
                GUILayout.Label(double.IsNaN(guidanceInfo.CommandThrottle) ? "N/A" : $"{guidanceInfo.CommandThrottle * 100:F0}%", GUILayout.Width(110));
                GUILayout.EndHorizontal();

                GUILayout.Space(10);

                GUILayout.BeginHorizontal();
                GUILayout.Label("", GUILayout.Width(80));
                GUILayout.Label("Target", GUILayout.Width(60));
                GUILayout.Label("Predicted", GUILayout.Width(60));
                GUILayout.Label("Dev.", GUILayout.Width(60));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Apoapsis", GUILayout.Width(80));
                GUILayout.Label(FormatKm(guidanceInfo.TargetApoapsisAlt, "F0"), GUILayout.Width(60));
                GUILayout.Label(FormatKm(guidanceInfo.PredictedApoapsisAlt, "F0"), GUILayout.Width(60));
                GUILayout.Label(FormatKm(guidanceInfo.ApoapsisErrorMeters, "F1"), GUILayout.Width(60));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Periapsis", GUILayout.Width(80));
                GUILayout.Label(FormatKm(guidanceInfo.TargetPeriapsisAlt, "F0"), GUILayout.Width(60));
                GUILayout.Label(FormatKm(guidanceInfo.PredictedPeriapsisAlt, "F0"), GUILayout.Width(60));
                GUILayout.Label(FormatKm(guidanceInfo.PeriapsisErrorMeters, "F1"), GUILayout.Width(60));
                GUILayout.EndHorizontal();

                PhasingOrbit phasing = _launchHandler.CurrentPlan?.PhasingOrbit;
                if (phasing != null)
                {
                    GUILayout.Label(phasing.IsFasterThanTarget
                        ? "Phasing: insertion orbit is faster than target"
                        : "Phasing: insertion orbit is slower than target");
                    if (phasing.HasRendezvousEstimate)
                    {
                        GUILayout.Label($"Rendezvous Orbits: {phasing.EstimatedOrbitsToRendezvous:F1}");
                        GUILayout.Label($"Rendezvous Time: {BlackbirdHelpers.FormatDuration(phasing.EstimatedTimeToRendezvousSeconds)}");
                    }
                    else
                    {
                        GUILayout.Label("Rendezvous Estimate: unavailable");
                    }
                }

                GUILayout.Label($"Rmg. Velocity: {FormatNum(guidanceInfo.GuidanceVelocityToGoMetersPerSecond, "F0", "m/s")}");
                GUILayout.Label($"Rmg. Time: {FormatNum(guidanceInfo.GuidanceTimeToGoSeconds, "F1", "s")}");
                GUILayout.Label($"Rmg. dV: {FormatNum(guidanceInfo.VesselRemainingDeltaV, "F0", "m/s")}");
                GUILayout.Label($"Phase Error: {FormatNum(guidanceInfo.PhaseErrorDeg, "F2", "°")}");
                GUILayout.Label($"Plane Error: {FormatNum(guidanceInfo.PlaneErrorDeg, "F2", "°")}");

                _showAdvancedDetails = GUILayout.Toggle(_showAdvancedDetails, "Show Advanced Details");
                if (_showAdvancedDetails)
                    DrawAdvancedDetails(_launchHandler.CurrentPlan, _launchHandler.CurrentPlan.TargetVessel);
            }

            GUI.DragWindow();
        }

        private void DrawAdvancedDetails(LaunchPlan launchPlan, Vessel targetVessel)
        {
            if (launchPlan.ActiveOrbit == null)
            {
                GUILayout.Label("Orbit details unavailable for manual plans.");
                return;
            }

            GUILayout.Space(10);
            GUILayout.Label("-- Active Orbit --");
            GUILayout.Label($"Inclination: {launchPlan.ActiveOrbit.InclinationDeg:F2}°");
            GUILayout.Label($"LAN: {launchPlan.ActiveOrbit.LanDeg:F2}°");
            GUILayout.Label($"Apoapsis: {launchPlan.ActiveOrbit.ApoapsisAlt / 1000:F0} km");
            GUILayout.Label($"Periapsis: {launchPlan.ActiveOrbit.PeriapsisAlt / 1000:F0} km");
            GUILayout.Label($"Period: {launchPlan.ActiveOrbit.PeriodSeconds:F1}s");

            if (targetVessel != null && launchPlan.TargetOrbit != null)
            {
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
            }

            if (launchPlan.PhasingOrbit != null)
            {
                GUILayout.Space(10);
                GUILayout.Label("-- Phasing Period --");
                GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferenceSeconds:F1}s");
                GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferenceMinutes:F2} min");
                GUILayout.Label($"Period Diff: {launchPlan.PhasingOrbit.PeriodDifferencePercent:F3}%");
                GUILayout.Label($"Phase Gain: {launchPlan.PhasingOrbit.RelativePhaseGainDegPerOrbit:F2}°/orbit");

                if (launchPlan.PhasingOrbit.HasRendezvousEstimate)
                {
                    GUILayout.Label($"Rendezvous Orbits: {launchPlan.PhasingOrbit.EstimatedOrbitsToRendezvous:F1}");
                    GUILayout.Label($"Rendezvous Time: {BlackbirdHelpers.FormatDuration(launchPlan.PhasingOrbit.EstimatedTimeToRendezvousSeconds)}");
                }
                else
                {
                    GUILayout.Label("Rendezvous Estimate: unavailable");
                }

                GUILayout.Label(launchPlan.PhasingOrbit.IsFasterThanTarget
                    ? "Phasing: insertion orbit is faster than target"
                    : "Phasing: insertion orbit is slower than target");
            }

            GUILayout.Space(10);
            GUILayout.Label("-- Phasing Recommendation Details --");
            PhasingRecommendation recommendation = launchPlan.PhasingRecommendation;
            if (recommendation != null && recommendation.HasRecommendation)
            {
                GUILayout.Label($"Period Diff: {recommendation.PeriodDifferenceSeconds:N1}s");
                GUILayout.Label($"Phase Gain: {recommendation.PhaseGainDegPerOrbit:N2}°/orbit");
                if (launchPlan.TargetOrbit != null)
                    GUILayout.Label("Offset: " + ((recommendation.ApoapsisAlt - launchPlan.TargetOrbit.ApoapsisAlt) / 1000.0).ToString("N0") + " km");
            }
            else
            {
                GUILayout.Label("Unavailable");
            }

            if (launchPlan.LaunchWindow != null)
            {
                GUILayout.Space(10);
                GUILayout.Label("-- Launch Window Details --");
                GUILayout.Label($"Asc Node Lon: {launchPlan.LaunchWindow.AscendingNodeLongitudeDeg:F2}°");
                GUILayout.Label($"Desc Node Lon: {launchPlan.LaunchWindow.DescendingNodeLongitudeDeg:F2}°");
                GUILayout.Label($"Time to Asc: {launchPlan.LaunchWindow.TimeToAscendingNodeSeconds:F0}s");
                GUILayout.Label($"Time to Desc: {launchPlan.LaunchWindow.TimeToDescendingNodeSeconds:F0}s");
                GUILayout.Label($"Selected Offset: {launchPlan.LaunchWindow.PlaneOffsetDeg:F2}°");
            }
        }

        private void DrawSelectGuidanceMethod()
        {
            GUILayout.Space(10);
            GUILayout.Label("Flight Mode");

            int selectedIndex =
                _launchHandler.GuidanceMode == GuidanceMode.Manual ? 1 :
                _launchHandler.GuidanceMode == GuidanceMode.Autopilot ? 2 :
                0;

            int newSelectedIndex = GUILayout.SelectionGrid(selectedIndex, _guidanceModeLabels, 3);

            GuidanceMode newMode =
                newSelectedIndex == 1 ? GuidanceMode.Manual :
                newSelectedIndex == 2 ? GuidanceMode.Autopilot :
                GuidanceMode.None;

            if (newMode != _launchHandler.GuidanceMode) _launchHandler.SetGuidanceMode(newMode, FlightGlobals.ActiveVessel);
        }

        // Formats a metre value as kilometres, or "N/A" when the source metric is unavailable
        // (e.g. PSG-only fields that the classic/Stock guidance path never populates).
        private static string FormatKm(double meters, string format) =>
            double.IsNaN(meters) || double.IsInfinity(meters)
                ? "N/A"
                : $"{(meters / 1000.0).ToString(format)} km";

        // Formats a numeric metric with a unit, or "N/A" when it isn't available.
        private static string FormatNum(double value, string format, string unit) =>
            double.IsNaN(value) || double.IsInfinity(value)
                ? "N/A"
                : $"{value.ToString(format)} {unit}";

        private double GetDisplayedLaunchCountdownSeconds(LaunchPlan launchPlan)
        {
            if (_launchHandler.State == LaunchGuidanceState.WarpingToLaunch ||
                _launchHandler.State == LaunchGuidanceState.AwaitingLaunch ||
                _launchHandler.State == LaunchGuidanceState.GuidingAscent)
            {
                return Math.Max(0.0, _launchHandler.SecondsUntilLaunch);
            }

            // PlanAccepted: _targetUt not set yet — compute live from the selected candidate's LaunchUt.
            if (_launchHandler.State == LaunchGuidanceState.PlanAccepted)
            {
                LaunchCandidate selected = launchPlan?.SelectedCandidate;
                if (selected != null && !double.IsNaN(selected.LaunchUt))
                    return Math.Max(0.0, selected.LaunchUt - Planetarium.GetUniversalTime());
            }

            return launchPlan?.LaunchWindow != null
                ? launchPlan.LaunchWindow.TimeToPlaneCrossingSeconds
                : double.NaN;
        }
    }
}
