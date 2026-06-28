using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Mathematics;
using Blackbird.Models;
using System;
using UnityEngine;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private SharedState bbState;
        private static readonly int WindowId = "Blackbird.GuidanceComputer".GetHashCode();
        private Rect _windowRect = new Rect(560, 620, 380, 300);
        private string _pitchInputText = "90";
        private string _headingInputText = "90";
        private string _rollInputText = "90";
        private string _throttleInputText = "0";
        private bool _showAdvancedDetails;
        private const double MinSecondsToUseWarp = 10.0;
        private readonly string[] _guidanceModeLabels = { "None", "Manual", "Autopilot" };

        private const float BtnW = 60f;
        private const float BtnH = 34f;

        bool _wasVisible = false; // used to reset window height after closed

        private LaunchHandler _launchHandler;
        public void Init(LaunchHandler handler, SharedState s)
        {
             _launchHandler = handler;
            bbState = s;
        }

        public void Draw()
        {
            if (bbState.GuidanceVisible && !_wasVisible)
            {
                _windowRect.height = 0f;
                _wasVisible = true;
                _showAdvancedDetails = false;
            }

            if (bbState == null || !bbState.GuidanceVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
        }

        private void DrawContents(int _)
        {
            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), " ")) bbState.GuidanceVisible = false;
            // null guard before any State access
            if (bbState.LaunchPlan == null)
            {
                GUILayout.Label("Guidance unavailable - select a flight plan");
                GUI.DragWindow();
                return;
            }

            // Finished flight: show the insertion result, not the pre-launch arming details.
            if (_launchHandler.State == LaunchGuidanceState.Complete)
            {
                DrawGuidanceResult(_launchHandler.GuidanceInfo);
                GUI.DragWindow();
                return;
            }

            Orbit orbit = FlightGlobals.ActiveVessel.orbit;

            if (_launchHandler.TargetVessel != null) {
                double relInc = OrbitMath.GetRelativeInclination(FlightGlobals.ActiveVessel, _launchHandler.TargetVessel);
                GUILayout.Label($"Rel. Inclination: {relInc:F2}°");
                GUILayout.Label($"Inclination: {orbit.inclination:F2}° vs {_launchHandler.TargetVessel.orbit.inclination:F2}°");
                GUILayout.Label($"RAAN (LAN): {orbit.LAN:F2}° vs {_launchHandler.TargetVessel.orbit.LAN:F2}°");
            }

            if (_launchHandler.State != LaunchGuidanceState.GuidingAscent)
            {
                LaunchWindowInfo lw = bbState.LaunchPlan.LaunchWindow;
                if (lw != null)
                {
                    GUILayout.Label($"Asc Node Lon: {lw.AscendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Desc Node Lon: {lw.DescendingNodeLongitudeDeg:F2}°");
                    GUILayout.Label($"Time to Asc: {lw.TimeToAscendingNodeSeconds:F0}s");
                    GUILayout.Label($"Time to Desc: {lw.TimeToDescendingNodeSeconds:F0}s");
                    GUILayout.Label($"Selected Offset: {lw.PlaneOffsetDeg:F2}°");
                }

                double countdown = GetDisplayedLaunchCountdownSeconds(bbState.LaunchPlan);
                GUILayout.Label(double.IsNaN(countdown) ? "T- -- seconds" : $"T- {countdown:F0} seconds");

                // Armed: warp to the window, then pick a flight mode (which begins the ascent).
                GUI.enabled = countdown >= MinSecondsToUseWarp;
                if (GUILayout.Button("Warp To Launch")) _launchHandler.WarpToLaunch();
                GUI.enabled = true;

                DrawSelectGuidanceMethod();
                // Begin the ascent once a flight mode is chosen AND we're at the launch window (countdown within
                // the warp-stop lead). Gating on the countdown lets the operator pick a mode and then Warp To
                // Launch: while the countdown is large, BeginAscent holds off (so it can't zero the warp rate
                // each frame); the warp stops at the window, the countdown drops into the lead, and ascent begins.
                if (_launchHandler.GuidanceMode != GuidanceMode.None && countdown <= MinSecondsToUseWarp)
                {
                    _launchHandler.BeginAscent();
                }

                GUILayout.BeginHorizontal();
                _launchHandler.MinVSpeedToPitch = GUILayout.TextField(_launchHandler.MinVSpeedToPitch, GUILayout.Width(50));
                GUILayout.Label("m/s", GUILayout.Width(50));
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                GUILayout.BeginHorizontal();
                _launchHandler.MinAltitudeForPitch = GUILayout.TextField(_launchHandler.MinAltitudeForPitch, GUILayout.Width(50));
                GUILayout.Label("m", GUILayout.Width(50));
                GUILayout.EndHorizontal();

                bool armed = _launchHandler.State == LaunchGuidanceState.AwaitingLaunch
                          || _launchHandler.State == LaunchGuidanceState.WarpingToLaunch;

                if (!armed)
                {
                    // arm the launch
                    GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanAccepted
                                  && bbState.CanClaimControl(BlackbirdModule.LaunchGuidance);
                    if (GUILayout.Button("Start Guidance")) _launchHandler.StartGuidance();
                }

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

            double ascentCountdown = GetDisplayedLaunchCountdownSeconds(bbState.LaunchPlan);
            GUILayout.Label(double.IsNaN(ascentCountdown) ? "T- -- seconds" : $"T- {ascentCountdown:F0} seconds");

            DrawSelectGuidanceMethod();

            GUILayout.Space(10);

            string gMode = _launchHandler.GuidanceMode == GuidanceMode.Autopilot ? "Autopilot" :
               _launchHandler.GuidanceMode == GuidanceMode.Manual ? "Manual" : "None";
            GUILayout.Label($"Mode: {gMode}");

            GUILayout.Space(8);

            GUILayout.Label($"Time to Apoapsis: {BlackbirdHelpers.FormatDuration(orbit.timeToAp)}");
            GUILayout.Label($"Time to Periapsis: {BlackbirdHelpers.FormatDuration(orbit.timeToPe)}");
            GUILayout.Label($"Orbital period: {BlackbirdHelpers.FormatDuration(orbit.period)}");
            GUILayout.Label($"Orbital velocity: {orbit.orbitalSpeed:F1} m/s"); // fixme: bugged/doesn't during ascent
            GUILayout.Label($"Semi-major axis: {orbit.semiMajorAxis / 1000.0:F1} km");
            GUILayout.Label($"Eccentricity: {orbit.eccentricity:F4}");

            GUILayout.Space(8);
            bbState.LockRollOnAscent = GUILayout.Toggle(bbState.LockRollOnAscent, "Lock Roll at 0°");
            GUILayout.Space(8);
            GUILayout.Label($"Flight Status: {guidanceInfo.GuidancePhase}");

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
            if (_launchHandler.GuidanceMode == GuidanceMode.Manual)
            {
                // PITCH
                GUILayout.Label($"Pitch: {guidanceInfo.CommandPitchDeg:F2}°");
                GUILayout.BeginHorizontal();
                _pitchInputText = GUILayout.TextField(_pitchInputText, GUILayout.Width(50));
                double.TryParse(_pitchInputText, out double pitch);
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) _launchHandler.SetPitchCommand(pitch);
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) _launchHandler.DecreaseManualPitchCommand();
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) _launchHandler.IncreaseManualPitchCommand();
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) _launchHandler.ResetPitchCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // HEADING
                GUILayout.Label($"Heading: {guidanceInfo.CommandHeadingDeg:F2}°");
                GUILayout.BeginHorizontal();
                _headingInputText = GUILayout.TextField(_headingInputText, GUILayout.Width(50));
                double.TryParse(_headingInputText, out double hdg);

                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetHeadingCommand(hdg); _headingInputText = _launchHandler.ManualHeadingCommandDeg.ToString("F0"); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualHeadingCommand(); _headingInputText = _launchHandler.ManualHeadingCommandDeg.ToString("F0"); };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualHeadingCommand(); _headingInputText = _launchHandler.ManualHeadingCommandDeg.ToString("F0"); };
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.ResetHeadingCommand(); _headingInputText = _launchHandler.ManualHeadingCommandDeg.ToString("F0"); };
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // ROLL
                GUILayout.Label($"Roll: {guidanceInfo.CommandRoll:F2}°");
                GUILayout.BeginHorizontal();
                _rollInputText = GUILayout.TextField(_rollInputText, GUILayout.Width(50));
                double.TryParse(_rollInputText, out double roll);
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetRollCommand(hdg); _rollInputText = _launchHandler.ManualRollCommand.ToString("F0"); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualRollCommand(); _rollInputText = _launchHandler.ManualRollCommand.ToString("F0"); };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualRollCommand(); _rollInputText = _launchHandler.ManualRollCommand.ToString("F0"); };
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.ResetRollCommand(); _rollInputText = _launchHandler.ManualRollCommand.ToString("F0"); };
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // THROTTLE
                GUILayout.Label($"Throttle: {BlackbirdHelpers.FormatThrottle(guidanceInfo.CommandThrottle)}");
                GUILayout.BeginHorizontal();
                _throttleInputText = GUILayout.TextField(_throttleInputText, GUILayout.Width(50));
                double.TryParse(_throttleInputText, out double thtl);
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetThrottleCommand(hdg); _throttleInputText = _launchHandler.ManualThrottleCommand.ToString("F0"); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualThrottleCommand(); _throttleInputText = _launchHandler.ManualThrottleCommand.ToString("F0"); };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualThrottleCommand(); _throttleInputText = _launchHandler.ManualThrottleCommand.ToString("F0"); };
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.ResetThrottleCommand(); _throttleInputText = _launchHandler.ManualThrottleCommand.ToString("F0"); };
                GUILayout.EndHorizontal();
            }
            else if (_launchHandler.GuidanceMode == GuidanceMode.Autopilot)
            {
                PhasingOrbit phasing = bbState.LaunchPlan?.PhasingOrbit;

                if (phasing != null && bbState?.LaunchPlan?.TargetVessel != null)
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
            }

            _showAdvancedDetails = GUILayout.Toggle(_showAdvancedDetails, "Show Advanced Details");
            if (_showAdvancedDetails)
            {
                DrawAdvancedDetails(bbState.LaunchPlan, bbState.LaunchPlan.TargetVessel);
            }

            GUI.DragWindow();
        }

        private void DrawAdvancedDetails(LaunchPlan launchPlan, Vessel targetVessel)
        {
            GUILayout.Space(10);
            if (launchPlan == null)
            {
                GUILayout.Label("Launch plan not available");
                return;
            }
            
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

        // Insertion result after a completed ascent: target apsides and the achieved error. GuidanceInfo holds
        // the final guidance frame until the plan is aborted or replaced.
        private void DrawGuidanceResult(AscentGuidanceInfo guidanceInfo)
        {
            GUILayout.Label("Ascent complete");
            if (guidanceInfo == null)
            {
                GUILayout.Label("No guidance result available");
                return;
            }

            GUILayout.Space(6);
            GUILayout.BeginHorizontal();
            GUILayout.Label("", GUILayout.Width(80));
            GUILayout.Label("Target", GUILayout.Width(70));
            GUILayout.Label("Error", GUILayout.Width(70));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Apoapsis", GUILayout.Width(80));
            GUILayout.Label(FormatKm(guidanceInfo.TargetApoapsisAlt, "F0"), GUILayout.Width(70));
            GUILayout.Label(FormatKm(guidanceInfo.ApoapsisErrorMeters, "F1"), GUILayout.Width(70));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Periapsis", GUILayout.Width(80));
            GUILayout.Label(FormatKm(guidanceInfo.TargetPeriapsisAlt, "F0"), GUILayout.Width(70));
            GUILayout.Label(FormatKm(guidanceInfo.PeriapsisErrorMeters, "F1"), GUILayout.Width(70));
            GUILayout.EndHorizontal();
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
                // note: not flooring this at zero so i can see if we overshot plan
                return _launchHandler.SecondsUntilLaunch;
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
