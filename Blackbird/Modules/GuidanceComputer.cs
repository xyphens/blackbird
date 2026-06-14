using Blackbird.Enums;
using Blackbird.Guidance;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private static readonly int WindowId = "Blackbird.GuidanceComputer".GetHashCode();
        private Rect _windowRect = new Rect(560, 620, 380, 300);
        public bool IsVisible { get; set; }

        public void Toggle() => IsVisible = !IsVisible;

        public void Draw()
        {
            if (!IsVisible) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
        }
        // TODO: implement this
        private void DrawContents(int _windowId)
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
    }
}
