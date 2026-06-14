using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;
using Blackbird.Trajectory;
using UnityEngine;
using static alglib;

namespace Blackbird.Modules
{
    public sealed class Planner
    {
        private static readonly int WindowId = "Blackbird.Planner".GetHashCode();
        private Rect _windowRect = new Rect(560, 200, 420, 500);

        private string _insertionApText = "";
        private string _insertionPeText = "";
        private string _insertionHdgText = "";
        private string _launchTimeText = "";

        private bool _showInsertionOptions = false;
        private LaunchPlan _selectedPlan;
        private LaunchPlan _cachedCandidates;

        private LaunchHandler _launchHandler;

        public bool IsVisible { get; set; }

        public void Initialize(LaunchHandler handler)
        {
            _launchHandler = handler;
        }

        public void Toggle() => IsVisible = !IsVisible;

        public void Draw()
        {
            if (!IsVisible) return;
            if (FlightGlobals.ActiveVessel == null) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Planner");
        }

        private void DrawContents(int _)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null) { GUI.DragWindow(); return; }

            // -- TARGET SECTION --
            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target is Vessel targetVessel &&
                !ReferenceEquals(vessel, targetVessel) &&
                vessel.id != targetVessel.id)
            {
                GUILayout.Label($"Target: {targetVessel.vesselName}");

                bool prevShow = _showInsertionOptions;
                _showInsertionOptions = GUILayout.Toggle(_showInsertionOptions, "Show Insertion Options");

                if (!_showInsertionOptions && prevShow)
                {
                    _cachedCandidates = null;
                    _launchTimeText = "";
                }

                if (_showInsertionOptions)
                {
                    if (_cachedCandidates == null)
                        GenerateCachedPlan(vessel, targetVessel);

                    if (_cachedCandidates != null)
                        DisplayLaunchPlanCandidates();
                }
            }
            else if (_showInsertionOptions)
            {
                _showInsertionOptions = false;
                _cachedCandidates = null;
                _launchTimeText = "";
            }

            GUILayout.Space(10);

            // -- USER INPUTS / EDITS --
            GUILayout.BeginHorizontal();
            GUILayout.Label("Launch At:", GUILayout.Width(70));
            GUILayout.Label(string.IsNullOrEmpty(_launchTimeText) ? "--" : _launchTimeText);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Apoapsis:", GUILayout.Width(70));
            _insertionApText = GUILayout.TextField(_insertionApText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Periapsis:", GUILayout.Width(70));
            _insertionPeText = GUILayout.TextField(_insertionPeText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Heading:", GUILayout.Width(70));
            _insertionHdgText = GUILayout.TextField(_insertionHdgText, GUILayout.Width(100));
            GUILayout.Label("°");
            GUILayout.EndHorizontal();

            GUILayout.Space(5);

            bool canCommit = _launchHandler != null &&
                (_launchHandler.State == LaunchGuidanceState.Idle || _launchHandler.State == LaunchGuidanceState.PlanReady);


            GUI.enabled = canCommit;
            if (GUILayout.Button("Accept Plan"))
                CommitPlanInputs(vessel);
            GUI.enabled = true;

            GUI.DragWindow();
        }

        private void DisplayLaunchPlanCandidates()
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Candidates]");

            if (_cachedCandidates?.Candidates == null || _cachedCandidates.Candidates.Length == 0)
            {
                GUILayout.Label("No launch candidates.");
                return;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Launch In", GUILayout.Width(80));
            GUILayout.Label("Ap", GUILayout.Width(45));
            GUILayout.Label("Pe", GUILayout.Width(45));
            GUILayout.Label("Hdg", GUILayout.Width(50));
            GUILayout.Label("Num Orbits", GUILayout.Width(40));
            GUILayout.Label("dV Start", GUILayout.Width(45));
            GUILayout.Label("dv End", GUILayout.Width(45));
            GUILayout.Label("-", GUILayout.Width(45));
            GUILayout.EndHorizontal();

            for (int i = 0; i < _cachedCandidates.Candidates.Length; i++)
            {
                LaunchCandidate candidate = _cachedCandidates.Candidates[i];

                GUILayout.BeginHorizontal();

                GUILayout.Label(
                    candidate.IsValid ? BlackbirdHelpers.FormatDuration(candidate.SecondsUntilLaunch) : "N/A",
                    GUILayout.Width(80));
                GUILayout.Label(FormatKm(candidate.InsertionApoapsisAlt), GUILayout.Width(45));
                GUILayout.Label(FormatKm(candidate.InsertionPeriapsisAlt), GUILayout.Width(45));
                GUILayout.Label(FormatValue(candidate.LaunchHeadingDeg, "F1"), GUILayout.Width(50));
                GUILayout.Label(FormatValue(candidate.EstimatedOrbitsToRendezvous, "F1"), GUILayout.Width(40));
                GUILayout.Label(FormatValue(candidate.EstimatedDeltaVUsed, "F0"), GUILayout.Width(45));
                GUILayout.Label(FormatValue(candidate.EstimatedRemainingDeltaV, "F0"), GUILayout.Width(45));

                // start choose button ...
                bool isSelected = _selectedPlan == _cachedCandidates && _cachedCandidates.SelectedCandidateIndex == i;
                bool canChoose = candidate.IsValid && _launchHandler != null &&
                    (_launchHandler.State == LaunchGuidanceState.Idle ||
                     _launchHandler.State == LaunchGuidanceState.PlanReady);

                GUI.enabled = canChoose && !isSelected;
                if (GUILayout.Button(isSelected ? "Active" : "Choose", GUILayout.Width(60))) SelectCandidate(_cachedCandidates, i);
                // ... end choose button
                GUI.enabled = true;

                GUILayout.EndHorizontal();

                if (!candidate.IsValid && !string.IsNullOrEmpty(candidate.ReasonUnavailable))
                    GUILayout.Label(candidate.ReasonUnavailable);
            }
        }

        private void SelectCandidate(LaunchPlan launchPlan, int index)
        {
            if (launchPlan?.Candidates == null) return;
            if (index < 0 || index >= launchPlan.Candidates.Length) return;

            launchPlan.SelectedCandidateIndex = index;
            _selectedPlan = launchPlan;

            LaunchCandidate c = launchPlan.Candidates[index];
            _insertionApText = c.InsertionApoapsisAlt.ToString("F0");
            _insertionPeText = c.InsertionPeriapsisAlt.ToString("F0");
            _insertionHdgText = c.LaunchHeadingDeg.ToString("F1");
            _launchTimeText = BlackbirdHelpers.FormatDuration(c.SecondsUntilLaunch);
        }

        private void CommitPlanInputs(Vessel vessel)
        {
            if (_launchHandler == null || vessel == null) return;

            InsertionTarget it = CreateInsertionTargetFromUi();
            double launchUt = _selectedPlan?.SelectedCandidate?.LaunchUt ?? double.NaN;

            _launchHandler.ConstructLaunchPlan(vessel, it.ApoapsisAlt, it.PeriapsisAlt, it.Heading, launchUt);
            _launchHandler.AcceptPlan();
        }

        private void GenerateCachedPlan(Vessel vessel, Vessel targetVessel)
        {
            LaunchLocation ll = LaunchLocation.FromVessel(vessel);
            InsertionTarget targetIt = new InsertionTarget
            {
                ApoapsisAlt = TrajectoryProvider.GetApoapsisAlt(targetVessel),
                PeriapsisAlt = TrajectoryProvider.GetPeriapsisAlt(targetVessel),
                Heading = 0
            };
            _cachedCandidates = LaunchPlanner.Create(vessel, targetVessel, targetIt, ll);
        }

        private InsertionTarget CreateInsertionTargetFromUi()
        {
            double.TryParse(_insertionApText, out double ap);
            double.TryParse(_insertionPeText, out double pe);
            double.TryParse(_insertionHdgText, out double hdg);

            if (pe > ap) { double t = ap; ap = pe; pe = t; }

            return new InsertionTarget { ApoapsisAlt = ap, PeriapsisAlt = pe, Heading = hdg };
        }

        private static string FormatKm(double meters) =>
            double.IsNaN(meters) || double.IsInfinity(meters) ? "N/A" : (meters / 1000.0).ToString("F0");

        private static string FormatValue(double value, string format) =>
            double.IsNaN(value) || double.IsInfinity(value) ? "N/A" : value.ToString(format);
    }
}
