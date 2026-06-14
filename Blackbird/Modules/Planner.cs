using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;
using Blackbird.Trajectory;
using UnityEngine;
using System;


namespace Blackbird.Modules
{
    public sealed class Planner
    {
        private static readonly int WindowId = "Blackbird.Planner".GetHashCode();
        private Rect _windowRect = new Rect(560, 200, 380, 500);
        private string _insertionApText = "";
        private string _insertionPeText = "";
        private string _insertionHdgText = "";
        private Vessel CurrentVessel;
        private Vessel TargetVessel;

        // launch plan
        private readonly LaunchHandler _launchHandler = new LaunchHandler();
        private bool _showInsertionOptions = false;
        private LaunchPlan _currentPlan;
        private LaunchPlan _selectedPlan;
        //private LaunchPlan _cachedLaunchPlan;
        private LaunchPlan _cachedCandidates;
        // timeouts so we don't lag the game w/ constant re-calcs
        private double _lastPlanComputeTime = double.NegativeInfinity;
        private const double PlanRecomputeIntervalSeconds = 2.0;

        public bool IsVisible { get; set; }

        public void Toggle() => IsVisible = !IsVisible;

        public void Draw()
        {
            if (!IsVisible || CurrentVessel == null) return;
            if (CurrentVessel == null) return;
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Planner");

        }

        private void DrawContents(int _windowId)
        {
            GUILayout.Label("[Planner]");

            // check and set if we have a target selected
            ITargetable target = FlightGlobals.fetch.VesselTarget;

            if (target is Vessel targetVessel)
            {
                if (!ReferenceEquals(CurrentVessel, targetVessel) && CurrentVessel.id != targetVessel.id)
                {
                    TargetVessel = targetVessel;
                    GUILayout.Label($"Target: {TargetVessel.vesselName}");
                    _showInsertionOptions = GUILayout.Toggle(_showInsertionOptions, "Show Insertion Options");
                }
            }

            bool canPlan = _launchHandler.State == LaunchGuidanceState.Idle || _launchHandler.State == LaunchGuidanceState.PlanReady;
            double now = Planetarium.GetUniversalTime();

            if (_showInsertionOptions)
            {
                
                if (TargetVessel != null && canPlan && _showInsertionOptions && (now - _lastPlanComputeTime >= PlanRecomputeIntervalSeconds || _cachedCandidates == null))
                {
                    _lastPlanComputeTime = now;
                    GenerateCachedPlan();
                    if (_cachedCandidates != null)
                    {
                        // render the table of launch plans
                        // clicking "Select" will populate our text inputs
                        DisplayLaunchPlanCandidates();
                    }
                }
            } else
            {
                // allows toggle/reload cached
                _cachedCandidates = null;
            }


            // Ap input
            GUILayout.BeginHorizontal();
            GUILayout.Label("Apoapsis:", GUILayout.Width(40));
            _insertionApText = GUILayout.TextField(_insertionApText, GUILayout.Width(100));
            GUILayout.Label("km");
            GUILayout.EndHorizontal();

            // Pe input
            GUILayout.BeginHorizontal();
            GUILayout.Label("Periapsis:", GUILayout.Width(40));
            _insertionPeText = GUILayout.TextField(_insertionPeText, GUILayout.Width(100));
            GUILayout.Label("km");
            GUILayout.EndHorizontal();

            // heading input
            GUILayout.BeginHorizontal();
            GUILayout.Label("Heading:", GUILayout.Width(40));
            _insertionPeText = GUILayout.TextField(_insertionHdgText, GUILayout.Width(100));
            GUILayout.Label("°");
            GUILayout.EndHorizontal();

            // commit + create a plan based on the inputs
            GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanReady;
            if (GUILayout.Button("Accept Plan")) CommitPlanInputs();
            _launchHandler.AcceptPlan();
            GUI.enabled = true;

            GUI.DragWindow();
        }

        private void DisplayLaunchPlanCandidates()
        {
            GUILayout.Space(10);
            GUILayout.Label("[Launch Candidates]");

            if (_cachedCandidates == null || _cachedCandidates.Candidates == null || _cachedCandidates.Candidates.Length == 0)
            {
                GUILayout.Label("No launch candidates.");
                return;
            }

            GUILayout.BeginHorizontal();

            GUILayout.Label("Launch In", GUILayout.Width(80));
            GUILayout.Label("Ap", GUILayout.Width(45));
            GUILayout.Label("Pe", GUILayout.Width(45));
            GUILayout.Label("Hdg", GUILayout.Width(50));
            GUILayout.Label("Orb", GUILayout.Width(40));
            GUILayout.Label("Start dV", GUILayout.Width(45));
            GUILayout.Label("End dV", GUILayout.Width(60));
            //GUILayout.Label("Err", GUILayout.Width(45));
            GUILayout.Label("-", GUILayout.Width(60));
            GUILayout.EndHorizontal();

            for (int i = 0; i < _cachedCandidates.Candidates.Length; i++)
            {
                LaunchCandidate candidate = _cachedCandidates.Candidates[i];
                bool isSelected = _cachedCandidates.SelectedCandidateIndex == i;
                bool canChoose =
                    candidate.IsValid &&
                    (_launchHandler.State == LaunchGuidanceState.Idle || _launchHandler.State == LaunchGuidanceState.PlanReady);

                GUILayout.BeginHorizontal();

                GUI.enabled = canChoose && !isSelected;
                if (GUILayout.Button(isSelected ? "Active" : "Choose", GUILayout.Width(60)))
                {
                    SelectCandidate(_cachedCandidates, i);
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
                //GUILayout.Label(FormatValue(candidate.EstimatedRemainingDeltaV, "F0"), GUILayout.Width(60));
                //GUILayout.Label(FormatValue(Math.Abs(candidate.PhaseErrorDeg), "F1"), GUILayout.Width(45));

                GUILayout.EndHorizontal();

                if (!candidate.IsValid && !string.IsNullOrEmpty(candidate.ReasonUnavailable))
                {
                    GUILayout.Label(candidate.ReasonUnavailable);
                }
            }
        }
        // populate the text inputs with the auto-generated plans
        private void SelectCandidate(LaunchPlan launchPlan, int candidateIndex)
        {
            if (launchPlan == null || launchPlan.Candidates == null) return;
            if (candidateIndex < 0 || candidateIndex >= launchPlan.Candidates.Length) return;

            launchPlan.SelectedCandidateIndex = candidateIndex;
            _selectedPlan = launchPlan;

            if (_launchHandler.State == LaunchGuidanceState.Idle || _launchHandler.State == LaunchGuidanceState.PlanReady)
            {
                _insertionApText = launchPlan.RecommendedApAlt.ToString("F0");
                _insertionPeText = launchPlan.RecommendedPeAlt.ToString("F0");
                _insertionHdgText = launchPlan.LaunchAzimuthDeg.ToString("F1");
            }
        }

        // generate a real launch plan from the provided inputs
        private void CommitPlanInputs()
        {
            InsertionTarget it = CreateInsertionTargetFromUi();
            if (_selectedPlan != null)
            {
                // using a launch candidate
                LaunchPlan launchPlan = _selectedPlan;
                // augment selected plan (if there is one) with launch details
            } else
            {
                // flying from scratch
            }
        }

        // generate a table of options for user to select, the selected one will prefill _insertion variables
        private void GenerateCachedPlan()
        {
            // create InsertionTarget from the targetVessel rather than the inputs
            if (CurrentVessel == null || TargetVessel == null) return;
            LaunchLocation ll = LaunchLocation.FromVessel(CurrentVessel);
            _cachedCandidates = LaunchPlanner.Create(CurrentVessel, TargetVessel, GetTargetInsertionTarget(), ll);
        }

        private InsertionTarget GetTargetInsertionTarget()
        {
            if (TargetVessel == null) return null;

            return new InsertionTarget
            {
                ApoapsisAlt = TrajectoryProvider.GetApoapsisAlt(TargetVessel),
                PeriapsisAlt = TrajectoryProvider.GetPeriapsisAlt(TargetVessel),
                Heading = 0 // do not need Heading when generating
            };
        }

        private InsertionTarget CreateInsertionTargetFromUi()
        {
            double ap;
            double pe;
            double hdg;

            bool validAp = double.TryParse(_insertionApText, out ap);
            bool validPe = double.TryParse(_insertionPeText, out pe);
            bool validHdg = double.TryParse(_insertionHdgText, out hdg);

            if (pe > ap)
            {
                double temp = ap;
                ap = pe;
                pe = temp;
            }

            return new InsertionTarget
            {
                ApoapsisAlt = ap,
                PeriapsisAlt = pe,
                Heading = hdg
            };
        }
        private static string FormatKm(double meters)
            {
                return double.IsNaN(meters) || double.IsInfinity(meters) ? "N/A" : (meters / 1000.0).ToString("F0");
            }

            private static string FormatValue(double value, string format)
            {
                return double.IsNaN(value) || double.IsInfinity(value) ? "N/A" : value.ToString(format);
            }
        }
}
