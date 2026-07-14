using System;
using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;
using Blackbird.Trajectory;
using UnityEngine;
namespace Blackbird.Modules
{
    public sealed class Planner
    {
        private SharedState bbState;
        private static readonly int WindowId = "Blackbird.Planner".GetHashCode();
        private Rect _windowRect = new Rect(560, 200, 500, 500);

        private double _insertionAp = 0.0;
        public string InsertionAp
        {
            get { return (_insertionAp / 1000).ToString("F0"); }
            set { if (double.TryParse(value, out double v)) _insertionAp = v * 1000; }
        }

        private double _insertionPe = 0.0;
        public string InsertionPe
        {
            get { return (_insertionPe / 1000).ToString("F0"); }
            set { if (double.TryParse(value, out double v)) _insertionPe = v * 1000; }
        }

        private double _insertionHdg = 0.0;
        public string InsertionHdg
        {
            get { return _insertionHdg.ToString("F1"); }
            set { if (double.TryParse(value, out double v)) _insertionHdg = v; }
        }

        private LaunchHandler _launchHandler;

        public void Init(LaunchHandler handler, SharedState s)
        {
            _launchHandler = handler;
            bbState = s;
        }

        public void Draw()
        {
            if (bbState == null || !bbState.PlannerVisible || FlightGlobals.ActiveVessel == null) return;
            
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Flight Planner");
        }

        private void DrawContents(int _)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || !bbState.PlannerVisible) { GUI.DragWindow(); return; }

            
            if (bbState.TargetVessel != null && !ReferenceEquals(vessel, bbState.TargetVessel) && vessel.id != bbState.TargetVessel.id)
            {
                GUILayout.Label($"Target: {bbState.TargetVessel.vesselName}");

                GUILayout.Label($"Apoapsis: {FormatKm(bbState.TargetVessel.orbit.ApA)}km", GUILayout.Width(175));
                GUILayout.Label($"Periapsis: {FormatKm(bbState.TargetVessel.orbit.PeA)}km", GUILayout.Width(175));
                GUILayout.Label($"Orbital Inc.: {Math.Round(bbState.TargetVessel.orbit.inclination, 4)}°", GUILayout.Width(175));

                if (bbState.LaunchPlan == null) GeneratePlan(vessel, bbState.TargetVessel);
                DisplayLaunchPlanCandidates();
            }
            else
            {
                GUILayout.Label("Select a target to generate a flight plan");
            }

            GUILayout.Space(10);

            // -- USER INPUTS / EDITS --
            // Live countdown to the selected candidate's absolute launch UT (the stored seconds-to-launch is a
            // stale snapshot from plan time).
            if (bbState.TargetVessel != null)
            {
                double selectedLaunchUt = bbState.SelectedLaunchCandidate?.LaunchUt ?? double.NaN;
                string _ltFullText = !double.IsNaN(selectedLaunchUt)
                    ? BlackbirdHelpers.FormatDuration(selectedLaunchUt - Planetarium.GetUniversalTime())
                    : "--";
                GUILayout.Label($"Launch in: {_ltFullText}");
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Apoapsis:", GUILayout.Width(70));
            InsertionAp = GUILayout.TextField(InsertionAp, GUILayout.Width(50));
            GUILayout.Label("km");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Periapsis:", GUILayout.Width(70));
            InsertionPe = GUILayout.TextField(InsertionPe, GUILayout.Width(50));
            GUILayout.Label("km");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Heading:", GUILayout.Width(70));
            InsertionHdg = GUILayout.TextField(InsertionHdg, GUILayout.Width(100));
            GUILayout.Label("°");
            GUILayout.EndHorizontal();

            GUILayout.Space(5);

            bool canCommit = _launchHandler != null && (_launchHandler.State == LaunchGuidanceState.Idle || _launchHandler.State == LaunchGuidanceState.PlanReady);

            GUI.enabled = canCommit;

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Accept Plan")) CommitPlanInputs(vessel, bbState.TargetVessel);
            GUI.enabled = _launchHandler != null;
            if (GUILayout.Button("Reset Plan")) _launchHandler.Reset();
            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }

        private void DisplayLaunchPlanCandidates()
        {
            GUILayout.Space(10);

            if (bbState.LaunchPlan == null || bbState.LaunchPlan.Candidates.Length == 0)
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
            GUILayout.Label("Req. Dv", GUILayout.Width(45));
            GUILayout.Label("Rmg. Dv", GUILayout.Width(45));
            GUILayout.Label("-", GUILayout.Width(45));
            GUILayout.EndHorizontal();

            int shown = 0;
            for (int i = 0; i < bbState.LaunchPlan.Candidates.Length; i++)
            {
                LaunchCandidate candidate = bbState.LaunchPlan.Candidates[i];
                if (!candidate.IsValid) continue;   // hide unusable (all-N/A) rows
                shown++;

                GUILayout.BeginHorizontal();

                // Live countdown (LaunchUt is absolute), so the row keeps ticking down rather than freezing.
                GUILayout.Label(
                    BlackbirdHelpers.FormatDuration(candidate.LaunchUt - Planetarium.GetUniversalTime()),
                    GUILayout.Width(80));
                GUILayout.Label($"{FormatKm(candidate.InsertionApoapsisAlt)}km", GUILayout.Width(45));
                GUILayout.Label($"{FormatKm(candidate.InsertionPeriapsisAlt)}km", GUILayout.Width(45));
                GUILayout.Label(FormatValue(candidate.LaunchHeadingDeg, "F1"), GUILayout.Width(50));
                GUILayout.Label(FormatValue(candidate.EstimatedOrbitsToRendezvous, "F1"), GUILayout.Width(40));
                GUILayout.Label(FormatValue(candidate.EstimatedDeltaVUsed, "F0"), GUILayout.Width(45));
                GUILayout.Label(FormatValue(candidate.EstimatedRemainingDeltaV, "F0"), GUILayout.Width(45));

                // start choose button ...
                bool isSelected = bbState.LaunchPlan.SelectedCandidateIndex == i;
                bool canChoose = candidate.IsValid && _launchHandler != null &&
                    (_launchHandler.State == LaunchGuidanceState.Idle ||
                     _launchHandler.State == LaunchGuidanceState.PlanReady);

                GUI.enabled = canChoose && !isSelected;
                if (GUILayout.Button(isSelected ? "Active" : "Choose", GUILayout.Width(60))) SelectCandidate(bbState.LaunchPlan, i);
                // ... end choose button
                GUI.enabled = true;

                GUILayout.EndHorizontal();
            }

            if (shown == 0) GUILayout.Label("No viable launch window in the next day.");
        }

        private void SelectCandidate(LaunchPlan launchPlan, int index)
        {
            if (index < 0 || index >= launchPlan.Candidates.Length || launchPlan?.Candidates == null) return;

            launchPlan.SelectedCandidateIndex = index;
            bbState.SelectedLaunchCandidate = launchPlan.Candidates[index];

            LaunchCandidate c = launchPlan.Candidates[index];

            // plan gives it to us in meters, but input wants kilometers (which will then convert it back to km)
            // redundant but
            InsertionAp = (c.InsertionApoapsisAlt / 1000).ToString();
            InsertionPe = (c.InsertionPeriapsisAlt / 1000).ToString();
            InsertionHdg = c.LaunchHeadingDeg.ToString("F1");
        }

        private void CommitPlanInputs(Vessel vessel, Vessel targetVessel)
        {
            if (_launchHandler == null || vessel == null) return;

            // will pass a launch plan if exists
            _launchHandler.Init(bbState);
            _launchHandler.SetTargetVessel(targetVessel);   // plan may already exist (LaunchPlanner.Create path), which skips ConstructLaunchPlan

            if (targetVessel == null || bbState.LaunchPlan == null)
            {
                InsertionTarget it = CreateInsertionTargetFromUi();
                double launchUt = targetVessel == null ? double.NaN : bbState.SelectedLaunchCandidate?.LaunchUt ?? double.NaN;
                _launchHandler.ConstructLaunchPlan(vessel, targetVessel, it.ApoapsisAlt, it.PeriapsisAlt, it.Heading, launchUt);
            }

            _launchHandler.AcceptPlan();
        }

        private void GeneratePlan(Vessel vessel, Vessel targetVessel)
        {
            LaunchLocation ll = LaunchLocation.FromVessel(vessel);
            InsertionTarget targetIt = new InsertionTarget
            {
                ApoapsisAlt = TrajectoryProvider.GetApoapsisAlt(targetVessel),
                PeriapsisAlt = TrajectoryProvider.GetPeriapsisAlt(targetVessel),
                Heading = 0
            };
            bbState.LaunchPlan = LaunchPlanner.Create(vessel, targetVessel, targetIt, ll);
        }

        private InsertionTarget CreateInsertionTargetFromUi()
        {
            return new InsertionTarget { ApoapsisAlt = _insertionAp, PeriapsisAlt = _insertionPe, Heading = _insertionHdg };
        }

        private static string FormatKm(double meters) =>
            double.IsNaN(meters) || double.IsInfinity(meters) ? "N/A" : (meters / 1000.0).ToString("F0");

        private static string FormatValue(double value, string format) =>
            double.IsNaN(value) || double.IsInfinity(value) ? "N/A" : value.ToString(format);
    }
}
