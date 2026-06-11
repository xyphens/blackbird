using UnityEngine;
using System;
using Blackbird;
using Blackbird.Helpers;
using Blackbird.Models;
using Blackbird.Planning;

namespace Blackbird
{

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class RendezvousAssistant : MonoBehaviour
    {
        private Rect _windowRect = new Rect(200, 200, 350, 800);
        private string _insertionApText = "";
        private string _insertionPeText = "";
        private bool _useTargetOrbitInsertion = true;

        public void Start()
        {
            Debug.Log("[RendezvousAssistant] Loaded");
        }

        public void Update()
        {
            if (FlightGlobals.ActiveVessel != null)
            {
                Vessel vessel = FlightGlobals.ActiveVessel;
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
            // current vessel
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel != null)
            {
                GUILayout.Label($"Active: {vessel.vesselName}");
                GUILayout.Label($"Altitude: {vessel.altitude:N0} m");
                GUILayout.Label($"Body Radius: {vessel.mainBody.Radius:N0} m");
            } else
            {
                return;
            }

            // target vessel
            ITargetable target = FlightGlobals.fetch.VesselTarget;

            if (target is Vessel targetVessel)
            {
                // get and show our launch plan

                InsertionTarget it = DrawInsertionTargetInputs(targetVessel);

                LaunchLocation ll = LaunchLocation.FromVessel(vessel);

                LaunchPlan lp = LaunchPlanner.Create(vessel, targetVessel, it, ll);

                GUILayout.Label($"Running for Scale: {lp.ScaleLabel}");

                GUILayout.Space(10);
                GUILayout.Label("-- Active Orbit --");
                GUILayout.Label($"Inclination: {lp.ActiveOrbit.InclinationDeg:F2}°");
                GUILayout.Label($"LAN: {lp.ActiveOrbit.LanDeg:F2}°");
                GUILayout.Label($"Apoapsis: {lp.ActiveOrbit.ApoapsisAlt / 1000:F0} km");
                GUILayout.Label($"Periapsis: {lp.ActiveOrbit.PeriapsisAlt / 1000:F0} km");
                GUILayout.Label($"Period: {lp.ActiveOrbit.PeriodSeconds:F1}s");

                GUILayout.Space(10);
                GUILayout.Label("-- Target Info --");
                GUILayout.Label($"Name: {targetVessel.vesselName}");

                // distance
                GUILayout.Label($"Distance: {lp.DistanceMeters / 1000:F1} km");
                // inclination
                GUILayout.Label($"Inclination: {lp.TargetOrbit.InclinationDeg:F2}°");
                // LAN (longitude of ascending node
                GUILayout.Label($"LAN: {lp.TargetOrbit.LanDeg:F2}°");
                // apoapsis
                GUILayout.Label($"Apoapsis: {lp.TargetOrbit.ApoapsisAlt / 1000:F0} km");
                // periapsis
                GUILayout.Label($"Periapsis: {lp.TargetOrbit.PeriapsisAlt / 1000:F0} km");
                // phase angle
                GUILayout.Label($"Phase Angle: {lp.PhaseAngleDeg:F1}°");

                GUILayout.Label($"Period Diff: {lp.PhasingOrbit.PeriodDifferenceSeconds:F1}s");

                GUILayout.Label(
                    lp.PhasingOrbit.IsFasterThanTarget
                        ? "Phasing: Insertion orbit catches target"
                        : "Phasing: Target pulls away");

                // ORBITS
                GUILayout.Space(10);
                GUILayout.Label("-- Orbit Comparison --");
                GUILayout.Label($"Inc Delta: {lp.RelativeInclinationDeg:F2}°");
                GUILayout.Label($"LAN Delta: {lp.RelativeLanDeg:F2}°");
                GUILayout.Label($"Period Delta: {lp.RelativePeriodSeconds:F1}s");

                // PHASING
                GUILayout.Space(10);
                GUILayout.Label("-- Phasing Period -- ");
                GUILayout.Label($"Period Diff: {lp.PhasingOrbit.PeriodDifferenceSeconds:F1}s");
                GUILayout.Label($"Period Diff: {lp.PhasingOrbit.PeriodDifferenceMinutes:F2} min");
                GUILayout.Label($"Period Diff: {lp.PhasingOrbit.PeriodDifferencePercent:F3}%");
                GUILayout.Label($"Phase Gain: {lp.PhasingOrbit.RelativePhaseGainDegPerOrbit:F2}°/orbit");
                if (lp.PhasingOrbit.HasRendezvousEstimate)
                {
                    GUILayout.Label($"Rendezvous Orbits: {lp.PhasingOrbit.EstimatedOrbitsToRendezvous:F1}");
                    GUILayout.Label(
                        $"Rendezvous Time: {BlackbirdHelpers.FormatDuration(lp.PhasingOrbit.EstimatedTimeToRendezvousSeconds)}");
                }
                else
                {
                    GUILayout.Label("Rendezvous Estimate: unavailable");
                }

                GUILayout.Label(
                    lp.PhasingOrbit.IsFasterThanTarget
                        ? "Phasing: insertion orbit is faster than target"
                        : "Phasing: insertion orbit is slower than target");

                // RECOMMENDATION
                ShowPhasingRecommendation(lp.PhasingRecommendation, lp);

                GUILayout.Space(10);
                GUILayout.Label("-- Launch Recommendation -- ");
                GUILayout.Label(
                    double.IsNaN(lp.LaunchAzimuthDeg)
                        ? "Azimuth: unavailable"
                        : $"Azimuth: {lp.LaunchAzimuthDeg:F1}°");
                GUILayout.Label($"Apoapsis: {lp.RecommendedApAlt / 1000:F0} km");
                GUILayout.Label($"Periapsis: {lp.RecommendedPeAlt / 1000:F0} km");
                
                GUILayout.Space(10);
                GUILayout.Label("-- Launch Window --");
                GUILayout.Label($"Current Node: {lp.LaunchWindow.NodeName}");
                GUILayout.Label($"Asc Node Lon: {lp.LaunchWindow.AscendingNodeLongitudeDeg:F2}°");
                GUILayout.Label($"Desc Node Lon: {lp.LaunchWindow.DescendingNodeLongitudeDeg:F2}°");
                GUILayout.Label($"Time to Asc: {lp.LaunchWindow.TimeToAscendingNodeSeconds:F0}s");
                GUILayout.Label($"Time to Desc: {lp.LaunchWindow.TimeToDescendingNodeSeconds:F0}s");
                GUILayout.Label($"Selected Offset: {lp.LaunchWindow.PlaneOffsetDeg:F2}°");
                string fmtLaunchIn = BlackbirdHelpers.FormatDuration(lp.LaunchWindow.TimeToPlaneCrossingSeconds);
                GUILayout.Label($"Launch In: {fmtLaunchIn}");
            } else
            {
                GUILayout.Space(10);
                GUILayout.Label("No Target");
            }

            GUI.DragWindow();
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

            // apoapsis input
            GUILayout.BeginHorizontal();
            GUILayout.Label("Ap:", GUILayout.Width(40));
            _insertionApText = GUILayout.TextField(_insertionApText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            // periapsis input
            GUILayout.BeginHorizontal();
            GUILayout.Label("Pe:", GUILayout.Width(40));
            _insertionPeText = GUILayout.TextField(_insertionPeText, GUILayout.Width(100));
            GUILayout.Label("m");
            GUILayout.EndHorizontal();

            return CreateInsertionTargetFromUi(targetVessel);
        }

        private InsertionTarget CreateInsertionTargetFromUi(Vessel targetVessel) {
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

            // circular
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

        private void ShowPhasingRecommendation(PhasingRecommendation pr, LaunchPlan lp)
        {
            GUILayout.Label("-- Phasing Recommendation --");

            if (pr == null)
            {
                GUILayout.Label("Unavailable");
                return;
            }

            if (!pr.HasRecommendation)
            {
                GUILayout.Label("Unavailable");
                GUILayout.Label(pr.ReasonUnavailable);
                return;
            }



            GUILayout.Label("Mode: " + pr.Mode);
            GUILayout.Label("Apoapsis: " + (pr.ApoapsisAlt / 1000.0).ToString("N0") + " km");
            GUILayout.Label("Periapsis: " + (pr.PeriapsisAlt / 1000.0).ToString("N0") + " km");
            GUILayout.Label("Period Diff: " + pr.PeriodDifferenceSeconds.ToString("N1") + "s");
            GUILayout.Label("Phase Gain: " + pr.PhaseGainDegPerOrbit.ToString("N2") + "°/orbit");
            GUILayout.Label("Rendezvous Orbits: " + pr.EstimatedOrbitsToRendezvous.ToString("N1"));
            GUILayout.Label("Rendezvous Time: " + BlackbirdHelpers.FormatDuration(pr.EstimatedTimeToRendezvousSeconds));
            GUILayout.Label(
                "Offset: " +
                ((pr.ApoapsisAlt - lp.TargetOrbit.ApoapsisAlt) / 1000.0)
                    .ToString("N0") +
                " km");
        }
    }
}