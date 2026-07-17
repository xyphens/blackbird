using Blackbird.Guidance;
using Blackbird.Helpers;
using Blackbird.Mathematics;
using Blackbird.Models;
using Blackbird.Planning;
using System;
using UnityEngine;

namespace Blackbird.Modules
{
    public sealed class GuidanceComputer
    {
        private SharedState bbState;
        private const string WindowKey = "Blackbird.GuidanceComputer";
        private static readonly int WindowId = WindowKey.GetHashCode();
        private Rect _windowRect = WindowPositions.Restore(WindowKey, new Rect(560, 620, 380, 300));

        private double _pitchInputDegrees = 90.0;
        public string PitchInputText
        {
            get => _pitchInputDegrees.ToString("F0");
            set { if (double.TryParse(value, out double v)) _pitchInputDegrees = MathHelpers.Clamp(v, -90.0, 90.0); }
        }

        private double _headingInputDegrees = 90.0;
        public string HeadingInputText
        {
            get => _headingInputDegrees.ToString("F0");
            set { if (double.TryParse(value, out double v)) _headingInputDegrees = MathHelpers.Clamp(v, -180.0, 180.0); }
        }

        private double _rollInputDegrees = 0.0;
        public string RollInputText
        {
            get => _rollInputDegrees.ToString("F0");
            set { if (double.TryParse(value, out double v)) _rollInputDegrees = MathHelpers.Clamp(v, -180.0, 180.0); }
        }

        private double _throttleInputPct = 0;
        public string ThrottleInputText
        {
            get => _throttleInputPct.ToString();
            set { if (double.TryParse(value, out double v)) _throttleInputPct = MathHelpers.Clamp(v, 0.0, 100.0); }
        }

        private readonly TrajectoryPlot _trajectoryPlot = new Planning.TrajectoryPlot();
        private const double MinSecondsToUseWarp = 10.0;
        private readonly string[] _guidanceModeLabels = { "None", "Manual", "Autopilot" };

        private const float BtnW = 60f;
        private const float BtnH = 34f;

        bool _wasVisible = false; // used to reset the details toggle when the window is reopened

        // Which view DrawContents last laid out; a change means the window must re-fit its height.
        private bool _fitted = false;
        private bool _lastHadPlan;
        private LaunchGuidanceState _lastState;
        private GuidanceMode _lastMode;
        private bool _lastDetails;

        private LaunchHandler _launchHandler;
        public void Init(LaunchHandler handler, SharedState s)
        {
             _launchHandler = handler;
            bbState = s;
        }

        public void Draw()
        {
            if (bbState == null || !bbState.GuidanceVisible)
            {
                _wasVisible = false;
                return;
            }

            if (!_wasVisible)
            {
                _wasVisible = true;
                if (_launchHandler != null) _launchHandler.TrackTrajectory = false;
            }

            RefitIfViewChanged();
            _windowRect = GUILayout.Window(WindowId, _windowRect, DrawContents, "Guidance Computer");
            WindowPositions.Record(WindowKey, _windowRect);
        }

        // GUILayout.Window treats the passed height as a minimum: it grows to fit content but never shrinks.
        // Zeroing the height re-fits the window to whichever view is about to draw, so leaving a tall view (live
        // ascent) for a short one (the completed-flight result) doesn't strand the old height.
        private void RefitIfViewChanged()
        {
            bool hadPlan = bbState.LaunchPlan != null;
            LaunchGuidanceState state = _launchHandler != null ? _launchHandler.State : LaunchGuidanceState.Idle;
            GuidanceMode mode = _launchHandler != null ? _launchHandler.GuidanceMode : GuidanceMode.None;
            bool details = _launchHandler != null && _launchHandler.TrackTrajectory;

            if (_fitted && hadPlan == _lastHadPlan && state == _lastState && mode == _lastMode && details == _lastDetails) return;

            _lastHadPlan = hadPlan;
            _lastState = state;
            _lastMode = mode;
            _lastDetails = details;
            _fitted = true;
            _windowRect.height = 0f;
        }

        private void DrawContents(int _)
        {
            if (GUI.Button(new Rect(_windowRect.width - 22, 2, 18, 18), " ")) bbState.GuidanceVisible = false;
            // null guard before any State access
            if (bbState.LaunchPlan == null)
            {
                if (_launchHandler.State == LaunchGuidanceState.Complete)
                {
                    // finished flight: show the insertion result
                    DrawGuidanceResult(_launchHandler.GuidanceInfo);
                } else
                {
                    GUILayout.Label("Guidance unavailable - select a flight plan");
                }
                
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
                double countdown = double.NaN;
                if (_launchHandler.TargetVessel != null)
                {
                    bbState.LaunchPlan.LaunchWindow = LaunchWindowInfo.Create(
                                FlightGlobals.ActiveVessel,
                                OrbitInfo.Create(_launchHandler.TargetVessel.orbit),
                                LaunchLocation.FromVessel(FlightGlobals.ActiveVessel));

                    LaunchWindowInfo lw = bbState.LaunchPlan.LaunchWindow;
                    if (lw != null)
                    {
                        GUILayout.Label($"Asc Node Lon: {lw.AscendingNodeLongitudeDeg:F2}°");
                        GUILayout.Label($"Desc Node Lon: {lw.DescendingNodeLongitudeDeg:F2}°");
                        GUILayout.Label($"Time to Asc: {lw.TimeToAscendingNodeSeconds:F0}s");
                        GUILayout.Label($"Time to Desc: {lw.TimeToDescendingNodeSeconds:F0}s");
                        GUILayout.Label($"Selected Offset: {lw.PlaneOffsetDeg:F2}°");
                    }

                    countdown = GetDisplayedLaunchCountdownSeconds(bbState.LaunchPlan);
                    GUILayout.Label(double.IsNaN(countdown) ? "T- -- seconds" : $"T- {countdown:F0} seconds");

                    // Armed: warp to the window, then pick a flight mode (which begins the ascent).
                    GUI.enabled = countdown >= MinSecondsToUseWarp;
                    if (GUILayout.Button("Warp To Launch")) _launchHandler.WarpToLaunch();
                }

                GUI.enabled = !_launchHandler.OpenLoopBuilding;

                DrawSelectGuidanceMethod();
                // Begin the ascent once a flight mode is chosen AND we're at the launch window or there's no target (countdown)
                if (_launchHandler.OpenLoopBuilding)
                {
                    GUILayout.Label("Ascent plan building — launch enabled when it resolves");
                } else
                {
                    if (_launchHandler.GuidanceMode != GuidanceMode.None 
                        && (double.IsNaN(countdown) || !(countdown > MinSecondsToUseWarp)))
                    {
                        _launchHandler.StartAscentGuidance();
                    }

                    if (_launchHandler.OpenLoopPlan != null)
                    {
                        GUILayout.Label($"Ascent plan: {_launchHandler.OpenLoopStatus}");
                    }
                }

                GUI.enabled = true;

                GUILayout.Space(4);

                GUILayout.Label("Advanced Flight Tuning");

                GUILayout.BeginHorizontal();
                GUILayout.Label("PSG Transition margin:");
                _launchHandler.PsgTransitionMargin = GUILayout.TextField(_launchHandler.PsgTransitionMargin, GUILayout.Width(50));
                GUILayout.Label("°", GUILayout.Width(50));
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                GUILayout.BeginHorizontal();
                GUILayout.Label("PSG Handover kPa");
                _launchHandler.HandoverKpa = GUILayout.TextField(_launchHandler.HandoverKpa, GUILayout.Width(50));
                GUILayout.Label("kPa", GUILayout.Width(50));
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                GUILayout.BeginHorizontal();
                GUILayout.Label("Hold Pitch Until Velocity");
                _launchHandler.MinVSpeedToPitch = GUILayout.TextField(_launchHandler.MinVSpeedToPitch, GUILayout.Width(50));
                GUILayout.Label("m/s", GUILayout.Width(50));
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                GUILayout.BeginHorizontal();
                GUILayout.Label("Hold Pitch Until Altitude");
                _launchHandler.MinAltitudeForPitch = GUILayout.TextField(_launchHandler.MinAltitudeForPitch, GUILayout.Width(50));
                GUILayout.Label("m", GUILayout.Width(50));
                GUILayout.EndHorizontal();
                //bool armed = _launchHandler.State == LaunchGuidanceState.AwaitingLaunch
                //          || _launchHandler.State == LaunchGuidanceState.WarpingToLaunch;

                //if (!armed)
                //{
                //    // arm the launch
                //    GUI.enabled = _launchHandler.State == LaunchGuidanceState.PlanAccepted
                //                  && bbState.CanClaimControl(BlackbirdModule.LaunchGuidance);
                //    if (GUILayout.Button("Start Guidance")) _launchHandler.StartGuidance();
                //}

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
            GUILayout.Label("Predicted", GUILayout.Width(110));
            GUILayout.Label("Dev.", GUILayout.Width(60));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Apoapsis", GUILayout.Width(80));
            GUILayout.Label(FormatKm(guidanceInfo.TargetApoapsisAlt, "F0"), GUILayout.Width(60));
            if (FlightGlobals.ActiveVessel.situation == Vessel.Situations.PRELAUNCH)
            {
                GUILayout.Label("-", GUILayout.Width(110));
            } else
            {
                GUILayout.Label(FormatKm(guidanceInfo.PredictedApoapsisAlt, "F0"), GUILayout.Width(110));
            }
            GUILayout.Label(FormatKm(guidanceInfo.ApoapsisErrorMeters, "F1"), GUILayout.Width(60));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Periapsis", GUILayout.Width(80));
            GUILayout.Label(FormatKm(guidanceInfo.TargetPeriapsisAlt, "F0"), GUILayout.Width(60));
            if (FlightGlobals.ActiveVessel.situation == Vessel.Situations.PRELAUNCH)
            {
                GUILayout.Label("-", GUILayout.Width(110));
            }
            else
            {
                GUILayout.Label(FormatKm(guidanceInfo.PredictedPeriapsisAlt, "F0"), GUILayout.Width(110));
            }
            
            GUILayout.Label(FormatKm(guidanceInfo.PeriapsisErrorMeters, "F1"), GUILayout.Width(60));
            GUILayout.EndHorizontal();
            if (_launchHandler.GuidanceMode == GuidanceMode.Manual)
            {
                // PITCH
                GUILayout.Label($"Pitch: {guidanceInfo.CommandPitchDeg:F2}°");
                GUILayout.BeginHorizontal();
                PitchInputText = GUILayout.TextField(PitchInputText, GUILayout.Width(50), GUILayout.Height(BtnH));
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) _launchHandler.SetPitchCommand(_pitchInputDegrees);
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) _launchHandler.DecreaseManualPitchCommand();
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) _launchHandler.IncreaseManualPitchCommand();
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) _launchHandler.ResetPitchCommand();
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // HEADING
                GUILayout.Label($"Heading: {guidanceInfo.CommandHeadingDeg:F2}°");
                GUILayout.BeginHorizontal();
                HeadingInputText = GUILayout.TextField(HeadingInputText, GUILayout.Width(50), GUILayout.Height(BtnH));
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetHeadingCommand(_headingInputDegrees); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualHeadingCommand();  };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualHeadingCommand(); };
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.ResetHeadingCommand(); };
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // ROLL
                GUILayout.Label($"Roll: {guidanceInfo.CommandRoll:F2}°");
                GUILayout.BeginHorizontal();
                RollInputText = GUILayout.TextField(RollInputText, GUILayout.Width(50), GUILayout.Height(BtnH));
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetRollCommand(_rollInputDegrees); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualRollCommand(); };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualRollCommand(); };
                if (GUILayout.Button("Reset", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.ResetRollCommand(); };
                GUILayout.EndHorizontal();

                GUILayout.Space(6);

                // THROTTLE
                GUILayout.Label($"Throttle: {BlackbirdHelpers.FormatThrottle(guidanceInfo.CommandThrottle)}");
                GUILayout.BeginHorizontal();
                ThrottleInputText = GUILayout.TextField(ThrottleInputText, GUILayout.Width(50), GUILayout.Height(BtnH));
                if (GUILayout.Button("Apply", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.SetThrottleCommand(_throttleInputPct); };
                if (GUILayout.Button("−", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.DecreaseManualThrottleCommand(); };
                if (GUILayout.Button("+", GUILayout.Width(BtnW), GUILayout.Height(BtnH))) { _launchHandler.IncreaseManualThrottleCommand(); };
                if (GUILayout.Button("Cutoff", GUILayout.Width(55), GUILayout.Height(BtnH))) { _launchHandler.CutoffThrottleCommand(); };
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

            DrawTrajectory();

            _launchHandler.TrackTrajectory = GUILayout.Toggle(_launchHandler.TrackTrajectory, "Show Advanced Details");
            if (_launchHandler.TrackTrajectory) DrawAscentDetails();

            GUI.DragWindow();
        }

        private void DrawAscentDetails()
        {
            GUILayout.Space(6);
            var plan = _launchHandler.OpenLoopPlan;
            if (plan == null || !plan.IsValid) return;

            GUILayout.Label($"Pitch rate: {plan.PitchRateDegPerSecond:F2}°/s   Handoff: {plan.HandoffAltitudeMeters / 1000.0:F1} km");
            GUILayout.Label($"Predicted: {plan.PredictedInjectedMassKg / 1000.0:F1} t to orbit, T+{plan.PredictedTimeToOrbitSeconds:F0}s");
            GUILayout.Space(4);
            GUILayout.Label("chi table (m/s \u2192 pitch\u00b0)");
            double[] s = plan.TableSpeedMps, p = plan.TablePitchDeg;
            int step = Math.Max(1, s.Length / 10);
            for (int i = 0; i < s.Length; i += step)
                GUILayout.Label($"  {s[i],6:F0}  \u2192  {p[i],5:F1}");
            GUILayout.Label($"  {s[s.Length - 1],6:F0}  \u2192  {p[p.Length - 1],5:F1}  (handoff)");
        }

        private void DrawTrajectory()
        {
            GUILayout.Space(10);
            var rec = _launchHandler.AscentReport;
            rec.GetHistory(out double[] hAlt, out double[] hDown);
            rec.GetBallisticProjection(FlightGlobals.ActiveVessel, out double[] pAlt, out double[] pDown);

            double tgtAp = _launchHandler.GuidanceInfo != null ? _launchHandler.GuidanceInfo.TargetApoapsisAlt : rec.TargetApAlt;

            double[] planAlt = null;
            double[] planDown = null;
            // chart the open loop trajectory
            var olPlan = _launchHandler.OpenLoopPlan;
            if (olPlan != null && olPlan.IsValid && olPlan.Trace != null && olPlan.Trace.Length > 1)
            {
                int n = olPlan.Trace.Length;
                planAlt = new double[n];
                planDown = new double[n];
                for (int i = 0; i < n; i++)
                {
                    planAlt[i] = olPlan.Trace[i].AltMeters;
                    planDown[i] = olPlan.Trace[i].DownrangeMeters;
                }
            }

            _trajectoryPlot.Draw(hAlt, hDown, pAlt, pDown, planAlt, planDown, tgtAp, 360, 170);

            if (!double.IsNaN(_trajectoryPlot.MaxLoftAboveTargetMeters) && !double.IsInfinity(_trajectoryPlot.MaxLoftAboveTargetMeters))
            {
                GUILayout.Label($"Loft above target Ap: {_trajectoryPlot.MaxLoftAboveTargetMeters / 1000.0:F1} km");
            }
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

            GUILayout.Space(6);
            DrawTrajectory();   // flown history vs plan trace + the loft readout = the retrospective
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
