using System;
using System.Collections.Generic;
using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Models;
using UnityEngine;

namespace Blackbird.Logging
{
    public sealed class AscentRecorder
    {
        private const double SampleIntervalSeconds = 1.0;
        private const int MaxActualSamples = 4000;
        private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Trajectory);
        public bool LOG_ENABLED = true;

        public struct TrajectorySummary
        {
            public double LatchUt;
            public double TargetApKm;
            public double PeakProjKm;
            public double PeakActKm;
            public double LoftProjKm;
            public double LoftActKm;
            public double AltErrRmsKm;
            public double AltErrMaxKm;
            public double AltErrMaxAtT;
            public double TermProjAltKm;
            public double TermProjSpeed;
            public double TermActAltKm;
            public double TermActSpeed;
            public double MassUsedProjKg;
            public double MassUsedActKg;
        }

        private struct Sample
        {
            public double T; // seconds since latch
            public double AltMeters;
            public double DownrangeMeters;
            public double InertialSpeedMps;
            public double MassKg;
            public double ThrustOrThrottle; // throttle if projected, TWR if actual
        }

        private readonly List<Sample> _projected = new List<Sample>();
        private readonly List<Sample> _actual = new List<Sample>();

        private bool _latched;
        private double _latchUt = double.NaN;
        private Vector3d _refDir;
        private double _bodyRadius;
        private double _targetApAlt = double.NaN;
        private double _lastSampleUt = double.NegativeInfinity;

        public bool HasProjection => _latched;
        public bool HasData => _latched && _actual.Count > 0;

        public void Reset()
        {
            _projected.Clear();
            _actual.Clear();
            _latched = false;
            _latchUt = double.NaN;
            _lastSampleUt = double.NegativeInfinity;
            _targetApAlt = double.NaN;
        }
        public void LatchProjected(AscentPath path, double bodyRadius, double targetApAlt)
        {
            if (_latched || path == null || !path.IsValid) return;

            _bodyRadius = bodyRadius;
            _targetApAlt = targetApAlt;
            _latchUt = path.CreatedUniversalTime;
            _refDir = path.Points[0].RelativePosition.normalized;

            _projected.Clear();
            foreach (AscentPathPoint p in path.Points)   // was PsgSolutionPoint
            {
                _projected.Add(new Sample { /* identical body */ });
            }
            _latched = true;
        }

        // append flown state
        public void SampleActual(Vessel vessel, double ut)
        {
            if (!_latched || vessel == null 
                || ut - _lastSampleUt < SampleIntervalSeconds 
                || _actual.Count >= MaxActualSamples) return;
            _lastSampleUt = ut;

            VesselState vs = VesselState.FromVessel(vessel);
            if (vs == null || vs.Body == null) return;

            //Vector3d rel = vs.Position - vs.Body.position;
            // dufixme: can i use this instead?
            _actual.Add(new Sample
            {
                T = ut - _latchUt,
                AltMeters = vs.TrajectoryState.RelativeVelocity.magnitude,
                DownrangeMeters = Downrange(vs.TrajectoryState.RelativeVelocity),
                InertialSpeedMps = vs.OrbitalVelocity.magnitude,
                MassKg = vs.TotalMass * 1000.0, // vs mass in tons, PSG in kg
                ThrustOrThrottle = vs.ThrustToWeight
            });
        }

        private double Downrange(Vector3d rel)
        {
            double d = Vector3d.Dot(_refDir, rel.normalized);
            d = MathHelpers.Clamp(d, -1.0, 1.0);
            // d = d > 1.0 ? 1.0 : (d < -1.0 ? -1.0 : d);
            return Math.Acos(d) * _bodyRadius;
        }

        // write projected + actual paths with a diff summary to csv in KSP root
        public void WriteReport()
        {
            if (!_latched || !LOG_ENABLED) return;

            _log.Write("ascent-summary", BuildSummary());

            for (int i = 0; i < _projected.Count; i++) _log.Write("proj", i, _projected[i]);
            for (int i = 0; i < _actual.Count; i++) _log.Write("act", i, _actual[i]);
        }

        private TrajectorySummary BuildSummary()
        {
            double projPeak = PeakAlt(_projected);
            double actPeak = PeakAlt(_actual);

            double altRms, altMax, altMaxT;
            AltitudeError(out altRms, out altMax, out altMaxT);

            Sample projEnd = _projected.Count > 0 ? _projected[_projected.Count - 1] : default(Sample);
            Sample actEnd = _actual.Count > 0 ? _actual[_actual.Count - 1] : default(Sample);

            return new TrajectorySummary
            {
                LatchUt = _latchUt,
                TargetApKm = _targetApAlt / 1000.0,
                PeakProjKm = projPeak / 1000.0,
                PeakActKm = actPeak / 1000.0,
                LoftProjKm = (projPeak - _targetApAlt) / 1000.0,
                LoftActKm = (actPeak - _targetApAlt) / 1000.0,
                AltErrRmsKm = altRms / 1000.0,
                AltErrMaxKm = altMax / 1000.0,
                AltErrMaxAtT = altMaxT,
                TermProjAltKm = projEnd.AltMeters / 1000.0,
                TermProjSpeed = projEnd.InertialSpeedMps,
                TermActAltKm = actEnd.AltMeters / 1000.0,
                TermActSpeed = actEnd.InertialSpeedMps,
                MassUsedProjKg = _projected.Count > 0 ? _projected[0].MassKg - projEnd.MassKg : 0.0,
                MassUsedActKg = _actual.Count > 0 ? _actual[0].MassKg - actEnd.MassKg : 0.0
            };
        }
        private static double PeakAlt(List<Sample> s)
        {
            double m = double.NegativeInfinity;
            for (int i = 0; i < s.Count; i++) if (s[i].AltMeters > m) m = s[i].AltMeters;
            return m;
        }

        // Altitude error of actual vs projected, keyed on time-since-latch (frame-robust axis).
        private void AltitudeError(out double rms, out double max, out double maxT)
        {
            rms = 0.0; max = 0.0; maxT = 0.0;
            if (_projected.Count < 2 || _actual.Count == 0) return;

            double projEndT = _projected[_projected.Count - 1].T;
            double sumSq = 0.0; int cnt = 0;
            foreach (Sample a in _actual)
            {
                if (a.T < 0.0 || a.T > projEndT) continue;
                double e = Math.Abs(a.AltMeters - InterpProjAlt(a.T));
                sumSq += e * e; cnt++;
                if (e > max) { max = e; maxT = a.T; }
            }
            if (cnt > 0) rms = Math.Sqrt(sumSq / cnt);
        }

        private double InterpProjAlt(double t)
        {
            for (int i = 0; i < _projected.Count - 1; i++)
            {
                Sample a = _projected[i], b = _projected[i + 1];
                if (t > b.T) continue;
                double span = b.T - a.T;
                double f = span > 1e-9 ? (t - a.T) / span : 0.0;
                return a.AltMeters + (b.AltMeters - a.AltMeters) * f;
            }
            return _projected[_projected.Count - 1].AltMeters;
        }
    }
}
