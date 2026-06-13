using Blackbird.Enums;
using Blackbird.Guidance;

namespace Blackbird.Models
{
    public sealed class LaunchPlan
    {
        private double _launchAzimuthDeg;
        private double _recommendedApAlt;
        private double _recommendedPeAlt;
        private PhasingRecommendation _phasingRecommendation;

        public OrbitInfo ActiveOrbit { get; set; }
        public OrbitInfo TargetOrbit { get; set; }
        // expose launch window stats (valid window, time away, etc)
        public LaunchWindowInfo LaunchWindow { get; set; }

        // compare orbits
        public double RelativeInclinationDeg { get; set; }
        public double RelativeLanDeg { get; set; }
        public double RelativePeriodSeconds { get; set; }

        public double PhaseAngleDeg { get; set; }
        public double DistanceMeters { get; set; }

        public double LaunchAzimuthDeg
        {
            get { return SelectedCandidate != null ? SelectedCandidate.LaunchHeadingDeg : _launchAzimuthDeg; }
            set { _launchAzimuthDeg = value; }
        }

        public double RecommendedApAlt
        {
            get { return SelectedCandidate != null ? SelectedCandidate.InsertionApoapsisAlt : _recommendedApAlt; }
            set { _recommendedApAlt = value; }
        }

        public double RecommendedPeAlt
        {
            get { return SelectedCandidate != null ? SelectedCandidate.InsertionPeriapsisAlt : _recommendedPeAlt; }
            set { _recommendedPeAlt = value; }
        }

        public PlanetScale.PlanetScaleEnum ScaleLabel { get; set; }
        public InsertionTarget InsertionTarget { get; set; }
        public PhasingOrbit PhasingOrbit { get; set; }

        public PhasingRecommendation PhasingRecommendation
        {
            get { return SelectedCandidate != null ? SelectedCandidate.PhasingRecommendation : _phasingRecommendation; }
            set { _phasingRecommendation = value; }
        }

        public LaunchCandidate[] Candidates { get; set; }
        public int SelectedCandidateIndex { get; set; }

        public LaunchCandidate SelectedCandidate
        {
            get
            {
                if (Candidates == null || Candidates.Length == 0) return null;
                if (SelectedCandidateIndex < 0 || SelectedCandidateIndex >= Candidates.Length) return null;

                return Candidates[SelectedCandidateIndex];
            }
        }

        public AscentProfile AscentProfile
        {
            get { return SelectedCandidate != null ? SelectedCandidate.AscentProfile : null; }
        }
    }
}
