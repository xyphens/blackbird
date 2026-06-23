using UnityEngine;

namespace Blackbird.RCS
{
    public class TuningParams
    {
        public double WasteThreshold = 0.0;
        public double WasteFactor = 0.0;
        public double TorqueFactor = 0.0;
        public double TranslationFactor = 0.0;
    }

    public class SolverCache
    {
        private static int _precision = 2;
        private readonly int _hash; // hash value of solved result
        private static void SetPrecision(int precision) => _precision = precision;
        private float ToBucket(double d, int precision) => (float) Mathf.RoundToInt((float) d * precision) / precision;
        public SolverCache(ref Vector3 d, Vector3 rot)
        {
            if (d == Vector3.zero)
            {
                _hash = 0;
                return;
            }

            d.Normalize();
            float maxAbsVector = Mathf.Max(Mathf.Abs(d.x), Mathf.Max(Mathf.Abs(d.y), Mathf.Abs(d.z)));

            d.x = ToBucket(d.x / maxAbsVector, _precision);
            d.y = ToBucket(d.y / maxAbsVector, _precision);
            d.z = ToBucket(d.z / maxAbsVector, _precision);
            int x = (int)(d.x * 127);
            int y = (int)(d.y * 127);
            int z = (int)(d.z * 127);

            _hash = ((x & 0xFF) << 16) + ((y & 0xFF) << 8) + (z & 0xFF);
        }

        // compare hash values of a prior solution
        public override bool Equals(object other)
        {
            SolverCache o = other as SolverCache;
            return o != null && _hash == o._hash;
        }

        public override int GetHashCode() => _hash;
        public override string ToString() => _hash.ToString("x6");
    }

    public class Solver
    {

    }

    // TODO: implement this if craft has issues keeping orientation
    public class SolverThread
    {
        //private readonly MathHelpers.MovingAverage _computeErr = new MathHelpers.MovingAverage();
        //// current tasks
        //private readonly Queue _tasksQueue = Queue.Synchronized(new Queue());
        //// result tasks
        //private readonly Queue _resultsQueue = Queue.Synchronized(new Queue());
        //private readonly Dictionary<SolverCache, double[]> _results = new Dictionary<SolverCache, double[]>();
        //private readonly HashSet<SolverCache> _pending = new HashSet<SolverCache>();
        //private List <Solver.>
        //private readonly AutoResetEvent _event = new AutoResetEvent(false);
        //private bool _stop;
        //private Thread _thread;
        //private bool _inProgress;
        //private int _prevPartCount;
        //private readonly List<ModuleRCS> _lastDisabled = new List<ModuleRCS>();
        //private Vector3 _lastCom = Vector3.zero;

        //public double CalcTime { get; private set; }
        //public double ComputeError => _computeErr;
        //public double ComputeErrorThreshold { get; private set; }
        //public double MaxComputeError { get; private set; }
        //public int TaskCount
    }
    public class Balancer
    {
        public double overdrive = 1.0;
        // fractions overdrive to prevent unnecessary thrusters firing
        // fine-tuning variables
        public readonly double overdriveScale = 0.9;
        public readonly double torqueFactorScale = 1.0;
        public readonly double translationFactorScale = 0.005;
        public readonly double wasteFactorScale = 1.0;

        public void UpdateTuning()
        {
            double wasteThreshold = overdrive * overdriveScale;
            TuningParams tuningParams = new TuningParams
            {
                WasteThreshold = wasteThreshold,
                TorqueFactor = torqueFactorScale,
                TranslationFactor = translationFactorScale,
                WasteFactor = wasteFactorScale
            };
        }
    }
}
