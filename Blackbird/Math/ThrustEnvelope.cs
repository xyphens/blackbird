using System;
using Blackbird.Helpers;
namespace Blackbird.Mathematics
{
    public class ThrustEnvelope
    {
        public Vector3d Positive = Vector3d.zero, Negative = Vector3d.zero;
        public enum Orientation { UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3, FORWARD = 4, BACK = 5 };
        public static readonly Vector3d[] Orientations = { Vector3d.up, Vector3d.down, Vector3d.left, Vector3d.right, Vector3d.forward, Vector3d.back };
        public static readonly Orientation[] OrientationValues = (Orientation[])Enum.GetValues (typeof(Orientation));

        public double Up { get => Positive.y; set => Positive.y = value; }
        public double Down { get => Negative.y; set => Negative.y = value; }
        public double Left { get => Negative.x; set => Negative.x = value; }
        public double Right { get => Positive.x; set => Positive.x = value; }
        public double Forward { get => Positive.z; set => Positive.z = value; }
        public double Back { get => Negative.z; set => Negative.z = value; }

        public double this[Orientation index]
        {
            get
            {
                switch (index)
                {
                    case Orientation.UP: return Up;
                    case Orientation.DOWN: return Down;
                    case Orientation.LEFT: return Left;
                    case Orientation.RIGHT: return Right;
                    case Orientation.FORWARD: return Forward;
                    case Orientation.BACK: return Back;
                    default: return 0;
                };
            }
            set
            {
                switch (index)
                {
                    case Orientation.UP:
                        Up = value;
                        break;
                    case Orientation.DOWN:
                        Down = value;
                        break;
                    case Orientation.LEFT:
                        Left = value;
                        break;
                    case Orientation.RIGHT:
                        Right = value;
                        break;
                    case Orientation.FORWARD:
                        Forward = value;
                        break;
                    case Orientation.BACK:
                        Back = value;
                        break;
                }
            }
        }

        public ThrustEnvelope() { }

        public ThrustEnvelope(Vector3d positive, Vector3d negative)
        {
            Positive = positive;
            Negative = negative;
        }

        public void Reset()
        {
            Positive = Vector3d.zero;
            Negative = Vector3d.zero;
        }

        public void Add(Vector3d v)
        {
            for (int i = 0; i < OrientationValues.Length; i++)
            {
                Orientation o = OrientationValues[i];
                double projection = Vector3d.Dot(v, Orientations[(int) o]);
                if (projection < 0) this[o] += projection;
            }
        }

        public double GetMagnitude(Vector3d d)
        {
            double sqrMag = 0;
            for (int i = 0; i < OrientationValues.Length; i++)
            {
                Orientation o = OrientationValues[i];
                double projection = Vector3d.Dot(d.normalized, Orientations[(int)o]);
                
                if (projection < 0) sqrMag += Math.Pow(projection * this[o], 2);
            }

            return Math.Sqrt(sqrMag);
        }

        public double MaxMagnitude() => Math.Max(MathHelpers.MaxMagnitude(Positive), MathHelpers.MaxMagnitude(Negative));
        // note: CLAUDE - what's our actual input here?  mechjeb takes ConfigNode (an empty class)
        public void Init(Vector3d v, bool isPositive)
        {
            if (isPositive)
            {
                Positive = v;
            }
            else
            {
                 Negative = v;
            }
        }
    }
}
