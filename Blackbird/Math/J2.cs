using System;
using UnityEngine;

namespace Blackbird.Mathematics
{
    public static class J2Propagator
    {
        // point-mass + J2 gravity in physical units
        public static Vector3d Acceleration(Vector3d r, double mu, double j2, double reEq, Vector3d pole)
        {
            double rMag = r.magnitude;
            if (rMag <= 0.0) return Vector3d.zero;
            Vector3d a = -mu * r / (rMag * rMag * rMag); // more efficient than Math.Pow() apparently
            if (j2 != 0.0 && pole.sqrMagnitude > 0.0)
            {
                double z = Vector3d.Dot(r, pole);
                double r2 = rMag * rMag;
                double coeff = -1.5 * j2 * mu * reEq * reEq / (r2 * r2 * rMag);
                a += coeff * ((1.0 - 5.0 * (z * z) / r2) * r + (2.0 * z) * pole);
            }

            return a;
        }

        public static double NextPeriapsisRadius(Vector3d r0, Vector3d v0, double mu, double j2, double reEq, Vector3d pole, double maxSeconds, double dt)
        {
            Vector3d r = r0, v = v0;
            double t = 0.0, minR = r.magnitude, prevRadial = Vector3d.Dot(r, v);
            while (t < maxSeconds) // scary
            {
                Rk4Step(ref r, ref v, mu, j2, reEq, pole, dt);
                t += dt;
                if (r.magnitude < minR) minR = r.magnitude;
                double radial = Vector3d.Dot(r, v);
                if (prevRadial < 0.0 && radial >= 0.0) break;
                prevRadial = radial;
            }

            return minR;
        }

        // Min and max radius over one orbit (real Pe/Ap under J2). Scans until the radial-velocity sign has
        // flipped twice (one full pe->ap->pe or ap->pe->ap cycle) so both extrema are captured, or maxSeconds.
        public static void RadiusExtremes(Vector3d r0, Vector3d v0, double mu, double j2, double reEq, Vector3d pole, double maxSeconds, double dt, out double minR, out double maxR)
        {
            Vector3d r = r0, v = v0;
            double t = 0.0;
            minR = maxR = r.magnitude;
            double prevRadial = Vector3d.Dot(r, v);
            int flips = 0;
            while (t < maxSeconds)
            {
                Rk4Step(ref r, ref v, mu, j2, reEq, pole, dt);
                t += dt;
                double rm = r.magnitude;
                if (rm < minR) minR = rm;
                if (rm > maxR) maxR = rm;
                double radial = Vector3d.Dot(r, v);
                if ((prevRadial < 0.0) != (radial < 0.0)) flips++;
                if (flips >= 2) break;
                prevRadial = radial;
            }
        }

        private static void Rk4Step(ref Vector3d r, ref Vector3d v, double mu, double j2, double reEq, Vector3d pole, double dt) {
            Vector3d k1r = v, k1v = Acceleration(r, mu, j2, reEq, pole);
            Vector3d k2r = v + (0.5 * dt) * k1v,    k2v = Acceleration(r + (0.5 * dt) * k1r, mu, j2, reEq, pole);
            Vector3d k3r = v + (0.5 * dt) * k2v,    k3v = Acceleration(r + (0.5 * dt) * k2r, mu, j2, reEq, pole);
            Vector3d k4r = v + dt * k3v,            k4v = Acceleration(r + dt * k3r, mu, j2, reEq, pole);
            r += (dt / 6.0) * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
            v += (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);

        }
    }
}
