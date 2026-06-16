using System;
using UnityEngine;

namespace Blackbird.Mathematics
{
    // Two-body (Keplerian) primitives shared by the conic rendezvous planners: Stumpff functions
    // and universal-variable state propagation. Pure double math (only Vector3d from UnityEngine),
    // so it is unit-testable offline with no KSP runtime. This is the always-available conic floor
    // the rendezvous contract plans against: Principia (when present) supplies truer CURRENT state,
    // and the closed loop absorbs the conic-vs-n-body gap — we never integrate n-body to plan a burn.
    public static class TwoBody
    {
        private const double ConvergenceTolerance = 1e-10;   // on the universal anomaly chi
        private const int MaxIterations = 80;

        // Stumpff function C2(psi). psi = alpha * chi^2 (positive elliptic, negative hyperbolic).
        // Closed forms per regime; the small-psi branch returns the series limit 1/2.
        public static double StumpffC2(double psi)
        {
            if (psi > 1e-6)
            {
                double s = Math.Sqrt(psi);
                return (1.0 - Math.Cos(s)) / psi;
            }
            if (psi < -1e-6)
            {
                double s = Math.Sqrt(-psi);
                return (Math.Cosh(s) - 1.0) / (-psi);
            }
            return 0.5;
        }

        // Stumpff function C3(psi). Small-psi branch returns the series limit 1/6.
        public static double StumpffC3(double psi)
        {
            if (psi > 1e-6)
            {
                double s = Math.Sqrt(psi);
                return (s - Math.Sin(s)) / Math.Sqrt(psi * psi * psi);
            }
            if (psi < -1e-6)
            {
                double s = Math.Sqrt(-psi);
                return (Math.Sinh(s) - s) / Math.Sqrt((-psi) * (-psi) * (-psi));
            }
            return 1.0 / 6.0;
        }

        // Propagates a body-relative state (position + velocity) forward by dt seconds along its
        // two-body conic, using the universal-variable formulation (Vallado, Algorithm 8). Handles
        // elliptic and hyperbolic orbits and forward/backward dt. Returns false if inputs are
        // non-finite or the bounded Newton iteration cannot produce a finite result.
        public static bool Propagate(Vector3d r0, Vector3d v0, double mu, double dt,
                                     out Vector3d r, out Vector3d v)
        {
            r = r0;
            v = v0;

            if (mu <= 0.0 || !IsFinite(r0) || !IsFinite(v0) || double.IsNaN(dt) || double.IsInfinity(dt))
                return false;
            if (Math.Abs(dt) < 1e-12)
                return true;   // no motion; r0/v0 already assigned

            double sqrtMu = Math.Sqrt(mu);
            double r0mag = r0.magnitude;
            if (r0mag <= 0.0) return false;

            double v0mag = v0.magnitude;
            double rDotV = Vector3d.Dot(r0, v0);
            double alpha = 2.0 / r0mag - v0mag * v0mag / mu;   // reciprocal of semi-major axis (1/a)

            // Initial guess for the universal anomaly chi.
            double chi;
            if (alpha > 1e-9)                                  // ellipse
            {
                chi = sqrtMu * dt * alpha;
            }
            else if (alpha < -1e-9)                            // hyperbola
            {
                double a = 1.0 / alpha;
                double sign = Math.Sign(dt);
                chi = sign * Math.Sqrt(-a) *
                      Math.Log((-2.0 * mu * alpha * dt) /
                               (rDotV + sign * Math.Sqrt(-mu * a) * (1.0 - r0mag * alpha)));
            }
            else                                               // near-parabolic
            {
                chi = sqrtMu * dt / r0mag;
            }

            double psi = 0.0, c2 = 0.5, c3 = 1.0 / 6.0, rmag = r0mag;
            for (int i = 0; i < MaxIterations; i++)
            {
                psi = chi * chi * alpha;
                c2 = StumpffC2(psi);
                c3 = StumpffC3(psi);

                rmag = chi * chi * c2
                     + (rDotV / sqrtMu) * chi * (1.0 - psi * c3)
                     + r0mag * (1.0 - psi * c2);

                double timeComputed = chi * chi * chi * c3
                                    + (rDotV / sqrtMu) * chi * chi * c2
                                    + r0mag * chi * (1.0 - psi * c3);

                double chiNext = chi + (sqrtMu * dt - timeComputed) / rmag;
                if (Math.Abs(chiNext - chi) < ConvergenceTolerance) { chi = chiNext; break; }
                chi = chiNext;
            }

            // Final Lagrange coefficients from the converged chi.
            psi = chi * chi * alpha;
            c2 = StumpffC2(psi);
            c3 = StumpffC3(psi);

            double f = 1.0 - (chi * chi / r0mag) * c2;
            double g = dt - (chi * chi * chi / sqrtMu) * c3;
            Vector3d rVec = f * r0 + g * v0;

            double rNewMag = rVec.magnitude;
            if (rNewMag <= 0.0) return false;

            double fDot = (sqrtMu / (rNewMag * r0mag)) * chi * (psi * c3 - 1.0);
            double gDot = 1.0 - (chi * chi / rNewMag) * c2;

            r = rVec;
            v = fDot * r0 + gDot * v0;
            return IsFinite(r) && IsFinite(v);
        }

        public static bool IsFinite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
        public static bool IsFinite(Vector3d vec) => IsFinite(vec.x) && IsFinite(vec.y) && IsFinite(vec.z);
    }
}
