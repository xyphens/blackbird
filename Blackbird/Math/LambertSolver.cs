using System;
using UnityEngine;

namespace Blackbird.Mathematics
{
    // Outcome of a single-revolution Lambert solve: the two velocity vectors that put a conic
    // transfer through r1 and r2 in the requested time of flight. Success is false when no valid
    // single-rev solution was found within the bounded iteration budget.
    public struct LambertResult
    {
        public bool Success;
        public Vector3d V1;   // velocity at r1 (departure)
        public Vector3d V2;   // velocity at r2 (arrival)
    }

    public struct AlgLibArgs
    {
        public Vector3d R1;
        public Vector3d V1;
        public Vector3d R2;
        public Vector3d V2;
        public double Mu;
        public bool Capture;
    }

    // Universal-variable, single-revolution Lambert solver (Vallado, Algorithm 58). Given two
    // body-relative position vectors, a time of flight and mu, it finds the connecting conic via
    // bisection on the universal variable psi — so the iteration count (and thus runtime) is
    // bounded. Pure double math, unit-testable offline. This is GENERAL orbital math; the
    // rendezvous intercept planner layers the arrival-time sweep and ΔV bookkeeping on top.
    public static class LambertSolver
    {
        private const int MaxIterations = 80;
        public static void NlpFunction(double[] x, ref double func, object obj)
        {
            var a = (AlgLibArgs)obj;
            double dt = x[0], tt = x[1], offset = x[2];

            if (tt <= 0) { func = 1e300; return; }

            // chaser propagated to the BURN point (dt from now)
            TwoBody.Propagate(a.R1, a.V1, a.Mu, dt, out Vector3d rBurn, out Vector3d vBurn);
            // target propagated to the ARRIVAL point (dt + tt + offset from now)
            TwoBody.Propagate(a.R2, a.V2, a.Mu, dt + tt + offset, out Vector3d rArr, out Vector3d vArr);

            LambertResult lam = Solve(
                rBurn, rArr, tt, a.Mu,
                prograde: true,
                referenceNormal: Vector3d.Cross(rBurn, vBurn));   // == MJ's V3.Cross(rburn, vburn)
            if (!lam.Success) { func = 1e300; return; }            // ours can fail; Izzo basically doesn't — guard it

            Vector3d dv1 = lam.V1 - vBurn;   // departure burn   (MJ: vi - vburn)
            Vector3d dv2 = vArr - lam.V2;    // arrival burn      (MJ: vf2 - vf)

            func = a.Capture ? dv1.magnitude + dv2.magnitude : dv1.magnitude;
        }
        public static (Vector3d dv1, Vector3d dv2) LambertHohmann(double dt, double tt, double offset, AlgLibArgs args)
        {
            // chaser -> burn point (dt from now);  target -> arrival point (dt + tt + offset from now)
            TwoBody.Propagate(args.R1, args.V1, args.Mu, dt, out Vector3d rBurn, out Vector3d vBurn);
            TwoBody.Propagate(args.R2, args.V2, args.Mu, dt + tt + offset, out Vector3d rArr, out Vector3d vArr);

            LambertResult lam = LambertSolver.Solve(
                rBurn, rArr, tt, args.Mu,
                prograde: true,
                referenceNormal: Vector3d.Cross(rBurn, vBurn));

            if (!lam.Success)
                return (new Vector3d(1e150, 0, 0), new Vector3d(1e150, 0, 0)); // poison: makes the objective blow up

            return (lam.V1 - vBurn, vArr - lam.V2);
        }

        // Solves the transfer r1 -> r2 in time tof about a body of gravitational parameter mu.
        //   prograde         : traverse in the direction of referenceNormal (counterclockwise about it).
        //   referenceNormal  : disambiguates the short vs long way in 3D — pass the target's orbit
        //                      normal (r x v). It only needs to point to the correct side of the plane.
        public static LambertResult Solve(Vector3d r1, Vector3d r2, double tof, double mu,
                                          bool prograde, Vector3d referenceNormal)
        {
            LambertResult fail = new LambertResult { Success = false, V1 = Vector3d.zero, V2 = Vector3d.zero };

            if (mu <= 0.0 || tof <= 0.0 || !MathHelpers.IsFinite(r1) || !MathHelpers.IsFinite(r2))
                return fail;

            double r1mag = r1.magnitude;
            double r2mag = r2.magnitude;
            if (r1mag <= 0.0 || r2mag <= 0.0) return fail;

            double cosDeltaNu = MathHelpers.Clamp(Vector3d.Dot(r1, r2) / (r1mag * r2mag), -1.0, 1.0);

            // Direction of motion: tm = +1 short way, -1 long way, chosen so the arc is prograde.
            Vector3d cross = Vector3d.Cross(r1, r2);
            bool sameDir = Vector3d.Dot(cross, referenceNormal) >= 0.0;
            double tm = (prograde == sameDir) ? 1.0 : -1.0;

            double a = tm * Math.Sqrt(r1mag * r2mag * (1.0 + cosDeltaNu));
            if (Math.Abs(a) < 1e-9) return fail;   // ~180-degree / collinear transfer: plane undefined

            double sqrtMu = Math.Sqrt(mu);
            double tolerance = 1e-6 + 1e-8 * tof;  // seconds; relative slack for large tof

            double psi = 0.0;
            double psiUpper = 4.0 * Math.PI * Math.PI;
            double psiLower = -4.0 * Math.PI;
            double c2 = 0.5, c3 = 1.0 / 6.0;
            double y = 0.0;
            bool converged = false;

            for (int i = 0; i < MaxIterations; i++)
            {
                y = r1mag + r2mag + a * (psi * c3 - 1.0) / Math.Sqrt(c2);

                // Safeguard: if y goes negative for a positive-a transfer, raise the lower bound.
                if (a > 0.0 && y < 0.0)
                {
                    int guard = 0;
                    while (y < 0.0 && guard++ < MaxIterations)
                    {
                        psiLower += 0.1;
                        psi = psiLower;
                        c2 = TwoBody.StumpffC2(psi);
                        c3 = TwoBody.StumpffC3(psi);
                        y = r1mag + r2mag + a * (psi * c3 - 1.0) / Math.Sqrt(c2);
                    }
                }

                if (y < 0.0 || c2 <= 0.0) return fail;

                double chi = Math.Sqrt(y / c2);
                double timeComputed = (chi * chi * chi * c3 + a * Math.Sqrt(y)) / sqrtMu;

                if (Math.Abs(timeComputed - tof) < tolerance) { converged = true; break; }

                if (timeComputed <= tof) psiLower = psi; else psiUpper = psi;
                psi = (psiUpper + psiLower) / 2.0;
                c2 = TwoBody.StumpffC2(psi);
                c3 = TwoBody.StumpffC3(psi);
            }

            if (!converged || y <= 0.0) return fail;

            double f = 1.0 - y / r1mag;
            double gDot = 1.0 - y / r2mag;
            double g = a * Math.Sqrt(y / mu);
            if (Math.Abs(g) < 1e-12) return fail;

            Vector3d v1 = (r2 - f * r1) / g;
            Vector3d v2 = (gDot * r2 - r1) / g;
            if (!MathHelpers.IsFinite(v1) || !MathHelpers.IsFinite(v2)) return fail;

            return new LambertResult { Success = true, V1 = v1, V2 = v2 };
        }
    }
}
