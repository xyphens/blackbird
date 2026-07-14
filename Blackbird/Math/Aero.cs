using System;
using System.Reflection;
using UnityEngine;

namespace Blackbird.Mathematics
{
    public static class Aero
    {
        private delegate void VesselAeroForcesDelegate(Vessel vessel, out Vector3 aeroForce, out Vector3 aeroTorque, Vector3 surfaceVelocity, double altitudeAsl);

        private static VesselAeroForcesDelegate _aeroForces;
        private static bool _probed;

        // CdA cache (5 s / part-count keyed)
        private static Guid _cdaVesselId;
        private static int _cdaPartCount;
        private static double _cdaCachedUt;
        private static double _cdaCached;

        // force cache (change-thresholds so the voxel model isn't evaluated per tick)
        private static Guid _lastVesselId;
        private static Vector3d _lastVelocity;
        private static double _lastAltitude;
        private static Vector3 _lastForce;


        // Cd*A (m^2), nose-first: FAR voxel sampled at transonic/11 km, else stock drag cubes.
        public static double DragAreaCd(Vessel vessel)
        {
            if (vessel == null || vessel.ReferenceTransform == null) return 10.0;
            
            double ut = Planetarium.GetUniversalTime();

            if (vessel.id == _cdaVesselId && vessel.parts.Count == _cdaPartCount && ut - _cdaCachedUt < 5.0)
            {
                return _cdaCached;
            }

            double cdA = 0.0;

            // FAR path: sample the voxel model nose-first near max drag-loss (transonic, ~11 km)
            const double sampleSpeed = 340.0;
            const double sampleAltitude = 11000.0;
            Vector3d sampleVelocity = (Vector3d)vessel.ReferenceTransform.up * sampleSpeed;
            if (TryFARDragNewtons(vessel, sampleVelocity, sampleAltitude, out double farDragN))
            {
                double rho = DensityAt(vessel, sampleAltitude);
                if (rho > 0.0) cdA = farDragN / (0.5 * rho * sampleSpeed * sampleSpeed);
            }

            if (cdA <= 0.0)
            {
                // stock fallback: per-part drag cubes nose-first (FAR zeroes these, hence the branch above)
                Vector3d up = (Vector3d)vessel.ReferenceTransform.up;
                double areaDrag = 0.0;
                foreach (Part p in vessel.Parts)
                {
                    if (p == null || p.ShieldedFromAirstream) continue;
                    if (p.FindModuleImplementing<LaunchClamp>() != null) continue;
                    p.DragCubes.SetDragWeights();
                    p.DragCubes.SetDrag(p.transform.InverseTransformDirection(-up), 1.0f);
                    areaDrag += p.DragCubes.AreaDrag;
                }
                cdA = areaDrag * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragMultiplier;
            }

            if (cdA <= 0.0) return 10.0;   // aero data unavailable; don't cache the fallback
            _cdaCached = cdA;
            _cdaCachedUt = ut;
            _cdaPartCount = vessel.parts.Count;
            _cdaVesselId = vessel.id;
            return _cdaCached;
        }
        public static double DragNewtons(Vessel vessel, Vector3d surfaceVelocity, double altitudeMeters)
        {
            if (vessel == null) return 0.0;
            if (TryFARDragNewtons(vessel, surfaceVelocity, altitudeMeters, out double farDragN)) return farDragN;
            double rho = DensityAt(vessel, altitudeMeters);
            return 0.5 * rho * surfaceVelocity.sqrMagnitude * DragAreaCd(vessel);
        }

        private static double DensityAt(Vessel vessel, double altitudeMeters)
        {
            CelestialBody body = vessel.mainBody;
            if (body == null || altitudeMeters >= body.atmosphereDepth) return 0.0;
            return body.GetDensity(body.GetPressure(altitudeMeters), body.GetTemperature(altitudeMeters));
        }

        private static void ProbeFAR()
        {
            if (_probed) return;
            _probed = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly a in AssemblyLoader.loadedAssemblies)
                {
                    if (a.name != "FerramAerospaceResearch") continue;
                    Type api = a.assembly.GetType("FerramAerospaceResearch.FARAPI");
                    MethodInfo mi = api != null
                                ? api.GetMethod("CalculateVesselAeroForces", BindingFlags.Public | BindingFlags.Static, null,
                                new[] {  typeof(Vessel), typeof(Vector3).MakeByRefType(), typeof(Vector3).MakeByRefType(),
                                         typeof(Vector3), typeof(double)}, null)
                                : null;
                    if (mi != null)
                    {
                        _aeroForces = (VesselAeroForcesDelegate)Delegate.CreateDelegate(typeof(VesselAeroForcesDelegate), mi);
                    }
                    break;
                }
            }
            catch (Exception) {
                _aeroForces = null;
            }
            Debug.Log($"[Blackbird] FAR aero {(_aeroForces != null ? "bound" : "not present; stock aero path")}");
        }

        // drag force (N) oppositing SurfaceVelocity from FAR's voxel model at any velocity/altitude
        private static bool TryFARDragNewtons(Vessel vessel, Vector3d surfaceVelocity, double altitudeMeters, out double dragNewtons)
        {
            dragNewtons = 0.0;
            ProbeFAR();
            if (_aeroForces == null || vessel == null || surfaceVelocity.sqrMagnitude < 1e-6) return false;
         
            bool cacheValid = vessel.id == _lastVesselId
                            && (surfaceVelocity - _lastVelocity).sqrMagnitude < 100.0
                            && Math.Abs(altitudeMeters - _lastAltitude) < 300.0;
            if (!cacheValid)
            {
                try
                {
                    _aeroForces(vessel, out Vector3 force, out Vector3 _, (Vector3)surfaceVelocity, altitudeMeters);
                    _lastForce = force;
                }
                catch(Exception)
                {
                    _aeroForces = null; // API changed underneath us, fall back to stock
                    return false;
                }

                _lastVesselId = vessel.id;
                _lastVelocity = surfaceVelocity;
                _lastAltitude = altitudeMeters;
            }

            dragNewtons = Math.Max(0.0, Vector3d.Dot((Vector3d)_lastForce, -surfaceVelocity.normalized)) * 1000.0; // kN -> N
            return true;
        }
    }
}