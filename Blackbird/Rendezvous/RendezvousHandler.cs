using System;
using Blackbird.Guidance;
using Blackbird.Logging;
using Blackbird.Mathematics;
using Blackbird.Trajectory;
using UnityEngine;

namespace Blackbird.Rendezvous
{
    // Coordinates the terminal-rendezvous executor with the live vessel — the rendezvous-phase analogue
    // of LaunchHandler. Each frame it builds the world seam, runs the executor, caches the resulting
    // command + relative state, and logs while burning; on the fly-by-wire pass it actuates steering and
    // throttle, but ONLY while a stage is actively burning, so it never fights the player otherwise.
    // Arm/Trigger/Abort/Reset are the user gates surfaced to the UI. A master Engage toggle keeps the
    // whole thing dormant until the user opts in.
    public sealed class RendezvousHandler
    {
        private readonly TerminalRendezvousExecutor _executor = new TerminalRendezvousExecutor();
        private readonly AttitudeControl _attitude = new AttitudeControl();
        private readonly BlackbirdLog _log = new BlackbirdLog(LogContext.Rendezvous);

        private RendezvousCommand _command;
        private bool _hasCommand;
        private bool _engaged;
        private bool _burningLastApply;   // were we actuating thrust on the previous fly-by-wire pass?

        // Live closest-approach monitor: recomputed off the draw path on a throttle so it can be watched
        // collapse during/after a burn, independent of any armed plan.
        private const double CaRecomputeIntervalSeconds = 0.5;
        private const int CaSampleCount = 240;
        private double _lastCaComputeUt = double.NegativeInfinity;

        public bool Engaged => _engaged;
        public RendezvousPhase Phase => _executor.Phase;
        public RendezvousStage Stage => _executor.Stage;
        public bool HasInterceptPlan => _executor.HasInterceptPlan;
        public InterceptSolution InterceptPlan => _executor.InterceptPlan;
        public RendezvousCommand Command => _command;
        public bool HasCommand => _hasCommand;
        public RelativeState Relative { get; private set; }
        public bool HasRelative { get; private set; }
        public Vessel Target { get; private set; }

        // Live (continuously recomputed) closest approach from the CURRENT state, and the time until it.
        public double LiveClosestApproachMeters { get; private set; } = double.NaN;
        public double LiveTimeToClosestApproachSeconds { get; private set; } = double.NaN;

        // Master enable. Disengaging also drops attitude control so the player regains the craft.
        public void Engage() { _engaged = true; }
        public void Disengage() { _engaged = false; _attitude.Reset(); }

        // User gates (pass-through to the executor).
        public bool Arm() => _executor.Arm();
        public bool Trigger() => _executor.Trigger();
        public void Abort() { _executor.Abort(); _attitude.Reset(); }
        public void ResetSequence() { _executor.Reset(); _attitude.Reset(); }

        // Per-frame tick (from BlackBird.Update). Computes the command + relative state and logs while
        // executing. Bounded work; does not actuate (that is ApplyFlightControls).
        public void Update(Vessel active, Vessel target)
        {
            Target = target;
            _hasCommand = false;
            HasRelative = false;

            if (active == null || target == null || ReferenceEquals(active, target)) return;

            // Relative state + live closest approach are computed whenever a target exists, so the panel
            // can be monitored before engaging. The CA scan is throttled to keep it cheap.
            Relative = RelativeState.Compute(active, target);
            HasRelative = true;

            double now = Planetarium.GetUniversalTime();
            if (now - _lastCaComputeUt >= CaRecomputeIntervalSeconds)
            {
                ComputeLiveClosestApproach(active, target);
                _lastCaComputeUt = now;
            }

            if (!_engaged) return;

            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);
            _command = _executor.Update(world);
            _hasCommand = true;

            if (_executor.Phase == RendezvousPhase.Executing)
                _log.Write(_executor.Stage.ToString(), _command, Relative);
        }

        // Scans the next orbital period for the minimum chaser-target separation by propagating both
        // along their two-body conics (the always-available conic floor). Records the distance and the
        // time until it. Bounded sample count; called off the draw path on a throttle.
        private void ComputeLiveClosestApproach(Vessel active, Vessel target)
        {
            CelestialBody body = active.mainBody;
            if (body == null) return;

            double mu = body.gravParameter;
            Vector3d aPos = TrajectoryProvider.GetPosition(active) - body.position;
            Vector3d aVel = TrajectoryProvider.GetVelocity(active);
            Vector3d tPos = TrajectoryProvider.GetPosition(target) - body.position;
            Vector3d tVel = TrajectoryProvider.GetVelocity(target);

            double horizon = OrbitalPeriod(aPos, aVel, mu);
            if (!MathHelpers.IsFinite(horizon) || horizon <= 0.0) horizon = 3600.0;

            double minDistance = double.PositiveInfinity;
            double timeAtMin = 0.0;
            for (int i = 0; i <= CaSampleCount; i++)
            {
                double dt = horizon * i / CaSampleCount;
                if (!TwoBody.Propagate(aPos, aVel, mu, dt, out Vector3d ra, out _)) continue;
                if (!TwoBody.Propagate(tPos, tVel, mu, dt, out Vector3d rt, out _)) continue;

                double distance = (ra - rt).magnitude;
                if (distance < minDistance) { minDistance = distance; timeAtMin = dt; }
            }

            LiveClosestApproachMeters = minDistance;
            LiveTimeToClosestApproachSeconds = timeAtMin;
        }

        // Keplerian period from a state vector; NaN if unbound.
        private static double OrbitalPeriod(Vector3d r, Vector3d v, double mu)
        {
            double rmag = r.magnitude;
            if (rmag <= 0.0 || mu <= 0.0) return double.NaN;

            double energy = 0.5 * v.sqrMagnitude - mu / rmag;
            if (energy >= 0.0) return double.NaN;

            double a = -mu / (2.0 * energy);
            return 2.0 * Math.PI * Math.Sqrt(a * a * a / mu);
        }

        // Fly-by-wire actuation (from BlackBird.OnFlyByWire). Steers along the burn vector and sets
        // throttle only while burning; cuts throttle once on the frame the burn ends, then releases
        // control back to the player.
        public void ApplyFlightControls(FlightCtrlState state, Vessel vessel)
        {
            if (state == null || vessel == null) return;

            bool burning = _engaged && _hasCommand && _command.HasBurn
                           && _command.ThrustDirection.sqrMagnitude > 0.0;

            if (burning)
            {
                _attitude.DriveInertial(vessel, state, _command.ThrustDirection, 0.0);
                state.mainThrottle = Mathf.Clamp01((float)_command.Throttle);
                _burningLastApply = true;
                return;
            }

            if (_burningLastApply)
            {
                state.mainThrottle = 0.0f;   // cut throttle on the first non-burning frame after a burn
                _burningLastApply = false;
            }
        }
    }
}
