using Blackbird.Guidance;
using Blackbird.Logging;
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

            if (!_engaged || active == null || target == null || ReferenceEquals(active, target))
                return;

            VesselRendezvousWorld world = new VesselRendezvousWorld(active, target);
            Relative = RelativeState.Compute(active, target);
            HasRelative = true;

            _command = _executor.Update(world);
            _hasCommand = true;

            if (_executor.Phase == RendezvousPhase.Executing)
                _log.Write(_executor.Stage.ToString(), _command, Relative);
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
