using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Power
{
    /// <summary>The robot's arm-power state as the operator knows it: <see cref="Unknown"/> until a status arrives.</summary>
    public enum OperatorArmState
    {
        /// <summary>No status yet, or none for a while: an older RobotHost, or no robot.</summary>
        Unknown,
        Holding,
        Parking,
        Relaxed,
    }

    /// <summary>
    /// The operator's side of "park, then relax" (docs/adr/0023 decision 5): the three motion modes and
    /// the operator messages they need (docs/adr/0018 §9). Plain C#, Unity-free, time injected,
    /// allocation-free; the Bridge sends what <see cref="TryTakeMessage"/> returns and feeds every
    /// status to <see cref="OnStatus"/>.
    /// <list type="bullet">
    /// <item><b>Off</b> (set explicitly): no joint commands; a relax request is repeated until the
    /// status answers it (parking, relaxed, or refused). The mode the bridge starts in is Off too, but
    /// nothing is requested until the operator sets a mode: the arm may already be resting.</item>
    /// <item><b>Paused</b>: no joint commands and no request; a relaxed arm stays relaxed.</item>
    /// <item><b>On</b>: if the robot reports parking or relaxed, a hold request first, repeated until a
    /// status says holding; no joint command until then (<see cref="MayCommand"/>).</item>
    /// </list>
    /// A heartbeat (request none) goes once a second whatever the mode, so RobotHost knows a headset is
    /// there (automatic recording, decision 4) and in which mode.
    /// </summary>
    public sealed class OperatorArmPower
    {
        private readonly long _tps;
        private readonly long _heartbeatTicks;
        private readonly long _repeatTicks;
        private readonly long _slowRepeatTicks;
        private readonly long _staleTicks;

        private uint _sequence;
        private long _lastSentTicks = long.MinValue;
        private bool _relaxWanted;
        private bool _holdWanted;
        private uint _requestSequence;
        private bool _requestSent;
        private ArmPowerState _robotState;
        private bool _haveStatus;
        private long _lastStatusTicks;
        private bool _resumePending;
        private bool _wasDown;

        /// <param name="ticksPerSecond">The operator clock's rate.</param>
        /// <param name="heartbeatSeconds">How often a heartbeat goes when nothing is requested (1 s).</param>
        /// <param name="repeatSeconds">How often an unanswered request is repeated (0.3 s; once a second until any status has arrived).</param>
        /// <param name="staleSeconds">No status for this long: the state is <see cref="OperatorArmState.Unknown"/> again (3 s).</param>
        public OperatorArmPower(long ticksPerSecond, double heartbeatSeconds = 1.0, double repeatSeconds = 0.3, double staleSeconds = 3.0)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
            _heartbeatTicks = (long)(heartbeatSeconds * ticksPerSecond);
            _repeatTicks = (long)(repeatSeconds * ticksPerSecond);
            _slowRepeatTicks = Math.Max(_repeatTicks, _heartbeatTicks);
            _staleTicks = (long)(staleSeconds * ticksPerSecond);
        }

        /// <summary>The operator's mode. Off at start, with nothing requested.</summary>
        public OperatorMotionMode Mode { get; private set; } = OperatorMotionMode.Off;

        /// <summary>
        /// False: no relax or hold request is ever sent (the config does not allow commanding the robot);
        /// heartbeats still go. True by default.
        /// </summary>
        public bool RequestsAllowed { get; set; } = true;

        /// <summary>Sets the mode, as the operator's buttons do. Setting Off asks for a relax even when the mode already was Off.</summary>
        public void SetMode(OperatorMotionMode mode, long now)
        {
            Mode = mode;
            _requestSent = false;
            _relaxWanted = mode == OperatorMotionMode.Off;
            _holdWanted = mode == OperatorMotionMode.On && RobotDown(now);
            // A changed mode goes out at once, in a request or the next heartbeat.
            _lastSentTicks = long.MinValue;
        }

        /// <summary>A status from RobotHost.</summary>
        public void OnStatus(ArmPowerState state, ArmPowerReason reason, uint echoSequence, float remainingDeg, long now)
        {
            _robotState = state;
            _haveStatus = true;
            _lastStatusTicks = now;
            Reason = reason;
            RemainingDeg = remainingDeg;
            bool answersRequest = _requestSent && unchecked((int)(echoSequence - _requestSequence)) >= 0;

            if (_relaxWanted && answersRequest && (state != ArmPowerState.Holding || reason != ArmPowerReason.None))
            {
                // Parking, relaxed, or refused: answered. A refusal stays in Reason; the operator sets Off again to retry.
                _relaxWanted = false;
                _requestSent = false;
            }

            bool down = state == ArmPowerState.Parking || state == ArmPowerState.Relaxed;
            if (Mode == OperatorMotionMode.On)
            {
                if (down && !_holdWanted)
                {
                    _holdWanted = true;
                    _requestSent = false;
                    _lastSentTicks = long.MinValue;
                }
                else if (_holdWanted && answersRequest && state == ArmPowerState.Holding)
                {
                    _holdWanted = false;
                    _requestSent = false;
                    _resumePending = _wasDown;
                }
            }

            if (down) _wasDown = true;
            else if (!_holdWanted) _wasDown = false;
        }

        /// <summary>
        /// The operator message due now, if any: a relax or hold request (repeated until answered), or a
        /// heartbeat (<see cref="ArmPowerRequest.None"/>) once a second. The sequence numbers every message.
        /// </summary>
        public bool TryTakeMessage(long now, out ArmPowerRequest request, out uint sequence)
        {
            request = ArmPowerRequest.None;
            if (RequestsAllowed && _relaxWanted) request = ArmPowerRequest.Relax;
            else if (RequestsAllowed && _holdWanted) request = ArmPowerRequest.Hold;

            long interval = request == ArmPowerRequest.None ? _heartbeatTicks : (_haveStatus ? _repeatTicks : _slowRepeatTicks);
            if (_lastSentTicks != long.MinValue && now - _lastSentTicks < interval)
            {
                sequence = 0;
                return false;
            }

            _lastSentTicks = now;
            sequence = ++_sequence;
            if (request != ArmPowerRequest.None && !_requestSent)
            {
                _requestSent = true;
                _requestSequence = sequence;
            }

            return true;
        }

        /// <summary>
        /// Whether joint commands may go now: the mode is On, no hold is waiting for its answer, and the
        /// robot does not report parking or relaxed. An unknown state (an older RobotHost) does not block.
        /// </summary>
        public bool MayCommand(long now) => Mode == OperatorMotionMode.On && !_holdWanted && !RobotDown(now);

        /// <summary>True once each time a hold has been confirmed after the arm was parking or relaxed.</summary>
        public bool TakeResumed()
        {
            bool resumed = _resumePending;
            _resumePending = false;
            return resumed;
        }

        /// <summary>The robot's state by its latest status, or unknown when there is none recent.</summary>
        public OperatorArmState RobotState(long now)
        {
            if (!_haveStatus || now - _lastStatusTicks > _staleTicks) return OperatorArmState.Unknown;
            switch (_robotState)
            {
                case ArmPowerState.Holding: return OperatorArmState.Holding;
                case ArmPowerState.Parking: return OperatorArmState.Parking;
                default: return OperatorArmState.Relaxed;
            }
        }

        /// <summary>The latest status's reason: why a relax was refused or ended in holding; None otherwise.</summary>
        public ArmPowerReason Reason { get; private set; }

        /// <summary>The latest status's distance from the rest pose, degrees; NaN when unknown.</summary>
        public float RemainingDeg { get; private set; } = float.NaN;

        /// <summary>A relax or hold request is out and not answered yet.</summary>
        public bool RequestPending => RequestsAllowed && (_relaxWanted || _holdWanted);

        /// <summary>Seconds since the latest status; infinity before the first.</summary>
        public float SecondsSinceStatus(long now) => _haveStatus ? (now - _lastStatusTicks) / (float)_tps : float.PositiveInfinity;

        private bool RobotDown(long now)
        {
            OperatorArmState s = RobotState(now);
            return s == OperatorArmState.Parking || s == OperatorArmState.Relaxed;
        }
    }
}
