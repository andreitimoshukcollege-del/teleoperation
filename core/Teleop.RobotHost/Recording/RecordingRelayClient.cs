using System;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// The relay client the plant sees while recording is possible: every target sent (<c>RO</c>), every torque switch (<c>TQ</c>) and
    /// every feedback datagram taken (<c>RI</c>) goes to the <see cref="SessionRecorder"/>, then
    /// straight through to the real client. It changes nothing the plant does, and costs a string
    /// only while a session is open.
    /// </summary>
    public sealed class RecordingRelayClient : IRelayClient
    {
        private readonly IRelayClient _inner;
        private readonly SessionRecorder _recorder;
        private readonly ITimeAuthority _clock;

        public RecordingRelayClient(IRelayClient inner, SessionRecorder recorder, ITimeAuthority clock)
        {
            _inner = inner ?? throw new ArgumentNullException(nameof(inner));
            _recorder = recorder ?? throw new ArgumentNullException(nameof(recorder));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
        }

        public void Send(ReadOnlySpan<JointTarget> targets)
        {
            if (_recorder.IsActive) _recorder.Record(SessionLines.RelayOut(_clock.NowTicks, targets));
            _inner.Send(targets);
        }

        public void SendTorque(bool on)
        {
            if (_recorder.IsActive) _recorder.Record(SessionLines.Torque(_clock.NowTicks, on));
            _inner.SendTorque(on);
        }

        public bool TryReceiveFeedback(Span<JointFeedbackEntry> entriesBuffer, out int entryCount)
        {
            bool got = _inner.TryReceiveFeedback(entriesBuffer, out entryCount);
            if (got && _recorder.IsActive) _recorder.Record(SessionLines.RelayIn(_clock.NowTicks, entriesBuffer.Slice(0, entryCount)));
            return got;
        }
    }
}
