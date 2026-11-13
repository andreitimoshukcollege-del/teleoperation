using System;
using System.Diagnostics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// The headset's RECORD button, from a tool (docs/adr/0033, docs/adr/0036): RobotHost's scenario list and status from
    /// the joint port's 0x8A and 0x89 datagrams (the headset's own <see cref="RecordingState"/>), and its 0x88 requests,
    /// each sent until the status echoes it. <c>joint-drive --take</c> uses it, so a scripted drive is recorded as a take
    /// exactly as a headset session is, and becomes a LeRobot episode with nobody wearing the headset.
    /// <para>RobotHost answers status and list only to the joint port's subscriber, and keeps a take only while its
    /// sender keeps sending (commands, or the operator message). <c>joint-drive</c> is both, from one socket.</para>
    /// </summary>
    public sealed class TakeClient
    {
        private readonly Action<byte[], int> _send;
        private readonly byte[] _buffer = new byte[RecordCodec.RequestBytes];
        private uint _sequence;

        /// <param name="send">Sends a datagram to RobotHost's joint port, from the socket that subscribes and commands.</param>
        /// <param name="firstSequence">The first request's sequence. RobotHost ignores a request that repeats the last
        /// sequence it answered, from anyone, so a tool starts somewhere a headset is unlikely to have been.</param>
        public TakeClient(Action<byte[], int> send, uint firstSequence)
        {
            _send = send ?? throw new ArgumentNullException(nameof(send));
            _sequence = firstSequence == 0 ? 1 : firstSequence;
        }

        public RecordingState State { get; } = new RecordingState();

        /// <summary>Takes a datagram from the joint port if it is a status or a list part; false for anything else.</summary>
        public bool OnDatagram(ReadOnlySpan<byte> datagram)
        {
            if (RecordCodec.TryDecodeStatus(datagram, out RecordStatus status, out long robotTicks))
            {
                State.ApplyStatus(status, robotTicks);
                return true;
            }

            return datagram.Length > 0 && datagram[0] == RecordCodec.ListTag && State.ApplyListPart(datagram);
        }

        /// <summary>The scenario's index in RobotHost's list, or -1 (no list yet, or no such name).</summary>
        public int IndexOf(string scenario)
        {
            if (!State.HasList) return -1;
            for (int i = 0; i < State.Count; i++)
            {
                if (string.Equals(State.Name(i), scenario, StringComparison.Ordinal)) return i;
            }

            return -1;
        }

        /// <summary>The names RobotHost offers, for an error message.</summary>
        public string Names()
        {
            var names = new string[State.Count];
            for (int i = 0; i < names.Length; i++) names[i] = State.Name(i);
            return string.Join(", ", names);
        }

        /// <summary>A new request (its own sequence); call <see cref="Resend"/> until <see cref="Answered"/>.</summary>
        public uint Begin(RecordOp op, byte scenario, ushort listRevision, long nowTicks)
        {
            _sequence = _sequence == uint.MaxValue ? 1 : _sequence + 1;
            if (!RecordCodec.TryEncodeRequest(op, scenario, listRevision, _sequence, nowTicks, _buffer, out _))
            {
                throw new InvalidOperationException($"cannot encode a {op} request");
            }

            _send(_buffer, RecordCodec.RequestBytes);
            return _sequence;
        }

        /// <summary>Sends the current request again (RobotHost ignores a sequence it has answered).</summary>
        public void Resend() => _send(_buffer, RecordCodec.RequestBytes);

        /// <summary>Whether RobotHost's status echoes request <paramref name="sequence"/>; its outcome is then <c>State.Status.Outcome</c>.</summary>
        public bool Answered(uint sequence) => State.HasStatus && State.Status.Sequence == sequence;

        /// <summary>A sequence to start from: the low bits of the clock, so two runs, or a run and a headset, rarely meet.</summary>
        public static uint FirstSequence() => (uint)(Stopwatch.GetTimestamp() & 0x3FFF_FFFF) | 0x4000_0000;
    }
}
