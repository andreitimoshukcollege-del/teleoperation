using System;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>
    /// Scenario recording as the headset sees it (docs/adr/0033): RobotHost's last status and its scenario list, from the
    /// joint port's 0x89 and 0x8A datagrams (<see cref="RecordCodec"/>). <see cref="SerialChainJointLink"/> fills it. A list
    /// is read again only when its revision changes, so steady state allocates nothing.
    /// </summary>
    public sealed class RecordingState
    {
        private readonly string[] _names = new string[RecordCodec.MaxScenarios];
        private readonly string[] _incoming = new string[RecordCodec.MaxScenarios];
        private readonly bool[] _partSeen = new bool[(RecordCodec.MaxScenarios + RecordCodec.NamesPerPart - 1) / RecordCodec.NamesPerPart];
        private ushort _incomingRevision;
        private byte _incomingParts;
        private bool _haveIncoming;

        /// <summary>Whether any status has arrived.</summary>
        public bool HasStatus { get; private set; }

        public RecordStatus Status { get; private set; }

        /// <summary>The robot clock's ticks of the last status (its send stamp), for staleness.</summary>
        public long StatusRobotTicks { get; private set; }

        /// <summary>Whether a whole list has arrived.</summary>
        public bool HasList { get; private set; }

        /// <summary>The revision of <see cref="Names"/>.</summary>
        public ushort ListRevision { get; private set; }

        /// <summary>How many names <see cref="Names"/> holds.</summary>
        public int Count { get; private set; }

        /// <summary>The scenario names, in RobotHost's order; the first <see cref="Count"/> are valid.</summary>
        public string Name(int index) => index >= 0 && index < Count ? _names[index] : "";

        /// <summary>Whether the list matches the status (same revision): a request's index means what the operator saw.</summary>
        public bool ListCurrent => HasList && HasStatus && ListRevision == Status.ListRevision;

        public void ApplyStatus(in RecordStatus status, long robotSendTicks)
        {
            Status = status;
            StatusRobotTicks = robotSendTicks;
            HasStatus = true;
        }

        /// <summary>One 0x8A datagram. False when it is not a valid list part.</summary>
        public bool ApplyListPart(ReadOnlySpan<byte> datagram)
        {
            if (!RecordCodec.TryDecodeListHeader(datagram, out ushort revision, out byte part, out byte parts, out byte count)) return false;
            if (parts > _partSeen.Length) return false;
            if (HasList && revision == ListRevision) return true; // already have it
            if (!_haveIncoming || revision != _incomingRevision || parts != _incomingParts)
            {
                _haveIncoming = true;
                _incomingRevision = revision;
                _incomingParts = parts;
                Array.Clear(_partSeen, 0, _partSeen.Length);
                Array.Clear(_incoming, 0, _incoming.Length);
            }

            if (_partSeen[part]) return true;
            int offset = RecordCodec.ListHeaderBytes;
            int first = part * RecordCodec.NamesPerPart;
            for (int i = 0; i < count; i++)
            {
                if (first + i >= _incoming.Length || !RecordCodec.TryReadName(datagram, ref offset, out string name)) return false;
                _incoming[first + i] = name;
            }

            _partSeen[part] = true;
            for (int p = 0; p < parts; p++)
            {
                if (!_partSeen[p]) return true;
            }

            int n = 0;
            while (n < _incoming.Length && _incoming[n] != null) n++;
            Array.Copy(_incoming, _names, n);
            Count = n;
            ListRevision = revision;
            HasList = true;
            _haveIncoming = false;
            return true;
        }
    }
}
