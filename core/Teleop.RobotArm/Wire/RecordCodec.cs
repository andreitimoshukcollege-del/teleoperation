using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Wire
{
    /// <summary>What the headset asks of scenario recording (docs/adr/0033).</summary>
    public enum RecordOp : byte
    {
        /// <summary>Start a take of the chosen scenario.</summary>
        Start = 1,

        /// <summary>Stop the take being recorded, and keep it.</summary>
        Stop = 2,

        /// <summary>Set the last take aside (into the folder's <c>_discarded</c>), while nothing records.</summary>
        DiscardLast = 3,
    }

    /// <summary>Whether RobotHost records scenarios, and whether a take is going.</summary>
    public enum RecordState : byte
    {
        /// <summary>RobotHost runs without <c>--scenarios</c>: there is nothing to record into.</summary>
        Unavailable = 0,
        Idle = 1,
        Recording = 2,
    }

    /// <summary>The answer to the last request.</summary>
    public enum RecordOutcome : byte
    {
        None = 0,
        Started = 1,
        Saved = 2,
        Discarded = 3,

        /// <summary>A take is already going (Start, Discard), or none is (Stop).</summary>
        Busy = 4,

        /// <summary>No scenario at that index of that list revision (the list changed).</summary>
        UnknownScenario = 5,

        /// <summary>Less free disk than a take needs to start, or a take stopped because the disk filled.</summary>
        LowDisk = 6,

        /// <summary>The session could not be opened, or the take could not be moved aside.</summary>
        Failed = 7,

        /// <summary>No take of this RobotHost's to set aside (it keeps only the last one it recorded).</summary>
        NothingToDiscard = 8,

        Unavailable = 9,

        /// <summary>A take stopped by itself: the headset stopped sending, or the take reached its longest.</summary>
        StoppedByItself = 10,
    }

    /// <summary>RobotHost's recording status, as the headset sees it.</summary>
    public readonly struct RecordStatus : IEquatable<RecordStatus>
    {
        public const byte NoScenario = 255;

        public readonly RecordState State;
        public readonly RecordOutcome Outcome;
        public readonly uint Sequence;
        public readonly byte Scenario;
        public readonly byte ScenarioCount;
        public readonly ushort ListRevision;
        public readonly uint ElapsedMs;
        public readonly ushort Take;
        public readonly ushort TakesKept;
        public readonly ushort FreeGigabytes;

        public RecordStatus(RecordState state, RecordOutcome outcome, uint sequence, byte scenario, byte scenarioCount, ushort listRevision,
            uint elapsedMs, ushort take, ushort takesKept, ushort freeGigabytes)
        {
            State = state;
            Outcome = outcome;
            Sequence = sequence;
            Scenario = scenario;
            ScenarioCount = scenarioCount;
            ListRevision = listRevision;
            ElapsedMs = elapsedMs;
            Take = take;
            TakesKept = takesKept;
            FreeGigabytes = freeGigabytes;
        }

        public bool Equals(RecordStatus o) => State == o.State && Outcome == o.Outcome && Sequence == o.Sequence && Scenario == o.Scenario &&
            ScenarioCount == o.ScenarioCount && ListRevision == o.ListRevision && ElapsedMs == o.ElapsedMs && Take == o.Take &&
            TakesKept == o.TakesKept && FreeGigabytes == o.FreeGigabytes;

        public override bool Equals(object? obj) => obj is RecordStatus o && Equals(o);

        public override int GetHashCode() => HashCode.Combine(State, Outcome, Sequence, Scenario, ScenarioCount, ListRevision, ElapsedMs, HashCode.Combine(Take, TakesKept, FreeGigabytes));
    }

    /// <summary>
    /// Scenario recording on the joint port (docs/adr/0033), little-endian. Every decoder on the joint port checks its first
    /// byte exactly, so an older peer ignores these. They are control, not robot state: the headset applies them as they
    /// arrive, outside its network disturbance and playout.
    /// <list type="table">
    /// <item><term>0x88 request</term><description>headset → RobotHost, 20 bytes: tag, version, op (<see cref="RecordOp"/>), scenario
    /// index, uint32 sequence (not 0), int64 operator send ticks, uint16 the list revision the index is into, uint16 reserved.
    /// Re-sent until the status echoes the sequence.</description></item>
    /// <item><term>0x89 status</term><description>RobotHost → headset, 32 bytes: tag, version, state (<see cref="RecordState"/>),
    /// outcome of the last request (<see cref="RecordOutcome"/>), uint32 its sequence, uint8 scenario (the take's, or the last
    /// one's; 255 none), uint8 scenario count, uint16 list revision, uint32 the take's elapsed milliseconds, uint16 the take's
    /// number, uint16 takes kept of that scenario, uint16 free disk in GB, uint16 reserved, int64 robot send ticks. Once a
    /// second and on change.</description></item>
    /// <item><term>0x8A scenario list</term><description>RobotHost → headset, at most 256 bytes: tag, version, uint16 revision,
    /// uint8 part, uint8 parts, uint8 count (at most <see cref="NamesPerPart"/>), then per name uint8 length (1 to
    /// <see cref="MaxNameLength"/>) and that many ASCII bytes. Every few seconds and when the list changes.</description></item>
    /// </list>
    /// Allocation-free, except <see cref="TryReadName"/>, which makes the name's string.
    /// </summary>
    public static class RecordCodec
    {
        public const byte RequestTag = 0x88;
        public const byte StatusTag = 0x89;
        public const byte ListTag = 0x8A;
        public const byte Version = 1;
        public const int RequestBytes = 20;
        public const int StatusBytes = 32;
        public const int ListHeaderBytes = 7;
        public const int MaxNameLength = 24;
        public const int NamesPerPart = 8;
        public const int MaxScenarios = 16;

        /// <summary>A scenario name: 1 to 24 of letters, digits, '.', '_' and '-' (it names a folder).</summary>
        public static bool IsValidName(string? name)
        {
            if (name == null || name.Length < 1 || name.Length > MaxNameLength || name == "." || name == "..") return false;
            foreach (char c in name)
            {
                bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
                if (!ok) return false;
            }

            return name[0] != '_' && name[0] != '.'; // _discarded and dot-folders are the recorder's own
        }

        public static bool TryEncodeRequest(RecordOp op, byte scenario, ushort listRevision, uint sequence, long operatorSendTicks, Span<byte> output, out int written)
        {
            written = 0;
            if (output.Length < RequestBytes || sequence == 0 || op < RecordOp.Start || op > RecordOp.DiscardLast) return false;
            output[0] = RequestTag;
            output[1] = Version;
            output[2] = (byte)op;
            output[3] = scenario;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(4), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), operatorSendTicks);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(16), listRevision);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(18), 0);
            written = RequestBytes;
            return true;
        }

        public static bool TryDecodeRequest(ReadOnlySpan<byte> source, out RecordOp op, out byte scenario, out ushort listRevision, out uint sequence, out long operatorSendTicks)
        {
            op = 0;
            scenario = 0;
            listRevision = 0;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length != RequestBytes || source[0] != RequestTag || source[1] != Version) return false;
            if (source[2] < (byte)RecordOp.Start || source[2] > (byte)RecordOp.DiscardLast) return false;
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            if (sequence == 0) return false;
            op = (RecordOp)source[2];
            scenario = source[3];
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(8));
            listRevision = BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(16));
            return true;
        }

        public static bool TryEncodeStatus(in RecordStatus s, long robotSendTicks, Span<byte> output, out int written)
        {
            written = 0;
            if (output.Length < StatusBytes) return false;
            output[0] = StatusTag;
            output[1] = Version;
            output[2] = (byte)s.State;
            output[3] = (byte)s.Outcome;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(4), s.Sequence);
            output[8] = s.Scenario;
            output[9] = s.ScenarioCount;
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(10), s.ListRevision);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(12), s.ElapsedMs);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(16), s.Take);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(18), s.TakesKept);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(20), s.FreeGigabytes);
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(22), 0);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(24), robotSendTicks);
            written = StatusBytes;
            return true;
        }

        public static bool TryDecodeStatus(ReadOnlySpan<byte> source, out RecordStatus status, out long robotSendTicks)
        {
            status = default;
            robotSendTicks = 0;
            if (source.Length != StatusBytes || source[0] != StatusTag || source[1] != Version || source[2] > (byte)RecordState.Recording) return false;
            status = new RecordStatus((RecordState)source[2], (RecordOutcome)source[3], BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4)),
                source[8], source[9], BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(10)), BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(12)),
                BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(16)), BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(18)),
                BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(20)));
            robotSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(24));
            return true;
        }

        /// <summary>One part of the list: names <paramref name="first"/> to first + count of <paramref name="names"/>.</summary>
        public static bool TryEncodeList(ushort revision, byte part, byte parts, string[] names, int first, int count, Span<byte> output, out int written)
        {
            written = 0;
            if (names == null || count < 0 || count > NamesPerPart || first < 0 || first + count > names.Length || parts == 0 || part >= parts) return false;
            if (output.Length < ListHeaderBytes) return false;
            output[0] = ListTag;
            output[1] = Version;
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(2), revision);
            output[4] = part;
            output[5] = parts;
            output[6] = (byte)count;
            int at = ListHeaderBytes;
            for (int i = 0; i < count; i++)
            {
                string name = names[first + i];
                if (!IsValidName(name) || at + 1 + name.Length > output.Length) return false;
                output[at++] = (byte)name.Length;
                for (int k = 0; k < name.Length; k++) output[at++] = (byte)name[k];
            }

            written = at;
            return true;
        }

        /// <summary>A list part's header; the names follow at <see cref="ListHeaderBytes"/> (read them with <see cref="TryReadName"/>).</summary>
        public static bool TryDecodeListHeader(ReadOnlySpan<byte> source, out ushort revision, out byte part, out byte parts, out byte count)
        {
            revision = 0;
            part = parts = count = 0;
            if (source.Length < ListHeaderBytes || source[0] != ListTag || source[1] != Version) return false;
            revision = BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(2));
            part = source[4];
            parts = source[5];
            count = source[6];
            return parts > 0 && part < parts && count <= NamesPerPart;
        }

        /// <summary>The name at <paramref name="offset"/>, which moves past it; false when it is cut off or not a valid name.</summary>
        public static bool TryReadName(ReadOnlySpan<byte> source, ref int offset, out string name)
        {
            name = "";
            if (offset >= source.Length) return false;
            int length = source[offset];
            if (length < 1 || length > MaxNameLength || offset + 1 + length > source.Length) return false;
            var chars = new char[length];
            for (int k = 0; k < length; k++) chars[k] = (char)source[offset + 1 + k];
            string s = new string(chars);
            if (!IsValidName(s)) return false;
            offset += 1 + length;
            name = s;
            return true;
        }
    }
}
