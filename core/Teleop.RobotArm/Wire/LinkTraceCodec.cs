using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Wire
{
    /// <summary>What a link-trace record holds (docs/adr/0035 N1).</summary>
    public enum LinkTraceKind : byte
    {
        /// <summary>A joint-state frame arrived: a robot send, robot tps, arrival, send on the headset's clock (or <see cref="long.MinValue"/>), the reading's sensed time on the robot's clock (0: none).</summary>
        Frame = 1,

        /// <summary>A keepalive echo arrived, as <c>ClockSync</c> takes it: a the headset's keepalive send, b the robot's receipt of it, c the robot's send, d robot tps, e the arrival.</summary>
        RoundTrip = 2,

        /// <summary>The headset's own disturbance changed (the console's sliders, ADDED DELAY): a lag µs, b jitter µs, c loss in ppm, d added delay µs, e when. Frames while any is non-zero are not the real network.</summary>
        Conditions = 3,
    }

    /// <summary>
    /// The headset's link trace (docs/adr/0035 N1, trainer/contract/LINK_TRACE_FORMAT.md): the raw facts of the real network
    /// as the headset sees it, for training and testing the learned predictor on real conditions. A file is one header line
    /// (JSON, written by the host) and then fixed 48-byte records, little-endian:
    /// <c>u8 kind | u8[3] 0 | u32 sequence | i64 a | i64 b | i64 c | i64 d | i64 e</c>, the five fields' meanings by
    /// <see cref="LinkTraceKind"/>. Times are each clock's raw ticks (the headset's at the header's <c>headsetTps</c>, the
    /// robot's at the record's robot tps), so a reader can redo <c>ClockSync</c> as it likes. Pure and allocation-free;
    /// the file is the host's.
    /// </summary>
    public static class LinkTraceCodec
    {
        public const int RecordBytes = 48;

        public static bool TryEncode(LinkTraceKind kind, uint sequence, long a, long b, long c, long d, long e, Span<byte> output)
        {
            if (output.Length < RecordBytes || kind < LinkTraceKind.Frame || kind > LinkTraceKind.Conditions) return false;
            output[0] = (byte)kind;
            output[1] = output[2] = output[3] = 0;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(4), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), a);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(16), b);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(24), c);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(32), d);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(40), e);
            return true;
        }

        public static bool TryDecode(ReadOnlySpan<byte> source, out LinkTraceKind kind, out uint sequence, out long a, out long b, out long c, out long d, out long e)
        {
            kind = 0;
            sequence = 0;
            a = b = c = d = e = 0;
            if (source.Length < RecordBytes || source[0] < (byte)LinkTraceKind.Frame || source[0] > (byte)LinkTraceKind.Conditions) return false;
            kind = (LinkTraceKind)source[0];
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            a = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(8));
            b = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(16));
            c = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(24));
            d = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(32));
            e = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(40));
            return true;
        }
    }
}
