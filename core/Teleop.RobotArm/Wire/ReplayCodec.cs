using System;
using System.Buffers.Binary;
using System.Globalization;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Wire
{
    /// <summary>A <c>mixed</c> take's place in its replay group (docs/adr/0038 decision 2).</summary>
    public enum ReplayRole : byte
    {
        /// <summary>The take as planned, on the layout as RobotHost drew it.</summary>
        Source = 1,

        /// <summary>The source's commanded joint stream sent again, on the source's layout with one edit.</summary>
        Replay = 2,
    }

    /// <summary>The one edit a replay makes to its source's layout (docs/adr/0038 decision 2).</summary>
    public enum ReplayEdit : byte
    {
        /// <summary>The layout as it was (a source take).</summary>
        None = 0,

        /// <summary>One object taken off the table (parked).</summary>
        Remove = 1,

        /// <summary>One object shifted on the table by (<c>A</c>, <c>B</c>), metres in the arm base frame.</summary>
        Shift = 2,

        /// <summary>One object stood elsewhere: at (<c>A</c>, <c>B</c>), metres in the arm base frame, turned to yaw <c>C</c> radians.</summary>
        Move = 3,
    }

    /// <summary>
    /// A <c>mixed</c> take's replay group, tool → robot on the joint port (docs/adr/0038 decision 2), little-endian, 40 bytes:
    /// <c>u8 0x8E | u8 version (1) | u8 role | u8 edit | u8 objectId | u8 0 | u16 0 | u32 sequence | u64 group | f32 a | f32 b |
    /// f32 c | i64 operatorSendTicks</c>. <c>joint-drive</c> sends it once a take of the script has started: <c>group</c> names the
    /// source take (its <c>--seed</c>), the same for every replay of it; a replay re-sends the source's commanded joint stream at
    /// the same relative times on the source's layout with one edit (<see cref="ReplayEdit"/>; <c>a</c>, <c>b</c>, <c>c</c> its
    /// numbers, 0 when unused). RobotHost records it in the session as <c>RP</c>, once per sequence (the tool sends it a few times).
    /// It moves nothing. Every decoder on the joint port checks its first byte exactly, so an older RobotHost ignores it.
    /// </summary>
    public static class ReplayCodec
    {
        public const byte Tag = 0x8E;
        public const byte Version = 1;
        public const int Size = 12 + 8 + 3 * 4 + 8;

        public static bool IsReplay(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == Tag;

        public static bool TryEncode(in ReplayRecord r, uint sequence, long operatorSendTicks, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = Size;
            if (destination.Length < Size)
            {
                return false;
            }

            destination[0] = Tag;
            destination[1] = Version;
            destination[2] = (byte)r.Role;
            destination[3] = (byte)r.Edit;
            destination[4] = r.ObjectId;
            destination[5] = 0;
            destination[6] = 0;
            destination[7] = 0;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(8), sequence);
            BinaryPrimitives.WriteUInt64LittleEndian(destination.Slice(12), r.Group);
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(20), BitConverter.SingleToInt32Bits(r.A));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(24), BitConverter.SingleToInt32Bits(r.B));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(28), BitConverter.SingleToInt32Bits(r.C));
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(32), operatorSendTicks);
            return true;
        }

        /// <summary>
        /// Reads a replay record. False for a short record, another tag or version, a role or edit this build does not define, or a
        /// source with an edit.
        /// </summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out ReplayRecord r, out uint sequence, out long operatorSendTicks)
        {
            r = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < Size || source[0] != Tag || source[1] != Version) return false;
            if (source[2] < (byte)ReplayRole.Source || source[2] > (byte)ReplayRole.Replay || source[3] > (byte)ReplayEdit.Move) return false;
            if (source[2] == (byte)ReplayRole.Source && source[3] != (byte)ReplayEdit.None) return false;
            r = new ReplayRecord((ReplayRole)source[2], BinaryPrimitives.ReadUInt64LittleEndian(source.Slice(12)), (ReplayEdit)source[3], source[4],
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(20))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(24))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(28))));
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(8));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(32));
            return true;
        }

        public static string Name(ReplayRole role) => role switch
        {
            ReplayRole.Source => "source",
            ReplayRole.Replay => "replay",
            _ => "unknown",
        };

        /// <summary>
        /// The edit as the session's <c>RP</c> record writes it (docs/adr/0038 decision 2): <c>none</c>, <c>remove:&lt;id&gt;</c>,
        /// <c>shift:&lt;id&gt;:&lt;dx&gt;:&lt;dy&gt;</c> (m, arm base frame) or <c>move:&lt;id&gt;:&lt;x&gt;:&lt;y&gt;:&lt;yawRad&gt;</c>, the
        /// numbers in round-trip form.
        /// </summary>
        public static string Describe(in ReplayRecord r)
        {
            string id = r.ObjectId.ToString(CultureInfo.InvariantCulture);
            return r.Edit switch
            {
                ReplayEdit.Remove => "remove:" + id,
                ReplayEdit.Shift => "shift:" + id + ":" + F(r.A) + ":" + F(r.B),
                ReplayEdit.Move => "move:" + id + ":" + F(r.A) + ":" + F(r.B) + ":" + F(r.C),
                _ => "none",
            };
        }

        private static string F(float value) => value.ToString("R", CultureInfo.InvariantCulture);
    }

    /// <summary>A <c>mixed</c> take's replay group (<see cref="ReplayCodec"/>): its role, the source take it belongs to, and the one edit a replay makes.</summary>
    public readonly struct ReplayRecord : IEquatable<ReplayRecord>
    {
        public readonly ReplayRole Role;
        public readonly ulong Group;
        public readonly ReplayEdit Edit;
        public readonly byte ObjectId;
        public readonly float A, B, C;

        public ReplayRecord(ReplayRole role, ulong group, ReplayEdit edit = ReplayEdit.None, byte objectId = 0, float a = 0f, float b = 0f, float c = 0f)
        {
            Role = role;
            Group = group;
            Edit = edit;
            ObjectId = objectId;
            A = a;
            B = b;
            C = c;
        }

        /// <summary>A source take of group <paramref name="group"/>.</summary>
        public static ReplayRecord Source(ulong group) => new ReplayRecord(ReplayRole.Source, group);

        public bool Equals(ReplayRecord other) =>
            Role == other.Role && Group == other.Group && Edit == other.Edit && ObjectId == other.ObjectId && A.Equals(other.A) && B.Equals(other.B) && C.Equals(other.C);

        public override bool Equals(object? obj) => obj is ReplayRecord other && Equals(other);

        public override int GetHashCode() => HashCode.Combine((byte)Role, Group, (byte)Edit, ObjectId, A, B, C);
    }
}
