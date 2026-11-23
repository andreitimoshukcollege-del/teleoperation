using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Wire
{
    /// <summary>What one planned move of a <c>mixed</c> take does (docs/adr/0038 decision 1).</summary>
    public enum ScriptMoveKind : byte
    {
        /// <summary>A move to a point: a layout-blind goal, or where an aim or a pass starts from.</summary>
        Free = 1,

        /// <summary>A reach at a point on an object's surface, ending at a signed offset from it (below 0: into it).</summary>
        Aim = 2,

        /// <summary>A sweep whose path crosses an object, at a signed offset from its surface (below 0: through it).</summary>
        Pass = 3,

        /// <summary>The hand kept still with the clutch held.</summary>
        Pause = 4,

        /// <summary>Back the way the hand came: a pull back after a move, or back-and-forth strokes.</summary>
        Reverse = 5,
    }

    /// <summary>
    /// One planned move of a <c>mixed</c> take, tool → robot on the joint port (docs/adr/0038 decision 1), little-endian, 44 bytes:
    /// <c>u8 0x8D | u8 version (1) | u8 kind | u8 objectId | u16 index | u16 count | u32 sequence | f32 offsetMm | f32 gapMm |
    /// f32 startSeconds | f32 contactSeconds | f32 closestSeconds | f32 endSeconds | i64 operatorSendTicks</c>. <c>joint-drive</c>
    /// sends it as the move begins (the take's first moves as the take starts); the four times are seconds after that, as planned:
    /// the move begins (0, or below 0 for a move begun before it was sent), its first contact with the object by the collision
    /// model, its closest approach to the object, and its end. <c>index</c> counts the take's moves from 1, <c>count</c> is how many
    /// the take planned. For an aim or a pass, <c>objectId</c> is the object and <c>offsetMm</c> the signed offset the move was
    /// planned at, from the object's own surface (its shape, else its box): above 0 short of it, below 0 into it; <c>gapMm</c> is
    /// the arm's closest approach to the object over the move by the collision model with the twin's links, as deep as the model
    /// measures. For the other kinds the object is 0 and those four are NaN. RobotHost records it in the session as <c>MV</c>, once
    /// per sequence (the tool sends each a few times). A plan for the recording only; it moves nothing. Every decoder on the joint
    /// port checks its first byte exactly, so an older RobotHost ignores it.
    /// </summary>
    public static class ScriptMoveCodec
    {
        public const byte Tag = 0x8D;
        public const byte Version = 1;
        public const int Size = 12 + 6 * 4 + 8;

        public static bool IsScriptMove(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == Tag;

        public static bool TryEncode(in ScriptMove m, uint sequence, long operatorSendTicks, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = Size;
            if (destination.Length < Size)
            {
                return false;
            }

            destination[0] = Tag;
            destination[1] = Version;
            destination[2] = (byte)m.Kind;
            destination[3] = m.ObjectId;
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(4), m.Index);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(6), m.Count);
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(8), sequence);
            WriteSingle(destination.Slice(12), m.OffsetMm);
            WriteSingle(destination.Slice(16), m.GapMm);
            WriteSingle(destination.Slice(20), m.StartSeconds);
            WriteSingle(destination.Slice(24), m.ContactSeconds);
            WriteSingle(destination.Slice(28), m.ClosestSeconds);
            WriteSingle(destination.Slice(32), m.EndSeconds);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(36), operatorSendTicks);
            return true;
        }

        /// <summary>Reads a move. False for a short record, another tag or version, or a kind this build does not define.</summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out ScriptMove m, out uint sequence, out long operatorSendTicks)
        {
            m = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < Size || source[0] != Tag || source[1] != Version || source[2] < (byte)ScriptMoveKind.Free || source[2] > (byte)ScriptMoveKind.Reverse)
            {
                return false;
            }

            m = new ScriptMove(BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(4)), BinaryPrimitives.ReadUInt16LittleEndian(source.Slice(6)),
                (ScriptMoveKind)source[2], source[3], ReadSingle(source.Slice(12)), ReadSingle(source.Slice(16)), ReadSingle(source.Slice(20)),
                ReadSingle(source.Slice(24)), ReadSingle(source.Slice(28)), ReadSingle(source.Slice(32)));
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(8));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(36));
            return true;
        }

        public static string Name(ScriptMoveKind kind) => kind switch
        {
            ScriptMoveKind.Free => "free",
            ScriptMoveKind.Aim => "aim",
            ScriptMoveKind.Pass => "pass",
            ScriptMoveKind.Pause => "pause",
            ScriptMoveKind.Reverse => "reverse",
            _ => "unknown",
        };

        private static void WriteSingle(Span<byte> destination, float value) =>
            BinaryPrimitives.WriteInt32LittleEndian(destination, BitConverter.SingleToInt32Bits(value));

        private static float ReadSingle(ReadOnlySpan<byte> source) => BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source));
    }

    /// <summary>
    /// One planned move of a <c>mixed</c> take (<see cref="ScriptMoveCodec"/>): which, of how many, what, at which object and how
    /// close, and when, seconds after it was sent.
    /// </summary>
    public readonly struct ScriptMove : IEquatable<ScriptMove>
    {
        public readonly ushort Index;
        public readonly ushort Count;
        public readonly ScriptMoveKind Kind;
        public readonly byte ObjectId;
        public readonly float OffsetMm;
        public readonly float GapMm;
        public readonly float StartSeconds;
        public readonly float ContactSeconds;
        public readonly float ClosestSeconds;
        public readonly float EndSeconds;

        public ScriptMove(ushort index, ushort count, ScriptMoveKind kind, byte objectId, float offsetMm, float gapMm, float startSeconds,
            float contactSeconds, float closestSeconds, float endSeconds)
        {
            Index = index;
            Count = count;
            Kind = kind;
            ObjectId = objectId;
            OffsetMm = offsetMm;
            GapMm = gapMm;
            StartSeconds = startSeconds;
            ContactSeconds = contactSeconds;
            ClosestSeconds = closestSeconds;
            EndSeconds = endSeconds;
        }

        public bool Equals(ScriptMove other) =>
            Index == other.Index && Count == other.Count && Kind == other.Kind && ObjectId == other.ObjectId && OffsetMm.Equals(other.OffsetMm) &&
            GapMm.Equals(other.GapMm) && StartSeconds.Equals(other.StartSeconds) && ContactSeconds.Equals(other.ContactSeconds) &&
            ClosestSeconds.Equals(other.ClosestSeconds) && EndSeconds.Equals(other.EndSeconds);

        public override bool Equals(object? obj) => obj is ScriptMove other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Index, Count, (byte)Kind, ObjectId, OffsetMm, GapMm, StartSeconds, HashCode.Combine(ContactSeconds, ClosestSeconds, EndSeconds));
    }
}
