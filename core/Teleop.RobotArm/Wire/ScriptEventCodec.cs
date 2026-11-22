using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Wire
{
    /// <summary>What a scripted safety event does (docs/adr/0036 amendment 5).</summary>
    public enum ScriptEventKind : byte
    {
        /// <summary>stop_before_collision: an approach that stops 1-3 cm short of the object.</summary>
        Stop = 1,

        /// <summary>collide_and_recover: a touch, then backing off.</summary>
        Touch = 2,

        /// <summary>collide_with_obstacle: a swipe through the object that knocks it over.</summary>
        Knock = 3,
    }

    /// <summary>
    /// One event of a scripted take, tool → robot on the joint port (docs/adr/0036 amendment 5), little-endian, 36 bytes:
    /// <c>u8 0x8C | u8 version (1) | u8 index | u8 drawn | u8 kind | u8 objectId | u16 0 | u32 sequence | f32 finalSeconds |
    /// f32 contactSeconds | f32 endSeconds | f32 clearSeconds | i64 operatorSendTicks</c>. <c>joint-drive</c> sends it as the arm
    /// leaves free movement for the event's object; the four times are seconds after that, as planned: the final approach
    /// begins, the first contact (a stop: its closest approach; NaN when none was planned), the event ends, free movement takes
    /// over again. <c>index</c> counts from 1 in the take, <c>drawn</c> is how many the take drew. RobotHost records it in the
    /// session as <c>SE</c>, once per sequence (the tool sends each a few times). A label for the recording only; it moves
    /// nothing. Every decoder on the joint port checks its first byte exactly, so an older RobotHost ignores it.
    /// </summary>
    public static class ScriptEventCodec
    {
        public const byte Tag = 0x8C;
        public const byte Version = 1;
        public const int Size = 8 + 4 + 4 * 4 + 8;

        public static bool IsScriptEvent(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == Tag;

        public static bool TryEncode(in ScriptEvent e, uint sequence, long operatorSendTicks, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = Size;
            if (destination.Length < Size)
            {
                return false;
            }

            destination[0] = Tag;
            destination[1] = Version;
            destination[2] = e.Index;
            destination[3] = e.Drawn;
            destination[4] = (byte)e.Kind;
            destination[5] = e.ObjectId;
            destination[6] = 0;
            destination[7] = 0;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(8), sequence);
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(12), BitConverter.SingleToInt32Bits(e.FinalSeconds));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(16), BitConverter.SingleToInt32Bits(e.ContactSeconds));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(20), BitConverter.SingleToInt32Bits(e.EndSeconds));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(24), BitConverter.SingleToInt32Bits(e.ClearSeconds));
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(28), operatorSendTicks);
            return true;
        }

        /// <summary>Reads an event. False for a short record, another tag or version, or a kind this build does not define.</summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out ScriptEvent e, out uint sequence, out long operatorSendTicks)
        {
            e = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < Size || source[0] != Tag || source[1] != Version || source[4] < (byte)ScriptEventKind.Stop || source[4] > (byte)ScriptEventKind.Knock)
            {
                return false;
            }

            e = new ScriptEvent(source[2], source[3], (ScriptEventKind)source[4], source[5],
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(12))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(16))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(20))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(24))));
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(8));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(28));
            return true;
        }

        public static string Name(ScriptEventKind kind) => kind switch
        {
            ScriptEventKind.Stop => "stop",
            ScriptEventKind.Touch => "touch",
            ScriptEventKind.Knock => "knock",
            _ => "unknown",
        };
    }

    /// <summary>One event of a scripted take (<see cref="ScriptEventCodec"/>): which, of how many, what, at which object, and when, seconds after the approach began.</summary>
    public readonly struct ScriptEvent : IEquatable<ScriptEvent>
    {
        public readonly byte Index;
        public readonly byte Drawn;
        public readonly ScriptEventKind Kind;
        public readonly byte ObjectId;
        public readonly float FinalSeconds;
        public readonly float ContactSeconds;
        public readonly float EndSeconds;
        public readonly float ClearSeconds;

        public ScriptEvent(byte index, byte drawn, ScriptEventKind kind, byte objectId, float finalSeconds, float contactSeconds, float endSeconds, float clearSeconds)
        {
            Index = index;
            Drawn = drawn;
            Kind = kind;
            ObjectId = objectId;
            FinalSeconds = finalSeconds;
            ContactSeconds = contactSeconds;
            EndSeconds = endSeconds;
            ClearSeconds = clearSeconds;
        }

        public bool Equals(ScriptEvent other) =>
            Index == other.Index && Drawn == other.Drawn && Kind == other.Kind && ObjectId == other.ObjectId &&
            FinalSeconds.Equals(other.FinalSeconds) && ContactSeconds.Equals(other.ContactSeconds) && EndSeconds.Equals(other.EndSeconds) &&
            ClearSeconds.Equals(other.ClearSeconds);

        public override bool Equals(object? obj) => obj is ScriptEvent other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Index, Drawn, (byte)Kind, ObjectId, FinalSeconds, ContactSeconds, EndSeconds, ClearSeconds);
    }
}
