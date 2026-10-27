using System;
using System.Buffers.Binary;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// What the headset is doing to its own link and view (docs/adr/0023 amendment, 2026-10-27): the
    /// console's network disturbance (one-way lag, jitter, loss, both ways, on the joint link) and the
    /// added delay on the robot's state reports. A recording made with any of them on is not a
    /// real-network measurement, and only the headset knows they are on.
    /// </summary>
    public readonly struct HeadsetConditions : IEquatable<HeadsetConditions>
    {
        /// <summary>One-way lag, ms; NaN when the disturbance is a named profile this message cannot spell out.</summary>
        public readonly float LagMs;
        public readonly float JitterMs;
        public readonly float LossPercent;

        /// <summary>The headset's added delay on the robot's state reports, ms (its view of the twin only).</summary>
        public readonly float AddedDelayMs;

        /// <summary>The disturbance's seed (its draws can be reproduced from it); 0 when off.</summary>
        public readonly ulong Seed;

        public HeadsetConditions(float lagMs, float jitterMs, float lossPercent, float addedDelayMs, ulong seed)
        {
            LagMs = lagMs;
            JitterMs = jitterMs;
            LossPercent = lossPercent;
            AddedDelayMs = addedDelayMs;
            Seed = seed;
        }

        /// <summary>Whether the link itself is disturbed (any lag, jitter or loss, or a named profile).</summary>
        public bool LinkDisturbed => float.IsNaN(LagMs) || LagMs > 0f || JitterMs > 0f || LossPercent > 0f;

        /// <summary>Whether anything is on: the link disturbed, or the view delayed.</summary>
        public bool Disturbed => LinkDisturbed || AddedDelayMs > 0f;

        public bool Equals(HeadsetConditions other) =>
            Same(LagMs, other.LagMs) && Same(JitterMs, other.JitterMs) && Same(LossPercent, other.LossPercent) &&
            Same(AddedDelayMs, other.AddedDelayMs) && Seed == other.Seed;

        public override bool Equals(object? obj) => obj is HeadsetConditions other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(LagMs, JitterMs, LossPercent, AddedDelayMs, Seed);

        private static bool Same(float a, float b) => a.Equals(b);
    }

    /// <summary>
    /// The headset conditions, operator → robot on the joint port, little-endian, 40 bytes:
    /// <c>u8 0x84 | u8 version (1) | u16 reserved (0) | f32 lagMs | f32 jitterMs | f32 lossPercent |
    /// f32 addedDelayMs | u64 seed | u32 sequence | i64 operatorSendTicks</c>. State, not an event: the
    /// headset sends it once a second and at every change. Every decoder on the joint port checks its
    /// first byte exactly, so an older RobotHost ignores it.
    /// </summary>
    public static class HeadsetConditionsCodec
    {
        public const byte Tag = 0x84;
        public const byte Version = 1;
        public const int Size = 4 + 16 + 8 + 4 + 8;

        public static bool IsConditions(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == Tag;

        public static bool TryEncode(in HeadsetConditions c, uint sequence, long operatorSendTicks, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = Size;
            if (destination.Length < Size)
            {
                return false;
            }

            destination[0] = Tag;
            destination[1] = Version;
            destination[2] = 0;
            destination[3] = 0;
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(4), BitConverter.SingleToInt32Bits(c.LagMs));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(8), BitConverter.SingleToInt32Bits(c.JitterMs));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(12), BitConverter.SingleToInt32Bits(c.LossPercent));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(16), BitConverter.SingleToInt32Bits(c.AddedDelayMs));
            BinaryPrimitives.WriteUInt64LittleEndian(destination.Slice(20), c.Seed);
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(28), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(32), operatorSendTicks);
            return true;
        }

        /// <summary>False for a short record, another tag or version.</summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out HeadsetConditions conditions, out uint sequence, out long operatorSendTicks)
        {
            conditions = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < Size || source[0] != Tag || source[1] != Version)
            {
                return false;
            }

            conditions = new HeadsetConditions(
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(4))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(8))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(12))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(16))),
                BinaryPrimitives.ReadUInt64LittleEndian(source.Slice(20)));
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(28));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(32));
            return true;
        }
    }
}
