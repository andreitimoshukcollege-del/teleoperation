using System;
using System.Buffers.Binary;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// What the headset is doing to its own link and view (docs/adr/0023 amendment, 2026-10-27): the
    /// console's network disturbance (one-way lag, jitter, loss, both ways, on the joint link) and the
    /// added delay on the robot's state reports. A recording made with any of them on is not a
    /// real-network measurement, and only the headset knows they are on. Also the filter it puts on the
    /// hands before they drive the arms (docs/adr/0029 amendment 7): that shapes every command, and only
    /// the headset knows it either.
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

        /// <summary>
        /// The hand filter (docs/adr/0029 amendment 7): its cutoff for a still hand (Hz), how fast the cutoff rises with
        /// the hand's speed and turning (Hz per m/s, Hz per rad/s), and its speed estimate's cutoff (Hz). A min cutoff of
        /// 0 is the hands going raw; NaN is a headset that does not say (the JetRover's, or one from before the report).
        /// </summary>
        public readonly float HandMinCutoffHz;
        public readonly float HandPositionBeta;
        public readonly float HandRotationBeta;
        public readonly float HandDerivativeCutoffHz;

        /// <summary>Conditions that say nothing about a hand filter.</summary>
        public HeadsetConditions(float lagMs, float jitterMs, float lossPercent, float addedDelayMs, ulong seed)
            : this(lagMs, jitterMs, lossPercent, addedDelayMs, seed, float.NaN, float.NaN, float.NaN, float.NaN)
        {
        }

        public HeadsetConditions(float lagMs, float jitterMs, float lossPercent, float addedDelayMs, ulong seed,
            float handMinCutoffHz, float handPositionBeta, float handRotationBeta, float handDerivativeCutoffHz)
        {
            LagMs = lagMs;
            JitterMs = jitterMs;
            LossPercent = lossPercent;
            AddedDelayMs = addedDelayMs;
            Seed = seed;
            HandMinCutoffHz = handMinCutoffHz;
            HandPositionBeta = handPositionBeta;
            HandRotationBeta = handRotationBeta;
            HandDerivativeCutoffHz = handDerivativeCutoffHz;
        }

        /// <summary>Whether the link itself is disturbed (any lag, jitter or loss, or a named profile).</summary>
        public bool LinkDisturbed => float.IsNaN(LagMs) || LagMs > 0f || JitterMs > 0f || LossPercent > 0f;

        /// <summary>Whether anything is on: the link disturbed, or the view delayed.</summary>
        public bool Disturbed => LinkDisturbed || AddedDelayMs > 0f;

        /// <summary>Whether the headset says what it does to the hands (raw included).</summary>
        public bool ReportsHandFilter => !float.IsNaN(HandMinCutoffHz);

        public bool Equals(HeadsetConditions other) =>
            Same(LagMs, other.LagMs) && Same(JitterMs, other.JitterMs) && Same(LossPercent, other.LossPercent) &&
            Same(AddedDelayMs, other.AddedDelayMs) && Seed == other.Seed &&
            Same(HandMinCutoffHz, other.HandMinCutoffHz) && Same(HandPositionBeta, other.HandPositionBeta) &&
            Same(HandRotationBeta, other.HandRotationBeta) && Same(HandDerivativeCutoffHz, other.HandDerivativeCutoffHz);

        public override bool Equals(object? obj) => obj is HeadsetConditions other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(HashCode.Combine(LagMs, JitterMs, LossPercent, AddedDelayMs, Seed),
            HandMinCutoffHz, HandPositionBeta, HandRotationBeta, HandDerivativeCutoffHz);

        private static bool Same(float a, float b) => a.Equals(b);
    }

    /// <summary>
    /// The headset conditions, operator → robot on the joint port, little-endian, 56 bytes:
    /// <c>u8 0x84 | u8 version (1) | u16 reserved (0) | f32 lagMs | f32 jitterMs | f32 lossPercent |
    /// f32 addedDelayMs | u64 seed | u32 sequence | i64 operatorSendTicks | f32 handMinCutoffHz |
    /// f32 handPositionBeta | f32 handRotationBeta | f32 handDerivativeCutoffHz</c>. State, not an event: the
    /// headset sends it once a second and at every change. Every decoder on the joint port checks its
    /// first byte exactly, so an older RobotHost ignores it.
    /// <para>The hand block was appended on 2026-11-01 (docs/adr/0029 amendment 7) at the same version: a
    /// version-1 decoder needs only the first <see cref="BaseSize"/> bytes and ignores what follows, so every
    /// RobotHost already deployed keeps recording the disturbance. A record of only those bytes, from a headset
    /// built before the block, decodes with the hand fields NaN.</para>
    /// </summary>
    public static class HeadsetConditionsCodec
    {
        public const byte Tag = 0x84;
        public const byte Version = 1;
        /// <summary>The record before the hand block: all a version-1 decoder reads.</summary>
        public const int BaseSize = 4 + 16 + 8 + 4 + 8;

        public const int Size = BaseSize + 16;

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
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(40), BitConverter.SingleToInt32Bits(c.HandMinCutoffHz));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(44), BitConverter.SingleToInt32Bits(c.HandPositionBeta));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(48), BitConverter.SingleToInt32Bits(c.HandRotationBeta));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(52), BitConverter.SingleToInt32Bits(c.HandDerivativeCutoffHz));
            return true;
        }

        /// <summary>False for a record shorter than <see cref="BaseSize"/>, another tag or version. Without the hand block, the hand fields are NaN.</summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out HeadsetConditions conditions, out uint sequence, out long operatorSendTicks)
        {
            conditions = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < BaseSize || source[0] != Tag || source[1] != Version)
            {
                return false;
            }

            bool hands = source.Length >= Size;
            conditions = new HeadsetConditions(
                Single(source, 4), Single(source, 8), Single(source, 12), Single(source, 16),
                BinaryPrimitives.ReadUInt64LittleEndian(source.Slice(20)),
                hands ? Single(source, 40) : float.NaN, hands ? Single(source, 44) : float.NaN,
                hands ? Single(source, 48) : float.NaN, hands ? Single(source, 52) : float.NaN);
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(28));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(32));
            return true;
        }

        private static float Single(ReadOnlySpan<byte> source, int offset) =>
            BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(offset)));
    }
}
