using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Pipeline
{
    /// <summary>
    /// Turns a <see cref="RobotStateFrame"/> into bytes and back. A plain <c>sealed class</c>,
    /// not behind a <c>Contracts/</c> interface — exactly one downlink wire shape exists in
    /// this project, not a family of competing implementations to select between, the same
    /// reasoning <c>Time/ClockSync.cs</c> gives for not being behind an interface either.
    ///
    /// Fixed 57-byte little-endian layout, mirroring <c>Transport/RawPoseCodec.cs</c>'s
    /// <see cref="ICommandCodec"/>-shaped contract by convention (false + required-length on a
    /// too-small buffer, no partial state on failure, reject rather than throw on a bad version
    /// byte) even though this type sits outside that interface: byte 0 version, bytes 1-4
    /// <c>Sequence</c> (uint32), bytes 5-12 <c>RobotRecvTicks</c> (int64), bytes 13-20
    /// <c>DownlinkSendTicks</c> (int64), bytes 21-48 <c>Pose</c> (3 position floats + 4 rotation
    /// floats), bytes 49-56 <c>TicksPerSecond</c> (int64). Binary, not text like
    /// <c>Recording/RecordFormat.cs</c>'s <c>.tlog</c> format: this crosses a bounded datagram on
    /// the per-frame hot path and is never committed or diffed, so the text-format rationale
    /// doesn't apply here.
    ///
    /// <b>Version 2</b> (docs/adr/0008-clocksync-cross-rate-normalization.md) appended
    /// <c>TicksPerSecond</c> after the pose rather than grouping it with the other two tick
    /// fields, keeping v1's byte offsets untouched so the diff is confined to the new trailing
    /// field. A v1 payload is rejected by the existing version-byte check, not misread: this is a
    /// deliberate breaking change, safe only because both ends of this hop
    /// (<c>Teleop.Eval</c>/Unity and <c>Teleop.RobotHost</c>) are built from this one Core source
    /// and redeployed together — there is no independently maintained decoder of this frame, in
    /// contrast to the JetRover relay protocol, which has a hand-mirrored Python twin.
    ///
    /// Floats are written as their raw bit pattern (<see cref="BitConverter.SingleToInt32Bits"/>)
    /// through <see cref="BinaryPrimitives.WriteInt32LittleEndian"/>/
    /// <see cref="BinaryPrimitives.ReadInt32LittleEndian"/> rather than
    /// <c>BinaryPrimitives.WriteSingleLittleEndian</c>, because that overload does not exist in
    /// this project's <c>netstandard2.1</c> target (confirmed by a direct build check) — the
    /// bit-pattern round trip is exact regardless.
    /// </summary>
    public sealed class RobotStateFrameCodec
    {
        public const byte Version = 2;
        public const int EncodedSize = 57;

        public int MaxEncodedBytes => EncodedSize;

        /// <summary>
        /// Encode one frame into <paramref name="destination"/>. Returns false when the buffer
        /// is too small, setting <paramref name="bytesWritten"/> to the length required and
        /// touching no destination bytes. Deterministic. Allocation-free.
        /// </summary>
        public bool TryEncode(in RobotStateFrame frame, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < EncodedSize)
            {
                bytesWritten = EncodedSize;
                return false;
            }

            destination[0] = Version;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(1, 4), frame.Sequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(5, 8), frame.RobotRecvTicks);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(13, 8), frame.DownlinkSendTicks);

            WriteSingle(destination.Slice(21, 4), frame.Pose.Position.X);
            WriteSingle(destination.Slice(25, 4), frame.Pose.Position.Y);
            WriteSingle(destination.Slice(29, 4), frame.Pose.Position.Z);
            WriteSingle(destination.Slice(33, 4), frame.Pose.Rotation.X);
            WriteSingle(destination.Slice(37, 4), frame.Pose.Rotation.Y);
            WriteSingle(destination.Slice(41, 4), frame.Pose.Rotation.Z);
            WriteSingle(destination.Slice(45, 4), frame.Pose.Rotation.W);

            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(49, 8), frame.TicksPerSecond);

            bytesWritten = EncodedSize;
            return true;
        }

        /// <summary>
        /// Decode one frame from <paramref name="source"/>. Returns false on a truncated,
        /// corrupt, or unsupported-version payload; never throws. <paramref name="frame"/> is
        /// <c>default</c> when this returns false. Allocation-free.
        /// </summary>
        public bool TryDecode(ReadOnlySpan<byte> source, out RobotStateFrame frame)
        {
            if (source.Length < EncodedSize || source[0] != Version)
            {
                frame = default;
                return false;
            }

            uint sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(1, 4));
            long robotRecvTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(5, 8));
            long downlinkSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(13, 8));

            float posX = ReadSingle(source.Slice(21, 4));
            float posY = ReadSingle(source.Slice(25, 4));
            float posZ = ReadSingle(source.Slice(29, 4));
            float rotX = ReadSingle(source.Slice(33, 4));
            float rotY = ReadSingle(source.Slice(37, 4));
            float rotZ = ReadSingle(source.Slice(41, 4));
            float rotW = ReadSingle(source.Slice(45, 4));

            long ticksPerSecond = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(49, 8));

            var pose = new Types.Pose(
                new System.Numerics.Vector3(posX, posY, posZ),
                new System.Numerics.Quaternion(rotX, rotY, rotZ, rotW));

            frame = new RobotStateFrame(sequence, robotRecvTicks, downlinkSendTicks, ticksPerSecond, pose);
            return true;
        }

        /// <summary>
        /// Intentional no-op: this codec carries no delta baseline, no redundancy history, no
        /// quantization residual, so there is nothing to reset.
        /// </summary>
        public void Reset()
        {
        }

        private static void WriteSingle(Span<byte> destination, float value) =>
            BinaryPrimitives.WriteInt32LittleEndian(destination, BitConverter.SingleToInt32Bits(value));

        private static float ReadSingle(ReadOnlySpan<byte> source) =>
            BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source));
    }
}
