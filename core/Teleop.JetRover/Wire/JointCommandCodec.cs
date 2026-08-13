using System;
using System.Buffers.Binary;

namespace Teleop.JetRover.Wire
{
    /// <summary>
    /// Fixed-size, versioned, little-endian encode/decode for <see cref="JointCommandFrame"/> --
    /// same style as <c>Teleop.Core.Transport.RawPoseCodec</c>, but not an <c>ICommandCodec</c>:
    /// that interface's <c>TryDecode</c> is pinned to producing a <c>CommandFrame</c> (a Cartesian
    /// pose), and this frame's whole reason for existing is to avoid ever reconstructing one on
    /// the robot side (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). A different
    /// version byte than <c>RawPoseCodec.Version</c> so the two are never confused if ever sent to
    /// the same socket by mistake, even though in practice each has its own dedicated port.
    ///
    /// Stateless and allocation-free, same reasoning as <c>RawPoseCodec</c>: no delta, no
    /// quantization, nothing to reset between calls.
    /// </summary>
    public static class JointCommandCodec
    {
        /// <summary>Wire-format version, byte 0 of every record. <see cref="TryDecode"/> rejects any other value.</summary>
        public const byte Version = 1;

        private const int VersionSize = sizeof(byte);
        private const int UInt32Size = sizeof(uint);
        private const int Int64Size = sizeof(long);
        private const int FloatSize = sizeof(float);

        /// <summary>BaseYaw, LowerPitch, MiddlePitch, UpperPitch, Gripper.</summary>
        private const int FloatFieldCount = 5;

        /// <summary>
        /// Exact size of one encoded record, in bytes: version, <c>Sequence</c>, <c>CaptureTicks</c>,
        /// then <see cref="FloatFieldCount"/> floats. Every record is this length.
        /// </summary>
        public const int EncodedSize = VersionSize + UInt32Size + Int64Size + (FloatFieldCount * FloatSize);

        /// <summary>
        /// Writes <paramref name="frame"/> as a fixed <see cref="EncodedSize"/>-byte little-endian
        /// record. A destination shorter than <see cref="EncodedSize"/> returns false with no
        /// destination byte touched -- same "never a partial record" contract as
        /// <c>RawPoseCodec.TryEncode</c>.
        /// </summary>
        public static bool TryEncode(in JointCommandFrame frame, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < EncodedSize)
            {
                bytesWritten = EncodedSize;
                return false;
            }

            int pos = 0;
            destination[pos] = Version;
            pos += VersionSize;

            WriteUInt32(destination, ref pos, frame.Sequence);
            WriteInt64(destination, ref pos, frame.CaptureTicks);
            WriteSingle(destination, ref pos, frame.BaseYaw);
            WriteSingle(destination, ref pos, frame.LowerPitch);
            WriteSingle(destination, ref pos, frame.MiddlePitch);
            WriteSingle(destination, ref pos, frame.UpperPitch);
            WriteSingle(destination, ref pos, frame.Gripper);

            bytesWritten = pos;
            return true;
        }

        /// <summary>
        /// Reads one record written by <see cref="TryEncode"/>. Returns false, never throws, when
        /// <paramref name="source"/> is shorter than <see cref="EncodedSize"/> or carries a
        /// version byte this build does not understand; <paramref name="frame"/> is <c>default</c>
        /// in both cases.
        /// </summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out JointCommandFrame frame)
        {
            if (source.Length < EncodedSize)
            {
                frame = default;
                return false;
            }

            int pos = 0;
            if (source[pos] != Version)
            {
                frame = default;
                return false;
            }
            pos += VersionSize;

            uint sequence = ReadUInt32(source, ref pos);
            long captureTicks = ReadInt64(source, ref pos);
            float baseYaw = ReadSingle(source, ref pos);
            float lowerPitch = ReadSingle(source, ref pos);
            float middlePitch = ReadSingle(source, ref pos);
            float upperPitch = ReadSingle(source, ref pos);
            float gripper = ReadSingle(source, ref pos);

            frame = new JointCommandFrame(sequence, captureTicks, baseYaw, lowerPitch, middlePitch, upperPitch, gripper);
            return true;
        }

        private static void WriteUInt32(Span<byte> destination, ref int pos, uint value)
        {
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(pos, UInt32Size), value);
            pos += UInt32Size;
        }

        private static void WriteInt64(Span<byte> destination, ref int pos, long value)
        {
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos, Int64Size), value);
            pos += Int64Size;
        }

        /// <summary>
        /// netstandard2.1's <see cref="BinaryPrimitives"/> has no float overload -- see
        /// <c>RawPoseCodec.WriteSingle</c>'s identical note. Reinterpreting to raw IEEE-754 bits
        /// is exact and keeps endianness explicit.
        /// </summary>
        private static void WriteSingle(Span<byte> destination, ref int pos, float value)
        {
            BinaryPrimitives.WriteInt32LittleEndian(
                destination.Slice(pos, FloatSize), BitConverter.SingleToInt32Bits(value));
            pos += FloatSize;
        }

        private static uint ReadUInt32(ReadOnlySpan<byte> source, ref int pos)
        {
            uint value = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(pos, UInt32Size));
            pos += UInt32Size;
            return value;
        }

        private static long ReadInt64(ReadOnlySpan<byte> source, ref int pos)
        {
            long value = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos, Int64Size));
            pos += Int64Size;
            return value;
        }

        private static float ReadSingle(ReadOnlySpan<byte> source, ref int pos)
        {
            float value = BitConverter.Int32BitsToSingle(
                BinaryPrimitives.ReadInt32LittleEndian(source.Slice(pos, FloatSize)));
            pos += FloatSize;
            return value;
        }
    }
}
