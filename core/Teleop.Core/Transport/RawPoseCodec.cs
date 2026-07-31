using System;
using System.Buffers.Binary;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// The uncompressed baseline <see cref="ICommandCodec"/>: every field of a
    /// <see cref="CommandFrame"/> written verbatim, little-endian, into a fixed
    /// <see cref="EncodedSize"/>-byte record. No delta, no quantization, no redundancy, no
    /// prediction hint — the control condition every other codec is measured against.
    ///
    /// Its research value is precisely that it throws nothing away. A `raw` run isolates the
    /// bandwidth cost of an *unencoded* command stream and gives every other codec a bit-exact
    /// reference to be compared to: any pose error observed downstream of this codec came from
    /// the link or the pipeline, never from the wire format, because encode-then-decode here is
    /// the identity function on a <see cref="CommandFrame"/> (IEEE-754 bits are copied, not
    /// reformatted, so floats round-trip exactly).
    ///
    /// It is also the codec that leaves the receiver the least to work with: an instantaneous
    /// pose plus the commanded velocity fields and nothing more, so a lost packet is simply a
    /// missing sample. That is the baseline the `trajectory` and `redundant` codecs exist to
    /// beat.
    ///
    /// <b>Stateless, deliberately.</b> Unlike the delta and redundancy codecs this carries no
    /// baseline across calls, which is why <see cref="MaxEncodedBytes"/> is a true fixed size
    /// rather than an upper bound and why encode order does not affect the bytes produced.
    ///
    /// <b>No config struct, deliberately.</b> There is no knob here anyone would sweep — a
    /// quantized variant is a different codec (`delta-quant`), not a parameterization of this
    /// one. A config type would imply a tunable that does not exist.
    ///
    /// Not thread-safe, by contract. Allocation-free after construction.
    /// </summary>
    public sealed class RawPoseCodec : ICommandCodec
    {
        /// <summary>
        /// Wire-format version, byte 0 of every record. Bumped only by a breaking layout change;
        /// <see cref="TryDecode"/> rejects any other value rather than misinterpreting the bytes
        /// that follow it.
        /// </summary>
        public const byte Version = 1;

        private const int VersionSize = sizeof(byte);
        private const int UInt32Size = sizeof(uint);
        private const int Int64Size = sizeof(long);
        private const int FloatSize = sizeof(float);

        /// <summary>
        /// Position (3) + rotation (4) + linear velocity (3) + angular velocity (3) + gripper (1).
        /// </summary>
        private const int FloatFieldCount = 14;

        /// <summary>
        /// Exact size of one encoded record, in bytes: version, <c>Sequence</c>,
        /// <c>AckSequence</c>, <c>CaptureTicks</c>, then <see cref="FloatFieldCount"/> floats.
        /// Every record is this length — there is no variable-length case.
        /// </summary>
        public const int EncodedSize =
            VersionSize + (2 * UInt32Size) + Int64Size + (FloatFieldCount * FloatSize);

        /// <inheritdoc/>
        /// <remarks>
        /// Exact, not an upper bound: this codec has no compression, delta, or redundancy, so
        /// every frame encodes to precisely <see cref="EncodedSize"/> bytes.
        /// </remarks>
        public int MaxEncodedBytes => EncodedSize;

        /// <summary>
        /// Writes <paramref name="frame"/> as a fixed <see cref="EncodedSize"/>-byte
        /// little-endian record.
        ///
        /// A destination shorter than <see cref="EncodedSize"/> returns false with
        /// <paramref name="bytesWritten"/> set to <see cref="EncodedSize"/> and <b>no</b>
        /// destination byte touched — the contract forbids a partial record, since a reader
        /// cannot tell a truncated write from a corrupt one. Allocation-free and deterministic.
        /// </summary>
        public bool TryEncode(in CommandFrame frame, Span<byte> destination, out int bytesWritten)
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
            WriteUInt32(destination, ref pos, frame.AckSequence);
            WriteInt64(destination, ref pos, frame.CaptureTicks);

            WriteVector3(destination, ref pos, frame.Pose.Position);
            WriteQuaternion(destination, ref pos, frame.Pose.Rotation);
            WriteVector3(destination, ref pos, frame.LinearVelocity);
            WriteVector3(destination, ref pos, frame.AngularVelocity);
            WriteSingle(destination, ref pos, frame.Gripper);

            bytesWritten = pos;
            return true;
        }

        /// <summary>
        /// Reads one record written by <see cref="TryEncode"/>. Returns false, never throws, when
        /// <paramref name="source"/> is shorter than <see cref="EncodedSize"/> or carries a
        /// version byte this build does not understand; <paramref name="frame"/> is
        /// <c>default</c> in both cases.
        ///
        /// Bytes beyond <see cref="EncodedSize"/> are ignored rather than rejected, so a caller
        /// may hand over a whole receive buffer. There is no payload-level checksum: framing and
        /// integrity belong to the transport, and a per-record checksum here would be measured as
        /// codec overhead in every benchmark that uses this as the baseline.
        /// Allocation-free and deterministic.
        /// </summary>
        public bool TryDecode(ReadOnlySpan<byte> source, out CommandFrame frame)
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
            uint ackSequence = ReadUInt32(source, ref pos);
            long captureTicks = ReadInt64(source, ref pos);

            Vector3 position = ReadVector3(source, ref pos);
            Quaternion rotation = ReadQuaternion(source, ref pos);
            Vector3 linearVelocity = ReadVector3(source, ref pos);
            Vector3 angularVelocity = ReadVector3(source, ref pos);
            float gripper = ReadSingle(source, ref pos);

            frame = new CommandFrame(
                sequence,
                ackSequence,
                captureTicks,
                new Pose(position, rotation),
                linearVelocity,
                angularVelocity,
                gripper);
            return true;
        }

        /// <summary>
        /// Intentionally a no-op. <see cref="ICommandCodec.Reset"/> exists to drop "no delta
        /// baseline, no redundancy history, no quantization residual" — this codec holds none of
        /// those, so its as-constructed state and its state after any number of calls are the
        /// same state, and there is nothing to clear. (Compare
        /// <see cref="LoopbackTransport.Reset"/>, which notes it owns no RNG and so has nothing
        /// to reseed.) Kept rather than omitted because a sweep resets every component between
        /// trials without knowing which are stateful, and because the day this file grows a
        /// baseline is the day the omission would silently desynchronize the stream.
        /// </summary>
        public void Reset()
        {
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
        /// netstandard2.1's <see cref="BinaryPrimitives"/> has no float overload — the
        /// <c>Single</c>/<c>Double</c> members arrived in a later BCL wave than the integer ones
        /// and are not in this TFM's surface. Reinterpreting to the raw IEEE-754 bits and writing
        /// those as an <c>Int32</c> is exact (a bit copy, not a conversion: NaN payloads and
        /// signed zero survive) and keeps the endianness explicit, which
        /// <see cref="BitConverter.GetBytes(float)"/> would not.
        /// </summary>
        private static void WriteSingle(Span<byte> destination, ref int pos, float value)
        {
            BinaryPrimitives.WriteInt32LittleEndian(
                destination.Slice(pos, FloatSize), BitConverter.SingleToInt32Bits(value));
            pos += FloatSize;
        }

        private static void WriteVector3(Span<byte> destination, ref int pos, Vector3 value)
        {
            WriteSingle(destination, ref pos, value.X);
            WriteSingle(destination, ref pos, value.Y);
            WriteSingle(destination, ref pos, value.Z);
        }

        private static void WriteQuaternion(Span<byte> destination, ref int pos, Quaternion value)
        {
            WriteSingle(destination, ref pos, value.X);
            WriteSingle(destination, ref pos, value.Y);
            WriteSingle(destination, ref pos, value.Z);
            WriteSingle(destination, ref pos, value.W);
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

        /// <summary>Mirror of <see cref="WriteSingle"/>; see its note on the missing TFM overload.</summary>
        private static float ReadSingle(ReadOnlySpan<byte> source, ref int pos)
        {
            float value = BitConverter.Int32BitsToSingle(
                BinaryPrimitives.ReadInt32LittleEndian(source.Slice(pos, FloatSize)));
            pos += FloatSize;
            return value;
        }

        private static Vector3 ReadVector3(ReadOnlySpan<byte> source, ref int pos)
        {
            float x = ReadSingle(source, ref pos);
            float y = ReadSingle(source, ref pos);
            float z = ReadSingle(source, ref pos);
            return new Vector3(x, y, z);
        }

        private static Quaternion ReadQuaternion(ReadOnlySpan<byte> source, ref int pos)
        {
            float x = ReadSingle(source, ref pos);
            float y = ReadSingle(source, ref pos);
            float z = ReadSingle(source, ref pos);
            float w = ReadSingle(source, ref pos);
            return new Quaternion(x, y, z, w);
        }
    }
}
