using System;
using System.Buffers.Binary;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// Fixed-header, count-prefixed, versioned, little-endian encode/decode for an operator
    /// command carrying already-computed joint angles, as it crosses the wire to
    /// <c>Teleop.RobotHost</c>'s joint listener (docs/adr/0011-generic-robot-arm-profiles.md,
    /// generalizing docs/adr/0009's fixed 4-float <c>JointCommandFrame</c>). Deliberately not an
    /// <c>ICommandCodec</c>: that interface's <c>TryDecode</c> is pinned to producing a
    /// <c>CommandFrame</c> (a Cartesian pose), and this wire's whole reason for existing is to
    /// avoid ever reconstructing one on the robot side.
    ///
    /// Carries one <see cref="JointTarget"/> per joint the sending profile has -- any joint count
    /// up to <see cref="MaxJointsPerMessage"/>, not a fixed 4. <see cref="JointTarget.Angle"/> is
    /// radians on this hop (Core's convention); <see cref="JointTarget.Speed"/> is radians/second,
    /// currently always encoded as 0 by the one caller that exists (<c>JetRoverOperatorBridge</c>
    /// has no per-joint speed concept yet) and ignored on decode -- present in the wire shape for
    /// symmetry with the RobotHost-&gt;ROS hop, not yet load-bearing on this one.
    ///
    /// Uplink-only: there is no matching downlink frame. A caller sending these still gets robot
    /// state feedback through its own separate, unmodified Cartesian
    /// <c>OperatorEndpoint</c>/<c>CommandFrame</c> connection.
    ///
    /// Stateless and allocation-free: both directions work over caller-supplied
    /// <see cref="Span{T}"/> buffers (typically <c>stackalloc</c>), same discipline as
    /// <c>RawPoseCodec</c>.
    /// </summary>
    public static class JointCommandCodec
    {
        /// <summary>
        /// Wire-format version, byte 0 of every record. Bumped from 1 (docs/adr/0009's fixed
        /// 4-float shape) to 2 for this count-prefixed generalization -- a breaking layout change,
        /// so <see cref="TryDecode"/> must reject a mismatched version, not attempt to reinterpret
        /// a differently-shaped record.
        /// </summary>
        public const byte Version = 2;

        private const int VersionSize = sizeof(byte);
        private const int UInt32Size = sizeof(uint);
        private const int Int64Size = sizeof(long);
        private const int FloatSize = sizeof(float);
        private const int MotorIdSize = sizeof(byte);
        private const int CountSize = sizeof(byte);

        /// <summary>Version, Sequence, CaptureTicks, Count.</summary>
        private const int HeaderSize = VersionSize + UInt32Size + Int64Size + CountSize;

        /// <summary>MotorId, Angle, Speed.</summary>
        private const int PerJointSize = MotorIdSize + FloatSize + FloatSize;

        /// <summary>
        /// Upper bound on how many joints one record can carry, derived from this hop's own
        /// datagram size budget (<c>Teleop.RobotHost.Program</c>'s <c>MaxJointDatagramBytes</c> =
        /// 128, separate from the unrelated Cartesian path's own <c>MaxDatagramBytes</c> -- each
        /// <c>UdpTransport</c> owns its own buffer): <c>(128 - HeaderSize) / PerJointSize</c>.
        /// </summary>
        public const int MaxJointsPerMessage = (128 - HeaderSize) / PerJointSize;

        /// <summary>Exact size of an encoded record carrying exactly <paramref name="jointCount"/> joints.</summary>
        public static int EncodedSize(int jointCount) => HeaderSize + jointCount * PerJointSize;

        /// <summary>
        /// Writes <paramref name="targets"/> as a header + one <see cref="JointTarget"/> block per
        /// entry. Returns false, touching no destination bytes, if <paramref name="targets"/>
        /// exceeds <see cref="MaxJointsPerMessage"/> or <paramref name="destination"/> is shorter
        /// than <see cref="EncodedSize"/> for this many joints -- same "never a partial record"
        /// contract as the old fixed-shape codec.
        /// </summary>
        public static bool TryEncode(
            uint sequence, long captureTicks, ReadOnlySpan<JointTarget> targets,
            Span<byte> destination, out int bytesWritten)
        {
            if (targets.Length > MaxJointsPerMessage)
            {
                bytesWritten = 0;
                return false;
            }

            int size = EncodedSize(targets.Length);
            if (destination.Length < size)
            {
                bytesWritten = size;
                return false;
            }

            int pos = 0;
            destination[pos] = Version;
            pos += VersionSize;

            WriteUInt32(destination, ref pos, sequence);
            WriteInt64(destination, ref pos, captureTicks);
            destination[pos] = (byte)targets.Length;
            pos += CountSize;

            foreach (JointTarget target in targets)
            {
                destination[pos] = target.MotorId;
                pos += MotorIdSize;
                WriteSingle(destination, ref pos, target.Angle);
                WriteSingle(destination, ref pos, target.Speed);
            }

            bytesWritten = pos;
            return true;
        }

        /// <summary>
        /// Reads one record written by <see cref="TryEncode"/> into <paramref name="targetsBuffer"/>
        /// (caller-supplied, typically <c>stackalloc JointTarget[MaxJointsPerMessage]</c>). Returns
        /// false, never throws, when <paramref name="source"/> is too short for its own declared
        /// count, carries a version byte this build doesn't understand, or declares more joints
        /// than either <see cref="MaxJointsPerMessage"/> or <paramref name="targetsBuffer"/> can
        /// hold.
        /// </summary>
        public static bool TryDecode(
            ReadOnlySpan<byte> source, out uint sequence, out long captureTicks,
            Span<JointTarget> targetsBuffer, out int targetCount)
        {
            sequence = 0;
            captureTicks = 0;
            targetCount = 0;

            if (source.Length < HeaderSize || source[0] != Version)
            {
                return false;
            }

            int pos = VersionSize;
            sequence = ReadUInt32(source, ref pos);
            captureTicks = ReadInt64(source, ref pos);
            int count = source[pos];
            pos += CountSize;

            if (count > MaxJointsPerMessage || count > targetsBuffer.Length || source.Length < EncodedSize(count))
            {
                sequence = 0;
                captureTicks = 0;
                return false;
            }

            for (int i = 0; i < count; i++)
            {
                byte motorId = source[pos];
                pos += MotorIdSize;
                float angle = ReadSingle(source, ref pos);
                float speed = ReadSingle(source, ref pos);
                targetsBuffer[i] = new JointTarget(motorId, angle, speed);
            }

            targetCount = count;
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
        /// <c>RawPoseCodec.WriteSingle</c>'s identical note. Reinterpreting to raw IEEE-754 bits is
        /// exact and keeps endianness explicit.
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
