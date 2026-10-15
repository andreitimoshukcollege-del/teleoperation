using System;
using System.Buffers.Binary;
using Teleop.Core.Types;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// The joint-state downlink and its keepalive (docs/adr/0018), both on the joint port.
    ///
    /// <para><b>Frame</b> (robot → operator, version 1, little-endian):
    /// <c>u8 version | u32 sequence | i64 robotSendTicks | i64 sensedTicks | i64 echoOperatorTicks |
    /// i64 echoRobotRecvTicks | i64 ticksPerSecond | u8 count | count × { u8 motorId, u8 flags (bit0
    /// sensed), f32 value }</c>. Values use <see cref="JointTarget.Angle"/>'s units: radians for arm
    /// joints and the wrist roll, a 0..1 fraction for a gripper.</para>
    ///
    /// <para><b>Subscribe</b> (operator → robot, 17 bytes): <c>u8 0x80 | i64 operatorSendTicks |
    /// i64 operatorTicksPerSecond</c>. Its first byte matches no <see cref="JointCommandCodec"/>
    /// version, so both share one socket. The robot streams to the latest subscriber and echoes
    /// its ticks, which is what lets the operator run <c>ClockSync</c> without sending commands.</para>
    /// </summary>
    public static class JointStateCodec
    {
        public const byte Version = 1;
        public const byte SubscribeTag = 0x80;
        public const int SubscribeSize = 1 + 8 + 8;

        private const int HeaderSize = 1 + 4 + 8 * 5 + 1;
        private const int PerJointSize = 1 + 1 + 4;
        private const byte SensedFlag = 0x01;

        public const int MaxJointsPerMessage = (128 - HeaderSize) / PerJointSize;

        public static int EncodedSize(int jointCount) => HeaderSize + jointCount * PerJointSize;

        public static bool IsSubscribe(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == SubscribeTag;

        public static bool TryEncodeSubscribe(long operatorSendTicks, long operatorTicksPerSecond, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = SubscribeSize;
            if (destination.Length < SubscribeSize)
            {
                return false;
            }

            destination[0] = SubscribeTag;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(1), operatorSendTicks);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(9), operatorTicksPerSecond);
            return true;
        }

        public static bool TryDecodeSubscribe(ReadOnlySpan<byte> source, out long operatorSendTicks, out long operatorTicksPerSecond)
        {
            operatorSendTicks = 0;
            operatorTicksPerSecond = 0;
            if (source.Length < SubscribeSize || source[0] != SubscribeTag)
            {
                return false;
            }

            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(1));
            operatorTicksPerSecond = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(9));
            return operatorTicksPerSecond > 0;
        }

        public static bool TryEncode(
            uint sequence, long robotSendTicks, long sensedTicks, long echoOperatorTicks, long echoRobotRecvTicks,
            long ticksPerSecond, ReadOnlySpan<JointSample> joints, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = 0;
            if (joints.Length > MaxJointsPerMessage)
            {
                return false;
            }

            int size = EncodedSize(joints.Length);
            if (destination.Length < size)
            {
                bytesWritten = size;
                return false;
            }

            int pos = 0;
            destination[pos++] = Version;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(pos), sequence); pos += 4;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos), robotSendTicks); pos += 8;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos), sensedTicks); pos += 8;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos), echoOperatorTicks); pos += 8;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos), echoRobotRecvTicks); pos += 8;
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(pos), ticksPerSecond); pos += 8;
            destination[pos++] = (byte)joints.Length;
            foreach (JointSample joint in joints)
            {
                destination[pos++] = joint.MotorId;
                destination[pos++] = joint.Sensed ? SensedFlag : (byte)0;
                BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(pos), BitConverter.SingleToInt32Bits(joint.Value)); pos += 4;
            }

            bytesWritten = pos;
            return true;
        }

        public static bool TryDecode(
            ReadOnlySpan<byte> source, out uint sequence, out long robotSendTicks, out long sensedTicks,
            out long echoOperatorTicks, out long echoRobotRecvTicks, out long ticksPerSecond,
            Span<JointSample> jointsBuffer, out int jointCount)
        {
            sequence = 0;
            robotSendTicks = sensedTicks = echoOperatorTicks = echoRobotRecvTicks = ticksPerSecond = 0;
            jointCount = 0;
            if (source.Length < HeaderSize || source[0] != Version)
            {
                return false;
            }

            int pos = 1;
            uint seq = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(pos)); pos += 4;
            long send = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos)); pos += 8;
            long sensed = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos)); pos += 8;
            long echoOp = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos)); pos += 8;
            long echoRecv = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos)); pos += 8;
            long tps = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(pos)); pos += 8;
            int count = source[pos++];
            if (tps <= 0 || count > MaxJointsPerMessage || count > jointsBuffer.Length || source.Length < EncodedSize(count))
            {
                return false;
            }

            for (int i = 0; i < count; i++)
            {
                byte id = source[pos++];
                bool isSensed = (source[pos++] & SensedFlag) != 0;
                float value = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(pos))); pos += 4;
                jointsBuffer[i] = new JointSample(id, value, isSensed);
            }

            sequence = seq;
            robotSendTicks = send;
            sensedTicks = sensed;
            echoOperatorTicks = echoOp;
            echoRobotRecvTicks = echoRecv;
            ticksPerSecond = tps;
            jointCount = count;
            return true;
        }
    }
}
