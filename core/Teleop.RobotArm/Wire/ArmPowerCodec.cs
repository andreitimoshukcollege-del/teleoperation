using System;
using System.Buffers.Binary;

namespace Teleop.RobotArm.Wire
{
    /// <summary>What the operator asks of the arm's servo torque (docs/adr/0023 decision 5).</summary>
    public enum ArmPowerRequest : byte
    {
        /// <summary>Park slowly at the rest pose, wait until the readings say it is there, then switch torque off.</summary>
        Relax = 1,

        /// <summary>Switch torque on (or keep it on), and take joint commands again.</summary>
        Hold = 2,
    }

    /// <summary>The arm's power state as RobotHost reports it.</summary>
    public enum ArmPowerState : byte
    {
        /// <summary>Torque on; joint commands are forwarded.</summary>
        Holding = 0,

        /// <summary>Moving to the rest pose; joint commands are ignored.</summary>
        Parking = 1,

        /// <summary>At the rest pose with torque off; joint commands are ignored until a hold.</summary>
        Relaxed = 2,
    }

    /// <summary>
    /// Why a relax did not happen, or ended back in <see cref="ArmPowerState.Holding"/>. <see cref="None"/>
    /// with any state is a plain report. A status in <see cref="ArmPowerState.Holding"/> with any other
    /// reason is a refusal.
    /// </summary>
    public enum ArmPowerReason : byte
    {
        None = 0,

        /// <summary>RobotHost was started without <c>--rest-pose-deg</c>.</summary>
        NoRestPose = 1,

        /// <summary>A sample of the straight joint path from the measured pose to the rest pose collides (the model of docs/adr/0020).</summary>
        PathCollides = 2,

        /// <summary>The readings did not reach the rest pose within the park timeout; the arm holds where it is.</summary>
        ParkTimedOut = 3,

        /// <summary>No fresh servo reading for an arm joint: the board is not answering, so nothing is known (and torque could not be switched anyway).</summary>
        BoardNotAnswering = 4,

        /// <summary>The robot profile has no collision model, so no path can be checked.</summary>
        NoCollisionModel = 5,

        /// <summary>The request byte was not one this RobotHost knows.</summary>
        UnknownRequest = 6,

        /// <summary>A hold request arrived while parking: the park was abandoned and the arm holds where it is.</summary>
        HoldRequested = 7,
    }

    /// <summary>
    /// The arm-power messages on the joint port (docs/adr/0018 amendment 2026-10-26, docs/adr/0023
    /// decision 5), little-endian.
    ///
    /// <para><b>Request</b> (operator → robot, 15 bytes): <c>u8 0x81 | u8 version (1) | u8 request |
    /// u32 sequence | i64 operatorSendTicks</c>.</para>
    ///
    /// <para><b>Status</b> (robot → operator, 28 bytes): <c>u8 0x82 | u8 version (1) | u8 state |
    /// u8 reason | u32 echoSequence | i64 echoOperatorTicks | i64 robotSendTicks |
    /// f32 remainingDeg</c>. <c>echoSequence</c> and <c>echoOperatorTicks</c> are the latest request's
    /// (0 before any); <c>remainingDeg</c> is the largest arm joint's distance to the rest pose by the
    /// last reading, NaN when unknown.</para>
    ///
    /// <para>The tags share the joint port with <see cref="JointCommandCodec"/> (first byte 2),
    /// <see cref="JointStateCodec"/>'s frame (first byte 1) and its subscribe (0x80). Every decoder on
    /// the port checks its first byte exactly, so each ignores the others, and a RobotHost or headset
    /// older than this amendment ignores both of these.</para>
    /// </summary>
    public static class ArmPowerCodec
    {
        public const byte RequestTag = 0x81;
        public const byte StatusTag = 0x82;
        public const byte Version = 1;

        public const int RequestSize = 1 + 1 + 1 + 4 + 8;
        public const int StatusSize = 1 + 1 + 1 + 1 + 4 + 8 + 8 + 4;

        public static bool IsRequest(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == RequestTag;

        public static bool IsStatus(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == StatusTag;

        /// <summary>
        /// Writes a request. False, writing nothing, when <paramref name="destination"/> is too short.
        /// The request byte is written as given, so a test can send one this build does not define.
        /// </summary>
        public static bool TryEncodeRequest(ArmPowerRequest request, uint sequence, long operatorSendTicks,
            Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = RequestSize;
            if (destination.Length < RequestSize)
            {
                return false;
            }

            destination[0] = RequestTag;
            destination[1] = Version;
            destination[2] = (byte)request;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(3), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(7), operatorSendTicks);
            return true;
        }

        /// <summary>
        /// Reads a request. False for a short record, another tag or another version. The request byte is
        /// returned raw, so the receiver can answer an unknown one (<see cref="ArmPowerReason.UnknownRequest"/>);
        /// <see cref="IsKnown(ArmPowerRequest)"/> says whether it is one.
        /// </summary>
        public static bool TryDecodeRequest(ReadOnlySpan<byte> source, out ArmPowerRequest request, out uint sequence,
            out long operatorSendTicks)
        {
            request = 0;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < RequestSize || source[0] != RequestTag || source[1] != Version)
            {
                return false;
            }

            request = (ArmPowerRequest)source[2];
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(3));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(7));
            return true;
        }

        public static bool IsKnown(ArmPowerRequest request) => request == ArmPowerRequest.Relax || request == ArmPowerRequest.Hold;

        public static bool TryEncodeStatus(ArmPowerState state, ArmPowerReason reason, uint echoSequence, long echoOperatorTicks,
            long robotSendTicks, float remainingDeg, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = StatusSize;
            if (destination.Length < StatusSize)
            {
                return false;
            }

            destination[0] = StatusTag;
            destination[1] = Version;
            destination[2] = (byte)state;
            destination[3] = (byte)reason;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(4), echoSequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(8), echoOperatorTicks);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(16), robotSendTicks);
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(24), BitConverter.SingleToInt32Bits(remainingDeg));
            return true;
        }

        /// <summary>
        /// Reads a status. False for a short record, another tag or version, or a state byte this build
        /// does not define (a status it cannot act on is no status). An unknown reason byte is returned
        /// as is: the state still holds.
        /// </summary>
        public static bool TryDecodeStatus(ReadOnlySpan<byte> source, out ArmPowerState state, out ArmPowerReason reason,
            out uint echoSequence, out long echoOperatorTicks, out long robotSendTicks, out float remainingDeg)
        {
            state = ArmPowerState.Holding;
            reason = ArmPowerReason.None;
            echoSequence = 0;
            echoOperatorTicks = 0;
            robotSendTicks = 0;
            remainingDeg = float.NaN;
            if (source.Length < StatusSize || source[0] != StatusTag || source[1] != Version || source[2] > (byte)ArmPowerState.Relaxed)
            {
                return false;
            }

            state = (ArmPowerState)source[2];
            reason = (ArmPowerReason)source[3];
            echoSequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            echoOperatorTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(8));
            robotSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(16));
            remainingDeg = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(24)));
            return true;
        }

        /// <summary>A short readable name for a reason, for logs and the headset's panel.</summary>
        public static string Describe(ArmPowerReason reason)
        {
            switch (reason)
            {
                case ArmPowerReason.None: return "";
                case ArmPowerReason.NoRestPose: return "no rest pose configured on the robot";
                case ArmPowerReason.PathCollides: return "the path to the rest pose is not collision-free";
                case ArmPowerReason.ParkTimedOut: return "the park timed out before the arm reached the rest pose";
                case ArmPowerReason.BoardNotAnswering: return "the servo board is not answering";
                case ArmPowerReason.NoCollisionModel: return "the robot profile has no collision model";
                case ArmPowerReason.UnknownRequest: return "the robot did not understand the request";
                case ArmPowerReason.HoldRequested: return "the park was stopped by a hold request";
                default: return "reason " + ((byte)reason).ToString(System.Globalization.CultureInfo.InvariantCulture);
            }
        }
    }
}
