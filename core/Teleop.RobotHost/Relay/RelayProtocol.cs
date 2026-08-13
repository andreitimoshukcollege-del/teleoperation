using System;
using System.Buffers.Binary;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// One arm command over the local relay channel (<see cref="IRelayClient"/>). Base/lower/
    /// middle/upper are absolute pulse targets (0-1000, hardware units) -- <b>not</b> relative
    /// direction deltas (changed in wire v3, docs/adr/0010-absolute-joint-targets-over-local-relay.md;
    /// see that ADR for why sending the resulting target rather than the delta used to reach it
    /// removes an entire independently-maintained belief-tracking system on the ROS side).
    /// <see cref="JetRoverPlant"/> already computes exactly this value
    /// (<c>_targetPulseBase</c>/etc.) as the last step before sending -- this field is that value,
    /// unmodified. <see cref="GripperDegrees"/> is a different unit space, unaffected by this
    /// change: the gripper's own topic (<c>ServoController.setGripperPos</c>) already takes an
    /// absolute target angle in degrees, not pulse, so this field is denormalized
    /// <c>CommandFrame.Gripper</c> (0=open..1=closed) exactly as before.
    /// </summary>
    public readonly struct LocalArmCommand
    {
        public readonly float BasePulse;
        public readonly float LowerPulse;
        public readonly float MiddlePulse;
        public readonly float UpperPulse;
        public readonly float GripperDegrees;

        public LocalArmCommand(
            float basePulse, float lowerPulse, float middlePulse, float upperPulse, float gripperDegrees)
        {
            BasePulse = basePulse;
            LowerPulse = lowerPulse;
            MiddlePulse = middlePulse;
            UpperPulse = upperPulse;
            GripperDegrees = gripperDegrees;
        }
    }

    /// <summary>
    /// One joint's last-known angle in degrees, sourced from the ROS node's own feedback
    /// publisher for that joint. <see cref="Valid"/> is false whenever that publisher's own read
    /// from the board failed (a real, observed occurrence -- the board's serial read can time
    /// out) rather than silently reporting a stale or zeroed value as if it were current.
    /// </summary>
    public readonly struct JointFeedback
    {
        public readonly bool Valid;
        public readonly int Degrees;

        public JointFeedback(bool valid, int degrees)
        {
            Valid = valid;
            Degrees = degrees;
        }
    }

    /// <summary>Feedback for all four position-affecting joints, over the local relay channel.</summary>
    public readonly struct LocalFeedback
    {
        public readonly JointFeedback Base;
        public readonly JointFeedback Lower;
        public readonly JointFeedback Middle;
        public readonly JointFeedback Upper;

        public LocalFeedback(JointFeedback @base, JointFeedback lower, JointFeedback middle, JointFeedback upper)
        {
            Base = @base;
            Lower = lower;
            Middle = middle;
            Upper = upper;
        }
    }

    /// <summary>
    /// Fixed-size, versioned, little-endian encode/decode for the local relay channel -- same
    /// style as Core's <c>RawPoseCodec</c>/<c>RobotStateFrameCodec</c>, but deliberately not an
    /// <c>ICommandCodec</c>: this wire format only ever crosses a local Unix domain socket
    /// between this host and the relay node, never the real network, so it has no staleness or
    /// sequencing fields at all -- see <see cref="IRelayClient"/>'s doc for why. Targeting
    /// net8.0 (unlike Core's netstandard2.1) means the modern
    /// <see cref="BinaryPrimitives.WriteSingleLittleEndian"/>/<c>ReadSingleLittleEndian</c> can
    /// be used directly, without Core's bit-pattern round trip through <c>Int32</c>.
    /// </summary>
    public static class RelayProtocol
    {
        // v3: base/lower/middle/upper are absolute pulse targets, not relative direction deltas
        // -- docs/adr/0010-absolute-joint-targets-over-local-relay.md. Byte layout is unchanged
        // from v2 (still 5 floats); only the meaning of the first four inverts, which is exactly
        // why the version must still bump -- a stale peer must reject, not misinterpret.
        public const byte Version = 3;

        // version + 5 floats (base/lower/middle/upper absolute pulse + gripper degrees)
        public const int ArmCommandEncodedSize = 1 + 5 * 4;

        // version + 4 * (1 valid byte + 4-byte int32 degrees)
        public const int FeedbackEncodedSize = 1 + 4 * (1 + 4);

        public static int EncodeCommand(in LocalArmCommand command, Span<byte> destination)
        {
            if (destination.Length < ArmCommandEncodedSize)
            {
                throw new ArgumentException("Destination too small for a LocalArmCommand.", nameof(destination));
            }

            destination[0] = Version;
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(1, 4), command.BasePulse);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(5, 4), command.LowerPulse);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(9, 4), command.MiddlePulse);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(13, 4), command.UpperPulse);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(17, 4), command.GripperDegrees);
            return ArmCommandEncodedSize;
        }

        public static bool TryDecodeCommand(ReadOnlySpan<byte> source, out LocalArmCommand command)
        {
            command = default;
            if (source.Length < ArmCommandEncodedSize || source[0] != Version)
            {
                return false;
            }

            float basePulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(1, 4));
            float lowerPulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(5, 4));
            float middlePulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(9, 4));
            float upperPulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(13, 4));
            float gripperDegrees = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(17, 4));
            command = new LocalArmCommand(basePulse, lowerPulse, middlePulse, upperPulse, gripperDegrees);
            return true;
        }

        public static int EncodeFeedback(in LocalFeedback feedback, Span<byte> destination)
        {
            if (destination.Length < FeedbackEncodedSize)
            {
                throw new ArgumentException("Destination too small for a LocalFeedback.", nameof(destination));
            }

            destination[0] = Version;
            WriteJoint(feedback.Base, destination.Slice(1, 5));
            WriteJoint(feedback.Lower, destination.Slice(6, 5));
            WriteJoint(feedback.Middle, destination.Slice(11, 5));
            WriteJoint(feedback.Upper, destination.Slice(16, 5));
            return FeedbackEncodedSize;
        }

        public static bool TryDecodeFeedback(ReadOnlySpan<byte> source, out LocalFeedback feedback)
        {
            feedback = default;
            if (source.Length < FeedbackEncodedSize || source[0] != Version)
            {
                return false;
            }

            feedback = new LocalFeedback(
                ReadJoint(source.Slice(1, 5)),
                ReadJoint(source.Slice(6, 5)),
                ReadJoint(source.Slice(11, 5)),
                ReadJoint(source.Slice(16, 5)));
            return true;
        }

        private static void WriteJoint(JointFeedback joint, Span<byte> destination)
        {
            destination[0] = joint.Valid ? (byte)1 : (byte)0;
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(1, 4), joint.Degrees);
        }

        private static JointFeedback ReadJoint(ReadOnlySpan<byte> source)
        {
            bool valid = source[0] != 0;
            int degrees = BinaryPrimitives.ReadInt32LittleEndian(source.Slice(1, 4));
            return new JointFeedback(valid, degrees);
        }
    }
}
