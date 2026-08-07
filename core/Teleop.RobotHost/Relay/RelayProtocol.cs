using System;
using System.Buffers.Binary;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// One arm command over the local relay channel (<see cref="IRelayClient"/>). Base/lower/
    /// middle/upper are relative "direction" multipliers, in the same units the existing
    /// <c>jetrover_arm_control</c> ROS node's <c>/arm/servo/*</c> topics already expect (see
    /// <c>ServoController.setPos</c> in that repo: <c>nextPos = currentPos + stepSize *
    /// direction</c>) -- not absolute angles. <see cref="GripperDegrees"/> is the one exception:
    /// the gripper's own topic (<c>ServoController.setGripperPos</c>) already takes an absolute
    /// target angle in degrees, not a relative step, so this field is denormalized
    /// <c>CommandFrame.Gripper</c> (0=open..1=closed), not a delta.
    ///
    /// <see cref="JetRoverPlant"/> computes each relative field as (IK target angle - its own
    /// believed current angle from the last feedback), not as an independent value per call --
    /// see that class's doc for why sending a raw relative nudge on every accepted
    /// <c>CommandFrame</c> would only ever have been safe for Phase 1's one-shot smoke test.
    /// </summary>
    public readonly struct LocalArmCommand
    {
        public readonly float BaseDirection;
        public readonly float LowerDirection;
        public readonly float MiddleDirection;
        public readonly float UpperDirection;
        public readonly float GripperDegrees;

        public LocalArmCommand(
            float baseDirection, float lowerDirection, float middleDirection, float upperDirection, float gripperDegrees)
        {
            BaseDirection = baseDirection;
            LowerDirection = lowerDirection;
            MiddleDirection = middleDirection;
            UpperDirection = upperDirection;
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
        public const byte Version = 2;

        // version + 5 floats (base/lower/middle/upper direction + gripper degrees)
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
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(1, 4), command.BaseDirection);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(5, 4), command.LowerDirection);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(9, 4), command.MiddleDirection);
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(13, 4), command.UpperDirection);
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

            float baseDirection = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(1, 4));
            float lowerDirection = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(5, 4));
            float middleDirection = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(9, 4));
            float upperDirection = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(13, 4));
            float gripperDegrees = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(17, 4));
            command = new LocalArmCommand(baseDirection, lowerDirection, middleDirection, upperDirection, gripperDegrees);
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
