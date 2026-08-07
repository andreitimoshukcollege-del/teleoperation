using System;
using System.Buffers.Binary;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// One arm-joint command over the local relay channel (<see cref="IRelayClient"/>). Phase 1
    /// scope only: a single value for the base servo, in the same units the existing
    /// <c>jetrover_arm_control</c> ROS node's <c>/arm/servo/base</c> topic already expects -- a
    /// small relative "direction" multiplier (see <c>ServoController.setPos</c> in that repo:
    /// <c>nextPos = currentPos + stepSize * direction</c>), not an absolute angle. Lower/middle/
    /// upper joints and the gripper are added in the phase that wires up real inverse kinematics
    /// (docs/adr/0007-jetrover-plant-and-robot-host.md), which is also where "absolute target
    /// angle" bookkeeping belongs -- sending a relative nudge on every accepted
    /// <c>CommandFrame</c> is only safe for this phase's one-shot smoke test, not for a
    /// continuous operator stream.
    /// </summary>
    public readonly struct LocalArmCommand
    {
        public readonly float BaseDirection;

        public LocalArmCommand(float baseDirection)
        {
            BaseDirection = baseDirection;
        }
    }

    /// <summary>
    /// Feedback over the local relay channel. Phase 1 scope only: the base servo's last known
    /// angle in degrees, sourced from the ROS node's own <c>/unity/robot/servo/base</c>
    /// publisher. <see cref="BaseDegreesValid"/> is false whenever that publisher's own read
    /// from the board failed (a real, observed occurrence -- the board's serial read can time
    /// out) rather than silently reporting a stale or zeroed value as if it were current.
    /// </summary>
    public readonly struct LocalFeedback
    {
        public readonly bool BaseDegreesValid;
        public readonly int BaseDegrees;

        public LocalFeedback(bool baseDegreesValid, int baseDegrees)
        {
            BaseDegreesValid = baseDegreesValid;
            BaseDegrees = baseDegrees;
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
        public const byte Version = 1;

        public const int ArmCommandEncodedSize = 1 + 4; // version + BaseDirection
        public const int FeedbackEncodedSize = 1 + 1 + 4; // version + BaseDegreesValid + BaseDegrees

        public static int EncodeCommand(in LocalArmCommand command, Span<byte> destination)
        {
            if (destination.Length < ArmCommandEncodedSize)
            {
                throw new ArgumentException("Destination too small for a LocalArmCommand.", nameof(destination));
            }

            destination[0] = Version;
            BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(1, 4), command.BaseDirection);
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
            command = new LocalArmCommand(baseDirection);
            return true;
        }

        public static int EncodeFeedback(in LocalFeedback feedback, Span<byte> destination)
        {
            if (destination.Length < FeedbackEncodedSize)
            {
                throw new ArgumentException("Destination too small for a LocalFeedback.", nameof(destination));
            }

            destination[0] = Version;
            destination[1] = feedback.BaseDegreesValid ? (byte)1 : (byte)0;
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(2, 4), feedback.BaseDegrees);
            return FeedbackEncodedSize;
        }

        public static bool TryDecodeFeedback(ReadOnlySpan<byte> source, out LocalFeedback feedback)
        {
            feedback = default;
            if (source.Length < FeedbackEncodedSize || source[0] != Version)
            {
                return false;
            }

            bool valid = source[1] != 0;
            int baseDegrees = BinaryPrimitives.ReadInt32LittleEndian(source.Slice(2, 4));
            feedback = new LocalFeedback(valid, baseDegrees);
            return true;
        }
    }
}
