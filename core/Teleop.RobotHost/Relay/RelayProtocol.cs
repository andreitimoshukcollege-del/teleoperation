using System;
using System.Buffers.Binary;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// One joint's last-known pulse value, sourced from the ROS node's own feedback for that motor
    /// id. <see cref="Valid"/> is false whenever that read failed (a real, observed occurrence --
    /// the board's serial read can time out) rather than silently reporting a stale or zeroed
    /// value as if it were current. <see cref="Pulse"/> is raw pulse (0-1000, hardware units),
    /// straight from <c>bus_servo_read_position</c> with no degree round-trip -- the old wire
    /// (v3) reported degrees only because that was the shape the fixed-4-joint protocol happened
    /// to choose; nothing about the underlying read was ever in degrees
    /// (docs/adr/0011-generic-robot-arm-profiles.md).
    /// </summary>
    public readonly struct JointFeedbackEntry
    {
        public readonly byte MotorId;
        public readonly bool Valid;
        public readonly float Pulse;

        public JointFeedbackEntry(byte motorId, bool valid, float pulse)
        {
            MotorId = motorId;
            Valid = valid;
            Pulse = pulse;
        }
    }

    /// <summary>
    /// Fixed-header, count-prefixed, versioned, little-endian encode/decode for the local relay
    /// channel -- same style as Core's <c>RawPoseCodec</c>/<c>RobotStateFrameCodec</c>, but
    /// deliberately not an <c>ICommandCodec</c>: this wire format only ever crosses a local Unix
    /// domain socket between this host and the relay node, never the real network, so it has no
    /// staleness or sequencing fields at all.
    ///
    /// Carries one <see cref="JointTarget"/> (commands) or <see cref="JointFeedbackEntry"/>
    /// (feedback) per motor id the sending <c>RobotArmProfile</c> has -- generalized from the old
    /// fixed 4-arm-joint-plus-gripper <c>LocalArmCommand</c>/<c>LocalFeedback</c>
    /// (docs/adr/0011-generic-robot-arm-profiles.md). Both <see cref="JointTarget.Angle"/> and
    /// <see cref="JointTarget.Speed"/> are in <b>pulse units</b> on this hop (pulse and
    /// pulses/second respectively) -- continuing docs/adr/0010's reasoning for choosing pulse over
    /// radians here: it avoids duplicating <c>PulsePerRadian</c>/<c>ZeroPulse</c> conversion on the
    /// Python side. The gripper is no longer a separate degrees-based special case -- it is just
    /// another joint in the profile, flowing through this exact same pulse-unit path.
    ///
    /// Targeting net8.0 (unlike Core's netstandard2.1) means the modern
    /// <see cref="BinaryPrimitives.WriteSingleLittleEndian"/>/<c>ReadSingleLittleEndian</c> can be
    /// used directly, without Core's bit-pattern round trip through <c>Int32</c>.
    /// </summary>
    public static class RelayProtocol
    {
        // v4: count-prefixed motor-id-keyed tuples, replacing the fixed 4-joint-plus-gripper
        // named-field structs entirely (docs/adr/0011). A structurally different layout from v3,
        // not just a reinterpreted meaning -- must still fail closed on any version mismatch.
        public const byte Version = 4;

        private const int VersionSize = sizeof(byte);
        private const int CountSize = sizeof(byte);
        private const int MotorIdSize = sizeof(byte);
        private const int ValidSize = sizeof(byte);
        private const int FloatSize = sizeof(float);

        private const int HeaderSize = VersionSize + CountSize;

        /// <summary>MotorId, PulseTarget, PulsesPerSecond.</summary>
        private const int PerJointCommandSize = MotorIdSize + FloatSize + FloatSize;

        /// <summary>MotorId, Valid, Pulse.</summary>
        private const int PerJointFeedbackSize = MotorIdSize + ValidSize + FloatSize;

        /// <summary>
        /// Upper bound on joints per record. Generous relative to any profile this platform
        /// realistically targets (JetRover has 5) -- this hop is a local Unix domain socket, not a
        /// UDP link with a practical MTU to budget against, so there is no equivalent pressure to
        /// keep this tight the way <c>JointCommandCodec.MaxJointsPerMessage</c> has for its UDP hop.
        /// </summary>
        public const int MaxJointsPerMessage = 32;

        public static int CommandEncodedSize(int jointCount) => HeaderSize + jointCount * PerJointCommandSize;

        public static int FeedbackEncodedSize(int jointCount) => HeaderSize + jointCount * PerJointFeedbackSize;

        public static int EncodeCommand(ReadOnlySpan<JointTarget> targets, Span<byte> destination)
        {
            if (targets.Length > MaxJointsPerMessage)
            {
                throw new ArgumentException(
                    $"Cannot encode {targets.Length} joints; MaxJointsPerMessage is {MaxJointsPerMessage}.", nameof(targets));
            }

            int size = CommandEncodedSize(targets.Length);
            if (destination.Length < size)
            {
                throw new ArgumentException("Destination too small for this many joint targets.", nameof(destination));
            }

            int pos = 0;
            destination[pos] = Version;
            pos += VersionSize;
            destination[pos] = (byte)targets.Length;
            pos += CountSize;

            foreach (JointTarget target in targets)
            {
                destination[pos] = target.MotorId;
                pos += MotorIdSize;
                BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(pos, FloatSize), target.Angle);
                pos += FloatSize;
                BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(pos, FloatSize), target.Speed);
                pos += FloatSize;
            }

            return pos;
        }

        public static bool TryDecodeCommand(ReadOnlySpan<byte> source, Span<JointTarget> targetsBuffer, out int targetCount)
        {
            targetCount = 0;
            if (source.Length < HeaderSize || source[0] != Version)
            {
                return false;
            }

            int pos = VersionSize;
            int count = source[pos];
            pos += CountSize;

            if (count > MaxJointsPerMessage || count > targetsBuffer.Length || source.Length < CommandEncodedSize(count))
            {
                return false;
            }

            for (int i = 0; i < count; i++)
            {
                byte motorId = source[pos];
                pos += MotorIdSize;
                float pulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(pos, FloatSize));
                pos += FloatSize;
                float pulsesPerSecond = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(pos, FloatSize));
                pos += FloatSize;
                targetsBuffer[i] = new JointTarget(motorId, pulse, pulsesPerSecond);
            }

            targetCount = count;
            return true;
        }

        public static int EncodeFeedback(ReadOnlySpan<JointFeedbackEntry> entries, Span<byte> destination)
        {
            if (entries.Length > MaxJointsPerMessage)
            {
                throw new ArgumentException(
                    $"Cannot encode {entries.Length} joints; MaxJointsPerMessage is {MaxJointsPerMessage}.", nameof(entries));
            }

            int size = FeedbackEncodedSize(entries.Length);
            if (destination.Length < size)
            {
                throw new ArgumentException("Destination too small for this many feedback entries.", nameof(destination));
            }

            int pos = 0;
            destination[pos] = Version;
            pos += VersionSize;
            destination[pos] = (byte)entries.Length;
            pos += CountSize;

            foreach (JointFeedbackEntry entry in entries)
            {
                destination[pos] = entry.MotorId;
                pos += MotorIdSize;
                destination[pos] = entry.Valid ? (byte)1 : (byte)0;
                pos += ValidSize;
                BinaryPrimitives.WriteSingleLittleEndian(destination.Slice(pos, FloatSize), entry.Pulse);
                pos += FloatSize;
            }

            return pos;
        }

        public static bool TryDecodeFeedback(ReadOnlySpan<byte> source, Span<JointFeedbackEntry> entriesBuffer, out int entryCount)
        {
            entryCount = 0;
            if (source.Length < HeaderSize || source[0] != Version)
            {
                return false;
            }

            int pos = VersionSize;
            int count = source[pos];
            pos += CountSize;

            if (count > MaxJointsPerMessage || count > entriesBuffer.Length || source.Length < FeedbackEncodedSize(count))
            {
                return false;
            }

            for (int i = 0; i < count; i++)
            {
                byte motorId = source[pos];
                pos += MotorIdSize;
                bool valid = source[pos] != 0;
                pos += ValidSize;
                float pulse = BinaryPrimitives.ReadSingleLittleEndian(source.Slice(pos, FloatSize));
                pos += FloatSize;
                entriesBuffer[i] = new JointFeedbackEntry(motorId, valid, pulse);
            }

            entryCount = count;
            return true;
        }
    }
}
