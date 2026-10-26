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
    /// Every message is <c>u8 version | u8 kind | body</c>:
    /// <list type="bullet">
    /// <item><see cref="KindCommand"/>: <c>u8 count | count × { u8 motorId, f32 pulse, f32 pulsesPerSecond }</c>,
    /// RobotHost to relay.</item>
    /// <item><see cref="KindFeedback"/>: <c>u8 count | count × { u8 motorId, u8 valid, f32 pulse }</c>, relay
    /// to RobotHost.</item>
    /// <item><see cref="KindTorque"/>: <c>u8 on</c> (1 on, 0 off), RobotHost to relay, which publishes it
    /// on the driver's <c>/arm/servo/set/torque</c> (docs/adr/0023 decision 5).</item>
    /// </list>
    ///
    /// Carries one <see cref="JointTarget"/> (commands) or <see cref="JointFeedbackEntry"/>
    /// (feedback) per motor id the sending <c>RobotArmProfile</c> has
    /// (docs/adr/0011-generic-robot-arm-profiles.md). Both <see cref="JointTarget.Angle"/> and
    /// <see cref="JointTarget.Speed"/> are in <b>pulse units</b> on this hop (pulse and
    /// pulses/second respectively) -- continuing docs/adr/0010's reasoning for choosing pulse over
    /// radians here: it avoids duplicating <c>PulsePerRadian</c>/<c>ZeroPulse</c> conversion on the
    /// Python side.
    ///
    /// Targeting net8.0 (unlike Core's netstandard2.1) means the modern
    /// <see cref="BinaryPrimitives.WriteSingleLittleEndian"/>/<c>ReadSingleLittleEndian</c> can be
    /// used directly, without Core's bit-pattern round trip through <c>Int32</c>.
    /// </summary>
    public static class RelayProtocol
    {
        // v4: count-prefixed motor-id-keyed tuples, replacing the fixed 4-joint-plus-gripper
        // named-field structs entirely (docs/adr/0011).
        // v5 (2026-10-26, docs/adr/0023 decision 5): a kind byte after the version, and a third kind,
        // the torque message.
        //
        // BOTH ENDS DEPLOY TOGETHER. This file and robot/ros/src/teleop_relay/teleop_relay/relay_protocol.py
        // change in the same commit and go out together (just deploy-robot; or just deploy-ros, then just
        // deploy-robothost). A mismatch fails closed: each end rejects every message whose version or
        // kind it does not know, so a v5 RobotHost against a v4 relay moves nothing and reads nothing
        // (the headset then reports the servo board silent), rather than misreading a layout.
        public const byte Version = 5;

        /// <summary>The second byte of every message: what follows.</summary>
        public const byte KindCommand = 1;
        public const byte KindFeedback = 2;
        public const byte KindTorque = 3;

        private const int VersionSize = sizeof(byte);
        private const int KindSize = sizeof(byte);
        private const int CountSize = sizeof(byte);
        private const int MotorIdSize = sizeof(byte);
        private const int ValidSize = sizeof(byte);
        private const int FloatSize = sizeof(float);

        private const int HeaderSize = VersionSize + KindSize + CountSize;

        /// <summary>Version, kind, then one byte: 1 torque on, 0 torque off. No other value is accepted.</summary>
        public const int TorqueEncodedSize = VersionSize + KindSize + 1;

        /// <summary>MotorId, PulseTarget, PulsesPerSecond.</summary>
        private const int PerJointCommandSize = MotorIdSize + FloatSize + FloatSize;

        /// <summary>MotorId, Valid, Pulse.</summary>
        private const int PerJointFeedbackSize = MotorIdSize + ValidSize + FloatSize;

        /// <summary>
        /// Upper bound on joints per record. Generous relative to any profile this platform
        /// realistically targets (JetRover has 6) -- this hop is a local Unix domain socket, not a
        /// UDP link with a practical MTU to budget against, so there is no equivalent pressure to
        /// keep this tight the way <c>JointCommandCodec.MaxJointsPerMessage</c> has for its UDP hop.
        /// </summary>
        public const int MaxJointsPerMessage = 32;

        public static int CommandEncodedSize(int jointCount) => HeaderSize + jointCount * PerJointCommandSize;

        public static int FeedbackEncodedSize(int jointCount) => HeaderSize + jointCount * PerJointFeedbackSize;

        /// <summary>The kind byte of a message of this version; 0 when it is too short or another version.</summary>
        public static byte KindOf(ReadOnlySpan<byte> source) =>
            source.Length >= VersionSize + KindSize && source[0] == Version ? source[1] : (byte)0;

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

            int pos = WriteHeader(destination, KindCommand, targets.Length);
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
            if (source.Length < HeaderSize || KindOf(source) != KindCommand)
            {
                return false;
            }

            int pos = VersionSize + KindSize;
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

            int pos = WriteHeader(destination, KindFeedback, entries.Length);
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
            if (source.Length < HeaderSize || KindOf(source) != KindFeedback)
            {
                return false;
            }

            int pos = VersionSize + KindSize;
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

        /// <summary>
        /// Torque on (<paramref name="on"/> true) or off for every servo the driver drives. The relay
        /// publishes it on the driver's <c>/arm/servo/set/torque</c> as Int32 1 (on) or 0 (off). Returns
        /// the bytes written.
        /// </summary>
        public static int EncodeTorque(bool on, Span<byte> destination)
        {
            if (destination.Length < TorqueEncodedSize)
            {
                throw new ArgumentException("Destination too small for a torque message.", nameof(destination));
            }

            destination[0] = Version;
            destination[1] = KindTorque;
            destination[2] = on ? (byte)1 : (byte)0;
            return TorqueEncodedSize;
        }

        /// <summary>False for a short message, another version or kind, or a value other than 0 and 1.</summary>
        public static bool TryDecodeTorque(ReadOnlySpan<byte> source, out bool on)
        {
            on = false;
            if (source.Length < TorqueEncodedSize || KindOf(source) != KindTorque || source[2] > 1)
            {
                return false;
            }

            on = source[2] == 1;
            return true;
        }

        private static int WriteHeader(Span<byte> destination, byte kind, int count)
        {
            destination[0] = Version;
            destination[1] = kind;
            destination[2] = (byte)count;
            return HeaderSize;
        }
    }
}
