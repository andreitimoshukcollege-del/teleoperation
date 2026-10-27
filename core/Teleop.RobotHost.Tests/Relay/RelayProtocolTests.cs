using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    public class RelayProtocolTests
    {
        private static readonly JointTarget[] ThreeTargets =
        {
            new JointTarget(motorId: 1, angle: -3.25f, speed: 300f),
            new JointTarget(motorId: 2, angle: 1.5f, speed: 300f),
            new JointTarget(motorId: 3, angle: -0.75f, speed: 250f),
        };

        [Fact]
        public void Command_RoundTripsThroughEncodeDecode()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];

            int written = RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, decodeBuffer, out int targetCount);

            Assert.Equal(RelayProtocol.CommandEncodedSize(ThreeTargets.Length), written);
            Assert.True(ok);
            Assert.Equal(ThreeTargets.Length, targetCount);
            for (int i = 0; i < ThreeTargets.Length; i++)
            {
                Assert.Equal(ThreeTargets[i].MotorId, decodeBuffer[i].MotorId);
                Assert.Equal(ThreeTargets[i].Angle, decodeBuffer[i].Angle);
                Assert.Equal(ThreeTargets[i].Speed, decodeBuffer[i].Speed);
            }
        }

        [Fact]
        public void Feedback_RoundTripsThroughEncodeDecode()
        {
            var original = new[]
            {
                new JointFeedbackEntry(motorId: 1, valid: true, pulse: -42f),
                new JointFeedbackEntry(motorId: 2, valid: false, pulse: 0f),
                new JointFeedbackEntry(motorId: 3, valid: true, pulse: 88f),
                new JointFeedbackEntry(motorId: 4, valid: true, pulse: -5f),
            };
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize(original.Length)];

            int written = RelayProtocol.EncodeFeedback(original, buffer);
            Span<JointFeedbackEntry> decodeBuffer = stackalloc JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, decodeBuffer, out int entryCount);

            Assert.Equal(RelayProtocol.FeedbackEncodedSize(original.Length), written);
            Assert.True(ok);
            Assert.Equal(original.Length, entryCount);
            for (int i = 0; i < original.Length; i++)
            {
                Assert.Equal(original[i].MotorId, decodeBuffer[i].MotorId);
                Assert.Equal(original[i].Valid, decodeBuffer[i].Valid);
                Assert.Equal(original[i].Pulse, decodeBuffer[i].Pulse);
            }
        }

        [Fact]
        public void TryDecodeCommand_RejectsWrongVersion()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            buffer[0] = RelayProtocol.Version + 1;

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, decodeBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecodeCommand_RejectsTooShortBuffer()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            Span<byte> truncated = buffer.Slice(0, buffer.Length - 1);

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(truncated, decodeBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecodeFeedback_RejectsWrongVersion()
        {
            var entries = new[]
            {
                new JointFeedbackEntry(1, true, 1f), new JointFeedbackEntry(2, true, 1f),
                new JointFeedbackEntry(3, true, 1f), new JointFeedbackEntry(4, true, 1f),
            };
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize(entries.Length)];
            RelayProtocol.EncodeFeedback(entries, buffer);
            buffer[0] = RelayProtocol.Version + 1;

            Span<JointFeedbackEntry> decodeBuffer = stackalloc JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, decodeBuffer, out int entryCount);

            Assert.False(ok);
            Assert.Equal(0, entryCount);
        }

        [Fact]
        public void Command_LayoutIsVersionKindCount()
        {
            var buffer = new byte[RelayProtocol.CommandEncodedSize(1)];
            RelayProtocol.EncodeCommand(new[] { new JointTarget(7, 500f, 119f) }, buffer);
            Assert.Equal(5, RelayProtocol.Version);
            Assert.Equal(new byte[] { 5, 1, 1, 7 }, buffer.AsSpan(0, 4).ToArray());
            Assert.Equal(12, buffer.Length);
        }

        [Fact]
        public void Torque_RoundTrips_AndHasAFixedLayout()
        {
            var buffer = new byte[RelayProtocol.TorqueEncodedSize];
            Assert.Equal(3, RelayProtocol.EncodeTorque(false, buffer));
            Assert.Equal(new byte[] { 5, 3, 0 }, buffer);
            Assert.True(RelayProtocol.TryDecodeTorque(buffer, out bool on));
            Assert.False(on);

            RelayProtocol.EncodeTorque(true, buffer);
            Assert.Equal(new byte[] { 5, 3, 1 }, buffer);
            Assert.True(RelayProtocol.TryDecodeTorque(buffer, out on));
            Assert.True(on);
        }

        [Fact]
        public void Torque_FailsClosed_OnAnythingElse()
        {
            Assert.False(RelayProtocol.TryDecodeTorque(new byte[] { 5, 3 }, out _));
            Assert.False(RelayProtocol.TryDecodeTorque(new byte[] { 4, 3, 0 }, out _));
            Assert.False(RelayProtocol.TryDecodeTorque(new byte[] { 5, 1, 0 }, out _));
            Assert.False(RelayProtocol.TryDecodeTorque(new byte[] { 5, 3, 2 }, out _));
        }

        [Fact]
        public void EachKind_IsRejectedByTheOthersDecoders()
        {
            var command = new byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, command);
            var feedback = new byte[RelayProtocol.FeedbackEncodedSize(1)];
            RelayProtocol.EncodeFeedback(new[] { new JointFeedbackEntry(1, true, 1f) }, feedback);
            var torque = new byte[RelayProtocol.TorqueEncodedSize];
            RelayProtocol.EncodeTorque(true, torque);
            var targets = new JointTarget[RelayProtocol.MaxJointsPerMessage];
            var entries = new JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];

            Assert.False(RelayProtocol.TryDecodeFeedback(command, entries, out _));
            Assert.False(RelayProtocol.TryDecodeTorque(command, out _));
            Assert.False(RelayProtocol.TryDecodeCommand(feedback, targets, out _));
            Assert.False(RelayProtocol.TryDecodeTorque(feedback, out _));
            Assert.False(RelayProtocol.TryDecodeCommand(torque, targets, out _));
            Assert.False(RelayProtocol.TryDecodeFeedback(torque, entries, out _));
            Assert.Equal(RelayProtocol.KindCommand, RelayProtocol.KindOf(command));
            Assert.Equal(RelayProtocol.KindFeedback, RelayProtocol.KindOf(feedback));
            Assert.Equal(RelayProtocol.KindTorque, RelayProtocol.KindOf(torque));
        }

        [Fact]
        public void AVersion4Message_IsRejected()
        {
            // The old layout: version, count, tuples. A v4 relay's feedback must not be read as v5.
            byte[] v4Feedback = { 4, 1, 1, 1, 0, 0, 0x80, 0x3F };
            Assert.False(RelayProtocol.TryDecodeFeedback(v4Feedback, new JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage], out _));
            Assert.Equal(0, RelayProtocol.KindOf(v4Feedback));
        }

        [Fact]
        public void TryDecodeCommand_DeclaredCountExceedsCallersBuffer_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);

            Span<JointTarget> tooSmallBuffer = stackalloc JointTarget[ThreeTargets.Length - 1];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, tooSmallBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }
    }
}
