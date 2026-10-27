using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class TrainingTaskCodecTests
    {
        [Fact]
        public void State_RoundTripsExactly()
        {
            var state = new TrainingState(TrainingTask.Trace, TrainingPhase.Suspended, TrainingPurpose.Test, true, 0xCAFEF00D, 17, 3.8f, 62.5f);
            var buffer = new byte[TrainingTaskCodec.Size];
            Assert.True(TrainingTaskCodec.TryEncode(state, 0xDEADBEEF, -123456789012, buffer, out int written));
            Assert.Equal(34, written);
            Assert.True(TrainingTaskCodec.IsTask(buffer));
            Assert.False(ArmPowerCodec.IsRequest(buffer));

            Assert.True(TrainingTaskCodec.TryDecode(buffer, out TrainingState back, out uint seq, out long ticks));
            Assert.Equal(state, back);
            Assert.Equal((0xDEADBEEFu, -123456789012L), (seq, ticks));
        }

        [Fact]
        public void LayoutIsFixed()
        {
            var state = new TrainingState(TrainingTask.Reach, TrainingPhase.Active, TrainingPurpose.Train, false, 1, 2, 1.0f, 0f);
            var buffer = new byte[TrainingTaskCodec.Size];
            Assert.True(TrainingTaskCodec.TryEncode(state, 3, 4, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x83, 1, 1, 1, 0, 0,
                1, 0, 0, 0,
                2, 0, 0, 0,
                0x00, 0x00, 0x80, 0x3F, // 1.0f
                0, 0, 0, 0,
                3, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0,
            }, buffer);
        }

        [Fact]
        public void AVariedLevel_IsNaN_AndStillTheSameLabel()
        {
            var a = new TrainingState(TrainingTask.Shuttle, TrainingPhase.Active, TrainingPurpose.Train, false, 5, 3, float.NaN, 4f);
            var buffer = new byte[TrainingTaskCodec.Size];
            Assert.True(TrainingTaskCodec.TryEncode(a, 1, 1, buffer, out _));
            Assert.True(TrainingTaskCodec.TryDecode(buffer, out TrainingState b, out _, out _));
            Assert.True(float.IsNaN(b.SpeedLevel));
            Assert.True(a.SameLabel(b));
        }

        [Fact]
        public void TheScore_IsNotPartOfTheLabel()
        {
            var a = new TrainingState(TrainingTask.Reach, TrainingPhase.Active, TrainingPurpose.Train, false, 5, 3, 2.5f, 1f);
            var b = new TrainingState(TrainingTask.Reach, TrainingPhase.Active, TrainingPurpose.Train, false, 5, 3, 2.5f, 4f);
            var c = new TrainingState(TrainingTask.Reach, TrainingPhase.Done, TrainingPurpose.Train, false, 5, 3, 2.5f, 4f);
            Assert.True(a.SameLabel(b));
            Assert.False(a.Equals(b));
            Assert.False(b.SameLabel(c));
        }

        [Theory]
        [InlineData(2, 4)]   // a task this build does not define
        [InlineData(3, 4)]   // a phase
        [InlineData(4, 2)]   // a purpose
        [InlineData(1, 2)]   // another version
        public void UnknownBytes_AreRefused(int index, byte value)
        {
            var buffer = new byte[TrainingTaskCodec.Size];
            Assert.True(TrainingTaskCodec.TryEncode(default, 0, 0, buffer, out _));
            buffer[index] = value;
            Assert.False(TrainingTaskCodec.TryDecode(buffer, out _, out _, out _));
        }

        [Fact]
        public void AShortRecord_IsRefused_AndNothingIsWrittenIntoAShortBuffer()
        {
            var buffer = new byte[TrainingTaskCodec.Size];
            Assert.True(TrainingTaskCodec.TryEncode(default, 0, 0, buffer, out _));
            Assert.False(TrainingTaskCodec.TryDecode(buffer.AsSpan(0, TrainingTaskCodec.Size - 1), out _, out _, out _));
            var shortBuffer = new byte[TrainingTaskCodec.Size - 1];
            Assert.False(TrainingTaskCodec.TryEncode(default, 0, 0, shortBuffer, out _));
            Assert.All(shortBuffer, b => Assert.Equal(0, b));
        }

        [Fact]
        public void OtherMessagesOnThePort_AreNotTasks()
        {
            var power = new byte[ArmPowerCodec.RequestSize];
            ArmPowerCodec.TryEncodeRequest(OperatorClientKind.Headset, OperatorMotionMode.On, ArmPowerRequest.None, 1, 1, power, out _);
            Assert.False(TrainingTaskCodec.IsTask(power));
            Assert.False(TrainingTaskCodec.TryDecode(power, out _, out _, out _));
        }
    }
}
