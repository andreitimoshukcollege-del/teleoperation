using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    /// <summary>A scripted take's event, joint-drive to RobotHost (docs/adr/0036 amendment 5), which records it as SE.</summary>
    public class ScriptEventCodecTests
    {
        [Fact]
        public void AnEvent_RoundTripsExactly()
        {
            var e = new ScriptEvent(2, 3, ScriptEventKind.Touch, 41, 1.25f, 1.875f, 2.5f, 3.75f);
            var buffer = new byte[ScriptEventCodec.Size];
            Assert.True(ScriptEventCodec.TryEncode(e, 0xDEADBEEF, -123456789012, buffer, out int written));
            Assert.Equal(36, written);
            Assert.True(ScriptEventCodec.IsScriptEvent(buffer));
            Assert.False(TrainingTaskCodec.IsTask(buffer));
            Assert.True(ScriptEventCodec.TryDecode(buffer, out ScriptEvent back, out uint seq, out long ticks));
            Assert.Equal(e, back);
            Assert.Equal((0xDEADBEEFu, -123456789012L), (seq, ticks));
            // Nothing else on the joint port takes it for one of its own.
            Assert.False(JointCommandCodec.TryDecode(buffer, out _, out _, new JointTarget[JointCommandCodec.MaxJointsPerMessage], out _));
            Assert.False(TrainingTaskCodec.TryDecode(buffer, out _, out _, out _));
            Assert.False(RecordCodec.TryDecodeRequest(buffer, out _, out _, out _, out _, out _));
        }

        [Fact]
        public void TheLayout_IsFixed()
        {
            var e = new ScriptEvent(1, 2, ScriptEventKind.Knock, 7, 1f, 0f, 0f, 0f);
            var buffer = new byte[ScriptEventCodec.Size];
            Assert.True(ScriptEventCodec.TryEncode(e, 3, 4, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x8C, 1, 1, 2, 3, 7, 0, 0,
                3, 0, 0, 0,
                0x00, 0x00, 0x80, 0x3F, // 1.0f
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0,
            }, buffer);
        }

        [Fact]
        public void AStopsContact_MayBeNaN()
        {
            var e = new ScriptEvent(1, 1, ScriptEventKind.Stop, 33, 1f, float.NaN, 2f, 3f);
            var buffer = new byte[ScriptEventCodec.Size];
            Assert.True(ScriptEventCodec.TryEncode(e, 1, 1, buffer, out _));
            Assert.True(ScriptEventCodec.TryDecode(buffer, out ScriptEvent back, out _, out _));
            Assert.True(float.IsNaN(back.ContactSeconds));
            Assert.Equal(e, back);
            Assert.Equal("stop", ScriptEventCodec.Name(back.Kind));
        }

        [Theory]
        [InlineData(4, 0)]   // no kind
        [InlineData(4, 4)]   // a kind this build does not define
        [InlineData(1, 2)]   // another version
        [InlineData(0, 0x83)] // another message
        public void UnknownBytes_AreRefused(int index, byte value)
        {
            var buffer = new byte[ScriptEventCodec.Size];
            Assert.True(ScriptEventCodec.TryEncode(new ScriptEvent(1, 1, ScriptEventKind.Touch, 1, 0f, 0f, 0f, 0f), 0, 0, buffer, out _));
            buffer[index] = value;
            Assert.False(ScriptEventCodec.TryDecode(buffer, out _, out _, out _));
        }

        [Fact]
        public void AShortRecord_IsRefused_AndNothingIsWrittenIntoAShortBuffer()
        {
            var buffer = new byte[ScriptEventCodec.Size];
            Assert.True(ScriptEventCodec.TryEncode(new ScriptEvent(1, 1, ScriptEventKind.Touch, 1, 0f, 0f, 0f, 0f), 0, 0, buffer, out _));
            Assert.False(ScriptEventCodec.TryDecode(buffer.AsSpan(0, ScriptEventCodec.Size - 1), out _, out _, out _));
            var shortBuffer = new byte[ScriptEventCodec.Size - 1];
            Assert.False(ScriptEventCodec.TryEncode(default, 0, 0, shortBuffer, out int written));
            Assert.Equal(ScriptEventCodec.Size, written);
            Assert.All(shortBuffer, b => Assert.Equal(0, b));
        }
    }
}
