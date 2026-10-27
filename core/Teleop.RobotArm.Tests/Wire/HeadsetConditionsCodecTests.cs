using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class HeadsetConditionsCodecTests
    {
        [Fact]
        public void RoundTripsExactly()
        {
            var c = new HeadsetConditions(150f, 20f, 2.5f, 200f, 0xFEEDFACECAFEBEEFUL);
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(c, 7, -42, buffer, out int written));
            Assert.Equal(40, written);
            Assert.True(HeadsetConditionsCodec.IsConditions(buffer));
            Assert.False(TrainingTaskCodec.IsTask(buffer));
            Assert.True(HeadsetConditionsCodec.TryDecode(buffer, out HeadsetConditions back, out uint seq, out long ticks));
            Assert.Equal(c, back);
            Assert.Equal((7u, -42L), (seq, ticks));
        }

        [Fact]
        public void LayoutIsFixed()
        {
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(new HeadsetConditions(1f, 0f, 0f, 0f, 2), 3, 4, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x84, 1, 0, 0,
                0x00, 0x00, 0x80, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 0, 0, 0, 0, 0, 0, 0,
                3, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0,
            }, buffer);
        }

        [Theory]
        [InlineData(0f, 0f, 0f, 0f, false, false)]
        [InlineData(100f, 0f, 0f, 0f, true, true)]
        [InlineData(0f, 5f, 0f, 0f, true, true)]
        [InlineData(0f, 0f, 1f, 0f, true, true)]
        [InlineData(0f, 0f, 0f, 200f, false, true)]
        [InlineData(float.NaN, 0f, 0f, 0f, true, true)]
        public void WhatCountsAsDisturbed(float lag, float jitter, float loss, float added, bool link, bool any)
        {
            var c = new HeadsetConditions(lag, jitter, loss, added, 0);
            Assert.Equal((link, any), (c.LinkDisturbed, c.Disturbed));
        }

        [Fact]
        public void AnotherVersionOrAShortRecord_IsRefused()
        {
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(default, 0, 0, buffer, out _));
            Assert.False(HeadsetConditionsCodec.TryDecode(buffer.AsSpan(0, buffer.Length - 1), out _, out _, out _));
            buffer[1] = 2;
            Assert.False(HeadsetConditionsCodec.TryDecode(buffer, out _, out _, out _));
        }
    }
}
