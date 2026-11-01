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
            Assert.Equal(56, written);
            Assert.True(HeadsetConditionsCodec.IsConditions(buffer));
            Assert.False(TrainingTaskCodec.IsTask(buffer));
            Assert.True(HeadsetConditionsCodec.TryDecode(buffer, out HeadsetConditions back, out uint seq, out long ticks));
            Assert.Equal(c, back);
            Assert.Equal((7u, -42L), (seq, ticks));
        }

        [Fact]
        public void TheHandFilter_RoundTrips()
        {
            var c = new HeadsetConditions(0f, 0f, 0f, 0f, 0, 1f, 80f, 16f, 1f);
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(c, 1, 2, buffer, out _));
            Assert.True(HeadsetConditionsCodec.TryDecode(buffer, out HeadsetConditions back, out _, out _));
            Assert.Equal(c, back);
            Assert.True(back.ReportsHandFilter);
            Assert.False(back.Disturbed);
            Assert.NotEqual(c, new HeadsetConditions(0f, 0f, 0f, 0f, 0, 1f, 80f, 16f, 2f));
        }

        [Fact]
        public void LayoutIsFixed()
        {
            // The first 40 bytes are the record as it was before the hand block: what every older decoder reads.
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(new HeadsetConditions(1f, 0f, 0f, 0f, 2, 0.5f, 80f, 16f, 2f), 3, 4, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x84, 1, 0, 0,
                0x00, 0x00, 0x80, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                2, 0, 0, 0, 0, 0, 0, 0,
                3, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0,
                0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0xA0, 0x42, 0x00, 0x00, 0x80, 0x41, 0x00, 0x00, 0x00, 0x40,
            }, buffer);
        }

        [Fact]
        public void ARecordWithoutTheHandBlock_DecodesWithTheHandsUnsaid()
        {
            // A headset built before the block sends only the first 40 bytes.
            var buffer = new byte[HeadsetConditionsCodec.Size];
            Assert.True(HeadsetConditionsCodec.TryEncode(new HeadsetConditions(150f, 20f, 2.5f, 200f, 9, 1f, 80f, 16f, 1f), 7, 8, buffer, out _));
            Assert.True(HeadsetConditionsCodec.TryDecode(buffer.AsSpan(0, HeadsetConditionsCodec.BaseSize), out HeadsetConditions old, out uint seq, out long ticks));
            Assert.Equal(new HeadsetConditions(150f, 20f, 2.5f, 200f, 9), old);
            Assert.False(old.ReportsHandFilter);
            Assert.Equal((7u, 8L), (seq, ticks));
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
            Assert.False(HeadsetConditionsCodec.TryDecode(buffer.AsSpan(0, HeadsetConditionsCodec.BaseSize - 1), out _, out _, out _));
            Assert.False(HeadsetConditionsCodec.TryEncode(default, 0, 0, new byte[HeadsetConditionsCodec.Size - 1], out _));
            buffer[1] = 2;
            Assert.False(HeadsetConditionsCodec.TryDecode(buffer, out _, out _, out _));
        }
    }
}
