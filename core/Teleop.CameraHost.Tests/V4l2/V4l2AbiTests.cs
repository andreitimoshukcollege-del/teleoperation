using System.Runtime.CompilerServices;
using Teleop.CameraHost.V4l2;

namespace Teleop.CameraHost.Tests.V4l2
{
    /// <summary>
    /// Pins the hand-declared V4L2 ABI to the 64-bit Linux kernel's own values. These run on any
    /// machine: a wrong struct size or request number fails here instead of producing EINVAL (or a
    /// silently misread field) on the robot.
    /// </summary>
    public class V4l2AbiTests
    {
        [Theory]
        [InlineData(nameof(V4l2Capability), 104)]
        [InlineData(nameof(V4l2Format), 208)]
        [InlineData(nameof(V4l2RequestBuffers), 20)]
        [InlineData(nameof(V4l2Buffer), 88)]
        [InlineData(nameof(V4l2StreamParm), 204)]
        [InlineData(nameof(PollFd), 8)]
        [InlineData(nameof(Timespec), 16)]
        public void StructSizes_MatchThe64BitKernel(string name, int expected)
        {
            int actual = name switch
            {
                nameof(V4l2Capability) => Unsafe.SizeOf<V4l2Capability>(),
                nameof(V4l2Format) => Unsafe.SizeOf<V4l2Format>(),
                nameof(V4l2RequestBuffers) => Unsafe.SizeOf<V4l2RequestBuffers>(),
                nameof(V4l2Buffer) => Unsafe.SizeOf<V4l2Buffer>(),
                nameof(V4l2StreamParm) => Unsafe.SizeOf<V4l2StreamParm>(),
                nameof(PollFd) => Unsafe.SizeOf<PollFd>(),
                nameof(Timespec) => Unsafe.SizeOf<Timespec>(),
                _ => throw new ArgumentOutOfRangeException(nameof(name)),
            };

            Assert.Equal(expected, actual);
        }

        // Values as the kernel headers expand them on a 64-bit build (e.g. from strace or
        // videodev2.h). The size is encoded in bits 16-29, so these also re-check the struct sizes.
        [Fact]
        public void IoctlRequestNumbers_MatchTheKernel()
        {
            Assert.Equal((nuint)0x80685600u, V4l2Native.VidiocQueryCap);
            Assert.Equal((nuint)0xC0D05605u, V4l2Native.VidiocSFmt);
            Assert.Equal((nuint)0xC0145608u, V4l2Native.VidiocReqBufs);
            Assert.Equal((nuint)0xC0585609u, V4l2Native.VidiocQueryBuf);
            Assert.Equal((nuint)0xC058560Fu, V4l2Native.VidiocQBuf);
            Assert.Equal((nuint)0xC0585611u, V4l2Native.VidiocDQBuf);
            Assert.Equal((nuint)0x40045612u, V4l2Native.VidiocStreamOn);
            Assert.Equal((nuint)0x40045613u, V4l2Native.VidiocStreamOff);
            Assert.Equal((nuint)0xC0CC5616u, V4l2Native.VidiocSParm);
        }

        [Fact]
        public void BufferTimestamp_SitsAtOffset24AfterPadding()
        {
            var buffer = new V4l2Buffer { TimestampSeconds = 0x0102030405060708, TimestampMicroseconds = 0x1112131415161718 };
            Span<byte> bytes = MemoryMarshalBytes(ref buffer);
            Assert.Equal(0x08, bytes[24]);
            Assert.Equal(0x01, bytes[31]);
            Assert.Equal(0x18, bytes[32]);
            Assert.Equal(0x11, bytes[39]);
        }

        [Fact]
        public void MjpegFourCc_IsLittleEndianMJPG()
        {
            Assert.Equal(0x47504A4Du, V4l2Native.PixFmtMjpeg);
        }

        [Fact]
        public void HasHuffmanTables_FindsADhtSegmentBeforeStartOfScan()
        {
            byte[] withDht =
            {
                0xFF, 0xD8,                         // SOI
                0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // APP0, length 4
                0xFF, 0xC4, 0x00, 0x03, 0x00,       // DHT, length 3
                0xFF, 0xDA, 0x00, 0x02,             // SOS
            };
            Assert.True(TimestampSpike.HasHuffmanTables(withDht));
        }

        [Fact]
        public void HasHuffmanTables_IsFalseForUvcStyleMjpegWithoutDht()
        {
            byte[] withoutDht =
            {
                0xFF, 0xD8,
                0xFF, 0xDB, 0x00, 0x03, 0x00,       // DQT
                0xFF, 0xC0, 0x00, 0x02,             // SOF0
                0xFF, 0xDA, 0x00, 0x02,             // SOS: no DHT before it
                0xFF, 0xC4,                         // a DHT-looking pair inside scan data must not count
            };
            Assert.False(TimestampSpike.HasHuffmanTables(withoutDht));
        }

        private static Span<byte> MemoryMarshalBytes(ref V4l2Buffer buffer) =>
            System.Runtime.InteropServices.MemoryMarshal.AsBytes(
                System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref buffer, 1));
    }
}
