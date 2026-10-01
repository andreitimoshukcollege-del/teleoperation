using System.Runtime.InteropServices;

namespace Teleop.CameraHost.V4l2
{
    /// <summary>
    /// The slice of the Linux V4L2 and libc ABI this host needs, declared by hand because .NET has
    /// no video-capture API (docs/adr/0014-camera-frame-downlink.md, resolved question 1).
    ///
    /// Every struct is <see cref="LayoutKind.Explicit"/> with its size pinned, and laid out for a
    /// 64-bit Linux kernel (the Jetson is aarch64). Getting one offset wrong does not crash: the
    /// kernel rejects the ioctl because the size is encoded in the request number, or worse, reads
    /// the wrong field silently. <c>V4l2AbiTests</c> pins every size and request number against the
    /// kernel's own values, so a layout mistake fails a test on any machine instead of misbehaving on
    /// the robot.
    /// </summary>
    internal static unsafe class V4l2Native
    {
        // --- ioctl request encoding (include/uapi/asm-generic/ioctl.h; aarch64 uses the generic one) ---
        private const int IocWrite = 1;
        private const int IocRead = 2;

        internal static nuint Ioc(int direction, char type, int number, int size) =>
            (nuint)(((uint)direction << 30) | ((uint)size << 16) | ((uint)type << 8) | (uint)number);

        internal static nuint Ior<T>(int number) where T : unmanaged => Ioc(IocRead, 'V', number, sizeof(T));
        internal static nuint Iow<T>(int number) where T : unmanaged => Ioc(IocWrite, 'V', number, sizeof(T));
        internal static nuint Iowr<T>(int number) where T : unmanaged => Ioc(IocRead | IocWrite, 'V', number, sizeof(T));

        internal static readonly nuint VidiocQueryCap = Ior<V4l2Capability>(0);
        internal static readonly nuint VidiocSFmt = Iowr<V4l2Format>(5);
        internal static readonly nuint VidiocReqBufs = Iowr<V4l2RequestBuffers>(8);
        internal static readonly nuint VidiocQueryBuf = Iowr<V4l2Buffer>(9);
        internal static readonly nuint VidiocQBuf = Iowr<V4l2Buffer>(15);
        internal static readonly nuint VidiocDQBuf = Iowr<V4l2Buffer>(17);
        internal static readonly nuint VidiocStreamOn = Iow<int>(18);
        internal static readonly nuint VidiocStreamOff = Iow<int>(19);
        internal static readonly nuint VidiocSParm = Iowr<V4l2StreamParm>(22);

        // --- V4L2 constants (include/uapi/linux/videodev2.h) ---
        internal const uint BufTypeVideoCapture = 1;
        internal const uint MemoryMmap = 1;
        internal const uint FieldNone = 1;

        internal const uint BufFlagTimestampMask = 0x0000e000;
        internal const uint BufFlagTimestampUnknown = 0x00000000;
        internal const uint BufFlagTimestampMonotonic = 0x00002000;
        internal const uint BufFlagTimestampCopy = 0x00004000;
        internal const uint BufFlagTstampSrcMask = 0x00070000;
        internal const uint BufFlagTstampSrcEof = 0x00000000;
        internal const uint BufFlagTstampSrcSoe = 0x00010000;

        internal static uint FourCc(char a, char b, char c, char d) =>
            (uint)a | ((uint)b << 8) | ((uint)c << 16) | ((uint)d << 24);

        internal static readonly uint PixFmtMjpeg = FourCc('M', 'J', 'P', 'G');

        // --- libc ---
        internal const int ORdWr = 2;
        internal const int ONonBlock = 0x800;
        internal const int ProtRead = 1;
        internal const int ProtWrite = 2;
        internal const int MapShared = 1;
        internal const int ClockMonotonic = 1;
        internal const short PollIn = 1;
        internal const int EIntr = 4;
        internal const int EAgain = 11;

        private const string Libc = "libc.so.6";

        [DllImport(Libc, SetLastError = true)]
        internal static extern int open([MarshalAs(UnmanagedType.LPStr)] string path, int flags);

        [DllImport(Libc, SetLastError = true)]
        internal static extern int close(int fd);

        [DllImport(Libc, SetLastError = true)]
        internal static extern int ioctl(int fd, nuint request, void* arg);

        [DllImport(Libc, SetLastError = true)]
        internal static extern IntPtr mmap(IntPtr address, nuint length, int protection, int flags, int fd, long offset);

        [DllImport(Libc, SetLastError = true)]
        internal static extern int munmap(IntPtr address, nuint length);

        [DllImport(Libc, SetLastError = true)]
        internal static extern int poll(PollFd* fds, nuint count, int timeoutMilliseconds);

        [DllImport(Libc, SetLastError = true)]
        internal static extern int clock_gettime(int clockId, Timespec* time);

        internal static readonly IntPtr MapFailed = new IntPtr(-1);
    }

    [StructLayout(LayoutKind.Explicit, Size = 104)]
    internal unsafe struct V4l2Capability
    {
        [FieldOffset(0)] public fixed byte Driver[16];
        [FieldOffset(16)] public fixed byte Card[32];
        [FieldOffset(48)] public fixed byte BusInfo[32];
        [FieldOffset(80)] public uint Version;
        [FieldOffset(84)] public uint Capabilities;
        [FieldOffset(88)] public uint DeviceCaps;
    }

    /// <summary><c>struct v4l2_format</c>: a u32 type, then a 200-byte union aligned to 8 (it holds pointers).</summary>
    [StructLayout(LayoutKind.Explicit, Size = 208)]
    internal struct V4l2Format
    {
        [FieldOffset(0)] public uint Type;
        [FieldOffset(8)] public uint Width;
        [FieldOffset(12)] public uint Height;
        [FieldOffset(16)] public uint PixelFormat;
        [FieldOffset(20)] public uint Field;
        [FieldOffset(24)] public uint BytesPerLine;
        [FieldOffset(28)] public uint SizeImage;
    }

    [StructLayout(LayoutKind.Explicit, Size = 20)]
    internal struct V4l2RequestBuffers
    {
        [FieldOffset(0)] public uint Count;
        [FieldOffset(4)] public uint Type;
        [FieldOffset(8)] public uint Memory;
    }

    /// <summary>
    /// <c>struct v4l2_buffer</c> on a 64-bit kernel. The <c>struct timeval</c> sits at offset 24 after 4
    /// bytes of padding, and holds two 64-bit longs; the driver's capture time is read from it.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 88)]
    internal struct V4l2Buffer
    {
        [FieldOffset(0)] public uint Index;
        [FieldOffset(4)] public uint Type;
        [FieldOffset(8)] public uint BytesUsed;
        [FieldOffset(12)] public uint Flags;
        [FieldOffset(16)] public uint Field;
        [FieldOffset(24)] public long TimestampSeconds;
        [FieldOffset(32)] public long TimestampMicroseconds;
        [FieldOffset(56)] public uint Sequence;
        [FieldOffset(60)] public uint Memory;
        [FieldOffset(64)] public uint Offset;
        [FieldOffset(72)] public uint Length;
    }

    /// <summary><c>struct v4l2_streamparm</c> with the capture member of its 200-byte union.</summary>
    [StructLayout(LayoutKind.Explicit, Size = 204)]
    internal struct V4l2StreamParm
    {
        [FieldOffset(0)] public uint Type;
        [FieldOffset(4)] public uint Capability;
        [FieldOffset(8)] public uint CaptureMode;
        [FieldOffset(12)] public uint TimePerFrameNumerator;
        [FieldOffset(16)] public uint TimePerFrameDenominator;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct PollFd
    {
        public int Fd;
        public short Events;
        public short ReturnedEvents;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct Timespec
    {
        public long Seconds;
        public long Nanoseconds;
    }
}
