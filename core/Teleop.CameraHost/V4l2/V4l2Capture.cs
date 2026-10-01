using System.Runtime.InteropServices;
using System.Text;

namespace Teleop.CameraHost.V4l2
{
    /// <summary>One dequeued frame: the driver's bytes, its stamp, and how the stamp was taken.</summary>
    internal readonly struct CapturedFrame
    {
        public readonly uint BufferIndex;
        public readonly uint Sequence;
        public readonly uint Flags;
        public readonly long TimestampNanoseconds;
        public readonly int BytesUsed;

        public CapturedFrame(uint bufferIndex, uint sequence, uint flags, long timestampNanoseconds, int bytesUsed)
        {
            BufferIndex = bufferIndex;
            Sequence = sequence;
            Flags = flags;
            TimestampNanoseconds = timestampNanoseconds;
            BytesUsed = bytesUsed;
        }
    }

    /// <summary>
    /// Memory-mapped V4L2 streaming capture of a camera's native MJPEG frames
    /// (docs/adr/0014-camera-frame-downlink.md §3, §4). The frame bytes stay in the driver's
    /// buffers; a caller reads them through <see cref="FrameBytes"/> and then hands the buffer back
    /// with <see cref="Requeue"/>. Nothing is decoded or re-encoded.
    ///
    /// Linux only. Not thread-safe.
    /// </summary>
    internal sealed unsafe class V4l2Capture : IDisposable
    {
        private readonly int _fd;
        // Null only if construction failed before buffers were requested; Dispose handles that.
        private readonly IntPtr[]? _maps;
        private readonly nuint[]? _lengths;
        private bool _streaming;

        public string Driver { get; }
        public string Card { get; }
        public uint Width { get; }
        public uint Height { get; }
        public uint FramesPerSecondNumerator { get; }
        public uint FramesPerSecondDenominator { get; }

        public V4l2Capture(string device, uint width, uint height, uint framesPerSecond, int bufferCount = 4)
        {
            _fd = V4l2Native.open(device, V4l2Native.ORdWr | V4l2Native.ONonBlock);
            if (_fd < 0)
            {
                throw Failure($"open {device}");
            }

            try
            {
                V4l2Capability capability = default;
                Ioctl(V4l2Native.VidiocQueryCap, &capability, "VIDIOC_QUERYCAP");
                Driver = CString(capability.Driver, 16);
                Card = CString(capability.Card, 32);

                var format = new V4l2Format
                {
                    Type = V4l2Native.BufTypeVideoCapture,
                    Width = width,
                    Height = height,
                    PixelFormat = V4l2Native.PixFmtMjpeg,
                    Field = V4l2Native.FieldNone,
                };
                Ioctl(V4l2Native.VidiocSFmt, &format, "VIDIOC_S_FMT");
                if (format.PixelFormat != V4l2Native.PixFmtMjpeg)
                {
                    throw new InvalidOperationException($"{device} would not deliver MJPEG (driver chose another format).");
                }

                Width = format.Width;
                Height = format.Height;

                // timeperframe is seconds per frame, so 15 fps is 1/15.
                var parm = new V4l2StreamParm
                {
                    Type = V4l2Native.BufTypeVideoCapture,
                    TimePerFrameNumerator = 1,
                    TimePerFrameDenominator = framesPerSecond,
                };
                Ioctl(V4l2Native.VidiocSParm, &parm, "VIDIOC_S_PARM");
                FramesPerSecondNumerator = parm.TimePerFrameDenominator;
                FramesPerSecondDenominator = parm.TimePerFrameNumerator;

                var request = new V4l2RequestBuffers
                {
                    Count = (uint)bufferCount,
                    Type = V4l2Native.BufTypeVideoCapture,
                    Memory = V4l2Native.MemoryMmap,
                };
                Ioctl(V4l2Native.VidiocReqBufs, &request, "VIDIOC_REQBUFS");

                _maps = new IntPtr[request.Count];
                _lengths = new nuint[request.Count];
                for (uint i = 0; i < request.Count; i++)
                {
                    var buffer = new V4l2Buffer
                    {
                        Index = i,
                        Type = V4l2Native.BufTypeVideoCapture,
                        Memory = V4l2Native.MemoryMmap,
                    };
                    Ioctl(V4l2Native.VidiocQueryBuf, &buffer, "VIDIOC_QUERYBUF");

                    IntPtr map = V4l2Native.mmap(
                        IntPtr.Zero, buffer.Length, V4l2Native.ProtRead | V4l2Native.ProtWrite,
                        V4l2Native.MapShared, _fd, buffer.Offset);
                    if (map == V4l2Native.MapFailed)
                    {
                        throw Failure($"mmap buffer {i}");
                    }

                    _maps[i] = map;
                    _lengths[i] = buffer.Length;
                    Ioctl(V4l2Native.VidiocQBuf, &buffer, "VIDIOC_QBUF");
                }

                int type = (int)V4l2Native.BufTypeVideoCapture;
                Ioctl(V4l2Native.VidiocStreamOn, &type, "VIDIOC_STREAMON");
                _streaming = true;
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        /// <summary>
        /// Waits up to <paramref name="timeoutMilliseconds"/> for the next frame and dequeues it.
        /// Returns false on timeout. The caller must <see cref="Requeue"/> the frame's buffer.
        /// </summary>
        public bool TryDequeue(int timeoutMilliseconds, out CapturedFrame frame)
        {
            frame = default;
            var pollFd = new PollFd { Fd = _fd, Events = V4l2Native.PollIn };
            int ready;
            do
            {
                ready = V4l2Native.poll(&pollFd, 1, timeoutMilliseconds);
            }
            while (ready < 0 && Marshal.GetLastPInvokeError() == V4l2Native.EIntr);

            if (ready < 0)
            {
                throw Failure("poll");
            }

            if (ready == 0)
            {
                return false;
            }

            var buffer = new V4l2Buffer { Type = V4l2Native.BufTypeVideoCapture, Memory = V4l2Native.MemoryMmap };
            if (V4l2Native.ioctl(_fd, V4l2Native.VidiocDQBuf, &buffer) < 0)
            {
                if (Marshal.GetLastPInvokeError() == V4l2Native.EAgain)
                {
                    return false;
                }

                throw Failure("VIDIOC_DQBUF");
            }

            long nanoseconds = (buffer.TimestampSeconds * 1_000_000_000L) + (buffer.TimestampMicroseconds * 1_000L);
            frame = new CapturedFrame(buffer.Index, buffer.Sequence, buffer.Flags, nanoseconds, (int)buffer.BytesUsed);
            return true;
        }

        public ReadOnlySpan<byte> FrameBytes(in CapturedFrame frame) =>
            new ReadOnlySpan<byte>((void*)_maps![frame.BufferIndex], frame.BytesUsed);

        public void Requeue(in CapturedFrame frame)
        {
            var buffer = new V4l2Buffer
            {
                Index = frame.BufferIndex,
                Type = V4l2Native.BufTypeVideoCapture,
                Memory = V4l2Native.MemoryMmap,
            };
            Ioctl(V4l2Native.VidiocQBuf, &buffer, "VIDIOC_QBUF");
        }

        public void Dispose()
        {
            if (_streaming)
            {
                int type = (int)V4l2Native.BufTypeVideoCapture;
                V4l2Native.ioctl(_fd, V4l2Native.VidiocStreamOff, &type);
                _streaming = false;
            }

            if (_maps != null)
            {
                for (int i = 0; i < _maps.Length; i++)
                {
                    if (_maps[i] != IntPtr.Zero)
                    {
                        V4l2Native.munmap(_maps[i], _lengths![i]);
                        _maps[i] = IntPtr.Zero;
                    }
                }
            }

            if (_fd >= 0)
            {
                V4l2Native.close(_fd);
            }
        }

        private void Ioctl(nuint request, void* argument, string name)
        {
            int result;
            do
            {
                result = V4l2Native.ioctl(_fd, request, argument);
            }
            while (result < 0 && Marshal.GetLastPInvokeError() == V4l2Native.EIntr);

            if (result < 0)
            {
                throw Failure(name);
            }
        }

        private static InvalidOperationException Failure(string what)
        {
            int errno = Marshal.GetLastPInvokeError();
            return new InvalidOperationException($"{what} failed: errno {errno} ({Marshal.GetPInvokeErrorMessage(errno)})");
        }

        private static string CString(byte* bytes, int max)
        {
            int length = 0;
            while (length < max && bytes[length] != 0)
            {
                length++;
            }

            return Encoding.ASCII.GetString(bytes, length);
        }
    }
}
