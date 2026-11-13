using System;
using System.IO;
using System.Linq;
using System.Text;
using Teleop.RobotArm.Wire;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// Writes the headset's link trace (docs/adr/0035 N1, trainer/contract/LINK_TRACE_FORMAT.md): every joint-state frame's
    /// raw stamps, every keepalive echo, and every change of the headset's own disturbance, as Core's
    /// <see cref="LinkTraceCodec"/> records, to <c>persistentDataPath/linktraces/&lt;UTC&gt;.ltrace</c>, one file a run.
    /// <c>just pull-galaxy-traces</c> brings them to the PC. About 2.5 KB/s: records go into a preallocated buffer and to the
    /// file once a second, so nothing allocates per frame. The newest <see cref="KeepFiles"/> files are kept.
    /// </summary>
    public sealed class LinkTraceRecorder : IDisposable
    {
        public const int KeepFiles = 200;
        private const int BufferRecords = 512;

        private readonly FileStream _file;
        private readonly byte[] _buffer = new byte[BufferRecords * LinkTraceCodec.RecordBytes];
        private readonly long _flushTicks;
        private int _used;
        private long _lastFlush;
        private bool _failed;

        public string Path { get; }
        public long Records { get; private set; }

        /// <param name="directory">Where the traces go (<c>persistentDataPath/linktraces</c>).</param>
        /// <param name="headerJson">The file's first line: who wrote it, the headset's clock rate, where it talked to.</param>
        public LinkTraceRecorder(string directory, string headerJson, long ticksPerSecond)
        {
            Directory.CreateDirectory(directory);
            Prune(directory);
            Path = System.IO.Path.Combine(directory, DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'") + ".ltrace");
            _file = new FileStream(Path, FileMode.CreateNew, FileAccess.Write, FileShare.Read, 1 << 16);
            byte[] header = Encoding.UTF8.GetBytes(headerJson.Replace("\n", " ") + "\n");
            _file.Write(header, 0, header.Length);
            _file.Flush();
            _flushTicks = ticksPerSecond;
        }

        public void Frame(uint sequence, long robotSend, long robotTps, long arrival, long sendOnOperator, long sensed) =>
            Add(LinkTraceKind.Frame, sequence, robotSend, robotTps, arrival, sendOnOperator, sensed);

        public void RoundTrip(long echoOperator, long echoRobotRecv, long robotSend, long robotTps, long arrival) =>
            Add(LinkTraceKind.RoundTrip, 0, echoOperator, echoRobotRecv, robotSend, robotTps, arrival);

        public void Conditions(float lagMs, float jitterMs, float lossPercent, float addedDelayMs, long now) =>
            Add(LinkTraceKind.Conditions, 0, (long)(lagMs * 1000), (long)(jitterMs * 1000), (long)(lossPercent * 10000), (long)(addedDelayMs * 1000), now);

        /// <summary>Once a frame: writes what has gathered, at most once a second (or when the buffer is full).</summary>
        public void Tick(long now)
        {
            if (_used > 0 && now - _lastFlush >= _flushTicks) Flush(now);
        }

        public void Dispose()
        {
            Flush(_lastFlush);
            _file.Dispose();
        }

        private void Add(LinkTraceKind kind, uint sequence, long a, long b, long c, long d, long e)
        {
            if (_failed) return;
            if (_used + LinkTraceCodec.RecordBytes > _buffer.Length) Flush(_lastFlush);
            if (LinkTraceCodec.TryEncode(kind, sequence, a, b, c, d, e, _buffer.AsSpan(_used)))
            {
                _used += LinkTraceCodec.RecordBytes;
                Records++;
            }
        }

        private void Flush(long now)
        {
            if (_used == 0 || _failed) return;
            try
            {
                _file.Write(_buffer, 0, _used);
                _file.Flush();
            }
            catch (IOException e)
            {
                _failed = true; // a full disk must not take the operator down: the trace stops, the robot does not
                Debug.LogError($"[linktrace] writing {Path} failed: {e.Message}; the trace stops here");
            }

            _used = 0;
            _lastFlush = now;
        }

        private static void Prune(string directory)
        {
            try
            {
                foreach (FileInfo old in new DirectoryInfo(directory).GetFiles("*.ltrace").OrderByDescending(f => f.Name).Skip(KeepFiles - 1))
                {
                    old.Delete();
                }
            }
            catch (IOException)
            {
                // an old trace that cannot go stays
            }
        }
    }
}
