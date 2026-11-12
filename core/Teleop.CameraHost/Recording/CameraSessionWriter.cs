using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Concurrent;
using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Teleop.Core.Types;

namespace Teleop.CameraHost.Recording
{
    /// <summary>
    /// Records the camera into the session the robot is recording (docs/adr/0014 amendment 3), so that a real take holds
    /// its frames as an Isaac take does (docs/adr/0036).
    /// <para>It follows the control file as the driver's <c>session_log.py</c> does. The file is polled every
    /// <c>pollSeconds</c>, and while it names a session directory this writes <c>camera.jlog</c> and
    /// <c>camera/front.rgb.frames</c> there. The format is the Isaac renderer's (<c>trainer/contract/SESSION_FORMAT.md</c>,
    /// "The robot's camera"): a <c>JSES</c> header, <c>CM</c>, one <c>VF</c> per frame, <c>FD</c> for a frame not
    /// recorded, <c>ER</c> when it stopped early, an <c>FS</c> line per sidecar, and the <c>END</c> trailer. A log that
    /// already exists is never truncated: after a restart in the same session it writes <c>camera.2.jlog</c> with
    /// <c>.2</c> sidecars.</para>
    /// <para>Frames go only into takes (a <c>session.json</c> with a <c>scenario</c>, docs/adr/0033) unless
    /// <c>takesOnly</c> is false. Another session, such as an automatic one opened whenever the headset drives the arm,
    /// gets a <c>camera.jlog</c> with no frames. That keeps the Jetson's disk for takes (about 4.5 GB an hour of camera),
    /// while RobotHost, which waits for every writer's trailer, still finds one.</para>
    /// <para>The capture loop calls <see cref="OfferFrame"/>, which copies the frame into a bounded queue and never waits
    /// on the disk. A full queue drops the frame from the recording, with an <c>FD</c>, never from the stream. The disk
    /// is written on a thread of its own. Below <c>minFreeBytes</c> free, the session's recording stops with an
    /// <c>ER</c>, and it is not reopened while the control file still names that session.</para>
    /// </summary>
    internal sealed class CameraSessionWriter : IDisposable
    {
        public const string Writer = "camera";
        public const string Folder = "camera";
        public const string Camera = "front";
        public const string Stream = Camera + ".rgb";
        private const string Clock = "CLOCK_MONOTONIC";

        private readonly string _controlPath;
        private readonly Func<long> _now;
        private readonly long _ticksPerSecond;
        private readonly int _capacity;
        private readonly long _minFreeBytes;
        private readonly Func<string, long> _freeBytes;
        private readonly Action<string> _log;
        private readonly long _pollTicks;
        private readonly bool _takesOnly;
        private readonly ConcurrentQueue<Item> _queue = new();
        private readonly Thread? _thread;
        private volatile bool _stop;
        private volatile bool _active;
        private volatile bool _framesWanted;
        private int _queued;
        private long _droppedFrames;

        private Session? _session;
        private string? _failedDirectory;
        private long _nextPoll = long.MinValue;

        private readonly struct Item
        {
            public Item(string? record, byte[]? frame, int length, long queuedTicks, uint frameId, long captureTicks, ushort width, ushort height)
            {
                Record = record;
                Frame = frame;
                Length = length;
                QueuedTicks = queuedTicks;
                FrameId = frameId;
                CaptureTicks = captureTicks;
                Width = width;
                Height = height;
            }

            public string? Record { get; }
            public byte[]? Frame { get; }
            public int Length { get; }
            public long QueuedTicks { get; }
            public uint FrameId { get; }
            public long CaptureTicks { get; }
            public ushort Width { get; }
            public ushort Height { get; }
        }

        private sealed class Session
        {
            public Session(string directory, string suffix, FileStream log, FileStream? sidecar)
            {
                Directory = directory;
                Suffix = suffix;
                Log = log;
                Sidecar = sidecar;
            }

            public string Directory { get; }
            public string Suffix { get; }
            public FileStream Log { get; }
            public FileStream? Sidecar { get; }
            public IncrementalHash LogHash { get; } = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
            public IncrementalHash SidecarHash { get; } = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
            public long SidecarBytes { get; set; }
            public long Records { get; set; }
            public long Dropped { get; set; }
            public long Frames { get; set; }
            public (ushort Width, ushort Height) Size { get; set; }
        }

        /// <param name="startThread">False for tests, which drive <see cref="Step"/> themselves.</param>
        public CameraSessionWriter(string controlPath, Func<long> now, long ticksPerSecond, long minFreeBytes,
            Func<string, long>? freeBytes = null, Action<string>? log = null, int capacity = 90, double pollSeconds = 0.5,
            bool takesOnly = true, bool startThread = true)
        {
            _takesOnly = takesOnly;
            _controlPath = controlPath ?? throw new ArgumentNullException(nameof(controlPath));
            _now = now ?? throw new ArgumentNullException(nameof(now));
            _ticksPerSecond = ticksPerSecond;
            _minFreeBytes = minFreeBytes;
            _freeBytes = freeBytes ?? (d => new DriveInfo(Path.GetFullPath(d)).AvailableFreeSpace);
            _log = log ?? (_ => { });
            _capacity = capacity;
            _pollTicks = (long)(pollSeconds * ticksPerSecond);
            if (startThread)
            {
                _thread = new Thread(Run) { IsBackground = true, Name = "camera-session" };
                _thread.Start();
            }
        }

        /// <summary>Whether a session is being recorded now.</summary>
        public bool Active => _active;

        /// <summary>Frames dropped from recordings since start, all sessions.</summary>
        public long DroppedFrames => Interlocked.Read(ref _droppedFrames);

        /// <summary>
        /// One captured frame, from the capture loop: copied into the queue, never waiting on the disk. A frame without a
        /// capture stamp on the robot's clock (ADR 0014 §4) is not recorded, since nothing could join it.
        /// </summary>
        public void OfferFrame(ReadOnlySpan<byte> jpeg, uint frameId, long captureTicks, ushort width, ushort height)
        {
            if (!_active || !_framesWanted)
            {
                return;
            }

            long now = _now();
            if (captureTicks == LatencyTrace.Unset)
            {
                Enqueue(Line("FD", now, frameId, "unstamped"));
                return;
            }

            if (Interlocked.Increment(ref _queued) > _capacity)
            {
                Interlocked.Decrement(ref _queued);
                Interlocked.Increment(ref _droppedFrames);
                Enqueue(Line("FD", now, frameId, "writer-behind"));
                return;
            }

            byte[] copy = ArrayPool<byte>.Shared.Rent(jpeg.Length);
            jpeg.CopyTo(copy);
            _queue.Enqueue(new Item(null, copy, jpeg.Length, now, frameId, captureTicks, width, height));
        }

        private void Enqueue(string record) => _queue.Enqueue(new Item(record, null, 0, 0, 0, 0, 0, 0));

        /// <summary>One turn of the writer: polls the control file when due, then writes what is queued.</summary>
        public void Step()
        {
            long now = _now();
            if (now >= _nextPoll)
            {
                _nextPoll = now + _pollTicks;
                Poll();
            }

            Drain();
        }

        private void Run()
        {
            while (!_stop)
            {
                try
                {
                    Step();
                }
                catch (Exception e)
                {
                    _log($"[record] {e.Message}");
                }

                Thread.Sleep(20);
            }
        }

        private string? Target()
        {
            try
            {
                string text = File.ReadAllText(_controlPath).Trim();
                return text.Length == 0 ? null : text;
            }
            catch (FileNotFoundException)
            {
                return null;
            }
            catch (DirectoryNotFoundException)
            {
                return null;
            }
        }

        private void Poll()
        {
            string? target;
            try
            {
                target = Target();
            }
            catch (IOException)
            {
                return; // unreadable for a moment: keep what we have
            }

            if (target == null)
            {
                _failedDirectory = null;
            }

            if (target != _session?.Directory)
            {
                Finish(string.Empty);
                if (target != null && target != _failedDirectory)
                {
                    try
                    {
                        Open(target);
                    }
                    catch (Exception e) when (e is IOException or UnauthorizedAccessException)
                    {
                        _log($"[record] cannot record into {target}: {e.Message}");
                        _failedDirectory = target;
                    }
                }
            }
            else if (_session != null && _freeBytes(_session.Directory) < _minFreeBytes)
            {
                string directory = _session.Directory;
                Finish($"disk below {_minFreeBytes / (1024.0 * 1024 * 1024):0.#} GB free");
                _failedDirectory = directory;
            }
        }

        private void Open(string directory)
        {
            if (_freeBytes(directory) < _minFreeBytes)
            {
                throw new IOException($"less than {_minFreeBytes / (1024.0 * 1024 * 1024):0.#} GB free");
            }

            bool frames = !_takesOnly || IsTake(directory);
            System.IO.Directory.CreateDirectory(frames ? Path.Combine(directory, Folder) : directory);
            FileStream? log = null;
            string suffix = string.Empty;
            for (int n = 1; log == null; n++)
            {
                suffix = n == 1 ? string.Empty : "." + n.ToString(CultureInfo.InvariantCulture);
                try
                {
                    log = new FileStream(Path.Combine(directory, Writer + suffix + ".jlog"), FileMode.CreateNew, FileAccess.Write);
                }
                catch (IOException) when (File.Exists(Path.Combine(directory, Writer + suffix + ".jlog")) && n < 999)
                {
                }
            }

            if (suffix.Length > 0)
            {
                _log($"[record] {directory}: {Writer}.jlog exists already (CameraHost restarted in this session?); continuing in {Writer}{suffix}.jlog");
            }

            FileStream? sidecar = null;
            Span<byte> head = stackalloc byte[8];
            if (frames)
            {
                try
                {
                    sidecar = new FileStream(Path.Combine(directory, Folder, Stream + suffix + ".frames"), FileMode.CreateNew, FileAccess.Write);
                    "TFRM"u8.CopyTo(head);
                    BinaryPrimitives.WriteUInt16LittleEndian(head.Slice(4), 1);
                    BinaryPrimitives.WriteUInt16LittleEndian(head.Slice(6), 0);
                    sidecar.Write(head);
                }
                catch
                {
                    sidecar?.Dispose();
                    log.Dispose(); // an empty log, never a header without its trailer
                    File.Delete(Path.Combine(directory, Writer + suffix + ".jlog"));
                    throw;
                }
            }

            var session = new Session(directory, suffix, log, sidecar);
            if (sidecar != null)
            {
                session.SidecarHash.AppendData(head);
                session.SidecarBytes = head.Length;
            }

            _session = session;
            string id = Path.GetFileName(directory.TrimEnd('/', '\\'));
            WriteLine(session, string.Join("|", "JSES", "1", Writer, _ticksPerSecond.ToString(CultureInfo.InvariantCulture), Clock, id), count: false);
            while (_queue.TryDequeue(out Item stale))
            {
                Release(stale); // frames queued before the session opened belong to no session
            }

            _framesWanted = frames;
            _active = true;
            _log(frames ? $"[record] recording the camera into {directory}" : $"[record] {directory} is not a take: its camera.jlog gets no frames");
        }

        /// <summary>Whether the session is a take from the headset's RECORD button: its session.json has a scenario.</summary>
        private static bool IsTake(string directory)
        {
            try
            {
                using JsonDocument doc = JsonDocument.Parse(File.ReadAllBytes(Path.Combine(directory, "session.json")));
                return doc.RootElement.TryGetProperty("scenario", out JsonElement scenario) && scenario.ValueKind == JsonValueKind.Object;
            }
            catch (Exception e) when (e is IOException or JsonException or UnauthorizedAccessException)
            {
                return false;
            }
        }

        private void Drain()
        {
            while (_queue.TryDequeue(out Item item))
            {
                try
                {
                    if (_session != null)
                    {
                        Write(_session, item);
                    }
                }
                catch (IOException e)
                {
                    string directory = _session!.Directory;
                    _log($"[record] write failed in {directory}: {e.Message}; closing the session");
                    Finish("i/o error");
                    _failedDirectory = directory;
                }
                finally
                {
                    Release(item);
                }
            }
        }

        private void Release(Item item)
        {
            if (item.Frame != null)
            {
                Interlocked.Decrement(ref _queued);
                ArrayPool<byte>.Shared.Return(item.Frame);
            }
        }

        private void Write(Session s, Item item)
        {
            if (item.Record != null)
            {
                if (item.Record.StartsWith("FD|", StringComparison.Ordinal))
                {
                    s.Dropped++;
                }

                WriteLine(s, item.Record);
                return;
            }

            if (s.Sidecar != null && s.Size != (item.Width, item.Height))
            {
                s.Size = (item.Width, item.Height);
                WriteLine(s, Line("CM", Camera, item.Width, item.Height));
            }

            if (s.Sidecar == null)
            {
                return; // queued before this session opened, for one that wanted frames
            }

            Span<byte> prefix = stackalloc byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(prefix, (uint)item.Length);
            s.Sidecar.Write(prefix);
            long offset = s.SidecarBytes + 4;
            s.Sidecar.Write(item.Frame!, 0, item.Length);
            s.SidecarHash.AppendData(prefix);
            s.SidecarHash.AppendData(item.Frame!, 0, item.Length);
            s.SidecarBytes = offset + item.Length;
            WriteLine(s, Line("VF", item.QueuedTicks, item.FrameId, item.CaptureTicks, string.Empty, $"{Stream}:{offset}:{item.Length}"));
            s.Frames++;
        }

        private static void WriteLine(Session s, string line, bool count = true)
        {
            byte[] data = Encoding.ASCII.GetBytes(line + "\n");
            s.LogHash.AppendData(data);
            s.Log.Write(data);
            if (count)
            {
                s.Records++;
            }
        }

        /// <summary>The session's FS line and trailer; the session is let go either way.</summary>
        private void Finish(string reason)
        {
            Session? s = _session;
            if (s == null)
            {
                return;
            }

            _active = false;
            _session = null;
            try
            {
                while (_queue.TryDequeue(out Item item))
                {
                    try
                    {
                        Write(s, item);
                    }
                    finally
                    {
                        Release(item);
                    }
                }

                if (reason.Length > 0)
                {
                    WriteLine(s, Line("ER", _now(), reason));
                }

                if (s.Sidecar != null)
                {
                    s.Sidecar.Flush(true);
                    s.Sidecar.Dispose();
                    string sha = Convert.ToHexString(s.SidecarHash.GetHashAndReset()).ToLowerInvariant();
                    WriteLine(s, Line("FS", $"{Folder}/{Stream}{s.Suffix}.frames", s.SidecarBytes, sha));
                }

                string logSha = Convert.ToHexString(s.LogHash.GetHashAndReset()).ToLowerInvariant();
                s.Log.Write(Encoding.ASCII.GetBytes($"END|{s.Records}|{s.Dropped}|{logSha}\n"));
                s.Log.Flush(true);
                _log($"[record] closed {s.Directory}: {s.Frames} frames, {s.Dropped} not recorded{(reason.Length > 0 ? ", " + reason : string.Empty)}");
            }
            catch (IOException e)
            {
                _log($"[record] {s.Directory}: the trailer was not written ({e.Message})");
            }
            finally
            {
                s.Sidecar?.Dispose();
                s.Log.Dispose();
                s.LogHash.Dispose();
                s.SidecarHash.Dispose();
            }
        }

        private static string Line(string tag, params object[] fields)
        {
            var b = new StringBuilder(tag);
            foreach (object f in fields)
            {
                b.Append('|').Append(Convert.ToString(f, CultureInfo.InvariantCulture));
            }

            return b.ToString();
        }

        /// <summary>Stops the writer thread and closes the open session with its trailer.</summary>
        public void Dispose()
        {
            _stop = true;
            _thread?.Join(TimeSpan.FromSeconds(10));
            Drain();
            Finish(string.Empty);
        }
    }
}
