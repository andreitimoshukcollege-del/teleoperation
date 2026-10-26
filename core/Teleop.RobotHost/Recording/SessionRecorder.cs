using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json.Nodes;
using System.Threading;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// RobotHost's half of a recorded session (docs/adr/0021 decision 3): <c>robothost.jlog</c> in
    /// whichever session directory the control file names (<c>just record-start</c> writes it,
    /// <c>just record-stop</c> removes it), so recording starts and stops without a restart. It also
    /// opens sessions of its own when <see cref="AutoSessionRecorder"/> asks (docs/adr/0023 decision 4):
    /// it then writes <c>session.json</c>, <c>robot-profile.json</c>, the marker
    /// <c>.active-auto</c> and the control file itself, so the driver follows as it does for a manual
    /// session, and completes the session when it closes (<see cref="SessionFinalizer"/>).
    ///
    /// <para><b>It must not change the timing it records.</b> The main loop only calls
    /// <see cref="Record"/> (and, for automatic sessions, <see cref="BeginAuto"/> and
    /// <see cref="EndAuto"/>), which enqueue and never wait: when the queue is full a line is dropped
    /// and counted. A background thread polls the control file, writes, and keeps the SHA-256 the
    /// trailer carries. A session with any drop fails <c>just recording-check</c>.</para>
    ///
    /// <para><b>Safety of what is already on disk:</b> a log file that exists is never truncated.
    /// When RobotHost restarts during a manual session, the control file still names it and
    /// <c>robothost.jlog</c> is already there (cut off, without a trailer): this writer continues in
    /// <c>robothost.2.jlog</c> (then <c>.3</c>, ...), a complete file of its own, and leaves the first as
    /// it was. An automatic session left open by an earlier process is closed and completed instead
    /// (its control file cleared), and the next command opens a new one. An I/O error (a full disk)
    /// closes the session, with its trailer if that can still be written; it never kills RobotHost.</para>
    ///
    /// <para>The file is the trainer's session format v1 (trainer/contract/SESSION_FORMAT.md): a
    /// <c>JSES</c> header, the records, and an <c>END|records|dropped|sha256</c> trailer, which is
    /// written when recording stops; a file without one was cut off.</para>
    /// </summary>
    public sealed class SessionRecorder : IDisposable, IAutoSessionSink
    {
        public const int DefaultCapacity = 200_000;

        /// <summary>
        /// Beside the control file: while it holds the same directory as the control file, the session
        /// is automatic (RobotHost opened it, and closes and completes it). Anything else is manual.
        /// </summary>
        public const string AutoMarkerName = ".active-auto";

        public const string AutoLabel = "auto";
        private const string Writer = "robothost";
        private const string Clock = "CLOCK_MONOTONIC";

        private readonly string _controlPath;
        private readonly string _markerPath;
        private readonly long _ticksPerSecond;
        private readonly int _capacity;
        private readonly TimeSpan _poll;
        private readonly AutoRecordOptions? _auto;
        private readonly Func<DateTime> _utcNow;
        private readonly Action<string> _log;
        private readonly ConcurrentQueue<Item> _queue = new ConcurrentQueue<Item>();
        private readonly AutoResetEvent _signal = new AutoResetEvent(false);
        private readonly SessionFinalizer _finalizer;
        private readonly Thread _thread;
        private readonly object _gate = new object();
        private volatile bool _stop;

        // Shared with the main loop, changed under _gate.
        private volatile bool _active;
        private volatile int _activeGeneration;
        private int _generation;
        private bool _preambleOwed;
        private volatile bool _controlBusy;
        private int _queued;
        private long _dropped;

        // The writer thread's own.
        private readonly Func<string, Stream> _openFile;
        private Stream? _file;
        private IncrementalHash? _hash;
        private volatile string? _directory;
        private int _fileGeneration;
        private bool _fileAuto;
        private long _records;
        private string? _failedDirectory;
        private int _manualGeneration;

        private readonly struct Item
        {
            public Item(int generation, string? line, AutoSessionSpec? open, string? closeReason)
            {
                Generation = generation;
                Line = line;
                Open = open;
                CloseReason = closeReason;
            }

            public int Generation { get; }
            public string? Line { get; }
            public AutoSessionSpec? Open { get; }
            public string? CloseReason { get; }
        }

        public SessionRecorder(string controlPath, long ticksPerSecond, int capacity = DefaultCapacity, double pollSeconds = 0.5,
            AutoRecordOptions? auto = null, Func<DateTime>? utcNow = null, Action<string>? log = null)
            : this(controlPath, ticksPerSecond, capacity, pollSeconds, auto, utcNow, log, openFile: null)
        {
        }

        /// <param name="openFile">Tests only: creates a log file (it must fail if the file exists), so a test can make writes fail as a full disk does.</param>
        internal SessionRecorder(string controlPath, long ticksPerSecond, int capacity, double pollSeconds,
            AutoRecordOptions? auto, Func<DateTime>? utcNow, Action<string>? log, Func<string, Stream>? openFile)
        {
            _openFile = openFile ?? (path => new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read));
            _controlPath = controlPath ?? throw new ArgumentNullException(nameof(controlPath));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));
            _markerPath = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(controlPath)) ?? ".", AutoMarkerName);
            _ticksPerSecond = ticksPerSecond;
            _capacity = capacity;
            _poll = TimeSpan.FromSeconds(pollSeconds);
            _auto = auto;
            _utcNow = utcNow ?? (() => DateTime.UtcNow);
            _log = log ?? Console.WriteLine;
            _finalizer = new SessionFinalizer(auto?.DriverWaitSeconds ?? new AutoRecordOptions().DriverWaitSeconds, _utcNow, _log);
            _thread = new Thread(Run) { IsBackground = true, Name = "session-recorder" };
            _thread.Start();
        }

        /// <summary>True while a session is open: records go to it.</summary>
        public bool IsActive => _active;

        /// <summary>Records dropped (queue full) since the current session opened.</summary>
        public long Dropped => Interlocked.Read(ref _dropped);

        /// <summary>The session directory being written, or null.</summary>
        public string? Directory => _directory;

        /// <summary>Where automatic sessions go.</summary>
        public string AutoRoot => _auto?.RecordingsRoot ??
            Path.Combine(Path.GetDirectoryName(Path.GetFullPath(_controlPath)) ?? ".", "jetrover");

        /// <summary>Automatic sessions closed and not yet completed (files.sha256 not yet written).</summary>
        public int PendingCompletions => _finalizer.Pending;

        /// <inheritdoc/>
        public bool ControlFileBusy => _controlBusy;

        /// <summary>
        /// Queues one record line (no newline). Called from the main loop: never blocks, never does
        /// I/O. Nothing is kept while no session is open.
        /// </summary>
        public void Record(string line)
        {
            if (!_active) return;
            int generation = _activeGeneration;
            if (Interlocked.Increment(ref _queued) > _capacity)
            {
                Interlocked.Decrement(ref _queued);
                Interlocked.Increment(ref _dropped);
                return;
            }

            _queue.Enqueue(new Item(generation, line, null, null));
            _signal.Set();
        }

        /// <summary>
        /// True once for every session that has just become active: the caller then records the state
        /// a self-contained file needs at its start (<c>SS</c>, <c>JS</c>, <c>UP</c>). Main loop only.
        /// </summary>
        public bool TakePreambleOwed()
        {
            lock (_gate)
            {
                if (!_preambleOwed || !_active) return false;
                _preambleOwed = false;
                return true;
            }
        }

        /// <inheritdoc/>
        public int BeginAuto(AutoSessionSpec spec)
        {
            if (spec == null) throw new ArgumentNullException(nameof(spec));
            if (_auto == null) throw new InvalidOperationException("this recorder was built without AutoRecordOptions");
            int generation;
            lock (_gate)
            {
                if (_active || _controlBusy) return 0;
                generation = ++_generation;
                _activeGeneration = generation;
                _active = true;
                _preambleOwed = true;
                Interlocked.Exchange(ref _dropped, 0);
            }

            _queue.Enqueue(new Item(generation, null, spec, null));
            _signal.Set();
            return generation;
        }

        /// <inheritdoc/>
        public bool IsSessionActive(int generation) => _active && _activeGeneration == generation;

        /// <inheritdoc/>
        public void EndAuto(int generation, string reason)
        {
            Deactivate(generation);
            _queue.Enqueue(new Item(generation, null, null, reason ?? "closed"));
            _signal.Set();
        }

        public void Dispose()
        {
            if (_stop) return;
            _stop = true;
            _signal.Set();
            _thread.Join(TimeSpan.FromSeconds(10));
            _finalizer.Dispose();
            _signal.Dispose();
        }

        private void Deactivate(int generation)
        {
            lock (_gate)
            {
                if (_activeGeneration != generation) return;
                _active = false;
                _preambleOwed = false;
            }
        }

        private void Run()
        {
            DateTime lastPoll = DateTime.MinValue;
            DateTime lastFlush = DateTime.UtcNow;
            while (!_stop)
            {
                try
                {
                    if (DateTime.UtcNow - lastPoll >= _poll)
                    {
                        lastPoll = DateTime.UtcNow;
                        Poll();
                    }

                    _signal.WaitOne(TimeSpan.FromMilliseconds(100));
                    ProcessQueue();
                    if (_file != null && DateTime.UtcNow - lastFlush >= TimeSpan.FromSeconds(1))
                    {
                        lastFlush = DateTime.UtcNow;
                        Guard(() => _file!.Flush());
                    }
                }
                catch (Exception e)
                {
                    // Nothing here may end the thread: recording would stop silently for good.
                    _log($"[record] unexpected {e.GetType().Name}: {e.Message}");
                    if (_file != null) Fail(e);
                    Thread.Sleep(100);
                }
            }

            try
            {
                ProcessQueue();
                CloseCurrent("RobotHost is stopping");
            }
            catch (Exception e)
            {
                _log($"[record] could not close the session on stop: {e.Message}");
            }
        }

        private void Poll()
        {
            if (!TryReadText(_controlPath, out string? target)) return;
            bool ours = _file != null && _fileAuto && string.Equals(target, _directory, StringComparison.Ordinal);
            _controlBusy = target != null && !ours;

            if (_file != null && !string.Equals(target, _directory, StringComparison.Ordinal))
            {
                CloseCurrent(_fileAuto
                    ? (target == null
                        ? "the control file was cleared (just record-start, record-stop or the onboard pilot)"
                        : "the control file now names " + target)
                    : "the control file changed");
            }

            if (target == null)
            {
                _failedDirectory = null;
                return;
            }

            if (_file != null || string.Equals(target, _failedDirectory, StringComparison.Ordinal)) return;

            if (string.Equals(ReadText(_markerPath), target, StringComparison.Ordinal))
            {
                // An automatic session no process is writing: an earlier RobotHost's. Close and
                // complete it; the next command opens a new one.
                _log($"[record] {target} is an automatic session an earlier RobotHost left open: closing and completing it");
                DeleteIfNames(_controlPath, target);
                DeleteIfNames(_markerPath, target);
                _controlBusy = false;
                _finalizer.Enqueue(target);
                return;
            }

            OpenManual(target);
        }

        private void OpenManual(string directory)
        {
            int generation;
            lock (_gate)
            {
                generation = ++_generation;
                _manualGeneration = generation;
            }

            try
            {
                System.IO.Directory.CreateDirectory(directory);
                OpenLog(directory, generation, auto: false);
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                _log($"[record] cannot record into {directory}: {e.Message}");
                _failedDirectory = directory;
                return;
            }

            lock (_gate)
            {
                // A manual session wins over an automatic one that was begun but not yet opened.
                _activeGeneration = generation;
                _active = true;
                _preambleOwed = true;
                Interlocked.Exchange(ref _dropped, 0);
            }
        }

        private void ProcessQueue()
        {
            while (_queue.TryDequeue(out Item item))
            {
                if (item.Line != null)
                {
                    Interlocked.Decrement(ref _queued);
                    if (_file != null && item.Generation == _fileGeneration)
                    {
                        if (Emit(item.Line)) _records++;
                    }
                }
                else if (item.Open != null)
                {
                    OpenAuto(item.Open, item.Generation);
                }
                else if (item.CloseReason != null && _file != null && item.Generation == _fileGeneration)
                {
                    CloseFile(item.CloseReason);
                }
            }
        }

        private void OpenAuto(AutoSessionSpec spec, int generation)
        {
            // A manual session opened after this was begun supersedes it. One the main loop already ended
            // is still opened: its records and its close follow it in the queue.
            if (generation < _manualGeneration) return;
            string? directory = null;
            bool wroteMarker = false;
            try
            {
                if (_file != null || ReadText(_controlPath) != null)
                {
                    Refuse(generation, "a session is already recording");
                    return;
                }

                string root = AutoRoot;
                string baseId = spec.StartedUtc.ToUniversalTime().ToString("yyyyMMdd'T'HHmmss'Z'", CultureInfo.InvariantCulture) + "-" + AutoLabel;
                string id = baseId;
                for (int n = 2; System.IO.Directory.Exists(Path.Combine(root, id)); n++) id = baseId + "-" + n.ToString(CultureInfo.InvariantCulture);
                directory = Path.Combine(root, id);
                System.IO.Directory.CreateDirectory(directory);
                WriteSessionJson(directory, id, spec);

                WriteText(_markerPath + ".tmp", directory);
                File.Move(_markerPath + ".tmp", _markerPath, overwrite: true);
                wroteMarker = true;
                WriteText(_controlPath + ".auto.tmp", directory);
                try
                {
                    // No overwrite: a manual session that started meanwhile keeps the control file.
                    File.Move(_controlPath + ".auto.tmp", _controlPath, overwrite: false);
                }
                catch (IOException)
                {
                    File.Delete(_controlPath + ".auto.tmp");
                    throw new IOException("a manual session took the control file first");
                }

                OpenLog(directory, generation, auto: true);
                _log($"[record] automatic session {id} into {directory} ({spec.Reason})");
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is ArgumentException)
            {
                _log($"[record] cannot open an automatic session: {e.Message}");
                if (wroteMarker && directory != null) DeleteIfNames(_markerPath, directory);
                if (directory != null && _file == null)
                {
                    DeleteIfNames(_controlPath, directory);
                    TryDeleteDirectory(directory);
                }

                Deactivate(generation);
            }
        }

        private void Refuse(int generation, string why)
        {
            _log($"[record] not opening an automatic session: {why}");
            Deactivate(generation);
        }

        private void WriteSessionJson(string directory, string id, AutoSessionSpec spec)
        {
            AutoRecordOptions o = _auto!;
            var network = new JsonObject();
            if (spec.Uplink.Schedule != null)
            {
                network["uplinkProfile"] = "schedule";
                network["uplinkSchedule"] = spec.Uplink.Schedule;
                network["uplinkSeed"] = spec.Uplink.Seed;
            }
            else
            {
                bool none = spec.Uplink.Profile == "none" || spec.Uplink.Profile == "lan";
                network["uplinkProfile"] = none ? "lan" : spec.Uplink.Profile;
                network["uplinkSeed"] = none ? 0UL : spec.Uplink.Seed;
            }

            // The headset's own added delay and disturbances are not known here (docs/adr/0023).
            network["addedDelayMs"] = null;

            string? profileSha = null;
            var notes = new StringBuilder("opened automatically by RobotHost (docs/adr/0023 decision 4): " + spec.Reason +
                "; load, operator and any headset-side network disturbance are unknown");
            if (o.ProfilePath != null && File.Exists(o.ProfilePath))
            {
                byte[] profile = File.ReadAllBytes(o.ProfilePath);
                File.WriteAllBytes(Path.Combine(directory, "robot-profile.json"), profile);
                profileSha = Convert.ToHexString(SHA256.HashData(profile)).ToLowerInvariant();
            }
            else
            {
                notes.Append("; no robot profile was copied (" + (o.ProfilePath ?? "no --auto-record-profile") + ")");
            }

            var meta = new JsonObject
            {
                ["format"] = "jetrover-session",
                ["version"] = 1,
                ["id"] = id,
                ["source"] = "robot",
                ["label"] = AutoLabel,
                ["startedUtc"] = spec.StartedUtc.ToUniversalTime().ToString("yyyy-MM-dd'T'HH:mm:ss'Z'", CultureInfo.InvariantCulture),
                ["conditions"] = new JsonObject
                {
                    // Set from the commands' speeds when the session closes.
                    ["speedSliderRadPerSec"] = "unknown",
                    ["load"] = new JsonObject { ["kind"] = "unknown", ["payloadGrams"] = null },
                    ["network"] = network,
                    ["motion"] = AutoLabel,
                    ["operator"] = "auto " + spec.Sender,
                },
                ["builds"] = new JsonObject
                {
                    ["robothost"] = ReadBuildInfo(o.RobotHostBuildInfoPath ?? Path.Combine(AppContext.BaseDirectory, "BUILD_INFO")),
                    ["driver"] = ReadBuildInfo(o.DriverBuildInfoPath),
                },
                ["robotProfileSha256"] = profileSha,
                ["notes"] = notes.ToString(),
            };
            SessionFinalizer.WriteJsonAtomically(Path.Combine(directory, "session.json"), meta);
        }

        private static JsonNode ReadBuildInfo(string? path)
        {
            try
            {
                if (path != null && File.Exists(path) && JsonNode.Parse(File.ReadAllText(path)) is JsonObject info) return info;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is System.Text.Json.JsonException)
            {
            }

            return new JsonObject();
        }

        /// <summary>Opens a fresh log in <paramref name="directory"/>, never truncating one that exists.</summary>
        private void OpenLog(string directory, int generation, bool auto)
        {
            string path = Path.Combine(directory, Writer + ".jlog");
            Stream? file = null;
            for (int n = 2; file == null; n++)
            {
                try
                {
                    file = _openFile(path);
                }
                catch (IOException) when (File.Exists(path) && n < 1000)
                {
                    path = Path.Combine(directory, Writer + "." + n.ToString(CultureInfo.InvariantCulture) + ".jlog");
                }
            }

            if (Path.GetFileName(path) != Writer + ".jlog")
            {
                _log($"[record] {directory}/{Writer}.jlog already exists (RobotHost restarted during the session?): kept as it is; continuing in {Path.GetFileName(path)}");
            }

            string sessionId = Path.GetFileName(directory.TrimEnd('/', '\\'));
            byte[] header = Encoding.ASCII.GetBytes("JSES|1|" + Writer + "|" + _ticksPerSecond.ToString(CultureInfo.InvariantCulture) + "|" + Clock + "|" +
                sessionId + "\n");
            try
            {
                file.Write(header, 0, header.Length);
            }
            catch
            {
                file.Dispose();
                throw;
            }

            _file = file;
            _hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
            _hash.AppendData(header);
            _directory = directory;
            _fileGeneration = generation;
            _fileAuto = auto;
            _records = 0;

            if (!auto) _log($"[record] recording into {directory}");
        }

        /// <summary>Closes the open session from this thread: what the main loop queued for it is written first.</summary>
        private void CloseCurrent(string reason)
        {
            if (_file == null) return;
            int generation = _fileGeneration;
            Deactivate(generation);
            ProcessQueue();
            if (_file != null && _fileGeneration == generation) CloseFile(reason);
        }

        private void CloseFile(string reason)
        {
            string directory = _directory!;
            bool auto = _fileAuto;
            long records = _records;
            long dropped = Interlocked.Read(ref _dropped);
            bool closed = WriteTrailerAndRelease();
            _log($"[record] stopped ({reason}): {records} records, {dropped} dropped, in {directory}" + (closed ? "" : "; the trailer could not be written"));
            if (auto) CompleteAuto(directory);
        }

        /// <summary>
        /// The trailer, if the disk still takes it, then the file is let go. After a failed write the
        /// hash covers what this writer meant to write, so a file that lost bytes fails the trainer's
        /// check rather than passing silently.
        /// </summary>
        private bool WriteTrailerAndRelease()
        {
            Deactivate(_fileGeneration);
            string sha = Convert.ToHexString(_hash!.GetHashAndReset()).ToLowerInvariant();
            byte[] trailer = Encoding.ASCII.GetBytes("END|" + _records.ToString(CultureInfo.InvariantCulture) + "|" +
                Interlocked.Read(ref _dropped).ToString(CultureInfo.InvariantCulture) + "|" + sha + "\n");
            bool closed = Guard(() =>
            {
                _file!.Write(trailer, 0, trailer.Length);
                if (_file is FileStream onDisk) onDisk.Flush(flushToDisk: true);
                else _file.Flush();
            }, reportFailure: false);
            Release();
            return closed;
        }

        /// <summary>An I/O error: close what can be closed, and never reopen the same manual session by itself.</summary>
        private void Fail(Exception e)
        {
            if (_file == null) return;
            _log($"[record] write failed in {_directory}: {e.Message}; closing the session");
            string directory = _directory!;
            bool auto = _fileAuto;
            long records = _records;
            bool closed = WriteTrailerAndRelease();
            _log($"[record] closed after the error: {records} records" + (closed ? ", with its trailer" : ", without a trailer (the disk refused it)"));
            if (auto) CompleteAuto(directory);
            else _failedDirectory = directory;
        }

        private void Release()
        {
            try
            {
                _file?.Dispose();
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
            }

            _hash?.Dispose();
            _file = null;
            _hash = null;
            _directory = null;
            _fileAuto = false;
        }

        private void CompleteAuto(string directory)
        {
            DeleteIfNames(_controlPath, directory);
            DeleteIfNames(_markerPath, directory);
            _finalizer.Enqueue(directory);
        }

        /// <summary>Writes one line; on an I/O error closes the session (<see cref="Fail"/>) and returns false.</summary>
        private bool Emit(string line)
        {
            if (_file == null) return false;
            byte[] data = Encoding.ASCII.GetBytes(line + "\n");
            _hash!.AppendData(data);
            return Guard(() => _file!.Write(data, 0, data.Length));
        }

        private bool Guard(Action write, bool reportFailure = true)
        {
            try
            {
                write();
                return true;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is ObjectDisposedException)
            {
                if (reportFailure) Fail(e);
                return false;
            }
        }

        private static string? ReadText(string path) => TryReadText(path, out string? text) ? text : null;

        /// <summary>
        /// A control file's content (null: absent or empty). False when it could not be read this time
        /// (being replaced, say): the caller then changes nothing, rather than taking a passing error
        /// for a cleared file and closing the session.
        /// </summary>
        private static bool TryReadText(string path, out string? text)
        {
            text = null;
            try
            {
                if (!File.Exists(path)) return true;
                using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
                using var reader = new StreamReader(stream, Encoding.UTF8);
                string content = reader.ReadToEnd().Trim();
                text = content.Length == 0 ? null : content;
                return true;
            }
            catch (FileNotFoundException)
            {
                return true;
            }
            catch (DirectoryNotFoundException)
            {
                return true;
            }
            catch (IOException)
            {
                return false;
            }
            catch (UnauthorizedAccessException)
            {
                return false;
            }
        }

        private static void WriteText(string path, string text) => File.WriteAllText(path, text + "\n", new UTF8Encoding(false));

        /// <summary>Deletes a control file only while it still names <paramref name="directory"/>: never someone else's.</summary>
        private void DeleteIfNames(string path, string directory)
        {
            try
            {
                if (string.Equals(ReadText(path), directory, StringComparison.Ordinal)) File.Delete(path);
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                _log($"[record] cannot remove {path}: {e.Message}");
            }
        }

        private static void TryDeleteDirectory(string directory)
        {
            try
            {
                if (System.IO.Directory.Exists(directory)) System.IO.Directory.Delete(directory, recursive: true);
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
            }
        }
    }
}
