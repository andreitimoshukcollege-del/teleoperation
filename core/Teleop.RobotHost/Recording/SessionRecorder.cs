using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// RobotHost's half of a recorded session (docs/adr/0021 decision 3): <c>robothost.jlog</c> in
    /// whichever session directory the control file names (<c>just record-start</c> writes it,
    /// <c>just record-stop</c> removes it), so recording starts and stops without a restart.
    ///
    /// <para><b>It must not change the timing it records.</b> The main loop only calls
    /// <see cref="Record"/>, which enqueues a line on a bounded queue and, when the queue is full,
    /// drops and counts it rather than waiting. A background thread polls the control file, writes,
    /// and keeps the SHA-256 the trailer carries. A session with any drop fails
    /// <c>just recording-check</c>.</para>
    ///
    /// <para>The file is the trainer's session format v1 (trainer/contract/SESSION_FORMAT.md): a
    /// <c>JSES</c> header, the records, and an <c>END|records|dropped|sha256</c> trailer, which is
    /// written when recording stops; a file without one was cut off.</para>
    /// </summary>
    public sealed class SessionRecorder : IDisposable
    {
        public const int DefaultCapacity = 200_000;
        private const string Writer = "robothost";
        private const string Clock = "CLOCK_MONOTONIC";

        private readonly string _controlPath;
        private readonly long _ticksPerSecond;
        private readonly int _capacity;
        private readonly TimeSpan _poll;
        private readonly ConcurrentQueue<string> _queue = new ConcurrentQueue<string>();
        private readonly AutoResetEvent _signal = new AutoResetEvent(false);
        private readonly Thread _thread;
        private volatile bool _stop;
        private volatile bool _active;
        private int _queued;
        private long _dropped;
        private FileStream? _file;
        private IncrementalHash? _hash;
        private string? _directory;
        private long _records;

        public SessionRecorder(string controlPath, long ticksPerSecond, int capacity = DefaultCapacity, double pollSeconds = 0.5)
        {
            _controlPath = controlPath ?? throw new ArgumentNullException(nameof(controlPath));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));
            _ticksPerSecond = ticksPerSecond;
            _capacity = capacity;
            _poll = TimeSpan.FromSeconds(pollSeconds);
            _thread = new Thread(Run) { IsBackground = true, Name = "session-recorder" };
            _thread.Start();
        }

        /// <summary>True while a session is open: records go to it.</summary>
        public bool IsActive => _active;

        /// <summary>Records dropped (queue full) since the current session opened.</summary>
        public long Dropped => Interlocked.Read(ref _dropped);

        /// <summary>The session directory being written, or null.</summary>
        public string? Directory => _directory;

        /// <summary>
        /// Queues one record line (no newline). Called from the main loop: never blocks, never does
        /// I/O. Nothing is kept while no session is open.
        /// </summary>
        public void Record(string line)
        {
            if (!_active) return;
            if (Interlocked.Increment(ref _queued) > _capacity)
            {
                Interlocked.Decrement(ref _queued);
                Interlocked.Increment(ref _dropped);
                return;
            }

            _queue.Enqueue(line);
            _signal.Set();
        }

        public void Dispose()
        {
            _stop = true;
            _signal.Set();
            _thread.Join(TimeSpan.FromSeconds(5));
            _signal.Dispose();
        }

        private void Run()
        {
            DateTime lastPoll = DateTime.MinValue;
            DateTime lastFlush = DateTime.UtcNow;
            while (!_stop)
            {
                if (DateTime.UtcNow - lastPoll >= _poll)
                {
                    lastPoll = DateTime.UtcNow;
                    string? target = ReadTarget();
                    if (!string.Equals(target, _directory, StringComparison.Ordinal))
                    {
                        Close();
                        if (target != null)
                        {
                            try
                            {
                                Open(target);
                            }
                            catch (IOException e)
                            {
                                Console.Error.WriteLine($"[record] cannot record into {target}: {e.Message}");
                            }
                            catch (UnauthorizedAccessException e)
                            {
                                Console.Error.WriteLine($"[record] cannot record into {target}: {e.Message}");
                            }
                        }
                    }
                }

                _signal.WaitOne(TimeSpan.FromMilliseconds(100));
                Drain();
                if (_file != null && DateTime.UtcNow - lastFlush >= TimeSpan.FromSeconds(1))
                {
                    _file.Flush();
                    lastFlush = DateTime.UtcNow;
                }
            }

            Close();
        }

        private string? ReadTarget()
        {
            try
            {
                if (!File.Exists(_controlPath)) return null;
                string text = File.ReadAllText(_controlPath).Trim();
                return text.Length == 0 ? null : text;
            }
            catch (IOException)
            {
                return null;
            }
            catch (UnauthorizedAccessException)
            {
                return null;
            }
        }

        private void Open(string directory)
        {
            System.IO.Directory.CreateDirectory(directory);
            _file = new FileStream(Path.Combine(directory, Writer + ".jlog"), FileMode.Create, FileAccess.Write, FileShare.Read);
            _hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
            _directory = directory;
            _records = 0;
            Interlocked.Exchange(ref _dropped, 0);
            string sessionId = Path.GetFileName(directory.TrimEnd('/', '\\'));
            Emit("JSES|1|" + Writer + "|" + _ticksPerSecond.ToString(CultureInfo.InvariantCulture) + "|" + Clock + "|" + sessionId);
            _active = true;
            Console.WriteLine($"[record] recording into {directory}");
        }

        private void Close()
        {
            if (_file == null || _hash == null) return;
            _active = false;
            Drain();
            string sha = Convert.ToHexString(_hash.GetHashAndReset()).ToLowerInvariant();
            byte[] trailer = Encoding.ASCII.GetBytes("END|" + _records.ToString(CultureInfo.InvariantCulture) + "|" +
                Interlocked.Read(ref _dropped).ToString(CultureInfo.InvariantCulture) + "|" + sha + "\n");
            _file.Write(trailer, 0, trailer.Length);
            _file.Flush(flushToDisk: true);
            _file.Dispose();
            _hash.Dispose();
            Console.WriteLine($"[record] stopped: {_records} records, {Interlocked.Read(ref _dropped)} dropped, in {_directory}");
            _file = null;
            _hash = null;
            _directory = null;
        }

        private void Drain()
        {
            while (_queue.TryDequeue(out string? line))
            {
                Interlocked.Decrement(ref _queued);
                if (_file == null) continue;
                Emit(line);
                _records++;
            }
        }

        private void Emit(string line)
        {
            byte[] data = Encoding.ASCII.GetBytes(line + "\n");
            _hash!.AppendData(data);
            _file!.Write(data, 0, data.Length);
        }
    }
}
