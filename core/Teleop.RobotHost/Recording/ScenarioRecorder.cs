using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Recording
{
    /// <summary>What <see cref="ScenarioRecorder"/> needs of the scenarios folder; <see cref="ScenarioFolder"/> on disk, a fake in tests.</summary>
    public interface IScenarioFiles
    {
        /// <summary>The folder's <c>scenarios.txt</c>, or null when there is none.</summary>
        string? ReadList();

        /// <summary>When <c>scenarios.txt</c> last changed (null: no file), so it is read again only then.</summary>
        DateTime? ListStamp();

        /// <summary>The takes kept of <paramref name="scenario"/>, and the highest take number in its folder or set aside (0: none).</summary>
        (int Kept, int Highest) Takes(string scenario);

        /// <summary>Whether a take is complete (its <c>files.sha256</c> is written), so it may be moved.</summary>
        bool IsComplete(string directory);

        /// <summary>Moves a take into <paramref name="aside"/>; false when it could not.</summary>
        bool MoveAside(string directory, string aside);
    }

    /// <summary>
    /// Takes of named scenarios from the headset's RECORD button (docs/adr/0033). Each take is a session of its own, opened
    /// and completed as an automatic one is (<see cref="SessionRecorder"/>, the Isaac renderer's frames included), but in
    /// the scenarios folder: <c>&lt;root&gt;/&lt;scenario&gt;/&lt;UTC&gt;-take&lt;n&gt;</c>, its <c>session.json</c> naming the
    /// scenario. While a take records, the automatic recorder is held off (<see cref="AutoSessionRecorder.Held"/>).
    ///
    /// <para><b>A take stops</b> when the headset asks; by itself when the headset that started it stops sending for
    /// <see cref="HeadsetLostSeconds"/>, after <see cref="MaxMinutes"/>, or below <see cref="StopFreeGigabytes"/> of free disk;
    /// or when the writer closed it (an I/O error). It starts only with <see cref="StartFreeGigabytes"/> free: the renderer
    /// stops writing frames below 20 GB.</para>
    ///
    /// <para><b>Discard</b> moves the last take this process recorded into <c>&lt;root&gt;/_discarded/&lt;scenario&gt;/</c>, once
    /// it is complete. Nothing is ever deleted.</para>
    ///
    /// <para>Called from RobotHost's main loop. It reads <c>scenarios.txt</c> only when it changed (checked every 2 s), and
    /// touches the folder otherwise only when a take starts, stops or is set aside.</para>
    /// </summary>
    public sealed class ScenarioRecorder
    {
        public const string DiscardedFolder = "_discarded";
        private const double GB = 1024.0 * 1024.0 * 1024.0;
        private const double ListCheckSeconds = 2.0;
        private const double ListSendSeconds = 3.0;
        private const double DiskCheckSeconds = 2.0;

        private readonly string _root;
        private readonly IAutoSessionSink _sink;
        private readonly AutoSessionRecorder? _auto;
        private readonly IScenarioFiles _files;
        private readonly long _tps;
        private readonly Func<DateTime> _utcNow;
        private readonly Func<long?> _freeBytes;
        private readonly Func<UplinkCondition> _uplink;
        private readonly Action<string> _log;
        private readonly byte[] _datagram = new byte[JointDatagramBytes];
        private const int JointDatagramBytes = 256;

        private IReadOnlyList<Scenario> _list = Array.Empty<Scenario>();
        private string[] _names = Array.Empty<string>();
        private ushort _revision;
        private DateTime? _listStamp;
        private bool _listRead;
        private long _listCheckedTicks = long.MinValue;
        private long _listSentTicks = long.MinValue;
        private ushort _listSentRevision;

        private int _generation;
        private long _startedTicks;
        private int _scenario = RecordStatus.NoScenario;
        private int _take;
        private int _kept;
        private string? _takeSender;
        private long _senderSeenTicks;
        private string? _lastTake;
        private string? _lastTakeScenario;

        private uint _sequence;
        private RecordOutcome _outcome;
        private long? _free;
        private long _diskCheckedTicks = long.MinValue;
        private RecordStatus _sent;
        private long _statusSentTicks = long.MinValue;

        public ScenarioRecorder(string root, IAutoSessionSink sink, AutoSessionRecorder? auto, IScenarioFiles files, long ticksPerSecond,
            Func<DateTime> utcNow, Func<long?> freeBytes, Func<UplinkCondition> uplink, Action<string>? log = null,
            ushort? firstRevision = null)
        {
            // Each process starts its list revisions somewhere new. Starting at 0, the first list after a restart was always
            // revision 1, the same number the headset held for the old process's list, so it kept the old names and called
            // them current: a take could be labelled with the wrong scenario (the 2026-11 bug hunt, E-02).
            _revision = firstRevision ?? (ushort)System.Security.Cryptography.RandomNumberGenerator.GetInt32(1, ushort.MaxValue);
            _root = Path.GetFullPath(root ?? throw new ArgumentNullException(nameof(root)));
            _sink = sink ?? throw new ArgumentNullException(nameof(sink));
            _auto = auto;
            _files = files ?? throw new ArgumentNullException(nameof(files));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
            _utcNow = utcNow ?? throw new ArgumentNullException(nameof(utcNow));
            _freeBytes = freeBytes ?? throw new ArgumentNullException(nameof(freeBytes));
            _uplink = uplink ?? throw new ArgumentNullException(nameof(uplink));
            _log = log ?? (_ => { });
        }

        public double MaxMinutes { get; set; } = 30;
        public double HeadsetLostSeconds { get; set; } = 10;
        public double StartFreeGigabytes { get; set; } = 25;
        public double StopFreeGigabytes { get; set; } = 21;

        public bool IsRecording => _generation != 0;

        /// <summary>The scenarios as the headset sees them.</summary>
        public IReadOnlyList<Scenario> Scenarios => _list;

        public ushort ListRevision => _revision;

        /// <summary>The last take this process recorded (its directory), or null.</summary>
        public string? LastTake => _lastTake;

        /// <summary>A request from the headset at <paramref name="sender"/>; one already answered (the same sequence) is ignored.</summary>
        public void HandleRequest(RecordOp op, byte scenario, ushort listRevision, uint sequence, string sender, long now)
        {
            if (sequence == 0 || sequence == _sequence) return;
            _sequence = sequence;
            ReadListIfChanged(now, force: false);
            _outcome = op switch
            {
                RecordOp.Start => Start(scenario, listRevision, sender, now),
                RecordOp.Stop => _generation == 0 ? RecordOutcome.Busy : StopTake("stopped from the headset", RecordOutcome.Saved),
                RecordOp.DiscardLast => Discard(),
                _ => RecordOutcome.None,
            };
        }

        /// <summary>The headset at <paramref name="sender"/> is still there (any datagram from it: heartbeat, command, keepalive).</summary>
        public void OnHeard(string sender, long now)
        {
            if (_generation != 0 && string.Equals(sender, _takeSender, StringComparison.Ordinal)) _senderSeenTicks = now;
        }

        /// <summary>Every main-loop iteration: the list, and the take's stops.</summary>
        public void Tick(long now)
        {
            ReadListIfChanged(now, force: false);
            if (_generation == 0) return;
            if (!_sink.IsSessionActive(_generation))
            {
                _log("[scenario] the take was closed by the writer (a manual session took over, or a write failed)");
                Finish(RecordOutcome.Failed);
                return;
            }

            if (now - _senderSeenTicks > (long)(HeadsetLostSeconds * _tps))
            {
                StopTake($"the headset stopped sending for {HeadsetLostSeconds:0.#} s", RecordOutcome.StoppedByItself);
                return;
            }

            if (now - _startedTicks > (long)(MaxMinutes * 60 * _tps))
            {
                StopTake($"the take reached {MaxMinutes:0.#} min", RecordOutcome.StoppedByItself);
                return;
            }

            long? free = FreeBytes(now);
            if (free.HasValue && free.Value < StopFreeGigabytes * GB)
            {
                StopTake(FormattableString.Invariant($"only {free.Value / GB:0.0} GB free"), RecordOutcome.LowDisk);
            }
        }

        public RecordStatus Status(long now)
        {
            long? free = FreeBytes(now);
            uint elapsed = _generation != 0 ? (uint)Math.Min(uint.MaxValue, (now - _startedTicks) * 1000 / _tps) : 0;
            return new RecordStatus(_generation != 0 ? RecordState.Recording : RecordState.Idle, _outcome, _sequence,
                (byte)_scenario, (byte)_names.Length, _revision, elapsed, (ushort)Math.Min(ushort.MaxValue, _take),
                (ushort)Math.Min(ushort.MaxValue, _kept), free.HasValue ? (ushort)Math.Min(ushort.MaxValue, free.Value / GB) : ushort.MaxValue);
        }

        /// <summary>The status (once a second and on change, the elapsed time aside) and the list (every 3 s and on change).</summary>
        public void Send(long now, Action<byte[], int> send)
        {
            RecordStatus status = Status(now);
            var compare = new RecordStatus(status.State, status.Outcome, status.Sequence, status.Scenario, status.ScenarioCount, status.ListRevision,
                _sent.ElapsedMs, status.Take, status.TakesKept, _sent.FreeGigabytes);
            if (_statusSentTicks == long.MinValue || now - _statusSentTicks >= _tps || !compare.Equals(_sent))
            {
                if (RecordCodec.TryEncodeStatus(status, now, _datagram, out int bytes)) send(_datagram, bytes);
                _sent = status;
                _statusSentTicks = now;
            }

            if (_listSentTicks == long.MinValue || now - _listSentTicks >= (long)(ListSendSeconds * _tps) || _listSentRevision != _revision)
            {
                int parts = Math.Max(1, (_names.Length + RecordCodec.NamesPerPart - 1) / RecordCodec.NamesPerPart);
                for (int p = 0; p < parts; p++)
                {
                    int first = p * RecordCodec.NamesPerPart, count = Math.Min(RecordCodec.NamesPerPart, _names.Length - first);
                    if (RecordCodec.TryEncodeList(_revision, (byte)p, (byte)parts, _names, first, count, _datagram, out int bytes)) send(_datagram, bytes);
                }

                _listSentTicks = now;
                _listSentRevision = _revision;
            }
        }

        /// <summary>Closes a take that is still going, as RobotHost stops.</summary>
        public void Stop(string reason)
        {
            if (_generation != 0) StopTake(reason, RecordOutcome.Saved);
        }

        private RecordOutcome Start(byte scenario, ushort listRevision, string sender, long now)
        {
            if (_generation != 0) return RecordOutcome.Busy;
            if (listRevision != _revision || scenario >= _list.Count) return RecordOutcome.UnknownScenario;
            if (_sink.ControlFileBusy) return RecordOutcome.Busy;
            long? free = FreeBytes(now, fresh: true);
            if (free.HasValue && free.Value < StartFreeGigabytes * GB)
            {
                _log(FormattableString.Invariant($"[scenario] not starting a take: only {free.Value / GB:0.0} GB free, below {StartFreeGigabytes} GB"));
                return RecordOutcome.LowDisk;
            }

            Scenario s = _list[scenario];
            (int kept, int highest) = _files.Takes(s.Name);
            int take = highest + 1;
            DateTime started = _utcNow();
            string id = started.ToUniversalTime().ToString("yyyyMMdd'T'HHmmss'Z'", CultureInfo.InvariantCulture) + "-take" + take.ToString("000", CultureInfo.InvariantCulture);
            string directory = Path.Combine(_root, s.Name, id);
            if (_auto != null) _auto.Held = true;
            var spec = new AutoSessionSpec(started, sender, _uplink(), "scenario take") { Take = new ScenarioTake(s.Name, s.Description, take, directory) };
            int generation = _sink.BeginAuto(spec);
            if (generation == 0)
            {
                if (_auto != null) _auto.Held = false;
                return RecordOutcome.Failed;
            }

            _generation = generation;
            _startedTicks = now;
            _scenario = scenario;
            _take = take;
            _kept = kept;
            _takeSender = sender;
            _senderSeenTicks = now;
            _lastTake = directory;
            _lastTakeScenario = s.Name;
            _log($"[scenario] recording '{s.Name}' take {take} (from {sender})");
            return RecordOutcome.Started;
        }

        private RecordOutcome StopTake(string reason, RecordOutcome outcome)
        {
            _sink.EndAuto(_generation, reason);
            _log($"[scenario] take {_take} of '{_lastTakeScenario}' stopped: {reason}");
            Finish(outcome);
            _kept++;
            return outcome;
        }

        private void Finish(RecordOutcome outcome)
        {
            _generation = 0;
            _takeSender = null;
            if (_auto != null) _auto.Held = false;
            _outcome = outcome;
        }

        private RecordOutcome Discard()
        {
            if (_generation != 0) return RecordOutcome.Busy;
            if (_lastTake == null || _lastTakeScenario == null) return RecordOutcome.NothingToDiscard;
            if (!_files.IsComplete(_lastTake)) return RecordOutcome.Busy; // still being completed: try again in a moment
            string aside = Path.Combine(_root, DiscardedFolder, _lastTakeScenario, Path.GetFileName(_lastTake));
            if (!_files.MoveAside(_lastTake, aside)) return RecordOutcome.Failed;
            _log($"[scenario] set aside {_lastTake} into {aside}");
            _lastTake = null;
            _kept = _files.Takes(_lastTakeScenario).Kept;
            return RecordOutcome.Discarded;
        }

        private void ReadListIfChanged(long now, bool force)
        {
            if (!force && _listRead && _listCheckedTicks != long.MinValue && now - _listCheckedTicks < (long)(ListCheckSeconds * _tps)) return;
            _listCheckedTicks = now;
            DateTime? stamp = _files.ListStamp();
            if (_listRead && stamp == _listStamp) return;
            _listStamp = stamp;
            _listRead = true;
            var problems = new List<string>();
            IReadOnlyList<Scenario> list = ScenarioList.Parse(_files.ReadList(), problems);
            foreach (string p in problems) _log($"[scenario] {ScenarioList.FileName}: {p}");
            bool same = list.Count == _list.Count;
            for (int i = 0; same && i < list.Count; i++) same = list[i].Name == _list[i].Name;
            _list = list;
            if (same && _names.Length > 0) return;
            var names = new string[list.Count];
            for (int i = 0; i < names.Length; i++) names[i] = list[i].Name;
            _names = names;
            _revision = (ushort)(_revision == ushort.MaxValue ? 1 : _revision + 1);
            _log($"[scenario] {names.Length} scenario(s) from {Path.Combine(_root, ScenarioList.FileName)}: {string.Join(", ", names)}");
        }

        private long? FreeBytes(long now, bool fresh = false)
        {
            if (fresh || _diskCheckedTicks == long.MinValue || now - _diskCheckedTicks >= (long)(DiskCheckSeconds * _tps))
            {
                _free = _freeBytes();
                _diskCheckedTicks = now;
            }

            return _free;
        }
    }

    /// <summary>The scenarios folder on disk (docs/adr/0033).</summary>
    public sealed class ScenarioFolder : IScenarioFiles
    {
        private readonly string _root;

        public ScenarioFolder(string root)
        {
            _root = Path.GetFullPath(root ?? throw new ArgumentNullException(nameof(root)));
        }

        private string ListPath => Path.Combine(_root, ScenarioList.FileName);

        /// <summary>Makes the folder, and a <c>scenarios.txt</c> from <see cref="ScenarioList.Template"/> when there is none.</summary>
        public void EnsureExists()
        {
            Directory.CreateDirectory(_root);
            if (!File.Exists(ListPath)) File.WriteAllText(ListPath, ScenarioList.Template);
        }

        public string? ReadList()
        {
            try
            {
                return File.Exists(ListPath) ? File.ReadAllText(ListPath) : null;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                return null;
            }
        }

        public DateTime? ListStamp()
        {
            try
            {
                return File.Exists(ListPath) ? File.GetLastWriteTimeUtc(ListPath) : (DateTime?)null;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                return null;
            }
        }

        public (int Kept, int Highest) Takes(string scenario)
        {
            int kept = 0, highest = 0;
            foreach (string dir in new[] { Path.Combine(_root, scenario), Path.Combine(_root, ScenarioRecorder.DiscardedFolder, scenario) })
            {
                if (!Directory.Exists(dir)) continue;
                bool keptFolder = dir == Path.Combine(_root, scenario);
                foreach (string take in Directory.GetDirectories(dir))
                {
                    int n = TakeNumber(Path.GetFileName(take));
                    if (n <= 0) continue;
                    if (keptFolder) kept++;
                    highest = Math.Max(highest, n);
                }
            }

            return (kept, highest);
        }

        /// <summary>The take number of a take folder's name (<c>...-take007</c> is 7), or 0.</summary>
        public static int TakeNumber(string name)
        {
            int at = name.LastIndexOf("-take", StringComparison.Ordinal);
            return at >= 0 && int.TryParse(name.Substring(at + 5), NumberStyles.None, CultureInfo.InvariantCulture, out int n) ? n : 0;
        }

        public bool IsComplete(string directory) => File.Exists(Path.Combine(directory, "files.sha256"));

        public bool MoveAside(string directory, string aside)
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(aside)!);
                Directory.Move(directory, aside);
                return true;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                return false;
            }
        }
    }
}
