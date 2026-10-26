using System;
using System.Globalization;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// Recording whenever the robot is commanded (docs/adr/0023 decision 4): decides when RobotHost
    /// opens and closes a session of its own. Pure logic, called from the main loop with that loop's
    /// clock; the <see cref="IAutoSessionSink"/> (the <see cref="SessionRecorder"/>) does the files.
    ///
    /// <para><b>Opens</b> on the first joint command after at least <see cref="AutoRecordOptions.IdleSeconds"/>
    /// without one (or since start-up), unless the control file names a manual session (manual
    /// sessions win: the automatic recorder stays out) or the disk has less than
    /// <see cref="AutoRecordOptions.StartFreeGigabytes"/> free (logged).</para>
    ///
    /// <para><b>Closes</b> after <see cref="AutoRecordOptions.IdleSeconds"/> without a command, and also:
    /// when the command sender or the joint-state subscriber changes (the trainer's check allows one
    /// of each per session), reopening on that command or the next; after
    /// <see cref="AutoRecordOptions.MaxMinutes"/> (rotation), reopening on the next command; and when
    /// free space falls below <see cref="AutoRecordOptions.StopFreeGigabytes"/>, reopening on a later
    /// command once there is <see cref="AutoRecordOptions.StartFreeGigabytes"/> again.</para>
    ///
    /// <para>A manual session that ends while commands keep coming does not hand over: the next
    /// automatic session opens after the next idle gap, as the rule says.</para>
    /// </summary>
    public sealed class AutoSessionRecorder
    {
        private const double DiskCheckSeconds = 2.0;
        private const double GB = 1024.0 * 1024.0 * 1024.0;

        private readonly AutoRecordOptions _options;
        private readonly IAutoSessionSink _sink;
        private readonly Func<DateTime> _utcNow;
        private readonly Func<long?> _freeBytes;
        private readonly Func<UplinkCondition> _uplink;
        private readonly Action<string> _log;
        private readonly long _ticksPerSecond;
        private readonly long _idleTicks;
        private readonly long _maxTicks;
        private readonly long _minSplitTicks;
        private readonly long _diskCheckTicks;
        private readonly long _stopBytes;
        private readonly long _startBytes;

        private bool _haveCommand;
        private long _lastCommandTicks;
        private int _generation;
        private long _openedTicks;
        private string? _sessionSender;
        private string? _sessionSubscriber;
        private string? _currentSubscriber;
        private bool _reopen;
        private string _reopenReason = "";
        private bool _haveSplit;
        private long _lastSplitTicks;
        private bool _haveDisk;
        private long _diskCheckedTicks;
        private long? _cachedFree;
        private bool _loggedBusy;
        private bool _loggedLowDisk;
        private bool _loggedSplitHold;

        public AutoSessionRecorder(AutoRecordOptions options, IAutoSessionSink sink, long ticksPerSecond, Func<DateTime> utcNow,
            Func<long?> freeBytes, Func<UplinkCondition> uplink, Action<string>? log = null)
        {
            _options = options ?? throw new ArgumentNullException(nameof(options));
            string? invalid = options.Validate();
            if (invalid != null) throw new ArgumentException(invalid, nameof(options));
            _sink = sink ?? throw new ArgumentNullException(nameof(sink));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _ticksPerSecond = ticksPerSecond;
            _utcNow = utcNow ?? throw new ArgumentNullException(nameof(utcNow));
            _freeBytes = freeBytes ?? throw new ArgumentNullException(nameof(freeBytes));
            _uplink = uplink ?? throw new ArgumentNullException(nameof(uplink));
            _log = log ?? (_ => { });
            _idleTicks = Ticks(options.IdleSeconds);
            _maxTicks = Ticks(options.MaxMinutes * 60);
            _minSplitTicks = Ticks(options.MinSplitSeconds);
            _diskCheckTicks = Ticks(DiskCheckSeconds);
            _stopBytes = options.BytesFromGigabytes(options.StopFreeGigabytes);
            _startBytes = options.BytesFromGigabytes(options.StartFreeGigabytes);
        }

        /// <summary>True while an automatic session this recorder opened is still open.</summary>
        public bool IsOpen
        {
            get
            {
                Refresh();
                return _generation != 0;
            }
        }

        /// <summary>How many sessions it has opened since start-up.</summary>
        public int SessionsOpened { get; private set; }

        /// <summary>
        /// The joint-state stream's subscriber is now <paramref name="endpoint"/> (call on a change),
        /// or lapsed (null: its keepalives stopped). A different subscriber during a session splits it.
        /// </summary>
        public void OnSubscriber(long nowTicks, string? endpoint)
        {
            if (string.Equals(endpoint, _currentSubscriber, StringComparison.Ordinal)) return;
            _currentSubscriber = endpoint;
            if (endpoint == null) return;
            Refresh();
            if (_generation == 0) return;
            if (_sessionSubscriber == null)
            {
                _sessionSubscriber = endpoint;
                return;
            }

            if (string.Equals(endpoint, _sessionSubscriber, StringComparison.Ordinal)) return;
            TrySplit(nowTicks, $"the joint-state subscriber changed from {_sessionSubscriber} to {endpoint}");
        }

        /// <summary>A joint command from <paramref name="sender"/> reached RobotHost. Call before it is recorded: it may open the session it belongs to.</summary>
        public void OnJointCommand(long nowTicks, string sender)
        {
            if (sender == null) throw new ArgumentNullException(nameof(sender));
            Refresh();
            if (_generation != 0 && !string.Equals(sender, _sessionSender, StringComparison.Ordinal))
            {
                TrySplit(nowTicks, $"the joint-command sender changed from {_sessionSender} to {sender}");
            }

            if (_generation == 0)
            {
                bool idleGap = !_haveCommand || nowTicks - _lastCommandTicks >= _idleTicks;
                if (idleGap || _reopen)
                {
                    TryOpen(nowTicks, sender, idleGap ? "first joint command after " + Seconds(_options.IdleSeconds) + " without one" : _reopenReason);
                }
            }

            _haveCommand = true;
            _lastCommandTicks = nowTicks;
        }

        /// <summary>Called every main-loop iteration: the idle close, the rotation and the disk guard.</summary>
        public void Tick(long nowTicks)
        {
            Refresh();
            if (_generation == 0) return;
            if (_haveCommand && nowTicks - _lastCommandTicks >= _idleTicks)
            {
                Close("no joint command for " + Seconds(_options.IdleSeconds), reopen: false);
                return;
            }

            if (nowTicks - _openedTicks >= _maxTicks)
            {
                Close("rotation after " + Seconds(_options.MaxMinutes * 60), reopen: true);
                return;
            }

            long? free = FreeBytes(nowTicks);
            if (free.HasValue && free.Value < _stopBytes)
            {
                Close(FormattableString.Invariant($"only {free.Value / GB:0.00} GB free, below {_options.StopFreeGigabytes} GB"), reopen: false);
                // Reopens on a later command once there is room again (TryOpen checks the start threshold).
                _reopen = true;
                _reopenReason = "free space recovered";
            }
        }

        private void TrySplit(long nowTicks, string why)
        {
            if (_haveSplit && nowTicks - _lastSplitTicks < _minSplitTicks)
            {
                if (!_loggedSplitHold)
                {
                    _log($"[record] {why}, again within {Seconds(_options.MinSplitSeconds)} of the last split: not splitting; this session will fail the trainer's check");
                    _loggedSplitHold = true;
                }

                return;
            }

            _loggedSplitHold = false;
            _haveSplit = true;
            _lastSplitTicks = nowTicks;
            Close(why, reopen: true);
        }

        private void TryOpen(long nowTicks, string sender, string reason)
        {
            if (_sink.ControlFileBusy)
            {
                if (!_loggedBusy) _log("[record] a manual session is recording: the automatic recorder stays out");
                _loggedBusy = true;
                return;
            }

            _loggedBusy = false;
            long? free = FreeBytes(nowTicks);
            if (free.HasValue && free.Value < _startBytes)
            {
                if (!_loggedLowDisk)
                {
                    _log(FormattableString.Invariant(
                        $"[record] not opening an automatic session: only {free.Value / GB:0.00} GB free, below {_options.StartFreeGigabytes} GB (just fetch-recordings, then just robot-prune-recordings true)"));
                }

                _loggedLowDisk = true;
                _reopen = true;
                _reopenReason = "free space recovered";
                return;
            }

            _loggedLowDisk = false;
            int generation = _sink.BeginAuto(new AutoSessionSpec(_utcNow(), sender, _uplink(), reason));
            if (generation == 0) return;
            _generation = generation;
            _openedTicks = nowTicks;
            _sessionSender = sender;
            _sessionSubscriber = _currentSubscriber;
            _reopen = false;
            SessionsOpened++;
            _log($"[record] automatic session opened: {reason}; commands from {sender}");
        }

        private void Close(string reason, bool reopen)
        {
            _sink.EndAuto(_generation, reason);
            _log($"[record] automatic session closed: {reason}");
            _generation = 0;
            _reopen = reopen;
            _reopenReason = reason;
        }

        /// <summary>The writer can end a session on its own: a manual session took over, or an I/O error.</summary>
        private void Refresh()
        {
            if (_generation == 0 || _sink.IsSessionActive(_generation)) return;
            _log("[record] the automatic session was closed by the writer (a manual session took over, or a write failed)");
            _generation = 0;
            _reopen = false;
        }

        private long? FreeBytes(long nowTicks)
        {
            if (!_haveDisk || nowTicks - _diskCheckedTicks >= _diskCheckTicks)
            {
                _cachedFree = _freeBytes();
                _diskCheckedTicks = nowTicks;
                _haveDisk = true;
            }

            return _cachedFree;
        }

        private long Ticks(double seconds) => (long)(seconds * _ticksPerSecond);

        private static string Seconds(double seconds) => seconds.ToString("0.###", CultureInfo.InvariantCulture) + " s";
    }
}
