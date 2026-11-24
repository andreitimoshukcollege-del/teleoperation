using System;
using System.Globalization;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// What a <c>stall:</c> uplink profile asks for (docs/adr/0038 N): a one-way delay of <see cref="BaseMs"/> plus up to
    /// <see cref="JitterMs"/>, and now and then a stall of <see cref="MinStallMs"/> to <see cref="MaxStallMs"/> that ends in a
    /// burst of the commands it held, the stalls <see cref="MeanGapSeconds"/> apart on average. The profile's name is
    /// <c>stall:&lt;baseMs&gt;:&lt;jitterMs&gt;:&lt;meanGapS&gt;:&lt;minStallMs&gt;:&lt;maxStallMs&gt;</c>, so the <c>UP</c> record
    /// and <c>session.json</c> carry every setting, and the seed beside it the draws.
    /// </summary>
    public readonly struct StallSettings
    {
        public const string Prefix = "stall:";

        public StallSettings(double baseMs, double jitterMs, double meanGapSeconds, double minStallMs, double maxStallMs)
        {
            BaseMs = baseMs;
            JitterMs = jitterMs;
            MeanGapSeconds = meanGapSeconds;
            MinStallMs = minStallMs;
            MaxStallMs = maxStallMs;
        }

        public double BaseMs { get; }
        public double JitterMs { get; }
        public double MeanGapSeconds { get; }
        public double MinStallMs { get; }
        public double MaxStallMs { get; }

        public static bool IsStall(string? name) => name != null && name.StartsWith(Prefix, StringComparison.Ordinal);

        /// <summary>Reads a <c>stall:</c> name; false, with the reason, for anything else or a value out of range.</summary>
        public static bool TryParse(string name, out StallSettings settings, out string? error)
        {
            settings = default;
            if (!IsStall(name))
            {
                error = $"'{name}' is not a {Prefix} profile";
                return false;
            }

            string[] parts = name.Substring(Prefix.Length).Split(':');
            var v = new double[5];
            if (parts.Length != 5)
            {
                error = $"'{name}' is not {Prefix}<baseMs>:<jitterMs>:<meanGapS>:<minStallMs>:<maxStallMs>";
                return false;
            }

            for (int i = 0; i < 5; i++)
            {
                if (!double.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v[i]) || !(v[i] >= 0) || double.IsInfinity(v[i]))
                {
                    error = $"'{name}': '{parts[i]}' is not a number of zero or more";
                    return false;
                }
            }

            if (v[0] > 2000 || v[1] > 1000 || v[2] < 0.2 || v[2] > 3600 || v[3] > v[4] || v[4] > 5000)
            {
                error = $"'{name}': want base at most 2000 ms, jitter at most 1000 ms, a mean gap of 0.2 to 3600 s, " +
                        "and stalls of at most 5000 ms with the shortest no longer than the longest";
                return false;
            }

            settings = new StallSettings(v[0], v[1], v[2], v[3], v[4]);
            error = null;
            return true;
        }

        public string Name => Prefix + string.Join(":", F(BaseMs), F(JitterMs), F(MeanGapSeconds), F(MinStallMs), F(MaxStallMs));

        private static string F(double x) => x.ToString("R", CultureInfo.InvariantCulture);
    }

    /// <summary>
    /// A robot-side uplink that is late, jittery and now and then stalls (docs/adr/0038 N), for the twin's takes: RobotHost's
    /// <see cref="UplinkImpairedReceiver"/> runs it on what the joint socket reads, as it runs Core's profiles (docs/adr/0035 N4).
    ///
    /// <para><b>A queue, as a real link is.</b> Each datagram's arrival is its read time plus the base delay and a jitter that
    /// is a function of time (one draw per <see cref="JitterBinMs"/> since the first datagram), never earlier than the
    /// datagram before it: the link keeps its order, so jitter alone reorders nothing and RobotHost's staleness rule (a
    /// capture stamp not newer than the last accepted) refuses nothing. A datagram read during a stall waits for its end and
    /// then crosses with the base delay: every one the stall held arrives at the same moment, in order, the burst a link that
    /// comes back delivers. Nothing is lost.</para>
    ///
    /// <para><b>A function of the time since the first datagram and the seed</b>, not of how many datagrams came, so two takes
    /// that send the same commands at the same times after their first see the same delays: RobotHost re-arms the profile at
    /// the start of every session (Program), which keeps a <c>mixed</c> take's replays on their source's network
    /// (docs/adr/0038 decision 2).</para>
    ///
    /// <para>RobotHost's own (not Core's): a host profile beside <c>trace:</c> (<see cref="UplinkProfiles"/>), with Core's
    /// <see cref="INetworkImpairment"/> contract and RNG. It reads no clock: time is each datagram's read stamp.</para>
    /// </summary>
    public sealed class StallUplinkImpairment : INetworkImpairment
    {
        /// <summary>How long one jitter draw holds.</summary>
        public const double JitterBinMs = 20;

        private readonly long _baseTicks, _jitterTicks, _binTicks, _minStallTicks, _maxStallTicks;
        private readonly double _meanGapTicks;
        private SeededRng _schedule;
        private ulong _jitterKey;
        private bool _bound, _started;
        private long _origin, _stallStart, _stallEnd, _lastArrival;

        public StallUplinkImpairment(in StallSettings s, long ticksPerSecond)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            Settings = s;
            double perMs = ticksPerSecond / 1000.0;
            _baseTicks = (long)Math.Round(s.BaseMs * perMs);
            _jitterTicks = (long)Math.Round(s.JitterMs * perMs);
            _binTicks = Math.Max(1L, (long)Math.Round(JitterBinMs * perMs));
            _minStallTicks = (long)Math.Round(s.MinStallMs * perMs);
            _maxStallTicks = (long)Math.Round(s.MaxStallMs * perMs);
            _meanGapTicks = s.MeanGapSeconds * ticksPerSecond;
        }

        public StallSettings Settings { get; }

        public string AxisName => "uplink-stall";

        public ImpairmentStages Stages => ImpairmentStages.Deliver;

        public void Bind(ulong substreamSeed)
        {
            if (_bound) throw new InvalidOperationException("This impairment is already bound to a transport. Each transport needs its own instances.");
            _bound = true;
            _schedule = new SeededRng(substreamSeed);
            _jitterKey = new SeededRng(substreamSeed ^ 0x9E3779B97F4A7C15UL).NextUInt64();
        }

        public void ApplyOnSend(ref DatagramFate fate)
        {
        }

        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            long read = fate.SendTicks;
            if (!_started)
            {
                _started = true;
                _origin = read;
                _lastArrival = long.MinValue;
                _stallEnd = read;
                NextStall();
            }

            while (read >= _stallEnd) NextStall();

            long elapsed = read - _origin;
            long bin = elapsed < 0 ? 0 : elapsed / _binTicks;
            long arrival = read + _baseTicks + (long)(Unit(_jitterKey, (ulong)bin) * _jitterTicks);
            if (read >= _stallStart) arrival = Math.Max(arrival, _stallEnd + _baseTicks);
            if (arrival < _lastArrival) arrival = _lastArrival;
            _lastArrival = arrival;
            fate.DelayTicks += arrival - read;
        }

        /// <summary>The stall after the current one: a gap uniform in half to one and a half times the mean, then a length.</summary>
        private void NextStall()
        {
            long gap = (long)((0.5 + _schedule.NextDouble()) * _meanGapTicks);
            _stallStart = _stallEnd + Math.Max(1L, gap);
            _stallEnd = _stallStart + _minStallTicks + (long)(_schedule.NextDouble() * (_maxStallTicks - _minStallTicks));
        }

        /// <summary>A uniform draw in [0, 1) from (key, index): SplitMix64's finaliser.</summary>
        private static double Unit(ulong key, ulong index)
        {
            ulong z = key + (index + 1) * 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            z ^= z >> 31;
            return (z >> 11) * (1.0 / (1UL << 53));
        }

        public void Reset()
        {
            _schedule.Reset();
            _started = false;
        }
    }
}
