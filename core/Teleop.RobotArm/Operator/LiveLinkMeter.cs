using System;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>
    /// The real network as the headset sees it, live, from the joint-state frames (docs/metrics.md §3, "Live link health"): what
    /// the operator console shows when the robot is far away and the link's own delay, jitter and loss are not simulated. Over
    /// the last <see cref="WindowSeconds"/>:
    /// <list type="bullet">
    /// <item><b>Lag</b>: the median one-way delay from the robot's send to the headset's arrival (<c>owd_downlink_ms</c>), with
    /// the send stamp put on the headset's clock by the caller's <c>ClockSync</c>; unknown until it has synced.</item>
    /// <item><b>Jitter</b>: RFC 3550 interarrival jitter (<c>net_downlink_jitter_ms</c>), from each side's raw stamps and own tick
    /// rate, so no clock offset enters it.</item>
    /// <item><b>Loss</b>: the share of frames the window's sequence numbers say were sent and never arrived, downlink only (the
    /// headset cannot see what the robot never received).</item>
    /// </list>
    /// The round trip is <c>ClockSync</c>'s own and not kept here. Diagnostics, never results: a frame the robot did not send
    /// (a RobotHost restart) restarts the window. Allocation-free after construction; no clock of its own.
    /// </summary>
    public sealed class LiveLinkMeter
    {
        public const double WindowSeconds = 5.0;
        private const int Capacity = 1024;

        private readonly long _tps;
        private readonly long[] _arrival = new long[Capacity];
        private readonly uint[] _sequence = new uint[Capacity];
        private readonly double[] _owdMs = new double[Capacity];
        private readonly double[] _sorted = new double[Capacity];
        private int _head, _count;
        private bool _hasPrevious;
        private long _previousArrival, _previousSend, _previousSendTps;
        private uint _previousSequence;
        private double _jitterMs;

        /// <param name="ticksPerSecond">The headset's clock rate: arrivals are in its ticks.</param>
        public LiveLinkMeter(long ticksPerSecond)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
        }

        /// <summary>Frames in the window.</summary>
        public int Frames => _count;

        /// <summary>RFC 3550 interarrival jitter (ms), smoothed over every frame since the last restart.</summary>
        public double JitterMs => _jitterMs;

        /// <summary>
        /// One frame: its sequence, the robot's raw send stamp and tick rate, its arrival on the headset's clock, and the send
        /// stamp on the headset's clock (<see cref="long.MinValue"/> before the clocks are synced).
        /// </summary>
        public void OnFrame(uint sequence, long robotSendTicks, long robotTicksPerSecond, long arrivalTicks, long sendOnOperatorTicks)
        {
            if (robotTicksPerSecond <= 0) return;
            if (_hasPrevious)
            {
                int step = unchecked((int)(sequence - _previousSequence));
                if (step <= 0 && step > -1000) return; // a duplicate or a late one: not counted twice
                if (step > 1000 || step <= -1000)
                {
                    Reset(); // the robot started counting again
                }
            }

            if (_hasPrevious && robotTicksPerSecond == _previousSendTps)
            {
                double received = (arrivalTicks - _previousArrival) * 1000.0 / _tps;
                double sent = (robotSendTicks - _previousSend) * 1000.0 / robotTicksPerSecond;
                double d = Math.Abs(received - sent);
                _jitterMs += (d - _jitterMs) / 16.0;
            }

            _hasPrevious = true;
            _previousArrival = arrivalTicks;
            _previousSend = robotSendTicks;
            _previousSendTps = robotTicksPerSecond;
            _previousSequence = sequence;

            int slot = (_head + _count) % Capacity;
            if (_count == Capacity)
            {
                _head = (_head + 1) % Capacity;
                _count--;
                slot = (_head + _count) % Capacity;
            }

            _arrival[slot] = arrivalTicks;
            _sequence[slot] = sequence;
            _owdMs[slot] = sendOnOperatorTicks == long.MinValue ? double.NaN : (arrivalTicks - sendOnOperatorTicks) * 1000.0 / _tps;
            _count++;
        }

        /// <summary>Drops what is older than the window at <paramref name="now"/>; call before reading.</summary>
        public void Trim(long now)
        {
            long oldest = now - (long)(WindowSeconds * _tps);
            while (_count > 0 && _arrival[_head] < oldest)
            {
                _head = (_head + 1) % Capacity;
                _count--;
            }
        }

        /// <summary>The share of the window's frames, by sequence number, that never arrived (0..1); 0 with fewer than two.</summary>
        public double LossFraction
        {
            get
            {
                if (_count < 2) return 0;
                uint first = _sequence[_head], last = _sequence[(_head + _count - 1) % Capacity];
                double expected = unchecked(last - first) + 1.0;
                return expected > _count ? 1.0 - _count / expected : 0.0;
            }
        }

        /// <summary>The window's median one-way delay (ms); NaN until a frame with synced clocks has come.</summary>
        public double LagMs
        {
            get
            {
                int n = 0;
                for (int i = 0; i < _count; i++)
                {
                    double v = _owdMs[(_head + i) % Capacity];
                    if (!double.IsNaN(v)) _sorted[n++] = v;
                }

                if (n == 0) return double.NaN;
                Array.Sort(_sorted, 0, n);
                return n % 2 == 1 ? _sorted[n / 2] : 0.5 * (_sorted[n / 2 - 1] + _sorted[n / 2]);
            }
        }

        public void Reset()
        {
            _head = _count = 0;
            _hasPrevious = false;
            _jitterMs = 0;
        }
    }
}
