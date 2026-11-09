using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Forecast
{
    /// <summary>
    /// The <c>persistence</c> forecast (docs/adr/0021 decision 12; a C# copy of <c>trainer/armpred/net/forecast.py</c>'s
    /// <c>_persistence</c>, pinned by <c>core/testdata/learned/forecast-golden-v1.json</c>): today's observations, held, with
    /// no look-ahead.
    ///
    /// <para><b>Downlink</b>, from the frames that arrived in the last <see cref="RecentSeconds"/> (at least
    /// <see cref="MinRecentFrames"/> of them, or the downlink is invalid): the delay quantiles of those frames (arrival minus
    /// ClockSync's send estimate); for a negative horizon h, those of the frames sent within ±100 ms of t + h, or, if none of
    /// them has arrived yet, the recent quantiles raised to at least −h (still in flight); loss from the gaps in their
    /// sequence; the floor, the smallest delay in the last <see cref="FloorSeconds"/>; an outage (1) when nothing has
    /// arrived for <see cref="OutageSeconds"/>.</para>
    ///
    /// <para><b>Uplink</b>, from the keepalive round trips of the last <see cref="FloorSeconds"/> whose echoing frame had a
    /// send estimate (at least two): each round trip minus that frame's downlink delay, at least 0; the quantiles of the
    /// newest five, the same at every horizon; the floor, their smallest. No loss and no outage for the uplink.</para>
    ///
    /// <para>Keeps the frames of the last <see cref="FloorSeconds"/> in a ring of <c>frameCapacity</c> (2048: 10 s up to
    /// 200 Hz); beyond that the oldest go first. Frames are kept in arrival order (one arriving out of order is put in its
    /// place). Allocation-free after construction.</para>
    /// </summary>
    public sealed class PersistenceForecaster : INetworkForecaster
    {
        public const double RecentSeconds = 2.0;
        public const double FloorSeconds = 10.0;
        public const double NearSeconds = 0.1;
        public const double OutageSeconds = 0.25; // FORECAST_FORMAT.md's outage: nothing for at least 250 ms
        public const int MinRecentFrames = 5;
        public const int MinRoundTrips = 2;
        public const int UplinkNewest = 5;

        private readonly long _tps;
        private readonly long _recent, _floorSpan, _near;
        private readonly int _frameCapacity, _rttCapacity;
        private readonly long[] _arr, _send, _seq;
        private int _head, _count;
        private readonly long[] _rttT;
        private readonly double[] _upMs;
        private int _rttHead, _rttCount;
        private readonly double[] _recentMs, _scratch, _q = new double[NetworkForecast.QuantileCount];

        public PersistenceForecaster(long ticksPerSecond, int frameCapacity = 2048, int roundTripCapacity = 256)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            if (frameCapacity < MinRecentFrames) throw new ArgumentOutOfRangeException(nameof(frameCapacity));
            if (roundTripCapacity < UplinkNewest) throw new ArgumentOutOfRangeException(nameof(roundTripCapacity));
            _tps = ticksPerSecond;
            // forecast.py: int(2 * tps), int(10 * tps), int(0.1 * tps): truncation, as C#'s (long).
            _recent = (long)(RecentSeconds * ticksPerSecond);
            _floorSpan = (long)(FloorSeconds * ticksPerSecond);
            _near = (long)(NearSeconds * ticksPerSecond);
            _frameCapacity = frameCapacity;
            _rttCapacity = roundTripCapacity;
            _arr = new long[frameCapacity];
            _send = new long[frameCapacity];
            _seq = new long[frameCapacity];
            _rttT = new long[roundTripCapacity];
            _upMs = new double[roundTripCapacity];
            _recentMs = new double[frameCapacity];
            _scratch = new double[frameCapacity];
        }

        public string Level => "persistence";

        public void ObserveFrame(long arrivalTicks, long sendEstimateTicks, long sequence)
        {
            if (sendEstimateTicks < 0) return; // before ClockSync synced: forecast.py keeps frame_send_est >= 0 only
            if (_count == _frameCapacity)
            {
                _head = (_head + 1) % _frameCapacity;
                _count--;
            }

            // In arrival order (forecast.py sorts by arrival, stably): a late stamp moves back past newer ones.
            int i = _count;
            while (i > 0 && _arr[Slot(i - 1)] > arrivalTicks)
            {
                int from = Slot(i - 1), to = Slot(i);
                _arr[to] = _arr[from];
                _send[to] = _send[from];
                _seq[to] = _seq[from];
                i--;
            }

            int s = Slot(i);
            _arr[s] = arrivalTicks;
            _send[s] = sendEstimateTicks;
            _seq[s] = sequence;
            _count++;
        }

        public void ObserveRoundTrip(long ticks, long rttTicks, long downlinkTicks, long syncUncertaintyTicks, long syncMinRttTicks, bool synced)
        {
            if (downlinkTicks < 0) return; // forecast.py: only round trips whose echoing frame had a send estimate
            int slot = (_rttHead + _rttCount) % _rttCapacity;
            if (_rttCount == _rttCapacity)
            {
                _rttHead = (_rttHead + 1) % _rttCapacity;
            }
            else
            {
                _rttCount++;
            }

            _rttT[slot] = ticks;
            _upMs[slot] = Math.Max(0.0, (double)(rttTicks - downlinkTicks) / _tps * 1e3);
        }

        public void Issue(long ticks, NetworkForecast output)
        {
            if (output == null) throw new ArgumentNullException(nameof(output));
            output.Clear(ticks);
            IssueDownlink(ticks, output);
            IssueUplink(ticks, output);
        }

        public void Reset()
        {
            _head = _count = 0;
            _rttHead = _rttCount = 0;
        }

        private int Slot(int logical) => (_head + logical) % _frameCapacity;

        /// <summary>The first logical frame index whose arrival is at least (<paramref name="inclusive"/>) or above <paramref name="t"/>.</summary>
        private int Bound(long t, bool inclusive)
        {
            int lo = 0, hi = _count;
            while (lo < hi)
            {
                int mid = (lo + hi) >> 1;
                long a = _arr[Slot(mid)];
                if (inclusive ? a < t : a <= t) lo = mid + 1;
                else hi = mid;
            }

            return lo;
        }

        private double DelayMs(int logical)
        {
            int s = Slot(logical);
            return (double)(_arr[s] - _send[s]) / _tps * 1e3;
        }

        private void IssueDownlink(long t, NetworkForecast f)
        {
            int a = Bound(t - _recent, inclusive: true), b = Bound(t, inclusive: false);
            int n = b - a;
            if (n < MinRecentFrames) return;
            long seqMin = long.MaxValue, seqMax = long.MinValue;
            for (int i = a; i < b; i++)
            {
                _recentMs[i - a] = DelayMs(i);
                long q = _seq[Slot(i)];
                if (q < seqMin) seqMin = q;
                if (q > seqMax) seqMax = q;
            }

            Array.Copy(_recentMs, _scratch, n);
            LinearPercentile.Sort(_scratch, n);
            for (int p = 0; p < NetworkForecast.QuantileCount; p++) _q[p] = LinearPercentile.OfSorted(_scratch, n, NetworkForecast.Quantile(p));
            long expected = seqMax - seqMin + 1;
            double loss = expected > 0 ? 1.0 - (double)n / expected : 0.0;
            double floor = double.MaxValue;
            for (int i = Bound(t - _floorSpan, inclusive: true); i < b; i++) floor = Math.Min(floor, DelayMs(i));

            const int d = NetworkForecast.Down;
            f.ValidQ[d] = f.ValidLoss[d] = f.ValidOutage[d] = true;
            f.FloorMs[d] = (float)floor;
            double silent = (double)(t - _arr[Slot(b - 1)]) / _tps;
            f.Outage[d] = silent >= OutageSeconds ? 1f : 0f;
            for (int h = 0; h < NetworkForecast.HorizonCount; h++)
            {
                int hMs = NetworkForecast.HorizonMs(h);
                if (hMs < 0)
                {
                    // Frames sent then: what those that arrived took; if none has arrived, they are still in flight.
                    long s = t + (long)(hMs / 1000.0 * _tps);
                    int m = 0;
                    for (int i = a; i < b; i++)
                    {
                        long send = _send[Slot(i)];
                        if (send >= s - _near && send < s + _near) _scratch[m++] = _recentMs[i - a];
                    }

                    if (m > 0)
                    {
                        LinearPercentile.Sort(_scratch, m);
                        for (int p = 0; p < NetworkForecast.QuantileCount; p++)
                        {
                            f.QMs[NetworkForecast.QIndex(d, h, p)] = (float)LinearPercentile.OfSorted(_scratch, m, NetworkForecast.Quantile(p));
                        }
                    }
                    else
                    {
                        for (int p = 0; p < NetworkForecast.QuantileCount; p++) f.QMs[NetworkForecast.QIndex(d, h, p)] = (float)Math.Max(_q[p], -hMs);
                    }
                }
                else
                {
                    for (int p = 0; p < NetworkForecast.QuantileCount; p++) f.QMs[NetworkForecast.QIndex(d, h, p)] = (float)_q[p];
                }

                f.Loss[NetworkForecast.LossIndex(d, h)] = (float)loss;
            }
        }

        private void IssueUplink(long t, NetworkForecast f)
        {
            // The round trips in [t - 10 s, t], in arrival order (they are observed in it).
            int first = -1, last = -1;
            for (int i = 0; i < _rttCount; i++)
            {
                long rt = _rttT[(_rttHead + i) % _rttCapacity];
                if (rt < t - _floorSpan) continue;
                if (rt > t) break;
                if (first < 0) first = i;
                last = i;
            }

            if (first < 0 || last - first + 1 < MinRoundTrips) return;
            int from = Math.Max(first, last + 1 - UplinkNewest);
            int n = 0;
            for (int i = from; i <= last; i++) _scratch[n++] = _upMs[(_rttHead + i) % _rttCapacity];
            LinearPercentile.Sort(_scratch, n);
            double floor = double.MaxValue;
            for (int i = first; i <= last; i++) floor = Math.Min(floor, _upMs[(_rttHead + i) % _rttCapacity]);
            const int u = NetworkForecast.Up;
            f.ValidQ[u] = true;
            f.FloorMs[u] = (float)floor;
            for (int p = 0; p < NetworkForecast.QuantileCount; p++)
            {
                float q = (float)LinearPercentile.OfSorted(_scratch, n, NetworkForecast.Quantile(p));
                for (int h = 0; h < NetworkForecast.HorizonCount; h++) f.QMs[NetworkForecast.QIndex(u, h, p)] = q;
            }
        }
    }
}
