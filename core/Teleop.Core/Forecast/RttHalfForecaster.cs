using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Forecast
{
    /// <summary>
    /// The <c>rtt-half</c> forecast (docs/adr/0021 decision 12; a C# copy of <c>trainer/armpred/net/forecast.py</c>'s
    /// <c>_rtt_half</c>, pinned by <c>core/testdata/learned/forecast-golden-v1.json</c>): today's estimate as a forecast.
    /// ClockSync's smoothed RTT/2 (its uncertainty) for both directions at every horizon and quantile, the floor half its
    /// window minimum; no spread, loss or outage. Invalid until ClockSync has synced. Allocation-free.
    /// </summary>
    public sealed class RttHalfForecaster : INetworkForecaster
    {
        private readonly long _tps;
        private bool _has;
        private long _t, _uncertainty, _minRtt;
        private bool _synced;

        public RttHalfForecaster(long ticksPerSecond)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
        }

        public string Level => "rtt-half";

        public void ObserveFrame(long arrivalTicks, long sendEstimateTicks, long sequence)
        {
        }

        public void ObserveRoundTrip(long ticks, long rttTicks, long downlinkTicks, long syncUncertaintyTicks, long syncMinRttTicks, bool synced)
        {
            _has = true;
            _t = ticks;
            _uncertainty = syncUncertaintyTicks;
            _minRtt = syncMinRttTicks;
            _synced = synced;
        }

        public void Issue(long ticks, NetworkForecast output)
        {
            if (output == null) throw new ArgumentNullException(nameof(output));
            output.Clear(ticks);
            // The newest ClockSync state at or before the issue; round trips are observed in order, so it is the last one.
            if (!_has || _t > ticks || !_synced) return;
            float half = (float)((double)_uncertainty / _tps * 1e3);
            float floor = (float)(_minRtt / 2.0 / _tps * 1e3);
            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                output.ValidQ[d] = true;
                output.FloorMs[d] = floor;
                for (int h = 0; h < NetworkForecast.HorizonCount; h++)
                {
                    for (int p = 0; p < NetworkForecast.QuantileCount; p++) output.QMs[NetworkForecast.QIndex(d, h, p)] = half;
                }
            }
        }

        public void Reset()
        {
            _has = false;
            _t = _uncertainty = _minRtt = 0;
            _synced = false;
        }
    }
}
