using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Forecast
{
    /// <summary>
    /// When a command takes effect, for a predictor that uses the forecast (docs/adr/0021 decision 11; a C# copy of
    /// <c>trainer/armpred/net/forecast.py</c>'s <c>effective_times</c>, pinned by the forecast golden): with the newest
    /// forecast's uplink valid, the send plus its median at h = 0 (rounded half to even, as Python's <c>round</c>);
    /// otherwise the host's own estimate (send + RTT/2). Never before the send, and never before the previous command's:
    /// a FIFO path cannot deliver a later datagram first. One instance per command stream; <see cref="Reset"/> with it.
    /// </summary>
    public sealed class ForecastEffectiveTimes
    {
        private readonly long _tps;
        private bool _has;
        private long _last;

        public ForecastEffectiveTimes(long ticksPerSecond)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
        }

        /// <param name="forecast">The newest forecast issued at or before the send; null when there is none.</param>
        public long Next(long sendTicks, long fallbackEffectiveTicks, NetworkForecast? forecast)
        {
            long e = fallbackEffectiveTicks;
            if (forecast != null && forecast.ValidQ[NetworkForecast.Up])
            {
                double q = forecast.QMs[NetworkForecast.QIndex(NetworkForecast.Up, NetworkForecast.H0, NetworkForecast.P50)];
                e = sendTicks + (long)Math.Round(q / 1e3 * _tps, MidpointRounding.ToEven);
            }

            if (e < sendTicks) e = sendTicks;
            if (_has && e < _last) e = _last;
            _has = true;
            _last = e;
            return e;
        }

        public void Reset()
        {
            _has = false;
            _last = 0;
        }
    }
}
