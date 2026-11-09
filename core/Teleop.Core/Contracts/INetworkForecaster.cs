using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Forecasts the network from what the operator observes (docs/adr/0035 decision 1, N3): a C# copy of one of
    /// <c>trainer/armpred/net/forecast.py</c>'s causal levels, so the learned joint model is told the link it is on in the
    /// form it was trained with (<see cref="NetworkForecast"/>, trainer/contract/FORECAST_FORMAT.md).
    /// <c>core/testdata/learned/forecast-golden-v1.json</c> (<c>armpred golden-forecast</c>) pins each copy to the trainer's.
    ///
    /// <para>All ticks are the operator's. Observations arrive in arrival order. Deterministic, allocation-free after
    /// construction, and <see cref="Reset"/> returns it to as constructed.</para>
    /// </summary>
    public interface INetworkForecaster
    {
        /// <summary>The trainer's level name (<c>persistence</c>, <c>rtt-half</c>).</summary>
        string Level { get; }

        /// <summary>
        /// A joint-state frame arrived at <paramref name="arrivalTicks"/>. <paramref name="sendEstimateTicks"/> is the
        /// robot's send stamp on the operator's clock by ClockSync, after any round trip the frame carried; negative before
        /// ClockSync has synced, and the frame is then ignored. <paramref name="sequence"/> is the frame's sequence, unwrapped.
        /// </summary>
        void ObserveFrame(long arrivalTicks, long sendEstimateTicks, long sequence);

        /// <summary>
        /// A frame carried a new keepalive echo at <paramref name="ticks"/>: the round trip ClockSync computed from it
        /// (offered, whether or not ClockSync accepted it), the downlink delay of that frame by its send estimate after the
        /// round trip (negative if unknown), and ClockSync's state after it.
        /// </summary>
        void ObserveRoundTrip(long ticks, long rttTicks, long downlinkTicks, long syncUncertaintyTicks, long syncMinRttTicks, bool synced);

        /// <summary>The forecast at <paramref name="ticks"/>, from what was observed at or before it, into <paramref name="output"/>.</summary>
        void Issue(long ticks, NetworkForecast output);

        void Reset();
    }
}
