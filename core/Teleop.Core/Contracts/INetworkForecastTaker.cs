using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// A joint predictor that takes the network forecast the host passes on (docs/adr/0021 decision 11, 0035 N3). The host
    /// also gives it each command at its forecast effective time (send + the forecast's uplink median at h = 0,
    /// <c>Forecast/ForecastEffectiveTimes</c>), as the trainer does for a predictor that uses the forecast; every other
    /// predictor keeps send + RTT/2. Not a research axis of its own: a capability an <see cref="ICommandAwarePredictor"/>
    /// may have.
    /// </summary>
    public interface INetworkForecastTaker
    {
        /// <summary>The newest forecast; the taker copies what it needs, the host reuses <paramref name="forecast"/>.</summary>
        void ObserveForecast(NetworkForecast forecast);

        /// <summary>No forecast from now on (the network block goes back to zeros, the <c>absent</c> level).</summary>
        void ClearForecast();
    }
}
