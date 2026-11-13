# Forecast

Implementations of `Contracts/INetworkForecaster.cs`: what the network is doing, from what the operator observes, in the
form the learned joint model was trained to read (`Types/NetworkForecast.cs`, trainer/contract/FORECAST_FORMAT.md;
docs/adr/0021 decisions 11 and 12, docs/adr/0035 N3). The host feeds each joint-state frame (arrival, ClockSync's send
estimate, sequence) and each keepalive round trip, and issues a forecast every 100 ms; `JointPredictorBank` passes it to
the members that take one (`Contracts/INetworkForecastTaker.cs`: today `learned`), with commands at their forecast
effective time.

**These are copies of the trainer, not new algorithms.** `trainer/armpred/net/forecast.py` is the source;
`core/testdata/learned/forecast-golden-v1.json`, written by `armpred golden-forecast` (`just gen-forecast-golden`), pins
every forecast, the network block and the effective times (`Teleop.Core.Tests/Forecast/NetworkForecasterTests.cs`). A
change starts in the trainer, regenerates the golden, and then fixes the copy. A copy that is "close enough" tells the
model about a network in a form it never trained on.

| Name (registry key, `Registries.NetworkForecasters`) | File | What it says |
|---|---|---|
| `persistence` | `PersistenceForecaster.cs` | today's observations held: downlink quantiles of the last 2 s of frames (by send time for negative horizons), loss from sequence gaps, the 10 s floor, an outage after 250 ms of silence; uplink from round trips minus the echoing frame's downlink delay |
| `rtt-half` | `RttHalfForecaster.cs` | ClockSync's RTT/2 both ways, no spread, loss or outage |

`absent` (no forecast) is no entry: the host builds no forecaster, and the network block stays zeros.

Helpers: `ForecastEffectiveTimes.cs` (send + the uplink median at h = 0, FIFO-clamped: forecast.py's `effective_times`),
`NetworkForecastBlock.cs` (layout.py's `encode_forecast`, used by both learned windows), `LinearPercentile.cs` (numpy's
`percentile`, linear method).

**No benchmark row.** A forecaster is not scored on its own: what it is for is the learned model's error and correction
with it, which the trainer evaluates per level (forecast-quality sweep) and `Teleop.Eval` scores once a model is
exported. The golden is what keeps each copy honest.
