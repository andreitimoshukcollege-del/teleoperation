# Prediction

Implementations of `Contracts/IPredictor.cs`. A predictor estimates state at a **future**
target time from stale observations.

```csharp
public interface IPredictor<TState> {
    void Observe(Stamped<TState> obs);      // authoritative sample arrived
    TState Predict(long targetTicks);       // estimate at target time
    void Reset();
    PredictorDiagnostics Diagnostics { get; }
}
```

## Two distinct problems — do not conflate them

- **Operator-side:** the robot state you have is stale; predict where the robot is *now* so
  the proxy renders in the right place.
- **Robot-side:** the commands you have are stale; predict what the operator wants *now*.

They have different signal statistics (human motion vs. robot dynamics), different failure
consequences, and often want different implementations. A predictor may serve both, but
benchmark it separately on each.

## Implemented

| Name (registry key) | File | Notes |
|---|---|---|
| `none` | `PassthroughPredictor.cs` | returns last observation; the baseline everything is measured against |
| `const-vel` | `ConstantVelocityPredictor.cs` | first-order dead reckoning |
| `double-exp` | `DoubleExponentialPredictor.cs` | Kalman-free, two parameters, strong baseline for head/hand pose |

Keep this table current — it is how the next session avoids reimplementing something. It
previously overclaimed `const-accel`/`ekf`/`seq-model` as implemented; they are **planned, not
built** — see below.

### Joint space, command-aware (`ICommandAwarePredictor`, `Registries.JointPredictors`; docs/adr/0018)

These see the commands sent as well as the stale measurement. Their state is joint values
(`JointSample`), not a `Pose`.

| Name (registry key) | File | Notes |
|---|---|---|
| `hold-last` | `HoldLastJointPredictor.cs` | the last measured joints, unchanged; the joint-space baseline |
| `servo-replay` | `ServoReplayPredictor.cs` | replays the commands sent since the measurement through a bounded-speed servo model (one parameter, `MaxJointSpeedRadPerSec`) |

Benchmark: `dotnet run --project Teleop.Eval -- joint-predict-bench`, which is deterministic. Its
simulation and assumptions are stated in `JointPredictBenchCommand`'s doc. On 2026-10-12, with 5
seeds × 20 s, a true speed of 4 rad/s and a model speed of 5 rad/s, the p95 joint error in degrees
was:

| Profile | `hold-last` | `servo-replay` |
|---|---|---|
| `lan` | 1.56 | 0.17 |
| `50ms-5j` | 6.05 | 0.63 |
| `150ms-20j-0.5loss` | 15.38 | 1.50 |
| `300ms-60j-2loss-bursty` | 26.73 | 3.70 |

**Read these as a check that the predictor's logic works, not as a result.** The simulated servo
matches the model's own assumption apart from its speed, and the operator's clock is perfect.
What the real robot does is measured from recordings (docs/metrics.md
`joint_prediction_error_deg`).

## Planned, not yet implemented

| Name (registry key) | File | Notes |
|---|---|---|
| `const-accel` | `ConstantAccelPredictor.cs` | second-order; overshoots on direction reversal |
| `ekf` | `ExtendedKalmanPredictor.cs` | reports covariance; the reconciler can use it |
| `seq-model` | `SequenceModelPredictor.cs` | uses `IInferenceBackend`, never an ONNX library directly |

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise.

## Tried and rejected

Record failures here with a link to the `results/` directory. This section is as valuable as
the one above; without it, ideas get retried indefinitely.

- *(none yet)*

## Requirements for a new predictor

1. Deterministic: same observations + same target times => bit-identical output.
2. Robust to out-of-order and duplicate `Observe` calls, and to gaps of several hundred ms.
   Real traces contain all three; a predictor that assumes monotonic arrival will silently
   produce garbage rather than fail.
3. No allocation in `Predict`. Preallocate in the constructor.
4. Parameters come from `PredictorConfig`. No magic numbers in the body.
5. `Diagnostics` exposes at minimum the horizon actually extrapolated and, where meaningful,
   an uncertainty estimate.
6. Benchmarked at horizons 50/100/200/400 ms across all profiles in
   `core/testdata/traces/`, reported as **percentiles, not means** — the tail is what the
   operator notices.

## Evaluation note

Prediction error is scored **counterfactually** and offline: at time *t* the predictor was
asked for *t+Δ*; when ground truth for *t+Δ* arrives in the recording, log the error. That is
why any predictor can be scored against a `.tlog` without a robot, a headset, or a network.
Never add a scoring path that requires live hardware.

Accuracy alone is not the objective. A predictor that wins on error while producing constant
micro-corrections is a worse system — always report correction cost alongside.
