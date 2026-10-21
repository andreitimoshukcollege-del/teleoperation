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
| `relay-replay` | `RelayReplayPredictor.cs` | replays the commands through a model of the JetRover relay's per-servo gate: newest-wins pending target during the write cooldown, deadband, timed linear moves of max(min, distance/speed); re-anchors mid-move from the measurement to that move's target; optional per-joint reading age (`Relay*`, `SensedAge*` in `JointPredictorConfig`, defaults the JetRover's gate and no age) |

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

#### 2026-10-19: both robots, error and correction

`joint-predict-bench --robot ideal|relay` (default `ideal`, whose error rows are unchanged from
the table above) now reports `joint_correction_deg` next to the error (docs/metrics.md §4). `relay`
simulates the JetRover's paced relay (robot/ros servo_controller.py with newest-wins pending
targets): the per-servo gate, the 5-pulse deadband, 50 ms minimum linear moves at 2200 pulses/s,
motors read one after another at 10 ms each, the frame stamped 5 ms after the last read, and a
50 Hz stream repeating 20 Hz fresh readings. `servo-replay` gets 6 rad/s, the headset's value.
`relay-replay` gets the relay's true gate parameters and the true per-motor reading ages (45 ms
for motor 1, down to 5 ms for motor 5). 5 seeds × 20 s, degrees.
**Read these as a check of logic under a stated simulation, not a result.** `relay-replay` is
given the simulation's exact gate, so its numbers measure what the model can do when it is
right about the robot. That is all they measure.

`--robot ideal` (true speed 4 rad/s, model speed 5 rad/s):

| profile | predictor | err p50 | err p95 | err p99 | err max | cor p50 | cor p95 | cor p99 | cor max |
|---|---|---|---|---|---|---|---|---|---|
| `lan` | `hold-last` | 0.25 | 1.56 | 2.29 | 4.64 | 0.60 | 2.05 | 2.94 | 4.18 |
| `lan` | `relay-replay` | 0.67 | 3.69 | 5.59 | 9.70 | 0.42 | 3.17 | 4.98 | 8.35 |
| `lan` | `servo-replay` | 0.00 | 0.17 | 0.29 | 1.24 | 0.00 | 0.00 | 0.13 | 1.38 |
| `50ms-5j` | `hold-last` | 1.74 | 6.05 | 8.71 | 14.92 | 0.57 | 2.15 | 3.16 | 4.58 |
| `50ms-5j` | `relay-replay` | 1.40 | 4.80 | 6.60 | 10.14 | 0.00 | 0.23 | 0.57 | 1.19 |
| `50ms-5j` | `servo-replay` | 0.00 | 0.63 | 0.97 | 1.66 | 0.00 | 0.00 | 0.00 | 0.00 |
| `150ms-20j-0.5loss` | `hold-last` | 4.68 | 15.38 | 21.89 | 40.79 | 0.61 | 2.98 | 4.58 | 9.17 |
| `150ms-20j-0.5loss` | `relay-replay` | 1.35 | 4.49 | 6.21 | 10.54 | 0.00 | 0.00 | 0.14 | 1.07 |
| `150ms-20j-0.5loss` | `servo-replay` | 0.18 | 1.50 | 2.46 | 9.50 | 0.00 | 0.00 | 0.00 | 0.00 |
| `300ms-60j-2loss-bursty` | `hold-last` | 7.70 | 26.73 | 38.23 | 68.44 | 0.92 | 5.34 | 9.17 | 18.33 |
| `300ms-60j-2loss-bursty` | `relay-replay` | 1.04 | 4.34 | 6.33 | 12.68 | 0.00 | 0.00 | 0.11 | 1.05 |
| `300ms-60j-2loss-bursty` | `servo-replay` | 0.63 | 3.70 | 6.68 | 17.53 | 0.00 | 0.00 | 0.00 | 0.00 |

`--robot relay`:

| profile | predictor | err p50 | err p95 | err p99 | err max | cor p50 | cor p95 | cor p99 | cor max |
|---|---|---|---|---|---|---|---|---|---|
| `lan` | `hold-last` | 1.44 | 5.63 | 8.92 | 18.71 | 1.20 | 4.32 | 6.48 | 10.08 |
| `lan` | `relay-replay` | 0.16 | 0.76 | 1.20 | 3.10 | 0.00 | 0.35 | 0.72 | 1.62 |
| `lan` | `servo-replay` | 1.17 | 3.72 | 5.26 | 9.27 | 0.00 | 3.69 | 7.21 | 14.09 |
| `50ms-5j` | `hold-last` | 2.66 | 9.26 | 14.41 | 28.49 | 1.20 | 4.32 | 6.48 | 10.56 |
| `50ms-5j` | `relay-replay` | 0.20 | 0.82 | 1.22 | 3.43 | 0.00 | 0.07 | 0.48 | 1.28 |
| `50ms-5j` | `servo-replay` | 1.20 | 4.07 | 5.91 | 10.89 | 0.00 | 0.00 | 0.00 | 7.04 |
| `150ms-20j-0.5loss` | `hold-last` | 5.30 | 16.99 | 27.44 | 52.14 | 1.44 | 4.56 | 7.20 | 15.60 |
| `150ms-20j-0.5loss` | `relay-replay` | 0.31 | 1.19 | 1.85 | 5.35 | 0.00 | 0.00 | 0.36 | 1.20 |
| `150ms-20j-0.5loss` | `servo-replay` | 1.21 | 4.19 | 6.60 | 14.45 | 0.00 | 0.00 | 0.00 | 2.62 |
| `300ms-60j-2loss-bursty` | `hold-last` | 8.75 | 29.10 | 42.53 | 74.16 | 1.44 | 5.76 | 10.08 | 25.92 |
| `300ms-60j-2loss-bursty` | `relay-replay` | 0.67 | 2.95 | 4.88 | 11.47 | 0.00 | 0.00 | 0.00 | 1.20 |
| `300ms-60j-2loss-bursty` | `servo-replay` | 0.77 | 3.37 | 5.45 | 11.80 | 0.00 | 0.00 | 0.00 | 0.00 |

What these say, within the simulation:

- **The sawtooth is there, and only at short horizons.** Under `relay` on `lan`, `servo-replay`
  jumps by up to 14° (p95 3.7°) when a fresh reading arrives. On a constant 1.5 rad/s ramp
  through the same gate (a scratch check, not in the bench), every fresh reading moved its
  prediction *backward*, by 3.6 to 5.5°, 20 times a second. Its model has each command take
  effect on arrival, while the relay writes a target up to one cooldown later and spends at
  least 50 ms reaching it, so the model runs ahead. At 50 ms and more of latency the jump
  vanishes: from an older measurement, the 6 rad/s replay has caught up with the latest target
  whatever it started from. The lead is still there, as error.
- **`relay-replay` cuts both** under `relay`: on `lan`, error p95 from 3.72 to 0.76 and correction
  p95 from 3.69 to 0.35. The reading age accounts for part of it. With
  `--model-sensed-age off`, `lan` gives error p95 1.02 and correction p95 1.47 (max 7.37). What
  remains at p95 is about 1.5 pulses of reading quantization (one pulse is 0.24°).
- **Under `ideal` it is worse than `servo-replay`**, as it should be: it models a gate that robot
  does not have. Choose by the robot, not by this table.
- **The move speed is not exercised.** The bench's waypoints never ask more than about 0.2 rad
  per 50 ms, which the relay covers in under 50 ms. So every move lasts the minimum, and
  `--relay-pps 1800` or `--model-relay-pps 1800` leave every p50 to p99 unchanged. Only one max
  moved, by 0.35°. The speed parameter needs a faster trajectory, or recordings.
- **Not modelled:** the relay flushes pending targets between feedback reads, up to about 10 ms
  after the cooldown ends. The executor handles commands in bursts between polls. Servo
  dynamics, load and stall. Clock-sync error. On the robot, each of these leaves a residual
  that this table cannot show.

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
