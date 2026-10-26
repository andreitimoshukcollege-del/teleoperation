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
| `relay-replay-servo` | `RelayReplayServoPredictor.cs` | `relay-replay`'s gate and replay as the servo's setpoint, with every command reaching the gate a constant pipeline delay late and a servo's first move sized from its sensed measurement; then a servo filter run from the anchor: first-order lag and speed cap (times a joint's speed scale), fixed steps plus a last partial one, in double (`Servo*` in `JointPredictorConfig`; the defaults, 5 rad/s, 20 ms, 20 ms, on, 2 ms, are guesses until fitted). Neutral parameters (no cap, no lag, no delay, first move off) reproduce `relay-replay` bit for bit. The exact model is in the class doc; `trainer/contract/golden/relay-replay-servo.json` pins it for a Python copy |

**Not a predictor: `JointPredictorBank.cs`** (docs/adr/0023). Several presets (a label, a registry
key and a config) fed the same commands and readings, one of them answering; switching keeps every
member's history, enabling a member resets it. The headset uses it to compare and switch predictors
live. It deliberately does not implement `ICommandAwarePredictor` (it is a container, and `audit`
requires every implementation to be registered), so it has no registry entry or benchmark row.

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

#### 2026-10-21: a servo behind the relay (`relay-replay-servo`)

`joint-predict-bench --robot relay` can now put a servo behind the relay's setpoint:
`--servo-max-speed` (rad/s) and `--servo-lag-ms` give it a speed cap and a first-order lag,
stepped at the bench's 1 ms with the setpoint sampled at each step's end (`relay-replay-servo`'s
own scheme). `--pipeline-ms` delays every arrived command by a constant before the staleness check
and the gate, and `--leg-scale` shortens the operator's waypoint legs. All default to off, and with
them off every existing row is unchanged (checked against both 2026-10-19 tables). The bench adds a
`relay-replay-servo` row, which with the servo off equals `relay-replay`'s on every profile.
`relay-replay-servo` is given the simulation's true cap, lag and pipeline delay, and no first move
sized from the measurement (the simulated relay's first write lasts the minimum). Each move still
starts from the setpoint, not from the lagging servo. 5 seeds × 20 s, degrees.
**Read these as a check of logic under a stated simulation, not a result.** The simulated servo has
exactly the model's structure and parameters, so the numbers measure the model when it is right
about the servo. That is all they measure.

`--robot relay --servo-max-speed 5 --servo-lag-ms 20` (no pipeline delay):

| profile | predictor | err p50 | err p95 | err p99 | err max | cor p50 | cor p95 | cor p99 | cor max |
|---|---|---|---|---|---|---|---|---|---|
| `lan` | `hold-last` | 1.41 | 5.59 | 8.94 | 18.63 | 1.20 | 4.08 | 6.48 | 9.60 |
| `lan` | `relay-replay` | 0.49 | 1.57 | 2.46 | 5.36 | 0.00 | 0.58 | 1.00 | 2.56 |
| `lan` | `relay-replay-servo` | 0.16 | 0.71 | 1.11 | 2.47 | 0.08 | 0.57 | 0.89 | 2.04 |
| `lan` | `servo-replay` | 1.57 | 4.96 | 6.86 | 12.50 | 0.00 | 5.14 | 8.94 | 14.81 |
| `50ms-5j` | `hold-last` | 2.65 | 9.23 | 14.25 | 27.96 | 1.20 | 4.08 | 6.48 | 10.32 |
| `50ms-5j` | `relay-replay` | 0.50 | 1.65 | 2.47 | 5.68 | 0.00 | 0.14 | 0.53 | 1.20 |
| `50ms-5j` | `relay-replay-servo` | 0.17 | 0.71 | 1.14 | 2.57 | 0.01 | 0.23 | 0.60 | 1.46 |
| `50ms-5j` | `servo-replay` | 1.67 | 5.56 | 8.10 | 13.76 | 0.00 | 0.00 | 0.00 | 9.92 |
| `150ms-20j-0.5loss` | `hold-last` | 5.28 | 16.99 | 27.35 | 52.13 | 1.44 | 4.32 | 6.96 | 13.44 |
| `150ms-20j-0.5loss` | `relay-replay` | 0.45 | 1.82 | 2.99 | 6.98 | 0.00 | 0.00 | 0.44 | 1.44 |
| `150ms-20j-0.5loss` | `relay-replay-servo` | 0.27 | 1.04 | 1.62 | 3.97 | 0.00 | 0.01 | 0.48 | 1.44 |
| `150ms-20j-0.5loss` | `servo-replay` | 1.72 | 5.68 | 9.05 | 17.22 | 0.00 | 0.00 | 0.00 | 4.30 |
| `300ms-60j-2loss-bursty` | `hold-last` | 8.78 | 28.97 | 42.26 | 73.72 | 1.44 | 5.76 | 9.60 | 24.96 |
| `300ms-60j-2loss-bursty` | `relay-replay` | 0.45 | 1.91 | 3.17 | 7.54 | 0.00 | 0.00 | 0.00 | 0.96 |
| `300ms-60j-2loss-bursty` | `relay-replay-servo` | 0.63 | 2.78 | 4.58 | 10.08 | 0.00 | 0.00 | 0.01 | 0.96 |
| `300ms-60j-2loss-bursty` | `servo-replay` | 1.23 | 4.75 | 7.38 | 14.48 | 0.00 | 0.00 | 0.00 | 0.00 |

The same with `--pipeline-ms 20`, and then with `--leg-scale 0.25` as well (legs of 0.125–0.375 s);
the other rows are left out:

| profile | predictor | err p50 | err p95 | err p99 | err max | cor p50 | cor p95 | cor p99 | cor max |
|---|---|---|---|---|---|---|---|---|---|
| `lan` | `relay-replay` | 0.98 | 3.14 | 4.83 | 8.50 | 0.00 | 0.99 | 1.90 | 4.72 |
| `lan` | `relay-replay-servo` | 0.15 | 0.70 | 1.08 | 2.20 | 0.07 | 0.50 | 0.82 | 1.57 |
| `50ms-5j` | `relay-replay` | 1.01 | 3.23 | 4.91 | 8.69 | 0.00 | 0.23 | 0.75 | 2.40 |
| `50ms-5j` | `relay-replay-servo` | 0.17 | 0.72 | 1.15 | 2.55 | 0.01 | 0.23 | 0.66 | 2.19 |
| `150ms-20j-0.5loss` | `relay-replay` | 0.97 | 3.31 | 5.39 | 11.82 | 0.00 | 0.00 | 0.51 | 2.16 |
| `150ms-20j-0.5loss` | `relay-replay-servo` | 0.27 | 1.05 | 1.64 | 3.96 | 0.00 | 0.01 | 0.48 | 2.16 |
| `300ms-60j-2loss-bursty` | `relay-replay` | 0.52 | 2.54 | 4.25 | 11.06 | 0.00 | 0.00 | 0.00 | 1.44 |
| `300ms-60j-2loss-bursty` | `relay-replay-servo` | 0.63 | 2.78 | 4.59 | 10.13 | 0.00 | 0.00 | 0.02 | 1.20 |
| `lan`, fast legs | `relay-replay` | 4.08 | 18.11 | 33.02 | 49.97 | 0.00 | 4.02 | 11.41 | 43.97 |
| `lan`, fast legs | `relay-replay-servo` | 0.30 | 2.11 | 3.58 | 22.05 | 0.21 | 1.55 | 3.13 | 14.40 |
| `50ms-5j`, fast legs | `relay-replay` | 4.18 | 18.63 | 33.69 | 51.14 | 0.00 | 0.00 | 0.90 | 37.77 |
| `50ms-5j`, fast legs | `relay-replay-servo` | 0.40 | 2.27 | 4.14 | 28.32 | 0.03 | 0.79 | 2.42 | 22.69 |
| `150ms-20j-0.5loss`, fast legs | `relay-replay` | 3.83 | 19.29 | 32.72 | 55.40 | 0.00 | 0.00 | 0.00 | 30.67 |
| `150ms-20j-0.5loss`, fast legs | `relay-replay-servo` | 0.72 | 3.74 | 5.81 | 14.89 | 0.00 | 0.29 | 2.11 | 14.96 |
| `300ms-60j-2loss-bursty`, fast legs | `relay-replay` | 2.09 | 16.63 | 30.54 | 60.91 | 0.00 | 0.00 | 0.00 | 15.35 |
| `300ms-60j-2loss-bursty`, fast legs | `relay-replay-servo` | 2.06 | 9.48 | 14.41 | 23.24 | 0.00 | 0.02 | 0.39 | 19.19 |

What these say, within the simulation:

- **Given the true servo, `relay-replay-servo` about halves `relay-replay`'s error** up to 150 ms (p95
  0.71 against 1.57 on `lan`), at a similar correction (p95 0.57 against 0.58 on `lan`, though p50
  rises from 0.00 to 0.08). `relay-replay` predicts the setpoint, and the servo is 20 ms behind it.
- **A pipeline delay costs `relay-replay` and not the model told about it.** 20 ms of it doubles
  `relay-replay`'s p95 on `lan` (1.57 to 3.14), and leaves `relay-replay-servo`'s at 0.70.
- **At 300 ms it is worse than `relay-replay`** (p95 2.78 against 1.91), because the simulated network
  runs the robot early. With ±60 ms of jitter, the staleness check keeps whichever newest command
  arrives first, so the command the robot holds leads send + base delay by about 14 ms on average
  (median 21 ms, p95 63 ms; a scratch replay of the bench's uplink logic, which finds no lead on the
  other three profiles). `relay-replay`'s unmodelled 20 ms lag cancels most of that lead.
  `relay-replay-servo` models the lag, and keeps the lead as error. Neither models the lead, which
  belongs to the host's effective time (send + RTT/2), not to a servo model.
- **The cap only matters with fast motion.** With the default waypoints the simulated cap binds in
  0.01% of 1 ms joint-steps. With `--leg-scale 0.25` (up to 16 rad/s asked) it binds in 14%, and
  `relay-replay`'s p95 is 16.6–19.3° across the profiles against `relay-replay-servo`'s 2.1–9.5°.
- **The tail stays large with fast motion** (max 22° on `lan`). The model keeps `relay-replay`'s
  re-anchoring, as specified, and that is one mechanism that gives errors this size (not isolated
  here). A measurement taken after the relay's move ended is read as the servo having stopped there, so
  the setpoint holds the measured value until the next write while the capped servo is still moving.
  The filter also restarts at each anchor from the measured position with no velocity.
- **Under `--robot ideal`** `relay-replay-servo` runs on its default guesses. It models a lag, a
  delay and a gate that robot does not have, and is worse than `relay-replay` beyond `lan` (p95 7.88
  against 4.80 on `50ms-5j`). Choose by the robot, not by this table.
- **Not modelled**, on top of the list above: a pipeline delay that varies (on the robot it is a
  loop, a poll and a cycle, not a constant), servo dynamics beyond a first-order lag and a symmetric
  cap, a different lag or cap per joint or per direction, and load. The servo's parameters are guesses
  until fitted from recordings (trainer `armpred sysid`, docs/adr/0021).

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
