# 18. A joint-state downlink, a command-aware joint predictor, and a three-arm twin

## Status

Accepted (2026-10-12). The user's choices:
- a new joint-state stream from the robot, over reconstructing the arm from today's wrist position;
- a command-aware predictor, over extrapolating the wrist pose with the existing predictors;
- three arms in the headset, each switchable in the scene: predicted (solid), commanded (ghost) and
  last measured (faint).

## Context

The Galaxy XR twin shows the operator's **command**: the IK of the clutch target, applied every
frame. Nothing in the scene shows where the robot actually is, or where it probably is now.

What the robot sends back today (`RobotStateFrameCodec`, 57 bytes) cannot support a real twin:
- **Too little.** It sends only the wrist position, from base yaw, shoulder and elbow, with an
  identity rotation. Wrist pitch, roll and gripper are never reported.
- **Not always.** A frame goes out only in reply to a Cartesian command. With the motion button
  off (ADR 0017), nothing comes back at all.
- **Badly timed.** Its stamp is when the frame was sent, not when the servos were read.

The existing predictors (`const-vel`, `double-exp`) are `IPredictor<Pose>`, and they extrapolate a
trajectory. This robot's motion, however, is mostly decided by what it was last told:
- a bus servo drives toward its target at a bounded speed;
- the operator knows every target it sent, and when.

A predictor that ignores that information throws away the best signal there is.

## Decision

### 1. A joint-state downlink on the joint port

`Teleop.RobotHost` streams a **`JointStateFrame`** at **50 Hz** to whoever last sent a joint-port
keepalive. It does so whether or not commands are arriving.
- **Codec:** `Teleop.RobotArm.Wire.JointStateCodec`, little-endian, version 1:

  ```
  u8  version (1)        u32 sequence
  i64 robotSendTicks     i64 sensedTicks        (robot clock: when the relay feedback was read)
  i64 echoOperatorTicks  i64 echoRobotRecvTicks (the latest keepalive, for ClockSync)
  i64 ticksPerSecond     u8  count
  count × { u8 motorId, u8 flags (bit0 sensed), f32 value }
  ```

  - **Values.** `value` uses `JointTarget.Angle`'s units: radians for arm joints and the wrist
    roll, a 0–1 fraction for the gripper.
  - **Sensed flag.** A joint without feedback reports its last commanded belief with `sensed`
    clear. The gripper is one, because it is open-loop.
  - **Size.** For six joints the frame is 46 + 6 × 6 = 82 bytes, inside the 128-byte cap.
- **Keepalive:** a 17-byte subscribe datagram (`JointStateCodec.TryEncodeSubscribe`). Its first
  byte, `0x80`, is unused by any command version. It carries the operator's send ticks and tick
  rate, and is sent every 0.5 s.
  - The robot streams for 3 s after the last keepalive.
  - Every frame echoes the latest keepalive's ticks. The operator therefore runs its own
    `ClockSync` from these frames, and it works with motion off.
- **The plant exposes what it knows:** `GenericArmPlant.ReadJointState` returns each joint's
  sensed (or believed) angle and the robot tick of the last feedback datagram. Its
  `ZeroOffsetRadians` is removed, as `State` already does.

The 57-byte Cartesian reply is unchanged (ADR 0009), so `move-arm`, `clocksync-check` and the
recorded metrics keep working.

### 2. A command-aware joint predictor (Core)

The state type is new: `JointState`, a fixed set of joint values keyed by motor id. So is the
input, because the predictor is also told the commands. Both need a new contract,
`Contracts/ICommandAwarePredictor.cs`:

```
void ObserveCommand(long sendTicks, ReadOnlySpan<JointTarget> targets)   // operator clock
void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)    // operator clock
bool TryPredict(long targetTicks, Span<JointSample> output)
```

The first implementation is **`ServoReplayPredictor`** (`Prediction/ServoReplayPredictor.cs`,
registered as `servo-replay`):
- **Start** from the last measured state, at its sensed time.
- **Replay** every command sent since. Each takes effect at
  `sendTicks + estimated uplink delay`, where the delay is half the `ClockSync` round trip.
- **Move** each joint toward the latest effective target at no more than its maximum speed
  (`MaxJointSpeedRadPerSec`, default 5 rad/s, about HTD-35H no-load speed; to be fitted from
  recordings).
- **Gripper:** it has no feedback, so the predicted gripper is the commanded fraction.
- **Memory:** commands are kept in a fixed ring of 64 entries, about 1.3 s at 48 Hz. A
  measurement older than the oldest command still in the ring predicts no further than the ring
  allows, and the predictor says so in its diagnostics.

**Baseline**, per docs/metrics.md §8: `hold-last`, the last measured state unchanged. That is the
"no prediction" row.

Both are allocation-free and deterministic, and both have unit tests. They are registered in
their own table, `Registries.JointPredictors`, which `audit`'s registry-completeness check covers.
**The benchmark row** comes from `Teleop.Eval joint-predict-bench`:
- **What it simulates:** a servo arm moving at a known speed toward a scripted command stream.
- **The network:** a frozen network profile delays both directions.
- **What it scores:** each predictor's per-joint error against the simulation's ground truth, on
  the same seeds.

Real-robot scoring needs recordings; see §5.

### 3. Three arms, and in-scene switches (TeleopXR)

- **Commanded:** the existing `JetRoverArmRig`, driven by the clutch target as today. Its twin
  visuals become a translucent **ghost**.
- **Predicted:** a second set of pivots with solid twin visuals, driven by the predictor at
  `onBeforeRender`, which is the last hook before photons (unity/CLAUDE.md).
- **Last measured:** a third set with faint visuals, driven by the newest received frame and not
  predicted. The gap between it and the predicted arm is the latency being hidden.
- **What each arm shows of the wrist:** the predicted and measured arms take wrist pitch and roll
  from the joint values directly; no IK. Their gripper shows the commanded fraction, marked as
  not sensed.
- **The chassis is shared and solid.** It does not move.
- **Switches.** Three fingertip switches next to the motion button (*PREDICTED*, *COMMANDED*,
  *MEASURED*) show or hide each arm. They use the same fingertip press as the motion button, with
  no ray or far pinch. All three are on at launch.
- **No state yet.** Until the first frame arrives, the predicted and measured arms are hidden, and
  the display caption says `NO ROBOT STATE`.

### 4. Where the code lives

**Core** (`Teleop.Core` and `Teleop.RobotArm`): the codec, `JointState`, the contract, both
predictors, the registry entries and their tests.

**Bridge:** `JetRoverJointStateBridge`. It owns the keepalive, the receive loop, its own
`ClockSync`, the predictor, and the command feed (the bridge's joint commands). It applies the
results to the predicted and measured rigs.

**`JetRoverArmRig`** gains an `ApplyJointState` helper so any rig can take a full joint state:
- base yaw, the three pitches;
- the wrist roll, through a roll pivot;
- the gripper, through finger transforms.

The roll pivot and finger transforms are optional fields, so TeleopVR is unaffected.

### 5. Recording (decided, not yet built)

**Not in the first implementation (2026-10-12).** Until it exists, real-robot prediction error
cannot be scored, only the simulated benchmark.

The Bridge appends every received frame, every sent joint command and every displayed prediction
to `jetrover-joints-<stamp>.csv`, next to the session's `.tlog`. This gives the offline scoring in
`analysis/` real data to replay. The metric is defined in docs/metrics.md in the same change:
`joint_prediction_error_deg`, the per-joint absolute error between the prediction made for time
*t* and the measurement sensed at *t*. Scoring real sessions is a follow-up. Until then, only the
simulated benchmark row exists.

### 6. Amendment (2026-10-12): unmeasured joints, an added-delay control, and the speed request

- **The first stream showed motors 5 (wrist roll) and 10 (gripper) unsensed.** `just joint-probe`
  found that the relay sends no feedback for them. Their values are RobotHost's commanded belief.
  - **The predicted arm:** `servo-replay` keeps its own replayed estimate for an unsensed,
    non-instant joint across measurements, instead of snapping to the belief. So the roll moves
    at servo speed there too.
  - **The measured arm:** it hides its gripper while the roll is unsensed.
- **Verified on the robot (2026-10-12).** With `just robot-set-feedback-ids`, the relay reads
  motors 1–5 and 10, and `just joint-probe` then shows motor 5 (wrist roll) measured.
  - **Motor 10 is the gripper, but it is never measured.** A `just move-arm … gripper=1` / `=0`
    closed and opened it, so the mapping is right. Its servo does not answer position reads: the
    relay's `getRawPos` returns nothing, and the read is skipped.
  - **So the gripper stays a commanded belief.** The predictor moves it at servo speed (a scaled
    joint), and the measured arm shows it only alongside the measured roll.
- **Added delay.** `JetRoverJointStateLink.AddedDelayMs`, a headset slider from 0 to 400 ms, holds
  frames back before they become the measured arm and reach the predictor.
  - Clock sync uses the real arrival, and commands are not delayed.
  - It exists because the LAN round trip is about 2 ms, which leaves prediction nothing visible to
    do. A session recorded with a delay set is not a real-network measurement.
- **Speed.** Joint commands carry a requested joint speed in their per-joint `Speed` field.
  - RobotHost caps it at its deployed maximum and uses it for both command paths.
  - `ICommandAwarePredictor.ObserveCommand` receives the same speed.

## Consequences

- **RobotHost must be redeployed** (`just deploy-robothost`). An older RobotHost ignores the
  keepalive, so the app shows `NO ROBOT STATE` and only the commanded arm. Nothing breaks.
- **One more stream on the link,** at 50 × 82 bytes, about 4.1 kB/s.
- **The predictor is a model of this robot's servos.** Its one speed parameter is a guess until it
  is fitted from recordings. A mismatch shows as the predicted arm leading or lagging the measured
  one, and the recorded CSV is what fits it.
- **The wrist pitch and roll are reported for the first time.** Whether the relay actually reads
  motors 4 and 5 is unverified. The `sensed` flag shows it, and the measured arm's wrist stays at
  the commanded value when it is clear.
- **No reconciler in joint space yet.** The predicted arm snaps to each new prediction. Because the
  model replays known commands, its corrections should be small, and the recordings will show
  whether a reconciler is needed. Adding one is a new `IReconciler<JointState>` with its own ADR
  note.

## Alternatives considered

- **Reconstruct the arm from today's wrist position by IK.** No robot change, but it loses wrist
  pitch, roll and gripper, and has no data while motion is off.
- **Predict the wrist pose with `const-vel` / `double-exp` and IK it.** This reuses existing code,
  but it is blind to commands that were already sent, and those commands are the strongest
  predictor of this robot's next motion.
- **Put joints into the existing 57-byte frame.** That breaks ADR 0009's frozen format and every
  tool that reads it, and the frame would still only flow while commands do.
