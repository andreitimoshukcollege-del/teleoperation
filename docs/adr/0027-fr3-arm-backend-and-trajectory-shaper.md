# 27. The FR3's arm backend, one trajectory shaper, and session format v2

## Status

Proposed (2026-10-28). It belongs to the FR3 Duo work (ADR 0026) and is the user's "hardware arrives
later" choice made concrete: a simulated arm now, behind the seam a real one will use.

## Context

**On the JetRover, the robot's behaviour between commands is someone else's code.** The ROS
relay's gate and the bus servos' own speed control decide what the arm does with a 48 Hz stream of
absolute targets. The command-aware predictors (ADR 0018, `relay-replay`, `relay-replay-servo`)
work because they reproduce that code. The learned model is a correction on top of one of them
(ADR 0021 decision 7).

**On the FR3, that code is ours to write.** Franka's control interface (FCI, through libfranka or
franka_ros2) takes a joint position, velocity or torque command **every millisecond**, from a
real-time callback.
- **A command that breaks the limits is refused.** If it violates the joint's velocity,
  acceleration or jerk limits, including the position-based velocity limits in the URDF, the robot
  stops with a reflex error.
- **A missed cycle aborts the motion.**

So a 48 Hz teleoperation stream cannot go to the robot as it is. Something has to shape it into a
1 kHz trajectory within the limits, and that shaper decides what the arm does between commands.
Whatever it is, a good prior has to model it exactly.

**The Franka Hand is a different device.** Its `move(width, speed)` and `grasp(...)` calls block
until the fingers finish (0.2 m/s maximum). It cannot take a 1 kHz stream: driving it from a 48 Hz
width needs a newest-wins gate, which is the structure of the JetRover's relay gate again.

**Today's RobotHost cannot host a real FR3 loop.**
- `Program.cs` runs one loop at `Thread.Sleep(5)` (`:554`), on .NET with a garbage collector. That
  cannot feed a 1 kHz real-time callback.
- `GenericArmPlant` works in Hiwonder pulses throughout.
- `IRelayClient` speaks the servo relay's protocol (RelayProtocol v5, pulse units).

**There is no FR3 to test against yet.** The live loop still has to run end to end before one
arrives: the headset, RobotHost, joint-state downlink, recording and the headless verbs.

## Decision

1. **One trajectory shaper, in Core** (`Prediction/JointTrajectoryShaper.cs`).
   - **What it does:** a deterministic, allocation-free, per-joint tracker. Every 1 ms it moves the
     shaped state (position, velocity, acceleration) toward the latest accepted target, in the style
     of libfranka's `limitRate`:
     - a braking-distance velocity reference toward the target;
     - bounded by the joint's velocity limit and its position-based velocity limit
       (`min(v_max, offset + sqrt(2 · deceleration · distance to the limit))`, each side);
     - then acceleration and jerk clamped per step.
   - **Limits:** they come from the `SerialChainProfile` (ADR 0026), with a configurable margin below
     the robot's own limits so the real robot never sees a limit-violating command.
   - **The finger:** a newest-wins gate. A new move starts when the hand is idle and the target is
     more than a deadband from the last move. While a move runs, only the newest target is kept
     pending, and the move runs at a fixed speed.
   - **Two refinements the tests forced** (2026-10-28):
     - as in `limitRate`, the acceleration toward a velocity bound shrinks in proportion to the headroom
       left. Without that, reaching the velocity limit cut the acceleration in one step, which breaks the
       jerk limit;
     - a stop is planned no harder than velocityScale² times the joint's position-based deceleration.
       Joint 2's (2.585 rad/s²) is below the planned braking, and a reference above the position-based
       curve ran the arm into a bound that shrinks as it moves, breaking the jerk limit again.

     With both, a randomized test holds every step of all seven joints within their velocity,
     acceleration, jerk and position limits (20 seeds).
   - **The exact arithmetic:** in the class doc, fixed by its tests, in double precision. It is an unregistered helper
     like `LearnedJointWindow`, not an algorithm under comparison.
   - **Why Core:** it is pure arithmetic of (targets, time), and two things need the *same* copy:
     - the backend, below, which drives the arm with it;
     - the `franka-replay` prior (decision 4), which predicts the arm with it.
   - **Pinned everywhere else:** the trainer's Python copy and the future real-time arm server's copy
     are pinned to it by golden files (`gen-trainer-golden`), as `relay-replay` is today.
2. **Gap policy.** Targets are absolute. When commands stop, the shaper finishes the move to the last
   accepted target under its limits and stays there; it never holds by freezing mid-move, which would
   be a jerk discontinuity on this robot. A target older than the staleness window is not applied,
   the same rule `GenericArmPlant` has today.
3. **A RobotHost-local seam, `IArmBackend`** (`Teleop.RobotHost/Backend/`):
   - `Send(targets in radians, host time)`;
   - `TryReceiveState(...)`, which gives each arm's robot time, measured joint positions and
     velocities, and finger width;
   - `SetEnabled(bool)`.

   A new `SerialChainPlant` (`Teleop.RobotHost/Plant/`) sits upstream of it, in radians. It owns:
   - staleness;
   - limit clamping;
   - the state it reports back.

   `GenericArmPlant` and `IRelayClient` are not generalized: they stay the JetRover's. Two
   implementations:
   - **`Fr3SimBackend`, now.** It runs in-process and steps the shaper in 1 ms substeps up to the
     injected time. It adds a tracking model that stands in for Franka's internal joint controller: a
     per-joint second-order lag, a small gravity/payload bias and encoder noise. Parameters and seed
     are arguments, and its sessions say what they were. It is a stand-in for hardware, so the live
     loop runs end to end; ADR 0021 decision 6 still holds, and the trainer's simulator is what
     generates training data at scale.
   - **`FciArmServerClient`, later, with its own ADR** when the robot arrives. It is a local-socket
     client to a separate real-time process that owns the 1 kHz FCI callbacks and runs the C++ copy
     of the shaper. That ADR decides:
     - libfranka or franka_ros2;
     - one FCI connection per arm;
     - the real-time PC's network and recipes;
     - parking as stop-and-brake.
4. **The prior is `franka-replay`.** It is a command-aware joint predictor (`ICommandAwarePredictor`,
   `Registries.JointPredictors`) that replays the commands it has seen through the same shaper from
   the last measurement, to the target time. On a robot running this backend it is exact up to the
   tracking model, so its error is the tracking lag plus the network.
5. **Hosts dispatch on the profile's `kind`** (ADR 0026 decision 7). A serial-chain RobotHost:
   - runs the joint port only, with no Cartesian endpoint. The joint keepalive already carries
     `ClockSync` (ADR 0018);
   - has arm power, park and relax off until the real-backend ADR defines them;
   - is launched by a new `just fr3-sim-host` recipe on the PC, so the headset and the Eval verbs
     (`joint-drive`, `joint-probe`) can talk to it on the LAN.
6. **Session format v2** (`trainer/contract/SESSION_FORMAT.md`). Readers keep reading v1.
   - **`session.json`:**
     - `"format": "teleop-session"`, `"version": 2`;
     - the profile's name and sha256;
     - `source`: `robot`, `sim-host` (this backend) or `sim` (the trainer).
   - **`robothost.jlog`:**
     - keeps every robot-agnostic record (`KA`, `JC`, `FR`, `SS`, `JS`, `UP`, `OM`, `TK`, `HD`);
     - replaces the relay's `RO`/`RI` with `AO` (targets to the backend, radians) and `AS` (a backend
       state taken by the plant).
   - **`controller.jlog`** replaces `driver.jlog`. It holds the shaper's accepted targets and the
     measured state, decimated to a rate fixed in the spec (4 ms).
   - **The truth** is the measured joint positions there, at the controller's own time stamps.
   - **A `sim-host` session's simulated arm is never the robot's,** and the trainer reports it
     separately. Its operator command streams, from a real human on the headset over a real network,
     are real, and are the first human FR3 data.

## Consequences

- **The prior is exact by construction, so the model learns less on the FR3.** On the JetRover the
  learned correction covered a relay and servos nobody here wrote. On the FR3 it covers the tracking
  lag and the network. That is a research result either way, and ADR 0028's gate is sized for it.
- **The real-time arm server is a future cost, and its correctness is a golden test, not a hope.**
  Its shaper must match Core's on the golden trajectories before it drives an arm.
- **`sim-host` sessions are cheap human data.** The headset can be flown against a simulated FR3 on
  the PC, in the lab, before the robot exists.
- **Nothing here touches the JetRover's path:** not its relay, ROS workspace, sessions (v1) or
  recipes.

## Alternatives considered

- **Generalizing `GenericArmPlant` to radians and many arms.** Rejected. Its belief tracking,
  stepping and gates are pulse-based and servo-specific (`GenericArmPlant.cs:144-169`). A second
  plant is simpler than a plant with two personalities.
- **A separate simulator process speaking a relay protocol.** Rejected for now. It adds a protocol
  to keep in sync before there is a real server to share it with. The Eval benches need the
  simulator in-process anyway. The real-backend ADR defines the socket protocol once, for the real
  server.
- **Sending 48 Hz targets to Franka's own joint impedance controller unshaped.** Rejected. Steps at
  48 Hz violate the jerk limits and cause reflex stops. Hidden internal smoothing would also leave
  the prior modelling something nobody can read.
- **A time-optimal jerk-limited planner (Ruckig-style).** Deferred. It is more exact, but larger
  and much harder to copy bit for bit into Python and C++. The `limitRate`-style tracker is a few
  lines per joint and is what Franka's own examples do.
