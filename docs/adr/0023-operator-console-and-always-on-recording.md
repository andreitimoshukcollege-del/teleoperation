# 23. An operator console on the headset, and recording whenever the robot is commanded

## Status

Accepted (2026-10-26).

The user asked for four things:
- the headset shows which joint predictor draws the twin, and lets the operator switch it;
- network disturbances added from inside the app, live;
- a live evaluation of the predictors;
- the robot records whenever it is commanded.

Their choices:
- disturbances apply **on the headset, in both directions**;
- the live statistics compare **every predictor side by side**, and the operator chooses which are
  computed and shown;
- recordings stay on the robot until `just fetch-recordings` has copied them and the checksums
  match, and are then pruned (`just robot-prune-recordings`).

## Context

- **The predictor is fixed at start-up.** `JetRoverJointStateLink` builds one
  `ICommandAwarePredictor` from `JointPredictorName` in its constructor and keeps it in a `readonly`
  field. Changing it means editing the config, `just push-galaxy-config`, and restarting the app.
  `Registries.JointPredictors` has four entries: `hold-last`, `servo-replay`, `relay-replay` and
  `relay-replay-servo`. The useful variants are parameter sets, not new types: for example the
  2026-10-25 fit (lag 50 ms, pipeline delay 16.8 ms) and the 2026-10-26 fit (40 ms, 3.25 rad/s,
  13.8 ms).
- **Nothing scores a predictor on the headset.** The joint metrics (`docs/metrics.md` §4) are
  computed offline: by `joint-predict-bench` in simulation, and by the trainer on recordings. A
  live comparison is also what an operator needs to judge a switch.
- **The only live disturbance is the added-delay slider.** It holds downlink frames before
  `ObserveState`. Core already has the parts for more:
  - `EmulatedTransport`;
  - delay, jitter, Gilbert-Elliott loss and reorder impairments (ADR 0013);
  - `NetworkProfileCatalog`'s named and parametric profiles.

  The joint socket is deliberately never wrapped.
- **Recording needs an explicit start.** RobotHost and the ROS driver write only while
  `~/recordings/.active` names a session, and only `just record-start` or the onboard pilot writes
  that file.

  The rates involved:
  - a commanded session is about 54 KB/s, roughly 200 MB per hour;
  - with nothing commanding, the driver alone still logs about 14 KB/s.

  Hazards in the current writers:
  - opening a session file truncates it, so a RobotHost restart mid-session erases its log;
  - an I/O error on the writer thread, such as a full disk, kills RobotHost.

  Disk space: the Jetson's card had 1.3 GB free before the 2026-10-26 cleanup and 6.4 GB after.
- **The robot already sees the uplink.** Each `JC` record carries the command's capture time on the
  headset's clock as well as its arrival time on the robot's, and the `KA` keepalives tie the two
  clocks together. A delay added on the headset therefore shows in the robot's own recording. A
  delay added on the downlink does not, and does not need to: the trainer synthesises the downlink
  for every view (ADR 0021 decision 4).

## Decision

### 1. A predictor bank in Core

`JointPredictorBank` in `Prediction/` holds several `ICommandAwarePredictor`s:
- **Fan-out:** every `ObserveCommand` and `ObserveState` goes to every *enabled* member.
- **One active member:** the one that draws the twin.
- **Members are presets**, a name plus a registry entry plus a `JointPredictorConfig`. They are
  listed in the headset config (`JetRoverArmConfig.JointPredictorPresets`), so a new fit is a config
  change, not code.
- **The default set:**
  - `hold-last`;
  - `relay-replay`;
  - `relay-replay-servo (2026-10-25 fit)`, which is today's default and stays the active one;
  - `relay-replay-servo (2026-10-26 fit)`.
- **Switching** changes which enabled member is drawn. It is instant and keeps every member's
  history.
- **Enabling a disabled member** starts it from nothing: it `Reset`s, and then warms up on the
  next commands and readings. The panel marks it "warming up" until it has seen one reading.
- **The active member cannot be disabled.**
- **Construction and allocation:** the bank allocates only when it is built. Feeding and
  predicting allocate nothing (invariant 8), and the allocation tests cover it.
- **The learned model is not a member.** It cannot be until it passes the trainer gate (ADR 0021
  decision 8) and the Phase 7 ADR picks an on-device runtime.

### 2. A live scorer in Core

`LiveJointScorer` in `Metrics/` scores each enabled member over a rolling window, 30 s by default.
It uses the existing definitions, computed at the headset:
- **`joint_prediction_error_deg`:** when a frame brings a new measurement (its sensed stamp is
  later than every earlier one), the scorer first asks each member for its prediction at that
  sensed time, before the frame is observed. The difference to the measured value is the error,
  pooled over the arm joints. The read instant is the frame's sensed stamp, converted to the
  headset's clock by the link's `ClockSync`.
- **`joint_correction_deg`:** at the same instant, `TryPredict(now)` just before and just after the
  observation.
- **`joint_display_jitter_deg`:** the second difference of each *shown* member's prediction, per
  display tick.

Details:
- **Reported:** p50 and p95 per member over the window, and the count of readings scored. The
  gripper is reported on its own, as in the trainer.
- **Not computed live:** `joint_prediction_lead_ms` needs truth from after the prediction. It is
  left out of version 1.
- **Implementation:** fixed-capacity rings and an in-place selection for the percentiles, so nothing
  allocates per frame.
- **These numbers are diagnostics, never results.** `docs/metrics.md` gains a note that the live
  variants exist: same definitions, rolling window, at the headset.

### 3. Network disturbances on the headset, both directions

A `JointLinkImpairment` in Bridge sits between the joint socket and everything that uses it:
- **Uplink:** commands and keepalives are held or dropped before the send.
- **Downlink:** frames are held or dropped after the receive. The added-delay slider becomes this
  stage's downlink delay.

How it works:
- **Building blocks:** Core's `INetworkImpairment`s, built by `NetworkProfileCatalog`. That covers
  the named profiles (`lan`, `50ms-5j`, `150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`) and
  parametric delay, jitter and loss for each direction.
- **Randomness:** an injected seed, logged with each change, so every draw is reproducible.
- **ClockSync's own traffic goes through it too.** The round trip it measures then includes the
  disturbance, as it would on a real network, so the effective times of commands adapt.
- **Every change is logged** to the headset's `.tlog`, with the time, the profile and the seed.

Limits and safety:
- **Caps:** 2 s added delay and 50% loss in either direction.
- **The controls stay local.** The ROBOT ON/OFF button and the clutch decide whether commands are
  *made*, so they are never delayed. Commands already in flight still arrive, as they would on a
  real network.
- **Labelling:** the panel says "network disturbed", in the twin's status line too, whenever any
  disturbance is on.

### 4. Recording whenever the robot is commanded

RobotHost opens a session itself (`AutoSessionRecorder`):
- **When it opens:** on the first joint command after at least 30 s without one, provided
  `~/recordings/.active` is empty.
- **What it writes:** `session.json` (format 1, `source: robot`, a label of `auto`), the builds
  from its own and the driver's `BUILD_INFO`, a copy of the robot profile, and `.active` (with a
  marker that the session is automatic). The driver follows `.active`, as it does today.
- **When it closes:** after 30 s without commands, and in three other cases:
  - **when the command sender or the joint-state subscriber changes**, starting a new session at
    once, because the checker allows one of each per session;
  - **after 60 minutes**, so no file grows past about 200 MB;
  - **when free space falls below 1 GB.** No new session opens below 2 GB, and RobotHost logs why.
- **What it writes on close:**
  - the slider value worked out from the `JC` speeds, as `record-stop` does;
  - the uplink impairment RobotHost itself applied (`UP`);
  - `files.sha256`.
- **Fields nobody on the robot knows** are written as unknown: load `unknown` and operator `auto`
  plus the sender's address. They can be set afterwards on the laptop, as the slider value is
  today.

Precedence and robustness:
- **Manual sessions win.** `record-start` and the onboard pilot first close an automatic session
  (clearing `.active` and waiting for the `END` trailers), then open theirs. While `.active` names
  a manual session, the automatic recorder stays out.
- **Both writers get two fixes:**
  - a session file that already exists is never truncated; a restart opens a new session;
  - an I/O error closes the session cleanly, with its trailer, instead of killing the process (in
    RobotHost) or silently stopping the writer thread (in the driver).
- **Retention:** `just fetch-recordings`, then `just robot-prune-recordings true`, which deletes only
  sessions this machine holds with identical sha256s.

## Consequences

- The headset can compare predictors under the same live conditions, switch between them without a
  restart, and stress them with a chosen disturbance. Every run then becomes recorded robot data,
  without anyone having to remember `record-start`.
- **Core gains two classes**, the bank and the scorer, with unit and allocation tests. They are not
  new algorithms, so invariant 9's registry entry and benchmark row do not apply.
- **Bridge changes, which want human review and an IL2CPP build:**
  - `JetRoverJointStateLink` holds the bank, the scorer and the impairment stage;
  - `JetRoverArmConfig` gains the presets and the impairment caps.
- **XR changes:** a console panel, built by the scene builder:
  - a predictor row with one button per preset;
  - a per-member "computed and shown" toggle;
  - a statistics table;
  - disturbance presets and sliders.

  `XrSceneCheck` learns the new wiring.
- **RobotHost changes:** the automatic recorder, the two writer fixes, a disk-space guard and its
  tests. `record-start` and `onboard.py` learn to close an automatic session first.
- **The driver** gets the I/O-error fix in `session_log.py`.
- **Cost on the headset:** every enabled member predicts once per scored reading (about 50 a
  second). The shown ones also predict once per display tick (90 a second), for 6 joints. That is
  cheap next to rendering, and members the operator turns off cost nothing.
- **Training data:**
  - automatic sessions have load unknown until labelled, and `sysid` keeps filtering on load `none`;
  - a headset-side uplink disturbance is visible in the robot's `JC` timing, but not named in
    `session.json`; the conditions note says so.

## Alternatives considered

- **Disturbances on the robot's uplink only**, through the existing `UplinkImpairedReceiver`. That
  would need a new wire message (an ADR 0018 amendment) and a RobotHost deploy, and leaves the
  downlink untouched. Not chosen, for now.
- **Scoring only the active predictor.** It is simpler, but switching would reset its statistics,
  and the operator could not compare. Not chosen.
- **A rolling size cap that deletes the oldest recordings**, fetched or not. It could lose data that
  never left the robot. Not chosen.
- **Recording continuously, idle time included.** That is about 1.2 GB a day of the driver's idle
  reads, and of little use for training. Not chosen: sessions open on commands.
