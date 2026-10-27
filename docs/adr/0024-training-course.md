# 24. A training course on the headset, and rounds labelled in the recordings

## Status

Accepted (2026-10-26).

The user asked for something to do while recording data for the learned joint predictor (ADR 0021),
instead of operating the robot aimlessly for hours. Their choices:
- **virtual targets first**; pick-and-place with real objects later, once the gripper is calibrated;
- **the course sets the arm speed** for each round, steering toward the speed with the least data;
- **rounds are labelled in the robot's recordings**, so training can split and stratify by them.

## Context

- **A fine-tune needs coverage, not hours.** About 3-5 hours of headset driving spread over the
  slider speeds (and later loads), plus about an hour recorded after the model is frozen and kept
  for testing. As of 2026-10-26 there are 39 minutes of real data, 13 of them headset driving.
- **What the simulator assumes the operator does** (`trainer/armpred/sim/operator.py`): free
  motion, reaching to marks, slow tracing, fast reversals and holding. Recorded human driving is
  meant to replace it. A course that asks for exactly these motions makes the recordings comparable
  with the simulator's sessions.
- **The robot records every headset session** (ADR 0023 decision 4), but a session says nothing
  about what the operator was doing. `session.json`'s `conditions` are set by `just record-start`;
  an automatic session's are `auto`/`unknown`.
- **The joint port already carries an operator message** (tag `0x81`, ADR 0018 §9): the headset's
  kind and motion mode, once a second. Every decoder on the port checks its first byte exactly, so a
  new tag is ignored by every older reader.
- **`.jlog` readers group records by tag** (`armpred/contract/jlog.py`), so a new record type is
  invisible to an older reader.

## Decision

1. **The course runs on the headset, over the twin.** Targets appear in the robot's workspace, in
   the arm's base frame; the operator drives the arm with the clutch as usual. What scores is the
   **commanded wrist** (the clutch's marker, after the collision-aware IK): it is what the operator
   controls, and every round still drives the real arm.
2. **Three tasks**, in turn:
   - **Reach:** eight targets, one at a time; hold the wrist within 2.5 cm for 0.4 s. Half the targets
     also ask for the gripper open or closed. Score: targets reached, and the time each took.
   - **Trace:** a closed curve through reachable points, with a guide bead moving along it; keep the
     wrist within 3 cm of the bead for 45 s. Score: the share of time inside.
   - **Shuttle:** two targets 15-30 cm apart; touch them alternately, as fast as possible, for 30 s.
     Score: touches.
3. **Every target is reachable by construction.** `Teleop.RobotArm`'s sampler draws joint angles
   inside the collision model's ranges less a margin, takes the wrist position from the forward
   kinematics, and keeps it only if the same `ArmSolver` the headset uses reaches it exactly (or with
   the pitch tilted) from the model's home pose, above the clutch's floor. A trace checks every 2 cm
   of its curve the same way. Draws use Core's `SeededRng`; the seed is logged.
4. **The course sets the speed.** Each round uses one level: 1.0, 2.5, 3.8 or 6.0 rad/s, or
   *varied* (a new level every 8-15 s). It picks the level with the fewest seconds so far for the
   current purpose; the counts persist on the headset. While a round runs, the arm-speed slider
   follows the course and cannot be moved.
5. **A round runs only while the robot is on and holding.** Paused, off or waiting for torque, its
   clock stops (`suspended`).
6. **Purpose: train or test.** A button on the course switches it. Test sessions are meant to be
   recorded only after a model is frozen, and never trained on.
7. **The headset sends the course state** (tag `0x83`, operator → robot, on the joint port) once a
   second and at every change. It is state, not events, so a lost datagram costs at most a second.
   RobotHost writes a `TK` record when it changes (score aside), and again at the start of every
   session while a round is going, as it does `OM`.
8. **Anything recorded while the headset's network disturbance or added delay is on is flagged**
   (`disturbed`): the trainer synthesises the network, so those sessions are not real-network
   measurements.
9. **The logic is headless.** Sampling, the round state machine, scoring and the speed schedule
   live in `Teleop.RobotArm/Training/` with an injected time and xUnit tests. Bridge adapts them to
   Unity's frames; `TeleopXR` only draws and reads the buttons.

### The task message (operator → robot, 34 bytes, little-endian)

`u8 0x83 | u8 version (1) | u8 task | u8 phase | u8 purpose | u8 flags | u32 courseId | u32 round |
f32 speedLevel | f32 score | u32 sequence | i64 operatorSendTicks`

- `task`: 0 none, 1 reach, 2 trace, 3 shuttle.
- `phase`: 0 idle, 1 active, 2 suspended, 3 done (the round's final score).
- `purpose`: 0 train, 1 test.
- `flags`: bit 0 `disturbed`.
- `courseId`: the course's seed, new each time the course starts; `round` counts from 1 within it.
- `speedLevel`: rad/s, NaN for *varied*. Each joint command still carries the speed actually sent.

### The record (`robothost.jlog`)

`TK|t|courseId|round|task|phase|purpose|speedLevel|score|disturbed`, with names (`reach`, `active`,
`train`, ...) and `varied` for a NaN level; `trainer/contract/SESSION_FORMAT.md` has the details.

## Consequences

- An older RobotHost ignores the message, and an older trainer ignores `TK`; neither needs a
  version change. Deploy RobotHost (`just deploy-robothost`) to record the labels.
- The trainer reads `TK` into `Session.rounds`. Using them in splits and per-task reports is later
  work.
- The commanded wrist is scored, not the robot. A course score says how well the operator drove,
  never how well a predictor did.
- **Not in this decision:**
  - pick-and-place with real objects (needs `GripperOpenPulse`/`GripperClosedPulse` calibrated
    first, docs/adr/0017);
  - loads (a payload or a hand) as course conditions;
  - wrist-roll targets.
