# 21. A learned joint predictor, trained outside the repo's build, on robot recordings and a simulator

## Status

Accepted (2026-10-20). The user's choices:
- **headless first:** train, then score; inference on the headset waits for a gate and for its own
  Phase 7 ADR;
- the model is **blind to the speed the headset requests**; a speed-aware model is trained only to
  measure what knowing it is worth;
- resistance comes from **a payload in the gripper and from hand resistance**;
- **simulation first**, then real-robot data, of which there can be a lot;
- the simulation and the training live **apart from the repo's build, as one file sent to the GPU
  machine** (an RTX 5090 box, and an H200 Slurm cluster);
- the C# side that runs a trained model and scores it officially is built **only after the model
  has won inside the trainer**.

Amends ADR 0018 §5 (recording) and `analysis/CLAUDE.md` ("Model training").

## Context

The operator should see where the arm really is, despite network delay, from the joint targets the
headset sent and the joint angles it has measured so far. It should adapt, without being told, to the
arm moving slower or faster, to the network getting worse, and to something resisting the arm.

What exists: the contract such a predictor plugs into (`Contracts/ICommandAwarePredictor.cs`, ADR
0018), three hand-built implementations of it (`hold-last`, `servo-replay`, `relay-replay`, the
headset default), Core's network impairments (ADR 0013), `ClockSync` (ADR 0008), and an unused
`IInferenceBackend`.

What does not:
- **Not one recording of joint commands or joint states.** ADR 0018 §5 decided on one; it was never
  built, so no predictor has ever been scored on the real robot. `relay-replay`'s parameters are
  guesses, and the only numbers are `joint-predict-bench`'s simulation.
- **Ground truth in the wire stream.** A `JointStateFrame` carries one stamp, taken when RobotHost
  read the relay's feedback datagram (`GenericArmPlant.LastFeedbackTicks`), not when any servo was
  read. The relay forwards readings at 20 Hz and hides up to 200 ms of staleness, and the sensed flag
  never clears. Only the driver knows when each servo was read.
- **A simulator with load or resistance.** `Plant/` is kinematic by deliberate choice, and the bench's
  relay simulation has no dynamics.

Two limits shape what can be claimed, and are stated here so no result implies otherwise:
- **No causal predictor can react to a push, a payload or a slower uplink before delayed data shows
  it.** That takes at least one downlink delay. The metrics report this window separately.
- **Most adapting to the network comes free.** A slower downlink only makes readings older, and
  replaying the commands sent since then already covers it. What learning can add is servo and load
  behaviour over the stale stretch, uplink lag, and speed it is not told.

## Decision

1. **The trainer is `trainer/`, a self-contained folder** (`trainer/CLAUDE.md` has its rules):
   - it imports nothing from the rest of the repo, and nothing in the repo imports it;
   - `just pack-trainer` packs it into one `dist/armpred-<sha>.tar.gz` with a `VERSION` file (git SHA
     and dirty flag); that file is what goes to the GPU machine;
   - it runs on the 5090 box from `setup.sh` (a venv from a pinned lock file; the 5090 is Blackwell,
     sm_120, so PyTorch must be built for CUDA 12.8 or later), and on the H200 cluster from an
     Apptainer image (`trainer/slurm/apptainer.def`) with one run per GPU as a Slurm job array;
   - it holds the simulator, the network synthesis, the features, the models, training, export and
     its own evaluation. Root `CLAUDE.md`'s Core invariants do not bind it; the rules in its own
     `CLAUDE.md` do.

2. **Three versioned file formats are the only things the repo and the trainer share.** Their specs
   and golden files live in `trainer/contract/`:
   - **session v1** (robot → trainer): what the robot recorded, packed by `just pack-dataset` into
     one `dist/jetrover-data-<UTC>.tar` with an index and a copy of the robot profile;
   - **model v1** (trainer → repo): an `.onnx` with a fixed opset, static shapes and metadata
     (`teleop.layout_id`, the trainer SHA, the data hashes), plus `model-card.json`;
   - **views v1** (trainer → repo): sessions as the headset would have seen them under a named network
     schedule and seed, with the truth attached. Official scoring replays these.

3. **Recording** (amends ADR 0018 §5). The truth comes from the driver's own reads, not from frames:
   - the ROS driver logs every servo read (request and response time on `CLOCK_MONOTONIC`, motor,
     pulse or a miss), every board write packet, every gate decision and every battery report;
   - RobotHost logs every keepalive, every joint command (arrival, sequence, capture stamp, targets,
     speed, and whether it was accepted, stale or refused), every relay target sent, every relay
     feedback datagram and every joint-state frame sent;
   - both share the Jetson's `CLOCK_MONOTONIC` (.NET's `Stopwatch` and Python's `time.monotonic_ns`),
     checked by `just jetson-clock-check` (RobotHost's `--clock-echo` answers its clock between two
     Python reads, so the check resolves a fraction of a millisecond);
   - both write from a background thread through a bounded ring, count every record they drop, and
     are switched on by polling `~/recordings/.active`, so recording restarts nothing;
   - sessions go to `recordings/jetrover/<UTC>-<label>/` (gitignored; archived to a GitHub Release,
     as `.gitignore` already says), and `just recording-check` fails a session with any drop, a
     clock-causality failure or a read gap over 500 ms;
   - ADR 0018 §5's operator-side CSV is not needed for truth. An operator-side log (`operator.jlog`)
     is written by `joint-drive` and can later be written by the Bridge.

4. **How a recorded session becomes training and test data.** The downlink never affects the arm, so
   it is synthesised in full: delay, jitter, Gilbert-Elliott loss, reordering and recorded traces, with
   schedules that switch mid-session. The uplink does affect the arm, so it is only ever a slow time
   shift of the command stream (|slope| ≤ 0.05), which leaves the recorded physics exact. Uplink jitter
   and loss come only from sessions recorded with the uplink physically impaired on the robot
   (`UplinkImpairedReceiver`, a receive-only wrapper, because wrapping all of RobotHost's
   `UdpTransport` would also drop the frames it sends). It reads a Core profile name from a second
   control file (`~/recordings/.uplink`, written by `just uplink-impair` and by `record-start
   network=...`), so it too needs no restart, and every change is recorded (`UP`).

5. **The trainer's copies of Core are pinned to Core.** It needs Python copies of `SeededRng`
   (xorshift128+ seeded by splitmix64, pure 64-bit integer arithmetic), the impairments and
   `EmulatedTransport`'s ordering, the profile catalog, `ClockSync`, the headset's feed logic
   (`JetRoverJointStateLink`: sync gating, effective time = send + RTT/2) and `relay-replay`.
   `Teleop.Eval gen-trainer-golden` writes golden files from the C# originals into
   `trainer/contract/golden/`, an Eval test fails if Core no longer reproduces them, and the trainer's
   tests fail if its copies do not.

6. **The simulator lives in the trainer, so `Core/Plant` stays kinematic.** It writes session v1, so a
   simulated session travels the same path as a recorded one. It models the relay gate, per-joint
   servo speed limits scaled by battery and load, lag and droop, payload gravity, hand-resistance
   events, sequential reads with misses and board silences, 0.24° quantisation, an impaired uplink and
   operator motion. Its parameters are drawn wide and then re-centred on the pilot recordings. Its
   numbers are never presented as the robot's.

7. **The model (`seq-model`, joint space).** One stateless call per prediction: the prediction of
   `relay-replay` run speed-blind (the host passes 0), plus a learned correction, plus a per-joint
   uncertainty. Its inputs are a fixed window of the commands and readings it has seen, the prior's
   prediction and recent errors, and timing. It is **never given** the requested speed, the network
   profile, true delays, `AddedDelayMs` or the payload. The prior's gate parameters are fitted on
   training data and carried in the model card. It infers load from joint data alone, so root
   `CLAUDE.md`'s Direction paragraph is unchanged: nothing here models the environment. For joint
   prediction only, it replaces the proposal's "the fast loop stays analytic"
   (`docs/proposals/2026-09-29-learned-model-mediated-teleoperation.md`).

8. **The C# consumer waits for the trainer gate.** Until the model passes decision 9 inside the
   trainer, the repo gains only the recorder, the recipes, the golden files and data-collection tools
   (`joint-drive`, `UplinkImpairedReceiver`, a stall watchdog). After it passes, an amendment to this
   ADR adds `SequenceModelJointPredictor` to Core (with `IInferenceBackend` passed through optional
   `JointPredictorConfig` parameters, so Bridge is untouched), an ONNX Runtime backend to `Teleop.Eval`
   only, `relay-replay-adaptive` to Core, and `joint-score`, which replays views through every joint
   predictor and writes `results/`. **A model is not a result until that scoring has run.**

9. **The gate, fixed now, before any training.** The comparison is against the best non-learned
   predictor (fitted `relay-replay` or `relay-replay-adaptive`) on a test set recorded after the model
   is frozen. Each threshold uses the median of 5 seeds and a 95% session-bootstrap interval on the
   paired difference.

   | Gate | Pass condition |
   |---|---|
   | G1 | p95 error on `150ms-20j-0.5loss` and on the live-impaired sessions lower by at least 15% **and** at least 0.25° |
   | G2 | On `lan` and `50ms-5j`: p95 no more than 0.1° worse, p99 no more than 0.25° worse |
   | G3 | Correction p95 no more than 0.1° worse, maximum at most 1.5×, display jitter not worse |
   | G4 | p95 error in the first second after t_info, after a resistance onset, lower by at least 25%; median recovery at most 500 ms |
   | G5 | Every held-out condition within 0.25° at p95; fallback below 1% outside loss bursts |
   | G6 | Inference p99 at most 50 µs on one desktop thread; ONNX parity within 1e-5 rad; two replays bit-identical |

   **GO** means G2, G3, G5 and G6 all pass, and G1 or G4 passes. Inside the trainer the same thresholds
   decide whether to build decision 8; in `Teleop.Eval` they decide the result. Anything else is
   recorded as a negative result. The gap between the blind and the speed-aware models is reported
   either way. t_info, recovery and the other new metrics are defined in `docs/metrics.md` §4 by the
   change that first reports them.

## Consequences

- **The trainer duplicates five pieces of Core**, deliberately, so the GPU machine needs neither .NET
  nor the repo. The golden files are what keep the copies honest; a Core change that alters any of
  them fails CI on the repo side, and its golden files must be regenerated in the same change.
- **Recording adds two writer threads on the robot** and touches the driver, so `just deploy-ros`
  (which restarts the servo driver) is needed once, with someone watching.
- **The truth has a noise floor**: about 20 readings a second per motor, in 0.24° steps, each taken at
  some instant inside a bus transaction. Errors are scored only at logged read instants, the floor is
  measured on the pilot recordings, and no difference below one pulse is claimed.
- **Test data must be recorded after the model is frozen**, so the robot is needed twice: once for
  training data and once for the test set.
- **`analysis/` stays what it was.** It never trains; `analysis/CLAUDE.md` and the analyst agent now
  point at `trainer/`.

## Alternatives considered

- **Simulate, synthesise and export in `Teleop.Eval`, and train in `analysis/`.** One copy of the
  network model, but the GPU machine would then need .NET and the repo, and every training change would
  be a repo change. The user chose separation; the golden files buy back the single source of truth.
- **A separate git repository for the trainer.** Fully independent, but the shared formats and golden
  files would be copied across at each version bump, and a model card would need both repos' commits.
- **Take ground truth from the joint-state frames.** No robot change, but the frame stamp is not a read
  time, and the error it adds (tens of milliseconds, a degree or more at speed) is about as large as the
  differences being measured.
- **Synthesise uplink jitter and loss offline.** Not exact: they change which target the relay's
  newest-wins gate writes, so the recorded arm would no longer be the arm that would have moved.
