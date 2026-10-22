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

Amended 2026-10-21 (below, "Amendment (2026-10-21)"): the commanded path replaces the command list,
and the model may take a network forecast from a separate model, never the true network state.

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

## Amendment (2026-10-21): the commanded path, and a network forecast as an input

### Why

After the first simulation check (NO-GO, simulation only), the user and their colleagues asked for two
changes, and the user answered four questions:
- **The model must use the list of previous inputs.** The headset's predicted arm "goes directly to
  the target" instead of following the real arm's path.
- **A separate network-prediction model will feed this one network forecasts**, such as "there will be
  lag in 500 ms", and this model must compensate. Someone else builds that model, later. The forecast
  format must cover lag, jitter, buffering and the other impairments.
- **The predicted arm still shows where the arm is now**, not a look-ahead.
- **The hand-built predictor the headset runs today is fixed too**, as a new predictor.

What the investigation found:
- **Why the headset's arm heads for the target.** `relay-replay` uses each measurement only to place
  the move already in progress. About 50 ms later its prediction is the command schedule alone. It
  runs ahead of the real arm for four reasons:
  - it closes any gap within one move, at any speed;
  - it has no servo lag;
  - a servo's first move lasts the 50 ms minimum whatever its distance;
  - it assumes the robot acts at send time + RTT/2.
- **Layout v1 has the previous inputs, with a hole.** It keeps the 24 newest commands and 8 bins over
  [T−1.3 s, T−0.5 s). The list is counted back from the newest command, so once RTT/2 reaches a few
  hundred milliseconds, some commands executing between the last reading and the target time fall in
  neither the list nor a bin.
- **The headset's view of the network is thin.** Its only uplink estimate is `ClockSync`'s smoothed
  RTT/2, sampled from keepalives at 2 Hz. `ClockSync` rejects any RTT above 3× the minimum of its last
  32 samples, so a large jump in lag goes unmeasured for about 16 s. The joint-state frame echoes no
  command sequence number and no robot receive time, so nothing on the headset measures the uplink
  directly. A 2026-09-09 research run found nothing forecastable in the emulated profiles
  (`docs/research-log/2026-09-09-network-condition-estimation-feasibility.md`).

**The limit stated in the Context is amended.** No causal predictor can react to a push, a payload or a
slower uplink before delayed data shows it, **unless something announces it**:
- **The uplink can be announced.** A forecast can announce an uplink change before the arm's response
  shows up in the readings.
- **Nothing announces a push or a payload.**
- **Future lag does not move the arm.** Where the arm is now does not depend on future lag. A forecast
  helps through how late the commands already sent reach the robot, and by announcing changes that
  will affect commands about to be sent.

### Decision

10. **Layout v2: the commanded path replaces the command list and its bins.**
    - **The grid.** A uniform 20 ms grid over *effective* time, 80 points per joint, from T + 0.2 s
      back to T − 1.38 s. Each point holds the target in force (that of the newest command whose
      effective time is not later) and a mask, scaled as v1 scales targets. Effective time, not send
      time, because it is the only time the predictor's calls carry; with a forecast it is the send
      plus the forecast's uplink delay, so the path is laid out as the robot would receive it. It
      covers every command since the last reading at any delay up to the 1 s staleness fallback.
    - **Unchanged from v1:** the readings, the innovations, the prior's features, the rule against
      being told the speed, the output (prior + bounded correction + Laplace scale) and the fallbacks.
    - **The layout id is recorded** in the prepared data and the shards, and checked, so data built for
      one layout is never trained as another.

11. **The model may take a network forecast** (amends decision 7):
    - **What it is never given stays the same:** the network profile, the true delays and the payload.
    - **What it may be given:** an *estimate* from a separate model, in the format
      `trainer/contract/FORECAST_FORMAT.md` (network forecast v1).
    - **When a forecast is issued:** at time t, at least 10 Hz, separately for each direction (uplink =
      commands, downlink = joint frames).
    - **The horizon grid:** on send time relative to t, at h ∈ {−1000, −500, −250, −100, 0, +100, +250,
      +500, +1000} ms. Negative h describes packets already sent; positive h is the look-ahead.
    - **At each h:** one-way delay quantiles p10, p50, p90 and p99, and P(loss).
    - **Per direction:** the base delay (the floor), and the probability of an outage of at least 250 ms
      within the next second.
    - **Optional fields**, defined but not read by layout v2: reorder probability, loss-burst length,
      available bandwidth, the receiver's jitter-buffer target.
    - **How each impairment appears:** lag is the p50 level, jitter is p90 − p10, and buffering is p50
      above the floor and its rise across h.
    - **Effective times.** When a forecast is present, the host sets each command's effective time to
      send time + the forecast's p50 uplink delay at h = 0, kept non-decreasing, instead of RTT/2. The
      prior benefits too, without changing.
    - **The network block.** Layout v2 carries the newest forecast as an arm-level block of 101 floats:
      the quantiles and loss per direction and horizon, the floors, the outage probabilities, six
      validity flags and the forecast's age.

12. **Training and evaluation use forecasts of known quality, built from each view's true network
    schedule** (`trainer/armpred/net/forecast.py`). The levels are fixed now, before any v2 training.

    | Level | Learns of a change after | Changes announced in advance | Lead time | Error on delay above the floor (lognormal σ) | False alarms | Quantile spread × |
    |---|---|---|---|---|---|---|
    | `oracle` | 0 | all | all of the look-ahead | 0 | none | 1 |
    | `good` | 100 ms | 70 % | U(0.1, 1.0) s | 0.15 | 1 per 300 s | U(0.8, 1.25) |
    | `fair` | 300 ms | 40 % | U(0.1, 1.0) s | 0.30 | 1 per 60 s | U(0.6, 1.6) |
    | `poor` | 1000 ms | 10 % | U(0.1, 1.0) s | 0.50 | 1 per 30 s | U(0.5, 2.0) |

    How the four levels above behave:
    - **Before a change is known:** a forecast holds the last state it knows, except for changes it
      announced.
    - **After the learning latency:** it shows a change at every horizon at or after the change. That
      includes negative horizons: commands already sent.
    - **The magnitude error** is drawn once per change.
    - **A false alarm** is a spurious change of a typical size, lasting U(0.5, 3) s, announced like a
      real one.
    - **Loss probability** gets the same lognormal error.

    Three more levels use only what the headset can observe today:
    - **`persistence`:** downlink delays from frame stamps, uplink delays from keepalive round trips
      minus the downlink estimate, loss from frame sequence gaps, the floor as the 10 s minimum, and no
      look-ahead. It is a stand-in for testing, not the network model.
    - **`rtt-half`:** today's estimate, `ClockSync`'s RTT/2, in both directions, with no spread and no
      loss.
    - **`absent`:** the block is masked.

    **Training mixture.** Each training view draws a level from this mixture, and the model is never told
    which:

    | Level | `oracle` | `good` | `fair` | `poor` | `persistence` | `rtt-half` | `absent` |
    |---|---|---|---|---|---|---|---|
    | Share | 10 % | 20 % | 20 % | 15 % | 15 % | 10 % | 10 % |

    The block is also dropped for 10 % of the remaining samples. Forecast noise comes from its own seeded
    stream, so the schedule and the queries do not change with the level. **Evaluation runs every level
    separately**, and the benefit-versus-quality curve is the deliverable for whoever builds the network
    model.

13. **Training data gets fast uplink changes and two new impairments** (adds to decision 4, which
    still holds for offline synthesis):
    - **In the simulator,** the uplink follows mid-session schedules from the downlink's generator,
      so the simulated arm physically executes late commands. Keepalives share that uplink, as they do
      on the robot.
    - **Two new segment types,** for both directions:
      - `queue`: a sawtooth that fills and drains, with tail-drop loss at the peak (buffering);
      - `outage`: nothing is delivered for 0.2–2 s, and what was sent arrives in a burst at the end.
    - **On the robot,** `UplinkImpairedReceiver` gains a schedule mode, a time-varying profile read
      from a file with every change recorded, so live sessions contain mid-session uplink changes.
      Recorded `UP` changes become uplink events.
    - **Views v2** store each command's true uplink delay and the forecast stream (or its level and
      seed). v1 views stay readable.
    - **Every command the headset sent reaches a view's predictor**, including commands lost on the
      uplink. This is exact in simulation; in recorded sessions, gaps in the command sequence numbers
      are marked.

14. **Gate additions, fixed now** (adds to decision 9; same method). The best non-learned predictor now
    also includes `relay-replay-servo` (decision 15). An "uplink change" is a change of at least 50 ms
    in the uplink's one-way delay, as a step or within one second, at time t_e.

    | Gate | Pass condition |
    |---|---|
    | G0 | With the network block absent, v2 passes G1–G6 as already fixed |
    | F1 | With `oracle`, p95 error over [t_e, t_info + 1 s) after uplink changes is at least 25 % lower than v2 with the block absent |
    | F2 | At `poor`, every condition within 0.1° of `absent` at p95; correction and jitter within G3's limits |
    | F3 | At `good`, v2 beats the best non-learned predictor fed the same forecast's effective times, by G1's margin, on uplink-change conditions |

    - **GO for v2 with forecasts** means G0, F2, F3 and the decision 9 GO all pass. G0 and the decision
      9 GO are one and the same check, because v2 with the block absent is the model scored there.
    - **If F1 fails,** the forecast input is a NO-GO for this model and the report says so. A perfect
      forecast that does not help means nobody should build a network model for this purpose.
    - **Reported regardless:** the full quality curve, and the gap between `persistence` and `rtt-half`.
    - **New metric `joint_prediction_lead_ms`,** defined in `docs/metrics.md` §4 by the change that
      first reports it. It is the time shift that best aligns a prediction with the truth over a motion
      segment; positive means the prediction runs ahead. It measures the symptom that prompted this
      amendment.

15. **`relay-replay-servo`, a new analytic predictor in Core.** It keeps `relay-replay`'s gate and adds:
    - catch-up capped at the servo's speed;
    - a first-order servo lag;
    - each servo's first move sized from the measured position;
    - a fixed pipeline delay.

    Being a new approach to an existing question, it is built under `Prediction/` with `/new-impl` and
    needs no ADR of its own. It is recorded here because the trainer gets a pinned Python copy of it, so
    decision 5's list grows by one, and because it joins the baselines of decisions 9 and 14. Its
    parameters are guesses until `armpred sysid` fits them on the pilot recordings. It goes on the
    headset only after it beats `relay-replay` on those recordings, scored headlessly.

### Consequences

- **A forecast contract in Core, later.** After a trainer-side GO, the C# consumer of decision 8 also
  needs a Core contract for the forecast, with its own amendment: a `NetworkForecast` value type and an
  optional interface the joint predictor implements. The host would then build effective times from
  the forecast.
- **A prerequisite for a real network model.** No model on the headset can measure the uplink until
  the joint-state frame echoes the last command's sequence number and the robot's receive time. That
  is an ADR 0018 wire amendment, not decided here.
- **The input is wider:** about 350 floats per joint and roughly 70k parameters. G6's inference budget
  is unchanged.
- **Synthetic forecast errors are not a real network model's.** That is why the deliverable is a curve
  over quality levels, with `persistence` as a causal floor. A real network model's outputs go through
  the same evaluation once it exists.
