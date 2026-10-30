# 28. The learned joint predictor for the FR3: layout v4, weights v2 and an FR3 gate

## Status

Proposed (2026-10-28). Part of the FR3 Duo work (ADRs 0026, 0027). It amends ADR 0021: decision 5
(what the trainer copies), decision 6 (the simulator), decisions 7 and 9 (the model and the gate,
for the FR3). It also amends ADR 0025: decisions 1, 3 and 4 (the C# window and backend). The user
asked for the model to be retrained for the FR3 Duo, with the trainer's simulator an analytic
controller model rather than a physics engine.

## Context

**Layout v3 is the JetRover's in every dimension** (`trainer/armpred/features/layout.py`,
`core/Teleop.Core/Prediction/LearnedJointWindow.cs`):
- **the joints:** six fixed motor ids (`{1,2,3,4,5,10}`) and a six-wide one-hot identity;
- **gravity:** sin/cos of the three planar pitch joints (2, 3 and 4);
- **the gripper:** a fraction converted at 0.4513 per servo radian;
- **the head:** correction caps of 15° and 90° (gripper), the gripper flag at input index 247, and a
  readings-per-second feature scaled to a 20 Hz relay;
- **the prior:** hard-typed as `relay-replay`.

`LearnedJointPredictor` refuses any model that is not v3's 1620 → 12 (`:34`). The `TLMW` weights
header has no layout id.

**There are no FR3 recordings, and there will not be any until the robot arrives.** The trainer's
simulator models hobby servos, the relay gate, sequential bus reads and 0.24° quantisation. None of
that exists on an FR3, where:
- the controller reports every joint at 1 kHz with encoder noise far below a degree;
- the arm is driven by ADR 0027's shaper, which this repo owns.

**ADR 0021's gate does not fit.** Its absolute thresholds (0.25° at p95, 0.1° worse at most) were
sized for the JetRover's 0.24° pulse floor. Its baselines are relay predictors, and the FR3 has no
relay.

## Decision

1. **Layout v4 is new, and v3 does not change.** v3's files, golden (`golden-v3.json`), C# window and
   default JetRover model stay byte-identical, and every v3 test passes unmodified. v4 runs **per
   arm, with weights shared across arms and joints**:
   - **rows:** one per arm joint (7) plus one for the finger. The finger is in model units: its 0..1
     fraction times a fixed scale, recorded in the card;
   - **per row:** v3's prior, commanded-path, reading and innovation blocks. The window spans are
     re-derived for a 1 kHz controller reported at the joint stream's rate. The prior is
     `franka-replay` (ADR 0027 decision 4) instead of `relay-replay`;
   - **the identity:** an 8-wide one-hot (joint index within the arm);
   - **the arm block:**
     - sin/cos of all seven joints;
     - the gravity direction in that arm's base frame (fixed by the mount, so the two arms differ);
     - v3's timing features, rescaled to the joint stream's rate;
     - v3's network block, unchanged;
   - **the caps:** correction caps sized to the FR3 (degrees, not tens of degrees), pre-registered
     in `MODEL_FORMAT.md` before training;
   - **the exact sizes:** fixed in `MODEL_FORMAT.md` v4 by the trainer change that builds it, and
     derived there from the profile's joint count, not written as constants.
2. **Weights format v2** (`TLMW` version 2) adds a layout id and the per-arm row count to the
   header.
   - Core's `MlpWeights` reads both versions.
   - `learned` dispatches on the layout id: `v3` goes to today's window, `v4` to a new
     `LearnedJointWindowV4.cs`.
   - `MlpBackend` takes its caps and the finger flag's index from the layout instead of constants.
   - The model card is `teleop-model-card` v3. It lists the profile's name and sha256 and the motor
     ids per arm.
3. **The trainer gets a second simulator, `sim/fr3.py`, beside the JetRover's.** It generates
   session v2 (ADR 0027 decision 6). It contains:
   - **the shaper**, an exact copy of Core's, pinned by a golden file;
   - **a tracking model** standing in for Franka's internal controller: a per-joint second-order lag
     and a gravity/payload bias, with parameters drawn wide per session;
   - **encoder noise;**
   - **reflex aborts** when a command would violate a limit;
   - **the finger gate;**
   - **the network synthesis,** reused unchanged (`armpred/net/`);
   - **a joint-space operator** for two arms: smooth targets within the soft limits.

   It is analytic and seeded-deterministic, like the JetRover simulator. MuJoCo would not model
   Franka's proprietary controller either, and the predictor sees no contact. When the robot arrives,
   `sysid` refits the simulator's ranges from real sessions, as it does for the JetRover today.
4. **The trainer's code takes its joint lists from the layout and the profile,** not from module
   constants. This covers `ARM_MOTORS`, `PREDICTED_MOTORS`, `GRAVITY_MOTORS` and `GRIPPER_MOTOR` in
   `features/`, `data/`, `evaluate.py`, `train.py`, `sysid.py`, `baselines/` and the card writer in
   `cli.py`. The JetRover's lists stay what they are, as its profile's.
5. **The FR3 gate, fixed now, before any FR3 training.** The method is ADR 0021 decision 9's: the
   median of 5 seeds, a 95% session-bootstrap interval on the paired difference, and a test set
   frozen before the model. The baseline is the better of `franka-replay` and `hold-last`. The
   thresholds are scaled to the FR3:

   | Gate | Pass condition |
   |---|---|
   | F1 | p95 error on `150ms-20j-0.5loss` lower by at least 15% **and** at least 0.05° |
   | F2 | On `lan` and `50ms-5j`: p95 no more than 0.02° worse, p99 no more than 0.05° worse |
   | F3 | Correction p95 no more than 0.02° worse, maximum at most 1.5×, display jitter not worse |
   | F4 | p95 error in the first second after a payload change, lower by at least 25% |
   | F5 | Every held-out condition within 0.05° at p95; fallback below 1% outside loss bursts |
   | F6 | Inference p99 at most 100 µs on one desktop thread (two arms); ONNX parity within 1e-5 rad; two replays bit-identical |

   **GO** means F2, F3, F5 and F6 all pass, and F1 or F4 passes.

   **The gate cannot be passed on simulated data.** In the simulator, `franka-replay` is exact up to
   tracking dynamics we invented, so a simulated win only shows the model learned what we wrote. It
   is reported as a simulated result (`trainer/CLAUDE.md`), and it is a reason to fine-tune on real
   sessions, not a claim. The gate is scored on real FR3 sessions only, when they exist. Until then,
   `sim-host` sessions (ADR 0027) give real human command streams over a simulated arm, and their
   numbers are labelled that way too.
6. **The headset's FR3 default is `franka-replay`,** not `learned`. A learned FR3 model becomes the
   default only by the user's call, as ADR 0025 decision 5 was, and after it runs on real or
   `sim-host` sessions. The JetRover's default stays `finetune-pilot2-early7`.

## Consequences

- **Two layouts, two windows and two goldens live side by side,** each pinned the same way. A change
  to one must leave the other's golden untouched.
- **The FR3's first learned numbers are simulated by construction.** Every report says so in its
  title, and no gate is claimed until real sessions are scored by `Teleop.Eval`.
- **The trainer gets bigger:** a second simulator, a second layout and a generic joint list. The
  JetRover pipeline's tests must still pass unchanged, and they are the check that the refactor in
  decision 4 changed nothing.
- **Cost on the headset:** 16 rows per prediction instead of 6. That is about 2.7 times today's
  multiply-adds, still allocation-free. The benchmark row measures it, and F6 bounds it.

## Alternatives considered

- **One model with all 16 joints and one arm block.** Rejected. Sharing across arms halves the
  parameters the data must fit, and a mirrored mount is exactly what the gravity feature encodes.
- **Generalizing layout v3 in place.** Rejected. It would change the bytes the JetRover model and
  its golden were built on, for no JetRover benefit.
- **Keeping ADR 0021's absolute thresholds.** Rejected. At FR3 noise levels, 0.25° is larger than the
  error that matters, so the gate would pass anything.
