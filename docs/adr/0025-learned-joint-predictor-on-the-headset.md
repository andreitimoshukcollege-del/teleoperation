# 25. The learned joint predictor on the headset, in plain C#

## Status

Accepted (2026-10-27). This is ADR 0021's Phase 7 backend decision, and it amends ADR 0021 decision 8.

The user asked for the headset's twin to default to the learned joint predictor, because that model is
what this work is building. They chose:
- **the runtime:** a plain C# forward pass in Core;
- **the scope:** the learned model predicts **all six joints, gripper included**;
- **the default:** the fine-tuned model `finetune-pilot2-early7`.

## Context

- **ADR 0021 decision 8** said the repo's C# consumer would be built only after a trainer-side GO. No
  model has passed the trainer gate.
- **The best model so far** is `finetune-pilot2-early7`: `sim-pretrain-pilot2` fine-tuned on four
  real joint-drive sessions. On seven held-out sessions:
  - arm error p95 is **4.26°**, against **4.98°** for what the headset runs today (`servo 10-25`) and
    3.98° for the 2026-10-26 fit (`servo 10-26`);
  - its **gripper is worse**: 27.2 servo degrees, against 9.4, because the drive sessions never move
    the gripper.

  These are trainer evaluations, not results.
- **`IInferenceBackend` exists in Core**, with no implementations and no consumers. Sentis is banned
  until this decision.
- **The exported model is a small MLP.** `[joint row | arm row]` (251 + 114) goes through
  128 → 128 → 64 → 2 with ReLU, then a fixed head:
  - a correction, `cap · tanh`, with the cap at 15° or 90 servo degrees for the gripper;
  - a clamped log-scale.

  It runs once per joint, and the graph uses only Gemm, Relu, Tanh, Clip and reshaping ops.
- **The model's input is layout v3** (`trainer/contract/MODEL_FORMAT.md`), built call by call by
  `trainer/armpred/features/layout.py` from exactly what an `ICommandAwarePredictor` is given. The
  baseline it corrects (its "prior") is Core's `relay-replay`, run speed-blind with the headset's
  config and the fitted reading ages.

## Decision

1. **The forward pass is plain C# in Core** (`Inference/MlpBackend.cs`, an `IInferenceBackend`). It is
   the exported graph's arithmetic in single precision: the per-joint Gemm chain and the head, exactly
   as `FlatExport` defines them, with input `[1, 1620]` and output `[1, 12]`. No package, no native
   plugin, and no Sentis:
   - it compiles under dotnet and IL2CPP alike;
   - it is deterministic;
   - `Teleop.Eval` can run the same code headless, which is the path to citable numbers in `results/`.

   Only architectures written out here can run. Today that is A (`mlp`). B (`conv`) would be a new
   backend.
2. **The weights come from the scored `.onnx` itself.** `armpred export-weights` reads the graph's
   initializers and checks a NumPy forward pass against onnxruntime. It writes `model.weights`
   (`trainer/contract/MODEL_FORMAT.md`, *Weights*), a little-endian file that also carries the `.onnx`'s
   sha256. Core parses the bytes (`Inference/MlpWeights.cs`); the host reads the file, since Core does
   no I/O.
3. **The window is a C# port of layout v3** (`Prediction/LearnedJointWindow.cs`). It is computed in
   double precision and stored in float, as the Python does. It is pinned by a golden fixture that the
   trainer writes (`armpred golden-learned` → `core/testdata/learned/`):
   - a real view's predictor calls;
   - the windows, the model's outputs and the predictions at a series of display ticks.

   This is the reverse of `trainer/contract/golden/`: there the repo is the source and the trainer
   copies it; here the trainer is the source and the C# copy must match it.
4. **`learned` is a registered joint predictor** (`Prediction/LearnedJointPredictor.cs`). Its config
   carries the model (`JointPredictorConfig.Model`). It falls back to its baseline, as
   `MODEL_FORMAT.md` requires, when:
   - there is no fresh reading yet;
   - the newest reading is more than 1 s old;
   - the backend fails;
   - an output is not finite.

   It counts each fallback. It predicts the five arm joints and the gripper; any other joint takes the
   baseline.
5. **The headset defaults to it.** The Bridge config gains preset fields `Predictor: "learned"` and
   `Model` (a file name). The default model ships in Bridge's `Resources`. A file of the same name in
   `Application.persistentDataPath` overrides it (`just push-galaxy-model`), so a new model needs no
   rebuild. `JointPredictorActive` names the learned preset. The conventional presets stay on the
   console beside it, scored live, so every session compares them.
6. **ADR 0021 decision 8 is amended:** the C# consumer exists before a GO. The gate still decides what
   may be *claimed*: a learned model's number is a result only when `Teleop.Eval` has scored it into
   `results/`. Defaulting to it on the headset is the user's call about what to look at, not a claim.

## Consequences

- **The twin's gripper will be worse** than with `servo 10-25` until a model learns it. Course
  sessions with gripper targets (ADR 0024) are what fix it.
- **Parity is a test, not a hope.** The window and the outputs must match the trainer's to float
  tolerance, or the model sees inputs it never trained on.
- **A trainer change to layout v3, the prior, or the head** needs the golden regenerated, and the C#
  changed in the same commit.
- **Cost on the headset:** six 365-input MLPs per prediction, about 430k multiply-adds, at 90 Hz and
  at every fresh reading. It is allocation-free, and the benchmark row measures it.
- **The network block is zero** (no forecast) until a network model exists, which is the `absent`
  level the gates are measured at.
