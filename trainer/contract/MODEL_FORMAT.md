# Model format, version 1

What the trainer hands back to the repo (docs/adr/0021 decision 2): one `model.onnx` and its
`model-card.json`. The C# predictor that runs it (`SequenceModelJointPredictor`, built only after the
trainer gate) rebuilds the input window from its own calls, exactly as `armpred/features/layout.py`
does, and refuses a model whose layout id or lengths it does not know.

## The graph

| | Name | Shape | |
|---|---|---|---|
| input | `x` | float32 [1, 1359] (`v2`), [1, 1360] (`v2-aware`); [1, 898] / [1, 899] for v1 | the five joint rows, motors 1-5 in order, then the arm row |
| output | `y` | float32 [1, 10] | per joint, motors 1-5: the correction in rad (already bounded to +-15 degrees), then the Laplace log-scale (log rad) |

- **Static shapes, opset 17,** and only the ops in `armpred/export.py`'s `ALLOWED_OPS`, so whichever
  on-device runtime the Phase 7 ADR picks is not blocked by an exotic op.
- **The prediction is `prior + y[2j]`** for joint j, where the prior is `relay-replay` run speed-blind
  (the host passes 0 as the requested speed) with the config the model card names.
- **Fallback to the prior** (and count it, by cause): no fresh reading yet; the newest reading more
  than 1 s old; any output not finite; the backend failing.
- **Parity:** the exported graph matches the torch model to 1e-5 rad on real windows (the card says
  how close).

## Metadata (ONNX `metadata_props`)

| Key | Value |
|---|---|
| `teleop.layout_id` | `v2` or `v2-aware` (`v1`, `v1-aware` before 2026-10-21) |
| `teleop.model_format` | `1` |
| `teleop.input_length`, `teleop.output_length` | `1359` (or `1360`), `10` |
| `teleop.arch` | `mlp` or `conv` |
| `teleop.trainer_version`, `teleop.trainer_sha`, `teleop.trainer_dirty` | the trainer that made it |
| `teleop.data_sha256` | JSON: the sha256 of every shard it was trained and selected on |
| `teleop.source` | `sim`, `real` or `sim+real` |

## Layout v2 (the input window, docs/adr/0021 decisions 10 and 11)

Per arm joint, 249 floats; then 114 arm-wide floats (115 for `v2-aware`). Conventions as v1 below.

| Part | Floats | Content |
|---|---|---|
| prior | 2 | as v1 |
| commanded path | 80 x 2 | newest first, at effective times `T + 0.2 s - 0.02 s * i` (i = 0..79): `asinh((target in force - v) / 0.2)`, 1 if any command had taken effect by then. The target in force is the newest command (with this joint) whose effective time is not later |
| readings | 16 x 3 | as v1 |
| innovations | 8 x 4 | as v1 |
| identity | 5 | as v1 |
| angle | 2 | as v1 |

Arm-wide: v1's 13 (the last, "how full", is now `min(commands, 64) / 64`), then the **network block**,
101 floats, from the newest forecast the host passed (`FORECAST_FORMAT.md`):
- for the uplink then the downlink, for each of the 9 horizons: `asinh(q / 100 ms)` for p10, p50, p90,
  p99, then P(loss) (90);
- `asinh(floor / 100 ms)` uplink, downlink (2); P(outage) uplink, downlink (2);
- the validity flags, uplink then downlink, each quantiles, loss, outage (6);
- the forecast's age, `(T - issued) / 0.5` (1).

An invalid field is 0 and its flag 0; with no forecast the whole block is 0. For `v2-aware` the newest
command's speed / 6 rad/s follows the block. The host computes every command's effective time from the
forecast when it has one (`FORECAST_FORMAT.md`, "What the host does with it"), which also moves the
commanded path.

## Layout v1 (the input window, superseded)

Per arm joint, 177 floats; then 13 arm-wide floats (14 for `v1-aware`). Times are relative to the
target time T and divided by 0.5 s. Angles are relative to the joint's newest fresh reading `v` and
squashed: `asinh(x / 0.2 rad)` (innovations: `asinh(x / 0.05 rad)`). Missing entries are zero with a
zero mask.

| Part | Floats | Content |
|---|---|---|
| prior | 2 | `asinh((prior(T) - v) / 0.2)`; `(prior(T) - prior(T - 40 ms)) / 0.04 / 2.0` |
| commands | 24 x 3 | the 24 newest commands, newest first: `(t_eff - T) / 0.5`, `asinh((target - v) / 0.2)`, 1 |
| command bins | 8 x 2 | bin b covers [T - 0.5 - 0.1 (b + 1), T - 0.5 - 0.1 b) s: `asinh((mean target - v) / 0.2)`, 1 if any command |
| readings | 16 x 3 | the 16 newest fresh readings, newest first: `(t_sensed - T) / 0.5`, `asinh((value - v) / 0.2)`, 1 |
| innovations | 8 x 4 | the prior's error on each of the 8 newest fresh readings, taken just before it was observed: `(t_sensed - T) / 0.5`, `asinh(error / 0.05)`, `horizon / 0.5`, 1 |
| identity | 5 | one-hot of the joint (motors 1, 2, 3, 4, 5) |
| angle | 2 | `sin v`, `cos v` |

Arm-wide: `sin`/`cos` of the newest readings of motors 2, 3 and 4; `(T - newest fresh sensed) / 0.5`;
fresh readings in the last second / 20; the longest gap between fresh readings in the last 2 s / 0.5;
`(t_eff of the newest command - T) / 0.5`; the newest gripper command (0..1) and its change over the
last 0.5 s; `min(commands, 24) / 24`; and for `v1-aware`, the newest command's speed / 6 rad/s.

A *fresh* reading is a state whose sensed stamp is later than every earlier one. A command older
than the newest is dropped, as `relay-replay` drops it.

## The card (`model-card.json`)

```json
{
  "format": "jetrover-model-card", "version": 1,
  "model": { "path": "model.onnx", "sha256": "...", "opset": 17, "ops": ["Gemm", "..."],
             "inputLength": 898, "outputLength": 10, "layout": "v1" },
  "prior": { "predictor": "relay-replay", "speed": "blind", "config": "headset" },
  "parity": { "maxAbsRad": 9.3e-10, "rows": 64, "toleranceRad": 1e-5 },
  "trainer": { "version": "0.1.0", "sha": "...", "dirty": false },
  "formats": { "session": 1, "views": 1, "model": 1 },
  "data": { "train": "<sha256>", "val": "<sha256>" },
  "source": "sim",
  "training": { "config": {}, "best": {}, "val_prior": {}, "val_hold_last": {} },
  "notResult": "A model is not a result until Teleop.Eval has scored it (docs/adr/0021 decision 8)."
}
```

`prior.config` is `headset` (the `JetRoverArmConfig` defaults: reading ages 45/35/25/15/5 ms, the
relay's gate) until `sysid` fits one, when it becomes the fitted values themselves.
