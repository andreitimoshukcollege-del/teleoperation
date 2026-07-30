# analysis/

Python. Reads `results/`, produces figures and statistics, trains models, exports ONNX.

**This side never influences the runtime.** It consumes emitted data and produces artifacts.
It does not reimplement algorithms — if a plot needs a metric that Core doesn't emit, add the
metric to Core rather than recomputing it here. Two implementations of the same metric will
disagree, and you will trust the wrong one.

## Rules

- Read `results/<exp>/<run>/metrics.csv` and `manifest.json`. Never read from `core/` source
  to infer behavior, and never mutate anything under `results/`.
- Every figure-producing script takes a run directory as an argument and writes to
  `results/<exp>/<run>/figures/`. No hardcoded paths.
- Every figure caption states the network profile, the seed, and the git SHA from the
  manifest. An uncaptioned figure is not reusable at writeup time.
- Report **percentiles** (p50/p95/p99) for latency and error, not means. Distributions here
  are heavy-tailed and the tail is what the operator perceives.
- Pin dependencies in `requirements.txt`. State the statistical test being used and check its
  assumptions before applying it.

## Model training

- Training data comes from committed or archived `.tlog` recordings, loaded via
  `teleop_analysis/tlog.py` — which must stay compatible with `Recording/RecordFormat.cs`.
  If the format version bumps, update the loader in the same PR.
- Export to ONNX with a fixed opset and static shapes. Dynamic shapes are a problem for
  on-device inference.
- Verify parity after export: the ONNX model and the training-framework model must agree to
  tolerance on a held-out batch. Log the tolerance actually achieved.
- Only commit a model when a result depends on it. Checkpoints stay out of git — binary
  weights don't delta-compress and every retrain adds a permanent full copy.
- A model is not a result until `Teleop.Eval` has scored it through `SequenceModelPredictor`
  against the same traces as the analytic baselines. Held-out loss is not a result.
