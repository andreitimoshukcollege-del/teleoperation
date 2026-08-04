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

## Testing

- One-time setup, from `analysis/`:
  ```bash
  # No Linux Python/pip/venv under WSL here -- python.exe below is the real Windows
  # interpreter, reached the same way root CLAUDE.md's Environment section documents for
  # dotnet. Use whatever `python3`/`python` already works if this machine has one on the
  # WSL side; the point is a normal venv, not this specific path.
  /mnt/c/Users/andre/AppData/Local/Microsoft/WindowsApps/python.exe -m venv .venv
  ./.venv/Scripts/python.exe -m pip install -r requirements-dev.txt
  ./.venv/Scripts/python.exe -m pip install -e . --no-build-isolation
  ```
- Run the suite: `./.venv/Scripts/python.exe -m pytest -v`, from `analysis/`. This is the
  scriptable path — use it for CI or when an agent needs to verify pass/fail non-interactively;
  it doesn't change with the picker below.
- For a human at a terminal who wants to pick a subset instead of memorizing pytest node-id
  syntax: `./.venv/Scripts/python.exe run_tests.py` opens an interactive checklist (arrow keys
  + space to toggle, one entry per test grouped by file, all checked by default so Enter alone
  still runs everything). Requires a real console — it will not run through a piped/non-TTY
  shell. `run_tests.py --all` skips the prompt and runs everything, equivalent to `pytest -v`
  but useful if you want the same script for both cases.
- `tests/test_cli_against_committed_run.py` runs the real CLI end-to-end against the committed
  `results/exp-001-predictor-baseline/` run and checks a computed p50 against a hand
  computation from the raw CSV — it skips itself if that result directory isn't present, it
  never fabricates a pass. This is the test that actually proves the pipeline works, not just
  that its pieces don't crash; a change that breaks only this test is a real regression, not a
  fixture problem.
- Every new figure/aggregation function needs a test against the tiny synthetic fixture in
  `tests/conftest.py` (`synthetic_run`), not the real committed result — hand-computable values
  keep assertions meaningful instead of "whatever the code currently outputs."
- `.venv/` is gitignored; recreate it, don't commit it.

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
