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
  (`just analysis-setup` from the repo root does the same thing, if [`just`](
  https://github.com/casey/just) is installed — see root `CLAUDE.md`.)
- Run the suite: `./.venv/Scripts/python.exe -m pytest -v`, from `analysis/` (or `just test`
  from the repo root). This is the
  scriptable path — use it for CI or when an agent needs to verify pass/fail non-interactively;
  it doesn't change with the picker below.
- `./.venv/Scripts/python.exe run_tests.py` (or `just experiment-gui`) opens a **GUI window**
  (`test_gui.py`, plain tkinter — no extra dependency, ships with Python) for configuring and
  running a sweep — check which algorithms (raw predictor registry keys) and which impairments
  (jitter/delay/loss, each with its own min/max/step, independently combinable into one sweep)
  to include, click Run, watch the sweep's own output stream live, then switch to the Figures tab
  to generate/view that run's charts. Checkboxes there also let you pick which figure *kinds* to
  generate (bar graphs, line graphs, table) — useful because a dense sweep's bar charts (one per
  profile) can otherwise bury the handful of line charts. `experiments/*.yaml` generation is
  `experiment_builder.py` (pure, unit tested) — the GUI just writes what it returns and shells
  out to `dotnet run -- sweep`. Needs a real display, not a piped/non-interactive shell — in this
  repo that's never an issue since `analysis/` already runs on the Windows-side Python (see setup
  above), so it opens as a normal Windows window. This is a human-facing convenience, not a
  replacement for `pytest -v`/`run_tests.py --all` above, which stay the way to verify
  `analysis/`'s own code.
- The Figures tab's **Delete Run** button permanently removes a `results/<exp>/<run>/`
  directory, after a confirmation dialog — the one intentional exception to root CLAUDE.md's
  "never touch `results/`" boundary, since it's a human clicking a button and confirming, not an
  agent acting on its own. It exists for clearing scratch sweeps (e.g. repeated
  `exp-gui-sweep` runs from this same GUI), not for routine cleanup — nothing a paper or writeup
  cites should ever go through it.
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
