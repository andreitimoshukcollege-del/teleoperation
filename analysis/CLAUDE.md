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
  to include. A separate "Combined impairments" section below that uses the same per-axis
  min/max/step controls to generate `combo__delay-<N>ms__jitter-<N>ms__loss-<N>pct`-style
  profiles (docs/adr/0006-combined-impairment-profiles.md) — a **lockstep** walk across whichever
  axes are checked (point *i* takes the i-th value of every checked axis, so a 4-point delay
  range and a 4-point jitter range together produce 4 combined profiles, not 16). Axes don't need
  matching point counts -- the walk runs as long as the longest one, and any shorter axis holds
  at its last value for the remaining steps. For studying how the system degrades as the whole
  link gets simultaneously worse, plotted as a single line chart (see `combined-response` below)
  rather than one isolated variable at a time. Warns and asks for confirmation before launching a
  sweep whose combined-profile count exceeds 200, as a backstop. Click Run, watch the sweep's own
  output stream live, then switch to the Figures tab
  to generate/view that run's charts. Checkboxes there also let you pick which figure *kinds* to
  generate (bar graphs, line graphs, table) — useful because a dense sweep's bar charts (one per
  profile) can otherwise bury the handful of line charts; `combined-response` (the whole combined
  sweep as one chart, x-tick labels spelling out every axis's value at each step) is grouped
  under "Line graphs" alongside `impairment-response`. Selecting a figure embeds the **live
  matplotlib chart** (`FigureCanvasTkAgg`), not the saved PNG -- crisp at any zoom level since
  matplotlib redraws from the real data instead of resampling a raster image, and matplotlib's
  own `NavigationToolbar2Tk` (below the chart) gives pan/zoom-rectangle/save/home for free. The
  scroll wheel additionally zooms straight into wherever the cursor is pointing (in real data
  coordinates, via `_zoom_axes_around_point`), rather than needing the toolbar's zoom-rectangle
  tool for that. Each `figures/*.py` module exposes a `build_<x>_figure(...)` function (returns
  the `Figure` + caption, no I/O) alongside its existing `plot_<x>(...)` (builds, then saves to
  `results/<run>/figures/*.png` and closes) -- `test_gui.py`'s `_figure_builder_for_filename`
  maps a listed filename back to the right `build_*` function so the live view always matches
  what's on disk. The saved PNG stays the citable, disk-based artifact either way; the live
  embed is purely a nicer way to look at it. Built figures are cached per `(run, filename)` for
  the life of the GUI process (a run's data is immutable once written -- `results/CLAUDE.md`) and
  a cache miss builds off the Tk main thread (same background-thread-plus-`root.after`-polling
  pattern as running a sweep), so switching figures doesn't hang the window. Every figure is
  explicitly resized to the container's *current* pixel dimensions (`_show_figure`, before the
  canvas is even created) rather than relying on matplotlib's own `<Configure>`-triggered
  auto-resize to grow it afterward -- that auto-resize is reliable when the *same* figure is
  already showing and the window changes size around it, but not when a new, differently-sized
  canvas gets created directly into an already-large container (e.g. switching figures while
  already full-screened), which otherwise left the chart rendered at the wrong size. The canvas
  and toolbar are then reused across figure switches only when the new (now container-sized)
  figure is the exact same size as the one showing (`_figures_same_size`) --
  `FigureCanvasTkAgg`'s backing buffer is sized once, at construction, so reusing it across a
  size change (e.g. the window itself was resized since the last figure, or `combined-response`'s
  width, which grows with sweep density) leaves the previous figure's pixels visible around the
  edges of the new one; a size change always gets a fresh canvas/toolbar instead. Every axes gets
  a `ylim_changed` callback (`_clamp_ylim_nonnegative`) pulling the
  lower bound back to 0 if zoom/pan ever pushes it negative -- every metric plotted here is a
  non-negative magnitude (a distance in mm, a one-way delay in ms), so a negative y value is
  never real, only ever a zoom/pan artifact. The line-chart figures
  (`impairment-response`/`combined-response`, whose x-axis is a network-impairment magnitude or
  a step index -- never negative either) get the same treatment on x
  (`_clamp_xlim_nonnegative`), applied once immediately on load (fixing matplotlib's own default
  autoscale margin, not just future zoom/pan) as well as on every subsequent change. The
  bar-chart figures (`error-cost`/`latency`/`stack-comparison`) deliberately do *not* get the
  x-clamp -- their leftmost bar group is centered at x=0 and extends slightly left of it
  (`figures/_bars.py`), so clamping there would clip it.
  `experiments/*.yaml` generation is
  `experiment_builder.py` (pure, unit tested) — the GUI just writes what it returns and shells
  out to `dotnet run -- sweep`. Needs a real display, not a piped/non-interactive shell — in this
  repo that's never an issue since `analysis/` already runs on the Windows-side Python (see setup
  above), so it opens as a normal Windows window. This is a human-facing convenience, not a
  replacement for `pytest -v`/`run_tests.py --all` above, which stay the way to verify
  `analysis/`'s own code. `launch()` also does two DPI things Windows needs, in order: tell
  Windows the process handles its own scaling (`_set_windows_dpi_awareness`, otherwise the whole
  window comes out blurry on a >100%-scaled display), then tell Tk the real DPI
  (`_apply_dpi_scaling`, so widgets come out a normal physical size instead of sharp-but-tiny) --
  and, since that second step can otherwise make maximizing/full-screening request a window
  bigger than the screen (a >100%-scaled display's widgets add up to more space than the display
  has), `_cap_window_to_screen` caps the window's max size to the screen's own bounds right
  after.
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
