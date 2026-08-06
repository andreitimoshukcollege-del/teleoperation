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
  profile) can otherwise bury the handful of line charts; whichever bar-chart kinds are requested
  are generated in parallel across profiles (`teleop_analysis/cli.py`'s `ProcessPoolExecutor`
  over `_generate_profile_figures`, one process-pool task per profile, fed a pre-grouped
  (`df.groupby("profile", observed=True)`) per-profile slice rather than the whole run's
  dataframe) rather than the sequential per-profile loop this used to be -- a 121-profile dense
  sweep's bar charts went from ~118s to ~23s in measurement, with byte-identical PNG output,
  since matplotlib rendering is CPU/Python-bound (unlike `io_utils.py`'s I/O-bound CSV reads, so
  a thread pool wouldn't give real parallelism here). Note `dict(df.groupby(...))` -- without
  wrapping in `iter()` -- raises `TypeError: 'str' object is not callable`: `GroupBy` has a
  `.keys` *attribute* (whatever was passed to `by=`), and `dict()` decides an argument is a
  mapping by checking `hasattr(arg, "keys")`, then calls it as `arg.keys()` -- calling the string
  itself. `combined-response` (the whole combined
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
  pattern as running a sweep), so switching figures doesn't hang the window. The currently shown
  figure always stretches to exactly fill `self.canvas_container`'s *current* pixel dimensions
  (`_fit_figure_to_container`, called both when a figure is first shown and on the container's
  own `<Configure>` event, so a live window resize re-fits it too, via `pack(fill=tk.BOTH,
  expand=True)`) -- an earlier version tried to preserve each figure's own native aspect ratio
  instead (letterboxing it within the container via `.place()`), but that left large blank
  margins above/below a figure whose aspect ratio didn't match the window's, which is worse than
  the different figure kinds simply looking differently proportioned against each other. The
  canvas and toolbar are rebuilt from scratch on *every* figure switch -- an earlier version tried
  to reuse them when the new figure matched the current one's pixel size, to avoid
  `NavigationToolbar2Tk`'s per-construction cost (it re-decodes every toolbar icon from disk),
  but that reuse path caused three separate rendering bugs in a row (stale pixels from a
  differently-sized previous figure, a resize race, and a further failure switching repeatedly
  while full-screened); a fresh `FigureCanvasTkAgg` is always guaranteed to have a correctly
  sized backing buffer, which reuse evidently isn't across every window/figure-size combination.
  Rebuilding the canvas on every click has its own real gotcha on a >100%-scaled Windows
  display, though: `FigureCanvasBase.__init__` captures `figure._original_dpi = figure.dpi`
  ("we don't want to scale up the figure DPI more than once," per matplotlib's own comment
  there) assuming one canvas per figure for its whole lifetime, then a `<Map>` callback scales
  `dpi` up by the display's device pixel ratio -- rebuilding the canvas instead means each new
  one recaptures `_original_dpi` from whatever the *previous* canvas already scaled `dpi` to,
  compounding the scale-up (and every point-sized element -- fonts, line widths, markers --
  with it) on every single figure switch. `_reset_figure_dpi_to_native` resets `fig.dpi` back to
  its build-time value (stashed once as `fig._native_dpi` when the figure first enters the
  cache) before every construction, breaking that chain -- verified directly against the real
  `backend_bases.py` mechanism (`FigureCanvasTkAgg._set_device_pixel_ratio`), not just this
  reimplementation, since it's otherwise inert (and unreproducible) at 100% display scaling.
  Every axes gets a `ylim_changed` callback (`_clamp_ylim_nonnegative`) pulling the
  lower bound back to 0 if zoom/pan ever pushes it negative -- every metric plotted here is a
  non-negative magnitude (a distance in mm, a one-way delay in ms), so a negative y value is
  never real, only ever a zoom/pan artifact. The line-chart figures
  (`impairment-response`/`combined-response`, whose x-axis is a network-impairment magnitude or
  a step index -- never negative either) get the same treatment on x
  (`_clamp_xlim_nonnegative`), applied once immediately on load (fixing matplotlib's own default
  autoscale margin, not just future zoom/pan) as well as on every subsequent change. The
  bar-chart figures (`error-cost`/`latency`/`stack-comparison`) deliberately do *not* get the
  x-clamp -- their leftmost bar group is centered at x=0 and extends slightly left of it
  (`figures/_bars.py`), so clamping there would clip it. `_fit_figure_to_container` also
  recomputes the caption's bottom margin on every resize (`_caption_bottom_fraction`, from a
  fixed `_CAPTION_MARGIN_INCHES = 0.7` capped at `_MAX_CAPTION_FRACTION` of the figure's height)
  rather than leaving each figure's one-shot `fig.tight_layout(rect=(0, MARGIN, 1, 1))` margin
  (a *fraction*, sized for that figure's small build-time height) in place -- otherwise the same
  fraction becomes a much larger *absolute* gap once the container stretches the figure to fill a
  tall window, since the caption's font size is fixed in points, not scaled with the figure. None
  of the 5 figure builders set a persistent layout engine, so this later `subplots_adjust` call
  doesn't conflict with their one-shot `tight_layout`.
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
