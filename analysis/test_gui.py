"""Tkinter GUI for analysis/: configure and run a sweep, then view its figures.

Launched by run_tests.py when invoked without --all. Needs a real display -- this project's
analysis/ venv is a Windows-native Python reached via WSL interop (see analysis/CLAUDE.md), so
this opens as a normal Windows window with no extra display-server setup.
"""
from __future__ import annotations

import queue
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from typing import Callable, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import experiment_builder
from teleop_analysis import io_utils
from teleop_analysis.figures import (
    combined_response,
    error_vs_cost,
    impairment_response,
    latency_distribution,
    stack_comparison,
)
from teleop_analysis.manifest import Manifest

REPO_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = REPO_ROOT / "results"
CORE_EVAL_DIR = REPO_ROOT / "core" / "Teleop.Eval"
EXPERIMENTS_DIR = REPO_ROOT / "experiments"
GENERATED_YAML_PATH = EXPERIMENTS_DIR / "exp-gui-sweep.yaml"

# Registry/Registries.cs's Predictors keys -- hardcoded the same way labels.py's
# FRIENDLY_STACK_NAMES already is (no runtime way to query the C# registry from Python). Update
# both places by hand when a new predictor is registered.
PREDICTORS = ("none", "const-vel", "double-exp")

# Defaults match experiments/exp-002-impairment-sensitivity.yaml's current density.
AXIS_DEFAULTS = {
    "jitter": {"min": "0", "max": "60", "step": "1", "unit": "ms"},
    "delay": {"min": "0", "max": "300", "step": "1", "unit": "ms"},
    "loss": {"min": "0", "max": "5", "step": "0.1", "unit": "%"},
}


def discover_runs(results_dir: Path = RESULTS_DIR) -> List[Path]:
    """Every results/<exp>/<run>/ directory that has a manifest.json -- a directory without one
    isn't a real run (results/CLAUDE.md: a result without a manifest isn't citable), so it's
    never offered here. Newest-looking (by path, which sorts with the ISO-8601 run timestamp) first.
    """
    if not results_dir.is_dir():
        return []
    return sorted((p.parent for p in results_dir.glob("*/*/manifest.json")), reverse=True)


def figures_for_run(run_dir: Path) -> List[Path]:
    """PNGs already generated for a run, or an empty list if `report` hasn't been run for it yet."""
    figures_dir = run_dir / "figures"
    if not figures_dir.is_dir():
        return []
    return sorted(figures_dir.glob("*.png"))


def delete_run(run_dir: Path) -> None:
    """Irreversibly removes a results/<exp>/<run>/ directory. root CLAUDE.md documents results/
    as append-only -- this exists for clearing scratch sweeps (e.g. repeated exp-gui-sweep runs),
    not for routine cleanup of anything citable. The GUI is the only caller and always confirms
    with the human first; this function itself does not ask.
    """
    shutil.rmtree(run_dir)


# Groups teleop_analysis.cli's --figures kinds by chart shape, for the Figures tab's checkboxes.
# A dense sweep (hundreds of profiles) makes the bar-graph group alone hundreds of PNGs -- one
# per profile per bar-chart kind -- easy to let bury the handful of line-graph PNGs in the
# figure list, so letting either group be skipped is the point, not just a convenience.
FIGURE_GROUPS = {
    "Bar graphs": ("error-cost", "latency", "stack-comparison"),
    "Line graphs": ("impairment-response", "combined-response"),
    "Table": ("table",),
}

# Figures tab: the selected figure is embedded as a live matplotlib Figure (FigureCanvasTkAgg),
# not a saved PNG -- crisp at any zoom level (matplotlib redraws from the real data instead of
# resampling a raster image) and matplotlib's own NavigationToolbar2Tk supplies pan/zoom-
# rectangle/save/reset. Scroll-wheel zoom is added on top of that toolbar, anchored on the data
# point under the cursor.
_ZOOM_STEP = 1.25

# Maps a per-profile figure's filename *suffix* to the build_* function that (re)builds it as a
# live Figure -- the profile name is whatever's left after stripping the suffix, mirroring how
# each figures/*.py module names its own saved PNG (f"{profile}__<suffix>").
_PER_PROFILE_BUILDERS: Dict[str, Callable[[pd.DataFrame, Manifest, str], Tuple[Figure, str]]] = {
    "__error_vs_cost.png": error_vs_cost.build_error_vs_cost_figure,
    "__latency.png": latency_distribution.build_latency_distribution_figure,
    "__stack_comparison.png": stack_comparison.build_stack_comparison_figure,
}

# Whole-run figures (no per-profile suffix to strip) keyed by their exact, fixed filename.
_FIXED_BUILDERS: Dict[str, Callable[[pd.DataFrame, Manifest], Optional[Tuple[Figure, str]]]] = {
    "impairment__correction_vs_jitter.png": impairment_response.build_correction_vs_jitter_figure,
    "impairment__prediction_error_vs_jitter.png": impairment_response.build_prediction_error_vs_jitter_figure,
    "impairment__correction_vs_delay.png": impairment_response.build_correction_vs_delay_figure,
    "impairment__prediction_error_vs_delay.png": impairment_response.build_prediction_error_vs_delay_figure,
    "impairment__correction_vs_loss.png": impairment_response.build_correction_vs_loss_figure,
    "impairment__prediction_error_vs_loss.png": impairment_response.build_prediction_error_vs_loss_figure,
    "combined__correction.png": combined_response.build_correction_vs_combined_figure,
    "combined__prediction_error.png": combined_response.build_prediction_error_vs_combined_figure,
}


def _figure_builder_for_filename(
    filename: str,
) -> Optional[Callable[[pd.DataFrame, Manifest], Optional[Tuple[Figure, str]]]]:
    """Maps a figure PNG's filename back to the in-process function that (re)builds it as a live
    matplotlib Figure, for the Figures tab's embedded live view -- mirrors the filename patterns
    each figures/*.py module's plot_* wrapper already writes to disk. `None` for a name that
    doesn't match any known figure kind ("table"'s summary_table.csv never reaches this at all --
    it isn't a .png, so figures_for_run never lists it).
    """
    if filename in _FIXED_BUILDERS:
        return _FIXED_BUILDERS[filename]
    for suffix, build_fn in _PER_PROFILE_BUILDERS.items():
        if filename.endswith(suffix):
            profile = filename[: -len(suffix)]
            return lambda df, manifest: build_fn(df, manifest, profile)
    return None


def _zoom_axes_around_point(ax, x_px: float, y_px: float, zoom_in: bool) -> None:
    """Rescales `ax`'s x/y limits by _ZOOM_STEP around the data point under (x_px, y_px) (pixel
    coordinates in the figure's own space, bottom-left origin like matplotlib's), keeping that
    point fixed on screen -- "zooming into where the mouse is pointing," on real axis limits
    instead of a raster image, so it's exact at any zoom level instead of softening.
    """
    data_x, data_y = ax.transData.inverted().transform((x_px, y_px))
    factor = (1.0 / _ZOOM_STEP) if zoom_in else _ZOOM_STEP
    xlo, xhi = ax.get_xlim()
    ylo, yhi = ax.get_ylim()
    ax.set_xlim(data_x - (data_x - xlo) * factor, data_x + (xhi - data_x) * factor)
    ax.set_ylim(data_y - (data_y - ylo) * factor, data_y + (yhi - data_y) * factor)


def _select_zoom_target(axes, x_px: float, y_px: float):
    """Which of `axes` a scroll-wheel zoom at pixel position (x_px, y_px) should target. A
    single-axes figure has no "which panel" to disambiguate, so it's always the target
    regardless of exact cursor position -- a strict bbox-containment check there misses whenever
    the cursor is over a chart's caption/tick-label margin rather than the plotted area itself,
    which for e.g. combined_response.py's large bottom margin + rotated tick labels is often
    exactly where a user points to zoom into a dense chart's x-axis. A multi-axes figure
    (side-by-side panels) still needs the containment check to pick the right one; `None` if the
    point is over neither panel.
    """
    if not axes:
        return None
    if len(axes) == 1:
        return axes[0]
    return next((ax for ax in axes if ax.bbox.contains(x_px, y_px)), None)


def _clamp_ylim_nonnegative(ax) -> None:
    """Every metric this GUI plots is a non-negative magnitude -- a Euclidean position error in
    mm (docs/metrics.md's `PoseMath.PositionErrorMeters`), a correction distance in mm, or a
    one-way delay in ms. Negative y is never a real value; it's only ever an artifact of
    interactive zoom/pan pushing the visible range past the data's own floor of 0. Registered as
    a `ylim_changed` callback (see _poll_figure_build) so it self-corrects regardless of what
    caused the change -- our own scroll zoom, or the toolbar's pan/zoom-rectangle tools, which
    have no notion of this constraint on their own.

    Calling `set_ylim` again here re-fires `ylim_changed` once more, but with `ylo` now exactly
    0 (not negative), so the guard below is false on that second call and it stops there --
    bounded recursion, not a loop.
    """
    ylo, yhi = ax.get_ylim()
    if ylo < 0:
        ax.set_ylim(0, yhi)


def _clamp_xlim_nonnegative(ax) -> None:
    """Same idea as _clamp_ylim_nonnegative, for x -- but only wired up for the line-chart
    figures (impairment_response.py's jitter/delay/loss, combined_response.py's step index; see
    _poll_figure_build's `filename in _FIXED_BUILDERS` check), never the bar-chart figures
    (error_vs_cost.py etc.): a bar chart's leftmost group is centered *at* x=0 and extends
    slightly left of it (`_bars.py`'s `x - width`), so clamping there would clip it.
    """
    xlo, xhi = ax.get_xlim()
    if xlo < 0:
        ax.set_xlim(0, xhi)


def _figures_same_size(a: Optional[Figure], b: Optional[Figure]) -> bool:
    """Whether two figures would produce the same `FigureCanvasTkAgg` backing-buffer size --
    the condition `_show_figure` uses to decide whether reusing the existing canvas/toolbar is
    safe (see its docstring for why a size change specifically must not reuse them)."""
    if a is None or b is None:
        return False
    return tuple(a.get_size_inches()) == tuple(b.get_size_inches()) and a.dpi == b.dpi


def build_report_command(run_dir: Path, figures: Optional[str] = None) -> List[str]:
    """The exact subprocess argv used to (re)generate a run's figures -- same CLI `just report`
    wraps. Absolute path so it doesn't matter what the subprocess's cwd ends up being. `figures`
    is teleop_analysis.cli's own --figures value (comma-separated kinds); omitted entirely means
    its default (everything).
    """
    command = [sys.executable, "-m", "teleop_analysis.cli", str(run_dir.resolve())]
    if figures:
        command += ["--figures", figures]
    return command


def build_sweep_command(yaml_path: Path) -> List[str]:
    """The exact subprocess argv used to run a sweep from a generated experiment YAML. This
    process is a native Windows process throughout (the Windows-side python.exe that runs this
    GUI, per analysis/CLAUDE.md), so plain pathlib absolute paths already come out Windows-style
    -- no WSL/POSIX path translation needed here, unlike shelling out from a WSL shell.
    """
    return ["dotnet", "run", "--project", str(CORE_EVAL_DIR), "--", "sweep", str(yaml_path.resolve())]


class PickerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("analysis/ toolkit")

        self.process: Optional[subprocess.Popen] = None
        self.last_exit_code: Optional[int] = None
        self.output_queue: "queue.Queue" = queue.Queue()
        self._sweep_running = False

        self.predictor_vars: Dict[str, tk.BooleanVar] = {}
        self.axis_enabled_vars: Dict[str, tk.BooleanVar] = {}
        self.axis_entries: Dict[str, Dict[str, ttk.Entry]] = {}
        self.combined_axis_enabled_vars: Dict[str, tk.BooleanVar] = {}
        self.combined_axis_entries: Dict[str, Dict[str, ttk.Entry]] = {}

        self.run_display_to_path: Dict[str, Path] = {}
        self.figures_queue: "queue.Queue" = queue.Queue()
        self._current_figure: Optional[Figure] = None
        self._current_figure_canvas: Optional[FigureCanvasTkAgg] = None
        self._current_toolbar: Optional[NavigationToolbar2Tk] = None
        self._run_data_cache: Optional[Tuple[Path, Manifest, pd.DataFrame]] = None
        self.figure_group_vars: Dict[str, tk.BooleanVar] = {}

        # (run, filename) -> (Figure, caption) -- a run's data is immutable once written
        # (results/CLAUDE.md: append-only), so a cached figure never needs invalidating for the
        # same run; only closed (plt.close) when the whole cache is torn down in _on_close.
        self._figure_cache: Dict[Tuple[Path, str], Tuple[Figure, str]] = {}
        self._figure_build_queue: "queue.Queue" = queue.Queue()
        self._figure_request_id = 0

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True)
        notebook.add(self._build_experiment_tab(notebook), text="Experiment")
        notebook.add(self._build_figures_tab(notebook), text="Figures")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- Experiment tab ----

    def _build_experiment_tab(self, notebook: ttk.Notebook) -> ttk.Frame:
        container = ttk.Frame(notebook, padding=8)

        ttk.Label(container, text="Algorithms", font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        algo_row = ttk.Frame(container)
        algo_row.pack(anchor="w", pady=(0, 8))
        for predictor in PREDICTORS:
            var = tk.BooleanVar(value=True)
            self.predictor_vars[predictor] = var
            ttk.Checkbutton(algo_row, text=predictor, variable=var).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(container, text="Impairments", font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        for axis, defaults in AXIS_DEFAULTS.items():
            row = ttk.Frame(container)
            row.pack(anchor="w", pady=2, fill=tk.X)

            enabled = tk.BooleanVar(value=True)
            self.axis_enabled_vars[axis] = enabled
            ttk.Checkbutton(row, text=axis, variable=enabled, width=8).pack(side=tk.LEFT)

            entries: Dict[str, ttk.Entry] = {}
            for field in ("min", "max", "step"):
                ttk.Label(row, text=field).pack(side=tk.LEFT, padx=(8, 2))
                entry = ttk.Entry(row, width=8)
                entry.insert(0, defaults[field])
                entry.pack(side=tk.LEFT)
                entries[field] = entry
            ttk.Label(row, text=defaults["unit"]).pack(side=tk.LEFT, padx=(4, 0))
            self.axis_entries[axis] = entries

        ttk.Label(
            container, text="Combined impairments (cross-product -- check 2+ to combine)",
            font=("TkDefaultFont", 10, "bold"),
        ).pack(anchor="w", pady=(8, 0))
        for axis, defaults in AXIS_DEFAULTS.items():
            row = ttk.Frame(container)
            row.pack(anchor="w", pady=2, fill=tk.X)

            enabled = tk.BooleanVar(value=False)
            self.combined_axis_enabled_vars[axis] = enabled
            ttk.Checkbutton(row, text=axis, variable=enabled, width=8).pack(side=tk.LEFT)

            entries: Dict[str, ttk.Entry] = {}
            for field in ("min", "max", "step"):
                ttk.Label(row, text=field).pack(side=tk.LEFT, padx=(8, 2))
                entry = ttk.Entry(row, width=8)
                entry.insert(0, defaults[field])
                entry.pack(side=tk.LEFT)
                entries[field] = entry
            ttk.Label(row, text=defaults["unit"]).pack(side=tk.LEFT, padx=(4, 0))
            self.combined_axis_entries[axis] = entries

        settings_row = ttk.Frame(container)
        settings_row.pack(anchor="w", pady=(8, 0), fill=tk.X)
        ttk.Label(settings_row, text="Seeds").pack(side=tk.LEFT)
        self.seeds_entry = ttk.Entry(settings_row, width=16)
        self.seeds_entry.insert(0, "1,2,3,4,5")
        self.seeds_entry.pack(side=tk.LEFT, padx=(4, 16))
        ttk.Label(settings_row, text="Experiment ID").pack(side=tk.LEFT)
        self.experiment_id_entry = ttk.Entry(settings_row, width=24)
        self.experiment_id_entry.insert(0, "exp-gui-sweep")
        self.experiment_id_entry.pack(side=tk.LEFT, padx=(4, 0))

        button_row = ttk.Frame(container)
        button_row.pack(fill=tk.X, pady=(8, 0))
        self.run_sweep_button = ttk.Button(
            button_row, text="Run Sweep", command=self._on_run_sweep_clicked
        )
        self.run_sweep_button.pack(side=tk.LEFT)
        self.sweep_status = ttk.Label(button_row, text="")
        self.sweep_status.pack(side=tk.RIGHT)

        self.output = ScrolledText(container, height=16, font=("Consolas", 10), state=tk.DISABLED)
        self.output.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.output.tag_config("pass", foreground="#1a7f37")
        self.output.tag_config("fail", foreground="#cf222e")

        return container

    def _append_output(self, text: str, tag: Optional[str] = None) -> None:
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text, tag or ())
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _selected_predictors(self) -> List[str]:
        return [p for p, var in self.predictor_vars.items() if var.get()]

    def _build_profiles(self) -> Tuple[Optional[List[str]], Optional[str]]:
        """(profiles, None) on success, or (None, error message) -- never both/neither."""
        profiles: List[str] = []
        point_fns = {
            "jitter": experiment_builder.jitter_points,
            "delay": experiment_builder.delay_points,
            "loss": experiment_builder.loss_points,
        }
        for axis, enabled in self.axis_enabled_vars.items():
            if not enabled.get():
                continue
            entries = self.axis_entries[axis]
            try:
                min_v = float(entries["min"].get())
                max_v = float(entries["max"].get())
                step_v = float(entries["step"].get())
            except ValueError:
                return None, f"{axis}: min/max/step must be numbers"
            try:
                profiles.extend(point_fns[axis](min_v, max_v, step_v))
            except ValueError as exc:
                return None, f"{axis}: {exc}"

        combined_values: Dict[str, List[float]] = {}
        for axis, enabled in self.combined_axis_enabled_vars.items():
            if not enabled.get():
                continue
            entries = self.combined_axis_entries[axis]
            try:
                min_v = float(entries["min"].get())
                max_v = float(entries["max"].get())
                step_v = float(entries["step"].get())
            except ValueError:
                return None, f"combined {axis}: min/max/step must be numbers"
            try:
                combined_values[axis] = experiment_builder.axis_points(min_v, max_v, step_v)
            except ValueError as exc:
                return None, f"combined {axis}: {exc}"

        if combined_values:
            try:
                combined_profiles = experiment_builder.combined_points(
                    delay_ms=combined_values.get("delay", ()),
                    jitter_ms=combined_values.get("jitter", ()),
                    loss_pct=combined_values.get("loss", ()),
                )
            except ValueError as exc:
                return None, f"combined impairments: {exc}"

            if len(combined_profiles) > 200 and not messagebox.askyesno(
                "Large combined sweep",
                f"This will generate {len(combined_profiles)} combined profiles "
                f"(before multiplying by algorithms and seeds). Continue?",
            ):
                return None, "Combined sweep cancelled."

            profiles.extend(combined_profiles)

        return profiles, None

    def _on_run_sweep_clicked(self) -> None:
        predictors = self._selected_predictors()
        if not predictors:
            self.sweep_status.config(text="Select at least one algorithm.")
            return

        profiles, error = self._build_profiles()
        if error:
            self.sweep_status.config(text=error)
            return
        if not profiles:
            self.sweep_status.config(text="Select at least one impairment.")
            return

        try:
            seeds = [int(s.strip()) for s in self.seeds_entry.get().split(",") if s.strip()]
        except ValueError:
            self.sweep_status.config(text="Seeds must be a comma-separated list of integers.")
            return
        if not seeds:
            self.sweep_status.config(text="At least one seed is required.")
            return

        experiment_id = self.experiment_id_entry.get().strip() or "exp-gui-sweep"
        yaml_text = experiment_builder.build_experiment_yaml(experiment_id, predictors, seeds, profiles)
        GENERATED_YAML_PATH.write_text(yaml_text)

        self.run_sweep_button.config(state=tk.DISABLED)
        self.sweep_status.config(text="")
        self._sweep_running = True
        self.output.configure(state=tk.NORMAL)
        self.output.delete("1.0", tk.END)
        self.output.configure(state=tk.DISABLED)
        self._append_output(
            f"Running: {len(predictors)} algorithm(s) x {len(profiles)} profile(s) x "
            f"{len(seeds)} seed(s)...\n\n"
        )

        threading.Thread(target=self._run_sweep_in_background, daemon=True).start()
        self.root.after(50, self._poll_sweep_output)

    def _run_sweep_in_background(self) -> None:
        self.process = subprocess.Popen(
            build_sweep_command(GENERATED_YAML_PATH),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.output_queue.put(line)
        exit_code = self.process.wait()
        self.process = None
        self.output_queue.put(("__done__", exit_code))

    def _poll_sweep_output(self) -> None:
        try:
            while True:
                item = self.output_queue.get_nowait()
                if isinstance(item, tuple):
                    _, exit_code = item
                    self.last_exit_code = exit_code
                    tag = "pass" if exit_code == 0 else "fail"
                    label = "DONE" if exit_code == 0 else "FAILED"
                    self._append_output(f"\n{label}\n", tag)
                    self.run_sweep_button.config(state=tk.NORMAL)
                    self._sweep_running = False
                    self._refresh_run_list()  # new run shows up in Figures immediately
                    return
                self._append_output(item)
        except queue.Empty:
            pass
        if self._sweep_running:
            self.root.after(50, self._poll_sweep_output)

    # ---- Figures tab ----

    def _build_figures_tab(self, notebook: ttk.Notebook) -> ttk.Frame:
        container = ttk.Frame(notebook, padding=8)

        top_row = ttk.Frame(container)
        top_row.pack(fill=tk.X)
        ttk.Label(top_row, text="Run:").pack(side=tk.LEFT)
        self.run_combo = ttk.Combobox(top_row, state="readonly", width=45)
        self.run_combo.pack(side=tk.LEFT, padx=(6, 6))
        self.run_combo.bind("<<ComboboxSelected>>", lambda e: self._refresh_figure_list())
        self.delete_run_button = ttk.Button(
            top_row, text="Delete Run", command=self._on_delete_run_clicked
        )
        self.delete_run_button.pack(side=tk.LEFT, padx=(0, 12))

        for group_name in FIGURE_GROUPS:
            var = tk.BooleanVar(value=True)
            self.figure_group_vars[group_name] = var
            ttk.Checkbutton(top_row, text=group_name, variable=var).pack(side=tk.LEFT, padx=(0, 8))

        self.generate_button = ttk.Button(
            top_row, text="Generate / Refresh", command=self._on_generate_figures_clicked
        )
        self.generate_button.pack(side=tk.LEFT)
        self.figures_status = ttk.Label(top_row, text="")
        self.figures_status.pack(side=tk.RIGHT)

        hint_row = ttk.Frame(container)
        hint_row.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(
            hint_row,
            text="Scroll wheel over the chart zooms into wherever the cursor is pointing; "
            "the toolbar below the chart also has pan/zoom-rectangle/save/reset.",
            foreground="#666666",
        ).pack(side=tk.LEFT)

        body = ttk.Frame(container)
        body.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.figure_listbox = tk.Listbox(body, width=32, exportselection=False)
        self.figure_listbox.pack(side=tk.LEFT, fill=tk.Y)
        self.figure_listbox.bind("<<ListboxSelect>>", lambda e: self._on_figure_selected())

        image_frame = ttk.Frame(body)
        image_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        # The figure is a live matplotlib chart (FigureCanvasTkAgg), not a saved PNG -- crisp at
        # any zoom level since matplotlib redraws from the real data instead of resampling a
        # raster image. self.toolbar_container/self.canvas_container get torn down and rebuilt
        # each time a different figure is selected (see _show_figure/_clear_image).
        self.toolbar_container = ttk.Frame(image_frame)
        self.toolbar_container.pack(side=tk.TOP, fill=tk.X)
        self.canvas_container = ttk.Frame(image_frame)
        self.canvas_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self._refresh_run_list()
        return container

    def _refresh_run_list(self) -> None:
        runs = discover_runs()
        self.run_display_to_path = {
            str(run.relative_to(RESULTS_DIR)).replace("\\", "/"): run for run in runs
        }
        values = list(self.run_display_to_path.keys())
        self.run_combo["values"] = values
        if values and not self.run_combo.get():
            self.run_combo.set(values[0])
            self._refresh_figure_list()

    def _selected_run(self) -> Optional[Path]:
        return self.run_display_to_path.get(self.run_combo.get())

    def _on_delete_run_clicked(self) -> None:
        run = self._selected_run()
        if run is None:
            self.figures_status.config(text="No run selected.")
            return

        display_name = self.run_combo.get()
        confirmed = messagebox.askyesno(
            "Delete run",
            f"Permanently delete {display_name}?\n\n"
            "This cannot be undone. results/ is meant to be append-only -- only delete runs "
            "you know are scratch/test sweeps, not anything a paper or writeup cites.",
        )
        if not confirmed:
            return

        delete_run(run)
        self.run_combo.set("")
        self._refresh_run_list()
        self._refresh_figure_list()
        self.figures_status.config(text=f"Deleted {display_name}.")

    def _refresh_figure_list(self) -> None:
        self.figure_listbox.delete(0, tk.END)
        self._clear_image()
        run = self._selected_run()
        if run is None:
            return
        figures = figures_for_run(run)
        for path in figures:
            self.figure_listbox.insert(tk.END, path.name)
        self.figures_status.config(text="" if figures else "No figures yet -- click Generate.")

    def _run_data(self, run: Path) -> Tuple[Manifest, pd.DataFrame]:
        """Cached (manifest, df) for `run`, recomputed only when the selected run changes -- a
        run's metrics.csv is immutable once written (results/CLAUDE.md: append-only), so there's
        nothing to invalidate for the same path; a new sweep completing always writes a new,
        differently-timestamped run directory instead.
        """
        if self._run_data_cache is None or self._run_data_cache[0] != run:
            manifest, df = io_utils.discover_run(run)
            self._run_data_cache = (run, manifest, df)
        _, manifest, df = self._run_data_cache
        return manifest, df

    def _clear_image(self) -> None:
        """Hides the current display. Never closes a Figure -- every Figure that ever reaches
        `_current_figure` is already sitting in `self._figure_cache` by then (see
        `_poll_figure_build`), and closing it here would break re-selecting it later in the same
        session. Cached figures are only closed in `_on_close`, on app shutdown.
        """
        if self._current_toolbar is not None:
            self._current_toolbar.destroy()
            self._current_toolbar = None
        if self._current_figure_canvas is not None:
            self._current_figure_canvas.get_tk_widget().destroy()
            self._current_figure_canvas = None
        self._current_figure = None

    def _show_figure(self, fig: Figure) -> None:
        """Displays `fig`, reusing the existing canvas/toolbar only when the new figure is
        exactly the same size as the one currently shown -- recreating `NavigationToolbar2Tk` on
        every figure switch was the single largest fixed cost in the old per-click rebuild (it
        re-decodes and resizes every toolbar icon PNG from disk on construction), so it's worth
        avoiding when we safely can.

        Reuse is deliberately size-gated: `FigureCanvasTkAgg`'s internal backing buffer
        (`_tkphoto`) is sized once, at construction, to that first figure's pixel dimensions.
        Reconfiguring the *outer* Tk widget's width/height (as an earlier version of this method
        did) does not resize that inner buffer -- switching to a smaller figure then left the
        previous, larger figure's pixels visible around the edges of the new one, which is
        exactly the "stack on top of each other" bug this guards against. Only a fresh
        `FigureCanvasTkAgg` is guaranteed to have a correctly sized buffer, so any size change
        (combined_response's width grows with sweep density; every other figure type is a fixed
        size) falls back to a full rebuild.
        """
        if (
            self._current_figure_canvas is not None
            and _figures_same_size(fig, self._current_figure)
        ):
            canvas = self._current_figure_canvas
            canvas.figure = fig
            fig.set_canvas(canvas)
        else:
            self._clear_image()
            canvas = FigureCanvasTkAgg(fig, master=self.canvas_container)
            widget = canvas.get_tk_widget()
            widget.pack(fill=tk.BOTH, expand=True)
            widget.bind("<MouseWheel>", self._on_canvas_scroll)
            toolbar = NavigationToolbar2Tk(canvas, self.toolbar_container)
            toolbar.update()
            self._current_figure_canvas = canvas
            self._current_toolbar = toolbar

        self._current_figure = fig
        canvas.draw()

    def _on_figure_selected(self) -> None:
        selection = self.figure_listbox.curselection()
        run = self._selected_run()
        if not selection or run is None:
            return
        filename = self.figure_listbox.get(selection[0])
        builder = _figure_builder_for_filename(filename)
        if builder is None:
            self._clear_image()
            self.figures_status.config(text=f"No live view available for {filename}.")
            return

        self._figure_request_id += 1
        request_id = self._figure_request_id

        cache_key = (run, filename)
        cached = self._figure_cache.get(cache_key)
        if cached is not None:
            fig, _caption = cached
            self._show_figure(fig)
            self.figures_status.config(text="")
            return

        self.figures_status.config(text="Loading...")
        threading.Thread(
            target=self._build_figure_in_background,
            args=(request_id, run, filename, builder),
            daemon=True,
        ).start()
        self.root.after(50, self._poll_figure_build)

    def _build_figure_in_background(
        self, request_id: int, run: Path, filename: str, builder: Callable
    ) -> None:
        manifest, df = self._run_data(run)
        result = builder(df, manifest)
        self._figure_build_queue.put((request_id, run, filename, result))

    def _poll_figure_build(self) -> None:
        try:
            request_id, run, filename, result = self._figure_build_queue.get_nowait()
        except queue.Empty:
            self.root.after(50, self._poll_figure_build)
            return

        if request_id != self._figure_request_id:
            return  # stale -- the user has since selected a different figure

        if result is None:
            self._clear_image()
            self.figures_status.config(text=f"Nothing to show for {filename} in this run.")
            return

        fig, caption = result
        # Connected once, here, when the figure first enters the cache -- not in _show_figure,
        # which also runs on every cache-hit re-display of an already-connected figure.
        clamp_x = filename in _FIXED_BUILDERS  # line charts only -- see _clamp_xlim_nonnegative
        for ax in fig.axes:
            ax.callbacks.connect("ylim_changed", _clamp_ylim_nonnegative)
            _clamp_ylim_nonnegative(ax)  # also fixes up the initial autoscaled view, not just future zoom/pan
            if clamp_x:
                ax.callbacks.connect("xlim_changed", _clamp_xlim_nonnegative)
                _clamp_xlim_nonnegative(ax)
        self._figure_cache[(run, filename)] = (fig, caption)
        self._show_figure(fig)
        self.figures_status.config(text="")

    def _on_canvas_scroll(self, event) -> None:
        if self._current_figure is None or self._current_figure_canvas is None:
            return
        # Tk event coordinates are top-left origin; matplotlib figure pixel coordinates are
        # bottom-left origin -- flip y before hit-testing/zooming against the figure's axes.
        widget_height = self._current_figure_canvas.get_tk_widget().winfo_height()
        x_px, y_px = event.x, widget_height - event.y

        target = _select_zoom_target(self._current_figure.axes, x_px, y_px)
        if target is None:
            return
        _zoom_axes_around_point(target, x_px, y_px, zoom_in=event.delta > 0)
        self._current_figure_canvas.draw_idle()

    def _selected_figure_kinds(self) -> Optional[str]:
        selected_groups = [name for name, var in self.figure_group_vars.items() if var.get()]
        if len(selected_groups) == len(FIGURE_GROUPS):
            return None  # everything selected -- let the CLI use its own default
        kinds: List[str] = []
        for name in selected_groups:
            kinds.extend(FIGURE_GROUPS[name])
        return ",".join(kinds)

    def _on_generate_figures_clicked(self) -> None:
        run = self._selected_run()
        if run is None:
            self.figures_status.config(text="No run selected.")
            return
        if not any(var.get() for var in self.figure_group_vars.values()):
            self.figures_status.config(text="Select at least one figure kind.")
            return

        figures = self._selected_figure_kinds()
        self.generate_button.config(state=tk.DISABLED)
        self.figures_status.config(text="Generating...")
        threading.Thread(
            target=self._generate_figures_in_background, args=(run, figures), daemon=True
        ).start()
        self.root.after(100, self._poll_figures_generation)

    def _generate_figures_in_background(self, run: Path, figures: Optional[str]) -> None:
        proc = subprocess.run(
            build_report_command(run, figures=figures), capture_output=True, text=True
        )
        self.figures_queue.put((proc.returncode, proc.stdout, proc.stderr))

    def _poll_figures_generation(self) -> None:
        try:
            exit_code, stdout, stderr = self.figures_queue.get_nowait()
        except queue.Empty:
            self.root.after(100, self._poll_figures_generation)
            return

        self.generate_button.config(state=tk.NORMAL)
        if exit_code == 0:
            self.figures_status.config(text="Figures generated.")
        else:
            error_text = (stderr or stdout).strip()
            last_line = error_text.splitlines()[-1] if error_text else "unknown error"
            self.figures_status.config(text=f"Generation failed: {last_line}")
        self._refresh_figure_list()

    # ---- shared ----

    def _on_close(self) -> None:
        if self.process is not None:
            self.process.terminate()
        for fig, _caption in self._figure_cache.values():
            plt.close(fig)
        self.root.destroy()


def _set_windows_dpi_awareness() -> None:
    """Must run before tk.Tk() exists. Without this, Windows treats the process as DPI-unaware
    and bitmap-stretches the whole window to match the display's scale factor -- that stretch is
    exactly what makes a tkinter window look blurry on a high-DPI screen. Telling Windows we
    handle our own scaling stops it from doing that; _apply_dpi_scaling below does the "our own
    scaling" part so the window comes out sharp *and* a normal physical size, not sharp-but-tiny.
    """
    if sys.platform != "win32":
        return
    import ctypes

    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)  # PROCESS_SYSTEM_DPI_AWARE
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass


def _apply_dpi_scaling(root: tk.Tk) -> None:
    """Must run after tk.Tk() exists (needs a window to query real display metrics from)."""
    try:
        dpi = root.winfo_fpixels("1i")
        root.tk.call("tk", "scaling", dpi / 72.0)
    except tk.TclError:
        pass


def _cap_window_to_screen(root: tk.Tk) -> None:
    """Tk's own geometry manager doesn't clamp a window's requested size to the physical
    screen on its own -- combined with `_apply_dpi_scaling` above (needed so widgets aren't
    sharp-but-tiny on a >100% Windows display-scaling setting), the *sum* of every child
    widget's now-larger natural size can add up to more than the screen itself, and maximizing
    (or full-screening) then asks for a window bigger than the display -- the embedded chart
    included, since it's sized to whatever its container ends up getting. Setting an explicit
    maximum caps every subsequent resize (including maximize) to the screen's own bounds,
    regardless of what the DPI-scaled child widgets would otherwise add up to.
    """
    try:
        root.maxsize(root.winfo_screenwidth(), root.winfo_screenheight())
    except tk.TclError:
        pass


def launch() -> Optional[int]:
    _set_windows_dpi_awareness()
    root = tk.Tk()
    _apply_dpi_scaling(root)
    _cap_window_to_screen(root)
    app = PickerApp(root)
    root.mainloop()
    return app.last_exit_code


if __name__ == "__main__":
    sys.exit(launch() or 0)
