"""Tkinter GUI for analysis/: configure and run a sweep, then view its figures.

Launched by run_tests.py when invoked without --all. Needs a real display -- this project's
analysis/ venv is a Windows-native Python reached via WSL interop (see analysis/CLAUDE.md), so
this opens as a normal Windows window with no extra display-server setup.
"""
from __future__ import annotations

import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from typing import Dict, List, Optional, Tuple

import experiment_builder

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


def build_report_command(run_dir: Path) -> List[str]:
    """The exact subprocess argv used to (re)generate a run's figures -- same CLI `just report`
    wraps. Absolute path so it doesn't matter what the subprocess's cwd ends up being.
    """
    return [sys.executable, "-m", "teleop_analysis.cli", str(run_dir.resolve())]


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

        self.run_display_to_path: Dict[str, Path] = {}
        self.figures_queue: "queue.Queue" = queue.Queue()
        self._current_photo: Optional[tk.PhotoImage] = None

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
        self.run_combo = ttk.Combobox(top_row, state="readonly", width=55)
        self.run_combo.pack(side=tk.LEFT, padx=(6, 6))
        self.run_combo.bind("<<ComboboxSelected>>", lambda e: self._refresh_figure_list())
        self.generate_button = ttk.Button(
            top_row, text="Generate / Refresh Figures", command=self._on_generate_figures_clicked
        )
        self.generate_button.pack(side=tk.LEFT)
        self.figures_status = ttk.Label(top_row, text="")
        self.figures_status.pack(side=tk.RIGHT)

        body = ttk.Frame(container)
        body.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.figure_listbox = tk.Listbox(body, width=32, exportselection=False)
        self.figure_listbox.pack(side=tk.LEFT, fill=tk.Y)
        self.figure_listbox.bind("<<ListboxSelect>>", lambda e: self._on_figure_selected())

        image_frame = ttk.Frame(body)
        image_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))
        image_frame.grid_rowconfigure(0, weight=1)
        image_frame.grid_columnconfigure(0, weight=1)

        self.image_canvas = tk.Canvas(image_frame, background="white")
        v_scroll = ttk.Scrollbar(image_frame, orient="vertical", command=self.image_canvas.yview)
        h_scroll = ttk.Scrollbar(image_frame, orient="horizontal", command=self.image_canvas.xview)
        self.image_canvas.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)
        self.image_canvas.grid(row=0, column=0, sticky="nsew")
        v_scroll.grid(row=0, column=1, sticky="ns")
        h_scroll.grid(row=1, column=0, sticky="ew")

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

    def _clear_image(self) -> None:
        self.image_canvas.delete("all")
        self._current_photo = None

    def _on_figure_selected(self) -> None:
        selection = self.figure_listbox.curselection()
        run = self._selected_run()
        if not selection or run is None:
            return
        path = run / "figures" / self.figure_listbox.get(selection[0])
        photo = tk.PhotoImage(file=str(path))
        self._current_photo = photo  # tkinter drops the image if nothing keeps a reference
        self.image_canvas.delete("all")
        self.image_canvas.create_image(0, 0, anchor="nw", image=photo)
        self.image_canvas.configure(scrollregion=(0, 0, photo.width(), photo.height()))

    def _on_generate_figures_clicked(self) -> None:
        run = self._selected_run()
        if run is None:
            self.figures_status.config(text="No run selected.")
            return

        self.generate_button.config(state=tk.DISABLED)
        self.figures_status.config(text="Generating...")
        threading.Thread(
            target=self._generate_figures_in_background, args=(run,), daemon=True
        ).start()
        self.root.after(100, self._poll_figures_generation)

    def _generate_figures_in_background(self, run: Path) -> None:
        proc = subprocess.run(build_report_command(run), capture_output=True, text=True)
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


def launch() -> Optional[int]:
    _set_windows_dpi_awareness()
    root = tk.Tk()
    _apply_dpi_scaling(root)
    app = PickerApp(root)
    root.mainloop()
    return app.last_exit_code


if __name__ == "__main__":
    sys.exit(launch() or 0)
