"""Tkinter GUI for analysis/: run tests and view generated figures.

Launched by run_tests.py when invoked without --all. Needs a real display -- this project's
analysis/ venv is a Windows-native Python reached via WSL interop (see analysis/CLAUDE.md), so
this opens as a normal Windows window with no extra display-server setup.
"""
from __future__ import annotations

import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from typing import Dict, List, Optional

from run_tests import group_by_file, humanize_test_name

_SUMMARY_RE = re.compile(r"^=+\s*(.*?)\s*=+$")
RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


def build_pytest_command(node_ids: List[str]) -> List[str]:
    """The exact subprocess argv used to run a set of selected tests."""
    return [sys.executable, "-m", "pytest", *node_ids, "-v"]


def find_summary_line(lines: List[str]) -> Optional[str]:
    """Pull pytest's final '==== N passed/failed ... ====' line out of captured output, for
    display only -- pass/fail itself is decided by the subprocess exit code, never by parsing
    this text, so a summary line this doesn't recognize just means no summary text is shown.
    """
    for line in reversed(lines):
        match = _SUMMARY_RE.match(line.strip())
        if match and match.group(1):
            return match.group(1)
    return None


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


class PickerApp:
    def __init__(self, root: tk.Tk, node_ids: List[str]):
        self.root = root
        self.root.title("analysis/ toolkit")
        self.node_ids = node_ids
        self.vars: Dict[str, tk.BooleanVar] = {}
        self.process: Optional[subprocess.Popen] = None
        self.last_exit_code: Optional[int] = None
        self.output_queue: "queue.Queue" = queue.Queue()

        self.run_display_to_path: Dict[str, Path] = {}
        self.figures_queue: "queue.Queue" = queue.Queue()
        self._current_photo: Optional[tk.PhotoImage] = None

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True)
        notebook.add(self._build_tests_tab(notebook), text="Tests")
        notebook.add(self._build_figures_tab(notebook), text="Figures")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- Tests tab ----

    def _build_tests_tab(self, notebook: ttk.Notebook) -> ttk.Frame:
        container = ttk.Frame(notebook, padding=8)

        list_frame = ttk.Frame(container)
        list_frame.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(list_frame, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        self.checklist_frame = ttk.Frame(canvas)

        self.checklist_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self.checklist_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for file_path, ids_in_file in group_by_file(self.node_ids).items():
            display_file = file_path.split("/")[-1]
            ttk.Label(
                self.checklist_frame, text=display_file, font=("TkDefaultFont", 10, "bold")
            ).pack(anchor="w", pady=(6, 0))
            for node_id in ids_in_file:
                test_name = node_id.split("::", 1)[1]
                var = tk.BooleanVar(value=True)
                self.vars[node_id] = var
                ttk.Checkbutton(
                    self.checklist_frame,
                    text=humanize_test_name(test_name),
                    variable=var,
                    command=self._update_status,
                ).pack(anchor="w", padx=(16, 0))

        button_row = ttk.Frame(container)
        button_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(button_row, text="Select All", command=self._select_all).pack(side=tk.LEFT)
        ttk.Button(button_row, text="Select None", command=self._select_none).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        self.run_button = ttk.Button(button_row, text="Run Selected", command=self._on_run_clicked)
        self.run_button.pack(side=tk.LEFT, padx=(6, 0))
        self.status_label = ttk.Label(button_row, text="")
        self.status_label.pack(side=tk.RIGHT)

        self.output = ScrolledText(container, height=16, font=("Consolas", 10), state=tk.DISABLED)
        self.output.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.output.tag_config("pass", foreground="#1a7f37")
        self.output.tag_config("fail", foreground="#cf222e")

        self._update_status()
        return container

    def _selected_ids(self) -> List[str]:
        return [node_id for node_id, var in self.vars.items() if var.get()]

    def _update_status(self) -> None:
        self.status_label.config(text=f"{len(self._selected_ids())} of {len(self.vars)} selected")

    def _select_all(self) -> None:
        for var in self.vars.values():
            var.set(True)
        self._update_status()

    def _select_none(self) -> None:
        for var in self.vars.values():
            var.set(False)
        self._update_status()

    def _append_output(self, text: str, tag: Optional[str] = None) -> None:
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text, tag or ())
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _on_run_clicked(self) -> None:
        selected = self._selected_ids()
        if not selected:
            self._append_output("Nothing selected.\n")
            return

        self.run_button.config(state=tk.DISABLED)
        self.output.configure(state=tk.NORMAL)
        self.output.delete("1.0", tk.END)
        self.output.configure(state=tk.DISABLED)
        self._append_output(f"Running {len(selected)} test(s)...\n\n")

        threading.Thread(target=self._run_in_background, args=(selected,), daemon=True).start()
        self.root.after(50, self._poll_output)

    def _run_in_background(self, selected: List[str]) -> None:
        self.process = subprocess.Popen(
            build_pytest_command(selected),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        lines: List[str] = []
        assert self.process.stdout is not None
        for line in self.process.stdout:
            lines.append(line)
            self.output_queue.put(line)
        exit_code = self.process.wait()
        self.process = None
        self.output_queue.put(("__done__", exit_code, find_summary_line(lines)))

    def _poll_output(self) -> None:
        try:
            while True:
                item = self.output_queue.get_nowait()
                if isinstance(item, tuple):
                    _, exit_code, summary = item
                    self.last_exit_code = exit_code
                    tag = "pass" if exit_code == 0 else "fail"
                    label = "PASSED" if exit_code == 0 else "FAILED"
                    self._append_output(f"\n{label}", tag)
                    self._append_output(f" -- {summary}\n" if summary else "\n")
                    self.run_button.config(state=tk.NORMAL)
                    return
                self._append_output(item)
        except queue.Empty:
            pass
        if str(self.run_button["state"]) == "disabled":
            self.root.after(50, self._poll_output)

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


def launch(node_ids: List[str]) -> Optional[int]:
    if not node_ids:
        print("No tests collected -- check that analysis/tests/ exists and pytest can import it.")
        return 1

    _set_windows_dpi_awareness()
    root = tk.Tk()
    _apply_dpi_scaling(root)
    app = PickerApp(root, node_ids)
    root.mainloop()
    return app.last_exit_code


if __name__ == "__main__":
    from run_tests import collect_test_ids

    sys.exit(launch(collect_test_ids()) or 0)
