"""Tkinter GUI for selecting and running analysis/tests/.

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
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from typing import Dict, List, Optional

from run_tests import group_by_file, humanize_test_name

_SUMMARY_RE = re.compile(r"^=+\s*(.*?)\s*=+$")


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


class PickerApp:
    def __init__(self, root: tk.Tk, node_ids: List[str]):
        self.root = root
        self.root.title("analysis/ test picker")
        self.node_ids = node_ids
        self.vars: Dict[str, tk.BooleanVar] = {}
        self.process: Optional[subprocess.Popen] = None
        self.last_exit_code: Optional[int] = None
        self.output_queue: "queue.Queue" = queue.Queue()

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self) -> None:
        container = ttk.Frame(self.root, padding=8)
        container.pack(fill=tk.BOTH, expand=True)

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

    def _on_close(self) -> None:
        if self.process is not None:
            self.process.terminate()
        self.root.destroy()


def launch(node_ids: List[str]) -> Optional[int]:
    if not node_ids:
        print("No tests collected -- check that analysis/tests/ exists and pytest can import it.")
        return 1

    root = tk.Tk()
    app = PickerApp(root, node_ids)
    root.mainloop()
    return app.last_exit_code


if __name__ == "__main__":
    from run_tests import collect_test_ids

    sys.exit(launch(collect_test_ids()) or 0)
