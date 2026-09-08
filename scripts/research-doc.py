#!/usr/bin/env python3
"""Create and append to the human-facing research documentation Word file.

The .docx at docs/Teleop-Research-Documentation.docx is a **human-owned deliverable** -- it is
meant to be edited by hand in Word, and this script must never overwrite those edits. `init`
therefore refuses to run if the file already exists, and `append-solution` opens the existing
document and adds to the end of it.

It exists so an agent does not have to understand OOXML to record a decision. The research
organizer calls `append-solution` once per completed run; see .claude/agents/research-organizer.md.

Requires python-docx, which lives in analysis/.venv (see analysis/requirements-dev.txt for why a
document tool is recorded there rather than in a venv of its own). Run it with that interpreter:

    ./analysis/.venv/bin/python scripts/research-doc.py --help
"""

import argparse
import datetime as _dt
import pathlib
import sys

try:
    import docx
    from docx.shared import Pt
except ImportError:
    sys.exit(
        "python-docx is not installed. This script expects analysis/.venv:\n"
        "    ./analysis/.venv/bin/python scripts/research-doc.py ...\n"
        "or, to set that venv up: just analysis-setup"
    )

REPO = pathlib.Path(__file__).resolve().parent.parent
DOC = REPO / "docs" / "Teleop-Research-Documentation.docx"


def _heading(d, text, level):
    d.add_heading(text, level=level)


def _para(d, text, italic=False, bold=False):
    p = d.add_paragraph()
    run = p.add_run(text)
    run.italic = italic
    run.bold = bold
    return p


def _bullets(d, items):
    for item in items:
        d.add_paragraph(item, style="List Bullet")


def _table(d, headers, rows):
    t = d.add_table(rows=1, cols=len(headers))
    t.style = "Light Grid Accent 1"
    for i, h in enumerate(headers):
        cell = t.rows[0].cells[i]
        cell.text = ""
        run = cell.paragraphs[0].add_run(h)
        run.bold = True
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = str(v)
    return t


def cmd_init(args):
    if DOC.exists() and not args.force:
        sys.exit(
            f"{DOC.relative_to(REPO)} already exists. This file is edited by hand, so refusing "
            f"to overwrite it. Pass --force only if you are certain there is nothing to lose."
        )

    d = docx.Document()
    style = d.styles["Normal"]
    style.font.name = "Calibri"
    style.font.size = Pt(11)

    _heading(d, "Teleop Research Platform", 0)
    _para(
        d,
        "VR teleoperation of a remote robot (Meta Quest + Unity), built to produce measured, "
        "reproducible results about latency mitigation.",
        italic=True,
    )
    _para(
        d,
        f"Generated {_dt.date.today().isoformat()} from the state of the repository. This document "
        "is maintained by hand from here on; descriptions are deliberately brief and are meant to "
        "be expanded. Sections added automatically by the research organizer are marked as such.",
        italic=True,
    )

    # --- What it is -------------------------------------------------------
    _heading(d, "1. What this program is", 1)
    _para(
        d,
        "A research platform, not a product. The deliverable is measured, reproducible results "
        "about how to hide network latency between a VR operator and a remote robot: prediction, "
        "reconciliation, jitter buffering, autonomy arbitration.",
    )
    _para(
        d,
        "The rule that shapes every design decision: an algorithm that cannot be evaluated "
        "headlessly does not count. Anything that can only be checked by putting on a headset is "
        "either in the wrong place or built the wrong way.",
    )
    _bullets(
        d,
        [
            "Teleop.Core - every algorithm. Depends on nothing; no Unity, no clock reads, no I/O, "
            "no allocation on the per-frame path. Compiled by both .NET and Unity from one copy.",
            "Teleop.Eval - the headless CLI that runs experiments and writes results.",
            "Teleop.RobotHost / Teleop.RobotArm - the real robot side, running on a Hiwonder "
            "JetRover's Jetson Nano.",
            "unity/TeleopVR - the operator side: scenes, XR rig, rendering. A thin adapter layer; "
            "it hosts Core and never participates in an algorithm.",
            "analysis/ - Python; reads results and produces figures and percentile tables.",
        ],
    )

    # --- How a result is produced ----------------------------------------
    _heading(d, "2. How a result is produced", 1)
    _para(
        d,
        "An experiment is one YAML file naming the algorithms, the network profiles and the seeds. "
        "A sweep runs that matrix through the same loopback pipeline the unit tests exercise, and "
        "writes raw metrics plus a manifest recording the exact configuration and the git commit.",
    )
    _bullets(
        d,
        [
            "Percentiles (p50/p95/p99), never means - the tail is what an operator notices.",
            "Multiple seeds always; a winner is never declared from one.",
            "The baseline is always reported alongside, however bad it looks.",
            "Prediction error and correction cost are always reported together - a predictor that "
            "wins on accuracy while causing constant visual corrections is a worse system.",
            "A number is citable only if it has a manifest and a commit reachable from main.",
        ],
    )

    # --- Approaches -------------------------------------------------------
    _heading(d, "3. Approaches tried, by research axis", 1)

    _heading(d, "3.1 Reconciliation - how the view gets from prediction to truth", 2)
    _para(
        d,
        "The axis that decides whether the system is usable in VR: a hard visual snap when truth "
        "arrives is nausea, however good the prediction was. Four implementations, compared with "
        "the predictor held fixed.",
    )
    _table(
        d,
        ["Approach", "Idea", "Outcome"],
        [
            ["snap", "Jump straight to truth on the next frame.",
             "Baseline. Deliberately the worst case; every other approach is measured against it."],
            ["spring", "Critically damped decay of a residual offset; no overshoot.",
             "Works. Large jerk reduction over snap; converges in ~180-270 ms."],
            ["budget-blend", "Quintic blend that reaches truth exactly at a fixed deadline.",
             "Works. Lower jerk than spring; costs more convergence time."],
            ["velocity-match", "Slows the correction in proportion to the operator's apparent motion.",
             "Works. Lowest jerk on the axis; by far the slowest to converge."],
            ["rollback", "Rewind to truth and re-apply buffered inputs (GGPO-style).",
             "Rejected on argument. Not expressible as a reconciler - see 4.1."],
            ["exp-smooth", "Single time constant lag.",
             "Not built. Cannot satisfy the axis's continuity requirement as written - open question."],
        ],
    )

    _heading(d, "3.2 Prediction - estimating where the robot is now", 2)
    _table(
        d,
        ["Approach", "Idea", "Outcome"],
        [
            ["none", "Return the last observation.", "Baseline."],
            ["const-vel", "First-order dead reckoning.", "Works."],
            ["double-exp", "Two-parameter smoothing of position and orientation.",
             "Works; the strongest baseline predictor, used as the fixed predictor elsewhere."],
            ["const-accel / ekf / seq-model", "Second-order, Kalman, and learned sequence models.",
             "Planned, not built."],
        ],
    )

    _heading(d, "3.3 Transport and network emulation", 2)
    _para(
        d,
        "Impairment is applied by a decorator over any transport, driven by a seeded generator, so "
        "the same seed always reproduces the same network. Five profiles from clean LAN to "
        "300 ms with 60 ms jitter and 2% bursty loss.",
    )
    _bullets(
        d,
        [
            "Implemented: loopback transport, emulated transport (delay, jitter, burst loss, "
            "reordering), and an uncompressed pose codec.",
            "Planned: delta-quantised, trajectory (sending intended future motion rather than "
            "position) and redundant codecs.",
        ],
    )

    _heading(d, "3.4 Buffering and autonomy - not yet started", 2)
    _bullets(
        d,
        [
            "Buffering (when a received sample becomes usable) has contracts and configuration "
            "types but no implementation, and is not yet wired into the pipeline.",
            "Autonomy (how much direct authority the operator keeps as latency rises) is the same, "
            "and is harder: it is a closed-loop question, so it cannot be scored from a recording "
            "and needs a robot in the loop.",
        ],
    )

    _heading(d, "3.5 Real hardware", 2)
    _para(
        d,
        "A Hiwonder JetRover arm, reached over Tailscale, driven through the same Core pipeline as "
        "the simulation so the real robot is not exempt from the latency instrumentation. Inverse "
        "kinematics, gripper and cross-machine clock synchronisation are confirmed working on the "
        "physical robot.",
    )

    # --- Results ----------------------------------------------------------
    _heading(d, "4. Results", 1)
    _heading(d, "4.1 Reconciliation head-to-head", 2)
    _para(
        d,
        "Predictor fixed at double-exp, 5 seeds, 100 ms convergence budget. Jerk is the nausea "
        "proxy; convergence time is what it costs. Lower is better in both columns.",
    )
    _table(
        d,
        ["Approach", "Jerk p99 (150ms/20j)", "Jerk p99 (300ms bursty)", "Convergence p95"],
        [
            ["snap", "1,300 M", "1,593 M", "0 ms (by definition)"],
            ["spring", "206 M", "186 M", "180 / 270 ms"],
            ["budget-blend", "78 M", "82 M", "350 / 800 ms"],
            ["velocity-match", "72 M", "51 M", "1030 / 2180 ms"],
        ],
    )
    _para(
        d,
        "The finding is the shape, not the winner: a clean monotone trade-off with no free lunch. "
        "Every reduction in visual jerk is paid for in how long the correction takes. Which "
        "approach is right depends on whether comfort or responsiveness matters more for the task.",
    )
    _para(
        d,
        "Caveat: these numbers are not citable. They were produced from an untagged commit, and "
        "the results directory is not kept in version control.",
        italic=True,
    )

    _heading(d, "4.2 Approaches rejected, and why", 2)
    _bullets(
        d,
        [
            "rollback - infeasible as a reconciler. It needs an input history and a simulator; the "
            "interface supplies neither, and the only version that fits the contract turns out to "
            "be mathematically identical to snapping. The deeper reason is that the predictor "
            "already performs the rollback, so the reconciler is only ever hiding one that happened "
            "upstream. It belongs elsewhere in the pipeline, behind a design record.",
            "Removing the one-frame display hold - rejected on measurement. All the smoothed "
            "approaches briefly freeze the display when a correction arrives, which is a real "
            "defect. The obvious fix made things worse where corrections are frequent, and "
            "collapsed the measured difference between approaches to nothing. Kept as a documented "
            "trade-off instead, with the better fix written down as an open question.",
            "exp-smooth - not built. A single time constant cannot meet the axis's continuity "
            "requirement as currently written; changing that requirement is a decision for a human.",
        ],
    )

    # --- Open questions ---------------------------------------------------
    _heading(d, "5. Open questions", 1)
    _bullets(
        d,
        [
            "Whether to relax the reconciliation continuity requirement so exp-smooth can be built "
            "and measured as a second documented exception.",
            "A better fix for the one-frame display hold: keep the exact cancellation the current "
            "design gets for free while also preserving motion, by advancing the displayed pose "
            "before re-seeding.",
            "Buffering and autonomy are both untouched; buffering needs pipeline wiring first.",
            "Physical motion-to-photon validation has never been done, so the end-to-end latency "
            "figure is still an estimate rather than a measurement.",
            "Nothing recorded so far is citable: no run has been tagged.",
        ],
    )

    # --- Chosen solutions -------------------------------------------------
    _heading(d, "6. Chosen solutions", 1)
    _para(
        d,
        "One entry per completed research run, recording what was chosen and what was rejected. "
        "Appended automatically by the research organizer; safe to edit by hand afterwards.",
        italic=True,
    )

    DOC.parent.mkdir(parents=True, exist_ok=True)
    d.save(DOC)
    print(f"wrote {DOC.relative_to(REPO)}")


def cmd_append_solution(args):
    if not DOC.exists():
        sys.exit(f"{DOC.relative_to(REPO)} does not exist. Run `init` first.")

    d = docx.Document(str(DOC))
    _heading(d, f"{args.axis}: {args.chosen} ({_dt.date.today().isoformat()})", 2)
    _para(d, "Chosen: ", bold=True).add_run(args.chosen)
    _para(d, "Why: ", bold=True).add_run(args.why)
    if args.rejected:
        _para(d, "Rejected: ", bold=True).add_run(args.rejected)
    if args.results:
        _para(d, "Results: ", bold=True).add_run(args.results)
    if args.log:
        _para(d, "Full reasoning: ", bold=True).add_run(args.log)
    d.save(DOC)
    print(f"appended '{args.chosen}' to {DOC.relative_to(REPO)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_init = sub.add_parser("init", help="create the document (refuses if it already exists)")
    p_init.add_argument("--force", action="store_true", help="overwrite an existing document")
    p_init.set_defaults(func=cmd_init)

    p_add = sub.add_parser("append-solution", help="append one chosen-solution entry")
    p_add.add_argument("--axis", required=True, help="research axis, e.g. Reconciliation")
    p_add.add_argument("--chosen", required=True, help="the approach that was chosen")
    p_add.add_argument("--why", required=True, help="one or two sentences, in plain language")
    p_add.add_argument("--rejected", default="", help="what was rejected and why")
    p_add.add_argument("--results", default="", help="results/ path the numbers came from")
    p_add.add_argument("--log", default="", help="docs/research-log/ path for the full reasoning")
    p_add.set_defaults(func=cmd_append_solution)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
