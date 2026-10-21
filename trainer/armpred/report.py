"""The evaluation report (markdown) and the trainer gate (ADR 0021 decision 9).

The gate's thresholds are the ADR's, fixed before training. Inside the trainer they decide whether
the repo's C# consumer gets built; they are not a result. A simulated run's report says so first.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

NON_LEARNED = ("relay-replay", "relay-replay-blind", "relay-replay-fitted", "relay-replay-adaptive")
MODEL = "seq-model"
HELD_OUT_MARKERS = ("slider=5.0", "150g")
# Network schedules ADR 0021 holds out of training entirely.
HELD_OUT_SCHEDULES = ("300ms-60j-2loss-bursty@0", "synthetic-burst@0")


def _f(x: float, digits: int = 2) -> str:
    return "—" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{digits}f}"


def _best(stats: Dict[str, dict], metric: str, field: str) -> Optional[tuple]:
    candidates = [(name, stats[name][metric][field]) for name in NON_LEARNED if name in stats
                  and not math.isnan(stats[name][metric][field])]
    return min(candidates, key=lambda c: c[1]) if candidates else None


def trainer_gate(summary: dict, card: Optional[dict] = None) -> List[dict]:
    """G1-G6 as ADR 0021 fixes them, measured on this summary. A gate whose data is missing FAILS:
    an unmeasured gate must never read as passed (root CLAUDE.md invariant 10)."""
    out = []
    sched = summary.get("by_schedule", {})

    def gate(name: str, ok: Optional[bool], detail: str):
        out.append({"gate": name, "pass": bool(ok), "measured": ok is not None, "detail": detail})

    # G1: 150ms-20j-0.5loss: p95 lower by >= 15% and >= 0.25 deg, CI excluding 0.
    key = next((s for s in sched if s.startswith("150ms-20j-0.5loss") and ";" not in s), None)
    if key and MODEL in sched[key]:
        best = _best(sched[key], "joint_prediction_error_deg", "p95")
        mine = sched[key][MODEL]["joint_prediction_error_deg"]["p95"]
        ci = summary.get("bootstrap", {}).get(key, {}).get(best[0], {}) if best else {}
        ok = best is not None and mine <= 0.85 * best[1] and mine <= best[1] - 0.25 and ci.get("hi", 1.0) < 0
        gate("G1", ok, f"{key}: seq-model p95 {_f(mine)} vs {best[0] if best else '?'} {_f(best[1] if best else None)}; "
                       f"CI of the difference [{_f(ci.get('lo'))}, {_f(ci.get('hi'))}]")
    else:
        gate("G1", None, "no 150ms-20j-0.5loss views")

    # G2: lan and 50ms-5j: p95 no more than 0.1 deg worse, p99 no more than 0.25 deg worse.
    parts, ok2 = [], True
    for name in ("lan", "50ms-5j"):
        k = next((s for s in sched if s.startswith(name + "@") and ";" not in s), None)
        if not k or MODEL not in sched[k]:
            ok2 = None
            break
        b95 = _best(sched[k], "joint_prediction_error_deg", "p95")
        b99 = _best(sched[k], "joint_prediction_error_deg", "p99")
        m = sched[k][MODEL]["joint_prediction_error_deg"]
        ok2 = ok2 and m["p95"] <= b95[1] + 0.1 and m["p99"] <= b99[1] + 0.25
        parts.append(f"{name}: p95 {_f(m['p95'])} vs {_f(b95[1])}, p99 {_f(m['p99'])} vs {_f(b99[1])}")
    gate("G2", ok2, "; ".join(parts) if parts else "no lan / 50ms-5j views")

    overall = summary.get("overall", {})
    if MODEL in overall:
        m = overall[MODEL]
        bc95 = _best(overall, "joint_correction_deg", "p95")
        bcmax = _best(overall, "joint_correction_deg", "max")
        bj95 = _best(overall, "joint_display_jitter_deg", "p95")
        ok3 = (m["joint_correction_deg"]["p95"] <= bc95[1] + 0.1 and m["joint_correction_deg"]["max"] <= 1.5 * bcmax[1]
               and m["joint_display_jitter_deg"]["p95"] <= bj95[1])
        gate("G3", ok3, f"correction p95 {_f(m['joint_correction_deg']['p95'])} vs {_f(bc95[1])}, max "
                        f"{_f(m['joint_correction_deg']['max'])} vs 1.5 x {_f(bcmax[1])}; jitter p95 "
                        f"{_f(m['joint_display_jitter_deg']['p95'], 3)} vs {_f(bj95[1], 3)}")
        hand = m["joint_event_error_deg"].get("hand-on", {}).get("0..100 ms")
        window = _window_p95(overall, "hand-on")
        rec = m["joint_recovery_time_ms"].get("hand-on", {})
        if window and not math.isnan(window[0]):
            ok4 = window[0] <= 0.75 * window[1] and rec.get("p50", float("inf")) <= 500.0
            gate("G4", ok4, f"hand-on, first second after t_info: p95 {_f(window[0])} vs {_f(window[1])}; "
                            f"median recovery {_f(rec.get('p50'), 0)} ms")
        else:
            gate("G4", None, "no resistance-onset events")
    else:
        gate("G3", None, "no seq-model results")
        gate("G4", None, "no seq-model results")

    # G5: every held-out condition within 0.25 deg at p95 of the best non-learned; fallback < 1%.
    held = {k: v for k, v in summary.get("by_condition", {}).items() if any(h in k for h in HELD_OUT_MARKERS)}
    held.update({f"schedule {k}": v for k, v in sched.items() if k in HELD_OUT_SCHEDULES})
    fallback = summary.get("joint_predictor_fallback", {}).get(MODEL, {}).get("rate", float("nan"))
    if held and MODEL in next(iter(held.values())):
        rows, ok5 = [], True
        for k, v in held.items():
            b = _best(v, "joint_prediction_error_deg", "p95")
            mine = v[MODEL]["joint_prediction_error_deg"]["p95"]
            ok5 = ok5 and mine <= b[1] + 0.25
            rows.append(f"{k}: {_f(mine)} vs {_f(b[1])}")
        ok5 = ok5 and fallback < 0.01
        gate("G5", ok5, "; ".join(rows) + f"; fallback rate {_f(fallback * 100 if not math.isnan(fallback) else float('nan'), 2)}%")
    else:
        gate("G5", None, "no held-out conditions in the test set")

    inf = summary.get("joint_inference_us", {}).get(MODEL, {})
    parity = (card or {}).get("parity", {}).get("maxAbsRad")
    if inf and parity is not None:
        ok6 = inf.get("p99", float("inf")) <= 50.0 and parity <= 1e-5 and summary.get("deterministic", False)
        gate("G6", ok6, f"inference p99 {_f(inf.get('p99'), 1)} us (Python onnxruntime, one thread); parity {parity:.2e} rad; "
                        f"replays identical: {summary.get('deterministic')}")
    else:
        gate("G6", None, "no inference timing or parity")
    return out


def _window_p95(overall: dict, kind: str) -> Optional[tuple]:
    """p95 over the first second after t_info (0..1000 ms), seq-model vs the best non-learned."""

    def p95_first_second(name: str) -> float:
        bins = overall[name]["joint_event_error_deg"].get(kind, {})
        vals = [bins[b]["p95"] for b in ("0..100 ms", "100..250 ms", "250..500 ms", "500..1000 ms") if b in bins]
        vals = [v for v in vals if not math.isnan(v)]
        return max(vals) if vals else float("nan")

    if MODEL not in overall:
        return None
    best = [p95_first_second(n) for n in NON_LEARNED if n in overall]
    best = [b for b in best if not math.isnan(b)]
    return (p95_first_second(MODEL), min(best)) if best else None


def write_report(summary: dict, path: str, simulated: bool, card: Optional[dict] = None) -> None:
    lines = []
    if simulated:
        lines += ["# SIMULATION CHECK — not the robot's numbers", "",
                  "Every session here came from `armpred.sim`. These numbers check that the pipeline and the "
                  "adaptation logic work; they are not measurements of the JetRover (ADR 0021 decision 6).", ""]
    else:
        lines += ["# Trainer evaluation on recorded sessions", "",
                  "This decides whether the repo's C# consumer is built (ADR 0021 decision 8). It is not a result: "
                  "a citable number comes only from Teleop.Eval scoring the exported model.", ""]
    overall = summary.get("overall", {})
    lines += ["## All test views", "", "| Predictor | error p50 | p95 | p99 | max | correction p95 | max | jitter p95 |",
              "|---|---|---|---|---|---|---|---|"]
    for name, m in overall.items():
        e, c, j = m["joint_prediction_error_deg"], m["joint_correction_deg"], m["joint_display_jitter_deg"]
        lines.append(f"| {name} | {_f(e['p50'])} | {_f(e['p95'])} | {_f(e['p99'])} | {_f(e['max'])} | {_f(c['p95'])} | "
                     f"{_f(c['max'])} | {_f(j['p95'], 3)} |")
    lines += ["", "Degrees, at driver read instants (exact truth). Correction: the jump when a reading arrives. "
              "Jitter: |p_k - 2p_k-1 + p_k-2| per 90 Hz frame.", "", "## By schedule (error p95, degrees)", ""]
    names = list(overall)
    lines += ["| Schedule | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for sched, stats in summary.get("by_schedule", {}).items():
        lines.append(f"| `{sched}` | " + " | ".join(_f(stats[n]["joint_prediction_error_deg"]["p95"]) for n in names) + " |")
    lines += ["", "## By condition (error p95, degrees)", "", "| Condition | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for cond, stats in summary.get("by_condition", {}).items():
        lines.append(f"| {cond} | " + " | ".join(_f(stats[n]["joint_prediction_error_deg"]["p95"]) for n in names) + " |")
    for kind in ("network", "hand-on", "slider"):
        lines += ["", f"## Around `{kind}` events (error p95 by window, degrees)", ""]
        bins = sorted({b for n in names for b in overall[n]["joint_event_error_deg"].get(kind, {})}, key=_bin_order)
        if not bins:
            lines.append("No such events.")
            continue
        lines += ["| Window | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
        for b in bins:
            lines.append(f"| {b} | " + " | ".join(_f(overall[n]["joint_event_error_deg"][kind].get(b, {}).get("p95", float('nan'))) for n in names) + " |")
        lines.append("")
        lines.append("Recovery (ms, p50 / p95): " + "; ".join(
            f"{n} {_f(overall[n]['joint_recovery_time_ms'].get(kind, {}).get('p50', float('nan')), 0)} / "
            f"{_f(overall[n]['joint_recovery_time_ms'].get(kind, {}).get('p95', float('nan')), 0)}" for n in names))
    fb = summary.get("joint_predictor_fallback", {})
    inf = summary.get("joint_inference_us", {})
    if fb or inf:
        lines += ["", "## seq-model", ""]
        for n, v in fb.items():
            lines.append(f"- fallback: {v['fallbacks']} of {v['calls']} calls ({_f(v['rate'] * 100, 2)}%)")
        for n, v in inf.items():
            lines.append(f"- inference (Python onnxruntime, one thread): p50 {_f(v['p50'], 1)} us, p99 {_f(v['p99'], 1)} us")
    gate = trainer_gate(summary, card)
    lines += ["", "## Trainer gate (ADR 0021 decision 9)" + (" — on SIMULATED sessions" if simulated else ""), "",
              "| Gate | Pass | Detail |", "|---|---|---|"]
    for g in gate:
        lines.append(f"| {g['gate']} | {'PASS' if g['pass'] else ('FAIL' if g['measured'] else 'NOT MEASURED')} | {g['detail']} |")
    go = all(g["pass"] for g in gate if g["gate"] in ("G2", "G3", "G5", "G6")) and any(g["pass"] for g in gate if g["gate"] in ("G1", "G4"))
    lines += ["", f"**{'GO' if go else 'NO-GO'}** = G2, G3, G5 and G6 all pass, and G1 or G4 passes." +
              (" On simulated sessions this only says the pipeline can learn what the simulator contains." if simulated else "")]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def _bin_order(b: str) -> float:
    if b == "information-limited":
        return -0.5
    try:
        return float(b.split("..")[0])
    except ValueError:
        return 1e9
