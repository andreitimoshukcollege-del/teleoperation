"""The evaluation report (markdown) and the trainer gate (ADR 0021 decisions 9 and 14).

The gate's thresholds are the ADR's, fixed before training. Inside the trainer they decide whether
the repo's C# consumer gets built; they are not a result. A simulated run's report says so first.
G1-G6 are measured on the model with no network forecast (that is G0); F1-F3 on the forecast sweep.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

NON_LEARNED = ("relay-replay", "relay-replay-blind", "relay-replay-fitted", "relay-replay-adaptive", "relay-replay-servo")
MODEL = "seq-model"
FORECAST_LEVELS = ("oracle", "good", "fair", "poor", "persistence", "rtt-half")
UPLINK_WINDOW = "uplink_change_window_deg"
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

    # G1: on 150ms-20j-0.5loss AND on the live-impaired sessions: p95 lower by >= 15% and >= 0.25 deg,
    # the CI excluding 0. Either half unmeasured fails G1.
    halves, parts = [], []
    key = next((s for s in sched if s.startswith("150ms-20j-0.5loss") and ";" not in s), None)
    if key and MODEL in sched[key]:
        best = _best(sched[key], "joint_prediction_error_deg", "p95")
        mine = sched[key][MODEL]["joint_prediction_error_deg"]["p95"]
        ci = summary.get("bootstrap", {}).get(key, {}).get(best[0], {}) if best else {}
        halves.append(best is not None and mine <= 0.85 * best[1] and mine <= best[1] - 0.25 and ci.get("hi", 1.0) < 0)
        parts.append(f"{key}: seq-model p95 {_f(mine)} vs {best[0] if best else '?'} {_f(best[1] if best else None)}, "
                     f"CI [{_f(ci.get('lo'))}, {_f(ci.get('hi'))}]")
    else:
        halves.append(None)
        parts.append("no 150ms-20j-0.5loss views")
    live = summary.get("live_impaired", {})
    if MODEL in live:
        best = _best(live, "joint_prediction_error_deg", "p95")
        mine = live[MODEL]["joint_prediction_error_deg"]["p95"]
        ci = summary.get("bootstrap_live", {}).get(best[0], {}) if best else {}
        halves.append(best is not None and mine <= 0.85 * best[1] and mine <= best[1] - 0.25 and ci.get("hi", 1.0) < 0)
        parts.append(f"live-impaired sessions: seq-model p95 {_f(mine)} vs {best[0] if best else '?'} "
                     f"{_f(best[1] if best else None)}, CI [{_f(ci.get('lo'))}, {_f(ci.get('hi'))}]")
    else:
        halves.append(None)
        parts.append("no live-impaired sessions")
    gate("G1", None if any(h is None for h in halves) else all(halves), "; ".join(parts))

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


def decision9_go(gates: List[dict]) -> bool:
    return all(g["pass"] for g in gates if g["gate"] in ("G2", "G3", "G5", "G6")) and \
        any(g["pass"] for g in gates if g["gate"] in ("G1", "G4"))


def forecast_gate(summary: dict, card: Optional[dict] = None) -> List[dict]:
    """G0 and F1-F3 (ADR 0021 decision 14), on the forecast sweep. Unmeasured fails."""
    out = []

    def gate(name: str, ok: Optional[bool], detail: str):
        out.append({"gate": name, "pass": bool(ok), "measured": ok is not None, "detail": detail})

    base = trainer_gate(summary, card)
    gate("G0", decision9_go(base), "decision 9's GO, measured on the model with no network forecast: "
         + ", ".join(f"{g['gate']} {'pass' if g['pass'] else 'FAIL'}" for g in base))
    overall = summary.get("overall", {})

    def window(name: str) -> float:
        return overall.get(name, {}).get(UPLINK_WINDOW, {}).get("p95", float("nan"))

    absent, oracle = window(MODEL), window(f"{MODEL}@oracle")
    if not (math.isnan(absent) or math.isnan(oracle)):
        gate("F1", oracle <= 0.75 * absent, f"after uplink changes, [t_e, t_info + 1 s): p95 {_f(oracle)} with an oracle "
                                            f"forecast vs {_f(absent)} with none (needs <= 0.75x)")
    else:
        gate("F1", None, "no uplink-change windows, or no oracle sweep")

    poor = f"{MODEL}@poor"
    rows, ok2, measured = [], True, False
    for scope, table in (("schedule", summary.get("by_schedule", {})), ("condition", summary.get("by_condition", {}))):
        for key, stats in table.items():
            if poor in stats and MODEL in stats:
                measured = True
                a = stats[poor]["joint_prediction_error_deg"]["p95"]
                b = stats[MODEL]["joint_prediction_error_deg"]["p95"]
                if not (math.isnan(a) or math.isnan(b)):
                    ok2 = ok2 and a <= b + 0.1
                    if a > b + 0.1:
                        rows.append(f"{scope} {key}: {_f(a)} vs {_f(b)}")
    if measured and poor in overall:
        m, z = overall[poor], overall[MODEL]
        ok2 = ok2 and m["joint_correction_deg"]["p95"] <= z["joint_correction_deg"]["p95"] + 0.1 \
            and m["joint_correction_deg"]["max"] <= 1.5 * z["joint_correction_deg"]["max"] \
            and m["joint_display_jitter_deg"]["p95"] <= z["joint_display_jitter_deg"]["p95"]
        gate("F2", ok2, ("worse than no forecast by more than 0.1 deg at p95: " + "; ".join(rows)) if rows else
             f"no condition worse than 0.1 deg at p95; correction p95 {_f(m['joint_correction_deg']['p95'])} vs "
             f"{_f(z['joint_correction_deg']['p95'])}, jitter p95 {_f(m['joint_display_jitter_deg']['p95'], 3)} vs "
             f"{_f(z['joint_display_jitter_deg']['p95'], 3)}")
    else:
        gate("F2", None, "no poor-forecast sweep")

    good = f"{MODEL}@good"
    baselines = [n for n in overall if n.endswith("@good") and not n.startswith(MODEL)]
    if good in overall and baselines:
        best = min(baselines, key=lambda n: window(n) if not math.isnan(window(n)) else float("inf"))
        mine, theirs = window(good), window(best)
        ci = summary.get("bootstrap_forecast", {}).get(best, {})
        ok3 = not math.isnan(mine) and not math.isnan(theirs) and mine <= 0.85 * theirs and mine <= theirs - 0.25 \
            and ci.get("hi", 1.0) < 0
        gate("F3", ok3, f"after uplink changes, a good forecast: seq-model p95 {_f(mine)} vs {best} {_f(theirs)}; "
                        f"CI of the difference [{_f(ci.get('lo'))}, {_f(ci.get('hi'))}]")
    else:
        gate("F3", None, "no good-forecast sweep")
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
    base = {n: m for n, m in overall.items() if "@" not in n}  # the forecast sweep has its own table
    lines += ["## All test views", "", "| Predictor | error p50 | p95 | p99 | max | correction p95 | max | jitter p95 |",
              "|---|---|---|---|---|---|---|---|"]
    for name, m in base.items():
        e, c, j = m["joint_prediction_error_deg"], m["joint_correction_deg"], m["joint_display_jitter_deg"]
        lines.append(f"| {name} | {_f(e['p50'])} | {_f(e['p95'])} | {_f(e['p99'])} | {_f(e['max'])} | {_f(c['p95'])} | "
                     f"{_f(c['max'])} | {_f(j['p95'], 3)} |")
    lines += ["", "Degrees, at driver read instants (exact truth). Correction: the jump when a reading arrives. "
              "Jitter: |p_k - 2p_k-1 + p_k-2| per 90 Hz frame.", "", "## By schedule (error p95, degrees)", ""]
    names = list(base)
    cell = lambda stats, n: _f(stats[n]["joint_prediction_error_deg"]["p95"]) if n in stats else "—"  # noqa: E731
    lines += ["| Schedule | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for sched, stats in summary.get("by_schedule", {}).items():
        lines.append(f"| `{sched}` | " + " | ".join(cell(stats, n) for n in names) + " |")
    lines += ["", "## By condition (error p95, degrees)", "", "| Condition | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for cond, stats in summary.get("by_condition", {}).items():
        lines.append(f"| {cond} | " + " | ".join(cell(stats, n) for n in names) + " |")
    for kind in ("network", "hand-on", "slider", "uplink-change"):
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
    lines += _lead_section(overall)
    lines += _forecast_section(overall)
    gate = trainer_gate(summary, card)
    lines += ["", "## Trainer gate (ADR 0021 decision 9)" + (" — on SIMULATED sessions" if simulated else ""), "",
              "Measured on the model with no network forecast, the headset as it is today.", "",
              "| Gate | Pass | Detail |", "|---|---|---|"]
    for g in gate:
        lines.append(f"| {g['gate']} | {'PASS' if g['pass'] else ('FAIL' if g['measured'] else 'NOT MEASURED')} | {g['detail']} |")
    go = decision9_go(gate)
    lines += ["", f"**{'GO' if go else 'NO-GO'}** = G2, G3, G5 and G6 all pass, and G1 or G4 passes." +
              (" On simulated sessions this only says the pipeline can learn what the simulator contains." if simulated else "")]
    fgate = forecast_gate(summary, card)
    lines += ["", "## Network forecast gate (ADR 0021 decision 14)" + (" — on SIMULATED sessions" if simulated else ""), "",
              "| Gate | Pass | Detail |", "|---|---|---|"]
    for g in fgate:
        lines.append(f"| {g['gate']} | {'PASS' if g['pass'] else ('FAIL' if g['measured'] else 'NOT MEASURED')} | {g['detail']} |")
    by = {g["gate"]: g for g in fgate}
    worth = by["F1"]["pass"]
    fgo = worth and all(by[k]["pass"] for k in ("G0", "F2", "F3"))
    lines += ["", f"**{'GO' if fgo else 'NO-GO'}** for the model with forecasts = F1, G0, F2 and F3 all pass. "
              + ("" if worth or not by["F1"]["measured"] else
                 "F1 failed: even a perfect forecast does not help this model after uplink changes, so a network model "
                 "is not worth building for it. ")]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def _lead_section(overall: dict) -> List[str]:
    rows = [(n, m["joint_prediction_lead_ms"]) for n, m in overall.items() if "joint_prediction_lead_ms" in m and "@" not in n]
    if not rows:
        return []
    out = ["", "## How far ahead of the arm (`joint_prediction_lead_ms`, display ticks in motion)", "",
           "| Predictor | p5 | p50 | p95 | at the ±300 ms edge | segments |", "|---|---|---|---|---|---|"]
    for n, v in rows:
        out.append(f"| {n} | {_f(v['p5'], 0)} | {_f(v['p50'], 0)} | {_f(v['p95'], 0)} | "
                   f"{_f(v['edge'] * 100 if v['n'] else float('nan'), 0)}% | {v['n']} |")
    out += ["", "Positive: the prediction shows where the arm will be (it runs ahead); negative: where it was."]
    return out


def _forecast_section(overall: dict) -> List[str]:
    levels = [lv for lv in FORECAST_LEVELS if f"{MODEL}@{lv}" in overall]
    if not levels or MODEL not in overall:
        return []
    out = ["", "## By network forecast quality (ADR 0021 decision 12)", "",
           "The same views and queries at every level; `absent` is the headset today. Uplink windows: "
           "[t_e, t_info + 1 s) after each uplink change of at least 50 ms.", "",
           "| Forecast | seq-model p95 | after uplink changes p95 | correction p95 | jitter p95 | relay-replay fed it, after uplink changes p95 |",
           "|---|---|---|---|---|---|"]
    for lv in ("absent",) + tuple(levels):
        name = MODEL if lv == "absent" else f"{MODEL}@{lv}"
        m = overall[name]
        fc = overall.get(f"relay-replay-fc@{lv}", overall.get("relay-replay") if lv == "absent" else None)
        out.append(f"| {lv} | {_f(m['joint_prediction_error_deg']['p95'])} | {_f(m[UPLINK_WINDOW]['p95'])} | "
                   f"{_f(m['joint_correction_deg']['p95'])} | {_f(m['joint_display_jitter_deg']['p95'], 3)} | "
                   f"{_f(fc[UPLINK_WINDOW]['p95']) if fc else '—'} |")
    return out


def _bin_order(b: str) -> float:
    if b == "information-limited":
        return -0.5
    try:
        return float(b.split("..")[0])
    except ValueError:
        return 1e9
