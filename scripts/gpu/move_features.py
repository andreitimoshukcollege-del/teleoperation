#!/usr/bin/env python3
"""How the mixed script's moves move, by kind (docs/adr/0038, "M, motion"): can the path tell a reach at an object from a free move?

    python3 scripts/gpu/move_features.py <features.csv> [--folds 5] [--seed 1]

Reads the CSV that ``joint-drive --script mixed --dry-run --layouts a-b --features <csv>`` writes (one row a planned move,
``core/Teleop.Eval/JointDrive/MoveFeatures.cs`` says what each column is) and prints:

- each feature's distribution by kind (n, p10 / p50 / p90 and the mean);
- how well a classifier tells aims (and passes, and both) from free moves: each feature's own AUC (whichever way round is larger),
  a decision stump's and a logistic regression's, both cross-validated over ``--folds`` folds of whole takes (a take's moves in one
  fold), their out-of-fold scores pooled into one AUC. The required features first, then without ``pullback_after`` (a move into a
  fixture always pulls back, whatever its kind: what that rule costs), then with the joint-space extras (roll, pitch, gripper, speed
  peaks, chord, any reverse after);
- the same for the take's main part only (``--main``: moves within [1.5 s, end - 2.5 s], where aims happen; the quiet start and the
  tail plan none, so there a move's time alone says it is free).
- the accidents (docs/adr/0038 "M, motion", when the CSV has the ``accident`` and ``touch`` columns): their count by kind, and how well
  "the move is an accident" predicts "the move makes a contact by the model" (each contact given to the move last begun): the AUC of
  that one yes/no, and of a logistic regression on the path features with and without it. Aims against free moves are also scored
  without the moves an accident came in (``accident`` set), which move as accidents do and come more often on aims.

numpy only (the GPU machine: ``~/isaacsim/lerobot-venv/bin/python``). An AUC of 0.5 is chance.
"""

from __future__ import annotations

import argparse
import csv
import sys

import numpy as np

REQUIRED = ["duration_s", "path_m", "peak_speed", "mean_speed", "peak_accel", "end_ratio", "stop_s", "pause_after", "pullback_after", "low_mm"]
EXTRA = ["chord_m", "peaks", "roll_range_deg", "pitch_range_deg", "grip_range", "reverse_after"]
KINDS = ["aim", "pass", "free", "reverse", "pause"]
ACCIDENTS = ["jerk", "overshoot", "wrong-way", "slip", "rush"]


def read(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path}: no rows")
    return rows


def column(rows, name):
    out = []
    for r in rows:
        try:
            out.append(float(r[name]))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return np.array(out, dtype=float)


def design(rows, names):
    """The features as a matrix, a NaN ``stop_s`` (the speed never fell to a fifth of its peak within the move and a second after)
    set past the largest seen with an indicator beside it, any other NaN 0."""
    cols, used = [], []
    for name in names:
        x = column(rows, name)
        if name == "stop_s":
            nan = np.isnan(x)
            cap = (np.nanmax(x) if (~nan).any() else 1.0) + 0.1
            cols.append(np.where(nan, cap, x))
            used.append(name)
            cols.append(nan.astype(float))
            used.append("stop_never")
            continue
        cols.append(np.nan_to_num(x, nan=0.0))
        used.append(name)
    return np.column_stack(cols), used


def auc(scores, labels):
    """The Mann-Whitney AUC of ``scores`` for ``labels`` (1 positive), ties counted half."""
    scores = np.asarray(scores, dtype=float)
    labels = np.asarray(labels, dtype=int)
    pos, neg = labels.sum(), len(labels) - labels.sum()
    if pos == 0 or neg == 0:
        return float("nan")
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores))
    s = scores[order]
    i = 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and s[j + 1] == s[i]:
            j += 1
        ranks[order[i:j + 1]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return float((ranks[labels == 1].sum() - pos * (pos + 1) / 2.0) / (pos * neg))


def folds_by_take(takes, k, seed):
    unique = np.array(sorted(set(takes)))
    rng = np.random.default_rng(seed)
    rng.shuffle(unique)
    fold_of = {t: i % k for i, t in enumerate(unique)}
    return np.array([fold_of[t] for t in takes])


def gini(p):
    return 2.0 * p * (1.0 - p)


def stump_fit(x, y):
    """The one split of one feature that lowers the Gini impurity most: (feature, threshold, rate at or below it, rate above it)."""
    n = len(y)
    best = (0, np.inf, y.mean(), y.mean())
    best_impurity = gini(y.mean())
    for j in range(x.shape[1]):
        v = x[:, j]
        cuts = np.unique(np.quantile(v, np.linspace(0.01, 0.99, 99)))
        for c in cuts:
            left = v <= c
            nl = left.sum()
            if nl == 0 or nl == n:
                continue
            pl, pr = y[left].mean(), y[~left].mean()
            impurity = (nl * gini(pl) + (n - nl) * gini(pr)) / n
            if impurity < best_impurity - 1e-12:
                best_impurity = impurity
                best = (j, c, pl, pr)
    return best


def logistic_fit(x, y, lam=1e-2, iterations=50):
    """L2-regularised logistic regression by Newton's method on standardised features: (weights, mean, sd)."""
    mean, sd = x.mean(axis=0), x.std(axis=0)
    sd[sd == 0] = 1.0
    z = np.column_stack([np.ones(len(x)), (x - mean) / sd])
    w = np.zeros(z.shape[1])
    reg = lam * np.eye(z.shape[1])
    reg[0, 0] = 0.0
    for _ in range(iterations):
        p = 1.0 / (1.0 + np.exp(-np.clip(z @ w, -30, 30)))
        g = z.T @ (p - y) + reg @ w
        h = (z * (p * (1 - p))[:, None]).T @ z + reg
        step = np.linalg.solve(h + 1e-9 * np.eye(len(w)), g)
        w -= step
        if np.abs(step).max() < 1e-8:
            break
    return w, mean, sd


def logistic_score(model, x):
    w, mean, sd = model
    z = np.column_stack([np.ones(len(x)), (x - mean) / sd])
    return z @ w


def main_part(rows, start=1.5, tail=2.5):
    """The moves of each take's main part: from ``start`` s to ``tail`` s before the take's last move ends."""
    ends = {}
    for r in rows:
        key = (r["layout"], r["seed"])
        ends[key] = max(ends.get(key, 0.0), float(r["end_s"]))
    return [r for r in rows if float(r["start_s"]) >= start and float(r["end_s"]) <= ends[(r["layout"], r["seed"])] - tail]


def separability(rows, positive, negative, names, k, seed):
    chosen = [r for r in rows if r["kind"] in positive + negative]
    y = np.array([1 if r["kind"] in positive else 0 for r in chosen])
    if y.sum() == 0 or y.sum() == len(y):
        return None
    x, used = design(chosen, names)
    takes = [r["layout"] + "/" + r["seed"] for r in chosen]
    fold = folds_by_take(takes, k, seed)
    stump_scores = np.zeros(len(y))
    lr_scores = np.zeros(len(y))
    stump_picks = []
    for f in range(k):
        train, test = fold != f, fold == f
        if y[train].sum() == 0 or y[train].sum() == train.sum() or test.sum() == 0:
            continue
        j, c, pl, pr = stump_fit(x[train], y[train])
        stump_picks.append(used[j])
        stump_scores[test] = np.where(x[test, j] <= c, pl, pr)
        lr_scores[test] = logistic_score(logistic_fit(x[train], y[train]), x[test])
    per = []
    for j, name in enumerate(used):
        a = auc(x[:, j], y)
        per.append((name, max(a, 1 - a), "+" if a >= 0.5 else "-"))
    full = logistic_fit(x, y)
    weights = sorted(zip(used, full[0][1:]), key=lambda t: -abs(t[1]))
    return {"n": len(y), "positives": int(y.sum()), "stump": auc(stump_scores, y), "logistic": auc(lr_scores, y), "per": per,
            "stumpPicks": stump_picks, "weights": weights}


def accidents(rows, k, seed):
    """The accidents' count by kind and their contacts, and how well "an accident" predicts "a contact" (docs/adr/0038 "M, motion")."""
    touch = np.array([1 if (r.get("touch") or "0") not in ("", "0") else 0 for r in rows])
    is_acc = np.array([1 if r["kind"] in ACCIDENTS else 0 for r in rows])
    print()
    n_acc = int(is_acc.sum())
    print(f"Accidents: {n_acc} of {len(rows)} records; " + ", ".join(
        f"{a} {sum(1 for r in rows if r['kind'] == a)} ({sum(1 for r, t in zip(rows, touch) if r['kind'] == a and t)} with a contact)" for a in ACCIDENTS))
    onsets = np.array([float(r.get("touch") or 0) for r in rows])
    print(f"  contacts by the model: {int(onsets.sum())} onsets, {int(onsets[is_acc == 1].sum())} under an accident "
          f"({100.0 * onsets[is_acc == 1].sum() / max(1.0, onsets.sum()):.0f}%); records with a contact {int(touch.sum())}, accidents among them {int((touch & is_acc).sum())}")
    if 0 < touch.sum() < len(touch):
        print(f"  'the move is an accident' for 'the move makes a contact': AUC {auc(is_acc, touch):.3f}")
        x, used = design(rows, REQUIRED)
        takes = [r["layout"] + "/" + r["seed"] for r in rows]
        fold = folds_by_take(takes, k, seed)
        for label, xx in (("path features", x), ("path features and the accident flag", np.column_stack([x, is_acc]))):
            scores = np.zeros(len(touch))
            for f in range(k):
                train, test = fold != f, fold == f
                if touch[train].sum() == 0 or test.sum() == 0:
                    continue
                scores[test] = logistic_score(logistic_fit(xx[train], touch[train]), xx[test])
            print(f"  a contact from the {label}: logistic AUC {auc(scores, touch):.3f} ({k}-fold by take)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv")
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--main", action="store_true", help="only the moves of each take's main part (1.5 s from the start to 2.5 s before the end)")
    args = ap.parse_args(argv)
    rows = read(args.csv)
    if args.main:
        rows = main_part(rows)
    takes = {(r["layout"], r["seed"]) for r in rows}
    counts = {kind: sum(1 for r in rows if r["kind"] == kind) for kind in KINDS}
    print(f"{len(rows)} moves of {len(takes)} takes: " + ", ".join(f"{kind} {counts[kind]}" for kind in KINDS))
    print()
    print("By kind: p10 / p50 / p90 (mean)")
    for name in REQUIRED + EXTRA:
        x = column(rows, name)
        parts = []
        for kind in ["aim", "pass", "free", "reverse"]:
            v = x[[r["kind"] == kind for r in rows]]
            v = v[~np.isnan(v)]
            if len(v) == 0:
                parts.append(f"{kind} -")
                continue
            p10, p50, p90 = np.quantile(v, [0.1, 0.5, 0.9])
            parts.append(f"{kind} {p10:.3g} / {p50:.3g} / {p90:.3g} ({v.mean():.3g})")
        nan = int(np.isnan(x).sum())
        print(f"  {name:16s} " + "; ".join(parts) + (f"; NaN in {nan}" if nan else ""))
    parts = sorted({r.get("part", "") for r in rows if r["kind"] == "free"} - {""})
    if parts:
        print()
        print("Free moves by part: n, and p50 of duration_s, path_m, peak_speed, low_mm")
        for part in parts:
            sel = [r for r in rows if r["kind"] == "free" and r.get("part") == part]
            med = [np.nanmedian(column(sel, name)) for name in ("duration_s", "path_m", "peak_speed", "low_mm")]
            print(f"  {part:16s} {len(sel):5d}  " + "  ".join(f"{m:.3g}" for m in med))
        forced = sum(1 for r in rows if r.get("part") == "forced-pull-back")
        drawn = sum(1 for r in rows if r.get("part") == "pull-back")
        print(f"  pull-backs: {drawn} drawn, {forced} more forced out of fixtures")
    if rows and "accident" in rows[0]:
        accidents(rows, args.folds, args.seed)
        clean = [r for r in rows if not r.get("accident")]
        res = separability(clean, ["aim"], ["free"], REQUIRED, args.folds, args.seed)
        if res is not None:
            print()
            print(f"aim against free without the moves an accident came in, required features ({res['positives']} of {res['n']} moves): "
                  f"stump AUC {res['stump']:.3f}, logistic AUC {res['logistic']:.3f} ({args.folds}-fold by take)")
    for label, positive in (("aim", ["aim"]), ("pass", ["pass"]), ("aim+pass", ["aim", "pass"])):
        for set_name, names in (("required", REQUIRED), ("required but pullback_after", [n for n in REQUIRED if n != "pullback_after"]),
                                ("required+extra", REQUIRED + EXTRA)):
            res = separability(rows, positive, ["free"], names, args.folds, args.seed)
            if res is None:
                continue
            print()
            print(f"{label} against free, {set_name} features ({res['positives']} of {res['n']} moves): "
                  f"stump AUC {res['stump']:.3f} (picked {', '.join(sorted(set(res['stumpPicks'])))}), "
                  f"logistic AUC {res['logistic']:.3f} ({args.folds}-fold by take)")
            print("  each feature's AUC: " + ", ".join(f"{n} {a:.3f}{s}" for n, a, s in sorted(res["per"], key=lambda t: -t[1])))
            print("  logistic weights (standardised, all moves): " + ", ".join(f"{n} {w:+.2f}" for n, w in res["weights"][:8]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
