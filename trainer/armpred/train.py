"""Training: the learned correction on top of the speed-blind prior.

Loss, per labelled joint (ADR 0021 / the methodology review):
- **Huber** (``huber_deg``, in degrees) on ``prior + correction - truth``;
- **Laplace NLL** (``nll_weight``) on the *detached* residual, so the scale head learns how wrong the
  mean is without pulling the mean;
- **consistency** (``consistency``) on pairs asked just before and just after a reading arrives: the
  before-prediction is pulled toward the (detached) after-prediction, which trades error for a
  smaller jump on the display.

The network block of the arm row is zeroed for ``forecast_dropout`` of each batch (ADR 0021 decision
12), on top of the views built with no forecast at all, so the model never depends on one arriving.

Each speed x load cell carries equal weight. The model is selected on validation p95 error, not on
loss, because the tail is what the operator sees.
"""

from __future__ import annotations

import copy
import json
import math
import os
import time
from dataclasses import asdict, dataclass
from typing import Dict, Optional

import numpy as np
import torch
from torch import nn

from .data.build import Dataset
from .features import layout as L
from .models.net import build

DEG = 180.0 / math.pi


@dataclass(frozen=True)
class TrainConfig:
    arch: str = "mlp"
    aware: bool = False
    steps: int = 150_000
    batch: int = 8192
    lr: float = 2e-3
    lr_min: float = 2e-5
    warmup: int = 2_000
    weight_decay: float = 0.01
    clip: float = 1.0
    ema: float = 0.999
    huber_deg: float = 0.3
    nll_weight: float = 0.1
    consistency: float = 0.0
    forecast_dropout: float = 0.1
    pair_batch: int = 1024
    val_every: int = 2_000
    seed: int = 1
    device: str = "auto"
    layout: str = "v3"  # "v4": the FR3's (docs/adr/0028)


def _device(name: str) -> torch.device:
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(name)


def cell_weights(ds: Dataset) -> np.ndarray:
    """Equal weight per (slider, load) cell, where load is none / payload / hand."""
    slider = ds.tags.get("slider", np.zeros(len(ds)))
    load = np.where(ds.tags.get("hand", np.zeros(len(ds))) > 0, 2, np.where(ds.tags.get("payload_g", np.zeros(len(ds))) > 0, 1, 0))
    cell = np.round(slider * 10).astype(np.int64) * 10 + load
    _, inverse, counts = np.unique(cell, return_inverse=True, return_counts=True)
    w = 1.0 / counts[inverse]
    w[~ds.samples.valid] = 0.0
    return w / w.sum()


def _tensors(ds: Dataset, device: torch.device) -> Dict[str, torch.Tensor]:
    s = ds.samples
    as32 = lambda a: torch.from_numpy(np.ascontiguousarray(a, dtype=np.float32))  # noqa: E731  (no copy if already float32)
    return {
        "joint": as32(s.joints).to(device),
        "arm": as32(s.arm).to(device),
        "target": as32(s.truth - s.prior).to(device),  # what the correction must be
    }


def data_device(datasets, device: torch.device, log=print) -> torch.device:
    """Where to keep the samples: on the GPU when they fit in 70% of its free memory (the fast case),
    otherwise in host memory with each batch copied over, so a big dataset runs slower rather than
    failing hours into a run."""
    if device.type != "cuda":
        return device
    need = sum(len(d) * (d.samples.joints.shape[1] + d.samples.arm.shape[1] + 1) * 4 for d in datasets)
    free, total = torch.cuda.mem_get_info(device)
    if need < 0.7 * free:
        log(f"data on the GPU: {need / 2**30:.1f} GiB of {free / 2**30:.1f} GiB free")
        return device
    log(f"data in host memory, batches streamed: {need / 2**30:.1f} GiB would not fit in {free / 2**30:.1f} GiB free on the GPU")
    return torch.device("cpu")


def _pairs(ds: Dataset) -> Optional[np.ndarray]:
    s = ds.samples
    ids = s.pair
    sel = np.nonzero((ids >= 0) & s.valid)[0]
    if sel.size == 0:
        return None
    order = sel[np.lexsort((sel, ids[sel]))]
    pairs = []
    i = 0
    while i + 1 < len(order):
        a, b = order[i], order[i + 1]
        if ids[a] == ids[b]:
            pairs.append((a, b))  # stored before, then after (dataset._queries)
            i += 2
        else:
            i += 1
    return np.array(pairs, dtype=np.int64) if pairs else None


def evaluate_errors(model: nn.Module, t: Dict[str, torch.Tensor], valid: np.ndarray, batch: int = 65536) -> np.ndarray:
    """|prior + correction - truth| in degrees, for every sample (NaN where the predictor falls back)."""
    model.eval()
    out = np.full(t["target"].shape[0], np.nan)
    device = next(model.parameters()).device
    with torch.no_grad():
        for a in range(0, t["target"].shape[0], batch):
            y = model(t["joint"][a:a + batch].to(device), t["arm"][a:a + batch].to(device))
            out[a:a + batch] = (torch.abs(y[:, 0] - t["target"][a:a + batch].to(device)) * DEG).cpu().numpy()
    out[~valid] = np.nan
    return out


def _percentiles(e: np.ndarray) -> Dict[str, float]:
    e = e[np.isfinite(e)]
    if e.size == 0:
        return {"p50": float("nan"), "p95": float("nan"), "p99": float("nan"), "max": float("nan"), "n": 0}
    return {"p50": float(np.percentile(e, 50)), "p95": float(np.percentile(e, 95)),
            "p99": float(np.percentile(e, 99)), "max": float(e.max()), "n": int(e.size)}


def train(train_ds: Dataset, val_ds: Dataset, cfg: TrainConfig, out_dir: str, log=print,
          init_state: Optional[dict] = None) -> Dict[str, object]:
    torch.manual_seed(cfg.seed)
    rng = np.random.default_rng(cfg.seed)
    device = _device(cfg.device)
    model = build(cfg.arch, cfg.aware, cfg.layout).to(device)
    lay = _layout(cfg.layout)
    if init_state is not None:
        model.load_state_dict(init_state)
    ema = copy.deepcopy(model).eval()
    for p in ema.parameters():
        p.requires_grad_(False)
    opt = torch.optim.AdamW(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    store = data_device((train_ds, val_ds), device, log)
    tr = _tensors(train_ds, store)
    va = _tensors(val_ds, store)
    weights = torch.from_numpy(cell_weights(train_ds)).to(store)
    pairs = _pairs(train_ds) if cfg.consistency > 0 else None
    pairs_t = torch.from_numpy(pairs).to(store) if pairs is not None else None
    on = lambda x: x.to(device, non_blocking=True)  # noqa: E731  (a no-op when the data is on the GPU)
    huber = nn.HuberLoss(delta=cfg.huber_deg, reduction="mean")
    os.makedirs(os.path.join(out_dir, "checkpoints"), exist_ok=True)

    # The checkpoint is chosen on the arm joints alone, as the trainer gate scores them; the gripper (the
    # last row, layout v3) is logged beside it, in servo degrees.
    val_arm = lay.is_arm_row()[val_ds.samples.joint]
    val_prior = np.abs(val_ds.samples.prior - val_ds.samples.truth) * DEG
    val_prior[~val_ds.samples.valid | ~val_arm] = np.nan
    val_hold = np.abs(val_ds.samples.newest - val_ds.samples.truth) * DEG
    val_hold[~val_ds.samples.valid | ~val_arm] = np.nan
    history = []
    best = {"p95": float("inf"), "step": -1}
    t0 = time.time()
    for step in range(1, cfg.steps + 1):
        lr = cfg.lr * step / cfg.warmup if step <= cfg.warmup else \
            cfg.lr_min + 0.5 * (cfg.lr - cfg.lr_min) * (1 + math.cos(math.pi * (step - cfg.warmup) / max(1, cfg.steps - cfg.warmup)))
        for g in opt.param_groups:
            g["lr"] = lr
        model.train()
        idx = torch.multinomial(weights, cfg.batch, replacement=True)
        arm = on(tr["arm"][idx])
        if cfg.forecast_dropout > 0.0:
            drop = torch.rand(arm.shape[0], device=device) < cfg.forecast_dropout
            arm = arm.clone()
            arm[drop, lay.BASE_ARM_FEATURES:lay.BASE_ARM_FEATURES + lay.NETWORK_FEATURES] = 0.0
        y = model(on(tr["joint"][idx]), arm)
        residual = y[:, 0] - on(tr["target"][idx])
        loss = huber(residual * DEG, torch.zeros_like(residual))
        b = torch.exp(y[:, 1])
        nll = (torch.abs(residual.detach()) / b + torch.log(2 * b)).mean()
        loss = loss + cfg.nll_weight * nll
        if pairs_t is not None:
            sel = pairs_t[torch.randint(0, pairs_t.shape[0], (cfg.pair_batch,), device=store)]
            before = model(on(tr["joint"][sel[:, 0]]), on(tr["arm"][sel[:, 0]]))[:, 0]
            after = model(on(tr["joint"][sel[:, 1]]), on(tr["arm"][sel[:, 1]]))[:, 0].detach()
            # Both share the label's time; their priors differ, so compare the full predictions.
            prior_gap = on(tr["target"][sel[:, 0]] - tr["target"][sel[:, 1]])  # (truth - prior_b) - (truth - prior_a)
            jump = (before - prior_gap) - after
            loss = loss + cfg.consistency * huber(jump * DEG, torch.zeros_like(jump))
        opt.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), cfg.clip)
        opt.step()
        with torch.no_grad():
            for pe, pm in zip(ema.parameters(), model.parameters()):
                pe.mul_(cfg.ema).add_(pm.detach(), alpha=1.0 - cfg.ema)
        if step % cfg.val_every == 0 or step == cfg.steps:
            e = evaluate_errors(ema, va, val_ds.samples.valid)
            stats = _percentiles(e[val_arm])
            gripper = _percentiles(e[~val_arm])
            row = {"step": step, "loss": float(loss.item()), "lr": lr, "val": stats, "val_gripper": gripper,
                   "seconds": round(time.time() - t0, 1)}
            history.append(row)
            log(f"step {step:7d} loss {row['loss']:.4f} val p50/p95/p99 {stats['p50']:.3f}/{stats['p95']:.3f}/{stats['p99']:.3f} deg "
                f"(prior p95 {np.nanpercentile(val_prior, 95):.3f}, hold-last {np.nanpercentile(val_hold, 95):.3f}); "
                f"gripper p95 {gripper['p95']:.3f} servo deg {row['seconds']}s")
            if stats["p95"] < best["p95"]:
                best = {"p95": stats["p95"], "step": step, "val": stats}
                torch.save(ema.state_dict(), os.path.join(out_dir, "checkpoints", "best.pt"))
    summary = {
        "config": asdict(cfg),
        "best": best,
        "history": history,
        "val_prior": _percentiles(val_prior),
        "val_hold_last": _percentiles(val_hold),
        "train_samples": len(train_ds),
        "val_samples": len(val_ds),
        "device": str(device),
        "torch": torch.__version__,
    }
    with open(os.path.join(out_dir, "train-summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    return summary


def _layout(layout: str):
    from .data.build import layout_module

    return layout_module(layout)


def load_best(out_dir: str, cfg: TrainConfig) -> nn.Module:
    model = build(cfg.arch, cfg.aware, cfg.layout)
    model.load_state_dict(torch.load(os.path.join(out_dir, "checkpoints", "best.pt"), map_location="cpu"))
    return model.eval()
