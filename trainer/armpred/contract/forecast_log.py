"""``forecast.jlog`` (contract/FORECAST_FORMAT.md): a network forecast stream on disk.

What a network model's output looks like recorded, so that whoever builds that model has a concrete
target, and its real outputs can be replayed through the same evaluation as the trainer's synthetic
levels. numpy only, like the rest of ``armpred.contract``.
"""

from __future__ import annotations

from typing import List

import numpy as np

from ..net.forecast import HORIZONS_MS, QUANTILES, ForecastStream
from .jlog import JlogError, JlogWriter, format_field, read_jlog, split_items

DIRECTIONS = ("up", "down")


def write_forecast_log(stream: ForecastStream, path: str, session_id: str, ticks_per_second: int) -> None:
    with JlogWriter(path, "forecast", ticks_per_second, "OPERATOR", session_id) as w:
        for i in range(len(stream)):
            for d, name in enumerate(DIRECTIONS):
                vq, vl, vo = bool(stream.valid_q[i, d]), bool(stream.valid_loss[i, d]), bool(stream.valid_outage[i, d])
                flags = ("q" if vq else "-") + ("l" if vl else "-") + ("o" if vo else "-")
                items = []
                for h in range(len(HORIZONS_MS)):
                    q = [format_field(float(x)) for x in stream.q_ms[i, d, h]] if vq else [""] * len(QUANTILES)
                    items.append(":".join(q + [format_field(float(stream.loss[i, d, h])) if vl else ""]))
                w.record("FC", int(stream.t[i]), name, flags, float(stream.floor_ms[i, d]) if vq else None,
                         float(stream.outage[i, d]) if vo else None, ";".join(items))


def read_forecast_log(path: str, level: str = "recorded") -> ForecastStream:
    log = read_jlog(path, "forecast")
    rows = log.tag("FC")
    times = sorted({int(r[0]) for r in rows})
    index = {t: k for k, t in enumerate(times)}
    out = ForecastStream.empty(level, np.array(times, dtype=np.int64))
    for r in rows:
        if len(r) != 6 or r[1] not in DIRECTIONS or len(r[2]) != 3:
            raise JlogError(f"{path}: malformed FC record {r!r}")
        k, d = index[int(r[0])], DIRECTIONS.index(r[1])
        vq, vl, vo = r[2][0] == "q", r[2][1] == "l", r[2][2] == "o"
        out.valid_q[k, d], out.valid_loss[k, d], out.valid_outage[k, d] = vq, vl, vo
        if vq:
            out.floor_ms[k, d] = float(r[3])
        if vo:
            out.outage[k, d] = float(r[4])
        items: List[List[str]] = split_items(r[5])
        if len(items) != len(HORIZONS_MS) or any(len(it) != len(QUANTILES) + 1 for it in items):
            raise JlogError(f"{path}: an FC record needs {len(HORIZONS_MS)} horizons of {len(QUANTILES) + 1} values")
        for h, it in enumerate(items):
            if vq:
                out.q_ms[k, d, h] = [float(x) for x in it[:len(QUANTILES)]]
            if vl:
                out.loss[k, d, h] = float(it[len(QUANTILES)])
    return out
