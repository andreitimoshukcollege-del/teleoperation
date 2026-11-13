"""The forecast golden for the C# copy (armpred/golden_forecast.py, docs/adr/0035 N3): deterministic, and exercising what
it claims to (both directions valid and invalid at times, the stall's outage, a negative-horizon fallback)."""

from __future__ import annotations

import json

from armpred import golden_forecast as G
from armpred.net import forecast as FC


def test_the_golden_is_deterministic_and_covers_its_cases():
    a, b = G.build(), G.build()
    assert json.dumps(a) == json.dumps(b)
    p = a["streams"]["persistence"]
    n = len(a["issues"])
    down = [v[FC.DOWN] for v in p["validQ"]]
    up = [v[FC.UP] for v in p["validQ"]]
    assert 0 < sum(down) < n and 0 < sum(up) < n  # before ClockSync syncs, then after
    assert any(o[FC.DOWN] == 1.0 for o in p["outage"])  # the 450 ms stall
    assert sum(a["streams"]["rtt-half"]["validQ"][i][0] for i in range(n)) > 0
    eff, base = a["commands"]["effPersistence"], a["commands"]["effNoForecast"]
    assert any(e != f for e, f in zip(eff, base)) and all(x <= y for x, y in zip(eff, eff[1:]))
    assert len(a["blocks"]["values"][0]) == 100
