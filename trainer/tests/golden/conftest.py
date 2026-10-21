from __future__ import annotations

import json
import os

import pytest

GOLDEN = os.path.join(os.path.dirname(__file__), "..", "..", "contract", "golden")
TRACES = os.path.join(os.path.dirname(__file__), "..", "..", "contract", "traces")


def load(name: str) -> dict:
    path = os.path.join(GOLDEN, name)
    if not os.path.exists(path):
        # A missing golden file is a failure, not a skip: these tests are what keep the copies honest.
        pytest.fail(f"{path} is missing: run 'dotnet run --project Teleop.Eval -- gen-trainer-golden' from core/")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    assert data["version"] == 1
    return data
