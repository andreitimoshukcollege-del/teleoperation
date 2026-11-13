"""armpred evaluate scores a model on the reading ages it was trained with (the 2026-11 bug hunt, T-02): without
--prior it used the headset's defaults even for a model trained on fitted ages, and its p50 and p95 moved."""

import json
import os

from armpred.cli import _card_ages


def _card(run, config, ages):
    os.makedirs(run, exist_ok=True)
    with open(os.path.join(run, "model-card.json"), "w", encoding="utf-8") as f:
        json.dump({"prior": {"predictor": "relay-replay", "config": config, "sensedAgeMotorIds": [1, 2, 3, 4, 5], "sensedAgeMs": ages}}, f)


def test_a_model_trained_on_fitted_ages_is_scored_on_them(tmp_path):
    run = str(tmp_path / "run")
    _card(run, "headset+fitted-ages", [52.0, 41.5, 30.0, 18.0, 6.5])
    assert _card_ages(run) == (52.0, 41.5, 30.0, 18.0, 6.5)


def test_a_model_trained_on_the_headsets_defaults_or_with_no_card_keeps_the_defaults(tmp_path):
    run = str(tmp_path / "run")
    _card(run, "headset", [45.0, 35.0, 25.0, 15.0, 5.0])
    assert _card_ages(run) is None
    assert _card_ages(str(tmp_path / "no-card")) is None
