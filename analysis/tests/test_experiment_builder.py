from __future__ import annotations

import sys
from pathlib import Path

# experiment_builder.py lives at analysis/ (one level above tests/), not inside the
# teleop_analysis package -- add it to sys.path explicitly, matching test_run_tests_ui.py.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiment_builder import (  # noqa: E402
    axis_points,
    build_experiment_yaml,
    combined_points,
    delay_points,
    jitter_points,
    loss_points,
)


def test_axis_points_is_the_public_min_max_step_generator_used_by_the_gui():
    # jitter_points/delay_points/loss_points/the GUI's combined-impairments controls all build
    # on this -- covered directly since it's public API now, not just an internal helper.
    assert axis_points(0, 10, 5) == [0, 5, 10]


def test_jitter_points_covers_the_full_range_inclusive():
    points = jitter_points(0, 60, 1)
    assert len(points) == 61
    assert points[0] == "jitter-0ms"
    assert points[-1] == "jitter-60ms"


def test_delay_points_covers_the_full_range_inclusive():
    points = delay_points(0, 300, 1)
    assert len(points) == 301
    assert points[0] == "delay-0ms"
    assert points[-1] == "delay-300ms"


def test_loss_points_formats_fractional_steps_without_float_noise():
    points = loss_points(0, 5, 0.1)
    assert len(points) == 51
    assert points[0] == "loss-0pct"
    assert points[3] == "loss-0.3pct"
    assert points[-1] == "loss-5pct"
    # No float-accumulation artifacts like "loss-0.30000000000000004pct".
    assert all(len(p) < 20 for p in points)


def test_points_stop_before_overshooting_when_step_does_not_evenly_divide_range():
    points = jitter_points(0, 10, 3)
    assert points == ["jitter-0ms", "jitter-3ms", "jitter-6ms", "jitter-9ms"]


def test_points_raises_on_nonpositive_step():
    import pytest

    with pytest.raises(ValueError):
        jitter_points(0, 10, 0)


def test_points_raises_when_max_below_min():
    import pytest

    with pytest.raises(ValueError):
        jitter_points(10, 0, 1)


def test_combined_points_marches_two_axes_forward_together_not_cross_product():
    # Lockstep, not a cross product: point i is (delay[i], jitter[i]), so 2 values per axis
    # produces 2 profiles, not 4.
    points = combined_points(delay_ms=[100, 150], jitter_ms=[20, 30])
    assert points == ["combo__delay-100ms__jitter-20ms", "combo__delay-150ms__jitter-30ms"]


def test_combined_points_supports_all_three_axes_in_canonical_order():
    points = combined_points(delay_ms=[0, 150], jitter_ms=[0, 20], loss_pct=[0, 0.5])
    assert points == [
        "combo__delay-0ms__jitter-0ms__loss-0pct",
        "combo__delay-150ms__jitter-20ms__loss-0.5pct",
    ]


def test_combined_points_omits_empty_axes_entirely():
    points = combined_points(jitter_ms=[30, 40], loss_pct=[1, 2])
    assert points == ["combo__jitter-30ms__loss-1pct", "combo__jitter-40ms__loss-2pct"]


def test_combined_points_raises_when_axis_lengths_differ():
    import pytest

    with pytest.raises(ValueError):
        combined_points(delay_ms=[0, 100, 200], jitter_ms=[5, 10])


def test_combined_points_raises_with_fewer_than_two_axes():
    import pytest

    with pytest.raises(ValueError):
        combined_points(jitter_ms=[10, 20])


def test_combined_points_raises_with_no_axes():
    import pytest

    with pytest.raises(ValueError):
        combined_points()


def test_build_experiment_yaml_shape():
    yaml_text = build_experiment_yaml(
        experiment_id="exp-gui-sweep",
        predictors=["none", "double-exp"],
        seeds=[1, 2, 3],
        profiles=["jitter-0ms", "jitter-5ms", "loss-1pct"],
    )
    assert "id: exp-gui-sweep" in yaml_text
    assert "seeds: [1, 2, 3]" in yaml_text
    assert "  - none" in yaml_text
    assert "  - double-exp" in yaml_text
    assert "reconciler: snap" in yaml_text
    assert "  - jitter-0ms" in yaml_text
    assert "  - loss-1pct" in yaml_text
    assert "trialSteps: 500" in yaml_text
    assert "stepIntervalTicks: 100000" in yaml_text


def test_build_experiment_yaml_requires_at_least_one_predictor():
    import pytest

    with pytest.raises(ValueError):
        build_experiment_yaml("exp-x", predictors=[], seeds=[1], profiles=["jitter-0ms"])


def test_build_experiment_yaml_requires_at_least_one_profile():
    import pytest

    with pytest.raises(ValueError):
        build_experiment_yaml("exp-x", predictors=["none"], seeds=[1], profiles=[])
