"""Tests for reading Core's catalog of selectable algorithms and impairments.

Everything here exercises `parse`, not `load`: parsing is the part with logic, and it can be tested
on a machine with no .NET toolchain. There is one opt-in test at the bottom that runs the real verb,
skipped by default, because the whole point of this module is that the Python side has no list of
its own to check against a fixture -- Core is the source of truth, and asserting a hardcoded
expected set here would recreate the drift the module exists to remove.
"""

from __future__ import annotations

import os
import shutil

import pytest

from teleop_analysis import catalog

SAMPLE = """{
  "predictors": ["const-vel", "double-exp", "none"],
  "reconcilers": ["snap", "spring"],
  "playoutPolicies": ["fixed", "immediate"],
  "codecs": ["raw"],
  "transports": ["loopback"],
  "arbiters": [],
  "namedProfiles": ["lan", "50ms-5j"],
  "traceProfiles": ["synthetic-burst"],
  "isolatedAxes": [
    { "name": "delay", "unitSuffix": "ms" },
    { "name": "loss", "unitSuffix": "pct" }
  ]
}
"""


def test_parse_reads_every_axis():
    c = catalog.parse(SAMPLE)

    assert c.predictors == ["const-vel", "double-exp", "none"]
    assert c.reconcilers == ["snap", "spring"]
    assert c.playout_policies == ["fixed", "immediate"]
    assert c.named_profiles == ["lan", "50ms-5j"]
    assert c.trace_profiles == ["synthetic-burst"]
    assert [a.name for a in c.isolated_axes] == ["delay", "loss"]


def test_isolated_axis_builds_the_profile_name_core_would_resolve():
    c = catalog.parse(SAMPLE)
    delay, loss = c.isolated_axes

    assert delay.profile_name("100") == "delay-100ms"
    assert loss.profile_name("0.5") == "loss-0.5pct"


def test_algorithm_axes_offers_only_the_axes_a_sweep_can_vary():
    """Codecs and transports are real registry tables but `sweep` does not vary them, so offering
    them in the GUI would be a control that silently does nothing."""
    axes = catalog.parse(SAMPLE).algorithm_axes

    assert set(axes) == {"predictor", "reconciler", "playout"}
    assert axes["reconciler"] == ["snap", "spring"]


def test_algorithm_axes_omits_an_empty_table():
    """`arbiters` is declared but unpopulated; an empty checkbox row is worse than none."""
    assert "arbiter" not in catalog.parse(SAMPLE).algorithm_axes


def test_parse_tolerates_a_build_banner_before_the_json():
    """`dotnet run` can print MSBuild noise on a cold build, which would otherwise be fatal."""
    noisy = "Determining projects to restore...\nRestored Teleop.Core.\n" + SAMPLE

    assert catalog.parse(noisy).predictors == ["const-vel", "double-exp", "none"]


def test_parse_rejects_output_with_no_json():
    with pytest.raises(catalog.CatalogUnavailable, match="no JSON object"):
        catalog.parse("error MSB1009: Project file does not exist.")


def test_parse_rejects_malformed_json():
    with pytest.raises(catalog.CatalogUnavailable, match="unparseable"):
        catalog.parse('{ "predictors": [ }')


def test_parse_rejects_an_empty_predictor_list():
    """`none` is always registered, so an empty list means the output is not what we think it is.

    Failing loudly matters more here than it looks: the alternative is a GUI that opens with no
    algorithms and no explanation, which is indistinguishable from "this project has no predictors".
    """
    with pytest.raises(catalog.CatalogUnavailable, match="no predictors"):
        catalog.parse('{ "predictors": [] }')


@pytest.mark.skipif(
    shutil.which("dotnet") is None or not os.environ.get("TELEOP_TEST_DOTNET"),
    reason="opt-in: set TELEOP_TEST_DOTNET=1 with a .NET SDK available to run the real verb",
)
def test_load_against_the_real_catalog_verb():
    """End-to-end against `Teleop.Eval catalog`.

    Asserts only invariants that cannot drift -- that it runs, and that `none`/`snap` (the
    zero-mitigation baselines every experiment config in the repo uses) are present. Asserting the
    full expected set would be a second hardcoded list, which is the exact thing this module exists
    to eliminate.
    """
    c = catalog.load()

    assert "none" in c.predictors
    assert "snap" in c.reconcilers
    assert {a.name for a in c.isolated_axes} >= {"delay", "jitter", "loss"}
