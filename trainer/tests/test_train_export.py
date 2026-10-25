from __future__ import annotations

import os

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("onnxruntime")

from armpred.contract.profile import JETROVER  # noqa: E402
from armpred.data import schedule as S  # noqa: E402
from armpred.data.build import Dataset  # noqa: E402
from armpred.data.dataset import SampleConfig, view_samples  # noqa: E402
from armpred.data.views import synthesize  # noqa: E402
from armpred.evaluate import default_predictors, evaluate_view  # noqa: E402
from armpred.export import export_onnx, flat_inputs  # noqa: E402
from armpred.features import layout as L  # noqa: E402
from armpred.train import TrainConfig, load_best, train  # noqa: E402

from .conftest import TRACES  # noqa: E402


def _dataset(view, seed):
    s = view_samples(view, np.random.default_rng(seed), SampleConfig(queries_per_s=40.0))
    n = len(s)
    return Dataset(samples=s, view=np.zeros(n, np.int32), tags={"slider": np.full(n, 2.5, np.float32),
                                                                "payload_g": np.zeros(n, np.float32),
                                                                "hand": np.zeros(n, np.float32)})


@pytest.mark.parametrize("arch", ["mlp", "conv"])
def test_train_export_parity_and_evaluate(sim_session, tmp_path, arch):
    view = synthesize(sim_session, JETROVER, S.parse("150ms-20j-0.5loss@0"), seed=3, traces_dir=TRACES)
    tr, va = _dataset(view, 1), _dataset(view, 2)
    cfg = TrainConfig(arch=arch, steps=30, batch=256, warmup=5, val_every=10, device="cpu", consistency=0.25)
    out = str(tmp_path / arch)
    summary = train(tr, va, cfg, out, log=lambda m: None)
    assert np.isfinite(summary["best"]["p95"])
    model = load_best(out, cfg)
    n = len(L.PREDICTED_MOTORS)
    rows = va.samples.joints[: 10 * n].reshape(-1, n, L.JOINT_FEATURES)
    info = export_onnx(model, False, os.path.join(out, "model.onnx"), {"teleop.arch": arch},
                       parity_inputs=flat_inputs(rows, va.samples.arm[: rows.shape[0]]))
    assert info["parity_max_abs"] <= 1e-5
    assert info["input_length"] == 6 * L.JOINT_FEATURES + L.ARM_FEATURES
    # The exported model runs inside the evaluator as the C# predictor will run it.
    result = evaluate_view(view, default_predictors(os.path.join(out, "model.onnx"), view.tps), jitter_s=2.0)
    assert result.errors["seq-model"].shape[0] > 100
    # The gripper is scored on its own (layout v3), and the arm's errors are arm joints only.
    assert result.gripper_errors["seq-model"].shape[0] > 10 and result.gripper_errors["relay-replay"].shape[0] > 10
    assert set(result.errors["seq-model"][:, 1].astype(int)) <= set(range(len(L.ARM_MOTORS)))
    assert result.fallbacks["seq-model"][0] < result.fallbacks["seq-model"][1]
