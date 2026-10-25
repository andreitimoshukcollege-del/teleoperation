"""ONNX export (model v1, contract/MODEL_FORMAT.md): a fixed opset, static shapes, the layout id and
provenance in the metadata, an op allowlist, and parity with torch checked on real windows."""

from __future__ import annotations

import hashlib
import json
import os
from typing import Dict, List, Optional

import numpy as np
import torch

from . import MODEL_FORMAT_VERSION, __version__
from .features import layout as L
from .models.net import FlatExport

OPSET = 17
PARITY_TOLERANCE_RAD = 1e-5
# Ops a small on-device runtime can be expected to have; an exported graph outside this set fails
# export, so the Phase 7 backend choice is never blocked by an exotic op (ADR 0021 decision 2).
ALLOWED_OPS = {
    "Add", "Clip", "Concat", "Constant", "Conv", "Div", "Exp", "Expand", "Flatten", "Gather", "Gemm", "Identity",
    "Log", "MatMul", "Max", "Min", "Mul", "ReduceSum", "Relu", "Reshape", "Shape", "Slice", "Softmax", "Squeeze",
    "Sub", "Tanh", "Transpose", "Unsqueeze", "Where", "Cast", "Neg",
}


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def input_length(aware: bool) -> int:
    return len(L.PREDICTED_MOTORS) * L.JOINT_FEATURES + L.arm_features(aware)


def export_onnx(model: torch.nn.Module, aware: bool, path: str, metadata: Dict[str, str],
                parity_inputs: Optional[np.ndarray] = None) -> Dict[str, object]:
    import onnx
    import onnxruntime as ort

    flat = FlatExport(model.eval(), L.arm_features(aware)).eval()
    n_in = input_length(aware)
    example = torch.zeros(1, n_in)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    try:
        torch.onnx.export(flat, (example,), path, input_names=["x"], output_names=["y"], opset_version=OPSET,
                          dynamo=False)
    except TypeError:
        torch.onnx.export(flat, (example,), path, input_names=["x"], output_names=["y"], opset_version=OPSET)
    proto = onnx.load(path)
    onnx.checker.check_model(proto)
    ops = sorted({node.op_type for node in proto.graph.node})
    outside = [op for op in ops if op not in ALLOWED_OPS]
    if outside:
        raise RuntimeError(f"exported graph uses ops outside the allowlist: {outside}")
    meta = {"teleop.layout_id": L.layout_id(aware),
            "teleop.model_format": str(MODEL_FORMAT_VERSION),
            "teleop.trainer_version": __version__,
            "teleop.input_length": str(n_in),
            "teleop.output_length": str(2 * len(L.PREDICTED_MOTORS)), **metadata}
    for key, value in meta.items():
        entry = proto.metadata_props.add()
        entry.key, entry.value = key, value
    onnx.save(proto, path)

    # Parity: the artifact against the torch model, on real windows when given.
    if parity_inputs is None:
        parity_inputs = np.random.default_rng(0).normal(0.0, 1.0, size=(64, n_in)).astype(np.float32)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
    session = ort.InferenceSession(path, options, providers=["CPUExecutionProvider"])
    worst = 0.0
    with torch.no_grad():
        for row in parity_inputs:
            x = row.reshape(1, n_in).astype(np.float32)
            a = session.run(["y"], {"x": x})[0]
            b = flat(torch.from_numpy(x)).numpy()
            worst = max(worst, float(np.abs(a - b).max()))
    if worst > PARITY_TOLERANCE_RAD:
        raise RuntimeError(f"ONNX and torch disagree by {worst:.3g}, above {PARITY_TOLERANCE_RAD}")
    return {"path": path, "sha256": sha256(path), "opset": OPSET, "ops": ops, "parity_max_abs": worst,
            "parity_rows": int(len(parity_inputs)), "input_length": n_in, "metadata": meta}


def flat_inputs(joint_rows: np.ndarray, arm_rows: np.ndarray) -> np.ndarray:
    """[N, 6, J] and [N, A] windows -> [N, 6 * J + A], the order FlatExport reads."""
    return np.concatenate([joint_rows.reshape(joint_rows.shape[0], -1), arm_rows], axis=1).astype(np.float32)
