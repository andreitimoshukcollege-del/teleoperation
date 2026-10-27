"""ONNX export (model v1, contract/MODEL_FORMAT.md): a fixed opset, static shapes, the layout id and
provenance in the metadata, an op allowlist, and parity with torch checked on real windows."""

from __future__ import annotations

import hashlib
import json
import math
import os
import struct
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


# -- the weights file (contract/MODEL_FORMAT.md, "Weights"; docs/adr/0025) ----------------------------

WEIGHTS_MAGIC = b"TLMW"
WEIGHTS_VERSION = 1
MLP_LAYERS = ("model.net.0", "model.net.2", "model.net.4", "model.net.6")


def mlp_layers_from_onnx(onnx_path: str):
    """The architecture-A (mlp) graph's four Gemm layers, as [(W [out, in] float32, b [out] float32)]."""
    import onnx
    from onnx import numpy_helper

    proto = onnx.load(onnx_path)
    init = {i.name: numpy_helper.to_array(i) for i in proto.graph.initializer}
    expected = {f"{p}.{k}" for p in MLP_LAYERS for k in ("weight", "bias")}
    if set(init) != expected:
        raise ValueError(f"{onnx_path}: not an mlp export (initializers {sorted(init)})")
    gemms = [n for n in proto.graph.node if n.op_type == "Gemm"]
    for n, prefix in zip(gemms, MLP_LAYERS):
        attrs = {a.name: a for a in n.attribute}
        if list(n.input[1:]) != [f"{prefix}.weight", f"{prefix}.bias"] or attrs["transB"].i != 1:
            raise ValueError(f"{onnx_path}: unexpected Gemm {n.name}")
    return [(init[f"{p}.weight"].astype(np.float32), init[f"{p}.bias"].astype(np.float32)) for p in MLP_LAYERS]


def mlp_forward(layers, x: np.ndarray) -> np.ndarray:
    """FlatExport's arithmetic in NumPy float32: x [N, 1620] -> y [N, 12]. What the C# backend does."""
    n = len(L.PREDICTED_MOTORS)
    rows = []
    for row in x.astype(np.float32):
        joints = row[: n * L.JOINT_FEATURES].reshape(n, L.JOINT_FEATURES)
        arm = row[n * L.JOINT_FEATURES:]
        h = np.concatenate([joints, np.repeat(arm[None, :], n, axis=0)], axis=1)
        for k, (w, b) in enumerate(layers):
            h = h @ w.T + b
            if k < len(layers) - 1:
                h = np.maximum(h, 0.0)
        flag = joints[:, L.JOINT_FEATURES - 3 - 1]  # the identity block's last entry: the gripper
        cap = flag * np.float32(L.GRIPPER_MAX_CORRECTION_RAD - L.MAX_CORRECTION_RAD) + np.float32(L.MAX_CORRECTION_RAD)
        corr = cap * np.tanh(h[:, 0])
        log_b = np.clip(h[:, 1], np.float32(math.log(1e-4)), np.float32(math.log(0.5)))
        rows.append(np.stack([corr, log_b], axis=1).reshape(-1))
    return np.array(rows, dtype=np.float32)


def write_weights(onnx_path: str, out_path: str, model_id: str, check_inputs: np.ndarray) -> Dict[str, object]:
    """Writes ``model.weights`` from the scored ``.onnx``, after checking the NumPy forward pass against
    onnxruntime on ``check_inputs``: real windows (the golden's), held to ``PARITY_TOLERANCE_RAD``."""
    import onnxruntime as ort

    layers = mlp_layers_from_onnx(onnx_path)
    if layers[0][0].shape[1] != L.JOINT_FEATURES + L.ARM_FEATURES:
        raise ValueError(f"{onnx_path}: first layer takes {layers[0][0].shape[1]}, layout v3 is {L.JOINT_FEATURES + L.ARM_FEATURES}")
    if check_inputs is None or len(check_inputs) == 0:
        raise ValueError("write_weights needs real windows to check against")
    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    worst = 0.0
    for row in check_inputs:
        a = session.run(["y"], {"x": row.reshape(1, -1).astype(np.float32)})[0].reshape(-1)
        b = mlp_forward(layers, row.reshape(1, -1))[0]
        worst = max(worst, float(np.abs(a - b).max()))
    if worst > PARITY_TOLERANCE_RAD:
        raise RuntimeError(f"the weights' forward pass and the .onnx disagree by {worst:.3g}")
    digest = bytes.fromhex(sha256(onnx_path))
    ident = model_id.encode("utf-8")
    with open(out_path, "wb") as f:
        f.write(WEIGHTS_MAGIC)
        f.write(struct.pack("<IIIII", WEIGHTS_VERSION, L.JOINT_FEATURES, L.ARM_FEATURES, len(L.PREDICTED_MOTORS), len(layers)))
        for w, b in layers:
            f.write(struct.pack("<II", w.shape[1], w.shape[0]))
            f.write(np.ascontiguousarray(w, dtype="<f4").tobytes())
            f.write(np.ascontiguousarray(b, dtype="<f4").tobytes())
        f.write(digest)
        f.write(struct.pack("<I", len(ident)))
        f.write(ident)
    return {"path": out_path, "sha256": sha256(out_path), "onnxSha256": digest.hex(), "parityMaxAbs": worst,
            "modelId": model_id}
