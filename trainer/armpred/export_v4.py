"""Layout v4's export (docs/adr/0028 decision 2): the ONNX graph through ``FlatExportV4``, the NumPy forward pass
the repo's C# backend computes, and ``model.weights`` format 2 (contract/MODEL_FORMAT.md, "Weights, version 2").

Layout v3's export (``export.py``) is not touched: its files, its golden and the JetRover's model keep format 1.
"""

from __future__ import annotations

import math
import os
import struct
from typing import Dict, Optional, Sequence

import numpy as np
import torch

from . import MODEL_FORMAT_VERSION, __version__
from .export import ALLOWED_OPS, MLP_LAYERS, OPSET, PARITY_TOLERANCE_RAD, WEIGHTS_MAGIC, sha256
from .features import layout_v4 as L4
from .models.net import FlatExportV4

WEIGHTS_VERSION_V2 = 2
LOG_SCALE_MIN, LOG_SCALE_MAX = math.log(1e-4), math.log(0.5)


def arm_of_row(profile=None) -> Sequence[int]:
    return [a for _, a, _ in L4._rows(profile or L4.default_profile())]


def input_length(profile=None) -> int:
    rows = arm_of_row(profile)
    arms = max(rows) + 1
    return len(rows) * L4.JOINT_FEATURES + arms * L4.ARM_FEATURES


def flat_inputs_v4(joints: np.ndarray, arms: np.ndarray) -> np.ndarray:
    """[N, rows, J] and [N, arms, A] windows -> [N, rows * J + arms * A], the order FlatExportV4 reads."""
    return np.concatenate([joints.reshape(joints.shape[0], -1), arms.reshape(arms.shape[0], -1)], axis=1).astype(np.float32)


def export_onnx_v4(model: torch.nn.Module, path: str, metadata: Dict[str, str], parity_inputs: Optional[np.ndarray] = None,
                   profile=None) -> Dict[str, object]:
    import onnx
    import onnxruntime as ort

    rows = list(arm_of_row(profile))
    arms = max(rows) + 1
    flat = FlatExportV4(model.eval(), L4.JOINT_FEATURES, L4.ARM_FEATURES, tuple(rows), arms).eval()
    n_in = input_length(profile)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    example = torch.zeros(1, n_in)
    try:
        torch.onnx.export(flat, (example,), path, input_names=["x"], output_names=["y"], opset_version=OPSET, dynamo=False)
    except TypeError:
        torch.onnx.export(flat, (example,), path, input_names=["x"], output_names=["y"], opset_version=OPSET)
    proto = onnx.load(path)
    onnx.checker.check_model(proto)
    ops = sorted({node.op_type for node in proto.graph.node})
    outside = [op for op in ops if op not in ALLOWED_OPS]
    if outside:
        raise RuntimeError(f"exported graph uses ops outside the allowlist: {outside}")
    meta = {"teleop.layout_id": L4.LAYOUT_ID, "teleop.model_format": str(MODEL_FORMAT_VERSION), "teleop.trainer_version": __version__,
            "teleop.input_length": str(n_in), "teleop.output_length": str(2 * len(rows)),
            "teleop.arm_of_row": ",".join(str(a) for a in rows), **metadata}
    for key, value in meta.items():
        entry = proto.metadata_props.add()
        entry.key, entry.value = key, value
    onnx.save(proto, path)
    if parity_inputs is None:
        parity_inputs = np.random.default_rng(0).normal(0.0, 1.0, size=(64, n_in)).astype(np.float32)
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(path, options, providers=["CPUExecutionProvider"])
    worst = 0.0
    with torch.no_grad():
        for row in parity_inputs:
            x = row.reshape(1, n_in).astype(np.float32)
            worst = max(worst, float(np.abs(session.run(["y"], {"x": x})[0] - flat(torch.from_numpy(x)).numpy()).max()))
    if worst > PARITY_TOLERANCE_RAD:
        raise RuntimeError(f"ONNX and torch disagree by {worst:.3g}, above {PARITY_TOLERANCE_RAD}")
    return {"path": path, "sha256": sha256(path), "opset": OPSET, "ops": ops, "parity_max_abs": worst,
            "parity_rows": int(len(parity_inputs)), "input_length": n_in, "metadata": meta}


def mlp_layers_from_onnx_v4(onnx_path: str):
    """The four Gemm layers of a v4 mlp export, as [(W [out, in], b [out])] float32."""
    import onnx
    from onnx import numpy_helper

    proto = onnx.load(onnx_path)
    init = {i.name: numpy_helper.to_array(i) for i in proto.graph.initializer}
    out = []
    for p in MLP_LAYERS:
        if f"{p}.weight" not in init:
            raise ValueError(f"{onnx_path}: not an mlp export (no {p}.weight)")
        out.append((init[f"{p}.weight"].astype(np.float32), init[f"{p}.bias"].astype(np.float32)))
    return out


def mlp_forward_v4(layers, x: np.ndarray, rows: Sequence[int], arms: int) -> np.ndarray:
    """FlatExportV4's arithmetic in NumPy float32: x [N, input] -> y [N, rows * 2]. What the C# backend does."""
    n = len(rows)
    jf, af = L4.JOINT_FEATURES, L4.ARM_FEATURES
    out = []
    for row in x.astype(np.float32):
        joints = row[: n * jf].reshape(n, jf)
        blocks = row[n * jf:].reshape(arms, af)
        h = np.concatenate([joints, blocks[np.asarray(rows)]], axis=1)
        for k, (w, b) in enumerate(layers):
            h = h @ w.T + b
            if k < len(layers) - 1:
                h = np.maximum(h, 0.0)
        flag = joints[:, L4.HAND_FLAG_INDEX]
        cap = np.float32(L4.MAX_CORRECTION_RAD) + (np.float32(L4.HAND_MAX_CORRECTION) - np.float32(L4.MAX_CORRECTION_RAD)) * flag
        corr = cap * np.tanh(h[:, 0])
        log_b = np.clip(h[:, 1], np.float32(LOG_SCALE_MIN), np.float32(LOG_SCALE_MAX))
        out.append(np.stack([corr, log_b], axis=1).reshape(-1))
    return np.array(out, dtype=np.float32)


def write_weights_v2(onnx_path: str, out_path: str, model_id: str, check_inputs: np.ndarray, profile=None) -> Dict[str, object]:
    """``model.weights`` format 2 from the scored ``.onnx``, after checking the NumPy forward pass against onnxruntime
    on real windows. The header carries what format 1 left to the layout: its id, the rows and arms, each row's arm,
    the hand flag's index and both caps."""
    import onnxruntime as ort

    rows = list(arm_of_row(profile))
    arms = max(rows) + 1
    layers = mlp_layers_from_onnx_v4(onnx_path)
    if layers[0][0].shape[1] != L4.JOINT_FEATURES + L4.ARM_FEATURES:
        raise ValueError(f"{onnx_path}: first layer takes {layers[0][0].shape[1]}, layout v4 is {L4.JOINT_FEATURES + L4.ARM_FEATURES}")
    if check_inputs is None or len(check_inputs) == 0:
        raise ValueError("write_weights_v2 needs real windows to check against")
    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    worst = 0.0
    for x in check_inputs:
        a = session.run(["y"], {"x": x.reshape(1, -1).astype(np.float32)})[0].reshape(-1)
        b = mlp_forward_v4(layers, x.reshape(1, -1), rows, arms)[0]
        worst = max(worst, float(np.abs(a - b).max()))
    if worst > PARITY_TOLERANCE_RAD:
        raise RuntimeError(f"the weights' forward pass and the .onnx disagree by {worst:.3g}")
    digest = bytes.fromhex(sha256(onnx_path))
    ident, layout = model_id.encode("utf-8"), L4.LAYOUT_ID.encode("utf-8")
    with open(out_path, "wb") as f:
        f.write(WEIGHTS_MAGIC)
        f.write(struct.pack("<IIIIII", WEIGHTS_VERSION_V2, L4.JOINT_FEATURES, L4.ARM_FEATURES, len(rows), arms, len(layers)))
        f.write(struct.pack("<I", len(layout)))
        f.write(layout)
        f.write(bytes(rows))
        f.write(struct.pack("<Iff", L4.HAND_FLAG_INDEX, L4.MAX_CORRECTION_RAD, L4.HAND_MAX_CORRECTION))
        for w, b in layers:
            f.write(struct.pack("<II", w.shape[1], w.shape[0]))
            f.write(np.ascontiguousarray(w, dtype="<f4").tobytes())
            f.write(np.ascontiguousarray(b, dtype="<f4").tobytes())
        f.write(digest)
        f.write(struct.pack("<I", len(ident)))
        f.write(ident)
    return {"path": out_path, "sha256": sha256(out_path), "onnxSha256": digest.hex(), "parityMaxAbs": worst, "modelId": model_id}
