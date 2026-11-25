"""A twin take with the depth model's streams and records (docs/adr/0039: FM records, <camera>.depth_femto parts in every
VF, isaac/depth_model.json) is read exactly as the take without them by every reader of takes: teleop_lerobot's take,
camera sidecars and visibility, which key on CM's cameras and their rgb parts and pass over other tags and streams. The
synthetic takes come from robot/lerobot/tests/conftest.py, loaded by path (robot/lerobot is not changed by this work)."""

import importlib.util
import os
import sys

import pytest

from conftest import REPO

LEROBOT = os.path.join(REPO, "robot", "lerobot")


@pytest.fixture(scope="module")
def lr():
    pytest.importorskip("numpy")
    spec = importlib.util.spec_from_file_location("lerobot_take_helpers", os.path.join(LEROBOT, "tests", "conftest.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # puts robot/lerobot on sys.path
    from teleop_lerobot import sidecars, visibility
    from teleop_lerobot.jlog import read_jlog
    from teleop_lerobot.takes import read_take

    return mod, read_take, read_jlog, sidecars, visibility


FM = [["FM", cam, "depth_femto", "sensor", 640, 576, 515.9885, 515.4943, 326.7456, 326.086, 0.0,
       "-0.3435:0.2326:0.0:0.0:0.0:0.0:0.0:0.0", 0.2, 5.46, 0.88, 0.48, 0.32, 0.7, 0.1, 0.2, 0.6, "base", "ab" * 32, 0.81, 0.0004]
      for cam in ("third-left", "third-right")]


def _with_femto(mod, take, cams=("third-left", "third-right")):
    """Add the model's parts to every VF of ``take``'s isaac.jlog, their sidecars and depth_model.json, as the renderer
    writes them, and its files.sha256 again."""
    path = os.path.join(take, "isaac.jlog")
    with open(path, encoding="ascii") as f:
        lines = f.read().splitlines()
    head, rows = lines[0].split("|"), [line.split("|") for line in lines[1:-1]]
    cars = {c: mod.Sidecar(os.path.join(take, "isaac", f"{c}.depth_femto.frames")) for c in cams}
    for r in rows:
        if r[0] == "VF":
            for c in cams:
                off, length = cars[c].add(f"{c}-femto-{r[2]}".encode())
                r[5] = f"{r[5]};{c}.depth_femto:{off}:{length}"
    for car in cars.values():
        car.close()
    mod.write_jlog(path, "isaac", head[5], rows)
    with open(os.path.join(take, "isaac", "depth_model.json"), "w", encoding="utf-8") as f:
        f.write('{"format": "teleop-depth-model-used"}\n')
    mod.finish(take)
    return take


def _pair(lr, tmp_path):
    mod = lr[0]
    cams = ("third-left", "third-right", "wrist")
    plain = mod.jetrover_take(str(tmp_path / "a"), cameras=cams)
    femto = _with_femto(mod, mod.jetrover_take(str(tmp_path / "b"), cameras=cams, isaac_extra=FM))
    return plain, femto


def test_a_take_with_the_depth_model_builds_the_same_episode(lr, tmp_path):
    mod, read_take, read_jlog, sidecars, visibility = lr
    plain, femto = _pair(lr, tmp_path)
    assert os.path.exists(os.path.join(femto, "isaac", "third-right.depth_femto.frames"))
    jb = read_jlog(os.path.join(femto, "isaac.jlog"), "isaac")
    assert len(jb.tag("FM")) == 2 and all("depth_femto" in r[4] for r in jb.tag("VF"))
    a, b = read_take(plain), read_take(femto)
    assert [c.name for c in b.cameras] == [c.name for c in a.cameras] == ["third-left", "third-right", "wrist"]
    assert len(a.frames) == len(b.frames) > 0
    for fa, fb in zip(a.frames, b.frames):
        assert fb.parts == fa.parts and fb.state == fa.state and fb.action == fa.action and fb.timing == fa.timing
    assert [b.jpeg(f, c) for f in b.frames for c in b.cameras] == [a.jpeg(f, c) for f in a.frames for c in a.cameras]


def test_the_camera_sidecars_and_visibility_see_only_the_cm_cameras(lr, tmp_path):
    mod, read_take, read_jlog, sidecars, visibility = lr
    plain, femto = _pair(lr, tmp_path)
    ja, jb = (read_jlog(os.path.join(t, "isaac.jlog"), "isaac") for t in (plain, femto))
    rename = {"wrist": "front"}
    assert sidecars.cameras(jb, rename) == sidecars.cameras(ja, rename)
    assert [repr(v) for v in visibility.views(jb, rename)] == [repr(v) for v in visibility.views(ja, rename)]


def test_the_twin_check_reads_the_modelled_stream(lr, tmp_path):
    """depth_stats reads a take's depth_femto frames and their FM intrinsics (the twin check's tool)."""
    import numpy as np
    import cv2

    import depth_stats

    mod = lr[0]
    take = mod.jetrover_take(str(tmp_path / "c"), cameras=("third-left",), isaac_extra=FM[:1])
    path = os.path.join(take, "isaac.jlog")
    with open(path, encoding="ascii") as f:
        lines = f.read().splitlines()
    head, rows = lines[0].split("|"), [line.split("|") for line in lines[1:-1]]
    car = mod.Sidecar(os.path.join(take, "isaac", "third-left.depth_femto.frames"))
    img = np.full((576, 640), 900, np.uint16)
    ok, png = cv2.imencode(".png", img)
    for r in rows:
        if r[0] == "VF":
            off, length = car.add(png.tobytes())
            r[5] = f"{r[5]};third-left.depth_femto:{off}:{length}"
    car.close()
    mod.write_jlog(path, "isaac", head[5], rows)
    frames, intr = depth_stats.read_take_stream(take, "third-left", "depth_femto")
    assert len(frames) == 91 and (frames[0] == 900).all() and intr["fx"] == 515.9885 and intr["cy"] == 326.086
