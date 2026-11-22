"""A take written as a real LeRobot dataset and read back by LeRobot (docs/adr/0036). Runs only where ``lerobot`` is
installed: the GPU machine's converter venv (just lerobot-test)."""

import io
import os

import pytest

lerobot = pytest.importorskip("lerobot")
from PIL import Image  # noqa: E402  (lerobot brings Pillow)

from conftest import jetrover_take  # noqa: E402
from teleop_lerobot.takes import read_take  # noqa: E402
from teleop_lerobot.writer import LeRobotWriter, append_take, read_ledger, repo_id  # noqa: E402


def jpeg(cam, n):
    buf = io.BytesIO()
    Image.new("RGB", (64, 48), ((n * 7) % 256, 40 if cam == "wrist" else 200, 90)).save(buf, "JPEG", quality=92)
    return buf.getvalue()


def test_two_takes_become_two_episodes_lerobot_reads_back(scenarios, tmp_path):
    from lerobot.datasets import LeRobotDataset

    first = read_take(jetrover_take(scenarios, image=jpeg))
    second = read_take(jetrover_take(scenarios, name="20261112T100500Z-take002", image=jpeg))
    root = str(tmp_path / "jetrover-isaac" / "reach")
    writer = LeRobotWriter()
    assert append_take(writer, root, first)["episode"] == 0
    assert append_take(writer, root, second)["episode"] == 1  # resumed, not recreated
    ds = LeRobotDataset(repo_id(first.dataset_group, first.scenario), root=root)
    assert ds.meta.total_episodes == 2 and ds.meta.fps == 15 and ds.meta.robot_type == "jetrover"
    assert len(ds) == len(first.frames) + len(second.frames)
    assert ds.meta.features["observation.state"]["names"] == first.names
    assert {"observation.images.third-left", "observation.images.wrist"} <= set(ds.meta.features)
    item = ds[0]
    assert list(item["observation.state"].numpy()) == pytest.approx(first.frames[0].state)
    assert list(item["action"].numpy()) == pytest.approx(first.frames[0].action)
    assert item["task"] == "reach the red cube"
    assert tuple(item["observation.images.wrist"].shape) == (3, 48, 64)
    assert [e["episode"] for e in read_ledger(root)] == [0, 1]
    assert os.path.exists(os.path.join(root, "meta", "teleop", "episode_000001.timing.jsonl"))


def test_a_safety_writer_encodes_at_its_own_crf_and_says_so(scenarios, tmp_path, monkeypatch):
    # LeRobot's own CRF 30 softened the twin's wrist camera (2026-11-15): the safety datasets are encoded at SAFETY_CRF.
    import json

    from lerobot.datasets import LeRobotDataset, dataset_writer
    from teleop_lerobot.writer import SAFETY_CRF, SAFETY_PRESET

    seen = []
    real = dataset_writer.encode_video_frames

    def spy(*args, **kwargs):
        seen.append((kwargs.get("crf"), kwargs.get("preset")))
        return real(*args, **kwargs)

    monkeypatch.setattr(dataset_writer, "encode_video_frames", spy)
    take = read_take(jetrover_take(scenarios, image=jpeg))
    root = str(tmp_path / "safety")
    append_take(LeRobotWriter(crf=SAFETY_CRF, preset=SAFETY_PRESET), root, take)
    assert seen and set(seen) == {(SAFETY_CRF, SAFETY_PRESET)}
    assert dataset_writer.encode_video_frames is spy  # put back after the episode
    assert json.load(open(os.path.join(root, "meta", "teleop", "encoding.json")))["crf"] == SAFETY_CRF
    assert LeRobotDataset(repo_id(take.dataset_group, take.scenario), root=root).meta.total_episodes == 1


def test_a_parallel_safety_build_writes_the_files_a_sequential_one_does(scenarios, tmp_path, monkeypatch):
    # docs/adr/0036 amendment 6: workers decode and encode ahead, the writer appends in order; every file is the one the
    # sequential build writes, byte for byte (but each ledger line's convertedUtc). Two workers as threads here: spawned
    # processes would import torch twice for a test of 64x48 frames.
    from concurrent.futures import ThreadPoolExecutor

    from lerobot.datasets import dataset_writer
    from teleop_lerobot import safety
    from teleop_lerobot.writer import SAFETY_CRF, SAFETY_PRESET

    def jpeg_frame(cam, n):  # a different picture on each camera and frame, so a swap or a shift would show
        buf = io.BytesIO()
        Image.new("RGB", (64, 48), ((n * 11) % 256, 30 if cam == "wrist" else 220, (n * 5) % 256)).save(buf, "JPEG", quality=90)
        return buf.getvalue()

    paths = [jetrover_take(scenarios, name=f"20261121T1000{i}0Z-take00{i}", scenario=c, image=jpeg_frame)
             for i, c in enumerate(("reach_no_obstacle", "stop_before_collision", "reach_no_obstacle"))]
    takes = [read_take(p) for p in paths]
    alone, together = str(tmp_path / "alone" / "jetrover_x"), str(tmp_path / "together" / "jetrover_x")  # one repo name
    safety.build(LeRobotWriter(crf=SAFETY_CRF, preset=SAFETY_PRESET), alone, takes, log=lambda m: None)

    calls = []
    real = dataset_writer.encode_video_frames
    monkeypatch.setattr(dataset_writer, "encode_video_frames", lambda *a, **k: calls.append(1) or real(*a, **k))
    with ThreadPoolExecutor(2) as pool:
        safety.build(LeRobotWriter(crf=SAFETY_CRF, preset=SAFETY_PRESET), together, takes, log=lambda m: None, jobs=2,
                     executor=pool)
    assert calls == []  # every video came from the workers, none from LeRobot's own encode
    assert safety.compare_datasets(alone, together) == []
    files = lambda root: sorted(os.path.relpath(os.path.join(d, n), root) for d, _, ns in os.walk(root) for n in ns)
    assert files(alone) == files(together)
    assert sum(1 for f in files(together) if f.endswith(".mp4")) == 3 * 2  # an episode a file, each camera
    assert not os.path.exists(together + ".prepare")
    from lerobot.datasets import LeRobotDataset

    ds = LeRobotDataset("local/jetrover-isaac-safety_jetrover_x", root=together)
    assert ds.meta.total_episodes == 3 and len(ds) == sum(len(t.frames) for t in takes)
    assert tuple(ds[len(takes[0].frames)]["observation.images.front"].shape) == (3, 48, 64)


def test_safety_build_with_spawned_workers_writes_what_one_process_does(scenarios, tmp_path):
    # The command as the batch runs it, its workers real spawned processes (each imports LeRobot: a few seconds).
    import subprocess
    import sys

    from teleop_lerobot import safety
    from teleop_lerobot.writer import SAFETY_CRF, SAFETY_PRESET

    paths = [jetrover_take(scenarios, name=f"20261121T1100{i}0Z-take00{i}", scenario="reach_no_obstacle", image=jpeg)
             for i in range(2)]
    alone, spawned = str(tmp_path / "a" / "jetrover_y"), str(tmp_path / "b" / "jetrover_y")
    safety.build(LeRobotWriter(crf=SAFETY_CRF, preset=SAFETY_PRESET), alone, [read_take(p) for p in paths], log=lambda m: None)
    run = subprocess.run([sys.executable, "-m", "teleop_lerobot", "safety-build", "--jobs", "2", "--out", spawned, *paths],
                         cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))), capture_output=True, text=True,
                         timeout=300)
    assert run.returncode == 0, run.stdout[-2000:] + run.stderr[-2000:]
    assert "with 2 worker(s)" in run.stdout
    assert safety.compare_datasets(alone, spawned) == []
    assert not os.path.exists(spawned + ".prepare")
