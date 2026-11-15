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
