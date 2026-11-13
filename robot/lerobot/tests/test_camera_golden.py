"""The converter reads CameraHost's recording as CameraHost writes it (docs/adr/0014 amendment 3, docs/adr/0036).

``golden/camera/`` is written by the C# test ``CameraSessionGoldenTests`` (``just gen-camera-session-golden``):
frame n captured at 0.5 s + n/30 s, its bytes ``front-n``, and frame 45 without a capture stamp."""

import os
import shutil

from conftest import HERE, finish, jetrover_take
from teleop_lerobot.takes import read_take, verify_checksums

GOLDEN = os.path.join(HERE, "golden", "camera")


def test_a_real_take_with_camerahosts_own_recording_becomes_an_episode_of_front_frames(scenarios):
    take_dir = jetrover_take(scenarios, source="robot")  # the robot's joints, at the golden's frame times
    os.remove(os.path.join(take_dir, "camera.jlog"))
    shutil.rmtree(os.path.join(take_dir, "camera"))
    shutil.copy(os.path.join(GOLDEN, "camera.jlog"), take_dir)
    shutil.copytree(os.path.join(GOLDEN, "camera"), os.path.join(take_dir, "camera"))
    finish(take_dir)
    verify_checksums(take_dir)

    take = read_take(take_dir)
    assert take.dataset_group == "jetrover-real"
    assert [(c.name, c.width, c.height) for c in take.cameras] == [("front", 640, 360)]
    frames = [f.timing["frame"] for f in take.frames]
    # The episode runs 1.0..3.0 s: frames 15 to 75 at 15 fps. Frame 45, never recorded, is stood in for by 44.
    assert frames[0] == 15 and frames[-1] == 75 and len(frames) == 31
    assert 45 not in frames and 44 in frames
    assert [f.timing["filled"] for f in take.frames].count(True) == 0  # 44 stands in on the grid, no repeat
    for f in take.frames:
        assert take.jpeg(f, take.cameras[0]) == f"front-{f.timing['frame']}".encode()
