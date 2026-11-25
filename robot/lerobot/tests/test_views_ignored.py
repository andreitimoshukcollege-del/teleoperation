"""A twin take with view cameras (docs/adr/0038 "V, views": VS, VW and VP records, view<k> streams in every VF) builds the
same episode as the take without them: every reader here keys on CM's cameras and skips other streams and tags, so the
safety datasets never carry a view."""

import os

from conftest import jetrover_take
from teleop_lerobot import sidecars, visibility
from teleop_lerobot.jlog import read_jlog
from teleop_lerobot.takes import read_take

VS = ["VS", 0, 7, "ab" * 32, 2]
VW = [["VW", 0, f"view{k}", 640, 360, 320.0, 320.0, 320.0, 180.0, 0.05, 6.0, 1.2, 0.3, 0.6, 0.7, 0.1, 0.2, 0.6,
       "rgb;depth;mask", "moving" if k else "static", f"75.0:20.0:35.0:1.3:1.5:0.3:0.0:-0.1:{k + 2}", 1.1,
       "20.0:30.0:5.0:0.1:0.02:0.1:0.2:0.3:0.4" if k else ""] for k in range(2)]


def _pair(tmp_path):
    plain = jetrover_take(str(tmp_path / "a"), cameras=("third-left", "third-right", "wrist"))
    viewed = jetrover_take(str(tmp_path / "b"), cameras=("third-left", "third-right", "wrist"), isaac_extra=[VS, *VW],
                           views=("view0", "view1"))
    return plain, viewed


def test_a_take_with_view_cameras_builds_the_same_episode(tmp_path):
    plain, viewed = _pair(tmp_path)
    assert os.path.exists(os.path.join(viewed, "isaac", "view1.mask.frames"))
    a, b = read_take(plain), read_take(viewed)
    assert [c.name for c in b.cameras] == [c.name for c in a.cameras] == ["third-left", "third-right", "wrist"]
    assert len(a.frames) == len(b.frames) > 0
    for fa, fb in zip(a.frames, b.frames):
        assert fb.parts == fa.parts and fb.state == fa.state and fb.action == fa.action and fb.timing == fa.timing
    assert [b.jpeg(f, c) for f in b.frames for c in b.cameras] == [a.jpeg(f, c) for f in a.frames for c in a.cameras]


def test_the_camera_sidecars_and_visibility_see_only_the_cm_cameras(tmp_path):
    plain, viewed = _pair(tmp_path)
    ja, jb = (read_jlog(os.path.join(t, "isaac.jlog"), "isaac") for t in (plain, viewed))
    assert jb.tag("VW") and jb.tag("VP") and jb.tag("VS")
    rename = {"wrist": "front"}
    assert sidecars.cameras(jb, rename) == sidecars.cameras(ja, rename)
    assert [repr(v) for v in visibility.views(jb, rename)] == [repr(v) for v in visibility.views(ja, rename)]
