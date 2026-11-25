"""A new view for every source take, and exactly its source's view for each replay (docs/adr/0038 V, "A new view for every
source take"): the renderer's change logic (renderer.LookGroup) driven the way a safety batch drives it, session by session,
with the keep file made and removed as the batch does, and views.draw applied to the change each session shows."""

import os
import subprocess
import sys
import tempfile

from conftest import REPO, ROOT
from teleop_isaac import scene as scene_module
from teleop_isaac import views
from teleop_isaac.isaac_session import field
from teleop_isaac.renderer import KEEP_LOOK_FILE, LookGroup

SAFETY = os.path.join(ROOT, "views", "safety.json")
RECON = os.path.join(ROOT, "views", "recon.json")
MIXED = os.path.join(REPO, "core", "Scenes", "jetrover-mixed.json")
HERE = os.path.dirname(os.path.abspath(__file__))

# Twin 0 in the views pilot (2026-11-25): groups 95100001 and 97100001-97200002, each asked for 2 replays, with the replays
# each recorded. The pilot's takes showed changes 0, 1, 2, 2, 2, 2, 3.
PILOT = [(2, 2), (2, 2), (2, 1), (2, 1), (2, 1), (2, 2), (2, 0)]


def simulate(folder, groups, release=True):
    """The change each session of each group shows. ``groups`` is (replays asked for, replays recorded) per source take.

    As on the twin: RobotHost writes the control file when it opens a take and removes it when it closes it; the renderer's
    writer sees each a poll later; joint-drive's ``keep-look: on`` (the batch touches the keep file) comes while the source
    records, its ``keep-look: off`` while the last replay asked for records, or after the replays end; the render loop asks
    ``released`` every frame and makes a pending change. ``release=False`` is the renderer before the fix."""
    control, keep = os.path.join(folder, ".active"), os.path.join(folder, KEEP_LOOK_FILE)
    looks = LookGroup(control)
    state = {"change": 0, "pending": False}  # change 0: the renderer's first, at start-up

    def due():
        state["pending"] = True

    def frame():
        if release and looks.kept_in_a_row:
            looks.released(then=due)
        if state["pending"]:
            state["pending"] = False
            state["change"] += 1

    def touch():
        open(keep, "w").close()

    def remove():
        if os.path.exists(keep):
            os.remove(keep)

    def session(name, during=None):
        with open(control, "w") as f:
            f.write(name)  # RobotHost opens the take
        frame()  # the writer has not polled yet
        looks.opened()
        shown = state["change"]
        frame()
        if during is not None:
            during()
        frame()
        os.remove(control)  # RobotHost closes it
        frame()  # the writer has not closed it yet
        looks.closed(then=due)
        frame()
        return shown

    out = []
    for g, (asked, recorded) in enumerate(groups):
        shown = [session(f"g{g}-source", touch if asked else None)]
        for i in range(1, recorded + 1):
            shown.append(session(f"g{g}-replay{i}", remove if i == asked else None))
        remove()  # joint-drive's keep-look: off when the replays end early, and the batch's rm after the run
        for _ in range(3):
            frame()
        out.append(shown)
    return out


def view_lines(spec_path, seed, groups, release=True):
    """Each session's VS and VW lines, as the renderer writes them for the change it shows (gain 1)."""
    spec = views.load(spec_path)
    geo = views.geometry(spec, scene_module.load(MIXED))
    with tempfile.TemporaryDirectory() as folder:
        changes = simulate(folder, groups, release)
    out = []
    for shown in changes:
        group = []
        for change in shown:
            rows = [("VS", *views.vs_fields(spec, seed, change))]
            rows += [("VW", *views.vw_fields(spec, v, change, 1.0)) for v in views.draw(spec, geo, seed, change)]
            group.append(["|".join(field(x) for x in r) for r in rows])
        out.append(group)
    return out


def test_the_pilots_shared_views_came_from_groups_whose_replays_ended_early(tmp_path):
    """Before the fix: a group whose last replay asked for never recorded closed its last take with the keep file there, and
    no take closed after the file went, so the next source take kept the look and the views. The pilot's changes, exactly."""
    assert [g[0] for g in simulate(str(tmp_path), PILOT, release=False)] == [0, 1, 2, 2, 2, 2, 3]


def test_every_source_take_gets_a_new_change_and_its_replays_keep_it(tmp_path):
    changes = simulate(str(tmp_path), PILOT)
    assert [g[0] for g in changes] == [0, 1, 2, 3, 4, 5, 6]
    for (asked, recorded), shown in zip(PILOT, changes):
        assert len(shown) == 1 + recorded and len(set(shown)) == 1
    # Mixed with batches without replays and groups whose source alone recorded: still one change a group.
    plan = [(0, 0), (3, 3), (3, 0), (0, 0), (1, 1), (3, 2), (1, 0), (2, 2)]
    changes = simulate(str(tmp_path), plan)
    assert [g[0] for g in changes] == list(range(len(plan)))
    assert all(len(set(g)) == 1 for g in changes)


def test_a_batch_whose_groups_all_record_every_replay_changes_exactly_as_before(tmp_path):
    """Where every group records all its replays (and in batches without replays) the change sequence, so every draw, is the
    renderer's before the fix: the fix only adds a change where a group ended early."""
    for plan in ([(2, 2)] * 5, [(0, 0)] * 6, [(1, 1), (0, 0), (3, 3)]):
        assert simulate(str(tmp_path), plan) == simulate(str(tmp_path), plan, release=False)


def test_a_new_draw_per_source_take_and_the_same_draw_within_its_replay_group():
    """The safety view of each session: identical VS and VW within a group (source and its replays), a different view for
    every group, the pilot's plan included."""
    groups = view_lines(SAFETY, 1803303505, PILOT)
    for g in groups:
        assert all(lines == g[0] for lines in g)
    vws = [g[0][1:] for g in groups]
    assert len({tuple(v) for v in vws}) == len(groups)
    poses = [tuple(line.split("|")[11:18]) for v in vws for line in v]
    assert len(set(poses)) == len(groups)  # every group's camera stands somewhere else
    # Before the fix, the pilot's groups 2-5 shared one view.
    before = view_lines(SAFETY, 1803303505, PILOT, release=False)
    assert len({tuple(g[0][1:]) for g in before}) == 4


def test_recon_views_are_also_per_group():
    groups = view_lines(RECON, 4242, [(1, 1), (1, 0), (0, 0), (2, 1)])
    assert all(all(lines == g[0] for lines in g) for g in groups)
    assert len({tuple(g[0][1:]) for g in groups}) == 4


def test_the_per_group_draws_are_the_same_in_another_process_whatever_its_hash_seed():
    here = view_lines(SAFETY, 1803303505, PILOT)
    code = ("import sys, json; sys.path.insert(0, sys.argv[1]); sys.path.insert(0, sys.argv[2]); import test_view_groups as t; "
            "print(json.dumps(t.view_lines(t.SAFETY, 1803303505, t.PILOT)))")
    import json
    for hash_seed in ("1", "977"):
        out = subprocess.run([sys.executable, "-c", code, ROOT, HERE], capture_output=True, text=True, check=True,
                             env=dict(os.environ, PYTHONHASHSEED=hash_seed)).stdout
        assert json.loads(out) == here
