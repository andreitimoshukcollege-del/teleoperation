"""View cameras at random viewpoints (docs/adr/0038 "V, views") without Isaac: the spec, the seeded draws, every rejection
rule, a moving view's loop, the records, and the renderer's opt-in wiring."""

import glob
import json
import math
import os

import pytest

from conftest import REPO, ROOT
from teleop_isaac import scene as scene_module
from teleop_isaac import views
from teleop_isaac.isaac_session import field
from teleop_isaac.renderer import look_at as renderer_look_at
from teleop_isaac.renderer import out_of_sight, parse_args

RECON = os.path.join(ROOT, "views", "recon.json")
RECON_720 = os.path.join(ROOT, "views", "recon-720.json")
MIXED = os.path.join(REPO, "core", "Scenes", "jetrover-mixed.json")
SCENES = sorted(p for p in glob.glob(os.path.join(REPO, "core", "Scenes", "*.json")) if not p.endswith(".shapes.json"))


def doc(path=RECON):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="module")
def mixed():
    return scene_module.load(MIXED)


@pytest.fixture(scope="module")
def spec():
    return views.load(RECON)


@pytest.fixture(scope="module")
def geo(spec, mixed):
    return views.geometry(spec, mixed)


# --- the spec, and that it is opt-in ---------------------------------------------------------------------------------

def test_no_shipped_scene_has_a_views_section_so_the_default_records_as_before():
    for path in SCENES:
        s = scene_module.load(path)
        assert views.SECTION not in s.extra, path
        assert views.load("", s) is None, path


def test_the_shipped_specs_read_with_two_cameras_and_the_720p_one_only_in_its_second_slot():
    s, s720 = views.load(RECON), views.load(RECON_720)
    assert s.count == 2 and s720.count == 2
    assert [(v.width, v.height) for v in s.slots] == [(640, 360), (640, 360)]
    assert [(v.width, v.height) for v in s720.slots] == [(640, 360), (1280, 720)]
    assert s.moving.share == 0.3 and s.record == ("rgb", "depth", "mask")
    a, b = doc(RECON), doc(RECON_720)
    a["slots"][1] = b["slots"][1]
    assert a == b  # nothing else differs


def test_a_count_of_zero_or_the_count_option_turns_views_off_or_resizes_them():
    assert views.load(RECON, count=0) is None
    assert views.load(RECON, count=3).count == 3
    assert views.load(RECON, count=-1).count == 2
    with pytest.raises(views.ViewsError):
        views.load(RECON, count=9)


def test_a_scene_section_is_read_and_the_file_wins_over_it(mixed):
    class Fake:
        extra = {"views": dict(doc(), count=1)}
    assert views.load("", Fake()).count == 1
    assert views.load(RECON, Fake()).count == 2


@pytest.mark.parametrize("change", [
    {"format": "other"},
    {"version": 2},
    {"count": 9},
    {"count": True},
    {"slots": []},
    {"slots": [{"width": 640, "height": 360}]},
    {"slots": [{"width": 640, "height": 360, "hfovDeg": [90, 60]}]},
    {"slots": [{"width": 640, "height": 360, "hfovDeg": [10, 60]}]},
    {"slots": [{"width": 640, "height": 360, "hfovDeg": [60, 90], "depth": 1}]},
    {"target": {"min": [0.5, 0, 0], "max": [0.1, 0, 0]}},
    {"distanceM": [0.1, 2.0]},
    {"elevationDeg": [8, 95]},
    {"rollDeg": 45},
    {"record": ["rgb", "normals"]},
    {"moving": {"share": 1.5, "periodS": [8, 30]}},
    {"moving": {"share": 0.3}},
    {"keepOut": {"boxes": [{"id": "b", "min": [1, 0, 0], "max": [0, 1, 1]}]}},
    {"accept": {"inFrame": 2}},
    {"accept": {"wobble": 1}},
    {"far": 2.5},
    {"colour": [1, 0, 0]},
])
def test_a_bad_spec_is_refused(change):
    d = doc()
    d.update(change)
    with pytest.raises(views.ViewsError):
        views.parse(d)


def test_the_spec_hash_is_of_its_content_and_views_json_carries_it(spec):
    assert spec.sha256 == views.parse(json.loads(json.dumps(doc()))).sha256
    assert views.load(RECON_720).sha256 != spec.sha256
    out = json.loads(views.views_json(spec, "recon.json"))
    assert out["sha256"] == spec.sha256 and out["spec"] == doc() and out["source"] == "recon.json"


# --- geometry ------------------------------------------------------------------------------------------------------------

def test_look_at_is_the_renderers_and_a_roll_turns_about_the_optical_axis():
    for p, t in [((0.9, 0.5, 0.3), (0.1, 0.0, -0.1)), ((-0.6, -1.2, 1.4), (0.3, 0.2, -0.19)), ((1.5, 0.0, 0.2), (0.2, 0.0, 0.0))]:
        a, b = views.look_at(p, t), renderer_look_at(p, t)
        assert abs(abs(sum(x * y for x, y in zip(a, b))) - 1.0) < 1e-12
        intr = views.intrinsics(640, 360, 70.0)
        for roll in (0.0, 4.0, -4.0):
            u, v, _ = views.project(t, p, views.look_at(p, t, roll), intr)
            assert abs(u - 320.0) < 1e-6 and abs(v - 180.0) < 1e-6  # the target stays at the centre
        # A rolled camera sees a point above the target turned by the roll.
        up = (t[0], t[1], t[2] + 0.1)
        u0, v0, _ = views.project(up, p, views.look_at(p, t), intr)
        u1, v1, _ = views.project(up, p, views.look_at(p, t, 4.0), intr)
        turn = math.degrees(math.atan2(v1 - 180, u1 - 320) - math.atan2(v0 - 180, u0 - 320))
        assert abs(abs(turn) - 4.0) < 0.2


def test_place_puts_the_camera_on_its_shell():
    t = (0.3, 0.1, -0.15)
    p = views.place(t, 90.0, 30.0, 1.0)
    assert abs(math.dist(p, t) - 1.0) < 1e-12
    assert abs(p[0] - t[0]) < 1e-12 and p[1] > t[1] and abs(p[2] - t[2] - 0.5) < 1e-12


def test_a_segment_through_a_box_hits_it_and_one_beside_it_does_not():
    box = views.Box("b", (0, 0, 0), (1, 1, 1))
    assert views.segment_hits_box((-1, 0.5, 0.5), (2, 0.5, 0.5), box)
    assert not views.segment_hits_box((-1, 1.5, 0.5), (2, 1.5, 0.5), box)
    assert not views.segment_hits_box((-1, 0.5, 0.5), (-0.5, 0.5, 0.5), box)  # stops short


# --- the rules ---------------------------------------------------------------------------------------------------------

def _rules(spec, geo, p, t=(0.28, 0.0, -0.1), hfov=75.0, others=()):
    return views.check(spec, geo, p, views.look_at(p, t), 640, 360, hfov, t, others)


def test_a_tripod_in_front_of_the_table_passes(spec, geo):
    assert _rules(spec, geo, views.place((0.28, 0.0, -0.1), 20.0, 35.0, 1.3)) == []


@pytest.mark.parametrize("where,rule", [
    ((-0.10, 0.0, 0.0), "body"),                # in the robot's chassis
    ((0.0, 0.3, 0.3), "arm"),                   # where the arm sweeps
    ((0.4, 0.5, 0.2), "surface"),               # just above the table, among its objects
    ((1.6, 0.0, -0.9), "floor"),                # on the floor
    ((0.9, 0.0, 1.7), "ceiling"),               # through the ceiling
    ((2.8, 0.0, 0.6), "walls"),                 # beyond the room's walls
])
def test_each_rule_refuses_its_case(spec, geo, where, rule):
    assert rule in _rules(spec, geo, where)


def test_a_camera_looking_away_from_the_workspace_is_out_of_frame(spec, geo):
    p = (1.3, 0.0, 0.4)
    assert "in_frame" in views.check(spec, geo, p, views.look_at(p, (2.5, 1.0, 0.4)), 640, 360, 70.0, (0.28, 0, -0.1))


def test_a_low_camera_behind_the_robot_sees_the_workspace_through_its_body_and_is_refused(spec, geo):
    p = (-1.4, 0.0, -0.05)
    assert "visible" in _rules(spec, geo, p)


def test_a_far_wide_camera_sees_the_workspace_too_small(spec, geo):
    t = (0.28, 0.0, -0.1)
    p = views.place(t, 0.0, 30.0, 2.0)
    assert views.check(spec, geo, p, views.look_at(p, t), 640, 360, 110.0, t) == ["cover"]


def test_two_views_along_the_same_line_are_refused_by_separation(spec, geo):
    t = (0.28, 0.0, -0.1)
    p = views.place(t, 20.0, 35.0, 1.3)
    other = views.unit([a - b for a, b in zip(t, views.place(t, 25.0, 38.0, 1.0))])
    assert "separation" in _rules(spec, geo, p, t, others=[other])
    far = views.unit([a - b for a, b in zip(t, views.place(t, 120.0, 38.0, 1.0))])
    assert "separation" not in _rules(spec, geo, p, t, others=[far])


# --- the draws ---------------------------------------------------------------------------------------------------------

def test_a_draw_is_a_pure_function_of_seed_change_and_index(spec, geo):
    a = views.draw(spec, geo, 1234, 5)
    assert a == views.draw(spec, geo, 1234, 5)
    assert a != views.draw(spec, geo, 1234, 6)
    assert a != views.draw(spec, geo, 1235, 5)
    assert [v.id for v in a] == ["view0", "view1"]


def test_more_views_leave_the_earlier_ones_as_they_were(spec, geo):
    two = views.draw(spec, geo, 77, 3)
    four = views.draw(spec.with_count(4), geo, 77, 3)
    assert four[:2] == two and len(four) == 4


def test_every_draw_passes_every_rule_within_its_bounds_and_moves_about_three_in_ten(spec, geo):
    n = moving = 0
    for seed in range(1, 201):
        drawn = views.draw(spec, geo, seed, seed % 7)
        for i, v in enumerate(drawn):
            n += 1
            assert v.attempts != views.FALLBACK
            assert views.broken_rules(spec, geo, v, [w.sight() for w in drawn[:i]]) == []
            assert 0 <= v.azimuth < 360 and spec.elevation[0] <= v.elevation <= spec.elevation[1]
            assert spec.distance[0] <= v.distance <= spec.distance[1] and abs(v.roll) <= spec.roll
            assert spec.slot(i).hfov[0] <= v.hfov <= spec.slot(i).hfov[1]
            assert all(a <= c <= b for a, c, b in zip(spec.target_lo, v.target, spec.target_hi))
            moving += v.motion is not None
    assert 0.22 < moving / n < 0.38


def test_the_draws_go_all_round_and_cover_heights_and_distances(spec, geo):
    vs = [v for seed in range(1, 151) for v in views.draw(spec, geo, seed, 0)]
    octants = {int(v.azimuth // 45) for v in vs}
    assert octants == set(range(8))  # all the way round: behind the robot only where its body allows
    assert min(v.elevation for v in vs) < 15 and max(v.elevation for v in vs) > 70
    assert min(v.distance for v in vs) < 0.8 and max(v.distance for v in vs) > 1.8


def test_the_720p_spec_draws_its_second_view_at_1280x720(mixed):
    s = views.load(RECON_720)
    v = views.draw(s, views.geometry(s, mixed), 9, 1)
    assert (v[0].width, v[0].height) == (640, 360) and (v[1].width, v[1].height) == (1280, 720)
    assert v[1].intrinsics["cx"] == 640.0 and v[1].intrinsics["cy"] == 360.0


def test_with_no_candidate_passing_the_fewest_broken_is_kept_and_flagged(mixed):
    d = doc()
    d["accept"] = dict(d["accept"], maxAttempts=3, cover=0.99)  # nothing fills 99% of the image
    s = views.parse(d)
    out = views.draw(s, views.geometry(s, mixed), 1, 0)
    assert all(v.attempts == views.FALLBACK for v in out)


# --- a moving view -----------------------------------------------------------------------------------------------------

def _a_moving_view(spec, geo):
    for seed in range(1, 100):
        for v in views.draw(spec, geo, seed, 0):
            if v.motion is not None:
                return v
    raise AssertionError("no moving view drawn")


def test_a_moving_loop_closes_moves_smoothly_and_keeps_its_field_of_view(spec, geo):
    v = _a_moving_view(spec, geo)
    p0, q0 = v.pose_at(0.0)
    p1, q1 = v.pose_at(v.motion.period)
    assert math.dist(p0, p1) < 1e-9 and abs(abs(sum(a * b for a, b in zip(q0, q1))) - 1) < 1e-12
    fps = 30.0
    steps = [math.dist(views.pose_at_frame(v, k, fps)[0], views.pose_at_frame(v, k + 1, fps)[0]) for k in range(int(v.motion.period * fps))]
    assert max(steps) < 0.05  # under 1.5 m/s at 30 fps
    assert max(steps) > 0
    assert v.intrinsics == views.intrinsics(v.width, v.height, v.hfov)
    # Its target stays at the centre of the image all the way round.
    for k in range(0, int(v.motion.period * fps), 37):
        t = k / fps
        tgt = v.params_at(t)[0]
        p, q = v.pose_at(t)
        u, w, _ = views.project(tgt, p, q, v.intrinsics)
        assert abs(u - v.width / 2) < 1e-6 and abs(w - v.height / 2) < 1e-6


def test_a_static_view_holds_its_pose(spec, geo):
    v = next(v for seed in range(1, 50) for v in views.draw(spec, geo, seed, 0) if v.motion is None)
    assert views.pose_at_frame(v, 0, 30) == views.pose_at_frame(v, 900, 30) == v.centre_pose()


# --- records -----------------------------------------------------------------------------------------------------------

def test_vw_vs_and_vp_encode_as_documented_and_read_back(spec, geo):
    drawn = views.draw(spec, geo, 42, 2)
    v = drawn[0]
    f = views.vw_fields(spec, v, 2, 1.23456)
    line = "|".join(field(x) for x in ("VW", *f))
    parts = line.split("|")
    assert parts[0] == "VW" and len(parts) == 23
    r = views.parse_vw(parts[1:])
    assert r["camera"] == "view0" and r["change"] == 2 and r["width"] == 640 and r["height"] == 360
    assert r["streams"] == ["rgb", "depth", "mask"] and r["mode"] == v.mode and r["gain"] == 1.235
    assert r["hfovDeg"] == v.hfov and r["azimuthDeg"] == v.azimuth and r["target"] == v.target and r["attempts"] == v.attempts
    assert abs(r["fx"] - v.intrinsics["fx"]) < 1e-6 and r["cx"] == 320.0
    p, q = v.centre_pose()
    assert all(abs(a - b) < 1e-6 for a, b in zip(r["pose"], (*p, *q)))
    assert (r["motion"] is None) == (v.motion is None)
    assert field(views.vs_fields(spec, 7, 2)) == f"2;7;{spec.sha256};2"
    vp = field(views.vp_items([("view1", ((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0)))]))
    assert vp == "view1:1.0:2.0:3.0:1.0:0.0:0.0:0.0"


def test_a_moving_views_motion_field_reads_back(spec, geo):
    v = _a_moving_view(spec, geo)
    parts = "|".join(field(x) for x in ("VW", *views.vw_fields(spec, v, 0))).split("|")
    m = views.parse_vw(parts[1:])["motion"]
    assert m[:5] == [v.motion.period, v.motion.azimuth_amp, v.motion.elevation_amp, v.motion.distance_amp, v.motion.target_amp]
    assert m[5:] == list(v.motion.phases)


# --- the renderer's wiring ---------------------------------------------------------------------------------------------

def test_the_renderer_takes_views_only_when_asked():
    a = parse_args(["--scene", MIXED])
    assert a.views == "" and a.views_count == -1
    b = parse_args(["--scene", MIXED, "--views", RECON, "--views-count", "3"])
    assert b.views == RECON and b.views_count == 3
    with pytest.raises(SystemExit):
        parse_args(["--scene", MIXED, "--views-count", "9"])


def test_the_shelf_stays_out_of_every_views_sight_in_every_scene():
    from teleop_isaac.world import RESIZE_SCALES, IsaacWorld
    for path in SCENES:
        s = scene_module.load(path)
        for spec_path in (RECON, RECON_720):
            sp = views.load(spec_path, count=views.MAX_COUNT)
            sight = [views.sight(sp)]
            world = IsaacWorld(s, "/robot")
            for i in range(len(s.objects)):
                for k, f in enumerate(RESIZE_SCALES):
                    assert out_of_sight(s.parked_pose(i, f)[:3], sight), (path, s.objects[i].name, f)
                    assert out_of_sight(world.parked(i, k)[:3], sight), (path, s.objects[i].name, k)
            assert not out_of_sight((0.3, 0.0, s.table.top + 0.05), sight)


def test_every_view_place_drawn_is_within_its_sight(spec, geo):
    centre, reach = views.sight(spec)
    for seed in range(1, 60):
        for v in views.draw(spec, geo, seed, 0):
            for t in ([0.0] if v.motion is None else [v.motion.period * i / 12 for i in range(12)]):
                p, _ = v.pose_at(t)
                assert math.dist(p, centre) + spec.far <= reach
