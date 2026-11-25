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
    assert parts[0] == "VW" and len(parts) == 24 and parts[23] == ""  # a drawn view breaks no rule
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
        for spec_path in (RECON, RECON_720, os.path.join(ROOT, "views", "fixed-example.json"),
                          os.path.join(ROOT, "views", "safety.json"), os.path.join(ROOT, "views", "safety-holdout.json")):
            sp = views.load(spec_path, count=-1 if "fixed" in spec_path else views.MAX_COUNT)
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


def test_the_renderers_views_wiring_is_opt_in_and_records_what_it_drew():
    """Everything the renderer does for views hangs off view_spec, which is None without --views or a section; the views get
    no feed and stay out of CM and the mask check; their records come with the change and in each preamble; a moving view's
    VP is the pose set lag - 1 frames before the frame was read."""
    with open(os.path.join(ROOT, "teleop_isaac", "renderer.py"), encoding="utf-8") as f:
        src = f.read()
    run = src[src.index("def run("):]
    assert "view_spec = views_module.load(args.views, scene, args.views_count)" in run
    assert "if view_spec is not None:\n            apply_views(frame_now)" in run
    assert "out.extend(view_records())" in run and "for record in view_records():" in run
    assert "shown = max(0, frames - lag + 1 - view_change[\"frame\"])" in run and 'session_record("VP"' in run
    assert 'streams += [f"view{k}.{st}"' in run and 'out["views.json"]' in run
    assert "if look is not None or view_spec is not None" in run
    # No feed for a view, and views are not among the CM cameras (specs) or the mask check's room cameras.
    block = run[run.index("view_sensors = []"):run.index("app_utils.play(commit=True)")]
    assert "CameraFeed" not in block and "specs.append" not in block
    assert "encode_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix=\"encode\") if view_spec is not None else None" in run


# --- fixed views -------------------------------------------------------------------------------------------------------

FIXED = os.path.join(ROOT, "views", "fixed-example.json")


def _fixed_slot(position=(1.05, 0.62, 0.30), look=(0.26, 0.0, -0.13), **extra):
    return {"width": 640, "height": 360, "fixed": {"position": list(position), "lookAt": list(look), "hfovDeg": 75, **extra}}


def test_the_fixed_example_puts_two_cameras_exactly_where_it_says_every_take(mixed):
    s = views.load(FIXED)
    geo = views.geometry(s, mixed)
    a = views.draw(s, geo, 1, 0)
    assert a == views.draw(s, geo, 99, 7)  # no jitter: the same every take, whatever the seed
    assert [v.mode for v in a] == ["fixed", "fixed"] and all(v.motion is None and v.attempts == 0 for v in a)
    assert a[0].centre_pose()[0] == (1.05, 0.62, 0.30) and a[1].centre_pose()[0] == (1.05, -0.62, 0.30)
    assert all(v.broken == () for v in a)  # sensible spots: they pass every rule
    for v in a:
        p, q = v.pose_at(12.0)
        u, w, _ = views.project((0.26, 0.0, -0.13), p, q, v.intrinsics)
        assert abs(u - 320) < 1e-6 and abs(w - 180) < 1e-6
        assert abs(views.place(v.target, v.azimuth, v.elevation, v.distance)[0] - p[0]) < 1e-5  # the derived draw fields agree


def test_a_fixed_camera_that_breaks_the_rules_is_kept_and_warned_not_moved(mixed):
    d = doc()
    d["slots"] = [_fixed_slot(position=(-0.10, 0.0, 0.0))]  # inside the robot's chassis
    d["count"] = 1
    s = views.parse(d)
    v = views.draw(s, views.geometry(s, mixed), 3, 1)[0]
    assert v.centre_pose()[0] == (-0.10, 0.0, 0.0) and v.mode == "fixed"
    assert "body" in v.broken and v.attempts == 0
    parts = "|".join(field(x) for x in ("VW", *views.vw_fields(s, v, 1))).split("|")
    r = views.parse_vw(parts[1:])
    assert r["mode"] == "fixed" and "body" in r["broken"] and r["attempts"] == 0


def test_a_fixed_cameras_jitter_is_small_seeded_and_per_take(mixed):
    d = doc()
    d["slots"] = [_fixed_slot(jitter={"translationM": 0.01, "rotationDeg": 1.0})]
    d["count"] = 1
    s = views.parse(d)
    geo = views.geometry(s, mixed)
    a, b = views.draw(s, geo, 3, 1)[0], views.draw(s, geo, 3, 2)[0]
    assert a == views.draw(s, geo, 3, 1)[0] and a != b
    for v in (a, b):
        p = v.centre_pose()[0]
        assert max(abs(x - y) for x, y in zip(p, (1.05, 0.62, 0.30))) <= 0.01 + 1e-9
        assert abs(v.roll) <= 1.0
        nominal = views.unit([t - c for t, c in zip((0.26, 0.0, -0.13), (1.05, 0.62, 0.30))])
        assert math.degrees(math.acos(min(1.0, sum(x * y for x, y in zip(v.sight(), nominal))))) < 1.5


def test_a_spec_may_mix_fixed_and_drawn_slots(mixed):
    d = doc()
    d["slots"] = [_fixed_slot(), {"width": 640, "height": 360, "hfovDeg": [60, 90]}]
    s = views.parse(d)
    geo = views.geometry(s, mixed)
    for seed in range(1, 40):
        fixed, drawn = views.draw(s, geo, seed, 0)
        assert fixed.mode == "fixed" and fixed.centre_pose()[0] == (1.05, 0.62, 0.30)
        assert drawn.mode in ("static", "moving") and drawn.attempts > 0
        assert views.broken_rules(s, geo, drawn, [fixed.sight()]) == []  # kept apart from the fixed one


@pytest.mark.parametrize("fixed", [
    {"position": [1.0, 0.5, 0.3], "lookAt": [1.0, 0.5, 0.31], "hfovDeg": 75},    # looks at itself
    {"position": [1.0, 0.5, 0.3], "lookAt": [0.2, 0, 0], "hfovDeg": 150},        # field of view out of bounds
    {"position": [1.0, 0.5], "lookAt": [0.2, 0, 0], "hfovDeg": 75},              # not a point
    {"position": [1.0, 0.5, 0.3], "lookAt": [0.2, 0, 0]},                        # no field of view
    {"position": [1.0, 0.5, 0.3], "lookAt": [0.2, 0, 0], "hfovDeg": 75, "jitter": {"translationM": 0.2}},  # not small
    {"position": [1.0, 0.5, 0.3], "lookAt": [0.2, 0, 0], "hfovDeg": 75, "tilt": 3},  # unknown field
])
def test_a_bad_fixed_slot_is_refused(fixed):
    d = doc()
    d["slots"] = [{"width": 640, "height": 360, "fixed": fixed}]
    d["count"] = 1
    with pytest.raises(views.ViewsError):
        views.parse(d)


def test_a_fixed_slot_cannot_repeat_and_its_sight_covers_it():
    with pytest.raises(views.ViewsError):
        views.load(FIXED, count=3)
    d = doc()
    d["slots"] = [_fixed_slot(position=(3.0, 3.0, 1.0))]
    d["count"] = 1
    centre, reach = views.sight(views.parse(d))
    assert math.dist(centre, (3.0, 3.0, 1.0)) + 6.0 < reach


# --- views in safety batches (docs/adr/0038 V, "Views in safety batches") ------------------------------------------------

SAFETY = os.path.join(ROOT, "views", "safety.json")


def _view_lines(spec, geo, seed, change, gain=1.0):
    """The VS and VW records a session's preamble carries for this change, as the renderer writes them (view_records)."""
    drawn = views.draw(spec, geo, seed, change)
    rows = [("VS", *views.vs_fields(spec, seed, change))] + [("VW", *views.vw_fields(spec, v, change, gain)) for v in drawn]
    return ["|".join(field(x) for x in row) for row in rows]


def test_the_safety_spec_is_one_static_640x360_view_with_recons_rules():
    s = views.load(SAFETY)
    assert s.count == 1 and s.moving is None
    assert [(v.width, v.height, v.hfov, v.fixed) for v in s.slots] == [(640, 360, (60.0, 90.0), None)]
    a, b = doc(SAFETY), doc(RECON)
    for k in ("target", "distanceM", "rollDeg", "near", "far", "record", "recordQuality", "keepOut", "room", "workspace", "accept"):
        assert a[k] == b[k], k  # the same look-at box, rejection rules and records as the reconstruction spec


def test_the_safety_spec_never_draws_a_moving_view_at_any_count(mixed):
    s = views.load(SAFETY, count=3)
    geo = views.geometry(s, mixed)
    for seed in range(1, 121):
        for v in views.draw(s, geo, seed, seed % 5):
            assert v.mode == "static" and v.motion is None and v.attempts != views.FALLBACK
            assert views.pose_at_frame(v, 0, 30) == views.pose_at_frame(v, 451, 30) == v.centre_pose()


def test_a_replay_gets_exactly_its_source_takes_view_poses_and_intrinsics(mixed):
    """A counterfactual replay keeps its source's change (renderer.keep_look: no change after the source closes), so its BK
    has the source's seed and change, and its preamble's VS and VW are the same records: the draw is a pure function of the
    spec, the scene, the seed, the change and the camera index. Shown here with a fresh spec, scene and geometry for the
    replay, other changes drawn in between, and the gain the change metered (kept, since no change re-meters it)."""
    s = views.load(SAFETY)
    source = _view_lines(s, views.geometry(s, mixed), 812345, 17, 1.37)
    other = views.load(SAFETY)  # read again, as nothing is shared between two sessions but the renderer's state
    geo = views.geometry(other, scene_module.load(MIXED))
    for change in (18, 3, 17, 40):
        views.draw(other, geo, 812345, change)
    replay = _view_lines(other, geo, 812345, 17, 1.37)
    assert replay == source
    r = views.parse_vw(replay[1].split("|")[1:])
    v = views.draw(s, geo, 812345, 17)[0]
    assert r["pose"] == tuple(round(c, 7) for c in (*v.centre_pose()[0], *v.centre_pose()[1]))
    assert (r["fx"], r["fy"], r["cx"], r["cy"], r["width"], r["height"]) == \
           (round(v.intrinsics["fx"], 6), round(v.intrinsics["fy"], 6), 320.0, 180.0, 640, 360)
    # A take after a change (not a replay) gets other views: the replay's sameness comes from the kept change.
    assert _view_lines(s, geo, 812345, 18, 1.37)[1:] != source[1:]


def test_the_draw_is_the_same_in_another_process_whatever_its_hash_seed(mixed):
    """views_check recomputes each take's draws offline, in another process: string seeds go through random.Random's sha512,
    not hash(), so PYTHONHASHSEED cannot change them."""
    import subprocess
    import sys
    s = views.load(SAFETY)
    here = _view_lines(s, views.geometry(s, mixed), 4242, 6)
    code = ("import sys; sys.path.insert(0, sys.argv[1]); from teleop_isaac import views, scene; "
            "from teleop_isaac.isaac_session import field; s = views.load(sys.argv[2]); "
            "g = views.geometry(s, scene.load(sys.argv[3])); d = views.draw(s, g, 4242, 6); "
            "rows = [('VS', *views.vs_fields(s, 4242, 6))] + [('VW', *views.vw_fields(s, v, 6, 1.0)) for v in d]; "
            "print('\\n'.join('|'.join(field(x) for x in r) for r in rows))")
    for hash_seed in ("1", "977"):
        out = subprocess.run([sys.executable, "-c", code, ROOT, SAFETY, MIXED], capture_output=True, text=True, check=True,
                             env=dict(os.environ, PYTHONHASHSEED=hash_seed)).stdout.split("\n")
        assert out[:len(here)] == here


def test_the_renderer_draws_and_meters_views_only_at_a_change_so_a_kept_look_keeps_them():
    """The renderer's side of a replay's views, pinned on its source: views_now and view_change (the drawn views, their loop
    start and gains) change only in apply_views and settle_room; both run only in _change_backdrop; and on_closed skips the
    change while keep_look holds. So a replay's preamble repeats its source's VS and VW."""
    import ast
    with open(os.path.join(ROOT, "teleop_isaac", "renderer.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    parent = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parent[child] = node

    def owner(node):
        while node in parent:
            node = parent[node]
            if isinstance(node, ast.FunctionDef):
                return node.name
        return None

    calls = {}
    writes = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in ("apply_views", "settle_room"):
            calls.setdefault(node.func.id, []).append(owner(node))
        targets = node.targets if isinstance(node, ast.Assign) else [node.target] if isinstance(node, ast.AugAssign) else []
        for t in targets:
            base = t.value if isinstance(t, ast.Subscript) else None
            while isinstance(base, ast.Subscript):
                base = base.value
            if isinstance(base, ast.Name) and base.id in ("views_now", "view_change"):
                writes.add(owner(node))
    assert calls == {"apply_views": ["_change_backdrop"], "settle_room": ["_change_backdrop"]}
    assert writes == {"apply_views", "settle_room"}
    on_closed = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "on_closed")
    first_if = next(n for n in on_closed.body if isinstance(n, ast.If))
    assert "keep_look" in ast.unparse(first_if.test) and isinstance(first_if.body[-1], ast.Return)
    appends = [n for n in ast.walk(on_closed) if isinstance(n, ast.Call) and ast.unparse(n.func) == "want_backdrop_change.append"]
    assert len(appends) == 1 and appends[0].lineno > first_if.body[-1].lineno


# --- the held-out region (docs/adr/0038 V, "Views in safety batches") ----------------------------------------------------

HOLDOUT = os.path.join(ROOT, "views", "safety-holdout.json")


def test_the_specs_without_a_held_out_region_draw_exactly_as_before():
    """The draws of recon, recon-720 and fixed-example (and recon at 4 cameras), hashed over their VW records: the same as
    before heldOut existed (measured on c3503b9's views.py)."""
    import hashlib
    h = hashlib.sha256()
    for name in ("recon", "recon-720", "fixed-example"):
        s = views.load(os.path.join(ROOT, "views", name + ".json"))
        assert s.held_out is None and s.split == ""
        for sp in ([s] if name == "fixed-example" else [s, s.with_count(4)]):
            g = views.geometry(sp, scene_module.load(MIXED))
            for seed in range(1, 41):
                for v in views.draw(sp, g, seed, seed % 9):
                    h.update("|".join(field(x) for x in views.vw_fields(sp, v, seed % 9, 1.0)).encode())
    assert h.hexdigest() == "b772805635a1fd0976bba29d96f853b7465e943940482afb2c2e4ceba124f6f1"


def test_the_safety_specs_share_one_held_out_region_and_differ_only_in_their_side_of_it():
    train, test = views.load(SAFETY), views.load(HOLDOUT)
    assert (train.split, test.split) == ("train", "test")
    assert train.held_out == test.held_out == views.Region((60.0, 120.0), (-10.0, 89.0))
    a, b = doc(SAFETY), doc(HOLDOUT)
    assert a["heldOut"] == b["heldOut"]
    assert {k for k in set(a) | set(b) if a.get(k) != b.get(k)} == {"notes", "azimuthDeg", "split"}
    assert (a["azimuthDeg"], b["azimuthDeg"]) == ([0, 360], [65, 115])  # the test views 5 degrees in from the region's edges
    assert test.count == 1 and test.moving is None and train.sha256 != test.sha256


def test_training_never_draws_a_held_out_direction_and_the_test_spec_only_draws_there(mixed):
    for path, side in ((SAFETY, "train"), (HOLDOUT, "test")):
        s = views.load(path, count=2)
        geo = views.geometry(s, mixed)
        n = 0
        for seed in range(1, 301):
            drawn = views.draw(s, geo, seed, seed % 11)
            for i, v in enumerate(drawn):
                n += 1
                assert v.mode == "static" and v.attempts != views.FALLBACK
                assert views.broken_rules(s, geo, v, [w.sight() for w in drawn[:i]]) == []
                assert s.held_out.contains(v.azimuth, v.elevation) == (side == "test")
                parts = "|".join(field(x) for x in ("VW", *views.vw_fields(s, v, 1))).split("|")
                r = views.parse_vw(parts[1:])
                assert views.split_of(s.doc, r["azimuthDeg"], r["elevationDeg"]) == side  # as a dataset reader tells it
                if side == "test":
                    assert 65.0 <= v.azimuth <= 115.0
                # The line of sight's own direction is the drawn one: the region is one of viewing directions.
                sx, sy, sz = v.sight()
                assert abs(math.degrees(math.atan2(-sy, -sx)) % 360.0 - v.azimuth) < 1e-6
                assert abs(math.degrees(math.asin(-sz)) - v.elevation) < 1e-6
        assert n == 600


def test_the_test_views_cover_the_region_at_every_elevation_and_distance(mixed):
    s = views.load(HOLDOUT)
    vs = [views.draw(s, views.geometry(s, mixed), seed, 0)[0] for seed in range(1, 201)]
    assert min(v.azimuth for v in vs) < 70 and max(v.azimuth for v in vs) > 110
    assert min(v.elevation for v in vs) < 15 and max(v.elevation for v in vs) > 70
    assert min(v.distance for v in vs) < 0.8 and max(v.distance for v in vs) > 1.6


def test_even_a_fallback_train_view_stays_out_of_the_region(mixed):
    d = doc(SAFETY)
    d["accept"] = dict(d["accept"], maxAttempts=2, cover=0.99)  # nothing passes: every view is a fallback
    s = views.parse(d)
    geo = views.geometry(s, mixed)
    out = [views.draw(s, geo, seed, 0)[0] for seed in range(1, 200)]
    assert all(v.attempts == views.FALLBACK and not s.held_out.contains(v.azimuth, v.elevation) for v in out)


def test_a_view_from_a_spec_without_a_region_has_no_split():
    assert views.split_of(doc(RECON), 90.0, 30.0) == ""
    assert views.split_of(doc(SAFETY), 90.0, 30.0) == "test" and views.split_of(doc(SAFETY), 59.9, 30.0) == "train"
    assert views.split_of(doc(SAFETY), 120.0, 89.0) == "test" and views.split_of(doc(SAFETY), 120.01, 30.0) == "train"
    wrap = views.Region((330.0, 390.0), (-10.0, 89.0))  # a sector across +x
    assert wrap.contains(350.0, 10.0) and wrap.contains(20.0, 10.0) and not wrap.contains(40.0, 10.0)


@pytest.mark.parametrize("change", [
    {"split": None},                                                           # a region without its split
    {"heldOut": None},                                                         # a split without its region
    {"split": "validation"},
    {"heldOut": {"azimuthDeg": [60, 120]}},                                    # no elevation range
    {"heldOut": {"azimuthDeg": [60, 120], "elevationDeg": [-10, 89], "distanceM": [1, 2]}},
    {"heldOut": {"azimuthDeg": [60, 60], "elevationDeg": [-10, 89]}},          # empty
    {"heldOut": {"azimuthDeg": [0, 400], "elevationDeg": [-10, 89]}},          # more than all round
    {"heldOut": {"azimuthDeg": [0, 340], "elevationDeg": [-10, 89]}},          # a train spec left too little
    {"moving": {"share": 0.3, "periodS": [8, 30]}},                            # a loop could cross the region's edge
    {"slots": [{"width": 640, "height": 360, "fixed": {"position": [1.0, 0.5, 0.3], "lookAt": [0.2, 0, 0], "hfovDeg": 75}}]},
    {"split": "test"},                                                         # a test spec drawing all round
    {"split": "test", "azimuthDeg": [50, 110]},                                # partly outside the region
    {"split": "test", "azimuthDeg": [65, 115], "elevationDeg": [8, 80], "heldOut": {"azimuthDeg": [60, 120], "elevationDeg": [10, 89]}},
])
def test_a_bad_held_out_region_is_refused(change):
    d = doc(SAFETY)
    for k, v in change.items():
        if v is None:
            d.pop(k)
        else:
            d[k] = v
    with pytest.raises(views.ViewsError):
        views.parse(d)


def test_a_moving_share_of_zero_and_a_region_across_north_are_accepted(mixed):
    d = doc(SAFETY)
    d["moving"] = {"share": 0, "periodS": [8, 30]}
    d["heldOut"] = {"azimuthDeg": [-20, 20], "elevationDeg": [-10, 89]}
    s = views.parse(d)
    vs = [views.draw(s, views.geometry(s, mixed), seed, 1)[0] for seed in range(1, 120)]
    assert all(20.0 < v.azimuth < 340.0 and v.mode == "static" for v in vs)


def test_the_offline_statistics_count_draws_rejections_and_the_region(capsys):
    import views_stats
    assert views_stats.main([SAFETY, "--takes", "60", "--region", HOLDOUT]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["views"] == 60 and out["modes"] == {"static": 60} and out["fallbacks"] == 0
    assert out["heldOut"]["viewsInside"] == 0 and abs(out["heldOut"]["uniformShare"] - 1 / 6) < 1e-3
    assert out["azimuthDeg"]["60-90"] == out["azimuthDeg"]["90-120"] == 0 and sum(out["azimuthDeg"].values()) == 60
    assert out["rejected"]["candidates"] > 0 and sum(out["rejected"]["byRule"].values()) >= out["rejected"]["candidates"]
    assert set(out["rejected"]["byRule"]) <= set(views.RULES) and out["attempts"]["median"] >= 1
    assert views_stats.main([HOLDOUT, "--takes", "30"]) == 0
    held = json.loads(capsys.readouterr().out)
    assert held["heldOut"]["viewsInside"] == 30 and held["sha256"] == views.load(HOLDOUT).sha256
