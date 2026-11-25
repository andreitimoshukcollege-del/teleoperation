"""The Femto Bolt-like depth model (docs/adr/0039) without Isaac: the model files, the raw geometry and the depth sensor's
pose, determinism from the seeds, each invalidation rule, the noise statistics against the fitted parameters on planes at
several distances and angles, the aligned stream, the records, our own calibration's import, and the model off (nothing
added, every existing stream untouched). The readers of a take are checked in test_depth_model_readers.py."""

import copy
import inspect
import json
import math
import os

import numpy as np
import pytest

from conftest import REPO, ROOT
from teleop_isaac import depth_model as dm
from teleop_isaac import renderer
from teleop_isaac.isaac_session import field

MODELS = os.path.join(ROOT, "depth_models")
NFOV = os.path.join(MODELS, "femto-nfov.json")
BENCH = os.path.join(REPO, "core", "Scenes", "jetrover-bench.json")
SCENES = ("jetrover-bench", "jetrover-mixed", "jetrover-mixed-holdout")


def doc(name="femto-nfov"):
    with open(os.path.join(MODELS, name + ".json"), encoding="utf-8") as f:
        return json.load(f)


def cameras(scene="jetrover-bench"):
    with open(os.path.join(REPO, "core", "Scenes", scene + ".json"), encoding="utf-8") as f:
        return json.load(f)["cameras"]


@pytest.fixture(scope="module")
def spec():
    return dm.load(NFOV, cameras())


def with_doc(**changes):
    d = doc()
    for path, value in changes.items():
        node = d
        keys = path.split("__")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return dm.parse(d)


def plane_depth(spec, distance_m, tilt_deg=0.0, axis="x"):
    """Exact depth on the raw grid of a plane ``distance_m`` along the optical axis, turned ``tilt_deg`` about the camera's
    x (or y) axis: Z = d / (n . ray) with n the plane's normal."""
    g = dm.geometry(spec.mode)
    t = math.radians(tilt_deg)
    n = (0.0, math.sin(t), math.cos(t)) if axis == "x" else (math.sin(t), 0.0, math.cos(t))
    den = n[0] * g.x + n[1] * g.y + n[2]
    return np.where(den > 1e-3, distance_m * n[2] / den, np.inf).astype(np.float32)


def frame(spec, k=0, camera="third-left", seed=7, change=3):
    return dm.sensor_frame(spec, camera, seed, change, k)


def constant(spec, rho=0.5, **edits):
    d = copy.deepcopy(spec.doc)
    d["reflectance"].update({"source": "constant", "constant": rho})
    for k, v in edits.items():
        d[k] = v
    return dm.parse(d)


# --- the model files ---------------------------------------------------------------------------------------------------

def test_every_shipped_model_reads_against_every_jetrover_scene():
    for name in ("femto-nfov", "femto-binned", "femto-same"):
        for scene in SCENES:
            s = dm.load(os.path.join(MODELS, name + ".json"), cameras(scene))
            assert s.sha256 == dm.canonical_sha256(doc(name))
            assert {r.camera for r in s.rigs} == {"third-left", "third-right"}
    assert dm.load(NFOV).mode.distortion[:2] == (-0.3435, 0.2326)


@pytest.mark.parametrize("path,value,why", [
    ("format", "x", "teleop-depth-model"), ("mode__fx", -1, "fx"), ("mode__minM", 9.0, "minM"), ("mode__cx", 700, "principal"),
    ("mode__distortion", {"k9": 1}, "distortion"), ("mode__distortion", {"k1": 40.0}, "folds"),
    ("depthToColour__rotation", [[1, 0, 0], [0, 2, 0], [0, 0, 1]], "orthonormal"), ("depthToColour__translationM", [1, 0, 0], "translation"),
    ("illumination__polygonTan", [[0, 0], [1, 1]], "polygon"), ("noise__kernelPx", 0, "kernelPx"), ("reflectance__source", "nir", "source"),
    ("edges__windowPx", 4, "odd"), ("signal__saturation", -1, "saturation"), ("cameras", [], "nothing"),
    ("cameras", [{"camera": "nope"}], "not a camera"), ("cameras", [{"camera": "wrist"}], "rides"),
    ("cameras", [{"camera": "third-left", "geometry": "same", "streams": ["depth_femto_aligned"]}], "needs geometry sensor"),
    ("cameras", [{"camera": "third-left", "streams": ["depth"]}], "streams"),
    ("cameras", [{"camera": "third-left"}, {"camera": "third-left"}], "twice"),
])
def test_a_bad_model_is_refused(path, value, why):
    d = doc()
    node = d
    keys = path.split("__")
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value
    with pytest.raises(ValueError, match=why):
        dm.parse(d, cameras())


# --- off: nothing changes ---------------------------------------------------------------------------------------------------

def test_off_is_nothing():
    assert renderer.parse_args(["--scene", BENCH]).depth_model == ""
    assert dm.load("") is None
    assert dm.streams(None, ["view0"]) == []


def test_the_renderer_runs_the_model_only_behind_its_flag():
    """Every use of the model in run() is behind ``femto`` (None without --depth-model) or a rig it made: the cameras, records,
    sidecars, encode pool and frames are what they were."""
    src = inspect.getsource(renderer.run)
    assert "femto = depth_model_module.load(args.depth_model, cameras)" in src
    assert "if femto is not None:" in src
    assert "if i in femto_rigs and depth_model_module.due(femto, frames):" in src  # a frame's camera loop: no rig, no job
    assert "if encode_pool is None and femto is not None:" in src  # the encode pool as before without views or the model
    assert "femto_rigs.get(i, (None, None))[1] is not None" in src  # the jitter moves a depth sensor only if there is one
    for fn in ("femto_records", "femto_view_job"):
        body = src[src.index(f"def {fn}("):]
        body = body[:body.index("\n    def ", 10)]
        assert "if femto is None" in body


def test_the_model_never_touches_its_input(spec):
    z = plane_depth(spec, 0.9, 30.0)
    rgb = np.full(z.shape + (3,), 90, np.uint8)
    z0, rgb0 = z.copy(), rgb.copy()
    dm.apply(spec, z, frame(spec), rgb)
    assert np.array_equal(z, z0) and np.array_equal(rgb, rgb0)


# --- geometry and poses ------------------------------------------------------------------------------------------------------

def test_distortion_round_trips_and_the_render_covers_the_raw_grid(spec):
    g = dm.geometry(spec.mode)
    assert (g.render_width, g.render_height) == (646, 644) and abs(g.render_f - 0.86 * spec.mode.fx) < 1e-9
    assert g.inside.all()
    xd, yd = dm.distort(spec.mode, g.x.astype(np.float64), g.y.astype(np.float64))
    u, v = np.meshgrid(np.arange(640), np.arange(576))
    assert np.abs(xd * spec.mode.fx + spec.mode.cx - u).max() < 1e-3
    assert np.abs(yd * spec.mode.fy + spec.mode.cy - v).max() < 1e-3


def test_a_render_resampled_onto_the_raw_grid_is_the_raw_cameras_depth(spec):
    """A plane rendered in the centred pinhole and resampled is, at each raw pixel, the plane's depth along that pixel's
    ray, within the render's own pixel (nearest, never blended)."""
    g = dm.geometry(spec.mode)
    t = math.radians(35)
    n = (0.0, math.sin(t), math.cos(t))
    vr, ur = np.mgrid[0:g.render_height, 0:g.render_width]
    xr, yr = (ur + 0.5 - g.render_width / 2) / g.render_f, (vr + 0.5 - g.render_height / 2) / g.render_f
    render = (0.8 * n[2] / (n[0] * xr + n[1] * yr + n[2])).astype(np.float32)
    raw = dm.to_raw(spec.mode, render)
    exact = plane_depth(spec, 0.8, 35)
    slope = np.abs(np.gradient(exact, axis=0)) + np.abs(np.gradient(exact, axis=1))
    assert np.all(np.abs(raw - exact) <= 1.2 * slope + 1e-5)
    assert set(np.unique(raw)) <= set(np.unique(render))  # every value is a rendered one


def test_the_depth_sensor_stands_where_the_extrinsic_puts_it(spec):
    p, q = dm.sensor_local(spec)
    t = spec.translation
    assert p == pytest.approx((t[0], -t[1], -t[2]))
    forward = dm._q_rot(q, (0.0, 0.0, -1.0))  # a USD camera looks along -z
    tilt = math.degrees(math.asin(-forward[1]))
    assert tilt == pytest.approx(6.03, abs=0.05)  # Microsoft: the depth camera is tilted 6 degrees down from the colour camera
    assert abs(math.hypot(*p) - 0.0323) < 0.001
    # composed with a camera's pose, as the renderer poses the depth sensor and FM records it
    cam = ((1.0, 2.0, 3.0), renderer.look_at((1.0, 2.0, 3.0), (0.0, 0.0, 0.0)))
    wp, wq = dm.compose(cam, (p, q))
    rp, rq = renderer.compose(cam, (p, q))
    assert wp == pytest.approx(rp) and wq == pytest.approx(rq)


# --- determinism -------------------------------------------------------------------------------------------------------------

def test_the_same_take_camera_and_frame_give_the_same_depth(spec):
    z = plane_depth(spec, 0.9, 40.0)
    a = dm.apply(spec, z, frame(spec, 5))
    assert np.array_equal(a, dm.apply(spec, z, frame(spec, 5)))
    for other in (frame(spec, 6), frame(spec, 5, camera="third-right"), frame(spec, 5, seed=8), frame(spec, 5, change=4)):
        b = dm.apply(spec, z, other)
        assert not np.array_equal(a, b)
    dm._take_fields.cache_clear()  # a cold cache gives the same take
    assert np.array_equal(a, dm.apply(spec, z, frame(spec, 5)))
    assert dm.take_systematic(spec, 7, 3, "third-left", z.shape) == dm.take_systematic(spec, 7, 3, "third-left", z.shape)
    assert dm.take_systematic(spec, 7, 3, "third-left", z.shape) != dm.take_systematic(spec, 7, 4, "third-left", z.shape)


def test_a_job_resamples_then_models(spec):
    g = dm.geometry(spec.mode)
    render = np.full((g.render_height, g.render_width), 1.1, np.float32)
    rgb = np.full((g.render_height, g.render_width, 3), 200, np.uint8)
    out = dm.finish(dm.Job(spec, frame(spec, 2), render, rgb, raw=True))
    direct = dm.apply(spec, np.full((576, 640), 1.1, np.float32), frame(spec, 2), np.full((576, 640, 3), 200, np.uint8))
    assert out.dtype == np.uint16 and out.shape == (576, 640) and np.array_equal(out, direct)


# --- invalidation -------------------------------------------------------------------------------------------------------------

def test_no_hit_out_of_range_and_outside_the_illumination_are_zero(spec):
    s = constant(spec)
    z = plane_depth(s, 1.0)
    z[:40, :] = np.inf
    causes = {}
    d = dm.apply(s, z, frame(s), None, causes)
    assert (d[:40] == 0).all()
    lit = causes["illumination"] | causes["nohit"]
    assert (d[causes["illumination"]] == 0).all() and causes["illumination"].sum() > 0.08 * d.size  # the hexagon's corners
    assert d[0, 0] == 0 and d[575, 639] == 0 and d[288, 320] > 0
    assert (dm.apply(s, plane_depth(s, 0.15), frame(s)) == 0).all()  # nearer than minM
    assert (dm.apply(s, plane_depth(s, 5.6), frame(s)) == 0).all()  # beyond maxM
    assert not lit[200:400, 250:400].any()


def test_low_signal_and_grazing_invalidate(spec):
    dark = constant(spec, rho=0.001)  # a black surface far away: a = 0.001 / 1.5^2, well under a50
    d = dm.apply(dark, plane_depth(dark, 1.5), frame(dark))
    assert (d[200:400, 250:400] == 0).mean() > 0.95
    bright = constant(spec, rho=0.9)
    d = dm.apply(bright, plane_depth(bright, 1.5), frame(bright))
    assert (d[200:400, 250:400] == 0).mean() < 0.01
    # a plane turned 87 degrees about the camera's y axis, 3 m out along the axis: seen at 82-87 degrees left of centre
    g = dm.geometry(bright.mode)
    t = math.radians(87.0)
    z = plane_depth(bright, 3.0, 87.0, axis="y")
    causes = {}
    dm.apply(bright, z, frame(bright), None, causes)
    sel = (slice(260, 320), slice(340, 380))
    cos = (math.sin(t) * g.x[sel] + math.cos(t)) / np.sqrt(g.x[sel] ** 2 + g.y[sel] ** 2 + 1)
    ang = np.degrees(np.arccos(cos))
    want = 1 / (1 + np.exp(-(ang - bright.grazing["deg50"]) / bright.grazing["widthDeg"]))
    assert 0.1 < want.mean() < 0.9 and causes["grazing"][sel].mean() == pytest.approx(want.mean(), abs=0.05)
    causes = {}
    dm.apply(bright, plane_depth(bright, 0.9, 30.0), frame(bright), None, causes)
    assert causes["grazing"][250:330, 280:360].mean() < 0.001


def test_a_big_step_loses_its_far_side_and_a_small_one_ramps(spec):
    s = constant(spec, rho=0.6, dropout=0.0)
    z = plane_depth(s, 1.2)
    z[:, 330:] = 0.7  # 500 mm nearer on the right
    causes = {}
    d = dm.apply(s, z, frame(s), None, causes)
    rows = slice(220, 360)
    assert (d[rows, 328:330] == 0).mean() > 0.95  # the far side next to the step: invalid (bg 1.0, span 500 > r50 300)
    assert (d[rows, 330:332] == 0).mean() < 0.05  # the near side: kept (fg 0)
    assert (d[rows, 300:320] > 0).mean() > 0.97 and (d[rows, 340:360] > 0).mean() > 0.97
    # a 60 mm step: nothing invalidated by the edge, intermediate depths across it (the engine's smoothing)
    z = plane_depth(s, 1.0)
    z[:, 330:] -= 0.06
    causes = {}
    d = dm.apply(s, z, frame(s, 1), None, causes)
    assert causes["edge"][rows].sum() <= 3  # g(60 mm) is 3e-4 a pixel
    band = d[rows, 326:334].astype(float)
    flying = (band > 945) & (band < 995)  # between the sides (1000 and 940 mm, noise 0.6 mm)
    assert flying.mean() > 0.18


def test_an_object_against_nothing_keeps_its_near_side(spec):
    s = constant(spec, rho=0.6, dropout=0.0)
    z = plane_depth(s, 1.0)
    z[:, 330:] = np.inf  # the room: an HDRI at infinity
    d = dm.apply(s, z, frame(s))
    assert (d[220:360, 326:330] > 0).mean() > 0.9 and (d[:, 330:] == 0).all()


# --- noise -----------------------------------------------------------------------------------------------------------------

def expected_sigma(spec, z_m, rho, cos):
    n = spec.noise
    return math.sqrt(n["sigma0Mm"] ** 2 + n["sigma1MmPerM"] ** 2 * z_m * z_m / (rho * cos))


@pytest.mark.parametrize("dist,tilt,rho", [(0.6, 0.0, 0.5), (1.0, 0.0, 0.5), (1.5, 0.0, 0.9), (1.0, 60.0, 0.5), (1.0, 0.0, 0.15)])
def test_the_temporal_noise_matches_the_fitted_model(spec, dist, tilt, rho):
    """Frames of one take differ by the per-frame noise alone (the systematic and static fields are the take's): each
    pixel's standard deviation over the frames is the model's sigma, with 1 mm rounding's 1/12 mm^2 added. (The rare
    signal and grazing draws are switched off, so every pixel of the block is there in every frame.)"""
    s = constant(spec, rho=rho, dropout=0.0, signal={"a50": 1e-9, "width": 0.35, "saturation": None},
                 grazing={"deg50": 90.0, "widthDeg": 0.1})
    z = plane_depth(s, dist, tilt)
    frames = np.stack([dm.apply(s, z, frame(s, k)).astype(float) for k in range(40)])
    sel = (slice(258, 318), slice(290, 350))
    block = frames[:, sel[0], sel[1]]
    assert (block > 0).all()
    g = dm.geometry(s.mode)
    t = math.radians(tilt)
    cos = np.abs(np.sin(t) * g.y[sel] + np.cos(t)) / np.sqrt(g.x[sel] ** 2 + g.y[sel] ** 2 + 1)
    zc = float(np.median(z[sel]))
    want = math.sqrt(expected_sigma(s, zc, rho, float(np.median(cos))) ** 2 + 1 / 12)
    got = float(np.median(block.std(0)))
    assert got == pytest.approx(want, rel=0.12)


def test_the_noise_is_correlated_like_the_depth_engines(spec):
    """Two frames' difference (the per-frame noise twice) is correlated along a row as a 4-px box: rho 0.75, 0.5, 0.25, 0,
    against HILO's 0.73, 0.46, 0.22, 0.05 (rounding, small next to this sigma, lowers them a little)."""
    s = constant(spec, rho=0.05, dropout=0.0)  # dark: large sigma, so the rounding hardly matters
    z = plane_depth(s, 1.5)
    a = dm.apply(s, z, frame(s, 0)).astype(float)
    b = dm.apply(s, z, frame(s, 1)).astype(float)
    d = (a - b)[150:450, 150:500]
    ok = (a[150:450, 150:500] > 0) & (b[150:450, 150:500] > 0)
    d = np.where(ok, d - d[ok].mean(), np.nan)
    rho = []
    for k in (1, 2, 3, 4, 6):
        x, y = d[:, :-k].ravel(), d[:, k:].ravel()
        m = np.isfinite(x) & np.isfinite(y)
        rho.append(float(np.corrcoef(x[m], y[m])[0, 1]))
    assert rho[0] == pytest.approx(0.75, abs=0.06) and rho[1] == pytest.approx(0.5, abs=0.06)
    assert rho[2] == pytest.approx(0.25, abs=0.06) and abs(rho[3]) < 0.06 and abs(rho[4]) < 0.06


def test_a_takes_systematic_error_is_its_offset_and_field(spec):
    s = constant(spec, rho=0.9, dropout=0.0)
    z = plane_depth(s, 1.0)
    frames = np.stack([dm.apply(s, z, frame(s, k)).astype(float) for k in range(24)])
    mean_err = frames.mean(0)[200:400, 250:400] - z[200:400, 250:400] * 1000.0
    off, sc = dm.take_systematic(s, 7, 3, "third-left", z.shape)
    _, _, fields = dm._take_fields(7, 3, "third-left", 576, 640, 4, s.noise["staticMm"], s.systematic["fieldSdMm"],
                                   s.systematic["offsetSdMm"], s.systematic["scaleSd"])
    want = off + 1000.0 * sc + fields[200:400, 250:400]
    assert np.abs(mean_err - want).mean() < 0.3  # over 24 frames, with 0.5 mm per-frame noise
    offs = [dm.take_systematic(s, seed, 0, "third-left", (8, 8))[0] for seed in range(400)]  # drawn first: any shape
    assert np.std(offs) == pytest.approx(s.systematic["offsetSdMm"], rel=0.12)


def test_record_every_thins_the_modelled_streams(spec):
    assert spec.record_every == 1 and all(dm.due(spec, k) for k in range(5))
    half = constant(spec, recordEvery=2)
    assert [dm.due(half, k) for k in range(4)] == [True, False, True, False] and not dm.due(None, 0)


def test_depth_is_whole_millimetres_or_the_model_step(spec):
    d = dm.apply(constant(spec), plane_depth(spec, 1.0, 20), frame(spec))
    assert d.dtype == np.uint16
    coarse = constant(spec, quantMm=5)
    d = dm.apply(coarse, plane_depth(coarse, 1.0, 20), frame(coarse))
    assert d.any() and (d % 5 == 0).all()


# --- the aligned stream ------------------------------------------------------------------------------------------------------

def test_alignment_moves_depth_into_the_colour_camera(spec):
    """A fronto-parallel plane 1 m away, aligned into a colour pinhole: Z in the colour frame is the plane's Z there."""
    s = constant(spec, rho=0.9, dropout=0.0)
    g = dm.geometry(s.mode)
    d = dm.apply(s, plane_depth(s, 1.0), frame(s))
    colour = {"width": 640, "height": 360, "fx": 465.6, "fy": 465.6, "cx": 320.0, "cy": 180.0}
    ident = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    a = dm.align_to_colour(d, g.x, g.y, colour, ident, (0.0, 0.0, 0.0))
    assert a.shape == (360, 640) and (a[150:210, 290:350] > 0).all()
    assert abs(float(np.median(a[150:210, 290:350])) - float(np.median(d[d > 0]))) < 4
    # the real extrinsic: the plane is tilted 6 degrees in the colour frame, so Z grows towards the colour image's top
    a = dm.align_to_colour(d, g.x, g.y, colour, s.rotation, s.translation)
    top, bottom = np.median(a[60:80, 300:340][a[60:80, 300:340] > 0]), np.median(a[280:300, 300:340][a[280:300, 300:340] > 0])
    assert top - bottom > 20


# --- records -------------------------------------------------------------------------------------------------------------

def test_fm_records_encode_as_documented(spec):
    rig = spec.rig("third-left")
    raw = dm.raw_intrinsics(spec.mode)
    f = dm.fm_fields(spec, rig, dm.STREAM, raw, (1, 2, 3, 1, 0, 0, 0), "base", 7, 3, (576, 640))
    assert len(f) == 24 and f[0] == "third-left" and f[1] == "depth_femto" and f[2] == "sensor"
    assert f[9] == 0.0 and f[10] == "-0.3435:0.2326:0.0:0.0:0.0:0.0:0.0:0.0" and f[20] == "base" and f[21] == spec.sha256
    line = "|".join(field(x) for x in ("FM", *f))
    assert "|" not in "".join(str(x) for x in f) and line.count("|") == 24
    cm = {"width": 640, "height": 360, "fx": 465.6, "fy": 465.6, "cx": 320.0, "cy": 180.0}
    g = dm.fm_fields(spec, rig, dm.ALIGNED, cm, (1, 2, 3, 1, 0, 0, 0), "base", 7, 3, (576, 640))
    assert g[2] == "aligned" and g[9] == 0.5 and g[10] == "" and g[21:] == f[21:]  # the same take draws
    js = json.loads(dm.model_json(spec, "femto-nfov.json"))
    assert js["sha256"] == spec.sha256 and js["spec"] == doc()
    assert dm.streams(spec, ["view0"]) == ["third-left.depth_femto", "third-right.depth_femto"]
    same = dm.load(os.path.join(MODELS, "femto-same.json"), cameras())
    assert dm.streams(same, ["view0", "view1"])[-2:] == ["view0.depth_femto", "view1.depth_femto"]


def test_the_session_format_documents_fm():
    with open(os.path.join(REPO, "trainer", "contract", "SESSION_FORMAT.md"), encoding="utf-8") as f:
        text = f.read()
    assert "| `FM` |" in text and "depth_femto" in text and "depth_model.json" in text


# --- our own calibration -----------------------------------------------------------------------------------------------------

def test_a_femto_record_calibration_replaces_the_camera_and_extrinsic():
    cal = {"serial": "CL8K14101J3", "label": "left",
           "streams": {"depth": {"width": 640, "height": 576, "fps": 30, "intrinsic": {"fx": 504.1, "fy": 504.3, "cx": 321.2, "cy": 331.9},
                                 "distortion": {"k1": 0.4, "k2": 0.07, "k3": 0.004, "k4": 0.77, "k5": 0.15, "k6": 0.02, "p1": 6e-05,
                                                "p2": -5e-05, "model": 4}}},
           "extrinsics": {"depth_to_color": {"rot": [1, 0, 0, 0, 0.9945, 0.1045, 0, -0.1045, 0.9945], "trans": [-32.1, -1.9, 3.8]}}}
    out = dm.from_calibration(doc(), cal)
    s = dm.parse(out)
    assert (s.mode.fx, s.mode.cy) == (504.1, 331.9) and s.mode.distortion[3] == 0.77
    assert s.translation == pytest.approx((-0.0321, -0.0019, 0.0038)) and s.rotation[1][2] == 0.1045
    assert out["edges"] == doc()["edges"] and out["illumination"] == doc()["illumination"]
    bad = copy.deepcopy(cal)
    bad["streams"]["depth"]["distortion"]["model"] = 5
    with pytest.raises(ValueError, match="distortion model 5"):
        dm.from_calibration(doc(), bad)


# --- the statistics tool -------------------------------------------------------------------------------------------------------

def test_the_statistics_see_the_models_edges(spec):
    import depth_stats

    s = constant(spec, rho=0.6, dropout=0.0)
    z = plane_depth(s, 1.0)
    z[:, 330:] -= 0.2  # a 200 mm step
    d = dm.apply(s, z, frame(s))
    e = depth_stats.edge_summary(depth_stats.edge_transitions(d))
    assert e["byJump"]["100-300"]["n"] > 300
    exact = depth_stats.edge_summary(depth_stats.edge_transitions(np.round(z * 1000).astype(np.uint16)))
    assert exact["byJump"]["100-300"]["meanGap"] < e["byJump"]["100-300"]["meanGap"]
    V = depth_stats.lag_variances(np.random.default_rng(0).normal(size=(300, 300)), np.ones((300, 300), bool))
    fit = depth_stats.noise_from_lags(V)
    assert fit["sigmaMm"] == pytest.approx(math.sqrt(1 - 1 / 12), rel=0.05) and abs(fit["rho"][1]) < 0.05


def test_a_modelled_table_lands_back_on_the_table_through_the_pose(spec):
    """End to end: a table top at z -0.196 ray-cast in the render pinhole from a camera at the twin's third-left place,
    resampled onto the raw grid, modelled, and carried back through the pose and the raw rays to the base frame, lies at
    the table's height (the conventions of the render, the raw grid, the USD pose and depth_stats agree)."""
    import depth_stats

    s = constant(spec, rho=0.6)
    g = dm.geometry(s.mode)
    pos, target = (0.88, 0.51, 0.32), (0.08, 0.0, 0.07)
    intr = {"fx": g.render_f, "fy": g.render_f, "cx": g.render_width / 2, "cy": g.render_height / 2}
    z, _ = depth_stats.synthetic_depth(intr, (g.render_height, g.render_width), pos, target, table_z=-0.196)
    d = dm.finish(dm.Job(s, frame(s), z, None, raw=True))
    pose = (*pos, *renderer.look_at(pos, target))
    got = depth_stats.plane_in_base(d, (g.x, g.y), pose)
    assert got["points"] > 100000 and got["medianZ"] == pytest.approx(-0.196, abs=0.004) and got["p99AbsMm"] < 15
    lit = depth_stats.lit_polygon(d > 0, (g.x, g.y))
    assert 5 <= len(lit) <= 12 and max(p[1] for p in lit) > 0.5  # the table fills the lower view, cut by the hexagon
