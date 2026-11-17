"""Flat clutter, offline (docs/adr/0037, "Surfaces and flat clutter"): the scene's section and what it refuses, the seeded
draw, the 6 mm bound, where each kind may lie, the meshes and the CL record."""

import dataclasses
import math
import os
import time

import pytest

from conftest import REPO
from teleop_isaac import clutter
from teleop_isaac import scene as scene_module
from teleop_isaac.isaac_session import field
from teleop_isaac.renderer import MASK_IDS

BAND = clutter.Rect.bounds(0.12, 0.44, -0.24, 0.24)  # the placement region: where the movable objects stand


def _bench():
    return scene_module.load(os.path.join(REPO, "core", "Scenes", "jetrover-bench.json"))


def _with(s, **change):
    return dataclasses.replace(s, extra=dict(s.extra, clutter=dict(s.extra["clutter"], **change)))


def _kind(s, name, **change):
    return _with(s, kinds=[dict(k, **change) if k["kind"] == name else k for k in s.extra["clutter"]["kinds"]])


def test_the_benchs_clutter_is_papers_a_mat_cables_and_a_bag_none_taller_than_6_mm():
    s = clutter.spec(_bench())
    kinds = {k.kind: k for k in s.kinds}
    assert set(kinds) == {"a4", "sticky-note", "cutting-mat", "cable-blue", "cable-black", "cable-white", "cable-grey", "bag"}
    assert all(k.tallest <= clutter.MAX_HEIGHT for k in s.kinds)
    assert kinds["cutting-mat"].size == (0.42, 0.594) and kinds["cutting-mat"].thickness == 0.002  # A2, 2 mm thick
    assert kinds["a4"].size == (0.21, 0.297) and kinds["a4"].texture == kinds["sticky-note"].texture == "teleop_white_paper"
    assert all(0.0035 <= k.diameter[0] <= k.diameter[1] <= 0.005 for k in s.kinds if k.shape == "cable")
    assert kinds["bag"].height <= 0.005 and kinds["bag"].roughness < 0.3  # flat and glossy
    # Cables and the bag keep out of the band where the movable objects stand; sheets may lie under them.
    for k in s.kinds:
        covers = any(r.x - r.hw <= BAND.x - BAND.hw and r.x + r.hw >= BAND.x + BAND.hw and r.y - r.hl <= BAND.y - BAND.hl
                     and r.y + r.hl >= BAND.y + BAND.hl for r in k.keep_out)
        assert covers == (k.shape != "sheet"), k.kind
    assert MASK_IDS["clutter"] == 15 and len(set(MASK_IDS.values())) == len(MASK_IDS)


def test_a_clutter_section_is_refused_when_anything_could_stand_taller_than_6_mm_or_is_malformed():
    s = _bench()
    for t, word in (
            (_kind(s, "a4", stackMax=61), "thicker than 6 mm"),  # 61 sheets of 0.1 mm
            (_kind(s, "cutting-mat", thickness=0.0065), "thicker than 6 mm"),
            (_kind(s, "cable-blue", diameterMax=0.007), "thicker than 6 mm"),
            (_kind(s, "bag", height=0.008), "thicker than 6 mm"),
            (_kind(s, "a4", shape="box"), "shape"),
            (_kind(s, "a4", texture="nope"), "nope"),
            (_kind(s, "a4", stackMin=3, stackMax=2), "stackMin"),
            (_kind(s, "a4", weight=0), "weight"),
            (_kind(s, "a4", colours=[1.0, 1.0]), "colours"),
            (_kind(s, "cable-blue", colours=[0.1, 0.1, 1.2]), "colours"),
            (_kind(s, "cable-blue", lengthMin=2.0), "min above its max"),
            (_kind(s, "cable-blue", fan=0.3), "no field fan"),
            (_kind(s, "bag", sizeJitter=0.7), "sizeJitter"),
            (_kind(s, "a4", keepOut=[0.1, 0.0, 0.0, 0.1]), "min < max"),
            (_kind(s, "a4", kind="A4 Paper"), "lowercase"),
            (_with(s, kinds=s.extra["clutter"]["kinds"] + s.extra["clutter"]["kinds"][:1]), "two are called"),
            (_with(s, kinds=[]), "kinds"),
            (_with(s, countMin=5, countMax=2), "countMin"),
            (_with(s, region=[0.0, 0.6, -0.8, 0.78]), "on the table"),  # it would start on the bench
            (_with(s, keepOut=[0.1, 0.2, 0.3]), "keepOut"),
            (_with(s, colour=[1, 0, 0]), "no field colour")):
        with pytest.raises(ValueError, match=word):
            clutter.spec(t)
    assert clutter.spec(dataclasses.replace(s, extra={k: v for k, v in s.extra.items() if k != "clutter"})) is None


def test_the_draw_is_the_seed_and_the_change_alone():
    s = clutter.spec(_bench())
    assert clutter.draw(s, 7, 3) == clutter.draw(s, 7, 3)
    assert len({tuple((p.kind, round(p.x, 6)) for p in clutter.draw(s, 7, k)) for k in range(30)}) >= 25
    assert [clutter.draw(s, 8, k) for k in range(5)] != [clutter.draw(s, 7, k) for k in range(5)]
    counts = [len(clutter.draw(s, 7, k)) for k in range(60)]
    assert 0 in counts and max(counts) >= 10  # a clear table now and then, and a cluttered one


def test_kinds_whose_texture_is_not_fetched_are_left_out():
    s = clutter.spec(_bench())
    assert {k.kind for k in clutter.usable(s, ["teleop_white_paper"]).kinds} == {k.kind for k in s.kinds} - {"cutting-mat"}
    assert {k.kind for k in clutter.usable(s, []).kinds} == {k.kind for k in s.kinds if k.shape != "sheet"}
    assert clutter.usable(None, []) is None


def test_every_piece_lies_on_the_table_clear_of_the_robot_and_within_6_mm_cables_and_bags_off_the_band():
    s = clutter.spec(_bench())
    robot = s.keep_out[0]
    seen, sheets_under_objects = set(), 0
    turn = max(clutter.CABLE_TURNS) + clutter.CURVATURE_MAX * clutter.CABLE_STEP
    for change in range(120):
        pieces = clutter.draw(s, 7, change)
        bags = [p.rect for p in pieces if p.shape == "bag"]
        for p in pieces:
            seen.add(p.kind)
            assert 0.0 <= p.base < p.top <= clutter.MAX_HEIGHT + 1e-12, p
            if p.shape != "cable":
                r = p.rect
                assert clutter.inside(r, s.region) and not clutter.overlap(r, robot), p
                if p.shape == "bag":
                    assert not clutter.overlap(r, BAND) and all(0 < h <= 0.005 + 1e-12 for row in p.grid for h in row)
                elif clutter.overlap(r, BAND):
                    sheets_under_objects += 1
                if p.kind == "cutting-mat":
                    assert abs(p.yaw) <= 0.05 + 1e-12  # square to the table
                continue
            d = p.size[0]
            assert 0.0035 <= d <= 0.005 and 0.35 - 1e-9 <= p.size[1] <= 1.1 + 1e-9
            pts = [q[:2] for q in p.path]
            assert all(abs(math.dist(a, b) - clutter.CABLE_STEP) < 1e-9 for a, b in zip(pts, pts[1:]))
            x0, x1, y0, y1 = s.region
            for x, y in pts:
                assert x0 + d / 2 - 1e-9 <= x <= x1 - d / 2 + 1e-9 and y0 + d / 2 - 1e-9 <= y <= y1 - d / 2 + 1e-9
                assert not robot.contains(x, y, d / 2) and not BAND.contains(x, y, d / 2)
                assert not any(b.contains(x, y, d / 2) for b in bags)
            assert all(d / 2 <= z <= clutter.MAX_HEIGHT - d / 2 + 1e-12 for _, _, z in p.path)  # lying on what is under it
            heading = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])]
            assert all(abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi) <= turn + 1e-9 for h0, h1 in zip(heading, heading[1:]))
        cables = [p for p in pieces if p.shape == "cable"]
        for i, a in enumerate(cables):  # no two touch: a crossing would stand taller than 6 mm
            for b in cables[i + 1:]:
                gap = min(clutter.segment_distance(p0[:2], p1[:2], q0[:2], q1[:2])
                          for p0, p1 in zip(a.path, a.path[1:]) for q0, q1 in zip(b.path, b.path[1:]))
                assert gap >= (a.size[0] + b.size[0]) / 2
    assert seen == {k.kind for k in s.kinds} and sheets_under_objects > 0


def test_each_piece_is_a_mesh_on_the_table_within_6_mm_a_sheet_textured_at_its_real_size():
    scene = _bench()
    top = scene.table.top
    s = clutter.spec(scene)
    shapes = set()
    for change in range(40):
        for p in clutter.draw(s, 5, change):
            m = clutter.mesh(p, top)
            shapes.add(p.shape)
            assert sum(m.counts) == len(m.indices) and min(m.indices) >= 0 and max(m.indices) < len(m.points)
            assert len(m.normals) == len(m.points) and all(abs(math.sqrt(sum(c * c for c in v)) - 1.0) < 1e-6 for v in m.normals)
            assert all(top - 1e-9 <= z <= top + clutter.MAX_HEIGHT + 1e-9 for _, _, z in m.points)
            assert bool(m.st) == bool(p.texture) and (not m.st or len(m.st) == len(m.points))
            if p.shape == "sheet":
                assert max(u for u, _ in m.st) == pytest.approx(p.size[0]) and max(v for _, v in m.st) == pytest.approx(p.size[1])
                assert max(z for _, _, z in m.points) == pytest.approx(top + p.top)
            if p.shape == "cable":  # every ring point the tube's radius from its centreline
                x, y, z = p.path[0]
                assert all(abs(math.dist(q, (x, y, top + z)) - p.size[0] / 2) < 1e-9 for q in m.points[:clutter.CABLE_SIDES])
    assert shapes == {"sheet", "cable", "bag"}


def test_cl_records_each_piece_as_kind_x_y_yaw_in_plain_ascii():
    s = clutter.spec(_bench())
    pieces = next(d for d in (clutter.draw(s, 7, k) for k in range(50)) if len(d) >= 3)
    items = clutter.cl_items(pieces)
    assert [i[0] for i in items] == [p.kind for p in pieces]
    text = field(items)
    assert text.isascii() and "|" not in text and len(text.split(";")) == len(pieces)
    for part in text.split(";"):
        kind, x, y, yaw = part.split(":")
        assert clutter.KIND_ID.match(kind) and 0 <= float(yaw) < round(2 * math.pi, 4)


def test_drawing_a_take_is_quick():
    s = clutter.spec(_bench())
    start = time.perf_counter()
    for change in range(50):
        for p in clutter.draw(s, 3, change):
            clutter.mesh(p, 0.0)
    assert time.perf_counter() - start < 20.0  # about 6 ms a take on a laptop; the renderer draws once a take, between takes
