"""shape_survey.py's geometry without pxr (docs/adr/0037 amendment 5, third revision): sections from thin layers, slabs cut
to keep the excess small, polygons convex and outer, and the JSON it prints."""

import json
import math

import numpy as np

import shape_survey as ss


def box_mesh(sx, sy, sz):
    """A box centred on the origin: 8 corners, 12 triangles."""
    V = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)], float)
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def cone_mesh(radius, height, segments=48):
    """A cone standing on its base, centred on the origin: its radius falls from ``radius`` to 0 over ``height``."""
    ring = [[radius * math.cos(2 * math.pi * i / segments), radius * math.sin(2 * math.pi * i / segments), -height / 2]
            for i in range(segments)]
    V = np.array(ring + [[0, 0, height / 2], [0, 0, -height / 2]], float)
    apex, base = segments, segments + 1
    F = np.array([[i, (i + 1) % segments, apex] for i in range(segments)] + [[(i + 1) % segments, i, base] for i in range(segments)])
    return V, F


def inside(poly, p, tol=1e-9):
    """Whether point p is inside (or on) the convex counter-clockwise polygon."""
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -tol:
            return False
    return True


def test_a_box_is_its_rectangle_in_the_fewest_slabs():
    V, F = box_mesh(0.10, 0.06, 0.04)
    planes, sections = ss.layer_sections(V, F, np.array([0.10, 0.06, 0.04]), 64)
    # A layer between the corners has no vertex in it, only the edges' crossings of its planes: never empty.
    assert all(len(s) >= 3 for s in sections)
    groups, excess = ss.slabs_of(planes, sections, ss.FLOOR_BAND_M)
    assert len(groups) == ss.MIN_SLABS and excess < 1e-9
    assert [i for g in groups for i in g] == list(range(64))  # every layer once, bottom to top
    for g in groups:
        poly = ss.hull(np.concatenate([sections[i] for i in g]))
        assert len(poly) == 4
        assert np.allclose(np.abs(poly), [0.05, 0.03])


def test_a_cone_is_cut_until_no_slab_stands_off_it_by_more_than_the_goal():
    V, F = cone_mesh(0.05, 0.10)
    planes, sections = ss.layer_sections(V, F, np.array([0.10, 0.10, 0.10]), 128)
    groups, excess = ss.slabs_of(planes, sections, ss.FLOOR_BAND_M)
    assert excess <= ss.EXCESS_GOAL_M + ss.EXCESS_SLACK_M
    assert ss.MIN_SLABS < len(groups) <= ss.MAX_SLABS  # a cone narrows all the way up: more than the fewest
    # The bottom FLOOR_BAND_M is one slab of its own.
    assert planes[groups[0][-1] + 1] - planes[0] <= ss.FLOOR_BAND_M + 1e-9 < planes[groups[1][-1] + 1] - planes[0]
    # Each slab's polygon holds the cone's every section in it: the prism is outside the mesh, never inside.
    for g in groups:
        poly = ss.hull(np.concatenate([sections[i] for i in g]))
        for i in g:
            assert all(inside(poly, p) for p in sections[i])


def test_hull_is_counter_clockwise_and_cutting_corners_only_shrinks():
    pts = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5], [0.2, 0.7], [1, 0.5]], float)
    h = ss.hull(pts)
    assert len(h) == 4  # the interior and collinear points are dropped
    area = sum(h[i][0] * h[(i + 1) % 4][1] - h[(i + 1) % 4][0] * h[i][1] for i in range(4))
    assert area > 0  # counter-clockwise
    circle = np.array([[math.cos(a), math.sin(a)] for a in np.linspace(0, 2 * math.pi, 96, endpoint=False)]) * 0.1
    cut = ss.cut_corners(ss.hull(circle), 0.0003, ss.MAX_CORNERS)
    assert len(cut) <= ss.MAX_CORNERS
    assert all(inside(ss.hull(circle), p, 1e-12) for p in cut)  # its corners are the circle's own: it only shrinks


def test_the_json_is_one_slab_a_line_and_reads_back():
    out = {"format": "teleop-object-shapes", "version": 1, "source": "a test",
           "assets": {"lib/b": {"size": [0.1, 0.2, 0.3], "excessMm": 0.5,
                                "slabs": [{"z0": -0.5, "z1": 0.0, "outline": [[-0.5, -0.5], [0.5, -0.5], [0.0, 0.5]]},
                                          {"z0": 0.0, "z1": 0.5, "outline": [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5]]}]},
                      "lib/a": {"size": [0.1, 0.1, 0.1], "excessMm": 0.0,
                                "slabs": [{"z0": -0.5, "z1": 0.5, "outline": [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]}]}}}
    text = ss.dumps(out)
    assert json.loads(text) == out
    assert sum(1 for line in text.splitlines() if '"z0"' in line) == 3
    assert ss.fraction(-0.00001, 1.0) == 0.0 and str(ss.fraction(-0.00001, 1.0)) == "0.0"  # never -0.0
