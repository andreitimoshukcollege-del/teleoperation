"""The rooms' rules, offline (docs/adr/0037 decision 2): the Radiance reader, the luminance it is normalised by, what a
download must be, and the seeded pick."""

import hashlib
import json
import math

import numpy as np
import pytest

from teleop_isaac import environments as env


def panorama(h=16, seed=0):
    rng = np.random.default_rng(seed)
    return (rng.random((h, 2 * h, 3)) * 4.0 + 0.01).astype(np.float32)


def rle_rgbe(rgb):
    """New-style run-length scanlines, as Poly Haven's files are written."""
    flat = env.write_rgbe_flat(rgb)
    head_end = flat.index(b"\n-Y ") + 1
    head_end = flat.index(b"\n", head_end) + 1
    h, w, _ = rgb.shape
    px = np.frombuffer(flat[head_end:], np.uint8).reshape(h, w, 4)
    out = bytearray(flat[:head_end])
    for y in range(h):
        out += bytes([2, 2, w >> 8, w & 255])
        for c in range(4):
            row = px[y, :, c].tolist()
            x = 0
            while x < w:
                n = 1
                while x + n < w and row[x + n] == row[x] and n < 127:
                    n += 1
                if n >= 3:
                    out += bytes([128 + n, row[x]])
                    x += n
                    continue
                start = x
                while x < w and x - start < 128 and not (x + 2 < w and row[x] == row[x + 1] == row[x + 2]):
                    x += 1
                out += bytes([x - start]) + bytes(row[start:x])
    return bytes(out)


def test_a_flat_file_reads_back_to_within_rgbe_precision():
    rgb = panorama()
    back = env.read_rgbe(env.write_rgbe_flat(rgb))
    assert back.shape == rgb.shape
    # RGBE keeps 8 bits of mantissa for the brightest channel: every channel is exact to 1/128 of the pixel's peak.
    peak = rgb.max(axis=2, keepdims=True)
    assert (np.abs(back - rgb) <= peak / 128).all()


def test_a_run_length_file_reads_the_same_as_the_flat_one():
    rgb = panorama(h=24, seed=3)
    rgb[5, 3:30] = 1.0  # a run in every channel
    flat = env.read_rgbe(env.write_rgbe_flat(rgb))
    rle = env.read_rgbe(rle_rgbe(rgb))
    assert np.array_equal(flat, rle)


@pytest.mark.parametrize("bad", [
    b"<html>nope</html>",
    b"#?RADIANCE\nFORMAT=32-bit_rle_xyze\n\n-Y 2 +X 4\n" + b"\x00" * 32,
    b"#?RADIANCE\n\n+Y 2 +X 4\n" + b"\x00" * 32,      # flipped orientation
    b"#?RADIANCE\n\n-Y 2 +X 4\n" + b"\x00" * 31,      # one byte short
    b"#?RADIANCE\n\n-Y 99999 +X 4\n",
])
def test_anything_but_a_plain_radiance_file_is_refused(bad):
    with pytest.raises(ValueError):
        env.read_rgbe(bad)


def test_a_run_past_the_end_of_its_scanline_is_refused():
    head = b"#?RADIANCE\n\n-Y 1 +X 8\n"
    with pytest.raises(ValueError):
        env.read_rgbe(head + bytes([2, 2, 0, 8]) + bytes([128 + 9, 7]) + b"\x00" * 40)


def test_the_mean_luminance_weights_each_row_by_its_share_of_the_sphere():
    h = 64
    rgb = np.zeros((h, 2 * h, 3), np.float32)
    rgb[: h // 2] = 1.0  # the upper hemisphere white, the lower black: half the sphere
    assert env.mean_luminance(rgb) == pytest.approx(0.5, abs=1e-3)
    rgb[:] = 0
    rgb[:2] = 1.0  # a thin band at the pole: 2 of 64 rows, but far less of the sphere
    assert env.mean_luminance(rgb) < 0.1 * 2 / 64


def test_a_download_must_match_its_size_and_md5_and_be_a_two_to_one_panorama():
    data = env.write_rgbe_flat(panorama(h=64))
    md5 = hashlib.md5(data).hexdigest()
    w, h, lum = env.check_hdri(data, len(data), md5)
    assert (w, h) == (128, 64) and lum > 0
    assert env.check_hdri(data, len(data) + 1, md5) is None
    assert env.check_hdri(data, len(data), "0" * 32) is None
    square = env.write_rgbe_flat(np.ones((64, 64, 3), np.float32))
    assert env.check_hdri(square) is None
    black = env.write_rgbe_flat(np.zeros((64, 128, 3), np.float32))
    assert env.check_hdri(black) is None  # nothing to normalise by


def test_only_https_from_poly_havens_download_host_is_fetched():
    assert env.download_url_ok("https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/a_2k.hdr")
    assert not env.download_url_ok("http://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/a_2k.hdr")
    assert not env.download_url_ok("https://dl.polyhaven.org.evil.example/a_2k.hdr")
    assert not env.download_url_ok("https://dl.polyhaven.org/file/../../a_2k.hdr")
    assert not env.download_url_ok("https://dl.polyhaven.org/file/a_2k.exe")


def test_fetch_keeps_what_checks_out_under_its_own_sha_and_resumes(tmp_path):
    good = env.write_rgbe_flat(panorama(h=64, seed=1))
    other = env.write_rgbe_flat(panorama(h=64, seed=2))
    url = "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/{}_2k.hdr"
    files = {
        "room_a": (good, hashlib.md5(good).hexdigest()),
        "room_b": (other, "f" * 32),                       # md5 does not match: refused
        "room_c": (other, hashlib.md5(other).hexdigest()),
    }
    assets = {k: {"name": k.title(), "authors": {"Someone": "All"}, "categories": ["indoor"]} for k in files}
    assets["../etc"] = {"name": "x"}                       # not an id: never asked for
    asked = []

    def download(u, limit):
        asked.append(u)
        if u.endswith("/assets?t=hdris&c=indoor"):
            return json.dumps(assets).encode()
        if "/files/" in u:
            k = u.rsplit("/", 1)[1]
            data, md5 = files[k]
            return json.dumps({"hdri": {"2k": {"hdr": {"url": url.format(k), "size": len(data), "md5": md5}}}}).encode()
        k = u.rsplit("/", 1)[1][: -len("_2k.hdr")]
        return files[k][0]

    m = env.fetch(str(tmp_path), download=download, say=lambda *_: None)
    assert [e["id"] for e in m["hdris"]] == ["room_a", "room_c"]
    assert not any("etc" in u for u in asked)
    e = m["hdris"][0]
    assert e["file"] == f"hdri/{hashlib.sha256(good).hexdigest()}.hdr" and e["license"] == env.CC0
    assert e["authors"] == ["Someone"] and e["landingUrl"] == "https://polyhaven.com/a/room_a"
    assert (tmp_path / e["file"]).read_bytes() == good
    s = env.status(str(tmp_path))
    assert s["hdris"] == 2 and not s["bad"]
    asked.clear()
    env.fetch(str(tmp_path), download=download, say=lambda *_: None)
    assert not any("room_a" in u or "room_c" in u for u in asked)  # kept ones are not fetched again
    (tmp_path / e["file"]).write_bytes(b"tampered")
    assert env.status(str(tmp_path))["bad"] == [e["file"]]


def test_the_pick_is_drawn_from_the_seed_and_change_alone_and_never_repeats_the_last_room():
    hdris = [{"id": str(i), "meanLuminance": 1.0} for i in range(5)]
    a = env.pick(hdris, 7, 3)
    assert a == env.pick(hdris, 7, 3)
    assert env.pick(hdris, 8, 3) != a or env.pick(hdris, 9, 3) != a
    rooms = [env.pick(hdris, 7, k)[0]["id"] for k in range(200)]
    assert all(x != y for x, y in zip(rooms, rooms[1:]))
    assert len(set(rooms)) == 5
    yaws = [env.pick(hdris, 7, k)[1] for k in range(200)]
    assert 0.0 <= min(yaws) and max(yaws) <= 360.0 and max(yaws) - min(yaws) > 300
    assert env.pick(hdris[:1], 7, 4)[0]["id"] == "0"
    two = [env.pick(hdris[:2], 7, k)[0]["id"] for k in range(9)]
    assert all(x != y for x, y in zip(two, two[1:]))
    for k in range(0, 200, 5):  # each round shows every room once
        assert sorted(rooms[k:k + 5]) == ["0", "1", "2", "3", "4"]


def test_the_dome_is_dimmed_for_a_bright_room_and_raised_for_a_dark_one():
    assert env.dome_intensity(1000.0, {"meanLuminance": 2.0}) == pytest.approx(500.0)
    assert env.dome_intensity(1000.0, {"meanLuminance": 0.25}) == pytest.approx(4000.0)
    assert math.isfinite(env.dome_intensity(1000.0, {"meanLuminance": 1e-3}))
