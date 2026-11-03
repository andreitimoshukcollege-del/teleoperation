"""The renderer's session writer (teleop_isaac/isaac_session.py): it follows the control file, writes isaac.jlog and the
frame sidecars, and closes each session with FS lines and a trailer that hash what is on disk."""

import hashlib
import os
import time

from teleop_isaac.isaac_session import SIDECAR_HEADER, IsaacSession, read_sidecar


def wait_for(cond, seconds=5.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def lines(path):
    with open(path, encoding="ascii") as f:
        return f.read().splitlines()


def start(tmp_path, **kw):
    control = tmp_path / ".active"
    closed = []
    s = IsaacSession(str(control), ["head.rgb", "head.depth"], preamble=lambda: [("IV", "test", 1)],
                     on_closed=closed.append, poll_seconds=0.02, free_bytes=lambda d: 10 ** 12, **kw)
    return s, control, closed


def test_a_session_has_its_records_its_frames_and_a_trailer_that_hashes_them(tmp_path):
    s, control, closed = start(tmp_path)
    d = tmp_path / "20261103T000000Z-auto"
    try:
        assert not s.active
        control.write_text(str(d))
        assert wait_for(lambda: s.active)
        s.record("CM", "head", 465.6)
        assert s.frame(1, 1000, "", {"head.rgb": b"\xff\xd8one", "head.depth": b"png1"})
        assert s.frame(2, 2000, "B", {"head.rgb": b"\xff\xd8two"})
        time.sleep(0.2)
        control.unlink()
        assert wait_for(lambda: closed == [str(d)])
    finally:
        s.close()

    log = lines(d / "isaac.jlog")
    assert log[0] == f"JSES|1|isaac|1000000000|CLOCK_MONOTONIC|{d.name}"
    assert log[1] == "IV|test|1"
    tags = [x.split("|")[0] for x in log]
    assert tags.count("VF") == 2 and tags.count("FS") == 2 and tags[-1] == "END"
    vf = [x.split("|") for x in log if x.startswith("VF|")]
    assert vf[1][2:5] == ["2", "2000", "B"]
    rgb = dict((off, data) for off, data in read_sidecar(str(d / "isaac" / "head.rgb.frames")))
    for name, off, length in (part.split(":") for part in vf[0][5].split(";")):
        side = read_sidecar(str(d / "isaac" / (name + ".frames")))
        assert dict(side)[int(off)] == (b"\xff\xd8one" if name == "head.rgb" else b"png1")
        assert len(dict(side)[int(off)]) == int(length)
    assert list(rgb.values()) == [b"\xff\xd8one", b"\xff\xd8two"]
    for fs in (x.split("|") for x in log if x.startswith("FS|")):
        data = (d / fs[1]).read_bytes()
        assert data.startswith(SIDECAR_HEADER) and int(fs[2]) == len(data) and fs[3] == hashlib.sha256(data).hexdigest()
    end = log[-1].split("|")
    body = ("\n".join(log[:-1]) + "\n").encode("ascii")
    assert end[1] == str(len(log) - 2) and end[2] == "0" and end[3] == hashlib.sha256(body).hexdigest()


def test_a_second_writer_in_the_same_session_never_truncates_the_first(tmp_path):
    d = tmp_path / "s"
    for _ in range(2):
        s, control, closed = start(tmp_path)
        try:
            control.write_text(str(d))
            assert wait_for(lambda: s.active)
            s.frame(1, 5, "", {"head.rgb": b"x"})
            time.sleep(0.1)
            control.unlink()
            assert wait_for(lambda: len(closed) == 1)
        finally:
            s.close()
    assert (d / "isaac.jlog").exists() and (d / "isaac.2.jlog").exists()
    assert (d / "isaac" / "head.rgb.frames").exists() and (d / "isaac" / "head.rgb.2.frames").exists()
    assert lines(d / "isaac.jlog")[-1].startswith("END|")


def test_a_nearly_full_disk_ends_the_session_and_it_is_not_reopened(tmp_path):
    free = [10 ** 12]
    control = tmp_path / ".active"
    closed = []
    s = IsaacSession(str(control), ["head.rgb"], on_closed=closed.append, poll_seconds=0.02, free_bytes=lambda d: free[0],
                     min_free_bytes=10 ** 9)
    d = tmp_path / "s"
    try:
        control.write_text(str(d))
        assert wait_for(lambda: s.active)
        free[0] = 10 ** 6
        assert wait_for(lambda: closed == [str(d)])
        time.sleep(0.2)
        assert not s.active  # the control file still names it, but it is not reopened
    finally:
        s.close()
    log = lines(d / "isaac.jlog")
    assert any(x.startswith("ER|") and "GB free" in x for x in log) and log[-1].startswith("END|")


def test_nothing_is_written_without_a_session(tmp_path):
    s, control, closed = start(tmp_path)
    try:
        assert not s.frame(1, 1, "", {"head.rgb": b"x"})
        s.record("CM", 1)
    finally:
        s.close()
    assert not any(p.name.endswith(".jlog") for p in tmp_path.rglob("*"))
