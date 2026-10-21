from __future__ import annotations

from armpred.net.rng import SeededRng, derive_for_axis, stream_id_from_name

from .conftest import load


def test_seeded_rng_streams_match_core():
    for stream in load("rng-v1.json")["streams"]:
        rng = SeededRng(int(stream["seed"]))
        assert [rng.next_uint64() for _ in range(16)] == [int(v) for v in stream["uint64"]], stream["seed"]
        assert [rng.next_double() for _ in range(16)] == stream["double"], stream["seed"]


def test_reset_replays_the_stream():
    rng = SeededRng(42)
    first = [rng.next_uint64() for _ in range(5)]
    rng.reset()
    assert [rng.next_uint64() for _ in range(5)] == first


def test_stream_ids_and_substream_seeds_match_core():
    data = load("rng-v1.json")
    for axis, value in data["streamIds"].items():
        assert stream_id_from_name(axis) == int(value), axis
    for d in data["derive"]:
        assert derive_for_axis(int(d["seed"]), d["axis"]) == int(d["value"]), (d["seed"], d["axis"])
