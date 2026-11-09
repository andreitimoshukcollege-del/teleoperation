"""docs/adr/0035 N2's sources: each parser on a few lines of the real format (quoted from the source where it publishes
them; the FCC's is made up to its data dictionary's columns, as no real line was seen), and the conversions' rules."""

from __future__ import annotations

import io
import json
import struct
import zipfile

import numpy as np
import pytest

from armpred.net.datasets import fcc_mba, fetching, g1050, iso9660, mahimahi, puffer, ripe_atlas, synth
from armpred.net.datasets.tiering import assess

# G.1050 (07/2016) clause 6.6, Table 3 (Dw4, stream HD_cbr downstream), verbatim: header and the first rows.
G1050_OUT = """Source: TIA TR30.3: TIA-921C
Content-encoding: ASCII
Delay unit: ms
Time Delay Drop
14.5956974 77.32 0
14.5970787 77.47 0
14.5984581 77.08 0
14.5998375 77.28 0
14.6012169 76.95 0
14.6039776 76.95 1
14.6039776 77.25 0
14.6053569 76.91 0
"""
# Clause 6.6, Table 4 (residual unmanaged bandwidth and delay), verbatim: t Bandwidth Packet Mean m2.
G1050_BINS = """t Bandwith Packet Mean m2
1.4 485760 40 77.3 5976.6
1.6 728640 60 78.4 6147.6
1.8 1457280 120 80.0 6409.9
2.0 2064480 170 83.7 7027.3
2.2 2064480 170 82.2 6770.0
"""


def test_g1050_out_rows_are_datagrams_and_drops_are_losses():
    times, delays = g1050.parse_out(G1050_OUT)
    assert len(times) == 8 and delays[0] == 77.32 and delays[5] is None and delays[6] == 77.25


def test_g1050_bins_give_mean_and_spread_and_a_fifo_statistical_trace():
    bins = g1050.parse_bins(G1050_BINS)
    assert bins[0][0] == 1.4 and bins[0][1] == 77.3
    assert bins[0][2] == pytest.approx(np.sqrt(5976.6 - 77.3 ** 2))
    t = g1050.statistical_trace(bins, seed=7)
    assert len(t.delays) == 5 * 10 and t.interval_ms == 20.0 and assess(t).reorder_ratio == 0.0
    assert np.array_equal(t.delays, g1050.statistical_trace(bins, seed=7).delays)


def test_g1050_test_case_labels():
    m = g1050.CASE.match("Dw4")
    assert (m["tech"], m["profile"], m["severity"]) == ("D", "w", "4")
    assert g1050.CASE.match("Dp1t") and not g1050.CASE.match("Core")


def make_iso(files):
    """A minimal ISO 9660 image (primary volume only): {"DIR/NAME.OUT": bytes}."""
    sector = 2048
    blob = {}
    dirs = {"": []}
    for path in files:
        parts = path.split("/")
        for k in range(1, len(parts)):
            d = "/".join(parts[:k])
            if d not in dirs:
                dirs[d] = []
                dirs["/".join(parts[:k - 1])].append((parts[k - 1], d, True))
        dirs["/".join(parts[:-1])].append((parts[-1], path, False))
    lba = 18
    where = {}
    for d in dirs:
        where[d] = (lba, sector)
        lba += 1
    for p, data in files.items():
        n = max(1, -(-len(data) // sector))
        where[p] = (lba, len(data))
        blob[lba] = data
        lba += n

    def rec(name: bytes, extent: int, size: int, is_dir: bool) -> bytes:
        body = struct.pack("<B B I I I I 7s B B B H H B", 0, 0, extent, 0, size, 0, b"\0" * 7, 2 if is_dir else 0, 0, 0, 1, 0, len(name)) + name
        body = bytes([33 + len(name) + (len(name) + 1) % 2]) + body[1:]
        return body + (b"\0" if len(name) % 2 == 0 else b"")

    img = bytearray(lba * sector)
    for d, entries in dirs.items():
        ext, _ = where[d]
        recs = rec(b"\0", ext, sector, True) + rec(b"\1", ext, sector, True)
        for name, target, is_dir in entries:
            t_ext, t_size = where[target]
            recs += rec((name if is_dir else name + ";1").encode(), t_ext, t_size, is_dir)
        img[ext * sector:ext * sector + len(recs)] = recs
    for at, data in blob.items():
        img[at * sector:at * sector + len(data)] = data
    pvd = bytearray(sector)
    pvd[0:6] = b"\x01CD001"
    pvd[156:156 + 34] = rec(b"\0", where[""][0], sector, True)
    img[16 * sector:17 * sector] = pvd
    img[17 * sector:17 * sector + 6] = b"\xffCD001"
    return bytes(img)


def test_iso9660_finds_and_reads_files_by_path():
    img = make_iso({"D/BIDI/DW4/VOIP1_FWD.OUT": G1050_OUT.encode(), "README.TXT": b"hello"})
    f = io.BytesIO(img)
    entries = {e.path: e for e in iso9660.walk(f)}
    assert {"D", "D/BIDI", "D/BIDI/DW4", "D/BIDI/DW4/VOIP1_FWD.OUT", "README.TXT"} <= set(entries)
    assert iso9660.read(f, entries["README.TXT"], 100) == b"hello"
    with pytest.raises(ValueError):
        iso9660.read(f, entries["D/BIDI/DW4/VOIP1_FWD.OUT"], 10)


# Mahimahi traces/Verizon-LTE-short.down, its first lines verbatim.
MAHIMAHI_HEAD = "0\n0\n1\n1\n2\n2\n3\n3\n3\n3\n5\n5\n6\n6\n6\n"


def test_mahimahi_parse_and_a_fast_link_adds_almost_nothing():
    ops = mahimahi.parse(MAHIMAHI_HEAD)
    assert list(ops[:4]) == [0, 0, 1, 1]
    every_ms = np.arange(0, 60_000, dtype=np.int64)  # one 1500-byte packet a millisecond: 12 Mbit/s
    t = mahimahi.replay(every_ms)
    assert t.delays_ms().max() <= 1.0 and len(t.delays) == 3000  # sends at 0 .. 59 980 ms
    with pytest.raises(ValueError):
        mahimahi.parse("0\n5\n3\n")
    with pytest.raises(ValueError):
        mahimahi.parse("0\nrm -rf\n")


def test_mahimahi_an_outage_holds_the_stream_and_releases_it_together():
    ops = np.concatenate([np.arange(0, 10_000), np.arange(10_800, 30_000)]).astype(np.int64)  # 800 ms without capacity
    t = mahimahi.replay(ops)
    d = t.delays_ms()
    assert d.max() == pytest.approx(800.0, abs=1.0) and np.median(d) <= 1.0
    a = assess(t)
    assert a.reorder_ratio == 0.0 and a.worst_minute["ipdv_ms"] > 500 and a.tier == "very-bad"


def test_mahimahi_a_dead_zone_longer_than_3_s_is_loss_not_a_minute_of_delay():
    ops = np.concatenate([np.arange(0, 10_000), np.arange(20_000, 40_000)]).astype(np.int64)  # 10 s without capacity
    t = mahimahi.replay(ops)
    d = t.delays_ms()
    assert np.nanmax(d) <= 3000.0 + 1.0 and t.lost.sum() >= 340  # ~7 s of datagrams (350) dropped by age
    a = assess(t)
    assert a.burst_count == 1 and a.longest_burst_ms >= 6800
    assert a.g1050_profile == "beyond-C" and a.tier == "very-bad"  # the 3 s queued before the drops: IPDV past C's 500 ms


# Puffer's puffer-fake-sample.tar.gz, video_sent (2019 format), verbatim.
PUFFER = """time (ns GMT),session_id,index,expt_id,channel,video_ts,format,size,ssim_index,cwnd,in_flight,min_rtt,rtt,delivery_rate
1548500400788000000,kaXlFjXCNjAgx4nsvYsDetENDzv04n/e7R4HYutFWt0=,0,232,abc,38279961720,1280x720-20,579932,0.981149,294,0,75936,109157,669806
1548500401993000000,j1E2fJfiOe5TU3Fj3lC1cgA4CsKjg2NJYBcxdVcdJfk=,0,237,abc,38280322080,1280x720-20,269778,0.986832,846,0,53162,70263,10908953
1548500402853000000,kaXlFjXCNjAgx4nsvYsDetENDzv04n/e7R4HYutFWt0=,0,232,abc,38280141900,1280x720-20,475807,0.983754,295,0,75936,104773,2900000
1548500402957000000,j1E2fJfiOe5TU3Fj3lC1cgA4CsKjg2NJYBcxdVcdJfk=,0,237,abc,38280502260,1280x720-20,322657,0.986481,846,0,53162,72590,10908953
"""


def test_puffer_streams_by_session_and_index_in_ms_and_a_level_trace():
    s = puffer.streams(io.StringIO(PUFFER))
    assert len(s) == 2
    rows = next(v for v in s.values() if v[0][1] == pytest.approx(109.157))
    assert rows[1] == (pytest.approx(1548500402.853), pytest.approx(104.773), pytest.approx(75.936))
    t = synth.level_trace([0.0, 2.0, 4.0], [50.0, 100.0, 50.0])
    d = t.delays_ms()
    assert d[0] == 50.0 and d[100] == pytest.approx(100.0) and len(d) == 200 and assess(t).reorder_ratio == 0.0
    with pytest.raises(ValueError):
        puffer.streams(io.StringIO("time (ns GMT),session_id\n1,2\n"))


# Made up to the MBA data dictionary's columns (curr_udplatency: microseconds); not a real line.
FCC = """unit_id,dtime,ddate,target,rtt_avg,rtt_min,rtt_max,rtt_std,successes,failures
447,2023-07-03 14:00:00,2023-07-03,sp1.example,23000,18000,61000,4000,1990,10
447,2023-07-03 15:00:00,2023-07-03,sp1.example,0,0,0,0,0,0
"""


def test_fcc_rows_become_one_way_summaries_and_nonsense_is_skipped():
    rows = list(fcc_mba.rows_of(io.StringIO(FCC)))
    assert fcc_mba.summary_of(rows[0]) == (9.0, 11.5, 2.0, 10 / 2000)
    assert fcc_mba.summary_of(rows[1]) is None
    with pytest.raises(ValueError):
        list(fcc_mba.rows_of(io.StringIO("unit_id,rtt_avg\n1,2\n")))


def test_fcc_unit_profile_xlsx_is_read_without_libraries():
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("xl/sharedStrings.xml", '<sst xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
                   '<si><t>Unit ID</t></si><si><t>ISP</t></si><si><t>Technology</t></si><si><t>Verizon</t></si><si><t>Fiber</t></si></sst>')
        z.writestr("xl/worksheets/sheet1.xml", '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>'
                   '<row r="1"><c r="A1" t="s"><v>0</v></c><c r="B1" t="s"><v>1</v></c><c r="C1" t="s"><v>2</v></c></row>'
                   '<row r="2"><c r="A2"><v>447</v></c><c r="B2" t="s"><v>3</v></c><c r="C2" t="s"><v>4</v></c></row>'
                   '</sheetData></worksheet>')
    assert fcc_mba.unit_profiles(buf.getvalue()) == {"447": {"isp": "Verizon", "technology": "Fiber", "state": ""}}


# RIPE Atlas, measurement 1001, probe 13308 (rebuilt key by key from the API's answer), and a lost packet.
RIPE = {"fw": 5080, "mver": "2.6.2", "lts": 13, "dst_name": "193.0.14.129", "af": 4, "dst_addr": "193.0.14.129", "src_addr": "192.168.1.21",
        "proto": "ICMP", "ttl": 56, "size": 20, "result": [{"rtt": 35.036881}, {"rtt": 34.888817}, {"rtt": 32.463427}], "dup": 0, "rcvd": 3,
        "sent": 3, "min": 32.463427, "max": 35.036881, "avg": 34.12970833333333, "msm_id": 1001, "prb_id": 13308, "timestamp": 1794096069,
        "msm_name": "Ping", "from": "31.136.33.50", "type": "ping", "step": 240, "stored_timestamp": 1794096164}


def test_ripe_results_pool_into_blocks_with_losses():
    lost = dict(RIPE, timestamp=RIPE["timestamp"] + 240, result=[{"x": "*"}, {"rtt": 40.0}, {"error": "boom"}])
    (start, rtts, sent, n_lost), = list(ripe_atlas.blocks([RIPE, lost]))
    assert start % ripe_atlas.BLOCK_S == 0 and sent == 5 and n_lost == 1 and len(rtts) == 4


def test_summary_traces_keep_the_statistics_and_never_reorder():
    t = synth.summary_trace(seed=3, seconds=600.0, floor_ms=10.0, mean_ms=25.0, sd_ms=10.0, loss=0.01)
    d = t.delays_ms()[~t.lost]
    assert d.min() >= 10.0 and abs(np.mean(d) - 25.0) < 1.5 and abs(t.lost.mean() - 0.01) < 0.003
    assert assess(t).reorder_ratio == 0.0
    assert np.array_equal(t.delays, synth.summary_trace(3, 600.0, 10.0, 25.0, 10.0, 0.01).delays)


def test_downloads_only_over_https_from_the_datasets_hosts_and_under_our_own_names():
    src = mahimahi.SOURCE
    fetching.check_url("https://raw.githubusercontent.com/ravinet/mahimahi/master/traces/x", src.hosts)
    for bad in ("http://raw.githubusercontent.com/x", "https://evil.example/x", "file:///etc/passwd"):
        with pytest.raises(ValueError):
            fetching.check_url(bad, src.hosts)
    for bad in ("../x", ".hidden", "a/b", ""):
        with pytest.raises(ValueError):
            fetching.safe_name(bad)
