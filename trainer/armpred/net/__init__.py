"""Exact copies of the parts of Core that decide what the headset sees (docs/adr/0021 decision 5).

Each module names the C# file it copies. They reproduce Core's arithmetic bit for bit where Core is
integer, and its single-precision steps where Core uses ``float``; ``tests/golden`` pins them to
files the repo writes from the C# originals.
"""
