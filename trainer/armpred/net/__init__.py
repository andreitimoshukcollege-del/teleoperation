"""Exact copies of the parts of Core that decide what the headset sees (docs/adr/0021 decision 5).

Each module names the C# file it copies. They reproduce Core's arithmetic bit for bit where Core is
integer, and its single-precision steps where Core uses ``float``; ``tests/golden`` pins them to
files the repo writes from the C# originals.

One module here is *not* a copy: ``forecast.py``, network forecast v1 and the forecasters of known
quality (docs/adr/0021 decisions 11 and 12). No C# twin exists, and none is needed until the C# phase
gives the forecast a Core contract.
"""
