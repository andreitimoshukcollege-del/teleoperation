"""Copies of ``Types/SeededRng.cs`` and ``Transport/Impairments/ImpairmentStreams.cs``.

Pure 64-bit unsigned arithmetic, so the copy is exact: every operation is masked to 64 bits the way
C#'s ``unchecked`` ulong arithmetic wraps.
"""

from __future__ import annotations

MASK64 = (1 << 64) - 1
GOLDEN_RATIO_64 = 0x9E3779B97F4A7C15
_FNV1A64_OFFSET_BASIS = 14695981039346656037
_FNV1A64_PRIME = 1099511628211


def _splitmix64_next(state: int) -> tuple[int, int]:
    """SeededRng.SplitMix64Next: returns (output, new state)."""
    state = (state + GOLDEN_RATIO_64) & MASK64
    z = state
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return z ^ (z >> 31), state


def _splitmix64_init(seed: int) -> tuple[int, int]:
    state = seed & MASK64
    s0, state = _splitmix64_next(state)
    s1, state = _splitmix64_next(state)
    if s0 == 0 and s1 == 0:
        s0 = GOLDEN_RATIO_64
    return s0, s1


class SeededRng:
    """xorshift128+, seeded via splitmix64. Same seed, same stream as Core, forever."""

    __slots__ = ("_seed", "_s0", "_s1")

    def __init__(self, seed: int):
        self._seed = seed & MASK64
        self._s0, self._s1 = _splitmix64_init(self._seed)

    def next_uint64(self) -> int:
        x, y = self._s0, self._s1
        self._s0 = y
        x ^= (x << 23) & MASK64
        x ^= x >> 17
        x ^= y ^ (y >> 26)
        self._s1 = x
        return (x + y) & MASK64

    def next_double(self) -> float:
        """Uniform on [0, 1) from the top 53 bits, exactly as Core builds it."""
        return (self.next_uint64() >> 11) * (1.0 / (1 << 53))

    def reset(self) -> None:
        self._s0, self._s1 = _splitmix64_init(self._seed)


def stream_id_from_name(axis_name: str) -> int:
    """ImpairmentStreams.StreamIdFromName: FNV-1a 64 over the UTF-16 code units, low byte first."""
    h = _FNV1A64_OFFSET_BASIS
    for unit in axis_name.encode("utf-16-le"):
        h ^= unit
        h = (h * _FNV1A64_PRIME) & MASK64
    return h


def derive(trial_seed: int, stream_id: int) -> int:
    """ImpairmentStreams.Derive: the SplitMix64 finalizer over seed + streamId * golden."""
    z = (trial_seed + stream_id * GOLDEN_RATIO_64) & MASK64
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return z ^ (z >> 31)


def derive_for_axis(trial_seed: int, axis_name: str) -> int:
    return derive(trial_seed, stream_id_from_name(axis_name))
