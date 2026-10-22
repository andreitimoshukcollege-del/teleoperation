"""Copies of ``Transport/Impairments/*.cs`` and of ``Transport/EmulatedTransport.cs``'s ordering.

How Core decides a datagram's fate, which the copy reproduces exactly:
- **at send**, the send-stage impairments (Gilbert-Elliott loss) run in array order; a dropped
  datagram never enters the link and consumes no delivery-stage draw;
- **at delivery**, in send order (the loopback inner transport is FIFO and stamps arrival = send),
  the delivery-stage impairments add delay in array order: fixed, jitter (one ``NextUInt64``),
  reorder (one ``NextDouble``), trace (one cursor step). The total is clamped at zero once;
- datagrams come out ordered by ``(arrival, delivery sequence)``.

Each impairment draws only from its own substream, ``derive_for_axis(seed, axis_name)``.

One deliberate difference: Core's ``EmulatedTransport`` has a fixed in-flight capacity (64 in a
sweep) that can drop datagrams under very long delays. A real UDP path has no such cap, so
``EmulatedLink`` is unbounded; the golden files are written with a capacity that never binds.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .rng import SeededRng, derive_for_axis

SEND = 1
DELIVER = 2


class Impairment:
    axis_name: str = ""
    stages: int = 0

    def bind(self, substream_seed: int) -> None:
        if getattr(self, "_bound", False):
            raise RuntimeError("impairment already bound: each link needs its own instances")
        self._bound = True

    def apply_on_send(self) -> bool:
        """Returns True if the datagram is dropped."""
        return False

    def apply_on_deliver(self) -> int:
        """Returns the delay to add, in ticks (may be negative before the clamp)."""
        return 0

    def reset(self) -> None:
        pass


class FixedDelay(Impairment):
    axis_name = "delay"
    stages = DELIVER

    def __init__(self, delay_ticks: int):
        if delay_ticks < 0:
            raise ValueError("delay must not be negative")
        self.delay_ticks = int(delay_ticks)

    def apply_on_deliver(self) -> int:
        return self.delay_ticks


class UniformJitter(Impairment):
    axis_name = "jitter"
    stages = DELIVER

    def __init__(self, half_width_ticks: int):
        if half_width_ticks < 0:
            raise ValueError("jitter half-width must not be negative")
        self.half_width_ticks = int(half_width_ticks)
        self._rng: Optional[SeededRng] = None

    def bind(self, substream_seed: int) -> None:
        super().bind(substream_seed)
        self._rng = SeededRng(substream_seed)

    def apply_on_deliver(self) -> int:
        span = self.half_width_ticks * 2 + 1
        return (self._rng.next_uint64() % span) - self.half_width_ticks

    def reset(self) -> None:
        self._rng.reset()


class GilbertElliottLoss(Impairment):
    axis_name = "loss"
    stages = SEND

    def __init__(self, after_delivered: float, after_lost: float):
        for p in (after_delivered, after_lost):
            if not (0.0 <= p <= 1.0):
                raise ValueError("loss probabilities must be in [0, 1]")
        self.after_delivered = float(after_delivered)
        self.after_lost = float(after_lost)
        self._previous_lost = False
        self._rng: Optional[SeededRng] = None

    def bind(self, substream_seed: int) -> None:
        super().bind(substream_seed)
        self._rng = SeededRng(substream_seed)

    def apply_on_send(self) -> bool:
        p = self.after_lost if self._previous_lost else self.after_delivered
        lost = self._rng.next_double() < p
        self._previous_lost = lost
        return lost

    def reset(self) -> None:
        self._previous_lost = False
        self._rng.reset()


class Reorder(Impairment):
    axis_name = "reorder"
    stages = DELIVER

    def __init__(self, probability: float, extra_delay_ticks: int):
        if not (0.0 <= probability <= 1.0):
            raise ValueError("reorder probability must be in [0, 1]")
        if extra_delay_ticks < 0:
            raise ValueError("reorder delay must not be negative")
        self.probability = float(probability)
        self.extra_delay_ticks = int(extra_delay_ticks)
        self._rng: Optional[SeededRng] = None

    def bind(self, substream_seed: int) -> None:
        super().bind(substream_seed)
        self._rng = SeededRng(substream_seed)

    def apply_on_deliver(self) -> int:
        return self.extra_delay_ticks if self._rng.next_double() < self.probability else 0

    def reset(self) -> None:
        self._rng.reset()


class TraceDelay(Impairment):
    axis_name = "delay-trace"
    stages = DELIVER

    def __init__(self, delay_trace_ticks: Sequence[int]):
        samples = [int(s) for s in delay_trace_ticks]
        if not samples:
            raise ValueError("a delay trace needs at least one sample")
        if any(s < 0 for s in samples):
            raise ValueError("delay trace samples must not be negative")
        self.samples = samples
        self._cursor = 0

    def apply_on_deliver(self) -> int:
        d = self.samples[self._cursor]
        self._cursor = (self._cursor + 1) % len(self.samples)
        return d

    def reset(self) -> None:
        self._cursor = 0


@dataclass
class LinkResult:
    """What a link did to a sequence of datagrams, in send order."""

    delivered: np.ndarray  # bool
    arrival: np.ndarray  # int64; meaningless where not delivered
    delivery_order: np.ndarray  # indices of delivered datagrams, in the order they come out


class EmulatedLink:
    """One direction of an ``EmulatedTransport`` over a loopback, as a function of send times."""

    def __init__(self, impairments: List[Impairment], seed: int):
        names = [imp.axis_name for imp in impairments]
        if len(set(names)) != len(names):
            raise ValueError(f"two impairments share an axis name: {names}")
        self.impairments = list(impairments)
        self._send = [imp for imp in self.impairments if imp.stages & SEND]
        self._deliver = [imp for imp in self.impairments if imp.stages & DELIVER]
        for imp in self.impairments:
            imp.bind(derive_for_axis(seed, imp.axis_name))

    def transmit_one(self, send_ticks: int) -> Tuple[bool, int]:
        """One datagram, in send order: (delivered, arrival). Delivery-stage draws happen here, in
        send order, which is exactly the order Core's FIFO drain takes them in."""
        dropped = False
        for imp in self._send:
            if imp.apply_on_send():
                dropped = True
        if dropped:
            return False, 0
        delay = 0
        for imp in self._deliver:
            delay += imp.apply_on_deliver()
        return True, int(send_ticks) + (delay if delay > 0 else 0)

    def transmit(self, send_ticks: np.ndarray) -> LinkResult:
        n = int(len(send_ticks))
        delivered = np.zeros(n, dtype=bool)
        arrival = np.zeros(n, dtype=np.int64)
        for i in range(n):
            ok, a = self.transmit_one(int(send_ticks[i]))
            delivered[i] = ok
            arrival[i] = a
        idx = np.nonzero(delivered)[0]
        # Ties in arrival go out in delivery sequence, which is send order among the delivered.
        order = idx[np.lexsort((idx, arrival[idx]))]
        return LinkResult(delivered=delivered, arrival=arrival, delivery_order=order)

    def reset(self) -> None:
        for imp in self.impairments:
            imp.reset()
