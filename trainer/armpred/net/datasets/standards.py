"""The standards' numbers the network tiers are built on (docs/adr/0035 amendment 2), copied from the Recommendations'
text with the clause each comes from. Change a number only with the Recommendation in hand, and the ADR with it.

- **ITU-T Y.1541 (12/2011), clause 5.3, Table 1** "IP network QoS class definitions and network performance
  objectives": per class, the upper bound on the mean one-way delay (IPTD), on the 1 - 10^-3 quantile of IPTD minus the
  minimum IPTD (IPDV, the 2-point delay variation of ITU-T Y.1540), and on the loss ratio (IPLR). ``None`` is the
  table's "U" (unspecified). The general notes suggest an evaluation interval of 1 minute: "Any minute observed should
  meet these objectives."
- **ITU-T G.1050 (07/2016), clause 7.1, Table 6** "Impairment levels per service test profile": profile A
  (well-managed), B (partially-managed), C (unmanaged): one-way latency, peak-to-peak jitter, sequential (burst) loss
  length and rate, random loss and reordering. The total loss "is the sum of the sequential packet loss and random packet
  loss".
- **G.1050 clause 7.2** labels its standard test cases ``<access><profile><severity>``: access D (DSL) or G (GPON),
  profile w / p / u (A / B / C), severity 1.. (higher is harder): ``Dw4`` is DSL, well-managed, severity 4.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

Y1541 = "ITU-T Y.1541 (12/2011) clause 5.3 Table 1"
G1050 = "ITU-T G.1050 (07/2016) clause 7.1 Table 6"
EVALUATION_INTERVAL_S = 60.0  # Y.1541 Table 1, general notes


@dataclass(frozen=True)
class QosClass:
    number: int
    mean_delay_ms: Optional[float]  # IPTD
    ipdv_ms: Optional[float]  # 1 - 10^-3 quantile minus minimum
    loss_ratio: Optional[float]  # IPLR


Y1541_CLASSES: Tuple[QosClass, ...] = (
    QosClass(0, 100.0, 50.0, 1e-3),
    QosClass(1, 400.0, 50.0, 1e-3),
    QosClass(2, 100.0, None, 1e-3),
    QosClass(3, 400.0, None, 1e-3),
    QosClass(4, 1000.0, None, 1e-3),
    QosClass(5, None, None, None),
)
IPDV_QUANTILE = 1.0 - 1e-3


@dataclass(frozen=True)
class ServiceProfile:
    """One column of G.1050 Table 6 (upper ends of its ranges; the lower ends are all 0, or 20 ms of latency)."""

    name: str
    letter: str  # the test-case label's profile character
    latency_ms: float  # one-way; A and B: the intercontinental range's top
    jitter_pp_ms: float  # peak-to-peak
    burst_loss_ms: Optional[float]  # the longest sequential loss; None: "random loss only"
    burst_rate_per_s: Optional[float]  # "rate of sequential loss" upper bound (exclusive); None: random loss only
    random_loss_ratio: float  # random packet loss, as a ratio (the table gives %)
    reorder_ratio: float  # reordered packets, as a ratio (the table gives %)


G1050_PROFILES: Tuple[ServiceProfile, ...] = (
    ServiceProfile("A well-managed", "w", 300.0, 50.0, None, None, 0.0005, 0.00001),
    ServiceProfile("B partially-managed", "p", 400.0, 150.0, 200.0, 1e-3, 0.02, 0.0001),
    ServiceProfile("C unmanaged", "u", 500.0, 500.0, 10_000.0, 1e-1, 0.20, 0.001),
)
G1050_SHORTEST_BURST_MS = 40.0  # Table 6: sequential loss of B and C starts at 40 ms

# docs/adr/0035 decision 2: the three tiers are G.1050's three service test profiles.
TIER_OF_PROFILE: Dict[str, str] = {"w": "good", "p": "poor", "u": "very-bad"}
TIERS: Tuple[str, ...] = ("good", "poor", "very-bad")
