"""From a session to what the headset would have seen (docs/adr/0021 decision 4).

A *view* is one session replayed through one downlink schedule, one uplink shift and one seed: the
exact sequence of calls the headset's predictor would have received, plus the truth.
"""
