"""Network datasets for the network-aware predictor (docs/adr/0035 N2): public measurements and our own Wi-Fi, as delay
traces with loss (``.trace`` v2), each assessed against the standards' tiers (G.1050's service test profiles, measured
as Y.1541 measures), and split so some of every source and tier is never trained on.

    python -m armpred.net.datasets fetch [names...]     # on the GPU machine: just network-datasets-fetch
    python -m armpred.net.datasets convert [names...]
    python -m armpred.net.datasets split
    python -m armpred.net.datasets status
    python -m armpred.net.datasets ours <file.ltrace>...  # our own traces into <root>/ours-wifi

Sources: ``g1050-2016`` (the standard's own test cases), ``mahimahi`` (cellular), ``puffer`` (home connections),
``fcc-mba`` (US broadband, idle and loaded), ``ripe-atlas`` (probes by connection kind), ``ours-wifi`` (the headset).
"""
