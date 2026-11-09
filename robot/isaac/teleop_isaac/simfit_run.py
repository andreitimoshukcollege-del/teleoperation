"""``python -m teleop_isaac.simfit_run <sessions.json> [bridge flags]``: the sim-against-real fit (docs/adr/0034 J3), the bridge's
``--replay`` under a module name of its own, so ``just isaac-stop`` and the hosts' "already running" checks, which match
``teleop_isaac.(bridge|jetrover_bridge)``, never stop or count it. A host may run beside it."""

import sys

from .jetrover_bridge import main

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("usage: python -m teleop_isaac.simfit_run <sessions.json> [--fit-seconds N] [--fit-rounds N] [--out FILE]")
    sys.exit(main(["--replay", sys.argv[1], *sys.argv[2:]]))
