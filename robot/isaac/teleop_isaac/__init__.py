"""The Isaac bridge (docs/adr/0031): the FR3 Duo in Isaac Sim, behind RobotHost over the arm-server wire (docs/adr/0030).

``wire``, ``shaper`` and ``profile`` are plain Python (the standard library only), so their tests run anywhere; ``bridge``
needs Isaac Sim and runs on the GPU machine (``just isaac-host``).
"""
