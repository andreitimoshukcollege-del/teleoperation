import sys

from .cli import main

# Guarded: spawned worker processes (Windows, macOS) re-import the main module as __mp_main__.
if __name__ == "__main__":
    sys.exit(main())
