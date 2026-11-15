import os
import sys

# robot/gpu's two modules, imported by name (they are scripts, not a package).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
