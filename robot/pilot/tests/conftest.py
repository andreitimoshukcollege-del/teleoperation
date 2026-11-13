import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PILOT = os.path.dirname(HERE)  # robot/pilot
sys.path.insert(0, PILOT)
