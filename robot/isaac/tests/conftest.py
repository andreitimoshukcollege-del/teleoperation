import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)  # robot/isaac
REPO = os.path.dirname(os.path.dirname(ROOT))
sys.path.insert(0, ROOT)


def golden(name: str) -> dict:
    with open(os.path.join(ROOT, "golden", name), encoding="utf-8") as f:
        return json.load(f)


def profile_path() -> str:
    return os.path.join(REPO, "core", "RobotProfiles", "fr3-duo.json")
