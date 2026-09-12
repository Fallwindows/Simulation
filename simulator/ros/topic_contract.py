"""Public simulator topic/frame contract loaded from the canonical JSON YAML."""

from __future__ import annotations

import json
from pathlib import Path


CANONICAL_CONTRACT_PATH = Path(__file__).resolve().parents[2] / "config" / "contracts.yaml"
CONTRACT = json.loads(CANONICAL_CONTRACT_PATH.read_text(encoding="utf-8"))
FRAMES = dict(CONTRACT["frames"])
TOPICS = dict(CONTRACT["topics"])

GROUND_TRUTH_MESSAGE = "geometry_msgs/msg/PoseStamped"
GROUND_TRUTH_FRAME = FRAMES["sim_world"]
TRUTH_TF_CHILD_FRAME = FRAMES["truth_sensor_rig"]
