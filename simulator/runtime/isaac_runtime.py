"""Runtime capability detection without importing Isaac during unit tests."""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeCapabilities:
    isaac_usd: bool
    ros2: bool
    rtabmap: bool


def detect_capabilities() -> RuntimeCapabilities:
    return RuntimeCapabilities(
        isaac_usd=importlib.util.find_spec("omni.usd") is not None,
        ros2=importlib.util.find_spec("rclpy") is not None,
        rtabmap=False,
    )
