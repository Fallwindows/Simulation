"""Runtime capability detection without importing Isaac during unit tests."""

from __future__ import annotations

import importlib.util
import os
import shutil
from pathlib import Path
from dataclasses import dataclass


@dataclass(frozen=True)
class RuntimeCapabilities:
    isaac_usd: bool
    ros2: bool
    rtabmap: bool


def _rtabmap_available() -> bool:
    explicit = os.environ.get("RTABMAP_EXECUTABLE")
    if explicit and Path(explicit).exists():
        return True
    if shutil.which("rtabmap") or shutil.which("icp_odometry"):
        return True
    for variable in ("ISAACSIM_ROS_WORKSPACE", "GROCERY_ROS_WORKSPACE"):
        workspace = os.environ.get(variable)
        if workspace and (Path(workspace) / "install" / "Lib" / "rtabmap_slam" / "rtabmap.exe").exists():
            return True
    return False


def detect_capabilities() -> RuntimeCapabilities:
    return RuntimeCapabilities(
        isaac_usd=importlib.util.find_spec("omni.usd") is not None,
        ros2=importlib.util.find_spec("rclpy") is not None,
        rtabmap=_rtabmap_available(),
    )
