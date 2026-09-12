"""Thread-safe latest-value caches used by ROS callbacks and HTTP handlers."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class DashboardState:
    rgb_jpeg: bytes | None = None
    lidar_points: list[tuple[float, float, float]] = field(default_factory=list)
    map_points: list[tuple[float, float, float]] = field(default_factory=list)
    status: dict[str, Any] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    _updated: dict[str, float] = field(default_factory=dict, repr=False)
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False)

    def update_rgb(self, jpeg: bytes, receive_time: float | None = None) -> None:
        with self._lock:
            self.rgb_jpeg = bytes(jpeg)
            self._updated["rgb"] = receive_time if receive_time is not None else time.time()

    def update_points(self, kind: str, points: list[tuple[float, float, float]], receive_time: float | None = None) -> None:
        if kind not in {"lidar", "map"}:
            raise ValueError("kind must be lidar or map")
        with self._lock:
            setattr(self, f"{kind}_points", list(points))
            self._updated[kind] = receive_time if receive_time is not None else time.time()

    def update_metrics(self, metrics: dict[str, Any]) -> None:
        with self._lock:
            self.metrics = dict(metrics)

    def merge_metrics(self, metrics: dict[str, Any]) -> None:
        with self._lock:
            self.metrics.update(metrics)

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            status = dict(self.status)
            status.update({
                "rgb_connected": self.rgb_jpeg is not None,
                "lidar_connected": bool(self.lidar_points),
                "map_connected": bool(self.map_points),
                "rgb_last_frame_age_s": None if "rgb" not in self._updated else max(0.0, now - self._updated["rgb"]),
                "lidar_last_scan_age_s": None if "lidar" not in self._updated else max(0.0, now - self._updated["lidar"]),
                "map_last_update_age_s": None if "map" not in self._updated else max(0.0, now - self._updated["map"]),
                "lidar_point_count": len(self.lidar_points),
                "map_point_count": len(self.map_points),
            })
            return {"status": status, "metrics": dict(self.metrics)}

    def decimated_points(self, kind: str, budget: int) -> list[tuple[float, float, float]]:
        if budget < 1:
            return []
        with self._lock:
            points = list(getattr(self, f"{kind}_points"))
        if len(points) <= budget:
            return points
        stride = len(points) / budget
        return [points[int(i * stride)] for i in range(budget)]
