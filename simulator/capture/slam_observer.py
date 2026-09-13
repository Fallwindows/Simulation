"""Capture only estimator outputs during offline bag replay.

This process never subscribes to ground truth.  It records fresh odometry and
the latest RTAB-Map cloud, making the SLAM interface independently testable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path


def _stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        for point in points:
            handle.write(f"{point[0]:.8g} {point[1]:.8g} {point[2]:.8g}\n")


def _write_ply(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write(f"ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
        for point in points:
            handle.write(f"{point[0]:.8g} {point[1]:.8g} {point[2]:.8g}\n")


class SlamObserver:
    def __init__(self, output_dir: Path, duration_s: float, startup_timeout_s: float):
        import rclpy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        self.rclpy = rclpy
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        self.target_clock_s: float | None = None
        self.done_wall: float | None = None
        self.odom_rows: list[dict[str, float]] = []
        self.latest_map: list[tuple[float, float, float]] = []
        self.map_stamp_s: float | None = None
        self.map_messages = 0
        self.point_cloud2 = point_cloud2
        self.node = Node("grocery_sim_offline_slam_observer")
        self.node.create_subscription(Clock, "/clock", self._on_clock, 100)
        self.node.create_subscription(Odometry, "/slam/odom", self._on_odom, 50)
        self.node.create_subscription(PointCloud2, "/slam/map_cloud", self._on_map, 10)

    def _on_clock(self, message) -> None:
        stamp_s = _stamp(message.clock)
        self.last_clock_s = stamp_s if self.last_clock_s is None else max(self.last_clock_s, stamp_s)
        if self.first_clock_s is None:
            self.first_clock_s = stamp_s
            self.target_clock_s = self.first_clock_s + self.duration_s
        if self.target_clock_s is not None and stamp_s >= self.target_clock_s and self.done_wall is None:
            self.done_wall = time.monotonic() + 3.0

    def _on_odom(self, message) -> None:
        stamp = _stamp(message.header.stamp)
        q = (
            float(message.pose.pose.orientation.x),
            float(message.pose.pose.orientation.y),
            float(message.pose.pose.orientation.z),
            float(message.pose.pose.orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in q))
        self.odom_rows.append({
            "timestamp_s": stamp,
            "x_m": float(message.pose.pose.position.x),
            "y_m": float(message.pose.pose.position.y),
            "z_m": float(message.pose.pose.position.z),
            "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
            "quaternion_valid": 1.0 if math.isfinite(norm) and norm > 1e-12 else 0.0,
        })

    def _on_map(self, message) -> None:
        fields = {"x", "y", "z"}
        names = {field.name for field in message.fields}
        if not fields.issubset(names):
            return
        self.latest_map = [
            (float(x), float(y), float(z))
            for x, y, z in self.point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        ]
        self.map_stamp_s = _stamp(message.header.stamp)
        self.map_messages += 1

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            now = time.monotonic()
            if self.done_wall is not None and now >= self.done_wall:
                return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                raise RuntimeError("offline SLAM observer did not observe /clock")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        with (self.output_dir / "slam_poses.csv").open("w", newline="", encoding="utf-8") as handle:
            fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "quaternion_valid"]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.odom_rows)
        _write_pcd(self.output_dir / "slam_map.pcd", self.latest_map)
        _write_ply(self.output_dir / "slam_map.ply", self.latest_map)
        result = {
            "status": "complete" if self.odom_rows and self.latest_map else "incomplete",
            "odom_sample_count": len(self.odom_rows),
            "map_message_count": self.map_messages,
            "map_point_count": len(self.latest_map),
            "first_clock_s": self.first_clock_s,
            "last_clock_s": self.last_clock_s,
            "last_map_stamp_s": self.map_stamp_s,
            "fresh_odom": bool(self.odom_rows),
            "fresh_map": bool(self.latest_map),
            "ground_truth_subscribed": False,
        }
        (self.output_dir / "slam_observer.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.node.destroy_node()
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration-seconds", required=True, type=float)
    parser.add_argument("--startup-timeout-seconds", type=float, default=180.0)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    observer = SlamObserver(Path(args.output_dir), args.duration_seconds, args.startup_timeout_seconds)
    try:
        observer.spin_until_done()
        result = observer.close()
        print(json.dumps(result, indent=2))
        if result["status"] != "complete":
            raise RuntimeError(f"offline SLAM did not publish fresh odometry and map: {result}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
