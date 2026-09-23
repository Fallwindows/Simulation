"""Capture only estimator outputs during offline bag replay.

This process never subscribes to ground truth.  It records fresh odometry and
the latest RTAB-Map cloud, making the SLAM interface independently testable.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sqlite3
import time
from pathlib import Path


def _stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid transform quaternion")
    return tuple(value / norm for value in q)


def _quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize((aw * bx + ax * bw + ay * bz - az * by,
                           aw * by - ax * bz + ay * bw + az * bx,
                           aw * bz + ax * by - ay * bx + az * bw,
                           aw * bw - ax * bx - ay * by - az * bz))


def _quat_rotate(q: tuple[float, float, float, float], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = _quat_normalize(q)
    px, py, pz = p
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (px + w * tx + y * tz - z * ty,
            py + w * ty + z * tx - x * tz,
            pz + w * tz + x * ty - y * tx)


def _quat_slerp(a: tuple[float, float, float, float], b: tuple[float, float, float, float], fraction: float) -> tuple[float, float, float, float]:
    qa = _quat_normalize(a)
    qb = _quat_normalize(b)
    dot = sum(left * right for left, right in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-value for value in qb)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _quat_normalize(tuple(left + fraction * (right - left) for left, right in zip(qa, qb)))
    angle = math.acos(dot)
    sine = math.sin(angle)
    left_scale = math.sin((1.0 - fraction) * angle) / sine
    right_scale = math.sin(fraction * angle) / sine
    return _quat_normalize(tuple(left_scale * left + right_scale * right for left, right in zip(qa, qb)))


def _interpolate_map_to_odom(rows: list[dict[str, object]], stamp_s: float) -> dict[str, object] | None:
    if not math.isfinite(stamp_s):
        return None
    valid_rows = [
        row for row in rows
        if all(math.isfinite(float(row[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"))
    ]
    ordered = sorted(valid_rows, key=lambda row: float(row["timestamp_s"]))
    for row in ordered:
        if abs(float(row["timestamp_s"]) - stamp_s) <= 1e-9:
            return row
    before = next((row for row in reversed(ordered) if float(row["timestamp_s"]) < stamp_s), None)
    after = next((row for row in ordered if float(row["timestamp_s"]) > stamp_s), None)
    if before is None or after is None:
        return None
    t0, t1 = float(before["timestamp_s"]), float(after["timestamp_s"])
    fraction = (stamp_s - t0) / (t1 - t0)
    q = _quat_slerp(
        tuple(float(before[key]) for key in ("qx", "qy", "qz", "qw")),
        tuple(float(after[key]) for key in ("qx", "qy", "qz", "qw")),
        fraction,
    )
    return {
        "timestamp_s": stamp_s,
        "x_m": float(before["x_m"]) + fraction * (float(after["x_m"]) - float(before["x_m"])),
        "y_m": float(before["y_m"]) + fraction * (float(after["y_m"]) - float(before["y_m"])),
        "z_m": float(before["z_m"]) + fraction * (float(after["z_m"]) - float(before["z_m"])),
        "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
        "parent_frame_id": "map", "child_frame_id": "odom",
    }


def _correct_odom_pose(odom: dict[str, float], transform: dict[str, object]) -> dict[str, object]:
    if not all(math.isfinite(float(odom[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")):
        raise ValueError("non-finite odometry pose")
    if not all(math.isfinite(float(transform[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")):
        raise ValueError("non-finite map-to-odom transform")
    q_mo = _quat_normalize(tuple(float(transform[key]) for key in ("qx", "qy", "qz", "qw")))
    q_odom = _quat_normalize(tuple(float(odom[key]) for key in ("qx", "qy", "qz", "qw")))
    rotated = _quat_rotate(q_mo, (odom["x_m"], odom["y_m"], odom["z_m"]))
    q_map_pose = _quat_multiply(q_mo, q_odom)
    mapped_position = (float(transform["x_m"]) + rotated[0], float(transform["y_m"]) + rotated[1], float(transform["z_m"]) + rotated[2])
    if not all(math.isfinite(value) for value in (*mapped_position, *q_map_pose)):
        raise ValueError("map-frame odometry correction produced non-finite values")
    return {
        "timestamp_s": odom["timestamp_s"],
        "x_m": mapped_position[0],
        "y_m": mapped_position[1],
        "z_m": mapped_position[2],
        "qx": q_map_pose[0], "qy": q_map_pose[1], "qz": q_map_pose[2], "qw": q_map_pose[3],
        "frame_id": "map",
    }


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
        from rtabmap_msgs.srv import PublishMap
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        self.target_clock_s: float | None = None
        self.clock_regressions = 0
        self.clock_target_reached = False
        self.replay_complete_signal: Path | None = None
        self.replay_complete_signal_observed = False
        self.drain_complete = False
        self.callback_generation = 0
        self.drain_quiet_polls = 0
        self.expected_sensor_last_stamp_s: float | None = None
        self.sensor_scan_period_s = 0.0
        self.last_odom_stamp_s: float | None = None
        self.replay_drained = False
        self.publish_map_acknowledged = False
        self.final_map_span = False
        self.mapper_database_span = False
        self.database_path: Path | None = None
        self.database_node_count = 0
        self.database_last_stamp_s: float | None = None
        self.map_messages_before_publish = 0
        self.close_timeout_s = 60.0
        self.odom_rows: list[dict[str, float]] = []
        self.map_to_odom_rows: list[dict[str, object]] = []
        self.map_pose_rows: list[dict[str, object]] = []
        self.latest_map: list[tuple[float, float, float]] = []
        self.map_stamp_s: float | None = None
        self.map_messages = 0
        self.point_cloud2 = point_cloud2
        self.node = Node("grocery_sim_offline_slam_observer")
        self.node.create_subscription(Clock, "/clock", self._on_clock, 100)
        self.node.create_subscription(Odometry, "/slam/odom", self._on_odom, 50)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.node.create_subscription(PointCloud2, "/slam/map_cloud", self._on_map, 10)
        self.publish_map_client = self.node.create_client(PublishMap, "/rtabmap/publish_map")

    def _on_clock(self, message) -> None:
        stamp_s = _stamp(message.clock)
        self.callback_generation += 1
        if self.last_clock_s is not None and stamp_s < self.last_clock_s - 1e-9:
            self.clock_regressions += 1
        self.last_clock_s = stamp_s
        if self.first_clock_s is None:
            self.first_clock_s = stamp_s
            self.target_clock_s = self.first_clock_s + self.duration_s
        if self.target_clock_s is not None and stamp_s >= self.target_clock_s - 1e-3:
            self.clock_target_reached = True

    def _on_odom(self, message) -> None:
        self.callback_generation += 1
        stamp = _stamp(message.header.stamp)
        self.last_odom_stamp_s = stamp if self.last_odom_stamp_s is None else max(self.last_odom_stamp_s, stamp)
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

    def _on_tf(self, message) -> None:
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") != "map" or transform.child_frame_id.lstrip("/") != "odom":
                continue
            stamp = _stamp(transform.header.stamp)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            if not all(math.isfinite(value) for value in (stamp, float(translation.x), float(translation.y), float(translation.z))):
                continue
            try:
                q = _quat_normalize((float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)))
            except ValueError:
                continue
            self.map_to_odom_rows.append({
                "timestamp_s": stamp,
                "x_m": float(translation.x), "y_m": float(translation.y), "z_m": float(translation.z),
                "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
                "parent_frame_id": "map", "child_frame_id": "odom",
            })
            self.callback_generation += 1

    def _on_map(self, message) -> None:
        self.callback_generation += 1
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

    def _database_progress(self) -> tuple[int, float | None]:
        if self.database_path is None or not self.database_path.is_file():
            return 0, None
        try:
            connection = sqlite3.connect(self.database_path.as_uri() + "?mode=ro", uri=True, timeout=0.2)
            try:
                connection.execute("PRAGMA busy_timeout=200")
                count, stamp = connection.execute("SELECT count(*), max(stamp) FROM Node").fetchone()
            finally:
                connection.close()
            return int(count or 0), None if stamp is None else float(stamp)
        except (sqlite3.Error, OSError, ValueError):
            return 0, None

    def _database_covers_input(self) -> bool:
        self.database_node_count, self.database_last_stamp_s = self._database_progress()
        return (
            self.expected_sensor_last_stamp_s is not None
            and self.database_node_count > 0
            and self.database_last_stamp_s is not None
            and self.database_last_stamp_s >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
        )

    def _publish_final_map(self) -> None:
        service_deadline = time.monotonic() + self.close_timeout_s
        while self.rclpy.ok() and not self.publish_map_client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= service_deadline:
                raise RuntimeError("RTAB-Map PublishMap service did not become ready")
        if not self.rclpy.ok():
            raise RuntimeError("ROS shut down before final map publication")
        from rtabmap_msgs.srv import PublishMap

        request = PublishMap.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False
        self.map_messages_before_publish = self.map_messages
        future = self.publish_map_client.call_async(request)
        while self.rclpy.ok() and not future.done():
            if time.monotonic() >= service_deadline:
                raise RuntimeError("RTAB-Map PublishMap response timed out")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if not self.rclpy.ok():
            raise RuntimeError("ROS shut down while waiting for final map publication")
        response = future.result()
        if response is None or not bool(getattr(response, "success", False)):
            raise RuntimeError(f"RTAB-Map rejected final optimized map publication: {response}")
        self.publish_map_acknowledged = True

        post_ack_deadline = time.monotonic() + self.close_timeout_s
        quiet_since: float | None = None
        last_generation = self.callback_generation
        while self.rclpy.ok():
            now = time.monotonic()
            self.mapper_database_span = self._database_covers_input()
            self.final_map_span = (
                self.map_messages > self.map_messages_before_publish
                and self.map_stamp_s is not None
                and self.expected_sensor_last_stamp_s is not None
                and self.map_stamp_s >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
            )
            if self.callback_generation != last_generation:
                quiet_since = None
                last_generation = self.callback_generation
            elif self.publish_map_acknowledged and self.final_map_span and self.mapper_database_span:
                if quiet_since is None:
                    quiet_since = now
                elif now - quiet_since >= 1.0:
                    self.drain_complete = True
                    return
            if now >= post_ack_deadline:
                raise RuntimeError("final optimized map/database span did not settle within the bounded wait")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def spin_until_done(self) -> None:
        if self.replay_complete_signal is None:
            raise RuntimeError("offline SLAM observer requires a replay-complete signal path")
        signal_seen_wall: float | None = None
        quiet_since: float | None = None
        last_generation = self.callback_generation
        wall_deadline = self.started_wall + max(60.0, self.duration_s * 20.0 + 30.0)
        while self.rclpy.ok():
            now = time.monotonic()
            if self.replay_complete_signal.is_file():
                if signal_seen_wall is None:
                    signal_seen_wall = now
                    self.replay_complete_signal_observed = True
                if self.callback_generation == last_generation:
                    if quiet_since is None:
                        quiet_since = now
                    self.drain_quiet_polls += 1
                else:
                    quiet_since = None
                    self.drain_quiet_polls = 0
                    last_generation = self.callback_generation
                if now - signal_seen_wall >= 60.0:
                    raise RuntimeError("mapper did not commit the replay input span within the bounded wait")
                if quiet_since is not None and now - quiet_since >= 1.0 and self._database_covers_input():
                    if not self.clock_target_reached:
                        raise RuntimeError(
                            f"bag replay exited before /clock reached target {self.target_clock_s}; last clock was {self.last_clock_s}"
                        )
                    if self.clock_regressions:
                        raise RuntimeError(f"bag replay /clock moved backwards {self.clock_regressions} time(s)")
                    self.replay_drained = True
                    self._publish_final_map()
                    return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                raise RuntimeError("offline SLAM observer did not observe /clock")
            if now >= wall_deadline:
                raise RuntimeError("offline SLAM replay did not complete within the bounded wall-time guard")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        odom_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "quaternion_valid", "frame_id"]
        raw_rows = [{**row, "frame_id": "odom"} for row in self.odom_rows]
        with (self.output_dir / "slam_odom_poses.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=odom_fields)
            writer.writeheader()
            writer.writerows(raw_rows)

        transform_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "parent_frame_id", "child_frame_id"]
        with (self.output_dir / "map_to_odom.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=transform_fields)
            writer.writeheader()
            writer.writerows(sorted(self.map_to_odom_rows, key=lambda row: float(row["timestamp_s"])))

        self.map_pose_rows = []
        uncorrected_odom_count = 0
        for odom in self.odom_rows:
            transform = _interpolate_map_to_odom(self.map_to_odom_rows, float(odom["timestamp_s"]))
            if transform is None:
                uncorrected_odom_count += 1
                continue
            try:
                self.map_pose_rows.append(_correct_odom_pose(odom, transform))
            except ValueError:
                uncorrected_odom_count += 1

        map_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"]
        for filename in ("slam_map_poses.csv", "slam_poses.csv"):
            with (self.output_dir / filename).open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=map_fields)
                writer.writeheader()
                writer.writerows(self.map_pose_rows)

        _write_pcd(self.output_dir / "slam_map.pcd", self.latest_map)
        _write_ply(self.output_dir / "slam_map.ply", self.latest_map)
        map_pose_correction_complete = bool(self.map_pose_rows) and uncorrected_odom_count == 0 and len(self.map_pose_rows) == len(self.odom_rows)
        files = []
        for name in ("slam_map_poses.csv", "slam_odom_poses.csv", "map_to_odom.csv", "slam_poses.csv"):
            path = self.output_dir / name
            files.append({
                "path": name,
                "size_bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            })
        processed_sensor_span = (
            self.expected_sensor_last_stamp_s is not None
            and self.last_odom_stamp_s is not None
            and self.last_odom_stamp_s >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
        )
        result = {
            "status": "complete" if self.odom_rows and self.latest_map and self.replay_complete_signal_observed and self.clock_target_reached and self.replay_drained and self.publish_map_acknowledged and self.drain_complete and not self.clock_regressions and processed_sensor_span and self.final_map_span and self.mapper_database_span and map_pose_correction_complete else "incomplete",
            "odom_sample_count": len(self.odom_rows),
            "map_pose_frame_id": "map",
            "raw_odom_frame_id": "odom",
            "map_pose_sample_count": len(self.map_pose_rows),
            "uncorrected_odom_sample_count": uncorrected_odom_count,
            "map_pose_correction_complete": map_pose_correction_complete,
            "map_to_odom_sample_count": len(self.map_to_odom_rows),
            "files": files,
            "map_message_count": self.map_messages,
            "map_point_count": len(self.latest_map),
            "first_clock_s": self.first_clock_s,
            "last_clock_s": self.last_clock_s,
            "target_clock_s": self.target_clock_s,
            "clock_target_reached": self.clock_target_reached,
            "clock_regressions": self.clock_regressions,
            "replay_complete_signal_observed": self.replay_complete_signal_observed,
            "drain_complete": self.drain_complete,
            "drain_quiet_polls": self.drain_quiet_polls,
            "replay_drained": self.replay_drained,
            "publish_map_acknowledged": self.publish_map_acknowledged,
            "map_messages_before_publish": self.map_messages_before_publish,
            "final_map_span": self.final_map_span,
            "mapper_database_span": self.mapper_database_span,
            "database_node_count": self.database_node_count,
            "database_last_stamp_s": self.database_last_stamp_s,
            "last_map_stamp_s": self.map_stamp_s,
            "last_odom_stamp_s": self.last_odom_stamp_s,
            "expected_sensor_last_stamp_s": self.expected_sensor_last_stamp_s,
            "sensor_scan_period_s": self.sensor_scan_period_s,
            "processed_sensor_span": processed_sensor_span,
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
    parser.add_argument("--replay-complete-signal", required=True)
    parser.add_argument("--expected-sensor-last-stamp-seconds", required=True, type=float)
    parser.add_argument("--sensor-scan-period-seconds", required=True, type=float)
    parser.add_argument("--database-path", required=True)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    observer = SlamObserver(Path(args.output_dir), args.duration_seconds, args.startup_timeout_seconds)
    observer.replay_complete_signal = Path(args.replay_complete_signal)
    observer.expected_sensor_last_stamp_s = args.expected_sensor_last_stamp_seconds
    observer.sensor_scan_period_s = args.sensor_scan_period_seconds
    observer.database_path = Path(args.database_path)
    error: BaseException | None = None
    result: dict[str, object] | None = None
    try:
        observer.spin_until_done()
    except BaseException as exc:
        error = exc
    finally:
        try:
            result = observer.close()
        except BaseException as exc:
            if error is None:
                error = exc
            elif hasattr(error, "add_note"):
                error.add_note(f"Offline observer close also failed: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
    if result is not None:
        print(json.dumps(result, indent=2))
    if error is not None:
        raise error
    if result is None or result["status"] != "complete":
        raise RuntimeError(f"offline SLAM did not complete replay, clock drain, odometry and map capture: {result}")


if __name__ == "__main__":
    main()
