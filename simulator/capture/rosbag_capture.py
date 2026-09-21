"""Record the raw sensor ROS graph without policy-blocked aggregate imports.

Generated ROS types are loaded through their canonical module names while the
eager ``msg.__init__`` modules are replaced by narrow package shims. This keeps
the native type-support lifetime and identity expected by rclpy without loading
CameraInfo and its NumPy dependency on this Windows host.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.machinery
import json
import struct
import sys
import time
import types
from pathlib import Path


TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
    "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}


class _DisabledTypeDescriptionService:
    def __init__(self, _node):
        pass

    def destroy(self) -> None:
        pass


def _message_package(package_name: str):
    package = importlib.import_module(package_name)
    message_dir = Path(package.__file__).resolve().parent / "msg"
    qualified_name = f"{package_name}.msg"
    message_package = sys.modules.get(qualified_name)
    if message_package is None:
        message_package = types.ModuleType(qualified_name)
        message_package.__file__ = str(message_dir / "__init__.py")
        message_package.__package__ = qualified_name
        message_package.__path__ = [str(message_dir)]
        message_package.__spec__ = importlib.machinery.ModuleSpec(
            qualified_name, loader=None, is_package=True
        )
        message_package.__spec__.submodule_search_locations = [str(message_dir)]
        sys.modules[qualified_name] = message_package
        setattr(package, "msg", message_package)
    elif not hasattr(message_package, "__path__"):
        raise RuntimeError(f"Existing {qualified_name} module is not a package")
    return message_package


def _generated_type(message_package, module_name: str, class_name: str):
    module = importlib.import_module(f"{message_package.__name__}.{module_name}")
    message_type = getattr(module, class_name)
    setattr(message_package, class_name, message_type)
    return message_type


def _load_ros_message_types() -> dict[str, type]:
    """Load only the generated types needed by the raw bag contract."""

    sensor = _message_package("sensor_msgs")
    _generated_type(sensor, "_point_field", "PointField")
    Image = _generated_type(sensor, "_image", "Image")
    PointCloud2 = _generated_type(sensor, "_point_cloud2", "PointCloud2")

    rosgraph = _message_package("rosgraph_msgs")
    Clock = _generated_type(rosgraph, "_clock", "Clock")

    geometry = _message_package("geometry_msgs")
    _generated_type(geometry, "_vector3", "Vector3")
    _generated_type(geometry, "_quaternion", "Quaternion")
    _generated_type(geometry, "_transform", "Transform")
    _generated_type(geometry, "_transform_stamped", "TransformStamped")
    tf2 = _message_package("tf2_msgs")
    TFMessage = _generated_type(tf2, "_tf_message", "TFMessage")
    return {
        "/clock": Clock,
        "/sim/camera/rgb/image_raw": Image,
        "/sim/lidar/points": PointCloud2,
        "/tf": TFMessage,
        "/tf_static": TFMessage,
    }


def _create_observer_node():
    import rclpy.node as node_module

    original_service = node_module.TypeDescriptionService
    node_module.TypeDescriptionService = _DisabledTypeDescriptionService
    try:
        return node_module.Node(
            "grocery_sim_raw_capture_observer",
            start_parameter_services=False,
        )
    finally:
        node_module.TypeDescriptionService = original_service


def _stamp_s(message) -> float | None:
    if hasattr(message, "clock"):
        stamp = message.clock
    elif hasattr(message, "header"):
        stamp = message.header.stamp
    elif hasattr(message, "transforms") and message.transforms:
        stamp = message.transforms[0].header.stamp
    else:
        return None
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _cdr_time(payload: bytes, offset: int) -> float | None:
    """Decode a builtin_interfaces/Time at *offset* in a CDR1 payload."""

    if len(payload) < offset + 8 or len(payload) < 2:
        return None
    representation = payload[:2]
    if representation == b"\x00\x01":
        byte_order = "<"
    elif representation == b"\x00\x00":
        byte_order = ">"
    else:
        return None
    seconds, nanoseconds = struct.unpack_from(f"{byte_order}iI", payload, offset)
    if nanoseconds >= 1_000_000_000:
        return None
    return float(seconds) + float(nanoseconds) / 1_000_000_000.0


def _serialized_stamp_s(topic: str, payload: bytes, last_clock_s: float | None) -> float | None:
    if topic == "/clock":
        return _cdr_time(payload, 4)
    if topic in ("/sim/camera/rgb/image_raw", "/sim/lidar/points"):
        return _cdr_time(payload, 4)
    if topic in ("/tf", "/tf_static"):
        if len(payload) < 8:
            return None
        representation = payload[:2]
        byte_order = "<" if representation == b"\x00\x01" else ">" if representation == b"\x00\x00" else None
        if byte_order is None or struct.unpack_from(f"{byte_order}I", payload, 4)[0] == 0:
            return last_clock_s
        return _cdr_time(payload, 8)
    return None


def _bag_observations(
    output: Path,
    rosbag2_py,
) -> tuple[dict[str, int], dict[str, float | None], dict[str, float | None]]:
    counts = {topic: 0 for topic in TOPIC_TYPES}
    first_stamps = {topic: None for topic in TOPIC_TYPES}
    last_stamps = {topic: None for topic in TOPIC_TYPES}
    last_clock_s: float | None = None
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    while reader.has_next():
        topic, payload, _record_timestamp_ns = reader.read_next()
        if topic not in counts:
            continue
        stamp_s = _serialized_stamp_s(topic, bytes(payload), last_clock_s)
        if topic == "/clock" and stamp_s is not None:
            last_clock_s = stamp_s if last_clock_s is None else max(last_clock_s, stamp_s)
            stamp_s = last_clock_s
        counts[topic] += 1
        if first_stamps[topic] is None:
            first_stamps[topic] = stamp_s
        if stamp_s is not None:
            last_stamps[topic] = stamp_s
    return counts, first_stamps, last_stamps


class RawCaptureWriter:
    def __init__(
        self,
        output: Path,
        duration_s: float,
        startup_timeout_s: float,
        post_target_wall_s: float = 2.0,
        capture_end_clock_s: float | None = None,
        clock_stall_timeout_s: float = 30.0,
    ):
        import rclpy
        import rosbag2_py
        from rclpy.serialization import serialize_message

        message_types = _load_ros_message_types()
        self.rclpy = rclpy
        self.rosbag2_py = rosbag2_py
        self.serialize_message = serialize_message
        self.output = Path(output)
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.post_target_wall_s = float(post_target_wall_s)
        self.capture_end_clock_s = None if capture_end_clock_s is None else float(capture_end_clock_s)
        self.clock_stall_timeout_s = float(clock_stall_timeout_s)
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if self.capture_end_clock_s is not None and self.capture_end_clock_s <= 0.0:
            raise ValueError("capture_end_clock_s must be positive")
        if self.clock_stall_timeout_s <= 0.0:
            raise ValueError("clock_stall_timeout_s must be positive")
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        self.last_clock_progress_wall: float | None = None
        self.target_clock_s: float | None = None
        self.target_wall_deadline: float | None = None
        self.target_reached = False
        self.failure_reason: str | None = None
        self._closed_result: dict[str, object] | None = None

        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.output), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        for topic_id, (topic, type_name) in enumerate(TOPIC_TYPES.items()):
            self.writer.create_topic(rosbag2_py.TopicMetadata(topic_id, topic, type_name, "cdr", []))

        self.node = _create_observer_node()
        self.subscriptions = []
        for topic, message_type in message_types.items():
            qos = 100 if topic in ("/clock", "/tf", "/tf_static") else 10
            self.subscriptions.append(
                self.node.create_subscription(message_type, topic, self._callback(topic), qos)
            )

    def _callback(self, topic: str):
        def receive(message) -> None:
            stamp_s = _stamp_s(message)
            if stamp_s is None:
                stamp_s = self.last_clock_s or 0.0
            if topic == "/clock":
                if self.last_clock_s is None or stamp_s > self.last_clock_s:
                    self.last_clock_progress_wall = time.monotonic()
                self.last_clock_s = stamp_s if self.last_clock_s is None else max(self.last_clock_s, stamp_s)
                if self.first_clock_s is None:
                    self.first_clock_s = stamp_s
                    self.target_clock_s = (
                        self.capture_end_clock_s
                        if self.capture_end_clock_s is not None
                        else stamp_s + self.duration_s
                    )
                if self.target_clock_s is not None and stamp_s >= self.target_clock_s and self.target_wall_deadline is None:
                    self.target_reached = True
                    self.target_wall_deadline = time.monotonic() + self.post_target_wall_s
            self.writer.write(topic, self.serialize_message(message), int(round(stamp_s * 1_000_000_000.0)))

        return receive

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            now = time.monotonic()
            if self.target_wall_deadline is not None and now >= self.target_wall_deadline:
                return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                self.failure_reason = "clock_startup_timeout"
                raise RuntimeError("raw capture did not observe /clock before startup timeout")
            if (
                self.first_clock_s is not None
                and not self.target_reached
                and self.last_clock_progress_wall is not None
                and now - self.last_clock_progress_wall >= self.clock_stall_timeout_s
            ):
                self.failure_reason = "clock_stalled_before_target"
                raise RuntimeError(
                    "raw capture /clock stopped before the requested simulation-time target "
                    f"({self.last_clock_s} < {self.target_clock_s})"
                )
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        if self._closed_result is not None:
            return self._closed_result
        for subscription in getattr(self, "subscriptions", []):
            self.node.destroy_subscription(subscription)
        self.subscriptions = []
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None
        if getattr(self, "writer", None) is not None:
            close_method = getattr(self.writer, "close", None)
            if callable(close_method):
                close_method()
            self.writer = None

        counts, first_stamps, last_stamps = _bag_observations(self.output, self.rosbag2_py)
        missing = [topic for topic, count in counts.items() if count == 0]
        complete = not missing and self.target_reached
        self._closed_result = {
            "status": "complete" if complete else "incomplete",
            "uri": self.output.name,
            "storage_id": "sqlite3",
            "topics": list(TOPIC_TYPES),
            "topic_types": dict(TOPIC_TYPES),
            "counts": counts,
            "first_stamp_s": first_stamps,
            "last_stamp_s": last_stamps,
            "first_clock_s": first_stamps["/clock"],
            "last_clock_s": last_stamps["/clock"],
            "target_clock_s": self.target_clock_s,
            "target_reached": self.target_reached,
            "capture_window_mode": "absolute_simulation_horizon" if self.capture_end_clock_s is not None else "duration_after_first_clock",
            "completion_clock_source": "/clock",
            "missing_topics": missing,
            "failure_reason": self.failure_reason,
            "numpy_loaded": "numpy" in sys.modules,
        }
        return self._closed_result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--startup-timeout-seconds", type=float, default=120.0)
    parser.add_argument("--post-target-wall-seconds", type=float, default=2.0)
    parser.add_argument("--end-clock-seconds", type=float)
    parser.add_argument("--clock-stall-timeout-seconds", type=float, default=30.0)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()
    import rclpy

    writer = None
    rclpy.init()
    try:
        writer = RawCaptureWriter(
            Path(args.output),
            args.duration_seconds,
            args.startup_timeout_seconds,
            args.post_target_wall_seconds,
            args.end_clock_seconds,
            args.clock_stall_timeout_seconds,
        )
        try:
            writer.spin_until_done()
        except Exception:
            result = writer.close()
            Path(args.metadata).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            raise
        result = writer.close()
        Path(args.metadata).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if result["status"] != "complete":
            raise RuntimeError(
                "raw capture is incomplete: "
                f"target_reached={result['target_reached']}, "
                f"missing_topics={result['missing_topics']}"
            )
    finally:
        if writer is not None:
            writer.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
