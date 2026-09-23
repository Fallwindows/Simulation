"""Write the raw sensor ROS graph to a clean rosbag2 from Pixi's ROS runtime.

This avoids shelling out to ``ros2 bag record`` and gives the capture launcher
a deterministic simulation-time completion condition and a clean writer
shutdown on Windows.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path


TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
    "/sim/camera/rgb/camera_info": "sensor_msgs/msg/CameraInfo",
    "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
WRITER_CLOSE_TIMEOUT_S = 5.0


def _bounded_resource_close(resource, method_name: str, timeout_s: float) -> BaseException | None:
    """Give one daemon worker sole ownership of a possibly stalled native close."""

    errors: list[BaseException] = []

    def close_owned_resource(owned_resource) -> None:
        try:
            method = getattr(owned_resource, method_name, None)
            if callable(method):
                method()
        except BaseException as exc:
            errors.append(exc)

    worker = threading.Thread(
        target=close_owned_resource,
        args=(resource,),
        name=f"rosbag-writer-{method_name}",
        daemon=True,
    )
    worker.start()
    worker.join(max(0.0, timeout_s))
    if worker.is_alive():
        return TimeoutError(f"rosbag writer {method_name} exceeded {timeout_s:.3f}s close deadline")
    return errors[0] if errors else None


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


class RawCaptureWriter:
    def __init__(self, output: Path, duration_s: float, startup_timeout_s: float, post_target_wall_s: float = 2.0):
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from tf2_msgs.msg import TFMessage
        import rosbag2_py
        from rclpy.serialization import serialize_message

        self.rclpy = rclpy
        self.serialize_message = serialize_message
        self.output = output
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.post_target_wall_s = float(post_target_wall_s)
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        self.target_clock_s: float | None = None
        self.target_wall_deadline: float | None = None
        self.counts = {topic: 0 for topic in TOPIC_TYPES}
        self.first_stamp_s: dict[str, float | None] = {topic: None for topic in TOPIC_TYPES}
        self.last_stamp_s: dict[str, float | None] = {topic: None for topic in TOPIC_TYPES}
        self._closed = False
        self._close_result: dict[str, object] | None = None
        self._close_error: BaseException | None = None
        self._callback_error: BaseException | None = None
        self.close_timeout_s = WRITER_CLOSE_TIMEOUT_S
        self.node = Node("grocery_sim_raw_capture_writer")
        self.writer = rosbag2_py.SequentialWriter()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.writer.open(
            rosbag2_py.StorageOptions(uri=str(self.output), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        for topic_id, (topic, type_name) in enumerate(TOPIC_TYPES.items()):
            self.writer.create_topic(rosbag2_py.TopicMetadata(topic_id, topic, type_name, "cdr", []))

        message_types = {
            "/clock": Clock,
            "/sim/camera/rgb/image_raw": Image,
            "/sim/camera/rgb/camera_info": CameraInfo,
            "/sim/lidar/points": PointCloud2,
            "/tf": TFMessage,
            "/tf_static": TFMessage,
        }
        for topic, message_type in message_types.items():
            qos = 100 if topic in ("/clock", "/tf", "/tf_static") else 10
            self.node.create_subscription(message_type, topic, self._callback(topic), qos)

    def _callback(self, topic: str):
        def receive(message) -> None:
            stamp_s = _stamp_s(message)
            if stamp_s is None:
                stamp_s = self.last_clock_s or 0.0
            if topic == "/clock":
                # Isaac may emit a few zero-time clock messages while the
                # bridge/render graph is starting or tearing down.  They are
                # valid raw messages, but must not move completion metadata
                # backwards after simulation has advanced.
                self.last_clock_s = stamp_s if self.last_clock_s is None else max(self.last_clock_s, stamp_s)
                if self.first_clock_s is None:
                    self.first_clock_s = stamp_s
                    self.target_clock_s = stamp_s + self.duration_s
                if self.target_clock_s is not None and stamp_s >= self.target_clock_s and self.target_wall_deadline is None:
                    self.target_wall_deadline = time.monotonic() + self.post_target_wall_s
            try:
                self.writer.write(topic, self.serialize_message(message), int(round(stamp_s * 1_000_000_000.0)))
            except BaseException as exc:
                self._callback_error = exc
                raise
            self.counts[topic] += 1
            if self.first_stamp_s[topic] is None:
                self.first_stamp_s[topic] = stamp_s
            if topic == "/clock" and self.last_stamp_s[topic] is not None:
                self.last_stamp_s[topic] = max(float(self.last_stamp_s[topic]), stamp_s)
            else:
                self.last_stamp_s[topic] = stamp_s

        return receive

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            now = time.monotonic()
            if self.target_wall_deadline is not None and now >= self.target_wall_deadline:
                return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                raise RuntimeError("raw capture did not observe /clock before startup timeout")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        if self._closed:
            if self._close_error is not None:
                raise RuntimeError("raw capture writer close failed") from self._close_error
            if self._close_result is None:
                raise RuntimeError("raw capture writer was closed without complete metadata")
            return self._close_result
        # rosbag2_py closes the storage backend when the writer is released;
        # delete the node first so no callback can race the final metadata.
        node_error: BaseException | None = None
        try:
            self.node.destroy_node()
        except BaseException as exc:
            node_error = exc
        writer = self.writer
        close_error = _bounded_resource_close(writer, "close", self.close_timeout_s)
        self.writer = None
        self._closed = True
        if self._callback_error is not None or node_error is not None or close_error is not None:
            self._close_error = self._callback_error or node_error or close_error
            raise RuntimeError("raw capture did not finish cleanly; writer/node was closed") from self._close_error
        result = {
            "status": "complete",
            "uri": self.output.name,
            "storage_id": "sqlite3",
            "topics": list(TOPIC_TYPES),
            "counts": self.counts,
            "first_stamp_s": self.first_stamp_s,
            "last_stamp_s": self.last_stamp_s,
            "first_clock_s": self.first_clock_s,
            "last_clock_s": self.last_clock_s,
            "target_clock_s": self.target_clock_s,
        }
        self._close_result = result
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--startup-timeout-seconds", type=float, default=120.0)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    writer: RawCaptureWriter | None = None
    error: BaseException | None = None
    try:
        writer = RawCaptureWriter(Path(args.output), args.duration_seconds, args.startup_timeout_seconds)
        writer.spin_until_done()
        result = writer.close()
        Path(args.metadata).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except BaseException as exc:
        error = exc
    finally:
        if writer is not None and not writer._closed:
            try:
                writer.close()
            except BaseException as close_error:
                if error is None:
                    error = close_error
                elif hasattr(error, "add_note"):
                    error.add_note(f"Raw bag cleanup also failed: {close_error}")
        if rclpy.ok():
            rclpy.shutdown()
    if error is not None:
        raise error


if __name__ == "__main__":
    main()
