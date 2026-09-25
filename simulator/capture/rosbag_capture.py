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
DEFAULT_PROGRESS_TIMEOUT_S = 60.0
TARGET_TOLERANCE_S = 1e-3
RAW_IMAGE_QOS_DEPTH = 128
ROSBAG_CACHE_BYTES = 512 * 1024 * 1024


def _maximum_nearest_skew_s(reference_stamps: list[float], candidate_stamps: list[float]) -> float | None:
    """Return the worst nearest-neighbour timestamp skew for sorted samples."""

    if not reference_stamps or not candidate_stamps:
        return None
    candidates = sorted(candidate_stamps)
    cursor = 0
    maximum = 0.0
    for reference in sorted(reference_stamps):
        while cursor + 1 < len(candidates) and abs(candidates[cursor + 1] - reference) <= abs(candidates[cursor] - reference):
            cursor += 1
        maximum = max(maximum, abs(candidates[cursor] - reference))
    return maximum


def _strictly_increasing_stamps(stamps: list[float]) -> bool:
    """Reject duplicate and regressing sensor timestamps."""

    return all(current > previous + 1e-9 for previous, current in zip(stamps, stamps[1:]))


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
    def __init__(
        self,
        output: Path,
        duration_s: float,
        startup_timeout_s: float,
        progress_timeout_s: float = DEFAULT_PROGRESS_TIMEOUT_S,
        post_target_wall_s: float = 5.0,
    ):
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
        self.progress_timeout_s = float(progress_timeout_s)
        self.post_target_wall_s = float(post_target_wall_s)
        self.started_wall = time.monotonic()
        self.last_progress_wall = self.started_wall
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        # duration_s is the absolute scenario horizon.  Offsetting it by the
        # first observed clock makes the target unreachable when subscribers
        # discover the graph after simulation has already advanced.
        self.target_clock_s = self.duration_s
        self.target_wall_deadline: float | None = None
        self.clock_regressions = 0
        self.ignored_teardown_clock_regressions = 0
        self.counts = {topic: 0 for topic in TOPIC_TYPES}
        self.first_stamp_s: dict[str, float | None] = {topic: None for topic in TOPIC_TYPES}
        self.last_stamp_s: dict[str, float | None] = {topic: None for topic in TOPIC_TYPES}
        self.max_stamp_gap_s: dict[str, float | None] = {topic: None for topic in TOPIC_TYPES}
        self.stamp_regressions = {topic: 0 for topic in TOPIC_TYPES}
        self.stamp_nonincreasing = {topic: 0 for topic in TOPIC_TYPES}
        self.rgb_stamps_s: list[float] = []
        self.lidar_stamps_s: list[float] = []
        self.camera_info_frame_ids: set[str] = set()
        self.lidar_frame_ids: set[str] = set()
        self.lidar_point_counts: list[int] = []
        self._closed = False
        self._close_result: dict[str, object] | None = None
        self._close_error: BaseException | None = None
        self._callback_error: BaseException | None = None
        self.close_timeout_s = WRITER_CLOSE_TIMEOUT_S
        self.node = Node("grocery_sim_raw_capture_writer")
        self.writer = rosbag2_py.SequentialWriter()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        storage_options = rosbag2_py.StorageOptions(uri=str(self.output), storage_id="sqlite3")
        # 1080p RGB is about 6 MiB per sample.  A bounded rosbag cache keeps
        # SQLite flush latency out of the ROS callback and the deeper DDS
        # history absorbs short writer stalls without dropping source frames.
        storage_options.max_cache_size = ROSBAG_CACHE_BYTES
        self.writer.open(
            storage_options,
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
            if topic == "/sim/camera/rgb/image_raw":
                qos = RAW_IMAGE_QOS_DEPTH
            elif topic == "/sim/camera/rgb/camera_info":
                qos = RAW_IMAGE_QOS_DEPTH
            elif topic == "/sim/lidar/points":
                qos = 32
            else:
                qos = 256
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
                previous_clock = self.last_clock_s
                if previous_clock is not None and stamp_s < previous_clock - 1e-9:
                    target_clock_s = self.target_clock_s if self.target_clock_s is not None else self.duration_s
                    if previous_clock >= target_clock_s - TARGET_TOLERANCE_S:
                        self.ignored_teardown_clock_regressions = getattr(self, "ignored_teardown_clock_regressions", 0) + 1
                        return
                    self.clock_regressions = getattr(self, "clock_regressions", 0) + 1
                self.last_clock_s = stamp_s if previous_clock is None else max(previous_clock, stamp_s)
                if previous_clock is None or stamp_s > previous_clock + 1e-9:
                    self.last_progress_wall = time.monotonic()
                if self.first_clock_s is None:
                    self.first_clock_s = stamp_s
                    if self.target_clock_s is None:
                        self.target_clock_s = self.duration_s
                if stamp_s >= self.target_clock_s - TARGET_TOLERANCE_S and self.target_wall_deadline is None:
                    self.target_wall_deadline = time.monotonic() + self.post_target_wall_s
            try:
                self.writer.write(topic, self.serialize_message(message), int(round(stamp_s * 1_000_000_000.0)))
            except BaseException as exc:
                self._callback_error = exc
                raise
            self.counts[topic] += 1
            if self.first_stamp_s[topic] is None:
                self.first_stamp_s[topic] = stamp_s
            previous_stamp = self.last_stamp_s[topic]
            if previous_stamp is not None:
                gap = stamp_s - float(previous_stamp)
                if gap <= 1e-9:
                    self.stamp_nonincreasing[topic] += 1
                if gap < -1e-9:
                    self.stamp_regressions[topic] += 1
                elif gap > 1e-9:
                    previous_max = self.max_stamp_gap_s[topic]
                    self.max_stamp_gap_s[topic] = gap if previous_max is None else max(float(previous_max), gap)
            self.last_stamp_s[topic] = stamp_s if previous_stamp is None else max(float(previous_stamp), stamp_s)
            if topic == "/sim/camera/rgb/image_raw":
                self.rgb_stamps_s.append(stamp_s)
            elif topic == "/sim/camera/rgb/camera_info":
                self.camera_info_frame_ids.add(str(message.header.frame_id))
            elif topic == "/sim/lidar/points":
                self.lidar_stamps_s.append(stamp_s)
                self.lidar_frame_ids.add(str(message.header.frame_id))
                self.lidar_point_counts.append(int(message.width) * int(message.height))

        return receive

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            now = time.monotonic()
            if self.target_wall_deadline is not None and now >= self.target_wall_deadline:
                return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                raise RuntimeError("raw capture did not observe /clock before startup timeout")
            if (
                self.first_clock_s is not None
                and self.target_wall_deadline is None
                and now - self.last_progress_wall >= self.progress_timeout_s
            ):
                raise RuntimeError("raw capture clock stalled before absolute scenario target")
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
        target_reached = self.last_clock_s is not None and self.last_clock_s >= self.target_clock_s - TARGET_TOLERANCE_S
        maximum_skew = _maximum_nearest_skew_s(self.lidar_stamps_s, self.rgb_stamps_s)
        all_topics_positive = all(self.counts[topic] > 0 for topic in TOPIC_TYPES)
        lidar_nonempty = len(self.lidar_point_counts) >= 2 and min(self.lidar_point_counts) > 0
        ordered = (
            self.clock_regressions == 0
            and self.stamp_regressions["/sim/camera/rgb/image_raw"] == 0
            and self.stamp_regressions["/sim/lidar/points"] == 0
            and self.stamp_nonincreasing["/sim/camera/rgb/image_raw"] == 0
            and self.stamp_nonincreasing["/sim/lidar/points"] == 0
            and _strictly_increasing_stamps(self.rgb_stamps_s)
            and _strictly_increasing_stamps(self.lidar_stamps_s)
        )
        complete = target_reached and all_topics_positive and lidar_nonempty and ordered
        result = {
            "status": "complete" if complete else "failed",
            "uri": self.output.name,
            "storage_id": "sqlite3",
            "writer_cache_bytes": ROSBAG_CACHE_BYTES,
            "subscriber_qos_depths": {
                "rgb_image": RAW_IMAGE_QOS_DEPTH,
                "camera_info": RAW_IMAGE_QOS_DEPTH,
                "lidar": 32,
                "clock_tf": 256,
            },
            "topics": list(TOPIC_TYPES),
            "counts": self.counts,
            "first_stamp_s": self.first_stamp_s,
            "last_stamp_s": self.last_stamp_s,
            "max_stamp_gap_s": self.max_stamp_gap_s,
            "stamp_regressions": self.stamp_regressions,
            "stamp_nonincreasing": self.stamp_nonincreasing,
            "first_clock_s": self.first_clock_s,
            "last_clock_s": self.last_clock_s,
            "target_clock_s": self.target_clock_s,
            "target_reached": target_reached,
            "clock_regressions": self.clock_regressions,
            "ignored_teardown_clock_regressions": self.ignored_teardown_clock_regressions,
            "camera_info_frame_ids": sorted(self.camera_info_frame_ids),
            "lidar_frame_ids": sorted(self.lidar_frame_ids),
            "lidar_point_count_min": min(self.lidar_point_counts) if self.lidar_point_counts else None,
            "lidar_point_count_max": max(self.lidar_point_counts) if self.lidar_point_counts else None,
            "max_rgb_lidar_skew_s": maximum_skew,
        }
        self._close_result = result
        if not complete:
            self._close_error = RuntimeError(f"raw capture validation failed: {result}")
            raise self._close_error
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--startup-timeout-seconds", type=float, default=600.0)
    parser.add_argument("--progress-timeout-seconds", type=float, default=60.0)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    metadata_path = Path(args.metadata)
    writer: RawCaptureWriter | None = None
    error: BaseException | None = None
    try:
        writer = RawCaptureWriter(
            Path(args.output),
            args.duration_seconds,
            args.startup_timeout_seconds,
            args.progress_timeout_seconds,
        )
        writer.spin_until_done()
        writer.close()
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
    # Preserve the exact failed receipt as well as successful metadata.  The
    # production wrapper still rejects status=failed, but diagnostics must not
    # lose the stream counts/cadence that explain the failure.
    close_result = None if writer is None else getattr(writer, "_close_result", None)
    if close_result is not None:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(close_result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if error is not None:
        raise error


if __name__ == "__main__":
    main()
