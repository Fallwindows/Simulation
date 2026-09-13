"""Record the native ROS RGB camera stream to a compact MP4."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


RGB_TOPIC = "/sim/camera/rgb/image_raw"
CLOCK_TOPIC = "/clock"
SIM_CLOCK_START_TOLERANCE_S = 1.0
POST_TARGET_GRACE_S = 1.0


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0


def _clock_stamp(message) -> float:
    return float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0


def _decode_image(message) -> np.ndarray | None:
    """Decode a ROS Image while honoring encoding, width, height, and step."""

    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0 or step <= 0:
        return None
    encoding = str(message.encoding).lower()
    channel_map = {
        "mono8": 1,
        "8uc1": 1,
        "bgr8": 3,
        "rgb8": 3,
        "8uc3": 3,
        "bgra8": 4,
        "rgba8": 4,
        "8uc4": 4,
    }
    channels = channel_map.get(encoding)
    if channels is None:
        return None
    packed_row_bytes = width * channels
    if step < packed_row_bytes:
        return None
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required_bytes = step * height
    if data.size < required_bytes:
        return None
    rows = data[:required_bytes].reshape((height, step))
    packed = rows[:, :packed_row_bytes].reshape((height, width, channels))
    if channels == 1:
        return cv2.cvtColor(packed, cv2.COLOR_GRAY2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(packed, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(packed, cv2.COLOR_BGRA2BGR)
    return packed


class RgbVideoRecorder:
    def __init__(
        self,
        output: Path,
        metadata_path: Path,
        duration_s: float,
        startup_timeout_s: float,
        frames_path: Path | None = None,
        camera_info_path: Path | None = None,
    ):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo
        from sensor_msgs.msg import Image
        from rosgraph_msgs.msg import Clock

        self.rclpy = rclpy
        self.node = Node("grocery_sim_rgb_video_recorder")
        self.output = output
        self.metadata_path = metadata_path
        self.frames_path = frames_path
        self.camera_info_path = camera_info_path
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.target_s: float | None = None
        self.post_target_deadline: float | None = None
        self.first_image_stamp_s: float | None = None
        self.last_image_stamp_s: float | None = None
        self.last_clock_s: float | None = None
        self.frames_written = 0
        self.invalid_frames = 0
        self.writer: cv2.VideoWriter | None = None
        self.codec = "avc1"
        self.width: int | None = None
        self.height: int | None = None
        self.encoding: str | None = None
        self.done_reason = "not_started"
        self.frame_records: list[dict[str, object]] = []
        self.camera_info_record: dict[str, object] | None = None
        self.node.create_subscription(Image, RGB_TOPIC, self._on_image, 5)
        self.node.create_subscription(CameraInfo, "/sim/camera/rgb/camera_info", self._on_camera_info, 10)
        self.node.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, 20)

    def _on_clock(self, message) -> None:
        self.last_clock_s = _clock_stamp(message)
        if self.first_clock_s is None:
            self.first_clock_s = self.last_clock_s
            if self.first_clock_s <= SIM_CLOCK_START_TOLERANCE_S:
                self.target_s = self.duration_s
            else:
                self.target_s = self.first_clock_s + self.duration_s
        if self.target_s is not None and self.last_clock_s >= self.target_s - 1e-3 and self.post_target_deadline is None:
            self.post_target_deadline = time.monotonic() + POST_TARGET_GRACE_S
            self.done_reason = "simulation_time_reached"

    def _open_writer(self, frame: np.ndarray) -> bool:
        self.height, self.width = frame.shape[:2]
        self.output.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        self.writer = cv2.VideoWriter(str(self.output), fourcc, 30.0, (self.width, self.height))
        if not self.writer.isOpened():
            self.codec = "mp4v"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(str(self.output), fourcc, 30.0, (self.width, self.height))
        return self.writer.isOpened()

    def _on_camera_info(self, message) -> None:
        if self.camera_info_record is not None:
            return
        stamp = _stamp(message)
        self.camera_info_record = {
            "topic": "/sim/camera/rgb/camera_info",
            "stamp_s": stamp,
            "frame_id": str(message.header.frame_id),
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": str(message.distortion_model),
            "d": [float(value) for value in message.d],
            "k": [float(value) for value in message.k],
            "r": [float(value) for value in message.r],
            "p": [float(value) for value in message.p],
        }

    def _on_image(self, message) -> None:
        stamp = _stamp(message)
        if self.last_image_stamp_s is not None and stamp <= self.last_image_stamp_s:
            return
        if self.target_s is not None and stamp > self.target_s + 0.05:
            return
        frame = _decode_image(message)
        if frame is None:
            self.invalid_frames += 1
            return
        if self.writer is None and not self._open_writer(frame):
            self.done_reason = "video_writer_unavailable"
            return
        assert self.writer is not None
        self.writer.write(frame)
        self.last_image_stamp_s = stamp
        if self.first_image_stamp_s is None:
            self.first_image_stamp_s = stamp
            self.encoding = str(message.encoding)
        self.frames_written += 1
        self.frame_records.append({
            "frame_index": self.frames_written - 1,
            "stamp_s": stamp,
            "frame_id": str(message.header.frame_id),
            "width": int(message.width),
            "height": int(message.height),
            "encoding": str(message.encoding),
        })

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            now = time.monotonic()
            if self.post_target_deadline is not None and now >= self.post_target_deadline:
                break
            if self.post_target_deadline is None and now - self.started_wall >= self.startup_timeout_s:
                self.done_reason = "startup_timeout"
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        if self.writer is not None:
            self.writer.release()
        file_size = self.output.stat().st_size if self.output.exists() else 0
        duration_s = None
        actual_fps = None
        if self.first_image_stamp_s is not None and self.last_image_stamp_s is not None:
            duration_s = self.last_image_stamp_s - self.first_image_stamp_s
            if duration_s > 0 and self.frames_written > 1:
                actual_fps = (self.frames_written - 1) / duration_s
        complete = self.frames_written > 0 and self.done_reason == "simulation_time_reached" and file_size > 0
        metadata = {
            "status": "complete" if complete else "failed",
            "topic": RGB_TOPIC,
            "codec": self.codec if self.writer is not None else None,
            "width": self.width,
            "height": self.height,
            "nominal_fps": 30.0,
            "actual_fps_from_ros_timestamps": actual_fps,
            "frame_count": self.frames_written,
            "duration_s": duration_s,
            "first_image_stamp_s": self.first_image_stamp_s,
            "last_image_stamp_s": self.last_image_stamp_s,
            "target_sim_time_s": self.target_s,
            "last_clock_s": self.last_clock_s,
            "encoding": self.encoding,
            "invalid_frames": self.invalid_frames,
            "file_size_bytes": file_size,
            "completion_reason": self.done_reason,
        }
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        if self.frames_path is not None:
            self.frames_path.parent.mkdir(parents=True, exist_ok=True)
            with self.frames_path.open("w", encoding="utf-8", newline="\n") as handle:
                for record in self.frame_records:
                    handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
        if self.camera_info_path is not None and self.camera_info_record is not None:
            self.camera_info_path.parent.mkdir(parents=True, exist_ok=True)
            self.camera_info_path.write_text(json.dumps(self.camera_info_record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if not complete:
            raise RuntimeError(f"RGB video recording failed: {metadata}")
        return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--startup-timeout-seconds", type=float, default=90.0)
    parser.add_argument("--frames-jsonl", default="")
    parser.add_argument("--camera-info-json", default="")
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    recorder = RgbVideoRecorder(
        Path(args.output),
        Path(args.metadata),
        args.duration_seconds,
        args.startup_timeout_seconds,
        Path(args.frames_jsonl) if args.frames_jsonl else None,
        Path(args.camera_info_json) if args.camera_info_json else None,
    )
    try:
        recorder.spin_until_done()
        recorder.close()
    finally:
        recorder.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
