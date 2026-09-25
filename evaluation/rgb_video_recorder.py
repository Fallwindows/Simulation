"""Record the native ROS RGB camera stream to a compact MP4."""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np


RGB_TOPIC = "/sim/camera/rgb/image_raw"
CLOCK_TOPIC = "/clock"
POST_TARGET_GRACE_S = 5.0
WRITER_CLOSE_TIMEOUT_S = 5.0
DEFAULT_PROGRESS_TIMEOUT_S = 60.0
NOMINAL_FPS = 30.0
FRAME_PERIOD_S = 1.0 / NOMINAL_FPS
FRAME_GAP_TOLERANCE_S = 1e-6
RGB_SUBSCRIPTION_DEPTH = 64


def _bounded_resource_close(resource, method_name: str, timeout_s: float) -> BaseException | None:
    """Give one daemon worker sole ownership of a possibly stalled native close."""

    finished = threading.Event()
    errors: list[BaseException] = []

    def close_owned_resource(owned_resource) -> None:
        try:
            method = getattr(owned_resource, method_name, None)
            if callable(method):
                method()
        except BaseException as exc:
            errors.append(exc)
        finally:
            finished.set()

    worker = threading.Thread(
        target=close_owned_resource,
        args=(resource,),
        name=f"rgb-video-{method_name}",
        daemon=True,
    )
    worker.start()
    worker.join(max(0.0, timeout_s))
    if worker.is_alive():
        return TimeoutError(f"VideoWriter.{method_name} exceeded {timeout_s:.3f}s close deadline")
    return errors[0] if errors else None


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
    # rclpy's uint8 sequence supports the buffer protocol; avoid a full 6 MiB
    # Python bytes copy for every native 1080p frame.
    data = np.frombuffer(message.data, dtype=np.uint8)
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
        progress_timeout_s: float = DEFAULT_PROGRESS_TIMEOUT_S,
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
        self.progress_timeout_s = float(progress_timeout_s)
        self.started_wall = time.monotonic()
        self.last_progress_wall = self.started_wall
        self.first_clock_s: float | None = None
        self.target_s: float | None = None
        self.post_target_deadline: float | None = None
        self.first_image_stamp_s: float | None = None
        self.last_image_stamp_s: float | None = None
        self.last_clock_s: float | None = None
        self.frames_written = 0
        self.invalid_frames = 0
        self.nonincreasing_frames = 0
        self.clock_regressions = 0
        self.max_frame_gap_s: float | None = None
        self.frame_ids: set[str] = set()
        self.writer: cv2.VideoWriter | None = None
        self.codec = "avc1"
        self.width: int | None = None
        self.height: int | None = None
        self.encoding: str | None = None
        self.done_reason = "not_started"
        self.frame_records: list[dict[str, object]] = []
        self.camera_info_record: dict[str, object] | None = None
        self._closed = False
        self._close_metadata: dict[str, object] | None = None
        self.close_timeout_s = WRITER_CLOSE_TIMEOUT_S
        self.node.create_subscription(Image, RGB_TOPIC, self._on_image, RGB_SUBSCRIPTION_DEPTH)
        self.node.create_subscription(CameraInfo, "/sim/camera/rgb/camera_info", self._on_camera_info, 64)
        self.node.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, 256)

    def _on_clock(self, message) -> None:
        stamp_s = _clock_stamp(message)
        previous_clock = self.last_clock_s
        if previous_clock is not None and stamp_s < previous_clock - 1e-9:
            if self.post_target_deadline is not None:
                return
            self.clock_regressions += 1
        self.last_clock_s = stamp_s if previous_clock is None else max(previous_clock, stamp_s)
        if previous_clock is None or stamp_s > previous_clock + 1e-9:
            self.last_progress_wall = time.monotonic()
        if self.first_clock_s is None:
            self.first_clock_s = stamp_s
            # duration_s is the absolute scenario horizon.  Subscriber
            # discovery latency must not extend the requested simulation.
            self.target_s = self.duration_s
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
            self.nonincreasing_frames += 1
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
        if self.last_image_stamp_s is not None:
            gap_s = stamp - self.last_image_stamp_s
            self.max_frame_gap_s = gap_s if self.max_frame_gap_s is None else max(self.max_frame_gap_s, gap_s)
        self.last_image_stamp_s = stamp
        if self.first_image_stamp_s is None:
            self.first_image_stamp_s = stamp
            self.encoding = str(message.encoding)
        self.frames_written += 1
        self.frame_ids.add(str(message.header.frame_id))
        self.last_progress_wall = time.monotonic()
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
            awaiting_first_samples = self.first_clock_s is None or self.first_image_stamp_s is None
            if awaiting_first_samples and now - self.started_wall >= self.startup_timeout_s:
                self.done_reason = "startup_timeout"
                break
            if (
                not awaiting_first_samples
                and self.post_target_deadline is None
                and now - self.last_progress_wall >= self.progress_timeout_s
            ):
                self.done_reason = "progress_timeout"
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        if self._closed:
            if self._close_metadata is None:
                raise RuntimeError("RGB recorder was closed without metadata")
            if self._close_metadata["status"] != "complete":
                raise RuntimeError(f"RGB video recording failed: {self._close_metadata}")
            return self._close_metadata
        release_error: BaseException | None = None
        if self.writer is not None:
            writer = self.writer
            release_error = _bounded_resource_close(writer, "release", self.close_timeout_s)
            self.writer = None
            if release_error is not None:
                self.done_reason = "video_writer_release_failed"
        file_size = self.output.stat().st_size if self.output.exists() else 0
        duration_s = None
        actual_fps = None
        if self.first_image_stamp_s is not None and self.last_image_stamp_s is not None:
            duration_s = self.last_image_stamp_s - self.first_image_stamp_s
            if duration_s > 0 and self.frames_written > 1:
                actual_fps = (self.frames_written - 1) / duration_s
        max_frame_gap_s = getattr(self, "max_frame_gap_s", None)
        nonincreasing_frames = getattr(self, "nonincreasing_frames", 0)
        clock_regressions = getattr(self, "clock_regressions", 0)
        camera_info_record = getattr(self, "camera_info_record", None)
        frame_ids = getattr(self, "frame_ids", set())
        cadence_contiguous = (
            self.frames_written >= 2
            and nonincreasing_frames == 0
            and max_frame_gap_s is not None
            and max_frame_gap_s <= FRAME_PERIOD_S + FRAME_GAP_TOLERANCE_S
        )
        horizon_covered = (
            self.first_image_stamp_s is not None
            and self.first_image_stamp_s <= 0.1 + FRAME_GAP_TOLERANCE_S
            and self.last_image_stamp_s is not None
            and self.target_s is not None
            and self.last_image_stamp_s >= self.target_s - FRAME_PERIOD_S - FRAME_GAP_TOLERANCE_S
        )
        camera_info_valid = (
            camera_info_record is not None
            and camera_info_record.get("frame_id") == "camera_optical_frame"
        )
        complete = (
            cadence_contiguous
            and horizon_covered
            and camera_info_valid
            and frame_ids == {"camera_optical_frame"}
            and clock_regressions == 0
            and self.invalid_frames == 0
            and self.done_reason == "simulation_time_reached"
            and file_size > 0
            and release_error is None
        )
        metadata = {
            "status": "complete" if complete else "failed",
            "topic": RGB_TOPIC,
            "codec": self.codec if self.width is not None else None,
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
            "nonincreasing_frames": nonincreasing_frames,
            "max_frame_gap_s": max_frame_gap_s,
            "cadence_contiguous": cadence_contiguous,
            "horizon_covered": horizon_covered,
            "clock_regressions": clock_regressions,
            "frame_ids": sorted(frame_ids),
            "camera_info_frame_id": camera_info_record.get("frame_id") if camera_info_record else None,
            "file_size_bytes": file_size,
            "completion_reason": self.done_reason,
        }
        if release_error is not None:
            metadata["writer_error"] = f"{type(release_error).__name__}: {release_error}"
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
        self._close_metadata = metadata
        self._closed = True
        if not complete:
            raise RuntimeError(f"RGB video recording failed: {metadata}")
        return metadata


def _cleanup_recorder(recorder: RgbVideoRecorder, rclpy) -> BaseException | None:
    """Always release a failed or successful recording after callbacks stop."""

    error: BaseException | None = None
    try:
        recorder.node.destroy_node()
    except BaseException as exc:
        error = exc
    try:
        recorder.close()
    except BaseException as exc:
        if error is None:
            error = exc
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return error


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--startup-timeout-seconds", type=float, default=600.0)
    parser.add_argument("--progress-timeout-seconds", type=float, default=60.0)
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
        args.progress_timeout_seconds,
        Path(args.frames_jsonl) if args.frames_jsonl else None,
        Path(args.camera_info_json) if args.camera_info_json else None,
    )
    error: BaseException | None = None
    try:
        recorder.spin_until_done()
    except BaseException as exc:
        error = exc
    cleanup_error = _cleanup_recorder(recorder, rclpy)
    if error is not None:
        if cleanup_error is not None and hasattr(error, "add_note"):
            error.add_note(f"Recorder cleanup also failed: {cleanup_error}")
        raise error
    if cleanup_error is not None:
        raise cleanup_error


if __name__ == "__main__":
    main()
