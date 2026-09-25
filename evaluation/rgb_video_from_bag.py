"""Build the delivery RGB video from the already closed raw ROS bag.

The raw bag is the sole live subscriber for large 1080p image messages.  This
offline pass derives the MP4 and timestamp index from those exact messages, so
video encoding cannot cause an independent live-frame drop.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from evaluation.rgb_video_recorder import (
    FRAME_GAP_TOLERANCE_S,
    FRAME_PERIOD_S,
    NOMINAL_FPS,
    RGB_TOPIC,
    WRITER_CLOSE_TIMEOUT_S,
    _bounded_resource_close,
    _decode_image,
    _stamp,
)
from simulator.capture.stamp_digest import stamp_sequence_sha256


CAMERA_INFO_TOPIC = "/sim/camera/rgb/camera_info"


def _probe_decoded_video(path: Path) -> dict[str, object]:
    """Decode the closed delivery video so a lossy/silent writer cannot pass."""

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        capture.release()
        return {
            "opened": False,
            "frame_count": 0,
            "widths": [],
            "heights": [],
            "reported_fps": None,
        }
    frame_count = 0
    widths: set[int] = set()
    heights: set[int] = set()
    reported_fps = float(capture.get(cv2.CAP_PROP_FPS))
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame is None or len(frame.shape) < 2:
                continue
            frame_count += 1
            heights.add(int(frame.shape[0]))
            widths.add(int(frame.shape[1]))
    finally:
        capture.release()
    return {
        "opened": True,
        "frame_count": frame_count,
        "widths": sorted(widths),
        "heights": sorted(heights),
        "reported_fps": reported_fps,
    }


class BagRgbVideoBuilder:
    def __init__(
        self,
        output: Path,
        metadata_path: Path,
        frames_path: Path,
        camera_info_path: Path,
        duration_s: float,
        expected_width: int,
        expected_height: int,
        expected_fps: float,
        source_bag_uri: str,
    ):
        self.output = output
        self.metadata_path = metadata_path
        self.frames_path = frames_path
        self.camera_info_path = camera_info_path
        self.duration_s = float(duration_s)
        self.expected_width = int(expected_width)
        self.expected_height = int(expected_height)
        self.expected_fps = float(expected_fps)
        self.source_bag_uri = source_bag_uri
        self.writer: cv2.VideoWriter | None = None
        self.codec = "avc1"
        self.frame_records: list[dict[str, object]] = []
        self.camera_info_record: dict[str, object] | None = None
        self.camera_info_count = 0
        self.first_stamp_s: float | None = None
        self.last_stamp_s: float | None = None
        self.max_frame_gap_s: float | None = None
        self.invalid_frames = 0
        self.nonincreasing_frames = 0
        self.frame_ids: set[str] = set()
        self.finalized = False

    def _open_writer(self) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        self.writer = cv2.VideoWriter(
            str(self.output),
            fourcc,
            self.expected_fps,
            (self.expected_width, self.expected_height),
        )
        if not self.writer.isOpened():
            self.codec = "mp4v"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(
                str(self.output),
                fourcc,
                self.expected_fps,
                (self.expected_width, self.expected_height),
            )
        if not self.writer.isOpened():
            raise RuntimeError("offline RGB VideoWriter is unavailable")

    def consume_image(self, message) -> None:
        stamp_s = _stamp(message)
        if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s + 1e-9:
            self.nonincreasing_frames += 1
            return
        if int(message.width) != self.expected_width or int(message.height) != self.expected_height:
            self.invalid_frames += 1
            return
        frame = _decode_image(message)
        if frame is None:
            self.invalid_frames += 1
            return
        if self.writer is None:
            self._open_writer()
        assert self.writer is not None
        self.writer.write(np.ascontiguousarray(frame))
        if self.last_stamp_s is not None:
            gap_s = stamp_s - self.last_stamp_s
            self.max_frame_gap_s = gap_s if self.max_frame_gap_s is None else max(self.max_frame_gap_s, gap_s)
        if self.first_stamp_s is None:
            self.first_stamp_s = stamp_s
        self.last_stamp_s = stamp_s
        self.frame_ids.add(str(message.header.frame_id))
        self.frame_records.append(
            {
                "frame_index": len(self.frame_records),
                "stamp_s": stamp_s,
                "frame_id": str(message.header.frame_id),
                "width": int(message.width),
                "height": int(message.height),
                "encoding": str(message.encoding),
            }
        )

    def consume_camera_info(self, message) -> None:
        self.camera_info_count += 1
        if self.camera_info_record is not None:
            return
        self.camera_info_record = {
            "topic": CAMERA_INFO_TOPIC,
            "stamp_s": _stamp(message),
            "frame_id": str(message.header.frame_id),
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": str(message.distortion_model),
            "d": [float(value) for value in message.d],
            "k": [float(value) for value in message.k],
            "r": [float(value) for value in message.r],
            "p": [float(value) for value in message.p],
        }

    def finalize(self, expected_image_count: int, expected_camera_info_count: int) -> dict[str, object]:
        if self.finalized:
            raise RuntimeError("offline RGB builder was already finalized")
        self.finalized = True
        release_error = None
        if self.writer is not None:
            release_error = _bounded_resource_close(self.writer, "release", WRITER_CLOSE_TIMEOUT_S)
            self.writer = None
        file_size = self.output.stat().st_size if self.output.exists() else 0
        frame_count = len(self.frame_records)
        decoded_video = _probe_decoded_video(self.output) if file_size > 0 else {
            "opened": False,
            "frame_count": 0,
            "widths": [],
            "heights": [],
            "reported_fps": None,
        }
        decoded_video_valid = (
            bool(decoded_video["opened"])
            and int(decoded_video["frame_count"]) == frame_count
            and decoded_video["widths"] == [self.expected_width]
            and decoded_video["heights"] == [self.expected_height]
            and decoded_video["reported_fps"] is not None
            and abs(float(decoded_video["reported_fps"]) - self.expected_fps) <= 0.01
        )
        actual_fps = None
        if self.first_stamp_s is not None and self.last_stamp_s is not None and self.last_stamp_s > self.first_stamp_s:
            actual_fps = (frame_count - 1) / (self.last_stamp_s - self.first_stamp_s)
        cadence_contiguous = (
            frame_count >= 2
            and self.nonincreasing_frames == 0
            and self.max_frame_gap_s is not None
            and self.max_frame_gap_s <= FRAME_PERIOD_S + FRAME_GAP_TOLERANCE_S
        )
        horizon_covered = (
            self.first_stamp_s is not None
            and self.first_stamp_s <= 0.1 + FRAME_GAP_TOLERANCE_S
            and self.last_stamp_s is not None
            and self.last_stamp_s >= self.duration_s - 1e-3
        )
        camera_info_valid = (
            self.camera_info_record is not None
            and self.camera_info_record["frame_id"] == "camera_optical_frame"
            and self.camera_info_record["width"] == self.expected_width
            and self.camera_info_record["height"] == self.expected_height
        )
        complete = (
            frame_count == int(expected_image_count)
            and self.camera_info_count == int(expected_camera_info_count)
            and self.expected_fps == NOMINAL_FPS
            and cadence_contiguous
            and horizon_covered
            and camera_info_valid
            and self.frame_ids == {"camera_optical_frame"}
            and self.invalid_frames == 0
            and release_error is None
            and file_size > 0
            and decoded_video_valid
        )
        metadata = {
            "status": "complete" if complete else "failed",
            "topic": RGB_TOPIC,
            "source": "closed_rosbag2",
            "source_bag_uri": self.source_bag_uri,
            "codec": self.codec if frame_count else None,
            "width": self.expected_width,
            "height": self.expected_height,
            "nominal_fps": self.expected_fps,
            "actual_fps_from_ros_timestamps": actual_fps,
            "frame_count": frame_count,
            "source_bag_rgb_count": int(expected_image_count),
            "stamp_sha256": stamp_sequence_sha256(record["stamp_s"] for record in self.frame_records),
            "camera_info_count": self.camera_info_count,
            "source_bag_camera_info_count": int(expected_camera_info_count),
            "duration_s": None if self.first_stamp_s is None or self.last_stamp_s is None else self.last_stamp_s - self.first_stamp_s,
            "first_image_stamp_s": self.first_stamp_s,
            "last_image_stamp_s": self.last_stamp_s,
            "target_sim_time_s": self.duration_s,
            "encoding": None if not self.frame_records else self.frame_records[0]["encoding"],
            "invalid_frames": self.invalid_frames,
            "nonincreasing_frames": self.nonincreasing_frames,
            "max_frame_gap_s": self.max_frame_gap_s,
            "cadence_contiguous": cadence_contiguous,
            "horizon_covered": horizon_covered,
            "clock_regressions": 0,
            "frame_ids": sorted(self.frame_ids),
            "camera_info_frame_id": None if self.camera_info_record is None else self.camera_info_record["frame_id"],
            "file_size_bytes": file_size,
            "decoded_video": decoded_video,
            "decoded_video_valid": decoded_video_valid,
            "completion_reason": "closed_bag_exhausted",
        }
        if release_error is not None:
            metadata["writer_error"] = f"{type(release_error).__name__}: {release_error}"
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        self.frames_path.parent.mkdir(parents=True, exist_ok=True)
        with self.frames_path.open("w", encoding="utf-8", newline="\n") as handle:
            for record in self.frame_records:
                handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
        if self.camera_info_record is not None:
            self.camera_info_path.parent.mkdir(parents=True, exist_ok=True)
            self.camera_info_path.write_text(
                json.dumps(self.camera_info_record, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        if not complete:
            raise RuntimeError(f"offline RGB video validation failed: {metadata}")
        return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--bag-metadata", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--frames-jsonl", required=True)
    parser.add_argument("--camera-info-json", required=True)
    parser.add_argument("--duration-seconds", type=float, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--fps", type=float, required=True)
    args = parser.parse_args()

    bag_metadata = json.loads(Path(args.bag_metadata).read_text(encoding="utf-8"))
    if bag_metadata.get("status") != "complete":
        raise RuntimeError("offline RGB conversion requires a complete validated raw bag receipt")
    expected_image_count = int(bag_metadata["counts"][RGB_TOPIC])
    expected_camera_info_count = int(bag_metadata["counts"][CAMERA_INFO_TOPIC])

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(args.bag)), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (RGB_TOPIC, CAMERA_INFO_TOPIC)
    }
    builder = BagRgbVideoBuilder(
        Path(args.output),
        Path(args.metadata),
        Path(args.frames_jsonl),
        Path(args.camera_info_json),
        args.duration_seconds,
        args.width,
        args.height,
        args.fps,
        str(Path(args.bag).resolve()),
    )
    while reader.has_next():
        topic, serialized, _storage_timestamp_ns = reader.read_next()
        if topic == RGB_TOPIC:
            builder.consume_image(deserialize_message(serialized, message_types[topic]))
        elif topic == CAMERA_INFO_TOPIC:
            builder.consume_camera_info(deserialize_message(serialized, message_types[topic]))
    builder.finalize(expected_image_count, expected_camera_info_count)


if __name__ == "__main__":
    main()
