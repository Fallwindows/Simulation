"""Create and play a tiny serialized sensor bag for cross-process tests."""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

from simulator.capture.rosbag_capture import TOPIC_TYPES


class _Cdr:
    def __init__(self):
        self.data = bytearray(b"\x00\x01\x00\x00")

    def align(self, size: int) -> None:
        self.data.extend(b"\x00" * (-len(self.data) % size))

    def i32(self, value: int) -> None:
        self.align(4)
        self.data.extend(struct.pack("<i", value))

    def u32(self, value: int) -> None:
        self.align(4)
        self.data.extend(struct.pack("<I", value))

    def u8(self, value: int) -> None:
        self.data.extend(struct.pack("<B", value))

    def string(self, value: str) -> None:
        encoded = value.encode("utf-8") + b"\x00"
        self.u32(len(encoded))
        self.data.extend(encoded)

    def sequence(self, value: bytes) -> None:
        self.u32(len(value))
        self.data.extend(value)


def _header(payload: _Cdr, stamp_s: float, frame_id: str) -> None:
    seconds = int(stamp_s)
    payload.i32(seconds)
    payload.u32(int(round((stamp_s - seconds) * 1_000_000_000)))
    payload.string(frame_id)


def _clock(stamp_s: float) -> bytes:
    payload = _Cdr()
    seconds = int(stamp_s)
    payload.i32(seconds)
    payload.u32(int(round((stamp_s - seconds) * 1_000_000_000)))
    return bytes(payload.data)


def _image(stamp_s: float) -> bytes:
    payload = _Cdr()
    _header(payload, stamp_s, "camera_optical_frame")
    payload.u32(2)
    payload.u32(4)
    payload.string("rgb8")
    payload.u8(0)
    payload.u32(12)
    payload.sequence(bytes((255, 0, 0)) * 8)
    return bytes(payload.data)


def _point_cloud(stamp_s: float) -> bytes:
    payload = _Cdr()
    _header(payload, stamp_s, "lidar_link")
    payload.u32(1)
    payload.u32(1)
    payload.u32(0)  # fields
    payload.u8(0)
    payload.u32(12)
    payload.u32(12)
    payload.sequence(b"\x00" * 12)
    payload.u8(1)
    return bytes(payload.data)


def _empty_tf() -> bytes:
    payload = _Cdr()
    payload.u32(0)
    return bytes(payload.data)


def play(source: Path) -> None:
    import rosbag2_py

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    for topic_id, (topic, type_name) in enumerate(TOPIC_TYPES.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(topic_id, topic, type_name, "cdr", []))
    base_ns = 2_000_000_000
    for index, stamp_s in enumerate((2.0, 2.05, 2.1)):
        record_ns = base_ns + index * 50_000_000
        writer.write("/clock", _clock(stamp_s), record_ns)
        writer.write("/sim/camera/rgb/image_raw", _image(stamp_s), record_ns + 1)
        writer.write("/sim/lidar/points", _point_cloud(stamp_s), record_ns + 2)
        writer.write("/tf", _empty_tf(), record_ns + 3)
        if index == 0:
            writer.write("/tf_static", _empty_tf(), record_ns + 4)
    writer = None

    play_options = rosbag2_py.PlayOptions()
    play_options.delay = 5.0
    play_options.disable_keyboard_controls = True
    player = rosbag2_py.Player(
        rosbag2_py.StorageOptions(uri=str(source), storage_id="sqlite3"),
        play_options,
        "info",
        "grocery_sim_raw_capture_test_player",
    )
    player.play()
    if not player.wait_for_playback_to_finish(20.0):
        player.stop()
        raise RuntimeError("serialized sensor bag playback timed out")
    player.stop()
    print(f"published_topics={len(TOPIC_TYPES)} numpy_loaded={'numpy' in sys.modules}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    args = parser.parse_args()
    play(Path(args.source))


if __name__ == "__main__":
    main()
