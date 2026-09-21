"""Record the native ROS RGB camera stream to a compact MP4.

The recorder intentionally uses only the Python standard library until ROS is
imported. Frames are validated and streamed to FFmpeg as raw bytes so this
process does not depend on NumPy or OpenCV and their native BLAS dependency.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from typing import BinaryIO


RGB_TOPIC = "/sim/camera/rgb/image_raw"
SIM_CLOCK_START_TOLERANCE_S = 1.0
POST_TARGET_GRACE_S = 1.0
NOMINAL_FPS = 30.0
FFMPEG_CLOSE_TIMEOUT_S = 30.0
ENCODER_PRESET = "fast"
ENCODER_CRF = 18


@dataclass(frozen=True, slots=True)
class RawFrame:
    width: int
    height: int
    pixel_format: str
    data: bytes


@dataclass(frozen=True, slots=True)
class EncoderResult:
    returncode: int | None
    error: str | None


class _DisabledTypeDescriptionService:
    """No-op for an optional rclpy service whose generated type needs NumPy."""

    def __init__(self, _node):
        pass

    def destroy(self) -> None:
        pass


def _load_ros_image_type():
    """Load only the generated Image type, bypassing sensor_msgs.msg imports."""

    import sensor_msgs

    module_path = Path(sensor_msgs.__file__).resolve().parent / "msg" / "_image.py"
    spec = importlib.util.spec_from_file_location("_grocery_sim_sensor_msgs_image", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load generated ROS Image type from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Image


def _create_recorder_node():
    """Create the narrow recorder node without NumPy-dependent ROS services."""

    import rclpy.node as node_module

    original_service = node_module.TypeDescriptionService
    node_module.TypeDescriptionService = _DisabledTypeDescriptionService
    try:
        return node_module.Node(
            "grocery_sim_rgb_video_recorder",
            start_parameter_services=False,
        )
    finally:
        node_module.TypeDescriptionService = original_service


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0


def _decode_image(message) -> RawFrame | None:
    """Validate and pack a supported 8-bit ROS Image without color conversion."""

    try:
        width = int(message.width)
        height = int(message.height)
        step = int(message.step)
    except (TypeError, ValueError, OverflowError):
        return None
    if width <= 0 or height <= 0 or step <= 0:
        return None

    encoding = str(message.encoding).strip().lower()
    layouts = {
        "mono8": (1, "gray"),
        "8uc1": (1, "gray"),
        "rgb8": (3, "rgb24"),
        "bgr8": (3, "bgr24"),
        # ROS generic encodings do not specify channel meaning. Match the
        # previous OpenCV convention for common 3/4-channel byte layouts.
        "8uc3": (3, "bgr24"),
        "rgba8": (4, "rgba"),
        "bgra8": (4, "bgra"),
        "8uc4": (4, "bgra"),
    }
    layout = layouts.get(encoding)
    if layout is None:
        return None
    channels, pixel_format = layout
    packed_row_bytes = width * channels
    if step < packed_row_bytes:
        return None

    try:
        source = memoryview(message.data).cast("B")
    except (TypeError, ValueError):
        try:
            source = memoryview(bytes(message.data))
        except (TypeError, ValueError):
            return None
    required_bytes = step * height
    if source.nbytes < required_bytes:
        return None

    if step == packed_row_bytes:
        packed = bytes(source[:required_bytes])
    else:
        packed_buffer = bytearray(packed_row_bytes * height)
        for row in range(height):
            source_start = row * step
            target_start = row * packed_row_bytes
            packed_buffer[target_start : target_start + packed_row_bytes] = source[
                source_start : source_start + packed_row_bytes
            ]
        packed = bytes(packed_buffer)
    return RawFrame(width=width, height=height, pixel_format=pixel_format, data=packed)


class FfmpegVideoWriter:
    """Write fixed-layout raw frames to an H.264 MP4 using FFmpeg."""

    def __init__(self, output: Path, executable: str, fps: float = NOMINAL_FPS):
        self.output = output
        self.executable_requested = executable
        self.fps = float(fps)
        self.executable_resolved: str | None = None
        self.width: int | None = None
        self.height: int | None = None
        self.pixel_format: str | None = None
        self.process: subprocess.Popen[bytes] | None = None
        self._stderr: BinaryIO | None = None
        self.error: str | None = None
        self.returncode: int | None = None

    @property
    def started(self) -> bool:
        return self.process is not None

    def open(self, frame: RawFrame) -> bool:
        if self.process is not None:
            raise RuntimeError("FFmpeg writer is already open")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if self.output.exists():
            self.output.unlink()
        if frame.width % 2 or frame.height % 2:
            self.error = "H.264 yuv420p output requires even frame dimensions"
            return False

        executable = shutil.which(self.executable_requested)
        if executable is None:
            self.error = f"FFmpeg executable was not found: {self.executable_requested}"
            return False
        self.executable_resolved = executable
        self.width = frame.width
        self.height = frame.height
        self.pixel_format = frame.pixel_format
        command = [
            executable,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            frame.pixel_format,
            "-video_size",
            f"{frame.width}x{frame.height}",
            "-framerate",
            f"{self.fps:g}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            ENCODER_PRESET,
            "-crf",
            str(ENCODER_CRF),
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(self.output),
        ]
        self._stderr = tempfile.TemporaryFile(mode="w+b")
        try:
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
            )
        except OSError as exc:
            self.error = f"Could not start FFmpeg: {exc}"
            self._stderr.close()
            self._stderr = None
            return False
        return True

    def accepts(self, frame: RawFrame) -> bool:
        return (
            frame.width == self.width
            and frame.height == self.height
            and frame.pixel_format == self.pixel_format
        )

    def write(self, frame: RawFrame) -> bool:
        if self.process is None or self.process.stdin is None:
            self.error = "FFmpeg writer is not open"
            return False
        if not self.accepts(frame):
            self.error = (
                "Image layout changed after recording started: "
                f"expected {self.width}x{self.height} {self.pixel_format}, "
                f"received {frame.width}x{frame.height} {frame.pixel_format}"
            )
            return False
        if self.process.poll() is not None:
            self.returncode = self.process.returncode
            self.error = f"FFmpeg exited before accepting all frames (exit {self.returncode})"
            return False
        try:
            self.process.stdin.write(frame.data)
        except (BrokenPipeError, OSError) as exc:
            self.returncode = self.process.poll()
            self.error = f"FFmpeg frame write failed: {exc}"
            return False
        return True

    def close(self) -> EncoderResult:
        if self.process is None:
            return EncoderResult(returncode=self.returncode, error=self.error)
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except OSError as exc:
                if self.error is None:
                    self.error = f"Could not close FFmpeg input: {exc}"
            self.process.stdin = None
        try:
            self.returncode = self.process.wait(timeout=FFMPEG_CLOSE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.returncode = self.process.wait()
            self.error = f"FFmpeg did not exit within {FFMPEG_CLOSE_TIMEOUT_S:g} seconds"
        stderr_text = ""
        if self._stderr is not None:
            self._stderr.seek(0)
            stderr_text = self._stderr.read().decode("utf-8", errors="replace").strip()
            self._stderr.close()
            self._stderr = None
        if self.returncode != 0 and self.error is None:
            detail = stderr_text[-2000:] if stderr_text else "no stderr output"
            self.error = f"FFmpeg exited with code {self.returncode}: {detail}"
        return EncoderResult(returncode=self.returncode, error=self.error)


class RgbVideoRecorder:
    def __init__(
        self,
        output: Path,
        metadata_path: Path,
        duration_s: float,
        startup_timeout_s: float,
        frames_path: Path | None = None,
        camera_info_path: Path | None = None,
        ffmpeg_executable: str = "ffmpeg",
    ):
        if camera_info_path is not None:
            raise RuntimeError(
                "--camera-info-json cannot subscribe under the active policy; "
                "use the configured-intrinsics artifact produced by capture metadata export"
        )
        import rclpy

        self.rclpy = rclpy
        self.output = output
        self.metadata_path = metadata_path
        self.frames_path = frames_path
        self.duration_s = float(duration_s)
        self.startup_timeout_s = float(startup_timeout_s)
        self.ffmpeg_executable = ffmpeg_executable
        self.started_wall = time.monotonic()
        self.target_s: float | None = None
        self.post_target_deadline: float | None = None
        self.first_image_stamp_s: float | None = None
        self.last_image_stamp_s: float | None = None
        self.frames_written = 0
        self.invalid_frames = 0
        self.duplicate_frames = 0
        self.out_of_order_frames = 0
        self.frames_after_target = 0
        self.writer: FfmpegVideoWriter | None = None
        self.width: int | None = None
        self.height: int | None = None
        self.encoding: str | None = None
        self.done_reason = "not_started"
        self.encoder_error: str | None = None
        self.encoder_returncode: int | None = None
        self.fatal_error = False
        self.frame_records: list[dict[str, object]] = []
        Image = _load_ros_image_type()
        self.node = _create_recorder_node()
        try:
            self.node.create_subscription(Image, RGB_TOPIC, self._on_image, 5)
        except BaseException:
            self.node.destroy_node()
            raise

    def _open_writer(self, frame: RawFrame) -> bool:
        self.width = frame.width
        self.height = frame.height
        self.writer = FfmpegVideoWriter(self.output, self.ffmpeg_executable)
        if self.writer.open(frame):
            return True
        self.encoder_error = self.writer.error
        return False

    def _on_image(self, message) -> None:
        stamp = _stamp(message)
        if self.last_image_stamp_s is not None:
            if stamp == self.last_image_stamp_s:
                self.duplicate_frames += 1
                return
            if stamp < self.last_image_stamp_s:
                self.out_of_order_frames += 1
                return
        frame = _decode_image(message)
        if frame is None:
            self.invalid_frames += 1
            return
        if self.target_s is None:
            if stamp <= SIM_CLOCK_START_TOLERANCE_S:
                self.target_s = self.duration_s
            else:
                self.target_s = stamp + self.duration_s
        if stamp > self.target_s + 0.05:
            self.frames_after_target += 1
            if self.frames_written > 0 and self.post_target_deadline is None:
                self.post_target_deadline = time.monotonic() + POST_TARGET_GRACE_S
                self.done_reason = "simulation_time_reached"
            return
        if self.writer is None and not self._open_writer(frame):
            self.done_reason = "video_writer_unavailable"
            self.fatal_error = True
            return
        assert self.writer is not None
        if not self.writer.write(frame):
            self.encoder_error = self.writer.error
            self.done_reason = "video_writer_error"
            self.fatal_error = True
            return
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
        if stamp >= self.target_s - 1e-3 and self.post_target_deadline is None:
            self.post_target_deadline = time.monotonic() + POST_TARGET_GRACE_S
            self.done_reason = "simulation_time_reached"

    def spin_until_done(self) -> None:
        while self.rclpy.ok():
            if self.fatal_error:
                break
            now = time.monotonic()
            if self.post_target_deadline is not None and now >= self.post_target_deadline:
                break
            if self.post_target_deadline is None and now - self.started_wall >= self.startup_timeout_s:
                self.done_reason = "startup_timeout"
                break
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        if self.writer is not None:
            encoder_result = self.writer.close()
            self.encoder_returncode = encoder_result.returncode
            self.encoder_error = encoder_result.error
            if encoder_result.error is not None and self.done_reason == "simulation_time_reached":
                self.done_reason = "video_encoder_failed"
        file_size = self.output.stat().st_size if self.output.exists() else 0
        duration_s = None
        actual_fps = None
        if self.first_image_stamp_s is not None and self.last_image_stamp_s is not None:
            duration_s = self.last_image_stamp_s - self.first_image_stamp_s
            if duration_s > 0 and self.frames_written > 1:
                actual_fps = (self.frames_written - 1) / duration_s
        complete = (
            self.frames_written > 0
            and self.done_reason == "simulation_time_reached"
            and self.invalid_frames == 0
            and self.duplicate_frames == 0
            and self.out_of_order_frames == 0
            and self.encoder_error is None
            and self.encoder_returncode == 0
            and file_size > 0
        )
        metadata = {
            "status": "complete" if complete else "failed",
            "topic": RGB_TOPIC,
            "writer_backend": "ffmpeg",
            "codec": "h264" if self.writer is not None and self.writer.started else None,
            "encoder": "libx264" if self.writer is not None and self.writer.started else None,
            "encoder_preset": ENCODER_PRESET if self.writer is not None and self.writer.started else None,
            "encoder_crf": ENCODER_CRF if self.writer is not None and self.writer.started else None,
            "ffmpeg_executable": self.writer.executable_resolved if self.writer is not None else None,
            "encoder_returncode": self.encoder_returncode,
            "encoder_error": self.encoder_error,
            "input_pixel_format": self.writer.pixel_format if self.writer is not None else None,
            "width": self.width,
            "height": self.height,
            "nominal_fps": NOMINAL_FPS,
            "actual_fps_from_ros_timestamps": actual_fps,
            "frame_count": self.frames_written,
            "duration_s": duration_s,
            "first_image_stamp_s": self.first_image_stamp_s,
            "last_image_stamp_s": self.last_image_stamp_s,
            "target_sim_time_s": self.target_s,
            "last_clock_s": None,
            "completion_clock_source": "image_header",
            "encoding": self.encoding,
            "invalid_frames": self.invalid_frames,
            "duplicate_frames": self.duplicate_frames,
            "out_of_order_frames": self.out_of_order_frames,
            "frames_after_target": self.frames_after_target,
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
    parser.add_argument("--ffmpeg-executable", default="ffmpeg")
    args = parser.parse_args()
    if args.camera_info_json:
        parser.error(
            "--camera-info-json is unavailable under the active policy; "
            "capture metadata export provides configured camera intrinsics"
        )
    import rclpy

    rclpy.init()
    recorder = None
    try:
        recorder = RgbVideoRecorder(
            Path(args.output),
            Path(args.metadata),
            args.duration_seconds,
            args.startup_timeout_seconds,
            Path(args.frames_jsonl) if args.frames_jsonl else None,
            None,
            args.ffmpeg_executable,
        )
        try:
            recorder.spin_until_done()
        except BaseException as exc:
            recorder.done_reason = "recorder_exception"
            if recorder.encoder_error is None:
                recorder.encoder_error = f"{type(exc).__name__}: {exc}"
            try:
                recorder.close()
            except RuntimeError:
                pass
            raise
        recorder.close()
    finally:
        if recorder is not None:
            recorder.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
