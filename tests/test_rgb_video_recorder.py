import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

from evaluation.rgb_video_recorder import (
    FfmpegVideoWriter,
    RawFrame,
    RgbVideoRecorder,
    _decode_image,
    _decode_serialized_image,
    _load_ros_image_type,
)


class _Image:
    def __init__(self, encoding, width, height, step, data, stamp_s=0.0):
        self.encoding = encoding
        self.width = width
        self.height = height
        self.step = step
        self.data = data
        sec = int(stamp_s)
        nanosec = int(round((stamp_s - sec) * 1_000_000_000))
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
            frame_id="camera_optical_frame",
        )


class RgbVideoRecorderTests(unittest.TestCase):
    @staticmethod
    def _find_ffmpeg():
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is not None:
            return ffmpeg
        candidate = Path(r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe")
        return str(candidate) if candidate.exists() else None

    def test_rgb_decode_honors_row_stride_without_changing_channel_order(self):
        image = _Image(
            "rgb8",
            2,
            2,
            8,
            bytes([
                10, 20, 30, 40, 50, 60, 99, 98,
                70, 80, 90, 100, 110, 120, 97, 96,
            ]),
        )
        decoded = _decode_image(image)
        self.assertIsNotNone(decoded)
        assert decoded is not None
        self.assertEqual((decoded.width, decoded.height), (2, 2))
        self.assertEqual(decoded.pixel_format, "rgb24")
        self.assertEqual(
            decoded.data,
            bytes([10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]),
        )

    def test_serialized_rgb_decode_retains_cdr_buffer(self):
        from tests.rosbag_transport_peer import _image

        decoded = _decode_serialized_image(_image(2.0 + 1.0 / 30.0))
        self.assertIsNotNone(decoded)
        assert decoded is not None
        stamp_s, frame_id, encoding, frame = decoded
        self.assertAlmostEqual(stamp_s, 2.0 + 1.0 / 30.0)
        self.assertEqual((frame_id, encoding), ("camera_optical_frame", "rgb8"))
        self.assertEqual((frame.width, frame.height, frame.pixel_format), (4, 2, "rgb24"))
        self.assertIsInstance(frame.data, memoryview)
        self.assertEqual(bytes(frame.data), bytes((255, 0, 0)) * 8)

    def test_real_ros_recorder_constructs_without_numpy_or_camera_info_subscription(self):
        try:
            import rclpy
        except ImportError:
            self.skipTest("rclpy is not installed")

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            recorder = None
            numpy_was_loaded = "numpy" in sys.modules
            rclpy.init()
            try:
                recorder = RgbVideoRecorder(
                    root / "out.mp4",
                    root / "metadata.json",
                    duration_s=2.0 / 30.0,
                    startup_timeout_s=1.0,
                )
                topics = {subscription.topic_name for subscription in recorder.node.subscriptions}
                self.assertIn("/sim/camera/rgb/image_raw", topics)
                self.assertNotIn("/sim/camera/rgb/camera_info", topics)
                self.assertEqual(topics, {"/sim/camera/rgb/image_raw"})
                self.assertEqual("numpy" in sys.modules, numpy_was_loaded)
                Image = _load_ros_image_type()
                self.assertEqual(Image.__module__, "sensor_msgs.msg._image")
                self.assertIs(sys.modules["sensor_msgs.msg._image"].Image, Image)
                recorder._on_image(_Image("rgb8", 4, 2, 12, bytes([255, 0, 0]) * 8, 0.0))
                recorder._on_image(_Image("rgb8", 4, 2, 12, bytes([0, 255, 0]) * 8, 1.0 / 30.0))
                recorder._on_image(_Image("rgb8", 4, 2, 12, bytes([0, 0, 255]) * 8, 2.0 / 30.0))
                self.assertEqual(recorder.done_reason, "simulation_time_reached")
                metadata = recorder.close()
                self.assertEqual(metadata["status"], "complete")
                self.assertEqual(metadata["completion_clock_source"], "image_header")
                self.assertEqual(metadata["frame_count"], 3)
                self.assertTrue(metadata["rgb_cadence"]["contiguous"])
                self.assertEqual(metadata["encoder_returncode"], 0)
                self.assertEqual(metadata["numpy_loaded"], numpy_was_loaded)
            finally:
                if recorder is not None:
                    for subscription in list(recorder.node.subscriptions):
                        recorder.node.destroy_subscription(subscription)
                    self.assertEqual(list(recorder.node.subscriptions), [])
                    recorder.node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()

    def test_programmatic_camera_info_request_fails_before_ros_imports(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            with self.assertRaisesRegex(RuntimeError, "configured-intrinsics artifact"):
                RgbVideoRecorder(
                    root / "out.mp4",
                    root / "metadata.json",
                    duration_s=1.0,
                    startup_timeout_s=1.0,
                    camera_info_path=root / "camera_info.json",
                )

    def test_cross_process_zenoh_image_delivery_encodes_without_crash_or_numpy(self):
        if not os.environ.get("AMENT_PREFIX_PATH"):
            self.skipTest("requires the Pixi ROS environment")
        router_executable = (
            Path(sys.prefix) / "Library" / "lib" / "rmw_zenoh_cpp" / "rmw_zenohd.exe"
        )
        if not router_executable.exists():
            self.skipTest("rmw_zenohd is not installed")

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            output = root / "transport.mp4"
            metadata_path = root / "transport.json"
            environment = os.environ.copy()
            environment["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
            environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
            router_log_path = root / "router.log"
            router_log = router_log_path.open("w+b")
            router = subprocess.Popen(
                [str(router_executable)],
                env=environment,
                stdout=router_log,
                stderr=subprocess.STDOUT,
            )
            recorder = None
            publisher = None
            try:
                time.sleep(2.0)
                self.assertIsNone(router.poll(), router_log_path.read_text(encoding="utf-8"))
                recorder = subprocess.Popen(
                    [
                        sys.executable,
                        "-m",
                        "evaluation.rgb_video_recorder",
                        "--output",
                        str(output),
                        "--metadata",
                        str(metadata_path),
                        "--duration-seconds",
                        str(2.0 / 30.0),
                        "--startup-timeout-seconds",
                        "20",
                    ],
                    cwd=Path(__file__).resolve().parents[1],
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                time.sleep(2.0)
                self.assertIsNone(recorder.poll(), "recorder exited before publisher startup")
                publisher = subprocess.Popen(
                    [sys.executable, "-m", "tests.ros_image_transport_peer"],
                    cwd=Path(__file__).resolve().parents[1],
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                publisher_output, _ = publisher.communicate(timeout=20)
                self.assertEqual(publisher.returncode, 0, publisher_output)
                self.assertIn("published=3 numpy_loaded=False", publisher_output)
                recorder_output, _ = recorder.communicate(timeout=20)
                self.assertEqual(recorder.returncode, 0, recorder_output)
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                self.assertEqual(metadata["status"], "complete")
                self.assertEqual(metadata["frame_count"], 3)
                self.assertEqual((metadata["width"], metadata["height"]), (1280, 720))
                self.assertFalse(metadata["numpy_loaded"])
                self.assertTrue(metadata["rgb_cadence"]["contiguous"])
                self.assertGreater(output.stat().st_size, 0)
            finally:
                for process in (publisher, recorder, router):
                    if process is not None and process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=5)
                router_log.close()

    def test_supported_encodings_map_to_ffmpeg_raw_pixel_formats(self):
        cases = {
            "mono8": (1, "gray"),
            "8UC1": (1, "gray"),
            "rgb8": (3, "rgb24"),
            "bgr8": (3, "bgr24"),
            "8UC3": (3, "bgr24"),
            "rgba8": (4, "rgba"),
            "bgra8": (4, "bgra"),
            "8UC4": (4, "bgra"),
        }
        for encoding, (channels, pixel_format) in cases.items():
            with self.subTest(encoding=encoding):
                decoded = _decode_image(_Image(encoding, 2, 2, 2 * channels, bytes(range(4 * channels))))
                self.assertIsNotNone(decoded)
                assert decoded is not None
                self.assertEqual(decoded.pixel_format, pixel_format)
                self.assertEqual(len(decoded.data), 4 * channels)

    def test_invalid_layouts_are_rejected(self):
        invalid = [
            _Image("16UC1", 1, 1, 2, b"\x00\x00"),
            _Image("rgb8", 2, 1, 5, b"\x00" * 5),
            _Image("rgb8", 2, 2, 6, b"\x00" * 11),
            _Image("rgb8", 0, 1, 0, b""),
        ]
        for image in invalid:
            with self.subTest(encoding=image.encoding, width=image.width, step=image.step):
                self.assertIsNone(_decode_image(image))

    def test_writer_removes_stale_output_when_ffmpeg_is_unavailable(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "out.mp4"
            output.write_bytes(b"stale video")
            writer = FfmpegVideoWriter(output, "definitely-missing-ffmpeg")
            frame = RawFrame(4, 2, "rgb24", b"\x00" * 24)
            self.assertFalse(writer.open(frame))
            self.assertFalse(output.exists())
            self.assertIn("not found", writer.error)

    def test_installed_ffmpeg_encodes_raw_frames(self):
        ffmpeg = self._find_ffmpeg()
        if ffmpeg is None:
            self.skipTest("FFmpeg is not installed")

        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "tiny.mp4"
            writer = FfmpegVideoWriter(output, ffmpeg)
            frame = RawFrame(4, 2, "rgb24", bytes([255, 0, 0]) * 8)
            self.assertTrue(writer.open(frame), writer.error)
            self.assertTrue(writer.write(frame), writer.error)
            self.assertTrue(writer.write(frame), writer.error)
            result = writer.close()
            self.assertEqual(result.returncode, 0, result.error)
            self.assertIsNone(result.error)
            self.assertGreater(output.stat().st_size, 0)

            ffprobe = Path(ffmpeg).with_name("ffprobe.exe" if Path(ffmpeg).suffix.lower() == ".exe" else "ffprobe")
            if ffprobe.exists():
                probe = subprocess.run(
                    [
                        str(ffprobe), "-v", "error", "-select_streams", "v:0",
                        "-show_entries", "stream=codec_name,width,height,nb_frames",
                        "-of", "json", str(output),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                stream = json.loads(probe.stdout)["streams"][0]
                self.assertEqual(stream["codec_name"], "h264")
                self.assertEqual((stream["width"], stream["height"]), (4, 2))
                self.assertEqual(int(stream["nb_frames"]), 2)

    def test_writer_reports_layout_change_after_start(self):
        ffmpeg = self._find_ffmpeg()
        if ffmpeg is None:
            self.skipTest("FFmpeg is not installed")
        with tempfile.TemporaryDirectory() as temporary_directory:
            writer = FfmpegVideoWriter(Path(temporary_directory) / "out.mp4", ffmpeg)
            initial = RawFrame(4, 2, "rgb24", b"\x00" * 24)
            changed = RawFrame(2, 4, "rgb24", b"\x00" * 24)
            self.assertTrue(writer.open(initial), writer.error)
            self.assertTrue(writer.write(initial), writer.error)
            self.assertFalse(writer.write(changed))
            self.assertIn("Image layout changed", writer.error)
            result = writer.close()
            self.assertEqual(result.returncode, 0)
            self.assertIn("Image layout changed", result.error)

    def test_open_writer_rejects_odd_dimensions(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            writer = FfmpegVideoWriter(Path(temporary_directory) / "out.mp4", "ffmpeg")
            frame = RawFrame(3, 2, "rgb24", b"\x00" * 18)
            self.assertFalse(writer.open(frame))
            self.assertIn("even frame dimensions", writer.error)


if __name__ == "__main__":
    unittest.main()
