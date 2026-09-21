import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from simulator.capture.rosbag_capture import RawCaptureWriter, TOPIC_TYPES, _cdr_time


class RosbagCaptureTests(unittest.TestCase):
    def test_raw_contract_excludes_observed_camera_info(self):
        self.assertEqual(
            TOPIC_TYPES,
            {
                "/clock": "rosgraph_msgs/msg/Clock",
                "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
                "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
                "/tf": "tf2_msgs/msg/TFMessage",
                "/tf_static": "tf2_msgs/msg/TFMessage",
            },
        )

    def test_cdr_clock_decoder_handles_ros_little_endian_payload(self):
        payload = b"\x00\x01\x00\x00\x02\x00\x00\x00\x80\xf0\xfa\x02"
        self.assertAlmostEqual(_cdr_time(payload, 4), 2.05)

    def test_post_start_clock_stall_fails_before_target(self):
        class FakeRclpy:
            @staticmethod
            def ok():
                return True

            @staticmethod
            def spin_once(_node, timeout_sec):
                raise AssertionError(f"stall should be detected before spin_once({timeout_sec})")

        writer = RawCaptureWriter.__new__(RawCaptureWriter)
        writer.rclpy = FakeRclpy()
        writer.node = object()
        writer.target_wall_deadline = None
        writer.first_clock_s = 0.05
        writer.last_clock_s = 0.05
        writer.target_clock_s = 0.10
        writer.target_reached = False
        writer.last_clock_progress_wall = time.monotonic() - 1.0
        writer.clock_stall_timeout_s = 0.01
        writer.started_wall = time.monotonic() - 1.0
        writer.startup_timeout_s = 10.0
        writer.failure_reason = None

        with self.assertRaisesRegex(RuntimeError, "stopped before"):
            writer.spin_until_done()
        self.assertEqual(writer.failure_reason, "clock_stalled_before_target")

    def test_close_cannot_label_unreached_horizon_complete(self):
        writer = RawCaptureWriter.__new__(RawCaptureWriter)
        writer._closed_result = None
        writer.subscriptions = []
        writer.node = None
        writer.writer = None
        writer.output = Path("unused")
        writer.rosbag2_py = object()
        writer.target_clock_s = 0.10
        writer.target_reached = False
        writer.capture_end_clock_s = 0.10
        writer.failure_reason = "clock_stalled_before_target"
        counts = {topic: 1 for topic in TOPIC_TYPES}
        first = {topic: 0.05 for topic in TOPIC_TYPES}
        last = {topic: 0.05 for topic in TOPIC_TYPES}

        writer.expected_rgb_fps = 30.0
        writer.expected_rgb_start_s = 0.0
        writer.max_rgb_startup_delay_s = 0.1
        with patch("simulator.capture.rosbag_capture._bag_observations", return_value=(counts, first, last, [0.05])):
            metadata = writer.close()

        self.assertEqual(metadata["status"], "incomplete")
        self.assertFalse(metadata["target_reached"])
        self.assertEqual(metadata["missing_topics"], [])
        self.assertEqual(metadata["failure_reason"], "clock_stalled_before_target")

    def test_cross_process_zenoh_transport_writes_all_serialized_topics(self):
        if not os.environ.get("AMENT_PREFIX_PATH"):
            self.skipTest("requires the Pixi ROS environment")
        router_executable = Path(sys.prefix) / "Library" / "lib" / "rmw_zenoh_cpp" / "rmw_zenohd.exe"
        ros2_executable = shutil.which("ros2")
        if not router_executable.exists() or ros2_executable is None:
            self.skipTest("requires rmw_zenohd and ros2")

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            bag = root / "captured"
            source = root / "source"
            metadata_path = root / "bag_metadata.json"
            rgb_path = root / "rgb.mp4"
            rgb_metadata_path = root / "rgb.json"
            rgb_frames_path = root / "rgb_frames.jsonl"
            environment = os.environ.copy()
            environment["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
            environment["ROS_DOMAIN_ID"] = str(200 + os.getpid() % 20)
            router_log_path = root / "router.log"
            router_log = router_log_path.open("w+b")
            router = subprocess.Popen(
                [str(router_executable)],
                env=environment,
                stdout=router_log,
                stderr=subprocess.STDOUT,
            )
            writer = None
            publisher = None
            try:
                time.sleep(2.0)
                self.assertIsNone(router.poll(), router_log_path.read_text(encoding="utf-8"))
                writer = subprocess.Popen(
                    [
                        sys.executable,
                        "-m",
                        "simulator.capture.rosbag_capture",
                        "--output",
                        str(bag),
                        "--metadata",
                        str(metadata_path),
                        "--duration-seconds",
                        str(2.0 + 2.0 / 30.0),
                        "--end-clock-seconds",
                        str(2.0 + 2.0 / 30.0),
                        "--startup-timeout-seconds",
                        "20",
                        "--post-target-wall-seconds",
                        "0.5",
                        "--expected-rgb-start-seconds",
                        "2.0",
                        "--rgb-video",
                        str(rgb_path),
                        "--rgb-metadata",
                        str(rgb_metadata_path),
                        "--rgb-frames-jsonl",
                        str(rgb_frames_path),
                    ],
                    cwd=Path(__file__).resolve().parents[1],
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                time.sleep(2.0)
                self.assertIsNone(writer.poll(), "raw writer exited before publisher startup")
                publisher = subprocess.Popen(
                    [sys.executable, "-m", "tests.rosbag_transport_peer", "--source", str(source)],
                    cwd=Path(__file__).resolve().parents[1],
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                publisher_output, _ = publisher.communicate(timeout=30)
                self.assertEqual(publisher.returncode, 0, publisher_output)
                self.assertIn("published_topics=5 numpy_loaded=False", publisher_output)
                writer_output, _ = writer.communicate(timeout=30)
                self.assertEqual(writer.returncode, 0, writer_output)

                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                self.assertEqual(metadata["status"], "complete")
                self.assertEqual(metadata["topics"], list(TOPIC_TYPES))
                self.assertEqual(metadata["topic_types"], TOPIC_TYPES)
                self.assertEqual(metadata["counts"]["/clock"], 3)
                self.assertEqual(metadata["counts"]["/sim/camera/rgb/image_raw"], 3)
                self.assertEqual(metadata["counts"]["/sim/lidar/points"], 3)
                self.assertEqual(metadata["counts"]["/tf"], 3)
                self.assertEqual(metadata["counts"]["/tf_static"], 1)
                self.assertEqual(metadata["missing_topics"], [])
                self.assertAlmostEqual(metadata["first_clock_s"], 2.0)
                self.assertAlmostEqual(metadata["last_clock_s"], 2.0 + 2.0 / 30.0)
                self.assertAlmostEqual(metadata["target_clock_s"], 2.0 + 2.0 / 30.0)
                self.assertTrue(metadata["target_reached"])
                self.assertEqual(metadata["capture_window_mode"], "absolute_simulation_horizon")
                self.assertIsNone(metadata["failure_reason"])
                self.assertFalse(metadata["numpy_loaded"])
                self.assertEqual(metadata["subscription_serialization"], "raw_cdr")
                self.assertTrue(metadata["rgb_cadence"]["contiguous"])
                rgb_metadata = json.loads(rgb_metadata_path.read_text(encoding="utf-8"))
                self.assertEqual(rgb_metadata["status"], "complete")
                self.assertEqual(rgb_metadata["input_source"], "shared_raw_cdr_subscription")
                self.assertEqual(rgb_metadata["frame_count"], metadata["counts"]["/sim/camera/rgb/image_raw"])
                bag_stamps = metadata["rgb_cadence"]["stamps_s"]
                video_stamps = [
                    json.loads(line)["stamp_s"]
                    for line in rgb_frames_path.read_text(encoding="utf-8").splitlines()
                ]
                self.assertEqual(video_stamps, bag_stamps)
                self.assertNotIn("/sim/camera/rgb/camera_info", metadata["topics"])

                bag_info = subprocess.run(
                    [ros2_executable, "bag", "info", str(bag)],
                    cwd=Path(__file__).resolve().parents[1],
                    env=environment,
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=20,
                ).stdout
                for topic, type_name in TOPIC_TYPES.items():
                    self.assertIn(topic, bag_info)
                    self.assertIn(type_name, bag_info)
                self.assertFalse(list(bag.glob("*.db3-wal")))
                self.assertFalse(list(bag.glob("*.db3-shm")))
            finally:
                for process in (publisher, writer, router):
                    if process is not None and process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=5)
                router_log.close()


if __name__ == "__main__":
    unittest.main()
