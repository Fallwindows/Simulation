import time
import unittest
import os
import queue
import subprocess
import threading
from pathlib import Path
from unittest.mock import patch

from evaluation.rgb_video_recorder import RgbVideoRecorder
from evaluation.rgb_video_from_bag import BagRgbVideoBuilder
from simulator.capture.rosbag_capture import (
    _SerializedBagWriteQueue,
    _maximum_nearest_skew_s,
    _strictly_increasing_stamps,
)
from simulator.capture.stamp_digest import stamp_sequence_sha256
from simulator.config.loader import load_scenario
from simulator.sensors.rig import make_camera_intrinsics


ROOT = Path(__file__).resolve().parents[1]


class CaptureGateTests(unittest.TestCase):
    def test_stamp_digest_is_ordered_and_nanosecond_canonical(self):
        first = stamp_sequence_sha256([0.0, 1.0 / 30.0, 2.0 / 30.0])
        self.assertEqual(first, stamp_sequence_sha256([0, 0.033333333333, 0.066666666667]))
        self.assertNotEqual(first, stamp_sequence_sha256([0.0, 2.0 / 30.0, 1.0 / 30.0]))

    def test_async_bag_queue_drains_and_fails_on_bounded_overflow(self):
        class Writer:
            def __init__(self):
                self.writes = []
                self.close_calls = 0

            def write(self, *args):
                self.writes.append(args)

            def close(self):
                self.close_calls += 1

        writer = Writer()
        work = _SerializedBagWriteQueue(writer, max_items=4, max_bytes=16)
        work.submit("/first", b"1234", 1)
        work.submit("/second", b"5678", 2)
        self.assertIsNone(work.finish(1.0))
        receipt = work.receipt()
        self.assertEqual(receipt["submitted_count"], 2)
        self.assertEqual(receipt["written_count"], 2)
        self.assertGreaterEqual(receipt["high_water_items"], 1)
        self.assertTrue(receipt["drained"])
        self.assertEqual(receipt["overflow_count"], 0)
        self.assertEqual(writer.close_calls, 1)

        overflow_writer = Writer()
        overflow = _SerializedBagWriteQueue(overflow_writer, max_items=2, max_bytes=2)
        with self.assertRaisesRegex(OverflowError, "byte budget"):
            overflow.submit("/too-large", b"123", 3)
        self.assertIsNone(overflow.finish(1.0))
        self.assertEqual(overflow.receipt()["overflow_count"], 1)

        class FailingWriter(Writer):
            def write(self, *_args):
                raise OSError("storage failed")

        failing = _SerializedBagWriteQueue(FailingWriter(), max_items=2, max_bytes=16)
        failing.submit("/failure", b"123", 4)
        self.assertRegex(str(failing.finish(1.0)), "storage failed")
        failure_receipt = failing.receipt()
        self.assertFalse(failure_receipt["drained"])
        self.assertIn("OSError", failure_receipt["worker_error"])

    def test_async_bag_high_water_counts_item_even_if_worker_dequeues_immediately(self):
        class ImmediateDequeueQueue(queue.Queue):
            dequeued = threading.Event()

            def put_nowait(self, item):
                super().put_nowait(item)
                self.dequeued.wait(1.0)

            def get(self, *args, **kwargs):
                item = super().get(*args, **kwargs)
                if item is not _SerializedBagWriteQueue._STOP:
                    self.dequeued.set()
                return item

        class Writer:
            def write(self, *_args):
                pass

            def close(self):
                pass

        with patch("simulator.capture.rosbag_capture.queue.Queue", ImmediateDequeueQueue):
            work = _SerializedBagWriteQueue(Writer(), max_items=2, max_bytes=16)
            work.submit("/rgb", b"1234", 1)
            self.assertIsNone(work.finish(1.0))
        receipt = work.receipt()
        self.assertEqual(receipt["high_water_items"], 1)
        self.assertEqual(receipt["high_water_bytes"], 4)

    def test_offline_video_uses_exact_closed_bag_message_count(self):
        fixture = ROOT / "runs/offline_rgb_builder_fixture"
        fixture.mkdir(parents=True, exist_ok=True)
        paths = {
            "output": fixture / "rgb.mp4",
            "metadata": fixture / "rgb.json",
            "frames": fixture / "frames.jsonl",
            "camera": fixture / "camera.json",
        }
        for path in paths.values():
            path.unlink(missing_ok=True)

        class Writer:
            def __init__(self, output):
                self.output = output
                self.frames = 0

            def write(self, _frame):
                self.frames += 1
                self.output.write_bytes(b"offline-video")

            def release(self):
                pass

        def image(stamp_s):
            seconds = int(stamp_s)
            nanoseconds = int(round((stamp_s - seconds) * 1_000_000_000.0))
            header = type("Header", (), {"stamp": type("Stamp", (), {"sec": seconds, "nanosec": nanoseconds})(), "frame_id": "camera_optical_frame"})()
            return type("Image", (), {"header": header, "width": 2, "height": 1, "step": 6, "encoding": "rgb8", "data": bytes((10, 20, 30, 40, 50, 60))})()

        def camera_info(stamp_s):
            message = type("CameraInfo", (), {})()
            message.header = image(stamp_s).header
            message.width = 2
            message.height = 1
            message.distortion_model = "plumb_bob"
            message.d = []
            message.k = [1.0] * 9
            message.r = [1.0] * 9
            message.p = [1.0] * 12
            return message

        builder = BagRgbVideoBuilder(
            paths["output"], paths["metadata"], paths["frames"], paths["camera"],
            1.0 / 30.0, 2, 1, 30.0, "fixture-bag",
        )
        writer = Writer(paths["output"])
        builder.writer = writer
        builder.consume_image(image(0.0))
        builder.consume_image(image(1.0 / 30.0))
        builder.consume_camera_info(camera_info(0.0))
        builder.consume_camera_info(camera_info(1.0 / 30.0))
        decoded_receipt = {"opened": True, "frame_count": 2, "widths": [2], "heights": [1], "reported_fps": 30.0}
        with patch("evaluation.rgb_video_from_bag._probe_decoded_video", return_value=decoded_receipt):
            metadata = builder.finalize(expected_image_count=2, expected_camera_info_count=2)
        self.assertEqual(metadata["status"], "complete")
        self.assertEqual(metadata["source"], "closed_rosbag2")
        self.assertEqual(metadata["stamp_sha256"], stamp_sequence_sha256([0.0, 1.0 / 30.0]))
        self.assertEqual(len(paths["frames"].read_text(encoding="utf-8").splitlines()), 2)
        self.assertEqual(writer.frames, 2)
        for path in paths.values():
            path.unlink(missing_ok=True)

    def test_offline_video_fails_when_closed_file_decodes_fewer_frames(self):
        fixture = ROOT / "runs/offline_rgb_decode_gate_fixture"
        fixture.mkdir(parents=True, exist_ok=True)
        output = fixture / "rgb.mp4"
        metadata_path = fixture / "rgb.json"
        frames_path = fixture / "frames.jsonl"
        camera_path = fixture / "camera.json"
        for path in (output, metadata_path, frames_path, camera_path):
            path.unlink(missing_ok=True)

        class SilentWriter:
            def write(self, _frame):
                output.write_bytes(b"container-with-no-decoded-frames")

            def release(self):
                pass

        builder = BagRgbVideoBuilder(output, metadata_path, frames_path, camera_path, 1.0 / 30.0, 2, 1, 30.0, "fixture-bag")
        builder.writer = SilentWriter()
        builder.first_stamp_s = 0.0
        builder.last_stamp_s = 1.0 / 30.0
        builder.max_frame_gap_s = 1.0 / 30.0
        builder.frame_ids = {"camera_optical_frame"}
        builder.frame_records = [
            {"frame_index": 0, "stamp_s": 0.0, "frame_id": "camera_optical_frame", "width": 2, "height": 1, "encoding": "rgb8"},
            {"frame_index": 1, "stamp_s": 1.0 / 30.0, "frame_id": "camera_optical_frame", "width": 2, "height": 1, "encoding": "rgb8"},
        ]
        builder.camera_info_count = 2
        builder.camera_info_record = {"frame_id": "camera_optical_frame", "width": 2, "height": 1}
        decoded_receipt = {"opened": True, "frame_count": 0, "widths": [], "heights": [], "reported_fps": 30.0}
        with patch("evaluation.rgb_video_from_bag._probe_decoded_video", return_value=decoded_receipt):
            with self.assertRaisesRegex(RuntimeError, "offline RGB video validation failed"):
                builder.finalize(expected_image_count=2, expected_camera_info_count=2)
        receipt = __import__("json").loads(metadata_path.read_text(encoding="utf-8"))
        self.assertEqual(receipt["status"], "failed")
        self.assertFalse(receipt["decoded_video_valid"])
        for path in (output, metadata_path, frames_path, camera_path):
            path.unlink(missing_ok=True)

    def test_raw_alignment_uses_worst_nearest_rgb_sample(self):
        self.assertAlmostEqual(
            _maximum_nearest_skew_s([0.1, 0.2, 0.3], [0.083333333, 0.2, 0.316666667]),
            0.016666667,
            places=9,
        )
        self.assertIsNone(_maximum_nearest_skew_s([], [0.0]))

    def test_raw_rgb_timestamps_reject_duplicates_and_regressions(self):
        self.assertTrue(_strictly_increasing_stamps([0.0, 1.0 / 30.0, 2.0 / 30.0]))
        self.assertFalse(_strictly_increasing_stamps([0.0, 0.0, 1.0 / 30.0]))
        self.assertFalse(_strictly_increasing_stamps([0.0, 1.0 / 30.0, 0.02]))

    def test_large_message_zenoh_and_camera_queues_are_source_controlled(self):
        for name in ("production_zenoh_session.json5", "production_zenoh_router.json5"):
            source = (ROOT / "config/ros2" / name).read_text(encoding="utf-8")
            self.assertIn("data: 16,", source)
            self.assertIn("buffer_size: 16777216,", source)
            self.assertNotIn("data: 2,", source)
            self.assertNotIn("buffer_size: 65535,", source)
        runner = (ROOT / "simulator/runtime/isaac_sim_runner.py").read_text(encoding="utf-8")
        self.assertIn('(\"Rgb.inputs:queueSize\", 128)', runner)
        self.assertIn('(\"CameraInfo.inputs:queueSize\", 128)', runner)
        self.assertIn('node.create_subscription(Image, TOPICS[\"rgb_image\"], _on_rgb, 128)', runner)

    def test_rgb_clock_target_is_absolute_scenario_horizon(self):
        recorder = RgbVideoRecorder.__new__(RgbVideoRecorder)
        recorder.last_clock_s = None
        recorder.first_clock_s = None
        recorder.duration_s = 20.5
        recorder.target_s = None
        recorder.post_target_deadline = None
        recorder.clock_regressions = 0
        recorder.last_progress_wall = time.monotonic()
        message = type("ClockMessage", (), {
            "clock": type("Clock", (), {"sec": 5, "nanosec": 0})(),
        })()
        recorder._on_clock(message)
        self.assertEqual(recorder.first_clock_s, 5.0)
        self.assertEqual(recorder.target_s, 20.5)

    def test_production_launcher_forwards_sensor_gate_and_validates_receipts(self):
        source = (ROOT / "scripts/capture_simulation.ps1").read_text(encoding="utf-8")
        self.assertIn("RequireSensorSamples=$true", source)
        self.assertIn('StartupTimeoutSeconds = 600', source)
        self.assertIn('ProgressTimeoutSeconds = 60', source)
        self.assertIn('isaac_runtime_status.json', source)
        self.assertIn('Raw bag topic contract must contain exactly the six required topics.', source)
        self.assertIn('Raw bag did not reach the requested capture horizon.', source)
        self.assertIn('Raw bag RGB/LiDAR timestamp skew exceeds 17,000,001 ns.', source)
        self.assertIn('RGB video cadence is not contiguous and valid.', source)
        self.assertIn('walking_production_1080p.yaml', source)
        self.assertIn('Production capture requires native camera resolution of at least 1920x1080.', source)
        self.assertIn('camera_info_provenance="observed_ros_message"', source)
        self.assertIn('sensor_config_sha256=', source)
        self.assertIn('Production capture requires a clean source worktree.', source)
        self.assertIn('git_tree=$gitTree', source)
        self.assertIn('git_worktree_clean=$true', source)
        self.assertIn('Raw bag RGB cadence gap exceeds the configured frame period plus 1 ms.', source)
        self.assertIn('Raw bag RGB count does not reconcile with the RGB video frame count.', source)
        self.assertIn('Raw bag RGB image and CameraInfo counts do not match.', source)
        self.assertIn('$rgbCountBoundaryTolerance = 0', source)
        self.assertIn('rgb_reconciliation=', source)
        self.assertIn('Raw bag RGB timestamps must be strictly increasing with no duplicates.', source)
        self.assertIn('Production capture requires -Realtime pacing for lossless ROS consumers.', source)
        self.assertIn('requested_realtime_factor=$RealtimeFactor', source)
        self.assertIn('Isaac runtime pacing receipt does not match the requested realtime factor.', source)
        self.assertIn('evaluation.rgb_video_from_bag', source)
        self.assertIn('Raw bag asynchronous writer did not drain losslessly.', source)
        self.assertIn('Raw bag RGB and CameraInfo stamps are not paired exactly.', source)
        self.assertIn('Isaac, raw RGB, CameraInfo, and offline video stamp sequences do not match exactly.', source)
        self.assertIn('Closed RGB video decode audit did not match the source frame index.', source)
        self.assertIn('$captureHorizon', source)
        self.assertNotIn('evaluation.rgb_video_recorder', source)
        self.assertIn('production_zenoh_session.json5', source)
        self.assertIn('production_zenoh_router.json5', source)
        self.assertEqual(source.count('Assert-CaptureSourceUnchanged'), 2)
        self.assertLess(source.rindex('Assert-CaptureSourceUnchanged'), source.index('CAPTURE_COMPLETE'))
        self.assertLess(source.index('RGB video cadence is not contiguous and valid.'), source.index('CAPTURE_COMPLETE'))

    def test_capture_source_guard_rejects_tracked_mutation(self):
        guard = ROOT / "scripts/capture_source_guard.ps1"
        repository = ROOT / "runs/capture_source_guard_fixture"
        repository.mkdir(parents=True, exist_ok=True)
        git_command = repository / "git.cmd"
        dirty_flag = repository / "dirty.flag"
        dirty_flag.unlink(missing_ok=True)
        try:
            sha = "a" * 40
            tree = "b" * 40
            git_command.write_text(
                "@echo off\n"
                f'if "%3"=="rev-parse" if "%4"=="HEAD" echo {sha}\n'
                f'if "%3"=="rev-parse" if not "%4"=="HEAD" echo {tree}\n'
                'if "%3"=="status" if exist "%2\\dirty.flag" echo  M tracked.txt\n',
                encoding="ascii",
            )
            command = (
                f". '{guard}'; Assert-CaptureSourceUnchanged -Repo '{repository}' "
                f"-ExpectedSha '{sha}' -ExpectedTree '{tree}'"
            )
            environment = dict(os.environ)
            environment["PATH"] = f"{repository}{os.pathsep}{environment['PATH']}"
            clean = subprocess.run(
                ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
                capture_output=True,
                text=True,
                env=environment,
            )
            self.assertEqual(clean.returncode, 0, clean.stderr)
            dirty_flag.write_text("tracked mutation\n", encoding="utf-8")
            dirty = subprocess.run(
                ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
                capture_output=True,
                text=True,
                env=environment,
            )
            self.assertNotEqual(dirty.returncode, 0)
            self.assertIn("Capture source changed or became dirty", dirty.stderr)
        finally:
            dirty_flag.unlink(missing_ok=True)
            git_command.unlink(missing_ok=True)

    def test_production_camera_is_native_1080_with_scaled_intrinsics(self):
        preview = load_scenario(ROOT / "config/scenarios/walking_baseline.yaml")
        production = load_scenario(ROOT / "config/scenarios/walking_production_1080p.yaml")
        self.assertEqual((production.camera.width_px, production.camera.height_px), (1920, 1080))
        self.assertEqual(production.camera.horizontal_fov_deg, preview.camera.horizontal_fov_deg)
        self.assertEqual(production.camera.pose_in_rig, preview.camera.pose_in_rig)
        self.assertEqual(production.lidar, preview.lidar)
        preview_intrinsics = make_camera_intrinsics(preview.camera)
        production_intrinsics = make_camera_intrinsics(production.camera)
        self.assertAlmostEqual(production_intrinsics.fx_px, 960.0)
        self.assertAlmostEqual(production_intrinsics.fy_px, 960.0)
        self.assertAlmostEqual(production_intrinsics.cx_px, 959.5)
        self.assertAlmostEqual(production_intrinsics.cy_px, 539.5)
        self.assertAlmostEqual(production_intrinsics.fx_px / preview_intrinsics.fx_px, 1.5)


if __name__ == "__main__":
    unittest.main()
