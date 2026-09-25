import time
import unittest
from pathlib import Path

from evaluation.rgb_video_recorder import RgbVideoRecorder
from simulator.capture.rosbag_capture import _maximum_nearest_skew_s
from simulator.config.loader import load_scenario
from simulator.sensors.rig import make_camera_intrinsics


ROOT = Path(__file__).resolve().parents[1]


class CaptureGateTests(unittest.TestCase):
    def test_raw_alignment_uses_worst_nearest_rgb_sample(self):
        self.assertAlmostEqual(
            _maximum_nearest_skew_s([0.1, 0.2, 0.3], [0.083333333, 0.2, 0.316666667]),
            0.016666667,
            places=9,
        )
        self.assertIsNone(_maximum_nearest_skew_s([], [0.0]))

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
        self.assertIn('Raw bag did not reach the absolute scenario horizon.', source)
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
        self.assertIn('$rgbCountBoundaryTolerance = 1', source)
        self.assertIn('rgb_reconciliation=', source)
        self.assertLess(source.index('RGB video cadence is not contiguous and valid.'), source.index('CAPTURE_COMPLETE'))

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
