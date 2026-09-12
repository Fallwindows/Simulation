import time
import unittest

from dashboard.backend.ros_bridge import RosDashboardBridge
from dashboard.backend.state import DashboardState
from evaluation.metrics import PoseSample, compute_metrics
from simulator.sensors.noise import NoiseConfig, apply_lidar_noise, jitter_timestamp


class DashboardTests(unittest.TestCase):
    def test_point_budget_decimation_is_deterministic(self):
        state = DashboardState()
        points = [(float(i), 0.0, 0.0) for i in range(10)]
        state.update_points("lidar", points, receive_time=100.0)
        self.assertEqual(len(state.decimated_points("lidar", 4)), 4)
        self.assertEqual(state.decimated_points("lidar", 4), state.decimated_points("lidar", 4))

    def test_status_serialization(self):
        state = DashboardState()
        state.update_points("map", [(1.0, 2.0, 3.0)], receive_time=100.0)
        snapshot = state.snapshot()
        self.assertTrue(snapshot["status"]["map_connected"])
        self.assertEqual(snapshot["status"]["map_point_count"], 1)

    def test_tracking_becomes_stale_or_lost_when_odom_stops(self):
        bridge = object.__new__(RosDashboardBridge)
        bridge._ground_truth = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(0.8, (1.0, 0.0, 0.0))]
        bridge._estimate = [PoseSample(0.0, (0.0, 0.0, 0.0))]
        bridge._alignment = None
        self.assertEqual(bridge._tracking_payload()["tracking_state"], "stale")
        bridge._ground_truth.append(PoseSample(3.0, (2.0, 0.0, 0.0)))
        self.assertEqual(bridge._tracking_payload()["tracking_state"], "lost")

    def test_live_metric_recompute_is_throttled(self):
        state = DashboardState()
        bridge = object.__new__(RosDashboardBridge)
        bridge.state = state
        bridge._ground_truth = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(1.0, (1.0, 0.0, 0.0))]
        bridge._estimate = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(1.0, (1.0, 0.0, 0.0))]
        bridge._alignment = None
        bridge._last_metrics_refresh = time.monotonic()
        bridge._refresh_live_metrics()
        self.assertNotIn("ate_rmse_m", state.snapshot()["metrics"])
        bridge._last_metrics_refresh = 0.0
        bridge._refresh_live_metrics()
        self.assertAlmostEqual(state.snapshot()["metrics"]["ate_rmse_m"], 0.0)


class MetricTests(unittest.TestCase):
    def test_perfect_match(self):
        samples = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0)), (2.0, (2.0, 0.0, 0.0))]
        metrics = compute_metrics(samples, samples)
        self.assertAlmostEqual(metrics.ate_rmse_m, 0.0)
        self.assertAlmostEqual(metrics.distance_traveled_m, 2.0)

    def test_offset_is_measured(self):
        gt = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0))]
        estimate = [(0.0, (0.1, 0.0, 0.0)), (1.0, (1.1, 0.0, 0.0))]
        self.assertAlmostEqual(compute_metrics(gt, estimate).ate_rmse_m, 0.1)

    def test_initial_translation_alignment_is_explicit(self):
        gt = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0))]
        estimate = [(0.0, (0.1, 0.0, 0.0)), (1.0, (1.1, 0.0, 0.0))]
        metrics = compute_metrics(gt, estimate, alignment="initial_translation")
        self.assertAlmostEqual(metrics.ate_rmse_m, 0.0)
        self.assertEqual(metrics.alignment_policy, "initial_translation")

    def test_initial_se3_alignment_handles_translation_and_rotation(self):
        qz90 = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
        gt = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(1.0, (1.0, 0.0, 0.0))]
        estimate = [PoseSample(0.0, (5.0, -2.0, 0.0), qz90), PoseSample(1.0, (5.0, -1.0, 0.0), qz90)]
        metrics = compute_metrics(gt, estimate, max_time_gap_s=0.1, alignment="initial_se3")
        self.assertAlmostEqual(metrics.ate_rmse_m, 0.0, places=6)
        self.assertAlmostEqual(metrics.rpe_rmse_m or 0.0, 0.0, places=6)
        self.assertAlmostEqual(metrics.orientation_rmse_deg or 0.0, 0.0, places=6)

    def test_zero_estimate_quaternion_is_safe(self):
        ground_truth = [PoseSample(0.0, (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
        estimate = [PoseSample(0.0, (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 0.0))]
        metrics = compute_metrics(ground_truth, estimate, alignment="initial_se3")
        self.assertEqual(metrics.sample_count, 1)
        self.assertAlmostEqual(metrics.orientation_rmse_deg or 0.0, 0.0, places=6)

    def test_rpe_detects_wrong_direction(self):
        gt = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0)), (2.0, (2.0, 0.0, 0.0))]
        estimate = [(0.0, (0.0, 0.0, 0.0)), (1.0, (-1.0, 0.0, 0.0)), (2.0, (-2.0, 0.0, 0.0))]
        self.assertAlmostEqual(compute_metrics(gt, estimate, rpe_interval_s=1.0).rpe_rmse_m, 2.0)

    def test_matching_gap_is_bounded_and_loss_is_contiguous(self):
        gt = [(0.0, (0.0, 0.0, 0.0)), (1.0, (1.0, 0.0, 0.0)), (2.0, (2.0, 0.0, 0.0)), (3.0, (3.0, 0.0, 0.0))]
        estimate = [(0.0, (0.0, 0.0, 0.0)), (3.0, (3.0, 0.0, 0.0))]
        metrics = compute_metrics(gt, estimate, max_time_gap_s=0.1)
        self.assertEqual(metrics.sample_count, 2)
        self.assertEqual(metrics.tracking_loss_count, 1)

    def test_ideal_noise_mode_is_a_noop(self):
        points = [(1.0, 2.0, 3.0)]
        ideal = NoiseConfig()
        self.assertEqual(apply_lidar_noise(points, ideal, 7), points)
        self.assertEqual(jitter_timestamp(1.0, ideal, 7), 1.0)

    def test_enabled_noise_is_seeded(self):
        config = NoiseConfig(enabled=True, range_std_m=0.01, dropout_probability=0.0, timestamp_jitter_ms=1.0)
        self.assertEqual(apply_lidar_noise([(1.0, 0.0, 0.0)], config, 7), apply_lidar_noise([(1.0, 0.0, 0.0)], config, 7))
        self.assertEqual(jitter_timestamp(1.0, config, 7), jitter_timestamp(1.0, config, 7))
