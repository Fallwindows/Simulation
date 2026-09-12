import unittest

from dashboard.backend.state import DashboardState
from evaluation.metrics import compute_metrics
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
