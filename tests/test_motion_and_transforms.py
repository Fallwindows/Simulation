import math
import unittest
from types import SimpleNamespace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory
from simulator.sensors.rig import assert_no_ground_truth_odometry_leakage, build_sensor_rig_description
from simulator.runtime.isaac_sim_runner import lidar_runtime_spec
from simulator.sensors.transforms import (
    Transform,
    camera_optical_quaternion,
    camera_usd_quaternion,
    quaternion_from_rpy_deg,
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_wxyz,
    rpy_deg_from_quaternion,
    rotate_vector,
    transform_point,
)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.scenario = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/baseline_straight.yaml")

    def test_straight_distance(self):
        trajectory = StraightTrajectory(self.scenario.trajectory)
        start, end = trajectory.sample(0.0), trajectory.sample(self.scenario.trajectory.duration_s)
        self.assertAlmostEqual(end.position_m[0] - start.position_m[0], self.scenario.trajectory.speed_mps * self.scenario.trajectory.duration_s)

    def test_walking_is_bounded_and_changes_orientation(self):
        walking = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml")
        trajectory = WalkingTrajectory(walking.trajectory)
        samples = trajectory.sample_many()
        self.assertTrue(any(abs(s.position_m[1]) > 0 for s in samples))
        self.assertTrue(any(abs(s.orientation_xyzw[0]) > 1e-6 for s in samples))

    def test_straight_and_walking_match_nominal_forward_distance(self):
        straight = StraightTrajectory(self.scenario.trajectory)
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        walking = WalkingTrajectory(walking_config)
        straight_distance = straight.sample(self.scenario.trajectory.duration_s).position_m[0] - straight.sample(0.0).position_m[0]
        walking_distance = walking.sample(walking_config.duration_s).position_m[0] - walking.sample(0.0).position_m[0]
        self.assertAlmostEqual(straight_distance, walking_distance, places=6)
        self.assertAlmostEqual(self.scenario.trajectory.speed_mps, walking_config.speed_mps, places=6)

    def test_transform_applies_translation_and_rotation(self):
        transform = Transform("a", "b", (1.0, 2.0, 3.0), camera_optical_quaternion())
        result = transform_point(transform, (1.0, 0.0, 0.0))
        self.assertEqual(len(result), 3)
        self.assertTrue(all(math.isfinite(v) for v in result))

    def test_rpy_round_trip_preserves_full_orientation(self):
        original = (0.0, -7.5, 4.25)
        recovered = rpy_deg_from_quaternion(quaternion_from_rpy_deg(*original))
        for expected, actual in zip(original, recovered):
            self.assertAlmostEqual(expected, actual, places=6)

    def test_camera_usd_basis_and_ros_optical_basis(self):
        usd = camera_usd_quaternion()
        self.assertEqual(rotate_vector(usd, (0.0, 0.0, -1.0)), (1.0, 0.0, 0.0))
        self.assertEqual(rotate_vector(usd, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))
        self.assertEqual(rotate_vector(usd, (1.0, 0.0, 0.0)), (0.0, -1.0, 0.0))
        optical = camera_optical_quaternion()
        self.assertEqual(rotate_vector(optical, (1.0, 0.0, 0.0)), (0.0, -1.0, 0.0))
        self.assertEqual(rotate_vector(optical, (0.0, 1.0, 0.0)), (0.0, 0.0, -1.0))
        self.assertEqual(rotate_vector(optical, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))

    def test_sensor_rig_contract_keeps_truth_separate(self):
        rig = build_sensor_rig_description(self.scenario.camera, self.scenario.lidar)
        assert_no_ground_truth_odometry_leakage(rig)
        self.assertEqual(rig.frames["camera_optical"], "camera_optical_frame")
        self.assertGreater(rig.camera_intrinsics.fx_px, 0.0)

    def test_lidar_config_reaches_isaac_sensor_boundary(self):
        spec = lidar_runtime_spec(self.scenario.lidar)
        self.assertEqual(spec["tick_rate_hz"], self.scenario.lidar.hz)
        self.assertEqual(spec["near_range_m"], self.scenario.lidar.min_range_m)
        self.assertEqual(spec["far_range_m"], self.scenario.lidar.max_range_m)
        self.assertEqual(spec["translation_m"], list(self.scenario.lidar.pose_in_rig.position_m))
        self.assertEqual(len(spec["orientation_xyzw_ros"]), 4)
        self.assertEqual(len(spec["orientation_wxyz_isaac"]), 4)
        self.assertEqual(spec["mount_frame"], "sensor_rig")
        self.assertEqual(spec["mount_semantics"], "rig_relative")
        self.assertEqual(spec["sensor_asset"], "Example_Rotary")
        self.assertIn("scan_pattern_source", spec)

    def test_lidar_nonzero_mount_preserves_ros_basis_after_isaac_conversion(self):
        lidar = SimpleNamespace(
            hz=10.0,
            min_range_m=0.2,
            max_range_m=60.0,
            preset="test",
            vertical_fov_deg=(-20.0, 10.0),
            horizontal_samples=16,
            vertical_samples=4,
            pose_in_rig=SimpleNamespace(position_m=(0.25, -0.1, 1.4), rpy_deg=(17.0, -23.0, 31.0)),
        )
        spec = lidar_runtime_spec(lidar)
        ros_q = tuple(spec["orientation_xyzw_ros"])
        isaac_q = tuple(spec["orientation_wxyz_isaac"])
        self.assertEqual(quaternion_wxyz_to_xyzw(isaac_q), ros_q)
        for basis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            expected = rotate_vector(ros_q, basis)
            actual = rotate_vector(quaternion_wxyz_to_xyzw(isaac_q), basis)
            for expected_value, actual_value in zip(expected, actual):
                self.assertAlmostEqual(expected_value, actual_value, places=7)
