import math
import unittest
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory
from simulator.sensors.rig import assert_no_ground_truth_odometry_leakage, build_sensor_rig_description
from simulator.sensors.transforms import Transform, camera_optical_quaternion, transform_point


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

    def test_transform_applies_translation_and_rotation(self):
        transform = Transform("a", "b", (1.0, 2.0, 3.0), camera_optical_quaternion())
        result = transform_point(transform, (1.0, 0.0, 0.0))
        self.assertEqual(len(result), 3)
        self.assertTrue(all(math.isfinite(v) for v in result))

    def test_sensor_rig_contract_keeps_truth_separate(self):
        rig = build_sensor_rig_description(self.scenario.camera, self.scenario.lidar)
        assert_no_ground_truth_odometry_leakage(rig)
        self.assertEqual(rig.frames["camera_optical"], "camera_optical_frame")
        self.assertGreater(rig.camera_intrinsics.fx_px, 0.0)
