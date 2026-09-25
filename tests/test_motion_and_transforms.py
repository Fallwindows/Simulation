import math
import unittest
from dataclasses import replace
from types import SimpleNamespace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.motion.trajectory import PoseSample, StraightTrajectory, WalkingTrajectory, interpolate_pose
from simulator.sensors.rig import assert_no_ground_truth_odometry_leakage, build_sensor_rig_description
from simulator.runtime.isaac_sim_runner import lidar_runtime_spec
from simulator.sensors.transforms import (
    Transform,
    camera_optical_quaternion,
    camera_usd_quaternion,
    interpolate_position,
    interpolate_transform,
    quaternion_from_rpy_deg,
    quaternion_normalize,
    quaternion_slerp,
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_wxyz,
    rpy_deg_from_quaternion,
    rotate_vector,
    transform_point,
)


RIGHT_HERO_GROUPS_M = {
    9.0: ((12.11, -0.60, 1.30), (12.15, -0.83, 1.30), (12.12, -1.02, 1.30), (12.18, -1.23, 1.30)),
    17.0: ((21.26, -0.60, 1.30), (21.30, -0.83, 1.30), (21.27, -1.02, 1.30), (21.33, -1.23, 1.30)),
}

RIGHT_HERO_FOOTPRINTS_M = {
    9.0: tuple((x, y, z) for x in (11.90, 12.50) for y in (-0.52, -1.38) for z in (1.08, 1.65)),
    17.0: tuple((x, y, z) for x in (21.05, 21.65) for y in (-0.52, -1.38) for z in (1.08, 1.65)),
}


def _camera_projection_ndc(sample, point_m):
    camera_mount_m = (0.35, 0.0, 1.65)
    mount_world = rotate_vector(sample.orientation_xyzw, camera_mount_m)
    camera_world = tuple(sample.position_m[axis] + mount_world[axis] for axis in range(3))
    relative = tuple(point_m[axis] - camera_world[axis] for axis in range(3))
    forward = rotate_vector(sample.orientation_xyzw, (1.0, 0.0, 0.0))
    right = rotate_vector(sample.orientation_xyzw, (0.0, -1.0, 0.0))
    up = rotate_vector(sample.orientation_xyzw, (0.0, 0.0, 1.0))
    depth = sum(value * axis for value, axis in zip(relative, forward))
    horizontal = sum(value * axis for value, axis in zip(relative, right)) / depth
    vertical = sum(value * axis for value, axis in zip(relative, up)) / depth
    return depth, horizontal, vertical


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

    def test_eased_travel_is_rate_independent_and_bounded(self):
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        trajectory = StraightTrajectory(walking_config)
        common_30_hz = [trajectory.sample(i / 30.0) for i in range(round(walking_config.duration_s * 30.0) + 1)]
        common_60_hz = [trajectory.sample(i / 60.0) for i in range(round(walking_config.duration_s * 60.0) + 1)]
        self.assertEqual(common_30_hz, common_60_hz[::2])
        self.assertEqual(trajectory.sample(-1.0), common_30_hz[0])
        self.assertEqual(trajectory.sample(walking_config.duration_s + 1.0), common_30_hz[-1])

        for hz, samples in ((30.0, common_30_hz), (60.0, common_60_hz)):
            dt = 1.0 / hz
            x = [sample.position_m[0] for sample in samples]
            velocity = [(right - left) / dt for left, right in zip(x, x[1:])]
            acceleration = [(right - left) / dt for left, right in zip(velocity, velocity[1:])]
            jerk = [(right - left) / dt for left, right in zip(acceleration, acceleration[1:])]
            self.assertLess(velocity[0], 1e-5)
            self.assertLess(velocity[-1], 1e-5)
            self.assertTrue(all(value >= 0.0 and math.isfinite(value) for value in velocity))
            self.assertLess(max(velocity), 1.15)
            self.assertLess(max(map(abs, acceleration)), 1.05)
            self.assertLess(max(map(abs, jerk)), 1.5)

    def test_speed_variation_preserves_distance_and_forward_motion(self):
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        varied_config = replace(walking_config, speed_variation_fraction=0.3)
        trajectory = StraightTrajectory(varied_config)
        positions = [
            trajectory.sample(i / varied_config.sample_hz).position_m[0]
            for i in range(round(varied_config.duration_s * varied_config.sample_hz) + 1)
        ]
        self.assertAlmostEqual(positions[-1] - positions[0], varied_config.speed_mps * varied_config.duration_s)
        self.assertTrue(all(right > left for left, right in zip(positions, positions[1:])))

    def test_walking_pose_stays_finite_unit_and_settled_at_endpoints(self):
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        trajectory = WalkingTrajectory(walking_config)
        start = trajectory.sample(0.0)
        end = trajectory.sample(walking_config.duration_s)
        self.assertEqual(start.position_m, walking_config.start_position_m)
        self.assertEqual(end.position_m[1:], walking_config.start_position_m[1:])
        self.assertEqual(start.orientation_xyzw, quaternion_from_rpy_deg(0.0, 0.0, walking_config.yaw_deg))
        self.assertEqual(end.orientation_xyzw, start.orientation_xyzw)
        for sample in trajectory.sample_many():
            self.assertTrue(all(math.isfinite(value) for value in (*sample.position_m, *sample.orientation_xyzw)))
            self.assertAlmostEqual(math.sqrt(sum(value * value for value in sample.orientation_xyzw)), 1.0, places=12)

    def test_walking_full_pose_has_bounded_numeric_derivatives_at_30_and_60_hz(self):
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        trajectory = WalkingTrajectory(walking_config)
        for hz in (30.0, 60.0):
            dt = 1.0 / hz
            samples = [trajectory.sample(i * dt) for i in range(round(walking_config.duration_s * hz) + 1)]
            velocity = [
                tuple((right.position_m[axis] - left.position_m[axis]) / dt for axis in range(3))
                for left, right in zip(samples, samples[1:])
            ]
            acceleration = [
                tuple((right[axis] - left[axis]) / dt for axis in range(3))
                for left, right in zip(velocity, velocity[1:])
            ]
            jerk = [
                tuple((right[axis] - left[axis]) / dt for axis in range(3))
                for left, right in zip(acceleration, acceleration[1:])
            ]
            magnitude = lambda vector: math.sqrt(sum(value * value for value in vector))
            self.assertLess(magnitude(velocity[0]), 1e-5)
            self.assertLess(magnitude(velocity[-1]), 1e-5)
            self.assertLess(max(map(magnitude, velocity)), 1.2)
            self.assertLess(max(map(magnitude, acceleration)), 2.5)
            self.assertLess(max(map(magnitude, jerk)), 35.0)

    def test_configured_shelf_looks_are_c2_at_boundaries_and_peak(self):
        walking_config = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml").trajectory
        with_looks = WalkingTrajectory(walking_config)
        without_looks = WalkingTrajectory(replace(walking_config, look_beats=()))

        def look_delta(timestamp_s):
            pose = with_looks.sample(timestamp_s)
            baseline = without_looks.sample(timestamp_s)
            pose_rpy = rpy_deg_from_quaternion(pose.orientation_xyzw)
            baseline_rpy = rpy_deg_from_quaternion(baseline.orientation_xyzw)
            return (
                pose.position_m[1] - baseline.position_m[1],
                pose_rpy[1] - baseline_rpy[1],
                pose_rpy[2] - baseline_rpy[2],
            )

        h = 1e-4
        for beat in walking_config.look_beats:
            for boundary_s in (beat.center_s - beat.rise_s, beat.center_s, beat.center_s + beat.fall_s):
                left = look_delta(boundary_s - h)
                center = look_delta(boundary_s)
                right = look_delta(boundary_s + h)
                left_velocity = tuple((center[i] - left[i]) / h for i in range(3))
                right_velocity = tuple((right[i] - center[i]) / h for i in range(3))
                left_acceleration = tuple((center[i] - 2.0 * left[i] + look_delta(boundary_s - 2.0 * h)[i]) / (h * h) for i in range(3))
                right_acceleration = tuple((look_delta(boundary_s + 2.0 * h)[i] - 2.0 * right[i] + center[i]) / (h * h) for i in range(3))
                for left_value, right_value in zip(left_velocity, right_velocity):
                    self.assertAlmostEqual(left_value, right_value, delta=2e-5)
                for left_value, right_value in zip(left_acceleration, right_acceleration):
                    self.assertAlmostEqual(left_value, right_value, delta=1e-3)

    def test_right_shelf_look_path_clearance_and_hero_group_coverage(self):
        walking = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml")
        walking_config = walking.trajectory
        trajectory = WalkingTrajectory(walking_config)
        samples = trajectory.sample_many()
        self.assertEqual(walking_config.duration_s, 20.5)
        self.assertEqual(walking_config.sample_hz, 30.0)
        self.assertEqual(len(samples), 616)

        # The nearest right-side fixture edge is y=-0.535 m.  Keep at least a
        # 30 cm center clearance while the shared camera/LiDAR rig passes it.
        self.assertGreaterEqual(min(sample.position_m[1] for sample in samples), -0.235)
        sampled_rpy = [rpy_deg_from_quaternion(sample.orientation_xyzw) for sample in samples]
        dt = 1.0 / walking_config.sample_hz
        yaw_velocity = [(right[2] - left[2]) / dt for left, right in zip(sampled_rpy, sampled_rpy[1:])]
        pitch_velocity = [(right[1] - left[1]) / dt for left, right in zip(sampled_rpy, sampled_rpy[1:])]
        yaw_acceleration = [(right - left) / dt for left, right in zip(yaw_velocity, yaw_velocity[1:])]
        pitch_acceleration = [(right - left) / dt for left, right in zip(pitch_velocity, pitch_velocity[1:])]
        self.assertLess(max(map(abs, yaw_velocity)), 40.0)
        self.assertLess(max(map(abs, pitch_velocity)), 30.0)
        self.assertLess(max(map(abs, yaw_acceleration)), 120.0)
        self.assertLess(max(map(abs, pitch_acceleration)), 240.0)
        for timestamp_s, group in RIGHT_HERO_GROUPS_M.items():
            sample = trajectory.sample(timestamp_s)
            _roll, pitch_deg, yaw_deg = rpy_deg_from_quaternion(sample.orientation_xyzw)
            self.assertLess(yaw_deg, -20.0)
            self.assertGreater(pitch_deg, 8.0)
            projections = [_camera_projection_ndc(sample, point) for point in group]
            self.assertTrue(all(depth > 0.8 for depth, _horizontal, _vertical in projections))
            self.assertTrue(all(abs(horizontal) < 1.0 for _depth, horizontal, _vertical in projections))
            # 16:9 vertical half-FOV is tan^-1(9/16) for the configured 90° horizontal FOV.
            self.assertTrue(all(abs(vertical) < 9.0 / 16.0 for _depth, _horizontal, vertical in projections))
            horizontal_span = max(horizontal for _depth, horizontal, _vertical in projections) - min(horizontal for _depth, horizontal, _vertical in projections)
            self.assertGreater(horizontal_span, 0.25)
            footprint = [_camera_projection_ndc(sample, point) for point in RIGHT_HERO_FOOTPRINTS_M[timestamp_s]]
            self.assertTrue(all(abs(horizontal) < 0.8 for _depth, horizontal, _vertical in footprint))
            self.assertTrue(all(abs(vertical) < 9.0 / 16.0 for _depth, _horizontal, vertical in footprint))

            rig_world = Transform("sim_world", "sensor_rig", sample.position_m, sample.orientation_xyzw)
            camera_world = transform_point(rig_world, walking.camera.pose_in_rig.position_m)
            lidar_world = transform_point(rig_world, walking.lidar.pose_in_rig.position_m)
            static_baseline = math.dist(walking.camera.pose_in_rig.position_m, walking.lidar.pose_in_rig.position_m)
            self.assertAlmostEqual(math.dist(camera_world, lidar_world), static_baseline, places=12)
            self.assertLess(rotate_vector(sample.orientation_xyzw, (1.0, 0.0, 0.0))[1], -0.3)

    def test_pose_interpolation_is_continuous_and_uses_shortest_rotation(self):
        start = PoseSample(2.0, (1.0, -2.0, 0.5), quaternion_from_rpy_deg(0.0, 0.0, 170.0))
        end = PoseSample(4.0, (5.0, 2.0, 2.5), quaternion_from_rpy_deg(0.0, 0.0, -170.0))
        midpoint = interpolate_pose(start, end, 3.0)
        self.assertEqual(midpoint.position_m, (3.0, 0.0, 1.5))
        self.assertAlmostEqual(abs(rpy_deg_from_quaternion(midpoint.orientation_xyzw)[2]), 180.0, places=6)
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in midpoint.orientation_xyzw)), 1.0, places=12)
        with self.assertRaisesRegex(ValueError, "does not extrapolate"):
            interpolate_pose(start, end, 4.1)

    def test_transform_interpolation_preserves_frames_and_rejects_bad_inputs(self):
        start = Transform("map", "rig", (0.0, 0.0, 0.0), quaternion_from_rpy_deg(0.0, 0.0, 20.0))
        end = Transform("map", "rig", (4.0, -2.0, 6.0), quaternion_from_rpy_deg(0.0, 0.0, 80.0))
        midpoint = interpolate_transform(start, end, 0.5)
        self.assertEqual((midpoint.parent, midpoint.child), ("map", "rig"))
        self.assertEqual(midpoint.translation_m, (2.0, -1.0, 3.0))
        self.assertAlmostEqual(rpy_deg_from_quaternion(midpoint.rotation_xyzw)[2], 50.0, places=6)
        same_rotation = quaternion_slerp(start.rotation_xyzw, tuple(-value for value in start.rotation_xyzw), 0.5)
        self.assertAlmostEqual(abs(sum(a * b for a, b in zip(start.rotation_xyzw, same_rotation))), 1.0, places=12)
        with self.assertRaisesRegex(ValueError, "fraction"):
            interpolate_position((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), math.nan)
        with self.assertRaisesRegex(ValueError, "finite"):
            quaternion_slerp((0.0, 0.0, 0.0, math.inf), (0.0, 0.0, 0.0, 1.0), 0.5)

    def test_extreme_finite_quaternion_and_position_inputs_remain_finite(self):
        normalized = quaternion_normalize((1e308, 0.0, 0.0, 0.0))
        self.assertEqual(normalized, (1.0, 0.0, 0.0, 0.0))
        midpoint = interpolate_position((1e308, -1e308, 1e308), (-1e308, 1e308, 1e308), 0.5)
        self.assertEqual(midpoint, (0.0, 0.0, 1e308))
        self.assertTrue(all(math.isfinite(value) for value in midpoint))

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
