from __future__ import annotations

import ast
from argparse import Namespace
import copy
import contextlib
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import unittest
from unittest import mock

import robot_spike.production.run_locomotion_smoke as smoke_module
from robot_spike.production.isaac_feedback import (
    ASIMOV_SOLE_GEOMETRY,
    FeedbackUnavailableError,
    Isaac61LocomotionFeedbackAdapter,
    convex_hull_xy,
    support_polygon_center_and_margin,
)
from robot_spike.production.locomotion import (
    FootFeedback,
    FootPose,
    GaitConfig,
    LEG_JOINTS,
    LocomotionFeedback,
    PlanarPose,
)
from robot_spike.production.run_locomotion_smoke import (
    EVIDENCE_FILES,
    ISAAC_API_SOURCE_FILES,
    EvidenceIdentityGuard,
    EvidenceIdentityMismatchError,
    StartupValidationError,
    _execute_smoke,
    _interpolate_joint_targets,
    _persist_feedback_failure,
    _persist_pre_shutdown_runtime_error,
    _record_identity_boundary,
    _startup_step_budget,
    _startup_feedback_failures,
    _staged_double_support_startup,
    _terminate_process,
    _validate_arguments,
    acceptance_spec_identity,
)


ROOT = Path(__file__).resolve().parents[1]


class Array:
    def __init__(self, values):
        self.values = values

    def numpy(self):
        return self

    def tolist(self):
        return self.values


class FakeArticulation:
    dof_names = list(LEG_JOINTS)
    link_names = ["pelvis_link", "left_ankle_roll_link", "right_ankle_roll_link"]

    def get_world_poses(self):
        half = math.sqrt(0.5)
        return Array([[1.0, 2.0, 0.635]]), Array([[half, 0.0, 0.0, half]])

    def get_velocities(self):
        return Array([[0.0, 0.2, 0.0]]), Array([[0.0, 0.0, 0.1]])

    def get_dof_positions(self):
        return Array([[0.01 * index for index in range(len(self.dof_names))]])

    def get_dof_velocities(self):
        return Array([[0.001 * index for index in range(len(self.dof_names))]])

    def get_link_masses(self):
        return Array([[8.0, 1.0, 1.0]])

    def get_link_coms(self):
        return Array([[[0.0, 0.0, 0.0]] * 3]), Array([[[1.0, 0.0, 0.0, 0.0]] * 3])


class FakeLinks:
    def get_world_poses(self):
        return (
            Array([[1.0, 2.0, 0.60], [1.0, 2.10, 0.034], [1.0, 1.90, 0.034]]),
            Array([[1.0, 0.0, 0.0, 0.0]] * 3),
        )


@dataclass
class Reading:
    is_valid: bool = True
    in_contact: bool = True
    value: float = 150.0
    time: float = 1.0


class FakeSensor:
    def __init__(self, reading: Reading, points):
        self.reading = reading
        self.points = points

    def get_sensor_reading(self):
        return self.reading

    def get_raw_data(self):
        return [
            {"position": {"x": point[0], "y": point[1], "z": point[2]}}
            for point in self.points
        ]


def startup_diagnostics(
    *,
    root_pitch: float = 0.0,
    linear_speed: float = 0.0,
    root_z: float = 0.635,
    ankle_roll: float = 0.0,
) -> dict[str, object]:
    half_roll = 0.5 * ankle_roll
    ankle = {
        "link_position_world_m": [0.0, 0.0, 0.034],
        "link_orientation_world_wxyz": [
            math.cos(half_roll),
            math.sin(half_roll),
            0.0,
            0.0,
        ],
        "link_roll_pitch_yaw_rad": [ankle_roll, 0.0, 0.0],
        "sole_reference_world_m": [0.0385, 0.0, 0.0],
        "sphere_centers_world_m": [
            [center[0], center[1], 0.005]
            for center in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m
        ],
        "sphere_world_lowest_points_m": [
            [center[0], center[1], 0.0]
            for center in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m
        ],
    }
    return {
        "kinematics": {
            "sample_time_s": 1.0,
            "root": {
                "position_world_m": [0.0, 0.0, root_z],
                "orientation_world_wxyz": [1.0, 0.0, 0.0, 0.0],
                "roll_pitch_yaw_rad": [0.0, root_pitch, 0.0],
                "linear_velocity_world_mps": [linear_speed, 0.0, 0.0],
                "angular_velocity_world_rps": [0.0, 0.0, 0.0],
                "linear_velocity_body_mps": [linear_speed, 0.0, 0.0],
                "angular_velocity_body_rps": [0.0, 0.0, 0.0],
            },
            "leg_joint_position_rad": {name: 0.0 for name in LEG_JOINTS},
            "leg_joint_velocity_rad_s": {name: 0.0 for name in LEG_JOINTS},
            "ankles": {"left": copy.deepcopy(ankle), "right": copy.deepcopy(ankle)},
        },
        "contacts": {
            "left": {
                "is_valid": True,
                "in_contact": True,
                "force_n": 100.0,
                "age_s": 0.0,
            },
            "right": {
                "is_valid": True,
                "in_contact": True,
                "force_n": 100.0,
                "age_s": 0.0,
            },
        },
        "support": {"source": "measured_raw_contact_hull"},
    }


def startup_feedback(
    *,
    timestamp: float = 1.0,
    joints: dict[str, float] | None = None,
    left_contact: bool = True,
    right_contact: bool = True,
    root_pitch: float = 0.0,
    support_margin: float = 0.03,
) -> LocomotionFeedback:
    positions = joints if joints is not None else {name: 0.0 for name in LEG_JOINTS}
    return LocomotionFeedback(
        timestamp_s=timestamp,
        root_pose=PlanarPose(0.0, 0.0, 0.0),
        root_height_m=0.635,
        root_tilt_roll_pitch_rad=(0.0, root_pitch),
        root_linear_velocity_body_mps=(0.0, 0.0, 0.0),
        root_angular_velocity_body_rps=(0.0, 0.0, 0.0),
        left_foot=FootFeedback(FootPose((0.0, 0.0675, 0.0), 0.0), left_contact),
        right_foot=FootFeedback(FootPose((0.0, -0.0675, 0.0), 0.0), right_contact),
        joint_position_rad=positions,
        joint_velocity_rad_s={name: 0.0 for name in LEG_JOINTS},
        com_position_world_m=(0.0, 0.0, 0.50),
        support_center_world_m=(0.0, 0.0, 0.0),
        support_margin_m=support_margin,
    )


def test_balanced_crouch(_feedback: LocomotionFeedback) -> dict[str, float]:
    return {name: 0.18 for name in LEG_JOINTS}


class StartupController:
    def __init__(self):
        self.commands: list[dict[str, float]] = []
        self.command_read_counts: list[int] = []
        self.read_count_source = lambda: 0

    def command_joint_positions(self, targets):
        self.commands.append(dict(targets))
        self.command_read_counts.append(self.read_count_source())


class StartupAdapter:
    def __init__(self):
        self.current = startup_diagnostics()

    def diagnostics(self):
        return copy.deepcopy(self.current)


class StartupFeedbackSource:
    def __init__(self, controller: StartupController):
        self.controller = controller
        self.controller.read_count_source = lambda: self.read_count
        self.adapter = StartupAdapter()
        self.read_count = 0
        self.mode = "success"

    def read_feedback(self):
        self.read_count += 1
        if self.mode == "unavailable":
            raise FeedbackUnavailableError("measured support is not yet available")
        if self.mode == "unavailable_sole_tilt":
            self.adapter.current = startup_diagnostics(ankle_roll=math.radians(30.0))
            raise FeedbackUnavailableError("measured support is not yet available")
        if self.mode == "unavailable_clearance":
            self.adapter.current = startup_diagnostics(root_z=0.20)
            raise FeedbackUnavailableError("measured support is not yet available")
        if self.mode == "nonbilateral_sole_tilt":
            self.adapter.current = startup_diagnostics(ankle_roll=math.radians(30.0))
            return startup_feedback(right_contact=False)
        if self.mode == "unsafe_root":
            self.adapter.current = startup_diagnostics(root_pitch=math.radians(13.0))
            return startup_feedback(root_pitch=math.radians(13.0))
        if self.mode == "transient_contact_loss" and self.read_count == 2:
            return startup_feedback(right_contact=False)
        if self.mode == "contact_loss" and self.controller.commands:
            return startup_feedback(right_contact=False)
        if self.mode == "hold_tracking_failure" and not self.controller.commands:
            self.adapter.current = startup_diagnostics()
            self.adapter.current["kinematics"]["leg_joint_position_rad"] = {
                name: -1.0 for name in LEG_JOINTS
            }
            return startup_feedback(joints={name: -1.0 for name in LEG_JOINTS})
        if self.mode == "tracking_failure" and self.controller.commands:
            self.adapter.current = startup_diagnostics()
            self.adapter.current["kinematics"]["leg_joint_position_rad"] = {
                name: -1.0 for name in LEG_JOINTS
            }
            return startup_feedback(joints={name: -1.0 for name in LEG_JOINTS})
        if self.mode == "dwell_margin_failure" and len(self.controller.commands) >= 4:
            return startup_feedback(
                timestamp=1.0 + 0.1 * self.read_count,
                joints=self.controller.commands[-1],
                support_margin=-0.016,
            )
        if self.mode == "recorded_margin_progression" and self.controller.commands:
            command_count = len(self.controller.commands)
            if command_count <= 36:
                margin = 0.030 - 0.020 * (command_count - 1) / 35.0
            else:
                dwell_step = command_count - 36
                margin = 0.010 - 0.026013906163802466 * dwell_step / 22.0
            return startup_feedback(
                timestamp=1.0 + 0.1 * self.read_count,
                joints=self.controller.commands[-1],
                support_margin=margin,
            )
        joints = (
            self.controller.commands[-1]
            if self.controller.commands
            else {name: 0.02 for name in LEG_JOINTS}
        )
        return startup_feedback(timestamp=1.0 + 0.1 * self.read_count, joints=joints)


class IsaacFeedbackTests(unittest.TestCase):
    def adapter(self, left=Reading(), right=Reading()):
        left_points = [
            (0.955, 2.080, 0.0),
            (0.955, 2.120, 0.0),
            (1.122, 2.072, 0.0),
            (1.122, 2.128, 0.0),
        ] if left.in_contact else []
        right_points = [
            (0.955, 1.880, 0.0),
            (0.955, 1.920, 0.0),
            (1.122, 1.872, 0.0),
            (1.122, 1.928, 0.0),
        ] if right.in_contact else []
        return Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            FakeLinks(),
            FakeSensor(left, left_points),
            FakeSensor(right, right_points),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

    def test_adapter_normalizes_measured_frames_and_com(self):
        adapter = self.adapter()
        feedback = adapter.read_feedback()

        self.assertAlmostEqual(feedback.root_pose.x_m, 1.0)
        self.assertAlmostEqual(feedback.root_pose.y_m, 2.0)
        self.assertAlmostEqual(feedback.root_pose.yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(feedback.root_linear_velocity_body_mps[0], 0.2)
        self.assertAlmostEqual(feedback.root_linear_velocity_body_mps[1], 0.0, places=7)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[0], 1.0385)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[1], 2.10)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[2], 0.0)
        self.assertEqual(feedback.joint_position_rad[LEG_JOINTS[-1]], 0.11)
        self.assertEqual(feedback.joint_velocity_rad_s[LEG_JOINTS[-1]], 0.011)
        self.assertAlmostEqual(feedback.com_position_world_m[0], 1.0)
        self.assertAlmostEqual(feedback.com_position_world_m[1], 2.0)
        self.assertAlmostEqual(feedback.com_position_world_m[2], 0.4868)
        self.assertGreater(feedback.support_margin_m, 0.0)
        self.assertEqual(adapter.last_contact_forces_n, {"left": 150.0, "right": 150.0})
        self.assertEqual(adapter.diagnostics()["contacts"]["left"]["age_s"], 0.0)

    def test_single_contact_uses_only_that_sole_polygon(self):
        feedback = self.adapter(right=Reading(in_contact=False, value=0.0)).read_feedback()
        self.assertTrue(feedback.left_foot.in_contact)
        self.assertFalse(feedback.right_foot.in_contact)
        self.assertAlmostEqual(feedback.support_center_world_m[1], 2.10, places=7)
        self.assertLess(feedback.support_margin_m, 0.0)

    def test_reduced_raw_contacts_use_only_gated_inset_urdf_support(self):
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            FakeLinks(),
            FakeSensor(Reading(), [(0.955, 2.080, 0.0)]),
            FakeSensor(Reading(), [(1.122, 1.928, 0.0)]),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

        feedback = adapter.read_feedback()
        diagnostics = adapter.diagnostics()

        self.assertTrue(feedback.left_foot.in_contact)
        self.assertTrue(feedback.right_foot.in_contact)
        self.assertEqual(
            diagnostics["support"]["source"], "contact_conditioned_urdf_inset"
        )
        self.assertEqual(diagnostics["support"]["raw_point_count"], 2)
        self.assertEqual(
            diagnostics["contacts"]["left"]["raw_points_world_m"],
            [[0.955, 2.08, 0.0]],
        )
        inferred = diagnostics["support"]["support_points_world_m"]
        self.assertEqual(len(inferred), 8)
        inferred_left_width = max(point[0] for point in inferred[:4]) - min(
            point[0] for point in inferred[:4]
        )
        authored_width = (
            max(point[0] for point in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m)
            - min(point[0] for point in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m)
        )
        self.assertAlmostEqual(inferred_left_width, 0.5 * authored_width)

    def test_reduced_contacts_fail_closed_with_complete_geometry_diagnostics(self):
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            FakeLinks(),
            FakeSensor(Reading(), [(9.0, 9.0, 0.0)]),
            FakeSensor(Reading(in_contact=False, value=0.0), []),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

        with self.assertRaisesRegex(
            FeedbackUnavailableError, "raw contact is outside its sole spheres"
        ):
            adapter.read_feedback()
        diagnostics = adapter.diagnostics()
        self.assertEqual(
            diagnostics["contacts"]["left"]["raw_points_world_m"],
            [[9.0, 9.0, 0.0]],
        )
        self.assertIn("three distinct finite points", diagnostics["support"]["raw_hull_error"])
        gate = diagnostics["support"]["contact_conditioned_feet"]["left"]
        self.assertGreater(gate["maximum_raw_point_to_sphere_xy_distance_m"], 1.0)
        self.assertEqual(gate["error"], "raw point does not match an authored contact sphere")

    def test_reduced_contacts_do_not_infer_support_from_a_non_coplanar_sole(self):
        links = FakeLinks()
        links.get_world_poses = lambda: (
            Array([[1.0, 2.0, 0.60], [1.0, 2.10, 0.036], [1.0, 1.90, 0.034]]),
            Array([[1.0, 0.0, 0.0, 0.0]] * 3),
        )
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            links,
            FakeSensor(Reading(), [(0.955, 2.080, 0.0)]),
            FakeSensor(Reading(in_contact=False, value=0.0), []),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

        with self.assertRaisesRegex(
            FeedbackUnavailableError,
            "no authored collision sphere reaches the measured contact plane",
        ):
            adapter.read_feedback()
        gate = adapter.diagnostics()["support"]["contact_conditioned_feet"]["left"]
        self.assertEqual(gate["eligible_sphere_indices"], [])
        self.assertEqual(gate["inferred_support_points_world_m"], [])

    def test_tipped_sole_filters_high_spheres_and_stays_fail_closed(self):
        half = math.sqrt(0.5)
        links = FakeLinks()
        links.get_world_poses = lambda: (
            Array([[1.0, 2.0, 0.60], [0.0, 0.0, 0.05], [1.0, 1.90, 0.034]]),
            Array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [half, 0.0, -half, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                ]
            ),
        )
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            links,
            FakeSensor(Reading(), [(0.029, -0.020, 0.0), (0.029, 0.020, 0.0)]),
            FakeSensor(Reading(in_contact=False, value=0.0), []),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

        with self.assertRaisesRegex(
            FeedbackUnavailableError, "support polygon requires three distinct finite points"
        ):
            adapter.read_feedback()
        support = adapter.diagnostics()["support"]
        gate = support["contact_conditioned_feet"]["left"]
        self.assertEqual(gate["eligible_sphere_indices"], [0, 1])
        self.assertEqual(gate["inference_mode"], "explicit_points_insufficient_alone")
        self.assertEqual(len(gate["inferred_support_points_world_m"]), 2)
        self.assertEqual(len(support["support_points_world_m"]), 2)
        self.assertAlmostEqual(gate["sphere_centers_world_m"][0][0], 0.029)
        self.assertAlmostEqual(gate["sphere_world_lowest_points_m"][0][2], 0.0)
        self.assertAlmostEqual(gate["sphere_world_lowest_points_m"][2][2], 0.167)
        self.assertEqual(
            [match["sphere_index"] for match in gate["raw_point_sphere_matches"]],
            [0, 1],
        )

    def test_missing_stale_or_absent_contact_fails_closed(self):
        with self.assertRaisesRegex(FeedbackUnavailableError, "left contact sensor reading is invalid"):
            self.adapter(left=Reading(is_valid=False)).read_feedback()
        with self.assertRaisesRegex(FeedbackUnavailableError, "stale"):
            self.adapter(left=Reading(time=0.5)).read_feedback()
        with self.assertRaisesRegex(FeedbackUnavailableError, "neither foot"):
            self.adapter(
                left=Reading(in_contact=False, value=0.0),
                right=Reading(in_contact=False, value=0.0),
            ).read_feedback()

    def test_shape_and_mass_failures_do_not_create_plausible_feedback(self):
        articulation = FakeArticulation()
        articulation.get_link_masses = lambda: Array([[8.0, -1.0, 1.0]])
        adapter = Isaac61LocomotionFeedbackAdapter(
            articulation,
            FakeLinks(),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            lambda: 1.0,
        )
        with self.assertRaisesRegex(FeedbackUnavailableError, "finite and nonnegative"):
            adapter.read_feedback()

        links = FakeLinks()
        links.get_world_poses = lambda: (Array([[0.0, 0.0, 0.0]]), Array([[1.0, 0.0, 0.0, 0.0]]))
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            links,
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            lambda: 1.0,
        )
        with self.assertRaisesRegex(FeedbackUnavailableError, "link positions"):
            adapter.read_feedback()

    def test_support_hull_margin_and_centroid_are_deterministic(self):
        points = [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]
        self.assertEqual(convex_hull_xy(points), ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)))
        center, inside = support_polygon_center_and_margin(points, (0.0, 0.0))
        self.assertEqual(center, (0.0, 0.0))
        self.assertAlmostEqual(inside, 1.0)
        _center, outside = support_polygon_center_and_margin(points, (1.25, 0.0))
        self.assertAlmostEqual(outside, -0.25)

    def test_future_smoke_harness_has_no_direct_state_setters(self):
        path = ROOT / "robot_spike" / "production" / "run_locomotion_smoke.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        called_attributes = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        top_level_imports = [
            node
            for node in tree.body
            if isinstance(node, (ast.Import, ast.ImportFrom))
        ]
        self.assertFalse(
            any(
                (isinstance(node, ast.ImportFrom) and (node.module or "").startswith("isaacsim"))
                or (
                    isinstance(node, ast.Import)
                    and any(alias.name.startswith("isaacsim") for alias in node.names)
                )
                for node in top_level_imports
            )
        )
        self.assertTrue({"reset", "command_joint_positions"} <= called_attributes)
        self.assertFalse(
            {
                "set_world_poses",
                "set_velocities",
                "set_dof_positions",
                "set_dof_velocities",
            }
            & called_attributes
        )
        simulation_app_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "SimulationApp"
        ]
        self.assertEqual(len(simulation_app_calls), 1)
        launch_config = simulation_app_calls[0].args[0]
        self.assertIsInstance(launch_config, ast.Dict)
        config_values = {
            key.value: value.value
            for key, value in zip(launch_config.keys, launch_config.values)
            if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)
        }
        self.assertIn("fast_shutdown", config_values)
        self.assertIs(config_values["fast_shutdown"], False)
        startup_calls = [
            node.lineno
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_staged_double_support_startup"
        ]
        gait_constructors = [
            node.lineno
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "BipedLocomotionController"
        ]
        gait_commands = [
            node.lineno
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "start_route"
        ]
        self.assertEqual(len(startup_calls), 1)
        self.assertEqual(len(gait_constructors), 1)
        self.assertEqual(len(gait_commands), 1)
        self.assertLess(startup_calls[0], gait_constructors[0])
        self.assertLess(gait_constructors[0], gait_commands[0])

    def test_smoke_arguments_fail_before_isaac_for_invalid_values(self):
        valid = {
            "physics_hz": 120,
            "repeats": 2,
            "forward_m": 0.06,
            "settle_s": 2.0,
            "timeout_s": 12.0,
        }
        _validate_arguments(Namespace(**valid))
        for name, value in (
            ("physics_hz", 0),
            ("repeats", 0),
            ("forward_m", 0.0),
            ("settle_s", math.inf),
            ("settle_s", 0.8),
            ("timeout_s", math.nan),
        ):
            invalid = dict(valid)
            invalid[name] = value
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    _validate_arguments(Namespace(**invalid))

    def test_nonintegral_physics_rate_uses_discrete_startup_budget_preflight(self):
        invalid = Namespace(
            physics_hz=14,
            repeats=1,
            forward_m=0.06,
            settle_s=3.0 * 0.30 + 1.0 / 14.0,
            timeout_s=12.0,
        )
        with self.assertRaisesRegex(ValueError, "16 physics steps"):
            _validate_arguments(invalid)
        with mock.patch.object(smoke_module, "_execute_smoke") as runtime:
            with self.assertRaisesRegex(ValueError, "16 physics steps"):
                smoke_module.main(
                    [
                        "--output",
                        "unused",
                        "--expected-candidate-sha",
                        "candidate",
                        "--expected-candidate-tree-sha",
                        "tree",
                        "--physics-hz",
                        "14",
                        "--settle-s",
                        str(invalid.settle_s),
                    ]
                )
            runtime.assert_not_called()

        valid_boundary_s = 16.0 / 14.0
        valid = Namespace(**{**vars(invalid), "settle_s": valid_boundary_s})
        _validate_arguments(valid)
        budget = _startup_step_budget(1.0 / 14.0, valid_boundary_s)
        self.assertEqual(budget["total"], 16)
        self.assertEqual(budget["required_consecutive_stable_samples"], 5)
        self.assertEqual(budget["target_ramp"], 5)
        self.assertEqual(budget["verified_dwell"], 5)
        self.assertEqual(budget["pre_ramp_stabilization"], 6)

    def test_acceptance_spec_bytes_and_consumed_metadata_are_in_identity(self):
        self.assertIn(
            "robot_spike/production/production_manifest.json", EVIDENCE_FILES
        )
        self.assertIn("robot_spike/production/robot_config.json", EVIDENCE_FILES)
        self.assertIn(
            "robot_spike/production/asimov_orcahand_restocking.urdf",
            EVIDENCE_FILES,
        )
        self.assertIn("robot_spike/asimov_orcahand_right.urdf", EVIDENCE_FILES)
        self.assertIn("python.bat", ISAAC_API_SOURCE_FILES)

        class MemoryPath:
            def __init__(self, data: bytes):
                self.data = data
                self.exists = True

            def resolve(self):
                return self

            def is_file(self):
                return self.exists

            def open(self, mode):
                self.assert_binary_mode = mode
                return io.BytesIO(self.data)

            def __str__(self):
                return "memory://acceptance.md"

        original = b"owner acceptance bytes\r\n"
        expected = hashlib.sha256(original).hexdigest()
        path = MemoryPath(original)
        identity = acceptance_spec_identity(path, expected)  # type: ignore[arg-type]
        self.assertEqual(identity["sha256"], expected)
        self.assertEqual(identity["path"], str(path))
        self.assertEqual(path.assert_binary_mode, "rb")

        path.data = b"changed bytes\n"
        with self.assertRaisesRegex(RuntimeError, "SHA-256 mismatch"):
            acceptance_spec_identity(path, expected)  # type: ignore[arg-type]
        path.exists = False
        with self.assertRaisesRegex(RuntimeError, "is missing"):
            acceptance_spec_identity(path, expected)  # type: ignore[arg-type]

    def test_repeat_and_final_identity_checks_fail_closed_and_persist(self):
        initial = {
            "acceptance_spec": {"sha256": "spec"},
            "source": {
                "source_files_sha256": {
                    "robot_spike/production/production_manifest.json": "manifest",
                    "robot_spike/production/robot_config.json": "config",
                }
            },
            "installed_isaac": {
                "api_source_files_sha256": {"articulation.py": "api"}
            },
        }
        state = {"identity": copy.deepcopy(initial)}
        guard = EvidenceIdentityGuard(
            initial, lambda: copy.deepcopy(state["identity"])
        )
        result = {"status": "running", "identity_checks": guard.checks, "repeats": []}
        repeat = {"repeat_index": 0, "identity_checks": []}
        result["repeats"].append(repeat)

        class CapturingStatusPath:
            text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

        status_path = CapturingStatusPath()
        _record_identity_boundary(
            guard, "repeat_0_start", result, status_path, repeat  # type: ignore[arg-type]
        )
        del state["identity"]["source"]["source_files_sha256"][
            "robot_spike/production/production_manifest.json"
        ]
        with self.assertRaises(EvidenceIdentityMismatchError):
            _record_identity_boundary(
                guard,
                "repeat_0_end",
                result,
                status_path,  # type: ignore[arg-type]
                repeat,
            )
        persisted = json.loads(status_path.text)
        self.assertEqual(
            [check["status"] for check in persisted["repeats"][0]["identity_checks"]],
            ["match", "mismatch"],
        )
        self.assertIn(
            "production_manifest.json",
            persisted["repeats"][0]["identity_checks"][1]["mismatches"][0],
        )

        final_state = copy.deepcopy(initial)
        final_guard = EvidenceIdentityGuard(
            initial, lambda: copy.deepcopy(final_state)
        )
        final_result = {
            "status": "running",
            "identity_checks": final_guard.checks,
            "repeats": [],
        }
        final_state["installed_isaac"]["api_source_files_sha256"][
            "articulation.py"
        ] = "changed-api"
        with self.assertRaises(EvidenceIdentityMismatchError):
            _record_identity_boundary(
                final_guard,
                "final_success",
                final_result,
                status_path,  # type: ignore[arg-type]
            )
        persisted = json.loads(status_path.text)
        self.assertEqual(persisted["identity_checks"][0]["phase"], "final_success")
        self.assertEqual(persisted["identity_checks"][0]["status"], "mismatch")

    def test_preflight_identity_failures_are_durable_and_do_not_launch_runtime(self):
        class MemoryStatusFile:
            def __init__(self):
                self.text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

            def is_file(self):
                return bool(self.text)

            def read_text(self, **_kwargs):
                return self.text

        class MemoryOutput:
            def __init__(self):
                self.created = False
                self.status = MemoryStatusFile()

            def resolve(self):
                return self

            def exists(self):
                return False

            def mkdir(self, **_kwargs):
                self.created = True

            def __truediv__(self, name):
                self.assert_status_name = name
                return self.status

            def __str__(self):
                return "memory://smoke-output"

        for error in (
            "acceptance specification is missing: owner.md",
            "source identity changed: production_manifest.json",
        ):
            with self.subTest(error=error):
                output = MemoryOutput()
                launched = {"value": False}

                def identity_reader(*_args, **_kwargs):
                    raise RuntimeError(error)

                def runtime_runner(*_args, **_kwargs):
                    launched["value"] = True
                    raise AssertionError("runtime must not launch after preflight failure")

                args = Namespace(
                    output=output,
                    expected_candidate_sha="candidate",
                    expected_candidate_tree_sha="tree",
                    acceptance_spec=Path("owner.md"),
                )
                with contextlib.redirect_stderr(io.StringIO()):
                    exit_code = _execute_smoke(
                        args,
                        ROOT,
                        identity_reader=identity_reader,
                        runtime_runner=runtime_runner,
                    )
                self.assertEqual(exit_code, 2)
                self.assertTrue(output.created)
                self.assertEqual(
                    output.assert_status_name, "locomotion_smoke_status.json"
                )
                report = json.loads(output.status.text)
                self.assertEqual(report["phase"], "preflight_identity")
                self.assertFalse(report["runtime_invoked"])
                self.assertEqual(report["error"], error)
                self.assertFalse(launched["value"])

    def test_runtime_success_and_exception_are_reported_after_runner_returns(self):
        class MemoryStatusFile:
            def __init__(self):
                self.text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

            def is_file(self):
                return bool(self.text)

            def read_text(self, **_kwargs):
                return self.text

        class MemoryOutput:
            def __init__(self):
                self.status = MemoryStatusFile()

            def resolve(self):
                return self

            def exists(self):
                return False

            def mkdir(self, **_kwargs):
                pass

            def __truediv__(self, _name):
                return self.status

            def __str__(self):
                return "memory://runtime-output"

        initial_identity = {"source": {"candidate_sha": "candidate"}}

        def identity_reader(*_args, **_kwargs):
            return copy.deepcopy(initial_identity)

        def make_args(output):
            return Namespace(
                output=output,
                expected_candidate_sha="candidate",
                expected_candidate_tree_sha="tree",
                acceptance_spec=Path("owner.md"),
            )

        success_output = MemoryOutput()
        returned = {"schema_version": 1, "status": "pass", "shutdown_returned": True}

        def successful_runner(*_args, **_kwargs):
            return copy.deepcopy(returned)

        with contextlib.redirect_stdout(io.StringIO()):
            success_code = _execute_smoke(
                make_args(success_output),
                ROOT,
                identity_reader=identity_reader,
                runtime_runner=successful_runner,
            )
        self.assertEqual(success_code, 0)
        self.assertEqual(json.loads(success_output.status.text), returned)

        failed_output = MemoryOutput()

        def failed_runner(*_args, **_kwargs):
            return {"schema_version": 1, "status": "fail", "shutdown_returned": True}

        with contextlib.redirect_stdout(io.StringIO()):
            failed_code = _execute_smoke(
                make_args(failed_output),
                ROOT,
                identity_reader=identity_reader,
                runtime_runner=failed_runner,
            )
        self.assertEqual(failed_code, 1)
        self.assertEqual(json.loads(failed_output.status.text)["status"], "fail")

        error_output = MemoryOutput()

        def failing_runner(*_args, **_kwargs):
            raise RuntimeError("shutdown returned runtime failure")

        with contextlib.redirect_stderr(io.StringIO()):
            error_code = _execute_smoke(
                make_args(error_output),
                ROOT,
                identity_reader=identity_reader,
                runtime_runner=failing_runner,
            )
        self.assertEqual(error_code, 2)
        failure = json.loads(error_output.status.text)
        self.assertEqual(failure["status"], "error")
        self.assertEqual(failure["phase"], "runtime")
        self.assertTrue(failure["runtime_invoked"])
        self.assertEqual(failure["error"], "shutdown returned runtime failure")

    def test_feedback_failure_diagnostics_are_persisted_before_unwind(self):
        class FakeAdapter:
            def diagnostics(self):
                return {
                    "contacts": {
                        "left": {
                            "is_valid": True,
                            "in_contact": True,
                            "force_n": 12.0,
                            "reading_time_s": 1.0,
                            "raw_point_count": 1,
                            "raw_points_world_m": [[0.1, 0.2, 0.0]],
                        }
                    },
                    "support": {
                        "raw_point_count": 1,
                        "support_points_world_m": [],
                    },
                }

        class CapturingStatusPath:
            text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

        repeat = {"repeat_index": 0, "status": "running"}
        result = {"status": "running", "repeats": [repeat]}
        status = CapturingStatusPath()
        error = FeedbackUnavailableError("support unavailable")

        _persist_feedback_failure(
            adapter=FakeAdapter(),  # type: ignore[arg-type]
            repeat_result=repeat,
            result=result,
            status_path=status,  # type: ignore[arg-type]
            phase="initial_post_settle_feedback",
            error=error,
        )

        persisted = json.loads(status.text)
        failure = persisted["repeats"][0]["feedback_failure"]
        self.assertEqual(failure["phase"], "initial_post_settle_feedback")
        self.assertEqual(failure["error"], "support unavailable")
        self.assertEqual(
            failure["diagnostics"]["contacts"]["left"]["raw_point_count"], 1
        )
        self.assertEqual(
            failure["diagnostics"]["contacts"]["left"]["raw_points_world_m"],
            [[0.1, 0.2, 0.0]],
        )

    def test_terminal_exit_sets_child_status_after_reporting(self):
        observed = []

        for exit_code in (0, 1, 2):
            _terminate_process(exit_code, hard_exit=observed.append)

        self.assertEqual(observed, [0, 1, 2])

    def test_runtime_error_is_terminal_before_shutdown(self):
        class CapturingStatusPath:
            text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

        status = CapturingStatusPath()
        partial = {"schema_version": 1, "status": "running", "repeats": []}
        try:
            raise FeedbackUnavailableError("support polygon unavailable")
        except FeedbackUnavailableError as error:
            report = _persist_pre_shutdown_runtime_error(
                initial_identity={"source": {"candidate_sha": "candidate"}},
                status_path=status,  # type: ignore[arg-type]
                error=error,
                partial_result=partial,
            )

        persisted = json.loads(status.text)
        self.assertIs(report, partial)
        self.assertEqual(persisted["status"], "error")
        self.assertEqual(persisted["phase"], "runtime")
        self.assertFalse(persisted["shutdown_returned"])
        self.assertEqual(persisted["error_type"], "FeedbackUnavailableError")
        self.assertEqual(persisted["error"], "support polygon unavailable")
        self.assertIn("FeedbackUnavailableError", persisted["traceback"])

    def test_feedback_failure_retains_root_joint_and_both_ankle_kinematics(self):
        adapter = self.adapter(
            left=Reading(in_contact=True),
            right=Reading(in_contact=False, value=0.0),
        )
        adapter.left_contact_sensor = FakeSensor(
            Reading(in_contact=True), [(9.0, 9.0, 0.0)]
        )
        with self.assertRaisesRegex(FeedbackUnavailableError, "outside its sole spheres"):
            adapter.read_feedback()

        kinematics = adapter.diagnostics()["kinematics"]
        self.assertEqual(kinematics["root"]["position_world_m"], [1.0, 2.0, 0.635])
        self.assertEqual(len(kinematics["root"]["orientation_world_wxyz"]), 4)
        self.assertEqual(set(kinematics["leg_joint_position_rad"]), set(LEG_JOINTS))
        self.assertEqual(set(kinematics["leg_joint_velocity_rad_s"]), set(LEG_JOINTS))
        self.assertEqual(set(kinematics["ankles"]), {"left", "right"})
        for side in ("left", "right"):
            ankle = kinematics["ankles"][side]
            self.assertEqual(len(ankle["sphere_centers_world_m"]), 4)
            self.assertEqual(len(ankle["sphere_world_lowest_points_m"]), 4)
            self.assertEqual(len(ankle["link_roll_pitch_yaw_rad"]), 3)

    def test_staged_startup_interpolates_targets_and_observes_timing(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        crouch = {name: 0.18 for name in LEG_JOINTS}
        physics_steps = []
        progress = []

        final, report = _staged_double_support_startup(
            feedback_source=source,  # type: ignore[arg-type]
            joint_controller=controller,  # type: ignore[arg-type]
            step_physics=lambda: physics_steps.append(len(physics_steps) + 1),
            reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
            crouch_targets=crouch,
            balanced_crouch_targets=lambda _feedback: dict(crouch),
            physics_dt=0.1,
            maximum_duration_s=2.0,
            balance_observation=lambda: {
                "bounded_correction_rad": {"sagittal": 0.045, "lateral": 0.0}
            },
            progress=lambda value: progress.append(copy.deepcopy(value)),
        )

        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["contact_acquired_step"], 1)
        self.assertEqual(report["stability_achieved_step"], 3)
        self.assertEqual(
            report["step_budget"]["required_consecutive_stable_samples"], 3
        )
        self.assertEqual(report["step_budget"]["target_ramp"], 3)
        self.assertEqual(report["step_budget"]["verified_dwell"], 3)
        self.assertEqual(len(physics_steps), 9)
        self.assertEqual(len(controller.commands), 6)
        self.assertEqual(controller.command_read_counts[0], 3)
        self.assertEqual(
            report["pre_ramp_hold"]["post_reset_commands_before_stability"], 0
        )
        self.assertAlmostEqual(controller.commands[0][LEG_JOINTS[0]], 0.02 + (0.16 / 3.0))
        self.assertEqual(controller.commands[-1], crouch)
        self.assertEqual(final.joint_position_rad, crouch)
        commanded_events = [
            event
            for event in report["events"]
            if event["stage"] in {"target_ramp", "verified_dwell"}
        ]
        self.assertTrue(
            all(
                event["balance_observation"]["bounded_correction_rad"]
                == {"sagittal": 0.045, "lateral": 0.0}
                for event in commanded_events
            )
        )
        self.assertEqual(progress[-1]["status"], "pass")
        self.assertEqual(
            _interpolate_joint_targets({"joint": 1.0}, {"joint": 3.0}, 0.5),
            {"joint": 2.0},
        )

    def test_verified_dwell_margin_failure_aborts_before_gait(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "dwell_margin_failure"

        with self.assertRaisesRegex(
            StartupValidationError, "COM projection left the configured support margin"
        ) as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
            )

        self.assertEqual(raised.exception.phase, "verified_dwell")
        self.assertEqual(len(controller.commands), 4)
        event = raised.exception.report["events"][-1]
        self.assertEqual(event["stage"], "verified_dwell")
        self.assertEqual(event["status"], "rejected")
        self.assertAlmostEqual(event["metrics"]["support_margin_m"], -0.016)
        self.assertIn("commanded_joint_targets_rad", event)

    def test_recorded_ramp_and_dwell_margin_progression_fails_closed(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "recorded_margin_progression"

        with self.assertRaisesRegex(
            StartupValidationError, "COM projection left the configured support margin"
        ) as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=1.0 / 120.0,
                maximum_duration_s=2.0,
            )

        self.assertEqual(raised.exception.phase, "verified_dwell")
        ramp = [
            event
            for event in raised.exception.report["events"]
            if event["stage"] == "target_ramp"
        ]
        dwell = [
            event
            for event in raised.exception.report["events"]
            if event["stage"] == "verified_dwell"
        ]
        self.assertEqual(len(ramp), 36)
        self.assertEqual(len(dwell), 22)
        self.assertTrue(all(event["status"] == "accepted" for event in ramp))
        self.assertTrue(all(event["status"] == "accepted" for event in dwell[:-1]))
        self.assertEqual(dwell[-1]["status"], "rejected")
        self.assertAlmostEqual(ramp[0]["metrics"]["support_margin_m"], 0.030)
        self.assertAlmostEqual(ramp[-1]["metrics"]["support_margin_m"], 0.010)
        self.assertAlmostEqual(
            dwell[-1]["metrics"]["support_margin_m"], -0.016013906163802466
        )
        self.assertEqual(len(controller.commands), 58)

    def test_sole_normal_tilt_rejects_thirty_degrees_and_honors_boundary(self):
        config = GaitConfig()
        feedback = startup_feedback()
        at_boundary, boundary_metrics = _startup_feedback_failures(
            feedback,
            startup_diagnostics(ankle_roll=config.maximum_root_tilt_rad),
            None,
            config,
        )
        over_boundary, _over_metrics = _startup_feedback_failures(
            feedback,
            startup_diagnostics(
                ankle_roll=config.maximum_root_tilt_rad + math.radians(0.01)
            ),
            None,
            config,
        )
        thirty_degrees, thirty_metrics = _startup_feedback_failures(
            feedback,
            startup_diagnostics(ankle_roll=math.radians(30.0)),
            None,
            config,
        )

        self.assertFalse(any("sole tilt" in failure for failure in at_boundary))
        self.assertAlmostEqual(
            boundary_metrics["left_sole_tilt_rad"], config.maximum_root_tilt_rad
        )
        self.assertTrue(any("sole tilt" in failure for failure in over_boundary))
        self.assertTrue(any("sole tilt" in failure for failure in thirty_degrees))
        self.assertAlmostEqual(
            thirty_metrics["left_sole_tilt_rad"], math.radians(30.0)
        )

    def test_staged_startup_aborts_on_contact_loss_with_diagnostics(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "contact_loss"
        with self.assertRaises(StartupValidationError) as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
            )

        error = raised.exception
        self.assertEqual(error.phase, "target_ramp")
        self.assertIn("both measured feet are not in contact", str(error))
        rejected = error.report["events"][-1]
        self.assertEqual(rejected["status"], "rejected")
        self.assertIn("kinematics", rejected["diagnostics"])
        self.assertEqual(len(controller.commands), 1)

    def test_staged_startup_bounds_acquisition_and_persists_each_failed_read(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "unavailable"
        progress = []
        with self.assertRaisesRegex(
            StartupValidationError, "bounded window ended"
        ) as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
                progress=lambda value: progress.append(copy.deepcopy(value)),
            )

        stabilization_steps = raised.exception.report["step_budget"][
            "pre_ramp_stabilization"
        ]
        self.assertEqual(source.read_count, stabilization_steps)
        self.assertEqual(len(raised.exception.report["events"]), stabilization_steps)
        self.assertGreaterEqual(len(progress), stabilization_steps)
        self.assertTrue(
            all("kinematics" in event["diagnostics"] for event in progress[-2]["events"])
        )
        self.assertEqual(controller.commands, [])

    def test_acquisition_without_bilateral_support_gates_sole_tilt_and_clearance(self):
        for mode, expected in (
            ("unavailable_sole_tilt", "sole tilt"),
            ("unavailable_clearance", "root clearance"),
            ("nonbilateral_sole_tilt", "sole tilt"),
        ):
            with self.subTest(mode=mode):
                controller = StartupController()
                source = StartupFeedbackSource(controller)
                source.mode = mode
                physics_steps = []
                with self.assertRaisesRegex(StartupValidationError, expected) as raised:
                    _staged_double_support_startup(
                        feedback_source=source,  # type: ignore[arg-type]
                        joint_controller=controller,  # type: ignore[arg-type]
                        step_physics=lambda: physics_steps.append(1),
                        reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                        crouch_targets={name: 0.18 for name in LEG_JOINTS},
                        balanced_crouch_targets=test_balanced_crouch,
                        physics_dt=0.1,
                        maximum_duration_s=2.0,
                    )

                self.assertEqual(physics_steps, [1])
                self.assertEqual(controller.commands, [])
                event = raised.exception.report["events"][-1]
                self.assertEqual(event["status"], "rejected")
                self.assertTrue(
                    any(expected in failure for failure in event["gate_failures"])
                )
                self.assertIn("kinematics", event["diagnostics"])

    def test_pre_ramp_dwell_resets_after_transient_contact_loss_then_reacquires(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "transient_contact_loss"
        final, report = _staged_double_support_startup(
            feedback_source=source,  # type: ignore[arg-type]
            joint_controller=controller,  # type: ignore[arg-type]
            step_physics=lambda: None,
            reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
            crouch_targets={name: 0.18 for name in LEG_JOINTS},
            balanced_crouch_targets=test_balanced_crouch,
            physics_dt=0.1,
            maximum_duration_s=2.0,
        )

        pre_ramp = [
            event
            for event in report["events"]
            if event["stage"] == "pre_ramp_stabilization"
        ]
        self.assertEqual([event["status"] for event in pre_ramp[:5]], [
            "stabilizing",
            "dwell_reset",
            "stabilizing",
            "stabilizing",
            "stable",
        ])
        self.assertIn(
            "both measured feet are not in contact",
            pre_ramp[1]["stability_failures"],
        )
        self.assertEqual(report["stability_achieved_step"], 5)
        self.assertEqual(controller.command_read_counts[0], 5)
        self.assertEqual(len(controller.commands), 6)
        self.assertEqual(final.joint_position_rad, controller.commands[-1])

    def test_pre_ramp_hold_target_limit_aborts_without_a_command(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "hold_tracking_failure"
        with self.assertRaisesRegex(StartupValidationError, "target tracking") as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
            )

        self.assertEqual(raised.exception.phase, "pre_ramp_stabilization")
        self.assertEqual(controller.commands, [])
        event = raised.exception.report["events"][-1]
        self.assertEqual(event["status"], "rejected")
        self.assertGreater(
            event["gate_metrics"]["maximum_joint_target_error_rad"], 0.3
        )
        self.assertIn(
            "joint target tracking exceeded configured limit",
            event["gate_failures"],
        )

    def test_staged_startup_aborts_on_unstable_root_before_any_target(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "unsafe_root"
        with self.assertRaisesRegex(StartupValidationError, "root tilt") as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
            )

        self.assertEqual(raised.exception.phase, "pre_ramp_stabilization")
        self.assertEqual(controller.commands, [])
        self.assertEqual(raised.exception.report["status"], "error")

    def test_staged_startup_aborts_on_target_tracking_error(self):
        controller = StartupController()
        source = StartupFeedbackSource(controller)
        source.mode = "tracking_failure"
        with self.assertRaisesRegex(StartupValidationError, "target tracking") as raised:
            _staged_double_support_startup(
                feedback_source=source,  # type: ignore[arg-type]
                joint_controller=controller,  # type: ignore[arg-type]
                step_physics=lambda: None,
                reset_hold_targets={name: 0.0 for name in LEG_JOINTS},
                crouch_targets={name: 0.18 for name in LEG_JOINTS},
                balanced_crouch_targets=test_balanced_crouch,
                physics_dt=0.1,
                maximum_duration_s=2.0,
            )

        self.assertEqual(raised.exception.phase, "target_ramp")
        event = raised.exception.report["events"][-1]
        self.assertGreater(event["metrics"]["maximum_joint_target_error_rad"], 0.3)
        self.assertIn("kinematics", event["diagnostics"])


if __name__ == "__main__":
    unittest.main()
