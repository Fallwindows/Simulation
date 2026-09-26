from __future__ import annotations

import math
from pathlib import Path
import unittest

import numpy as np

from robot_spike.production.arm_reach import (
    ARM_DOF_NAMES,
    EXPECTED_CHAIN_JOINTS,
    KINEMATIC_BASE_FRAME,
    TOOL_FRAME,
    ArmKinematicsError,
    ArmReachPlanner,
    IKConvergenceError,
    RightArmKinematics,
    ToolPose,
    command_reach_waypoint,
)
from robot_spike.production.model import load_production_spec
from robot_spike.production.runtime import ArticulationController


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOT = ROOT / "robot_spike" / "production"


class FakeArticulation:
    def __init__(self, dof_names):
        self.dof_names = tuple(dof_names)
        self.calls = []

    def set_dof_position_targets(self, *args, **kwargs):
        self.calls.append(("set_dof_position_targets", args, kwargs))


def positions(*values: float) -> dict[str, float]:
    return dict(zip(ARM_DOF_NAMES, values))


class RobotArmReachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_production_spec(PRODUCTION_ROOT)
        cls.kinematics = RightArmKinematics(cls.spec)

    def test_chain_frames_dofs_and_limits_match_approved_production_urdf(self):
        self.assertEqual(KINEMATIC_BASE_FRAME, "waist_yaw_link")
        self.assertEqual(TOOL_FRAME, "right_palm")
        self.assertEqual(
            EXPECTED_CHAIN_JOINTS,
            (
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_joint",
                "right_wrist_yaw_joint",
                "asimov_to_orcahand_mount",
                "right_wrist",
                "right_wrist_offset",
            ),
        )
        expected_limits = {
            "right_shoulder_pitch_joint": (-0.872665, 3.141593),
            "right_shoulder_roll_joint": (0.0, 1.570796),
            "right_shoulder_yaw_joint": (-1.570796, 1.570796),
            "right_elbow_joint": (-2.443461, 0.0),
            "right_wrist_yaw_joint": (-3.141593, 3.141593),
            "right_wrist": (-0.6736, 0.8972),
        }
        self.assertEqual(tuple(expected_limits), ARM_DOF_NAMES)
        self.assertEqual(
            {
                name: (
                    self.spec.model.joint_limits[name].lower,
                    self.spec.model.joint_limits[name].upper,
                )
                for name in ARM_DOF_NAMES
            },
            expected_limits,
        )

    def test_forward_kinematics_zero_pose_matches_fixed_urdf_chain_result(self):
        pose = self.kinematics.forward(positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(
            pose.position_m,
            (-0.037103064743, -0.160413223230, -0.283275009995),
            atol=1e-7,
            rtol=0.0,
        )
        np.testing.assert_allclose(
            pose.orientation_xyzw,
            (0.0, 0.999048221582, 0.0, 0.043619387365),
            atol=1e-7,
            rtol=0.0,
        )

    def test_invalid_nonfinite_pose_and_joint_inputs_fail_closed(self):
        with self.assertRaisesRegex(ArmKinematicsError, "finite"):
            ToolPose((0.0, math.nan, 0.0), (0.0, 0.0, 0.0, 1.0))
        with self.assertRaisesRegex(ArmKinematicsError, "unit norm"):
            ToolPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0))
        with self.assertRaisesRegex(ArmKinematicsError, "three|3"):
            ToolPose(0.0, (0.0, 0.0, 0.0, 1.0))
        with self.assertRaisesRegex(ArmKinematicsError, "mapping"):
            self.kinematics.forward(None)
        with self.assertRaisesRegex(ArmKinematicsError, "missing"):
            self.kinematics.forward({ARM_DOF_NAMES[0]: 0.0})
        invalid = positions(0.0, 0.2, 0.0, -0.5, 0.0, 0.0)
        invalid["right_elbow_joint"] = float("inf")
        with self.assertRaisesRegex(ArmKinematicsError, "finite"):
            self.kinematics.forward(invalid)
        with self.assertRaisesRegex(ArmKinematicsError, "allowed range"):
            self.kinematics.forward(positions(0.0, -0.01, 0.0, -0.5, 0.0, 0.0))

    def test_bounded_damped_ik_converges_for_reachable_right_palm_pose(self):
        seed = positions(0.25, 0.35, -0.10, -0.75, 0.15, 0.05)
        goal = positions(0.38, 0.48, 0.08, -0.92, -0.12, 0.18)
        target = self.kinematics.forward(goal)
        result = self.kinematics.solve(target, seed)
        achieved = self.kinematics.forward(result.as_mapping())
        self.assertLessEqual(result.position_error_m, 2e-4)
        self.assertLessEqual(result.orientation_error_rad, 2e-3)
        np.testing.assert_allclose(achieved.position_m, target.position_m, atol=2e-4)
        for name, value in result.positions_rad:
            limit = self.spec.model.joint_limits[name]
            self.assertGreaterEqual(value, limit.lower)
            self.assertLessEqual(value, limit.upper)

    def test_unreachable_pose_reports_bounded_convergence_failure(self):
        seed = positions(0.25, 0.35, -0.10, -0.75, 0.15, 0.05)
        target = ToolPose((5.0, 5.0, 5.0), (0.0, 0.0, 0.0, 1.0))
        with self.assertRaises(IKConvergenceError) as raised:
            self.kinematics.solve(target, seed, max_iterations=12)
        self.assertLessEqual(raised.exception.iterations, 12)
        self.assertGreater(raised.exception.position_error_m, 1.0)

    def test_pregrasp_preplacement_plan_has_bounded_arm_only_interpolation(self):
        initial = positions(0.25, 0.35, -0.10, -0.75, 0.15, 0.05)
        pre_grasp_joints = positions(0.34, 0.44, 0.02, -0.86, -0.04, 0.14)
        pre_place_joints = positions(0.18, 0.58, -0.12, -1.02, 0.18, -0.02)
        planner = ArmReachPlanner(
            self.kinematics,
            maximum_joint_step_rad=0.025,
            command_period_s=0.04,
            velocity_limit_scale=0.2,
        )
        plan = planner.plan_pregrasp_preplacement(
            initial,
            self.kinematics.forward(pre_grasp_joints),
            self.kinematics.forward(pre_place_joints),
        )
        self.assertEqual(plan.base_frame, KINEMATIC_BASE_FRAME)
        self.assertEqual(plan.tool_frame, TOOL_FRAME)
        self.assertEqual(plan.arm_dof_names, ARM_DOF_NAMES)
        self.assertEqual(tuple(phase.name for phase in plan.phases), ("pre_grasp", "pre_place"))
        previous = initial
        for waypoint in plan.waypoints:
            current = waypoint.as_mapping()
            self.assertEqual(tuple(current), ARM_DOF_NAMES)
            for name in ARM_DOF_NAMES:
                self.assertLessEqual(
                    abs(current[name] - previous[name]),
                    planner.maximum_step_for_joint(name) + 1e-12,
                )
            previous = current
        final_pose = self.kinematics.forward(plan.waypoints[-1].as_mapping())
        np.testing.assert_allclose(
            final_pose.position_m,
            plan.phases[-1].target.position_m,
            atol=2e-4,
        )

    def test_waypoint_uses_existing_drive_targets_without_leg_or_waist_writes(self):
        runtime_names = tuple(reversed(self.spec.canonical_dof_order))
        articulation = FakeArticulation(runtime_names)
        controller = ArticulationController(self.spec, articulation)
        waypoint_values = positions(0.3, 0.4, 0.1, -0.8, -0.2, 0.15)
        from robot_spike.production.arm_reach import ReachWaypoint

        waypoint = ReachWaypoint(
            "pre_grasp",
            1,
            tuple((name, waypoint_values[name]) for name in ARM_DOF_NAMES),
        )
        commanded = command_reach_waypoint(controller, waypoint)
        self.assertEqual(commanded, ARM_DOF_NAMES)
        self.assertEqual(len(articulation.calls), 1)
        name, args, kwargs = articulation.calls[0]
        self.assertEqual(name, "set_dof_position_targets")
        self.assertEqual(args, ([[waypoint_values[name] for name in ARM_DOF_NAMES]],))
        self.assertEqual(
            kwargs["dof_indices"],
            [runtime_names.index(name) for name in ARM_DOF_NAMES],
        )
        commanded_names = {runtime_names[index] for index in kwargs["dof_indices"]}
        self.assertTrue(commanded_names.isdisjoint({"waist_yaw_joint", "right_knee_joint"}))


if __name__ == "__main__":
    unittest.main()
