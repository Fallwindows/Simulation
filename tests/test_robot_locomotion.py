from __future__ import annotations

import ast
from dataclasses import replace
import math
from pathlib import Path
import unittest

from robot_spike.production.locomotion import (
    BalanceCorrection,
    BalanceFeedbackController,
    BipedLocomotionController,
    ConservativeGaitTargetGenerator,
    FootFeedback,
    FootPose,
    FootSide,
    Footstep,
    GaitConfig,
    GaitPhase,
    LEG_JOINTS,
    LocomotionFeedback,
    LocomotionError,
    LocomotionState,
    PlanarPose,
    WaypointStepPlanner,
    swing_foot_trajectory,
    symmetric_crouch_targets,
)
from robot_spike.production.model import load_production_spec


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOT = ROOT / "robot_spike" / "production"


class RecordingJointController:
    def __init__(self, spec):
        self.spec = spec
        self.commands = []

    def command_joint_positions(self, targets):
        checked = self.spec.validate_targets(targets)
        self.commands.append(checked)
        return tuple(checked)


class MutableFeedbackSource:
    def __init__(self, feedback):
        self.feedback = feedback

    def read_feedback(self):
        return self.feedback


class RobotLocomotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_production_spec(PRODUCTION_ROOT)
        cls.config = GaitConfig()

    def feedback(
        self,
        timestamp_s,
        *,
        root_pose=PlanarPose(0.0, 0.0, 0.0),
        left_pose=FootPose((0.0, 0.0675, 0.0), 0.0),
        right_pose=FootPose((0.0, -0.0675, 0.0), 0.0),
        left_contact=True,
        right_contact=True,
        support_margin_m=0.03,
        linear=(0.0, 0.0, 0.0),
        angular=(0.0, 0.0, 0.0),
        tilt=(0.0, 0.0),
        root_height_m=None,
    ):
        joints = dict(self.spec.reset_joint_positions)
        velocities = {name: 0.0 for name in joints}
        return LocomotionFeedback(
            timestamp_s=timestamp_s,
            root_pose=root_pose,
            root_height_m=(
                self.spec.root_position_m[2]
                if root_height_m is None
                else root_height_m
            ),
            root_tilt_roll_pitch_rad=tilt,
            root_linear_velocity_body_mps=linear,
            root_angular_velocity_body_rps=angular,
            left_foot=FootFeedback(left_pose, left_contact),
            right_foot=FootFeedback(right_pose, right_contact),
            joint_position_rad=joints,
            joint_velocity_rad_s=velocities,
            com_position_world_m=(root_pose.x_m, root_pose.y_m, 0.58),
            support_center_world_m=(root_pose.x_m, root_pose.y_m, 0.0),
            support_margin_m=support_margin_m,
        )

    def test_mirrored_knee_signs_and_every_leg_target_stays_inside_urdf_limits(self):
        crouch = symmetric_crouch_targets(self.spec)
        self.assertGreater(crouch["left_knee_joint"], 0.0)
        self.assertLess(crouch["right_knee_joint"], 0.0)
        self.assertAlmostEqual(
            crouch["left_knee_joint"], -crouch["right_knee_joint"]
        )
        self.assertAlmostEqual(
            crouch["left_hip_pitch_joint"], -crouch["right_hip_pitch_joint"]
        )
        self.assertAlmostEqual(
            crouch["left_ankle_pitch_joint"],
            -crouch["right_ankle_pitch_joint"],
        )

        generator = ConservativeGaitTargetGenerator(self.spec, self.config)
        step = Footstep(
            FootSide.LEFT,
            PlanarPose(0.06, 0.0, 0.0),
            FootPose((0.06, 0.0675, 0.0), 0.0),
        )
        feedback = self.feedback(0.5)
        targets = generator.targets(
            feedback,
            GaitPhase.LEFT_SWING,
            0.5,
            BalanceCorrection(
                self.config.maximum_balance_correction_rad,
                -self.config.maximum_balance_correction_rad,
            ),
            step=step,
            swing_start_pose=feedback.left_foot.pose,
            swing_start_joint_positions=feedback.joint_position_rad,
        )
        self.assertGreater(targets["left_knee_joint"], 0.0)
        self.assertLess(targets["right_knee_joint"], 0.0)
        for name, value in targets.items():
            limit = self.spec.model.joint_limits[name]
            self.assertGreaterEqual(value, limit.lower)
            self.assertLessEqual(value, limit.upper)
        self.assertEqual(self.spec.validate_targets(targets), targets)

    def test_swing_reference_has_exact_endpoints_clearance_and_smooth_endpoint_velocity(self):
        start = FootPose((0.0, 0.0675, 0.0), 0.0)
        target = FootPose((0.06, 0.0675, 0.0), math.radians(5.0))
        self.assertEqual(swing_foot_trajectory(start, target, 0.0, 0.025), start)
        self.assertEqual(swing_foot_trajectory(start, target, 1.0, 0.025), target)
        middle = swing_foot_trajectory(start, target, 0.5, 0.025)
        self.assertAlmostEqual(middle.position_m[0], 0.03)
        self.assertAlmostEqual(middle.position_m[2], 0.025)

        epsilon = 1e-5
        near_start = swing_foot_trajectory(start, target, epsilon, 0.025)
        near_end = swing_foot_trajectory(start, target, 1.0 - epsilon, 0.025)
        start_speed_bound = math.dist(start.position_m, near_start.position_m) / epsilon
        end_speed_bound = math.dist(target.position_m, near_end.position_m) / epsilon
        self.assertLess(start_speed_bound, 1e-3)
        self.assertLess(end_speed_bound, 1e-3)

        before = swing_foot_trajectory(start, target, 0.5 - epsilon, 0.025)
        after = swing_foot_trajectory(start, target, 0.5 + epsilon, 0.025)
        self.assertLess(math.dist(before.position_m, after.position_m), 1e-4)

    def test_waypoints_become_low_speed_bounded_alternating_steps(self):
        planner = WaypointStepPlanner(self.config)
        target = PlanarPose(0.25, 0.10, math.radians(30.0))
        steps = planner.plan(PlanarPose(0.0, 0.0, 0.0), [target])
        self.assertGreater(len(steps), 1)
        self.assertEqual(steps[-1].body_target, target)
        previous = PlanarPose(0.0, 0.0, 0.0)
        previous_foot = {
            FootSide.LEFT: FootPose((0.0, 0.0675, 0.0), 0.0),
            FootSide.RIGHT: FootPose((0.0, -0.0675, 0.0), 0.0),
        }
        for index, step in enumerate(steps):
            self.assertIs(
                step.side,
                FootSide.LEFT if index % 2 == 0 else FootSide.RIGHT,
            )
            translation = math.hypot(
                step.body_target.x_m - previous.x_m,
                step.body_target.y_m - previous.y_m,
            )
            yaw = abs(
                math.atan2(
                    math.sin(step.body_target.yaw_rad - previous.yaw_rad),
                    math.cos(step.body_target.yaw_rad - previous.yaw_rad),
                )
            )
            self.assertLessEqual(translation, self.config.max_step_length_m + 1e-12)
            self.assertLessEqual(yaw, self.config.max_step_yaw_rad + 1e-12)
            implied_speed = translation / (
                self.config.swing_duration_s + self.config.double_support_duration_s
            )
            self.assertLessEqual(implied_speed, self.config.nominal_speed_mps + 1e-12)
            self.assertLessEqual(
                math.dist(
                    step.foot_target.position_m,
                    previous_foot[step.side].position_m,
                ),
                self.config.max_step_length_m + 1e-12,
            )
            self.assertLessEqual(
                abs(
                    math.atan2(
                        math.sin(
                            step.foot_target.yaw_rad
                            - previous_foot[step.side].yaw_rad
                        ),
                        math.cos(
                            step.foot_target.yaw_rad
                            - previous_foot[step.side].yaw_rad
                        ),
                    )
                ),
                self.config.max_step_yaw_rad + 1e-12,
            )
            previous_foot[step.side] = step.foot_target
            previous = step.body_target

    def test_lateral_waypoints_produce_distinct_bounded_roll_targets(self):
        planner = WaypointStepPlanner(self.config)
        feedback = self.feedback(0.5)
        generator = ConservativeGaitTargetGenerator(self.spec, self.config)
        commands = []
        for lateral_m in (0.015, -0.015):
            step = planner.plan(
                PlanarPose(0.0, 0.0, 0.0),
                [PlanarPose(0.02, lateral_m, 0.0)],
            )[0]
            targets = generator.targets(
                feedback,
                GaitPhase.LEFT_SWING,
                0.8,
                BalanceCorrection(0.0, 0.0),
                step=step,
                swing_start_pose=feedback.left_foot.pose,
                swing_start_joint_positions=feedback.joint_position_rad,
            )
            commands.append(targets)
        positive, negative = commands
        self.assertGreater(positive["left_hip_roll_joint"], 0.0)
        self.assertLess(negative["left_hip_roll_joint"], 0.0)
        self.assertNotEqual(
            positive["left_ankle_roll_joint"],
            negative["left_ankle_roll_joint"],
        )
        for targets in commands:
            self.assertEqual(self.spec.validate_targets(targets), targets)

    def test_hip_yaw_sign_matches_both_negative_z_axes_for_both_turn_directions(self):
        feedback = self.feedback(0.5)
        generator = ConservativeGaitTargetGenerator(self.spec, self.config)
        for side in FootSide:
            start_pose = feedback.foot(side).pose
            phase = (
                GaitPhase.LEFT_SWING
                if side is FootSide.LEFT
                else GaitPhase.RIGHT_SWING
            )
            for desired_yaw in (math.radians(5.0), math.radians(-5.0)):
                with self.subTest(side=side, desired_yaw=desired_yaw):
                    step = Footstep(
                        side,
                        PlanarPose(0.02, 0.0, desired_yaw),
                        FootPose(
                            (
                                start_pose.position_m[0] + 0.02,
                                start_pose.position_m[1],
                                start_pose.position_m[2],
                            ),
                            desired_yaw,
                        ),
                    )
                    targets = generator.targets(
                        feedback,
                        phase,
                        1.0,
                        BalanceCorrection(0.0, 0.0),
                        step=step,
                        swing_start_pose=start_pose,
                        swing_start_joint_positions=feedback.joint_position_rad,
                    )
                    self.assertAlmostEqual(
                        targets[f"{side.value}_hip_yaw_joint"],
                        -desired_yaw,
                    )

    def test_touchdown_rejects_materially_wrong_measured_foot_yaw(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        controller = BipedLocomotionController(
            self.spec, RecordingJointController(self.spec), source, config=self.config
        )
        controller.start_route([PlanarPose(0.0, 0.0, math.radians(10.0))])
        controller.update()
        source.feedback = self.feedback(0.31)
        self.assertEqual(controller.update().phase, GaitPhase.LEFT_SWING)
        source.feedback = self.feedback(0.45, left_contact=False)
        controller.update()

        target = controller.steps[0].foot_target
        wrong_yaw_pose = FootPose(target.position_m, -target.yaw_rad)
        source.feedback = self.feedback(
            1.05,
            left_pose=wrong_yaw_pose,
            left_contact=True,
        )
        rejected = controller.update()
        self.assertEqual(rejected.state, LocomotionState.ACTIVE)
        self.assertEqual(rejected.phase, GaitPhase.LEFT_SWING)
        self.assertEqual(rejected.active_step_index, 0)

    def test_measured_off_nominal_foot_distance_or_yaw_faults_before_liftoff(self):
        cases = (
            FootPose((-0.10, 0.0675, 0.0), 0.0),
            FootPose((0.0, 0.0675, 0.0), math.radians(20.0)),
        )
        for measured_pose in cases:
            with self.subTest(measured_pose=measured_pose):
                source = MutableFeedbackSource(
                    self.feedback(0.0, left_pose=measured_pose)
                )
                commands = RecordingJointController(self.spec)
                controller = BipedLocomotionController(
                    self.spec, commands, source, config=self.config
                )
                controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
                self.assertEqual(controller.update().phase, GaitPhase.DOUBLE_SUPPORT)
                command_count = len(commands.commands)
                source.feedback = self.feedback(0.31, left_pose=measured_pose)
                failed = controller.update()
                self.assertEqual(failed.state, LocomotionState.FAULT)
                self.assertEqual(failed.joint_targets_rad, {})
                self.assertEqual(len(commands.commands), command_count)
                self.assertIn("measured swing-foot", failed.failure_reason)

    def test_contact_gates_liftoff_and_touchdown_then_dock_requires_settle(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        commands = RecordingJointController(self.spec)
        controller = BipedLocomotionController(
            self.spec, commands, source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])

        first = controller.update()
        self.assertEqual((first.state, first.phase), (LocomotionState.ACTIVE, GaitPhase.DOUBLE_SUPPORT))

        source.feedback = self.feedback(0.31)
        liftoff = controller.update()
        self.assertEqual(liftoff.phase, GaitPhase.LEFT_SWING)

        source.feedback = self.feedback(0.45, left_contact=False)
        airborne = controller.update()
        self.assertEqual(airborne.phase, GaitPhase.LEFT_SWING)
        self.assertIsNotNone(airborne.swing_foot_reference)

        target = controller.steps[0].foot_target
        source.feedback = self.feedback(1.05, left_pose=target, left_contact=True)
        touchdown = controller.update()
        self.assertEqual(touchdown.phase, GaitPhase.DOUBLE_SUPPORT)
        self.assertEqual(touchdown.active_step_index, 1)

        source.feedback = self.feedback(
            1.36, root_pose=PlanarPose(0.03, 0.0, 0.0)
        )
        docking = controller.update()
        self.assertEqual(docking.state, LocomotionState.DOCKING)
        source.feedback = self.feedback(
            1.50, root_pose=PlanarPose(0.03, 0.0, 0.0)
        )
        self.assertEqual(controller.update().state, LocomotionState.DOCKING)
        source.feedback = self.feedback(
            1.91, root_pose=PlanarPose(0.03, 0.0, 0.0)
        )
        self.assertEqual(controller.update().state, LocomotionState.DOCKED)
        self.assertGreaterEqual(len(commands.commands), 7)

    def test_swing_requires_observed_unload_and_faults_on_touchdown_timeout(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        controller = BipedLocomotionController(
            self.spec, RecordingJointController(self.spec), source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
        controller.update()
        source.feedback = self.feedback(0.31)
        controller.update()

        # Contact remains true throughout, so elapsed time alone cannot count as
        # a completed step even after the nominal swing duration.
        source.feedback = self.feedback(1.25)
        still_swinging = controller.update()
        self.assertEqual(still_swinging.phase, GaitPhase.LEFT_SWING)
        self.assertEqual(still_swinging.state, LocomotionState.ACTIVE)
        source.feedback = self.feedback(2.0)
        failed = controller.update()
        self.assertEqual(failed.state, LocomotionState.FAULT)
        self.assertIn("touchdown timeout", failed.failure_reason)

    def test_double_support_requires_continuous_contact_and_has_a_timeout(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        controller = BipedLocomotionController(
            self.spec, RecordingJointController(self.spec), source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
        controller.update()
        source.feedback = self.feedback(0.15, left_contact=False)
        self.assertEqual(controller.update().phase, GaitPhase.DOUBLE_SUPPORT)
        source.feedback = self.feedback(0.40)
        self.assertEqual(controller.update().phase, GaitPhase.DOUBLE_SUPPORT)
        source.feedback = self.feedback(0.69)
        self.assertEqual(controller.update().phase, GaitPhase.DOUBLE_SUPPORT)
        source.feedback = self.feedback(0.71)
        self.assertEqual(controller.update().phase, GaitPhase.LEFT_SWING)

        source = MutableFeedbackSource(self.feedback(0.0, left_contact=False))
        controller = BipedLocomotionController(
            self.spec, RecordingJointController(self.spec), source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
        controller.update()
        source.feedback = self.feedback(1.21, left_contact=False)
        failed = controller.update()
        self.assertEqual(failed.state, LocomotionState.FAULT)
        self.assertIn("double-support contact timeout", failed.failure_reason)

    def test_stop_request_finishes_airborne_step_before_settling(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        controller = BipedLocomotionController(
            self.spec, RecordingJointController(self.spec), source, config=self.config
        )
        controller.start_route([PlanarPose(0.12, 0.0, 0.0)])
        controller.update()
        source.feedback = self.feedback(0.31)
        controller.update()
        source.feedback = self.feedback(0.45, left_contact=False)
        controller.update()
        controller.request_stop()
        self.assertEqual(controller.state, LocomotionState.ACTIVE)

        target = controller.steps[0].foot_target
        source.feedback = self.feedback(1.05, left_pose=target, left_contact=True)
        self.assertEqual(controller.update().state, LocomotionState.STOPPING)
        source.feedback = self.feedback(1.20, left_pose=target)
        self.assertEqual(controller.update().state, LocomotionState.STOPPING)
        source.feedback = self.feedback(1.51, left_pose=target)
        self.assertEqual(controller.update().state, LocomotionState.STOPPED)

    def test_balance_interface_uses_measured_tilt_velocity_and_support_margin(self):
        balance = BalanceFeedbackController(self.config)
        feedback = self.feedback(
            0.0,
            tilt=(0.03, -0.04),
            linear=(0.02, -0.01, 0.0),
            angular=(0.1, -0.2, 0.0),
        )
        correction = balance.evaluate(feedback, desired_forward_speed_mps=0.03)
        self.assertNotEqual(correction.sagittal_rad, 0.0)
        self.assertNotEqual(correction.lateral_rad, 0.0)
        self.assertIsNone(correction.unsafe_reason)

        unsafe = balance.evaluate(self.feedback(0.1, support_margin_m=-0.02))
        self.assertIn("support margin", unsafe.unsafe_reason)

    def test_sagittal_correction_opposes_recorded_backward_pitch(self):
        recorded_left_chain_pitch = (
            -0.047485753893852234
            + 0.22577346861362457
            - 0.024831147864460945
        )
        self.assertAlmostEqual(
            -0.15350127538066533 + recorded_left_chain_pitch,
            0.0,
            delta=5e-5,
        )
        feedback = self.feedback(
            0.8916666666666667,
            tilt=(0.0, -0.15350127538066533),
            linear=(-0.18182469615475158, 0.0, 0.0),
            angular=(0.0, -0.43750293447273125, 0.0),
            support_margin_m=0.01,
        )
        correction = BalanceFeedbackController(self.config).evaluate(feedback)
        self.assertAlmostEqual(
            correction.sagittal_rad,
            self.config.maximum_balance_correction_rad,
        )

        targets = ConservativeGaitTargetGenerator(self.spec, self.config).targets(
            feedback,
            GaitPhase.DOUBLE_SUPPORT,
            0.0,
            correction,
        )
        left_chain_pitch = sum(
            targets[name]
            for name in (
                "left_hip_pitch_joint",
                "left_knee_joint",
                "left_ankle_pitch_joint",
            )
        )
        right_chain_pitch = -sum(
            targets[name]
            for name in (
                "right_hip_pitch_joint",
                "right_knee_joint",
                "right_ankle_pitch_joint",
            )
        )
        self.assertAlmostEqual(left_chain_pitch, -correction.sagittal_rad)
        self.assertAlmostEqual(right_chain_pitch, -correction.sagittal_rad)
        # Flat-foot kinematics make pelvis pitch the negative chain sum, so
        # the command is positive and opposes the recorded negative pitch.
        self.assertGreater(-left_chain_pitch, 0.0)

    def test_drive_ratio_damping_opposes_recorded_chain_velocity_within_gate(self):
        positions = dict(self.spec.reset_joint_positions)
        velocities = {name: 0.0 for name in positions}
        recorded = {
            "left_hip_pitch_joint": (-0.07449323683977127, 0.1399080753326416),
            "left_knee_joint": (0.2539362907409668, -0.11158437281847),
            "left_ankle_pitch_joint": (-0.035009998828172684, 0.361137330532074),
            "right_hip_pitch_joint": (0.07713029533624649, -0.22978147864341736),
            "right_knee_joint": (-0.25878196954727173, 0.24592140316963196),
            "right_ankle_pitch_joint": (0.03714081645011902, -0.35391318798065186),
        }
        for name, (position, velocity) in recorded.items():
            positions[name] = position
            velocities[name] = velocity
        feedback = replace(
            self.feedback(
                0.9833333333333333,
                tilt=(0.0, -0.144358),
                linear=(-0.134456, 0.0, 0.0),
                angular=(0.0, -0.3233670677357086, 0.0),
            ),
            joint_position_rad=positions,
            joint_velocity_rad_s=velocities,
        )
        correction = BalanceCorrection(
            self.config.maximum_balance_correction_rad, 0.0
        )
        targets = ConservativeGaitTargetGenerator(self.spec, self.config).targets(
            feedback, GaitPhase.DOUBLE_SUPPORT, 0.0, correction
        )
        legacy_config = GaitConfig(joint_velocity_damping_s=0.012)
        legacy = ConservativeGaitTargetGenerator(
            self.spec, legacy_config
        ).targets(feedback, GaitPhase.DOUBLE_SUPPORT, 0.0, correction)

        for side, sign in (("left", 1.0), ("right", -1.0)):
            names = tuple(
                f"{side}_{joint}_joint"
                for joint in ("hip_pitch", "knee", "ankle_pitch")
            )
            measured_chain_velocity = sign * sum(velocities[name] for name in names)
            target_chain = sign * sum(targets[name] for name in names)
            legacy_chain = sign * sum(legacy[name] for name in names)
            self.assertGreater(measured_chain_velocity, 0.0)
            self.assertLess(target_chain, legacy_chain)
            self.assertLess(target_chain, 0.0)
        maximum_error = max(
            abs(targets[name] - positions[name]) for name in LEG_JOINTS
        )
        self.assertLessEqual(maximum_error, self.config.maximum_target_error_rad)
        self.assertAlmostEqual(self.config.joint_velocity_damping_s, 12.0 / 120.0)
        with self.assertRaisesRegex(LocomotionError, "damping_s exceeds"):
            GaitConfig(joint_velocity_damping_s=0.100001)

    def test_collapsed_or_invalid_root_height_fails_before_gait_command(self):
        balance = BalanceFeedbackController(self.config)
        for height in (-1.0, 1.5):
            with self.subTest(height=height):
                correction = balance.evaluate(
                    self.feedback(0.0, root_height_m=height)
                )
                self.assertIn("root clearance", correction.unsafe_reason)

        source = MutableFeedbackSource(self.feedback(0.0, root_height_m=-1.0))
        commands = RecordingJointController(self.spec)
        controller = BipedLocomotionController(
            self.spec, commands, source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
        failed = controller.update()
        self.assertEqual(failed.state, LocomotionState.FAULT)
        self.assertEqual(failed.joint_targets_rad, {})
        self.assertEqual(commands.commands, [])

        source.feedback = self.feedback(0.1, root_height_m=float("nan"))
        self.assertEqual(controller.update().state, LocomotionState.FAULT)
        self.assertEqual(commands.commands, [])

    def test_nonfinite_safety_sample_latches_fault_from_active_and_never_resumes(self):
        source = MutableFeedbackSource(self.feedback(0.0))
        commands = RecordingJointController(self.spec)
        controller = BipedLocomotionController(
            self.spec, commands, source, config=self.config
        )
        controller.start_route([PlanarPose(0.03, 0.0, 0.0)])
        healthy = controller.update()
        self.assertEqual(healthy.state, LocomotionState.ACTIVE)
        self.assertEqual(len(commands.commands), 1)

        source.feedback = self.feedback(0.1, root_height_m=float("nan"))
        invalid = controller.update()
        self.assertEqual(invalid.state, LocomotionState.FAULT)
        self.assertIn("root height must be finite", invalid.failure_reason)
        self.assertEqual(invalid.joint_targets_rad, {})
        self.assertEqual(len(commands.commands), 1)

        source.feedback = self.feedback(0.2)
        latched = controller.update()
        self.assertEqual(latched.state, LocomotionState.FAULT)
        self.assertEqual(latched.joint_targets_rad, {})
        self.assertEqual(len(commands.commands), 1)

    def test_direct_root_and_joint_state_writes_are_confined_to_explicit_reset(self):
        forbidden = {
            "set_world_poses",
            "set_velocities",
            "set_dof_positions",
            "set_dof_velocities",
        }
        runtime_tree = ast.parse((PRODUCTION_ROOT / "runtime.py").read_text(encoding="utf-8"))
        runtime_calls = []
        for function in (
            node for node in ast.walk(runtime_tree) if isinstance(node, ast.FunctionDef)
        ):
            for call in (node for node in ast.walk(function) if isinstance(node, ast.Call)):
                if isinstance(call.func, ast.Attribute) and call.func.attr in forbidden:
                    runtime_calls.append((function.name, call.func.attr))
        self.assertEqual(
            sorted(runtime_calls),
            [
                ("reset", "set_dof_positions"),
                ("reset", "set_dof_velocities"),
                ("reset", "set_velocities"),
                ("reset", "set_world_poses"),
            ],
        )

        locomotion_tree = ast.parse(
            (PRODUCTION_ROOT / "locomotion.py").read_text(encoding="utf-8")
        )
        state_writes = [
            node.func.attr
            for node in ast.walk(locomotion_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in forbidden
        ]
        self.assertEqual(state_writes, [])
        normal_commands = [
            node.func.attr
            for node in ast.walk(locomotion_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "command_joint_positions"
        ]
        self.assertGreaterEqual(len(normal_commands), 1)


if __name__ == "__main__":
    unittest.main()
