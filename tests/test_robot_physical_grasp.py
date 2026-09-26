from __future__ import annotations

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from robot_spike.production.arm_reach import ARM_DOF_NAMES
from robot_spike.production.model import load_production_spec
from robot_spike.production.physical_grasp import (
    BodyPose,
    CLOSE_TARGETS,
    ContactPair,
    GraspFailure,
    GraspLimits,
    GraspObservation,
    GraspPhase,
    HAND_DOF_NAMES,
    OPEN_TARGETS,
    OPPOSING_CONTACT_LINKS,
    PRESHAPE_TARGETS,
    PhysicalGraspController,
    RobotContactBodyMap,
    THUMB_CONTACT_LINKS,
)
from robot_spike.production.runtime import ArticulationController
from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.restocking_layout import build_restocking_layout


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOT = ROOT / "robot_spike" / "production"


class FakeArticulation:
    def __init__(self, dof_names):
        self.dof_names = tuple(dof_names)
        self.calls = []

    def set_dof_position_targets(self, *args, **kwargs):
        self.calls.append(("set_dof_position_targets", args, kwargs))


class QueueFeedback:
    def __init__(self):
        self.samples = []

    def push(self, *samples):
        self.samples.extend(samples)

    def read_observation(self):
        if not self.samples:
            return None
        return self.samples.pop(0)


class RecordingArmLift:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.requests = []

    def request_lift(self, request):
        self.requests.append(request)
        return self.accepted


class RobotPhysicalGraspTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_production_spec(PRODUCTION_ROOT)
        scenario = load_scenario(ROOT / "config/scenarios/baseline_straight.yaml")
        cls.layout = build_restocking_layout(build_aisle_layout(scenario.environment))
        cls.robot_contact_paths = {
            link: f"/World/Robot/contact_bodies/{link}"
            for link in THUMB_CONTACT_LINKS | OPPOSING_CONTACT_LINKS
        }
        cls.robot_contacts = RobotContactBodyMap(
            "/World/Robot", cls.robot_contact_paths
        )

    def make_controller(self, **limit_overrides):
        runtime_names = tuple(reversed(self.spec.canonical_dof_order))
        articulation = FakeArticulation(runtime_names)
        position_controller = ArticulationController(self.spec, articulation)
        feedback = QueueFeedback()
        lift = RecordingArmLift()
        values = {
            "maximum_observations_per_phase": 5,
            "contact_confirmation_samples": 2,
            "lift_confirmation_samples": 2,
        }
        values.update(limit_overrides)
        controller = PhysicalGraspController(
            position_controller,
            self.layout,
            feedback=feedback,
            arm_lift=lift,
            robot_contacts=self.robot_contacts,
            limits=GraspLimits(**values),
        )
        return controller, articulation, feedback, lift

    def contacts(
        self,
        *,
        support=True,
        thumb=True,
        opposing=True,
        wrong_product=False,
        support_robot_label=None,
    ):
        product = (
            "/World/Restocking/WrongProduct/Collider"
            if wrong_product
            else self.layout.product.collider_prim_path
        )
        result = []
        if support:
            result.append(
                ContactPair(
                    product,
                    self.layout.pickup_support.fixture.prim_path,
                    3.2,
                    support_robot_label,
                )
            )
        if thumb:
            result.append(
                ContactPair(
                    product,
                    self.robot_contact_paths["right_thumb_dp"],
                    1.1,
                    "right_thumb_dp",
                )
            )
        if opposing:
            result.append(
                ContactPair(
                    product,
                    self.robot_contact_paths["right_index_ip"],
                    0.9,
                    "right_index_ip",
                )
            )
        return tuple(result)

    def observation(
        self,
        timestamp,
        joints,
        *,
        product_lift=0.0,
        palm_lift=0.0,
        product_xy_offset=(0.0, 0.0),
        contacts=None,
        product_path=None,
    ):
        source = self.layout.product.source_reset_pose
        product_position = (
            source.position_m[0] + product_xy_offset[0],
            source.position_m[1] + product_xy_offset[1],
            source.position_m[2] + product_lift,
        )
        palm_position = (
            source.position_m[0],
            source.position_m[1] - 0.10,
            source.position_m[2] + palm_lift,
        )
        return GraspObservation(
            timestamp_s=timestamp,
            product_prim_path=(
                product_path or self.layout.product.rigid_body_prim_path
            ),
            hand_joint_positions_rad=dict(joints),
            palm_pose_world=BodyPose(palm_position, (0.0, 0.0, 0.0, 1.0)),
            product_pose_world=BodyPose(product_position, source.orientation_xyzw),
            contacts=tuple(contacts if contacts is not None else ()),
        )

    def advance_to_closing(self, controller, feedback):
        controller.start()
        feedback.push(self.observation(1.0, OPEN_TARGETS))
        self.assertEqual(controller.step().phase, GraspPhase.PRESHAPING)
        feedback.push(self.observation(2.0, PRESHAPE_TARGETS))
        self.assertEqual(controller.step().phase, GraspPhase.CLOSING)

    def advance_to_lifting(self, controller, feedback, *, contacts=None):
        self.advance_to_closing(controller, feedback)
        source_contacts = contacts if contacts is not None else self.contacts()
        feedback.push(
            self.observation(3.0, CLOSE_TARGETS, contacts=source_contacts),
            self.observation(4.0, CLOSE_TARGETS, contacts=source_contacts),
        )
        self.assertEqual(controller.step().phase, GraspPhase.CLOSING)
        self.assertEqual(controller.step().phase, GraspPhase.LIFTING)

    def commanded_names(self, articulation, call_index):
        _, _args, kwargs = articulation.calls[call_index]
        return tuple(articulation.dof_names[index] for index in kwargs["dof_indices"])

    def test_joint_and_contact_contract_matches_preserved_orcahand_urdf(self):
        robot = ET.parse(PRODUCTION_ROOT / "asimov_orcahand_restocking.urdf").getroot()
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
        links = {link.attrib["name"]: link for link in robot.findall("link")}

        self.assertEqual(len(HAND_DOF_NAMES), 16)
        self.assertNotIn("right_wrist", HAND_DOF_NAMES)
        for name in HAND_DOF_NAMES:
            self.assertEqual(joints[name].attrib["type"], "revolute")
            limit = self.spec.model.joint_limits[name]
            for targets in (OPEN_TARGETS, PRESHAPE_TARGETS, CLOSE_TARGETS):
                self.assertGreaterEqual(targets[name], limit.lower)
                self.assertLessEqual(targets[name], limit.upper)

        for name in THUMB_CONTACT_LINKS | OPPOSING_CONTACT_LINKS:
            self.assertTrue(links[name].findall("collision"), name)
        for digit in ("thumb", "index", "middle", "ring", "pinky"):
            self.assertFalse(links[f"right_{digit}_fingertip"].findall("collision"))
        spoofed_map = dict(self.robot_contact_paths)
        spoofed_map["right_thumb_dp"] = (
            "/World/Robot/contact_bodies/right_thumb_fingertip"
        )
        with self.assertRaisesRegex(ValueError, "exact URDF link name"):
            RobotContactBodyMap("/World/Robot", spoofed_map)

    def test_hand_stages_use_only_name_bound_finger_drive_targets(self):
        controller, articulation, feedback, _lift = self.make_controller()
        self.advance_to_closing(controller, feedback)

        self.assertEqual(len(articulation.calls), 3)
        self.assertEqual(
            [call[0] for call in articulation.calls],
            ["set_dof_position_targets"] * 3,
        )
        for index in range(3):
            commanded = set(self.commanded_names(articulation, index))
            self.assertEqual(commanded, set(HAND_DOF_NAMES))
            self.assertTrue(commanded.isdisjoint(ARM_DOF_NAMES))
            self.assertNotIn("waist_yaw_joint", commanded)
            self.assertFalse(any("hip" in name or "knee" in name for name in commanded))

    def test_opposing_contact_then_measured_secured_lift_completes(self):
        controller, _articulation, feedback, lift = self.make_controller()
        self.advance_to_lifting(controller, feedback)
        self.assertEqual(len(lift.requests), 1)
        self.assertAlmostEqual(lift.requests[0].displacement_m, 0.075)

        lifted_contacts = self.contacts(support=False)
        feedback.push(
            self.observation(
                5.0,
                CLOSE_TARGETS,
                product_lift=0.060,
                palm_lift=0.060,
                contacts=lifted_contacts,
            ),
            self.observation(
                6.0,
                CLOSE_TARGETS,
                product_lift=0.065,
                palm_lift=0.065,
                contacts=lifted_contacts,
            ),
        )
        self.assertEqual(controller.step().phase, GraspPhase.LIFTING)
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.COMPLETE)
        self.assertIsNone(status.failure)

    def test_one_sided_contact_never_requests_lift_and_times_out(self):
        controller, _articulation, feedback, lift = self.make_controller(
            maximum_observations_per_phase=2,
            contact_confirmation_samples=1,
        )
        self.advance_to_closing(controller, feedback)
        one_sided = self.contacts(opposing=False)
        feedback.push(
            self.observation(3.0, CLOSE_TARGETS, contacts=one_sided),
            self.observation(4.0, CLOSE_TARGETS, contacts=one_sided),
        )
        controller.step()
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.CONTACT_NOT_VERIFIED)
        self.assertEqual(lift.requests, [])

    def test_wrong_product_contact_does_not_count_as_grasp(self):
        controller, _articulation, feedback, lift = self.make_controller(
            maximum_observations_per_phase=1,
            contact_confirmation_samples=1,
        )
        self.advance_to_closing(controller, feedback)
        feedback.push(
            self.observation(
                3.0,
                CLOSE_TARGETS,
                contacts=self.contacts(wrong_product=True),
            )
        )
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.INVALID_FEEDBACK)
        self.assertEqual(lift.requests, [])

    def test_two_sided_contact_without_source_support_fails_source_gate(self):
        controller, _articulation, feedback, lift = self.make_controller(
            contact_confirmation_samples=1,
        )
        self.advance_to_closing(controller, feedback)
        feedback.push(
            self.observation(
                3.0,
                CLOSE_TARGETS,
                contacts=self.contacts(support=False),
            )
        )
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.SOURCE_STATE_INVALID)
        self.assertEqual(lift.requests, [])

    def test_arm_lift_request_rejection_fails_without_claiming_motion(self):
        runtime_names = tuple(reversed(self.spec.canonical_dof_order))
        articulation = FakeArticulation(runtime_names)
        feedback = QueueFeedback()
        lift = RecordingArmLift(accepted=False)
        controller = PhysicalGraspController(
            ArticulationController(self.spec, articulation),
            self.layout,
            feedback=feedback,
            arm_lift=lift,
            robot_contacts=self.robot_contacts,
            limits=GraspLimits(contact_confirmation_samples=1),
        )
        self.advance_to_closing(controller, feedback)
        feedback.push(self.observation(3.0, CLOSE_TARGETS, contacts=self.contacts()))
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.LIFT_REQUEST_REJECTED)
        self.assertEqual(len(lift.requests), 1)

    def test_default_missing_feedback_fails_closed_after_bound(self):
        runtime_names = tuple(reversed(self.spec.canonical_dof_order))
        articulation = FakeArticulation(runtime_names)
        controller = PhysicalGraspController(
            ArticulationController(self.spec, articulation),
            self.layout,
            limits=GraspLimits(maximum_observations_per_phase=2),
        )
        controller.start()
        self.assertEqual(controller.step().phase, GraspPhase.OPENING)
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.FEEDBACK_UNAVAILABLE)
        self.assertEqual(len(articulation.calls), 1)

    def test_mislabeled_exact_support_pair_remains_authoritative(self):
        controller, _articulation, feedback, lift = self.make_controller(
            maximum_observations_per_phase=2,
            contact_confirmation_samples=1,
            lift_confirmation_samples=1,
        )
        self.advance_to_closing(controller, feedback)
        feedback.push(self.observation(3.0, CLOSE_TARGETS, contacts=self.contacts()))
        self.assertEqual(controller.step().phase, GraspPhase.LIFTING)
        self.assertEqual(len(lift.requests), 1)

        still_supported = self.contacts(
            support=True,
            support_robot_label="right_thumb_dp",
        )
        feedback.push(
            self.observation(4.0, CLOSE_TARGETS, contacts=still_supported),
            self.observation(5.0, CLOSE_TARGETS, contacts=still_supported),
        )
        controller.step()
        status = controller.step()
        self.assertEqual(status.phase, GraspPhase.FAILED)
        self.assertEqual(status.failure, GraspFailure.LIFT_NOT_VERIFIED)

    def test_spoofed_or_inconsistent_robot_body_paths_fail_closed(self):
        product = self.layout.product.collider_prim_path
        cases = (
            (
                "marker paths claiming collision links",
                ContactPair(
                    product,
                    "/World/Robot/right_thumb_fingertip",
                    1.1,
                    "right_thumb_dp",
                ),
                ContactPair(
                    product,
                    "/World/Robot/right_index_fingertip",
                    0.9,
                    "right_index_ip",
                ),
            ),
            (
                "mapped bodies with inconsistent labels",
                ContactPair(
                    product,
                    self.robot_contact_paths["right_thumb_dp"],
                    1.1,
                    "right_index_ip",
                ),
                ContactPair(
                    product,
                    self.robot_contact_paths["right_index_ip"],
                    0.9,
                    "right_thumb_dp",
                ),
            ),
            (
                "unrelated product contact bodies",
                ContactPair(product, "/World/Unrelated/BodyA", 1.1),
                ContactPair(product, "/World/Unrelated/BodyB", 0.9),
            ),
        )
        for label, thumb, opposing in cases:
            with self.subTest(label=label):
                controller, _articulation, feedback, lift = self.make_controller(
                    contact_confirmation_samples=1,
                )
                self.advance_to_closing(controller, feedback)
                spoofed = (
                    ContactPair(
                        product,
                        self.layout.pickup_support.fixture.prim_path,
                        3.2,
                    ),
                    thumb,
                    opposing,
                )
                feedback.push(
                    self.observation(3.0, CLOSE_TARGETS, contacts=spoofed)
                )
                status = controller.step()
                self.assertEqual(status.phase, GraspPhase.FAILED)
                self.assertEqual(status.failure, GraspFailure.INVALID_FEEDBACK)
                self.assertEqual(lift.requests, [])

    def test_lift_deadline_accepts_boundary_and_rejects_late_evidence(self):
        for timestamp, expected_phase, expected_failure in (
            (7.0, GraspPhase.COMPLETE, None),
            (7.000001, GraspPhase.FAILED, GraspFailure.LIFT_NOT_VERIFIED),
            (1000.0, GraspPhase.FAILED, GraspFailure.LIFT_NOT_VERIFIED),
        ):
            with self.subTest(timestamp=timestamp):
                controller, _articulation, feedback, _lift = self.make_controller(
                    contact_confirmation_samples=1,
                    lift_confirmation_samples=1,
                    maximum_lift_duration_s=4.0,
                )
                self.advance_to_closing(controller, feedback)
                feedback.push(
                    self.observation(3.0, CLOSE_TARGETS, contacts=self.contacts())
                )
                self.assertEqual(controller.step().phase, GraspPhase.LIFTING)
                feedback.push(
                    self.observation(
                        timestamp,
                        CLOSE_TARGETS,
                        product_lift=0.06,
                        palm_lift=0.06,
                        contacts=self.contacts(support=False),
                    )
                )
                status = controller.step()
                self.assertEqual(status.phase, expected_phase)
                self.assertEqual(status.failure, expected_failure)

    def test_relative_slip_or_lost_contact_is_a_drop(self):
        for label, contacts, xy_offset in (
            ("relative slip", self.contacts(support=False), (0.05, 0.0)),
            ("lost opposing contact", self.contacts(support=False, opposing=False), (0.0, 0.0)),
        ):
            with self.subTest(label=label):
                controller, _articulation, feedback, _lift = self.make_controller(
                    contact_confirmation_samples=1,
                    lift_confirmation_samples=1,
                )
                self.advance_to_closing(controller, feedback)
                feedback.push(self.observation(3.0, CLOSE_TARGETS, contacts=self.contacts()))
                self.assertEqual(controller.step().phase, GraspPhase.LIFTING)
                feedback.push(
                    self.observation(
                        4.0,
                        CLOSE_TARGETS,
                        product_lift=0.06,
                        palm_lift=0.06,
                        product_xy_offset=xy_offset,
                        contacts=contacts,
                    )
                )
                status = controller.step()
                self.assertEqual(status.phase, GraspPhase.FAILED)
                self.assertEqual(status.failure, GraspFailure.OBJECT_DROPPED)

    def test_stale_or_nonfinite_measurements_fail_closed(self):
        controller, _articulation, feedback, _lift = self.make_controller()
        controller.start()
        feedback.push(self.observation(1.0, OPEN_TARGETS))
        controller.step()
        stale = self.observation(1.0, PRESHAPE_TARGETS)
        feedback.push(stale)
        status = controller.step()
        self.assertEqual(status.failure, GraspFailure.INVALID_FEEDBACK)

        controller, _articulation, feedback, _lift = self.make_controller()
        controller.start()
        invalid = dict(OPEN_TARGETS)
        invalid[HAND_DOF_NAMES[0]] = math.nan
        feedback.push(self.observation(1.0, invalid))
        status = controller.step()
        self.assertEqual(status.failure, GraspFailure.INVALID_FEEDBACK)


if __name__ == "__main__":
    unittest.main()
