"""CPU-safe R4 hand sequence and fail-closed physical grasp verification.

The controller owns only the represented OrcaHand finger joints.  It sends
their position targets through :class:`ArticulationController` and delegates
the arm lift to a separate semantic port.  Completion depends entirely on
injected, measured contact and body-pose observations; drive commands are never
treated as evidence that the pasta box moved or remained grasped.

No Isaac/PhysX adapter is claimed here.  Until a runtime adapter can supply the
contract below from installed, version-specific APIs, the default feedback
source returns no sample and this controller reaches a bounded failure state.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping, Protocol, Sequence

from simulator.environment.restocking_layout import (
    PASTA_BOX_ASSET_KEY,
    PASTA_BOX_DIMENSIONS_M,
    RestockingLayout,
    validate_restocking_layout,
)

from .runtime import ArticulationController


Vec3 = tuple[float, float, float]
QuaternionXyzw = tuple[float, float, float, float]


# ``right_wrist`` belongs to the R3 arm kinematic chain and is deliberately
# absent.  Every name below is a revolute DOF in the preserved OrcaHand V1 URDF.
HAND_DOF_NAMES = (
    "right_thumb_mcp",
    "right_index_abd",
    "right_middle_abd",
    "right_ring_abd",
    "right_pinky_abd",
    "right_thumb_abd",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_pinky_mcp",
    "right_thumb_pip",
    "right_index_pip",
    "right_middle_pip",
    "right_ring_pip",
    "right_pinky_pip",
    "right_thumb_dip",
)


# The fixed ``*_fingertip`` marker links in the source URDF have zero mass and
# no collision geometry.  Physical contact must therefore be reported on the
# adjacent collision-bearing distal links or the collision-bearing palm.
THUMB_CONTACT_LINKS = frozenset(("right_thumb_ip", "right_thumb_dp"))
OPPOSING_CONTACT_LINKS = frozenset(
    (
        "right_index_ip",
        "right_middle_ip",
        "right_ring_ip",
        "right_pinky_ip",
        "right_palm",
    )
)


OPEN_TARGETS = {
    "right_thumb_mcp": 0.0,
    "right_index_abd": 0.0,
    "right_middle_abd": 0.0,
    "right_ring_abd": 0.0,
    "right_pinky_abd": 0.0,
    "right_thumb_abd": 0.0,
    "right_index_mcp": 0.0,
    "right_middle_mcp": 0.0,
    "right_ring_mcp": 0.0,
    "right_pinky_mcp": 0.0,
    "right_thumb_pip": 0.0,
    "right_index_pip": 0.0,
    "right_middle_pip": 0.0,
    "right_ring_pip": 0.0,
    "right_pinky_pip": 0.0,
    "right_thumb_dip": 0.0,
}


PRESHAPE_TARGETS = {
    "right_thumb_mcp": 0.12,
    "right_index_abd": 0.0,
    "right_middle_abd": 0.0,
    "right_ring_abd": 0.0,
    "right_pinky_abd": 0.0,
    "right_thumb_abd": -0.22,
    "right_index_mcp": 0.25,
    "right_middle_mcp": 0.22,
    "right_ring_mcp": 0.20,
    "right_pinky_mcp": 0.18,
    "right_thumb_pip": 0.25,
    "right_index_pip": 0.25,
    "right_middle_pip": 0.23,
    "right_ring_pip": 0.22,
    "right_pinky_pip": 0.20,
    "right_thumb_dip": 0.20,
}


CLOSE_TARGETS = {
    "right_thumb_mcp": 0.25,
    "right_index_abd": 0.0,
    "right_middle_abd": 0.0,
    "right_ring_abd": 0.0,
    "right_pinky_abd": 0.0,
    "right_thumb_abd": -0.40,
    "right_index_mcp": 0.70,
    "right_middle_mcp": 0.60,
    "right_ring_mcp": 0.55,
    "right_pinky_mcp": 0.50,
    "right_thumb_pip": 0.60,
    "right_index_pip": 0.80,
    "right_middle_pip": 0.75,
    "right_ring_pip": 0.70,
    "right_pinky_pip": 0.65,
    "right_thumb_dip": 0.50,
}


class GraspPhase(str, Enum):
    IDLE = "idle"
    OPENING = "opening"
    PRESHAPING = "preshaping"
    CLOSING = "closing"
    LIFTING = "lifting"
    COMPLETE = "complete"
    FAILED = "failed"


class GraspFailure(str, Enum):
    FEEDBACK_UNAVAILABLE = "feedback_unavailable"
    INVALID_FEEDBACK = "invalid_feedback"
    HAND_TARGET_TIMEOUT = "hand_target_timeout"
    CONTACT_NOT_VERIFIED = "contact_not_verified"
    SOURCE_STATE_INVALID = "source_state_invalid"
    LIFT_REQUEST_REJECTED = "lift_request_rejected"
    OBJECT_DROPPED = "object_dropped"
    LIFT_NOT_VERIFIED = "lift_not_verified"


@dataclass(frozen=True)
class BodyPose:
    position_m: Vec3
    orientation_xyzw: QuaternionXyzw


@dataclass(frozen=True)
class ContactPair:
    """One measured contact involving named USD prims.

    ``robot_link_name`` is required only for a robot/product contact.  The
    runtime adapter must resolve it from the actual collision-bearing robot
    link rather than a fixed fingertip marker or a requested joint target.
    ``normal_force_n`` is a force value; an impulse-based API must divide by
    the measured physics step before constructing this record.
    """

    body0_prim_path: str
    body1_prim_path: str
    normal_force_n: float
    robot_link_name: str | None = None


@dataclass(frozen=True)
class GraspObservation:
    """Co-timed measured state required by the R4 verifier."""

    timestamp_s: float
    product_prim_path: str
    hand_joint_positions_rad: Mapping[str, float]
    palm_pose_world: BodyPose
    product_pose_world: BodyPose
    contacts: tuple[ContactPair, ...]


class PhysicalFeedback(Protocol):
    def read_observation(self) -> GraspObservation | None: ...


@dataclass(frozen=True)
class LiftRequest:
    displacement_m: float
    maximum_duration_s: float
    product_prim_path: str


class ArmLiftPort(Protocol):
    """Arm-owned seam; accepting a request is never lift success evidence."""

    def request_lift(self, request: LiftRequest) -> bool: ...


class UnavailablePhysicalFeedback:
    """Safe default while the installed Isaac contact adapter is unresolved."""

    def read_observation(self) -> None:
        return None


class UnavailableArmLiftPort:
    def request_lift(self, request: LiftRequest) -> bool:
        del request
        return False


@dataclass(frozen=True)
class GraspLimits:
    joint_tolerance_rad: float = 0.04
    minimum_contact_force_n: float = 0.05
    source_pose_tolerance_m: float = 0.02
    requested_lift_m: float = 0.075
    minimum_verified_lift_m: float = 0.050
    maximum_relative_translation_drift_m: float = 0.018
    maximum_relative_rotation_drift_rad: float = 0.20
    maximum_lift_duration_s: float = 4.0
    maximum_observations_per_phase: int = 120
    contact_confirmation_samples: int = 3
    lift_confirmation_samples: int = 3

    def __post_init__(self) -> None:
        positive_floats = (
            self.joint_tolerance_rad,
            self.minimum_contact_force_n,
            self.source_pose_tolerance_m,
            self.requested_lift_m,
            self.minimum_verified_lift_m,
            self.maximum_relative_translation_drift_m,
            self.maximum_relative_rotation_drift_rad,
            self.maximum_lift_duration_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_floats):
            raise ValueError("all grasp limits must be finite and positive")
        if self.minimum_verified_lift_m > self.requested_lift_m:
            raise ValueError("verified lift cannot exceed requested lift")
        if self.maximum_observations_per_phase < 1:
            raise ValueError("maximum_observations_per_phase must be positive")
        if self.contact_confirmation_samples < 1 or self.lift_confirmation_samples < 1:
            raise ValueError("confirmation sample counts must be positive")


@dataclass(frozen=True)
class GraspStatus:
    phase: GraspPhase
    failure: GraspFailure | None
    phase_observations: int
    contact_confirmations: int
    lift_confirmations: int


@dataclass(frozen=True)
class _RelativePose:
    translation_m: Vec3
    orientation_xyzw: QuaternionXyzw


def _finite_vector(values: Sequence[float], length: int) -> bool:
    return len(values) == length and all(math.isfinite(float(value)) for value in values)


def _quat_conjugate(quaternion: QuaternionXyzw) -> QuaternionXyzw:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quat_multiply(
    left: QuaternionXyzw, right: QuaternionXyzw
) -> QuaternionXyzw:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion: QuaternionXyzw, vector: Vec3) -> Vec3:
    rotated = _quat_multiply(
        _quat_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
        _quat_conjugate(quaternion),
    )
    return rotated[:3]


def _relative_pose(palm: BodyPose, product: BodyPose) -> _RelativePose:
    inverse_palm = _quat_conjugate(palm.orientation_xyzw)
    delta = tuple(
        product.position_m[axis] - palm.position_m[axis] for axis in range(3)
    )
    return _RelativePose(
        _rotate_vector(inverse_palm, delta),
        _quat_multiply(inverse_palm, product.orientation_xyzw),
    )


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def _quaternion_angle(left: QuaternionXyzw, right: QuaternionXyzw) -> float:
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


class PhysicalGraspController:
    """Bounded OrcaHand sequence with contact and secured-lift gates."""

    def __init__(
        self,
        hand_controller: ArticulationController,
        layout: RestockingLayout,
        *,
        feedback: PhysicalFeedback | None = None,
        arm_lift: ArmLiftPort | None = None,
        limits: GraspLimits | None = None,
    ):
        validate_restocking_layout(layout)
        if layout.product.asset_key != PASTA_BOX_ASSET_KEY:
            raise ValueError("R4 controller is restricted to the pasta_box product")
        if layout.product.dimensions_m != PASTA_BOX_DIMENSIONS_M:
            raise ValueError("R4 pasta_box dimensions differ from the layout contract")
        self.hand_controller = hand_controller
        self.layout = layout
        self.feedback = feedback or UnavailablePhysicalFeedback()
        self.arm_lift = arm_lift or UnavailableArmLiftPort()
        self.limits = limits or GraspLimits()
        self.phase = GraspPhase.IDLE
        self.failure: GraspFailure | None = None
        self._phase_observations = 0
        self._missing_feedback = 0
        self._contact_confirmations = 0
        self._lift_confirmations = 0
        self._last_timestamp_s: float | None = None
        self._grasp_product_pose: BodyPose | None = None
        self._grasp_palm_pose: BodyPose | None = None
        self._grasp_relative_pose: _RelativePose | None = None

        represented = set(hand_controller.spec.canonical_dof_order)
        if set(HAND_DOF_NAMES) - represented:
            raise ValueError("production articulation does not contain every OrcaHand finger DOF")
        for targets in (OPEN_TARGETS, PRESHAPE_TARGETS, CLOSE_TARGETS):
            if set(targets) != set(HAND_DOF_NAMES):
                raise ValueError("every hand pose must target exactly the represented finger DOFs")
            hand_controller.spec.validate_targets(targets)

    @property
    def status(self) -> GraspStatus:
        return GraspStatus(
            phase=self.phase,
            failure=self.failure,
            phase_observations=self._phase_observations,
            contact_confirmations=self._contact_confirmations,
            lift_confirmations=self._lift_confirmations,
        )

    def start(self) -> GraspStatus:
        if self.phase is not GraspPhase.IDLE:
            raise RuntimeError("physical grasp sequence can only be started once")
        self.phase = GraspPhase.OPENING
        self._command_hand(OPEN_TARGETS)
        return self.status

    def step(self) -> GraspStatus:
        if self.phase in (GraspPhase.IDLE, GraspPhase.COMPLETE, GraspPhase.FAILED):
            return self.status
        try:
            observation = self.feedback.read_observation()
        except Exception:
            self._fail(GraspFailure.FEEDBACK_UNAVAILABLE)
            return self.status
        if observation is None:
            self._missing_feedback += 1
            self._phase_observations += 1
            if self._phase_observations >= self.limits.maximum_observations_per_phase:
                self._fail(GraspFailure.FEEDBACK_UNAVAILABLE)
            return self.status
        self._missing_feedback = 0
        if not self._observation_is_valid(observation):
            self._fail(GraspFailure.INVALID_FEEDBACK)
            return self.status

        self._last_timestamp_s = observation.timestamp_s
        self._phase_observations += 1
        if self.phase is GraspPhase.OPENING:
            if self._joints_reached(observation, OPEN_TARGETS):
                self._enter_hand_phase(GraspPhase.PRESHAPING, PRESHAPE_TARGETS)
            elif self._phase_timed_out():
                self._fail(GraspFailure.HAND_TARGET_TIMEOUT)
        elif self.phase is GraspPhase.PRESHAPING:
            if self._joints_reached(observation, PRESHAPE_TARGETS):
                self._enter_hand_phase(GraspPhase.CLOSING, CLOSE_TARGETS)
            elif self._phase_timed_out():
                self._fail(GraspFailure.HAND_TARGET_TIMEOUT)
        elif self.phase is GraspPhase.CLOSING:
            self._step_closing(observation)
        elif self.phase is GraspPhase.LIFTING:
            self._step_lifting(observation)
        return self.status

    def _command_hand(self, targets: Mapping[str, float]) -> None:
        commanded = self.hand_controller.command_joint_positions(dict(targets))
        if set(commanded) != set(HAND_DOF_NAMES) or len(commanded) != len(HAND_DOF_NAMES):
            raise RuntimeError("hand controller did not command exactly the OrcaHand finger DOFs")

    def _enter_hand_phase(
        self, phase: GraspPhase, targets: Mapping[str, float]
    ) -> None:
        self.phase = phase
        self._phase_observations = 0
        self._contact_confirmations = 0
        self._command_hand(targets)

    def _phase_timed_out(self) -> bool:
        return self._phase_observations >= self.limits.maximum_observations_per_phase

    def _fail(self, reason: GraspFailure) -> None:
        self.phase = GraspPhase.FAILED
        self.failure = reason

    def _observation_is_valid(self, observation: GraspObservation) -> bool:
        try:
            return self._observation_values_are_valid(observation)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return False

    def _observation_values_are_valid(self, observation: GraspObservation) -> bool:
        if not math.isfinite(observation.timestamp_s):
            return False
        if (
            self._last_timestamp_s is not None
            and observation.timestamp_s <= self._last_timestamp_s
        ):
            return False
        if observation.product_prim_path != self.layout.product.rigid_body_prim_path:
            return False
        for pose in (observation.palm_pose_world, observation.product_pose_world):
            if not _finite_vector(pose.position_m, 3) or not _finite_vector(
                pose.orientation_xyzw, 4
            ):
                return False
            norm = math.sqrt(sum(value * value for value in pose.orientation_xyzw))
            if not math.isclose(norm, 1.0, abs_tol=1e-5):
                return False
        for name in HAND_DOF_NAMES:
            value = float(observation.hand_joint_positions_rad[name])
            if not math.isfinite(value):
                return False
        for contact in observation.contacts:
            if not contact.body0_prim_path or not contact.body1_prim_path:
                return False
            if not math.isfinite(contact.normal_force_n) or contact.normal_force_n < 0.0:
                return False
        return True

    def _joints_reached(
        self, observation: GraspObservation, targets: Mapping[str, float]
    ) -> bool:
        return all(
            abs(float(observation.hand_joint_positions_rad[name]) - target)
            <= self.limits.joint_tolerance_rad
            for name, target in targets.items()
        )

    def _contact_has_product(self, contact: ContactPair) -> bool:
        product_paths = {
            self.layout.product.rigid_body_prim_path,
            self.layout.product.collider_prim_path,
        }
        return bool(product_paths.intersection((contact.body0_prim_path, contact.body1_prim_path)))

    def _contact_other_path(self, contact: ContactPair) -> str | None:
        product_paths = {
            self.layout.product.rigid_body_prim_path,
            self.layout.product.collider_prim_path,
        }
        if contact.body0_prim_path in product_paths:
            return contact.body1_prim_path
        if contact.body1_prim_path in product_paths:
            return contact.body0_prim_path
        return None

    def _two_sided_product_contact(self, observation: GraspObservation) -> bool:
        active_links = {
            contact.robot_link_name
            for contact in observation.contacts
            if self._contact_has_product(contact)
            and self._contact_other_path(contact)
            != self.layout.pickup_support.fixture.prim_path
            and contact.normal_force_n >= self.limits.minimum_contact_force_n
        }
        return bool(active_links.intersection(THUMB_CONTACT_LINKS)) and bool(
            active_links.intersection(OPPOSING_CONTACT_LINKS)
        )

    def _source_support_contact(self, observation: GraspObservation) -> bool:
        source_path = self.layout.pickup_support.fixture.prim_path
        return any(
            self._contact_other_path(contact) == source_path
            and contact.robot_link_name is None
            and contact.normal_force_n >= self.limits.minimum_contact_force_n
            for contact in observation.contacts
        )

    def _source_pose_is_valid(self, observation: GraspObservation) -> bool:
        expected = self.layout.product.source_reset_pose.position_m
        return _distance(observation.product_pose_world.position_m, expected) <= (
            self.limits.source_pose_tolerance_m
        )

    def _step_closing(self, observation: GraspObservation) -> None:
        if self._two_sided_product_contact(observation):
            if not self._source_support_contact(observation) or not self._source_pose_is_valid(
                observation
            ):
                self._fail(GraspFailure.SOURCE_STATE_INVALID)
                return
            self._contact_confirmations += 1
        else:
            self._contact_confirmations = 0

        if self._contact_confirmations >= self.limits.contact_confirmation_samples:
            self._grasp_product_pose = observation.product_pose_world
            self._grasp_palm_pose = observation.palm_pose_world
            self._grasp_relative_pose = _relative_pose(
                observation.palm_pose_world, observation.product_pose_world
            )
            request = LiftRequest(
                displacement_m=self.limits.requested_lift_m,
                maximum_duration_s=self.limits.maximum_lift_duration_s,
                product_prim_path=self.layout.product.rigid_body_prim_path,
            )
            try:
                accepted = self.arm_lift.request_lift(request)
            except Exception:
                accepted = False
            if accepted is not True:
                self._fail(GraspFailure.LIFT_REQUEST_REJECTED)
                return
            self.phase = GraspPhase.LIFTING
            self._phase_observations = 0
            self._lift_confirmations = 0
            return
        if self._phase_timed_out():
            self._fail(GraspFailure.CONTACT_NOT_VERIFIED)

    def _step_lifting(self, observation: GraspObservation) -> None:
        if not self._two_sided_product_contact(observation):
            self._fail(GraspFailure.OBJECT_DROPPED)
            return
        if self._relative_grasp_was_lost(observation):
            self._fail(GraspFailure.OBJECT_DROPPED)
            return

        product_lift = observation.product_pose_world.position_m[2] - (
            self._grasp_product_pose.position_m[2]  # type: ignore[union-attr]
        )
        palm_lift = observation.palm_pose_world.position_m[2] - (
            self._grasp_palm_pose.position_m[2]  # type: ignore[union-attr]
        )
        left_support = not self._source_support_contact(observation)
        lift_is_measured = (
            left_support
            and product_lift >= self.limits.minimum_verified_lift_m
            and palm_lift >= self.limits.minimum_verified_lift_m
        )
        if lift_is_measured:
            self._lift_confirmations += 1
        else:
            self._lift_confirmations = 0
        if self._lift_confirmations >= self.limits.lift_confirmation_samples:
            self.phase = GraspPhase.COMPLETE
            return
        if self._phase_timed_out():
            self._fail(GraspFailure.LIFT_NOT_VERIFIED)

    def _relative_grasp_was_lost(self, observation: GraspObservation) -> bool:
        baseline = self._grasp_relative_pose
        if baseline is None:
            return True
        current = _relative_pose(observation.palm_pose_world, observation.product_pose_world)
        return (
            _distance(current.translation_m, baseline.translation_m)
            > self.limits.maximum_relative_translation_drift_m
            or _quaternion_angle(current.orientation_xyzw, baseline.orientation_xyzw)
            > self.limits.maximum_relative_rotation_drift_rad
        )


__all__ = [
    "ArmLiftPort",
    "BodyPose",
    "CLOSE_TARGETS",
    "ContactPair",
    "GraspFailure",
    "GraspLimits",
    "GraspObservation",
    "GraspPhase",
    "GraspStatus",
    "HAND_DOF_NAMES",
    "LiftRequest",
    "OPEN_TARGETS",
    "OPPOSING_CONTACT_LINKS",
    "PRESHAPE_TARGETS",
    "PhysicalFeedback",
    "PhysicalGraspController",
    "THUMB_CONTACT_LINKS",
    "UnavailableArmLiftPort",
    "UnavailablePhysicalFeedback",
]
