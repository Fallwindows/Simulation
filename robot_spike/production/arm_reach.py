"""CPU-safe right-arm kinematics and bounded R3 reach planning.

The solver is deliberately independent of Isaac Sim.  It reads the validated
production URDF, computes poses from ``waist_yaw_link`` to the OrcaHand
``right_palm`` frame, and emits only name-bound position targets for the six
right-arm DOFs.  Physics execution and physical reach remain runtime gates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Protocol, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .model import JointTargetError, ModelValidationError, ProductionRobotSpec


KINEMATIC_BASE_FRAME = "waist_yaw_link"
TOOL_FRAME = "right_palm"
ARM_DOF_NAMES = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist",
)
EXPECTED_CHAIN_JOINTS = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "asimov_to_orcahand_mount",
    "right_wrist",
    "right_wrist_offset",
)


class ArmKinematicsError(ValueError):
    """The requested arm pose or kinematic input is invalid."""


class IKConvergenceError(ArmKinematicsError):
    """Bounded IK could not satisfy both pose tolerances."""

    def __init__(
        self,
        message: str,
        *,
        iterations: int,
        position_error_m: float,
        orientation_error_rad: float,
    ):
        super().__init__(message)
        self.iterations = iterations
        self.position_error_m = position_error_m
        self.orientation_error_rad = orientation_error_rad


class PositionTargetController(Protocol):
    def command_joint_positions(self, targets: dict[str, float]) -> tuple[str, ...]: ...


def _finite_vector(values: Sequence[object], length: int, label: str) -> np.ndarray:
    if isinstance(values, (str, bytes)):
        raise ArmKinematicsError(f"{label} must contain {length} numeric values")
    try:
        sequence = tuple(values)
    except TypeError as exc:
        raise ArmKinematicsError(
            f"{label} must contain {length} numeric values"
        ) from exc
    if len(sequence) != length:
        raise ArmKinematicsError(f"{label} must contain {length} numeric values")
    try:
        vector = np.asarray(sequence, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ArmKinematicsError(f"{label} must contain numeric values") from exc
    if vector.shape != (length,) or not np.isfinite(vector).all():
        raise ArmKinematicsError(f"{label} must contain {length} finite numeric values")
    return vector


@dataclass(frozen=True)
class ToolPose:
    """A right-palm pose expressed in ``waist_yaw_link`` coordinates."""

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        position = _finite_vector(self.position_m, 3, "tool position")
        quaternion = _finite_vector(self.orientation_xyzw, 4, "tool quaternion")
        norm = float(np.linalg.norm(quaternion))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ArmKinematicsError(
                f"tool quaternion must have unit norm within 1e-6, got {norm:.9g}"
            )
        object.__setattr__(self, "position_m", tuple(float(value) for value in position))
        object.__setattr__(
            self, "orientation_xyzw", tuple(float(value) for value in quaternion)
        )


@dataclass(frozen=True)
class _ChainJoint:
    name: str
    joint_type: str
    origin_position_m: np.ndarray
    origin_rotation: np.ndarray
    axis: np.ndarray | None


@dataclass(frozen=True)
class IKResult:
    positions_rad: tuple[tuple[str, float], ...]
    iterations: int
    position_error_m: float
    orientation_error_rad: float

    def as_mapping(self) -> dict[str, float]:
        return dict(self.positions_rad)


@dataclass(frozen=True)
class ReachWaypoint:
    phase: str
    step_index: int
    positions_rad: tuple[tuple[str, float], ...]

    def as_mapping(self) -> dict[str, float]:
        return dict(self.positions_rad)


@dataclass(frozen=True)
class ReachPhase:
    name: str
    target: ToolPose
    ik_result: IKResult
    waypoints: tuple[ReachWaypoint, ...]


@dataclass(frozen=True)
class ReachPlan:
    base_frame: str
    tool_frame: str
    arm_dof_names: tuple[str, ...]
    phases: tuple[ReachPhase, ...]

    @property
    def waypoints(self) -> tuple[ReachWaypoint, ...]:
        return tuple(waypoint for phase in self.phases for waypoint in phase.waypoints)


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        (
            (c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s),
            (y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s),
            (z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c),
        ),
        dtype=float,
    )


def _matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
        w = 0.25 * scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = math.sqrt(max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = math.sqrt(max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale
    quaternion = np.array((x, y, z, w), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return tuple(float(value) for value in quaternion)


def _quaternion_matrix_xyzw(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    x, y, z, w = _matrix_to_quaternion_xyzw(rotation)
    vector = np.array((x, y, z), dtype=float)
    sine_half = float(np.linalg.norm(vector))
    if sine_half < 1e-12:
        return np.zeros(3, dtype=float)
    angle = 2.0 * math.atan2(sine_half, w)
    return vector * (angle / sine_half)


class RightArmKinematics:
    """Validated FK and bounded damped-least-squares IK for the right palm."""

    def __init__(self, spec: ProductionRobotSpec):
        self.spec = spec
        self._chain = self._load_chain()
        if tuple(joint.name for joint in self._chain) != EXPECTED_CHAIN_JOINTS:
            raise ModelValidationError(
                "right-palm URDF chain changed; expected "
                f"{EXPECTED_CHAIN_JOINTS}, got {tuple(joint.name for joint in self._chain)}"
            )
        movable = tuple(joint.name for joint in self._chain if joint.joint_type != "fixed")
        if movable != ARM_DOF_NAMES:
            raise ModelValidationError(
                f"right-palm movable chain changed; expected {ARM_DOF_NAMES}, got {movable}"
            )

    def _load_chain(self) -> tuple[_ChainJoint, ...]:
        try:
            robot = ET.parse(self.spec.model.path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise ModelValidationError(f"cannot parse arm URDF: {exc}") from exc
        by_child: dict[str, ET.Element] = {}
        for joint in robot.findall("joint"):
            child = joint.find("child")
            if child is not None:
                by_child[child.attrib["link"]] = joint
        elements: list[ET.Element] = []
        link = TOOL_FRAME
        while link != KINEMATIC_BASE_FRAME:
            joint = by_child.get(link)
            if joint is None:
                raise ModelValidationError(
                    f"{TOOL_FRAME!r} is not descended from {KINEMATIC_BASE_FRAME!r}"
                )
            elements.append(joint)
            parent = joint.find("parent")
            if parent is None:
                raise ModelValidationError(f"joint {joint.attrib.get('name')!r} has no parent")
            link = parent.attrib["link"]
        elements.reverse()

        result: list[_ChainJoint] = []
        for joint in elements:
            name = joint.attrib["name"]
            joint_type = joint.attrib["type"]
            origin = joint.find("origin")
            xyz = _finite_vector(
                (origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0").split(),
                3,
                f"{name}.origin.xyz",
            )
            rpy = _finite_vector(
                (origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0").split(),
                3,
                f"{name}.origin.rpy",
            )
            axis_value: np.ndarray | None = None
            if joint_type != "fixed":
                axis = joint.find("axis")
                axis_value = _finite_vector(
                    (axis.attrib.get("xyz", "1 0 0") if axis is not None else "1 0 0").split(),
                    3,
                    f"{name}.axis",
                )
                norm = float(np.linalg.norm(axis_value))
                if norm <= 1e-12:
                    raise ModelValidationError(f"joint {name!r} has a zero axis")
                axis_value = axis_value / norm
            result.append(
                _ChainJoint(name, joint_type, xyz, _rpy_matrix(rpy), axis_value)
            )
        return tuple(result)

    def _validated_positions(self, positions_rad: Mapping[str, float]) -> np.ndarray:
        if not isinstance(positions_rad, Mapping):
            raise ArmKinematicsError("arm joint positions must be a name-to-value mapping")
        actual = set(positions_rad)
        expected = set(ARM_DOF_NAMES)
        if actual != expected:
            raise ArmKinematicsError(
                "arm joint map must contain exactly the right-arm DOFs; "
                f"missing={sorted(expected - actual)}, extra={sorted(actual - expected)}"
            )
        try:
            checked = self.spec.validate_targets(positions_rad)
        except JointTargetError as exc:
            raise ArmKinematicsError(str(exc)) from exc
        return np.array([checked[name] for name in ARM_DOF_NAMES], dtype=float)

    def _mapping(self, values: Sequence[float]) -> dict[str, float]:
        return {name: float(value) for name, value in zip(ARM_DOF_NAMES, values)}

    def _forward_matrix(self, values: np.ndarray) -> np.ndarray:
        joint_values = dict(zip(ARM_DOF_NAMES, values))
        transform = np.eye(4, dtype=float)
        for joint in self._chain:
            origin = np.eye(4, dtype=float)
            origin[:3, :3] = joint.origin_rotation
            origin[:3, 3] = joint.origin_position_m
            transform = transform @ origin
            if joint.joint_type != "fixed":
                rotation = np.eye(4, dtype=float)
                rotation[:3, :3] = _axis_angle_matrix(
                    joint.axis, float(joint_values[joint.name])
                )
                transform = transform @ rotation
        return transform

    def forward(self, positions_rad: Mapping[str, float]) -> ToolPose:
        values = self._validated_positions(positions_rad)
        transform = self._forward_matrix(values)
        return ToolPose(
            tuple(float(value) for value in transform[:3, 3]),
            _matrix_to_quaternion_xyzw(transform[:3, :3]),
        )

    def solve(
        self,
        target: ToolPose,
        seed_positions_rad: Mapping[str, float],
        *,
        max_iterations: int = 160,
        position_tolerance_m: float = 2e-4,
        orientation_tolerance_rad: float = 2e-3,
        damping: float = 0.01,
        maximum_iteration_step_rad: float = 0.12,
        orientation_weight_m_per_rad: float = 0.15,
    ) -> IKResult:
        for label, value in (
            ("position_tolerance_m", position_tolerance_m),
            ("orientation_tolerance_rad", orientation_tolerance_rad),
            ("damping", damping),
            ("maximum_iteration_step_rad", maximum_iteration_step_rad),
            ("orientation_weight_m_per_rad", orientation_weight_m_per_rad),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ArmKinematicsError(f"{label} must be finite and positive")
        if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations <= 0:
            raise ArmKinematicsError("max_iterations must be a positive integer")

        values = self._validated_positions(seed_positions_rad)
        lower = np.array([self.spec.model.joint_limits[name].lower for name in ARM_DOF_NAMES])
        upper = np.array([self.spec.model.joint_limits[name].upper for name in ARM_DOF_NAMES])
        target_position = np.asarray(target.position_m, dtype=float)
        target_rotation = _quaternion_matrix_xyzw(target.orientation_xyzw)

        def errors(candidate: np.ndarray) -> tuple[np.ndarray, float, float, float]:
            current = self._forward_matrix(candidate)
            position_error = target_position - current[:3, 3]
            rotation_error = _rotation_vector(target_rotation @ current[:3, :3].T)
            position_norm = float(np.linalg.norm(position_error))
            orientation_norm = float(np.linalg.norm(rotation_error))
            error = np.concatenate((position_error, rotation_error))
            weighted_error = error.copy()
            weighted_error[3:] *= orientation_weight_m_per_rad
            # DLS computes a Gauss-Newton step for this weighted least-squares
            # objective.  The line search must use the same objective or it can
            # reject a valid descent step near the convergence tolerances.
            merit = 0.5 * float(weighted_error @ weighted_error)
            return error, position_norm, orientation_norm, merit

        for iteration in range(max_iterations + 1):
            error, position_error, orientation_error, merit = errors(values)
            if (
                position_error <= position_tolerance_m
                and orientation_error <= orientation_tolerance_rad
            ):
                return IKResult(
                    tuple((name, float(value)) for name, value in zip(ARM_DOF_NAMES, values)),
                    iteration,
                    position_error,
                    orientation_error,
                )
            if iteration == max_iterations:
                break

            jacobian = np.zeros((6, len(ARM_DOF_NAMES)), dtype=float)
            epsilon = 1e-6
            for index in range(len(ARM_DOF_NAMES)):
                plus = values.copy()
                minus = values.copy()
                plus[index] = min(upper[index], plus[index] + epsilon)
                minus[index] = max(lower[index], minus[index] - epsilon)
                span = plus[index] - minus[index]
                if span <= 0.0:
                    continue
                plus_transform = self._forward_matrix(plus)
                minus_transform = self._forward_matrix(minus)
                jacobian[:3, index] = (
                    plus_transform[:3, 3] - minus_transform[:3, 3]
                ) / span
                jacobian[3:, index] = _rotation_vector(
                    plus_transform[:3, :3] @ minus_transform[:3, :3].T
                ) / span

            weighted_jacobian = jacobian.copy()
            weighted_error = error.copy()
            weighted_jacobian[3:, :] *= orientation_weight_m_per_rad
            weighted_error[3:] *= orientation_weight_m_per_rad
            normal = weighted_jacobian @ weighted_jacobian.T + (damping * damping) * np.eye(6)
            try:
                delta = weighted_jacobian.T @ np.linalg.solve(normal, weighted_error)
            except np.linalg.LinAlgError as exc:
                raise IKConvergenceError(
                    "damped IK linear solve failed",
                    iterations=iteration,
                    position_error_m=position_error,
                    orientation_error_rad=orientation_error,
                ) from exc
            maximum_delta = float(np.max(np.abs(delta)))
            if maximum_delta > maximum_iteration_step_rad:
                delta *= maximum_iteration_step_rad / maximum_delta

            accepted = False
            scale = 1.0
            for _ in range(10):
                candidate = np.clip(values + scale * delta, lower, upper)
                _, _, _, candidate_merit = errors(candidate)
                if candidate_merit < merit:
                    values = candidate
                    accepted = True
                    break
                scale *= 0.5
            if not accepted:
                break

        _, position_error, orientation_error, _ = errors(values)
        raise IKConvergenceError(
            "right-palm target did not converge within bounded IK; target may be unreachable "
            "from the supplied seed under the production joint limits "
            f"(position error {position_error:.9g} m, orientation error "
            f"{orientation_error:.9g} rad)",
            iterations=min(iteration, max_iterations),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
        )


class ArmReachPlanner:
    """Plan bounded pre-grasp and pre-place right-palm position targets."""

    def __init__(
        self,
        kinematics: RightArmKinematics,
        *,
        maximum_joint_step_rad: float = 0.04,
        command_period_s: float = 0.05,
        velocity_limit_scale: float = 0.25,
    ):
        for label, value in (
            ("maximum_joint_step_rad", maximum_joint_step_rad),
            ("command_period_s", command_period_s),
            ("velocity_limit_scale", velocity_limit_scale),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ArmKinematicsError(f"{label} must be finite and positive")
        if velocity_limit_scale > 1.0:
            raise ArmKinematicsError("velocity_limit_scale must be at most 1.0")
        self.kinematics = kinematics
        self.maximum_joint_step_rad = maximum_joint_step_rad
        self.command_period_s = command_period_s
        self.velocity_limit_scale = velocity_limit_scale

    def maximum_step_for_joint(self, name: str) -> float:
        if name not in ARM_DOF_NAMES:
            raise ArmKinematicsError(f"{name!r} is not a right-arm DOF")
        velocity = self.kinematics.spec.model.joint_limits[name].velocity
        return min(
            self.maximum_joint_step_rad,
            velocity * self.command_period_s * self.velocity_limit_scale,
        )

    def _interpolate(
        self, start: Mapping[str, float], goal: Mapping[str, float], phase: str
    ) -> tuple[ReachWaypoint, ...]:
        start_values = self.kinematics._validated_positions(start)
        goal_values = self.kinematics._validated_positions(goal)
        delta = goal_values - start_values
        steps = max(
            1,
            max(
                math.ceil(abs(delta[index]) / self.maximum_step_for_joint(name))
                for index, name in enumerate(ARM_DOF_NAMES)
            ),
        )
        waypoints: list[ReachWaypoint] = []
        for step in range(1, steps + 1):
            values = start_values + delta * (step / steps)
            mapping = self.kinematics._mapping(values)
            self.kinematics._validated_positions(mapping)
            waypoints.append(
                ReachWaypoint(
                    phase,
                    step,
                    tuple((name, mapping[name]) for name in ARM_DOF_NAMES),
                )
            )
        return tuple(waypoints)

    def plan_pregrasp_preplacement(
        self,
        initial_positions_rad: Mapping[str, float],
        pre_grasp: ToolPose,
        pre_place: ToolPose,
    ) -> ReachPlan:
        current = self.kinematics._mapping(
            self.kinematics._validated_positions(initial_positions_rad)
        )
        phases: list[ReachPhase] = []
        for phase_name, target in (("pre_grasp", pre_grasp), ("pre_place", pre_place)):
            try:
                result = self.kinematics.solve(target, current)
            except IKConvergenceError as exc:
                raise IKConvergenceError(
                    f"{phase_name} planning failed: {exc}",
                    iterations=exc.iterations,
                    position_error_m=exc.position_error_m,
                    orientation_error_rad=exc.orientation_error_rad,
                ) from exc
            goal = result.as_mapping()
            waypoints = self._interpolate(current, goal, phase_name)
            phases.append(ReachPhase(phase_name, target, result, waypoints))
            current = goal
        return ReachPlan(
            KINEMATIC_BASE_FRAME,
            TOOL_FRAME,
            ARM_DOF_NAMES,
            tuple(phases),
        )


def command_reach_waypoint(
    controller: PositionTargetController, waypoint: ReachWaypoint
) -> tuple[str, ...]:
    """Send one arm-only waypoint through the existing drive-target controller."""

    targets = waypoint.as_mapping()
    if tuple(targets) != ARM_DOF_NAMES:
        raise ArmKinematicsError("reach waypoint does not contain the exact right-arm DOF order")
    commanded = controller.command_joint_positions(targets)
    if tuple(commanded) != ARM_DOF_NAMES:
        raise ArmKinematicsError(
            f"controller reported unexpected commanded joints: {tuple(commanded)}"
        )
    return tuple(commanded)
