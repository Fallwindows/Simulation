"""Deterministic, feedback-gated locomotion policy for the Asimov biped.

This module is deliberately CPU safe: it contains no Isaac or PhysX imports.
The runtime-facing controller accepts an injected feedback source and sends
normal-motion commands only through
``ArticulationController.command_joint_positions``.  It never writes the free
root pose or joint state directly.

The gait is intentionally conservative and remains a runtime candidate until
it is tuned and validated with the imported robot under gravity and contact.
The swing-foot reference and the compact joint-space gait template are useful
control targets; CPU evaluation cannot prove that the physical foot tracks the
reference or that the free-root robot balances.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping, Protocol, Sequence

from .model import JointTargetError, ProductionRobotSpec
from .runtime import ArticulationController


Vec3 = tuple[float, float, float]

LEG_JOINTS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)


class LocomotionError(ValueError):
    """A plan, configuration, or measured feedback sample is unsafe."""


class FootSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"

    @property
    def opposite(self) -> "FootSide":
        return FootSide.RIGHT if self is FootSide.LEFT else FootSide.LEFT

    @property
    def pitch_axis_sign(self) -> float:
        """Map a physical sagittal angle to this URDF's mirrored joint sign."""

        return 1.0 if self is FootSide.LEFT else -1.0

    @property
    def lateral_sign(self) -> float:
        return 1.0 if self is FootSide.LEFT else -1.0


class GaitPhase(str, Enum):
    DOUBLE_SUPPORT = "double_support"
    LEFT_SWING = "left_swing"
    RIGHT_SWING = "right_swing"

    @property
    def swing_side(self) -> FootSide | None:
        if self is GaitPhase.LEFT_SWING:
            return FootSide.LEFT
        if self is GaitPhase.RIGHT_SWING:
            return FootSide.RIGHT
        return None


class LocomotionState(str, Enum):
    IDLE = "idle"
    ACTIVE = "active"
    STOPPING = "stopping"
    STOPPED = "stopped"
    DOCKING = "docking"
    DOCKED = "docked"
    FAULT = "fault"


@dataclass(frozen=True)
class PlanarPose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class FootPose:
    """World pose of a sole/contact reference point."""

    position_m: Vec3
    yaw_rad: float


@dataclass(frozen=True)
class FootFeedback:
    pose: FootPose
    in_contact: bool


@dataclass(frozen=True)
class LocomotionFeedback:
    """Measured state required by the gait and balance policies.

    Root velocities and tilt are expressed in the root/body frame;
    ``root_height_m`` is the world-frame root Z coordinate.  Foot,
    center-of-mass, and support-center positions are in the world frame.  A
    positive support margin means the projected COM is inside the active
    support polygon; a negative value means it is outside.
    """

    timestamp_s: float
    root_pose: PlanarPose
    root_height_m: float
    root_tilt_roll_pitch_rad: tuple[float, float]
    root_linear_velocity_body_mps: Vec3
    root_angular_velocity_body_rps: Vec3
    left_foot: FootFeedback
    right_foot: FootFeedback
    joint_position_rad: Mapping[str, float]
    joint_velocity_rad_s: Mapping[str, float]
    com_position_world_m: Vec3
    support_center_world_m: Vec3
    support_margin_m: float

    def foot(self, side: FootSide) -> FootFeedback:
        return self.left_foot if side is FootSide.LEFT else self.right_foot


class LocomotionFeedbackSource(Protocol):
    """Runtime adapter implemented by the Isaac/PhysX integration owner."""

    def read_feedback(self) -> LocomotionFeedback: ...


@dataclass(frozen=True)
class Footstep:
    side: FootSide
    body_target: PlanarPose
    foot_target: FootPose


@dataclass(frozen=True)
class GaitConfig:
    """Low-energy defaults with hard bounds for the first runtime tuning pass."""

    max_step_length_m: float = 0.060
    max_step_yaw_rad: float = math.radians(8.0)
    nominal_speed_mps: float = 0.050
    stance_width_m: float = 0.135
    swing_clearance_m: float = 0.025
    swing_duration_s: float = 0.90
    swing_timeout_s: float = 1.60
    double_support_duration_s: float = 0.30
    double_support_timeout_s: float = 1.20
    minimum_touchdown_progress: float = 0.70
    touchdown_position_tolerance_m: float = 0.045
    touchdown_height_tolerance_m: float = 0.025
    touchdown_yaw_tolerance_rad: float = math.radians(5.0)
    dock_position_tolerance_m: float = 0.050
    dock_yaw_tolerance_rad: float = math.radians(5.0)
    dock_linear_speed_tolerance_mps: float = 0.035
    dock_angular_speed_tolerance_rps: float = 0.10
    dock_settle_duration_s: float = 0.40
    dock_timeout_s: float = 3.0
    stop_settle_duration_s: float = 0.30
    stop_timeout_s: float = 2.0
    neutral_knee_flexion_rad: float = 0.18
    swing_knee_flexion_rad: float = 0.34
    landing_hip_pitch_rad: float = 0.075
    landing_hip_roll_rad: float = 0.040
    joint_limit_margin_rad: float = 0.005
    maximum_target_error_rad: float = 0.30
    # The imported force drives use Kp=120 and Kd=12.  Kd/Kp=0.1 s adds one
    # matching, velocity-aware damping term through the position target while
    # the measured target-error gate continues to bound every command.
    joint_velocity_damping_s: float = 0.100
    maximum_balance_correction_rad: float = 0.045
    maximum_root_tilt_rad: float = math.radians(12.0)
    maximum_root_angular_speed_rps: float = 1.0
    maximum_root_linear_speed_mps: float = 0.35
    minimum_root_clearance_m: float = 0.45
    maximum_root_clearance_m: float = 0.80
    minimum_support_margin_m: float = -0.015
    maximum_plan_steps: int = 512

    def __post_init__(self) -> None:
        positive_values = (
            self.max_step_length_m,
            self.max_step_yaw_rad,
            self.nominal_speed_mps,
            self.stance_width_m,
            self.swing_clearance_m,
            self.swing_duration_s,
            self.swing_timeout_s,
            self.double_support_duration_s,
            self.double_support_timeout_s,
            self.minimum_touchdown_progress,
            self.touchdown_position_tolerance_m,
            self.touchdown_height_tolerance_m,
            self.touchdown_yaw_tolerance_rad,
            self.dock_position_tolerance_m,
            self.dock_yaw_tolerance_rad,
            self.dock_linear_speed_tolerance_mps,
            self.dock_angular_speed_tolerance_rps,
            self.dock_settle_duration_s,
            self.dock_timeout_s,
            self.stop_settle_duration_s,
            self.stop_timeout_s,
            self.swing_knee_flexion_rad,
            self.landing_hip_pitch_rad,
            self.landing_hip_roll_rad,
            self.joint_limit_margin_rad,
            self.maximum_target_error_rad,
            self.joint_velocity_damping_s,
            self.maximum_balance_correction_rad,
            self.maximum_root_tilt_rad,
            self.maximum_root_angular_speed_rps,
            self.maximum_root_linear_speed_mps,
            self.minimum_root_clearance_m,
            self.maximum_root_clearance_m,
        )
        finite_values = (
            *positive_values,
            self.neutral_knee_flexion_rad,
            self.minimum_support_margin_m,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise LocomotionError("gait configuration values must be finite")
        if any(value <= 0.0 for value in positive_values):
            raise LocomotionError("positive gait configuration values must be greater than zero")
        if not 0.0 < self.minimum_touchdown_progress <= 1.0:
            raise LocomotionError("minimum_touchdown_progress must be in (0, 1]")
        if self.max_step_length_m > 0.12:
            raise LocomotionError("max_step_length_m exceeds the 0.12 m safety cap")
        if self.max_step_yaw_rad > math.radians(15.0):
            raise LocomotionError("max_step_yaw_rad exceeds the 15 degree safety cap")
        if self.nominal_speed_mps > 0.10:
            raise LocomotionError("nominal_speed_mps exceeds the 0.10 m/s safety cap")
        if self.swing_timeout_s <= self.swing_duration_s:
            raise LocomotionError("swing timeout must exceed nominal swing duration")
        if self.double_support_timeout_s <= self.double_support_duration_s:
            raise LocomotionError("double-support timeout must exceed its dwell duration")
        if self.swing_knee_flexion_rad < self.neutral_knee_flexion_rad:
            raise LocomotionError("swing knee flexion must not be below neutral flexion")
        if self.neutral_knee_flexion_rad < 0.0:
            raise LocomotionError("neutral knee flexion must be nonnegative")
        if self.minimum_root_clearance_m >= self.maximum_root_clearance_m:
            raise LocomotionError("root clearance envelope must have increasing bounds")
        if self.joint_velocity_damping_s > 0.100:
            raise LocomotionError("joint_velocity_damping_s exceeds the 0.100 s safety cap")
        if self.maximum_plan_steps < 1:
            raise LocomotionError("maximum_plan_steps must be positive")


def _finite_sequence(values: Sequence[float], expected: int, label: str) -> None:
    if len(values) != expected or not all(math.isfinite(float(value)) for value in values):
        raise LocomotionError(f"{label} must contain {expected} finite values")


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _smoothstep5(progress: float) -> float:
    value = min(1.0, max(0.0, progress))
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def swing_foot_trajectory(
    start: FootPose,
    target: FootPose,
    progress: float,
    clearance_m: float,
) -> FootPose:
    """Return a C1 endpoint-smooth flat-ground swing reference.

    The quintic horizontal/yaw blend has zero endpoint velocity and
    acceleration.  The quartic vertical bump also has zero endpoint velocity.
    ``clearance_m`` is added above the interpolated endpoint height.
    """

    if not math.isfinite(progress) or not math.isfinite(clearance_m) or clearance_m < 0.0:
        raise LocomotionError("swing progress and clearance must be finite and nonnegative")
    _finite_sequence(start.position_m, 3, "swing start position")
    _finite_sequence(target.position_m, 3, "swing target position")
    if not math.isfinite(start.yaw_rad) or not math.isfinite(target.yaw_rad):
        raise LocomotionError("swing yaw values must be finite")
    u = min(1.0, max(0.0, progress))
    blend = _smoothstep5(u)
    position = tuple(
        start.position_m[index]
        + blend * (target.position_m[index] - start.position_m[index])
        for index in range(3)
    )
    bump = 16.0 * clearance_m * u * u * (1.0 - u) * (1.0 - u)
    position = (position[0], position[1], position[2] + bump)
    yaw = start.yaw_rad + blend * _wrap_angle(target.yaw_rad - start.yaw_rad)
    return FootPose(position, _wrap_angle(yaw))


class WaypointStepPlanner:
    """Convert planar waypoints into bounded alternating flat-ground steps."""

    def __init__(self, config: GaitConfig = GaitConfig()):
        self.config = config

    def plan(
        self,
        start: PlanarPose,
        waypoints: Sequence[PlanarPose],
        *,
        first_swing: FootSide = FootSide.LEFT,
        ground_height_m: float = 0.0,
    ) -> tuple[Footstep, ...]:
        _validate_planar_pose(start, "plan start")
        if not waypoints:
            raise LocomotionError("a route requires at least one waypoint")
        if not math.isfinite(ground_height_m):
            raise LocomotionError("ground height must be finite")
        steps: list[Footstep] = []
        current = start
        side = first_swing
        cycle_time = self.config.swing_duration_s + self.config.double_support_duration_s
        speed_limited_body_increment = self.config.nominal_speed_mps * cycle_time
        foot_radius = self.config.stance_width_m / 2.0
        for waypoint_index, waypoint in enumerate(waypoints):
            _validate_planar_pose(waypoint, f"waypoint {waypoint_index}")
            dx = waypoint.x_m - current.x_m
            dy = waypoint.y_m - current.y_m
            distance = math.hypot(dx, dy)
            yaw_delta = _wrap_angle(waypoint.yaw_rad - current.yaw_rad)
            # A given foot moves every other phase.  Bound the arc length over
            # two body increments so ``max_step_length_m`` applies to the
            # physical swing-foot displacement rather than only the body-plan
            # interpolation increment.
            foot_path_bound = distance + foot_radius * abs(yaw_delta)
            count = max(
                int(math.ceil(distance / speed_limited_body_increment)) if distance else 0,
                int(math.ceil(2.0 * foot_path_bound / self.config.max_step_length_m))
                if foot_path_bound
                else 0,
                int(math.ceil(2.0 * abs(yaw_delta) / self.config.max_step_yaw_rad))
                if yaw_delta
                else 0,
            )
            if count == 0:
                current = waypoint
                continue
            if len(steps) + count > self.config.maximum_plan_steps:
                raise LocomotionError("route exceeds maximum_plan_steps")
            segment_start = current
            for index in range(1, count + 1):
                alpha = index / count
                body = PlanarPose(
                    segment_start.x_m + alpha * dx,
                    segment_start.y_m + alpha * dy,
                    _wrap_angle(segment_start.yaw_rad + alpha * yaw_delta),
                )
                lateral = side.lateral_sign * self.config.stance_width_m / 2.0
                foot_x = body.x_m - math.sin(body.yaw_rad) * lateral
                foot_y = body.y_m + math.cos(body.yaw_rad) * lateral
                steps.append(
                    Footstep(
                        side=side,
                        body_target=body,
                        foot_target=FootPose((foot_x, foot_y, ground_height_m), body.yaw_rad),
                    )
                )
                side = side.opposite
            current = waypoint
        return tuple(steps)


@dataclass(frozen=True)
class BalanceCorrection:
    sagittal_rad: float
    lateral_rad: float
    unsafe_reason: str | None = None


class BalanceFeedbackController:
    """Small bounded feedback correction using measured dynamic state."""

    def __init__(self, config: GaitConfig = GaitConfig()):
        self.config = config

    def evaluate(
        self,
        feedback: LocomotionFeedback,
        *,
        desired_forward_speed_mps: float = 0.0,
    ) -> BalanceCorrection:
        _validate_feedback(feedback)
        roll, pitch = feedback.root_tilt_roll_pitch_rad
        linear = feedback.root_linear_velocity_body_mps
        angular = feedback.root_angular_velocity_body_rps
        reasons: list[str] = []
        if max(abs(roll), abs(pitch)) > self.config.maximum_root_tilt_rad:
            reasons.append("root tilt exceeded limit")
        if max(abs(value) for value in angular) > self.config.maximum_root_angular_speed_rps:
            reasons.append("root angular speed exceeded limit")
        if math.sqrt(sum(value * value for value in linear)) > self.config.maximum_root_linear_speed_mps:
            reasons.append("root linear speed exceeded limit")
        if feedback.support_margin_m < self.config.minimum_support_margin_m:
            reasons.append("COM projection left support margin")
        contact_heights = [
            feedback.foot(side).pose.position_m[2]
            for side in FootSide
            if feedback.foot(side).in_contact
        ]
        support_height = (
            max(contact_heights)
            if contact_heights
            else min(
                feedback.left_foot.pose.position_m[2],
                feedback.right_foot.pose.position_m[2],
            )
        )
        root_clearance = feedback.root_height_m - support_height
        if not (
            self.config.minimum_root_clearance_m
            <= root_clearance
            <= self.config.maximum_root_clearance_m
        ):
            reasons.append(
                "root clearance outside "
                f"[{self.config.minimum_root_clearance_m:.3f}, "
                f"{self.config.maximum_root_clearance_m:.3f}] m envelope"
            )

        dx_world = feedback.com_position_world_m[0] - feedback.support_center_world_m[0]
        dy_world = feedback.com_position_world_m[1] - feedback.support_center_world_m[1]
        cosine = math.cos(feedback.root_pose.yaw_rad)
        sine = math.sin(feedback.root_pose.yaw_rad)
        com_forward = cosine * dx_world + sine * dy_world
        com_lateral = -sine * dx_world + cosine * dy_world
        sagittal = (
            -0.22 * pitch
            - 0.035 * angular[1]
            - 0.10 * (linear[0] - desired_forward_speed_mps)
            - 0.30 * com_forward
        )
        lateral = (
            -0.22 * roll
            - 0.035 * angular[0]
            - 0.10 * linear[1]
            - 0.30 * com_lateral
        )
        limit = self.config.maximum_balance_correction_rad
        return BalanceCorrection(
            sagittal_rad=max(-limit, min(limit, sagittal)),
            lateral_rad=max(-limit, min(limit, lateral)),
            unsafe_reason="; ".join(reasons) if reasons else None,
        )


def symmetric_crouch_targets(
    spec: ProductionRobotSpec,
    *,
    knee_flexion_rad: float = 0.18,
    limit_margin_rad: float = 0.005,
) -> dict[str, float]:
    """Return a symmetric double-support pose with mirrored sagittal signs."""

    if not math.isfinite(knee_flexion_rad) or knee_flexion_rad < 0.0:
        raise LocomotionError("knee flexion must be finite and nonnegative")
    targets: dict[str, float] = {}
    for side in FootSide:
        prefix = side.value
        sign = side.pitch_axis_sign
        physical_hip = -0.5 * knee_flexion_rad
        physical_ankle = -physical_hip - knee_flexion_rad
        targets[f"{prefix}_hip_pitch_joint"] = sign * physical_hip
        targets[f"{prefix}_hip_roll_joint"] = 0.0
        targets[f"{prefix}_hip_yaw_joint"] = 0.0
        targets[f"{prefix}_knee_joint"] = sign * knee_flexion_rad
        targets[f"{prefix}_ankle_pitch_joint"] = sign * physical_ankle
        targets[f"{prefix}_ankle_roll_joint"] = 0.0
    return _limit_targets(spec, targets, limit_margin_rad)


class ConservativeGaitTargetGenerator:
    """Map phase references to the 12 actual Asimov leg joints.

    This is a compact seed policy for Isaac tuning, not a rigid-body dynamics or
    whole-body IK solver.  Physical sagittal angles are mirrored according to
    the opposite left/right pitch and knee axes in the supplied URDF.
    """

    def __init__(self, spec: ProductionRobotSpec, config: GaitConfig = GaitConfig()):
        self.spec = spec
        self.config = config

    def targets(
        self,
        feedback: LocomotionFeedback,
        phase: GaitPhase,
        progress: float,
        correction: BalanceCorrection,
        *,
        step: Footstep | None = None,
        swing_start_pose: FootPose | None = None,
        swing_start_joint_positions: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        _validate_feedback(feedback)
        targets = symmetric_crouch_targets(
            self.spec,
            knee_flexion_rad=self.config.neutral_knee_flexion_rad,
            limit_margin_rad=self.config.joint_limit_margin_rad,
        )
        swing_side = phase.swing_side
        if swing_side is not None:
            if step is None or swing_start_pose is None or swing_start_joint_positions is None:
                raise LocomotionError("swing targets require the active step and captured start state")
            if step.side is not swing_side:
                raise LocomotionError("active step side does not match gait phase")
            u = min(1.0, max(0.0, progress))
            blend = _smoothstep5(u)
            bump = math.sin(math.pi * u)
            dx_world = step.foot_target.position_m[0] - swing_start_pose.position_m[0]
            dy_world = step.foot_target.position_m[1] - swing_start_pose.position_m[1]
            cosine = math.cos(feedback.root_pose.yaw_rad)
            sine = math.sin(feedback.root_pose.yaw_rad)
            forward = cosine * dx_world + sine * dy_world
            lateral = -sine * dx_world + cosine * dy_world
            stride_scale = max(-1.0, min(1.0, forward / self.config.max_step_length_m))
            lateral_scale = max(
                -1.0, min(1.0, lateral / self.config.max_step_length_m)
            )
            knee = (
                self.config.neutral_knee_flexion_rad
                + bump
                * (self.config.swing_knee_flexion_rad - self.config.neutral_knee_flexion_rad)
            )
            physical_hip = (
                -0.5 * self.config.neutral_knee_flexion_rad
                - blend * stride_scale * self.config.landing_hip_pitch_rad
                - 0.5 * bump * (knee - self.config.neutral_knee_flexion_rad)
            )
            physical_ankle = -physical_hip - knee
            sign = swing_side.pitch_axis_sign
            prefix = swing_side.value
            landing = {
                f"{prefix}_hip_pitch_joint": sign * physical_hip,
                f"{prefix}_knee_joint": sign * knee,
                f"{prefix}_ankle_pitch_joint": sign * physical_ankle,
                # Both hip-yaw axes are -Z in the supplied URDF, so the joint
                # target is the negative of the desired physical/world yaw.
                f"{prefix}_hip_yaw_joint": -_wrap_angle(
                    step.foot_target.yaw_rad - swing_start_pose.yaw_rad
                ),
                f"{prefix}_hip_roll_joint": lateral_scale
                * self.config.landing_hip_roll_rad,
                # Both hip-roll axes are +X and both ankle-roll axes are -X,
                # so equal numeric signs approximately preserve sole roll.
                f"{prefix}_ankle_roll_joint": lateral_scale
                * self.config.landing_hip_roll_rad,
            }
            for name, destination in landing.items():
                source = float(swing_start_joint_positions[name])
                targets[name] = source + blend * (destination - source)

        # Apply small physical-space balance corrections.  Pitch axes are
        # mirrored; hip-roll axes share +X while ankle-roll axes share -X.
        for side in FootSide:
            prefix = side.value
            sign = side.pitch_axis_sign
            # With a planted flat foot, pelvis pitch is the negative of the
            # leg-chain pitch sum.  Subtract the controller correction so a
            # positive correction for measured negative pitch/backward drift
            # commands a positive pelvis restoring direction.
            targets[f"{prefix}_hip_pitch_joint"] -= sign * 0.35 * correction.sagittal_rad
            targets[f"{prefix}_ankle_pitch_joint"] -= sign * 0.65 * correction.sagittal_rad
            targets[f"{prefix}_hip_roll_joint"] += 0.35 * correction.lateral_rad
            targets[f"{prefix}_ankle_roll_joint"] -= 0.65 * correction.lateral_rad

        # Measured q and qd bound target error and add modest velocity damping.
        for name in LEG_JOINTS:
            measured = float(feedback.joint_position_rad[name])
            velocity = float(feedback.joint_velocity_rad_s[name])
            damped = targets[name] - self.config.joint_velocity_damping_s * velocity
            low = measured - self.config.maximum_target_error_rad
            high = measured + self.config.maximum_target_error_rad
            targets[name] = max(low, min(high, damped))
        return _limit_targets(self.spec, targets, self.config.joint_limit_margin_rad)


@dataclass(frozen=True)
class LocomotionCommand:
    state: LocomotionState
    phase: GaitPhase
    joint_targets_rad: Mapping[str, float]
    swing_foot_reference: FootPose | None
    active_step_index: int
    planned_step_count: int
    failure_reason: str | None


class BipedLocomotionController:
    """Contact-gated step state machine with injected runtime feedback."""

    def __init__(
        self,
        spec: ProductionRobotSpec,
        articulation_controller: ArticulationController,
        feedback_source: LocomotionFeedbackSource,
        *,
        config: GaitConfig = GaitConfig(),
    ):
        self.spec = spec
        self.articulation_controller = articulation_controller
        self.feedback_source = feedback_source
        self.config = config
        self.planner = WaypointStepPlanner(config)
        self.balance = BalanceFeedbackController(config)
        self.gait_targets = ConservativeGaitTargetGenerator(spec, config)
        self.state = LocomotionState.IDLE
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.failure_reason: str | None = None
        self._pending_waypoints: tuple[PlanarPose, ...] = ()
        self._first_swing = FootSide.LEFT
        self._steps: tuple[Footstep, ...] = ()
        self._step_index = 0
        self._phase_started_s: float | None = None
        self._double_contact_started_s: float | None = None
        self._terminal_started_s: float | None = None
        self._settle_started_s: float | None = None
        self._last_timestamp_s: float | None = None
        self._stop_requested = False
        self._saw_swing_unloaded = False
        self._swing_start_pose: FootPose | None = None
        self._swing_start_joint_positions: dict[str, float] | None = None

    @property
    def steps(self) -> tuple[Footstep, ...]:
        return self._steps

    def start_route(
        self,
        waypoints: Sequence[PlanarPose],
        *,
        first_swing: FootSide = FootSide.LEFT,
    ) -> None:
        if self.state not in {
            LocomotionState.IDLE,
            LocomotionState.STOPPED,
            LocomotionState.DOCKED,
        }:
            raise LocomotionError(f"cannot start a route while controller state is {self.state.value}")
        if not waypoints:
            raise LocomotionError("a route requires at least one waypoint")
        for index, waypoint in enumerate(waypoints):
            _validate_planar_pose(waypoint, f"waypoint {index}")
        self._pending_waypoints = tuple(waypoints)
        self._first_swing = first_swing
        self._steps = ()
        self._step_index = 0
        self.state = LocomotionState.ACTIVE
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.failure_reason = None
        self._phase_started_s = None
        self._double_contact_started_s = None
        self._terminal_started_s = None
        self._settle_started_s = None
        self._last_timestamp_s = None
        self._stop_requested = False
        self._saw_swing_unloaded = False
        self._swing_start_pose = None
        self._swing_start_joint_positions = None

    def request_stop(self) -> None:
        """Finish an airborne step, then settle in double support."""

        if self.state is LocomotionState.ACTIVE:
            self._stop_requested = True
        elif self.state is LocomotionState.DOCKING:
            self.state = LocomotionState.STOPPING
            self.phase = GaitPhase.DOUBLE_SUPPORT
            self._terminal_started_s = self._last_timestamp_s
            self._settle_started_s = None

    def update(self) -> LocomotionCommand:
        feedback = self.feedback_source.read_feedback()
        try:
            _validate_feedback(feedback)
            if (
                self._last_timestamp_s is not None
                and feedback.timestamp_s <= self._last_timestamp_s
            ):
                raise LocomotionError("feedback timestamps must increase strictly")
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._fault(f"invalid locomotion feedback: {exc}")
            return self._fault_command()
        self._last_timestamp_s = feedback.timestamp_s

        if self.state is LocomotionState.ACTIVE and not self._steps:
            self._steps = self.planner.plan(
                feedback.root_pose,
                self._pending_waypoints,
                first_swing=self._first_swing,
                ground_height_m=min(
                    feedback.left_foot.pose.position_m[2],
                    feedback.right_foot.pose.position_m[2],
                ),
            )
            self._phase_started_s = feedback.timestamp_s
            self._double_contact_started_s = (
                feedback.timestamp_s
                if feedback.left_foot.in_contact and feedback.right_foot.in_contact
                else None
            )
            if not self._steps:
                self._enter_terminal(feedback, LocomotionState.DOCKING)

        correction = self.balance.evaluate(
            feedback,
            desired_forward_speed_mps=(
                self.config.nominal_speed_mps
                if self.state is LocomotionState.ACTIVE and self.phase.swing_side is not None
                else 0.0
            ),
        )
        if correction.unsafe_reason and self.state is not LocomotionState.FAULT:
            self._fault(correction.unsafe_reason)
            return self._fault_command()
        if self.state is LocomotionState.FAULT:
            return self._fault_command()

        swing_reference: FootPose | None = None
        active_step = self._active_step()
        progress = 0.0
        if self.state is LocomotionState.ACTIVE:
            if self.phase is GaitPhase.DOUBLE_SUPPORT:
                self._update_double_support(feedback)
            else:
                progress, swing_reference = self._update_swing(feedback)
        elif self.state is LocomotionState.DOCKING:
            self._update_docking(feedback)
        elif self.state is LocomotionState.STOPPING:
            self._update_stopping(feedback)

        if self.state is LocomotionState.FAULT:
            return self._fault_command()

        # Transitions above can change the active step or phase.
        active_step = self._active_step()
        if self.phase.swing_side is None:
            progress = 0.0
            swing_reference = None
        elif active_step is not None and self._swing_start_pose is not None:
            elapsed = feedback.timestamp_s - float(self._phase_started_s)
            progress = min(1.0, max(0.0, elapsed / self.config.swing_duration_s))
            swing_reference = swing_foot_trajectory(
                self._swing_start_pose,
                active_step.foot_target,
                progress,
                self.config.swing_clearance_m,
            )

        targets = self.gait_targets.targets(
            feedback,
            self.phase,
            progress,
            correction,
            step=active_step,
            swing_start_pose=self._swing_start_pose,
            swing_start_joint_positions=self._swing_start_joint_positions,
        )
        try:
            self.articulation_controller.command_joint_positions(dict(targets))
        except JointTargetError as exc:
            self._fault(f"joint target validation failed: {exc}")
            return self._fault_command()
        return LocomotionCommand(
            state=self.state,
            phase=self.phase,
            joint_targets_rad=dict(targets),
            swing_foot_reference=swing_reference,
            active_step_index=self._step_index,
            planned_step_count=len(self._steps),
            failure_reason=self.failure_reason,
        )

    def _active_step(self) -> Footstep | None:
        if self._step_index >= len(self._steps):
            return None
        return self._steps[self._step_index]

    def _update_double_support(self, feedback: LocomotionFeedback) -> None:
        if self._phase_started_s is None:
            self._phase_started_s = feedback.timestamp_s
        elapsed = feedback.timestamp_s - self._phase_started_s
        both_contact = feedback.left_foot.in_contact and feedback.right_foot.in_contact
        if both_contact:
            if self._double_contact_started_s is None:
                self._double_contact_started_s = feedback.timestamp_s
        else:
            self._double_contact_started_s = None
        if self._stop_requested:
            self._enter_terminal(feedback, LocomotionState.STOPPING)
            return
        step = self._active_step()
        if step is None:
            self._enter_terminal(feedback, LocomotionState.DOCKING)
            return
        continuous_contact_s = (
            feedback.timestamp_s - self._double_contact_started_s
            if self._double_contact_started_s is not None
            else 0.0
        )
        if both_contact and continuous_contact_s >= self.config.double_support_duration_s:
            measured_foot = feedback.foot(step.side).pose
            measured_step_distance = math.dist(
                measured_foot.position_m, step.foot_target.position_m
            )
            measured_step_yaw = abs(
                _wrap_angle(step.foot_target.yaw_rad - measured_foot.yaw_rad)
            )
            if measured_step_distance > self.config.max_step_length_m + 1e-12:
                self._fault(
                    "measured swing-foot displacement exceeds max_step_length_m: "
                    f"{measured_step_distance:.6f} m"
                )
                return
            if measured_step_yaw > self.config.max_step_yaw_rad + 1e-12:
                self._fault(
                    "measured swing-foot yaw exceeds max_step_yaw_rad: "
                    f"{measured_step_yaw:.6f} rad"
                )
                return
            self.phase = (
                GaitPhase.LEFT_SWING if step.side is FootSide.LEFT else GaitPhase.RIGHT_SWING
            )
            self._phase_started_s = feedback.timestamp_s
            self._double_contact_started_s = None
            self._saw_swing_unloaded = False
            self._swing_start_pose = feedback.foot(step.side).pose
            self._swing_start_joint_positions = {
                name: float(feedback.joint_position_rad[name]) for name in LEG_JOINTS
            }
            return
        if elapsed > self.config.double_support_timeout_s:
            self._fault("double-support contact timeout")

    def _update_swing(
        self, feedback: LocomotionFeedback
    ) -> tuple[float, FootPose | None]:
        step = self._active_step()
        if step is None or self._phase_started_s is None or self._swing_start_pose is None:
            self._fault("swing phase has no active step/start state")
            return 0.0, None
        elapsed = feedback.timestamp_s - self._phase_started_s
        progress = min(1.0, max(0.0, elapsed / self.config.swing_duration_s))
        reference = swing_foot_trajectory(
            self._swing_start_pose,
            step.foot_target,
            progress,
            self.config.swing_clearance_m,
        )
        foot = feedback.foot(step.side)
        if not foot.in_contact:
            self._saw_swing_unloaded = True
        position_error_xy = math.hypot(
            foot.pose.position_m[0] - step.foot_target.position_m[0],
            foot.pose.position_m[1] - step.foot_target.position_m[1],
        )
        height_error = abs(foot.pose.position_m[2] - step.foot_target.position_m[2])
        yaw_error = abs(_wrap_angle(foot.pose.yaw_rad - step.foot_target.yaw_rad))
        valid_touchdown = (
            self._saw_swing_unloaded
            and foot.in_contact
            and progress >= self.config.minimum_touchdown_progress
            and position_error_xy <= self.config.touchdown_position_tolerance_m
            and height_error <= self.config.touchdown_height_tolerance_m
            and yaw_error <= self.config.touchdown_yaw_tolerance_rad
        )
        if valid_touchdown:
            self._step_index += 1
            self.phase = GaitPhase.DOUBLE_SUPPORT
            self._phase_started_s = feedback.timestamp_s
            self._double_contact_started_s = (
                feedback.timestamp_s
                if feedback.left_foot.in_contact and feedback.right_foot.in_contact
                else None
            )
            self._saw_swing_unloaded = False
            self._swing_start_pose = None
            self._swing_start_joint_positions = None
            if self._stop_requested:
                self._enter_terminal(feedback, LocomotionState.STOPPING)
            return 1.0, None
        if elapsed > self.config.swing_timeout_s:
            self._fault(f"{step.side.value} swing touchdown timeout")
            return 0.0, None
        return progress, reference

    def _enter_terminal(
        self, feedback: LocomotionFeedback, state: LocomotionState
    ) -> None:
        self.state = state
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self._terminal_started_s = feedback.timestamp_s
        self._double_contact_started_s = None
        self._settle_started_s = None
        self._swing_start_pose = None
        self._swing_start_joint_positions = None

    def _update_stopping(self, feedback: LocomotionFeedback) -> None:
        if self._terminal_started_s is None:
            self._terminal_started_s = feedback.timestamp_s
        if self._settled(feedback, check_pose=False):
            if self._settle_started_s is None:
                self._settle_started_s = feedback.timestamp_s
            elif feedback.timestamp_s - self._settle_started_s >= self.config.stop_settle_duration_s:
                self.state = LocomotionState.STOPPED
        else:
            self._settle_started_s = None
        if feedback.timestamp_s - self._terminal_started_s > self.config.stop_timeout_s:
            self._fault("safe-stop settle timeout")

    def _update_docking(self, feedback: LocomotionFeedback) -> None:
        if self._terminal_started_s is None:
            self._terminal_started_s = feedback.timestamp_s
        if self._settled(feedback, check_pose=True):
            if self._settle_started_s is None:
                self._settle_started_s = feedback.timestamp_s
            elif feedback.timestamp_s - self._settle_started_s >= self.config.dock_settle_duration_s:
                self.state = LocomotionState.DOCKED
        else:
            self._settle_started_s = None
        if feedback.timestamp_s - self._terminal_started_s > self.config.dock_timeout_s:
            self._fault("dock pose/velocity/contact timeout")

    def _settled(self, feedback: LocomotionFeedback, *, check_pose: bool) -> bool:
        if not (feedback.left_foot.in_contact and feedback.right_foot.in_contact):
            return False
        linear_speed = math.sqrt(
            sum(value * value for value in feedback.root_linear_velocity_body_mps)
        )
        angular_speed = math.sqrt(
            sum(value * value for value in feedback.root_angular_velocity_body_rps)
        )
        if (
            linear_speed > self.config.dock_linear_speed_tolerance_mps
            or angular_speed > self.config.dock_angular_speed_tolerance_rps
        ):
            return False
        if not check_pose:
            return True
        target = self._pending_waypoints[-1]
        return (
            math.hypot(feedback.root_pose.x_m - target.x_m, feedback.root_pose.y_m - target.y_m)
            <= self.config.dock_position_tolerance_m
            and abs(_wrap_angle(feedback.root_pose.yaw_rad - target.yaw_rad))
            <= self.config.dock_yaw_tolerance_rad
        )

    def _fault(self, reason: str) -> None:
        self.state = LocomotionState.FAULT
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.failure_reason = reason
        self._swing_start_pose = None
        self._swing_start_joint_positions = None

    def _fault_command(self) -> LocomotionCommand:
        """Report a latched fault without issuing another articulation command."""

        return LocomotionCommand(
            state=LocomotionState.FAULT,
            phase=GaitPhase.DOUBLE_SUPPORT,
            joint_targets_rad={},
            swing_foot_reference=None,
            active_step_index=self._step_index,
            planned_step_count=len(self._steps),
            failure_reason=self.failure_reason,
        )


def _validate_planar_pose(pose: PlanarPose, label: str) -> None:
    if not all(math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad)):
        raise LocomotionError(f"{label} must contain finite values")


def _validate_feedback(feedback: LocomotionFeedback) -> None:
    if not math.isfinite(feedback.timestamp_s) or feedback.timestamp_s < 0.0:
        raise LocomotionError("feedback timestamp must be finite and nonnegative")
    _validate_planar_pose(feedback.root_pose, "root pose")
    if not math.isfinite(feedback.root_height_m):
        raise LocomotionError("root height must be finite")
    _finite_sequence(feedback.root_tilt_roll_pitch_rad, 2, "root tilt")
    _finite_sequence(feedback.root_linear_velocity_body_mps, 3, "root linear velocity")
    _finite_sequence(feedback.root_angular_velocity_body_rps, 3, "root angular velocity")
    _finite_sequence(feedback.com_position_world_m, 3, "COM position")
    _finite_sequence(feedback.support_center_world_m, 3, "support center")
    if not math.isfinite(feedback.support_margin_m):
        raise LocomotionError("support margin must be finite")
    for side in FootSide:
        foot = feedback.foot(side)
        _finite_sequence(foot.pose.position_m, 3, f"{side.value} foot position")
        if not math.isfinite(foot.pose.yaw_rad):
            raise LocomotionError(f"{side.value} foot yaw must be finite")
        if not isinstance(foot.in_contact, bool):
            raise LocomotionError(f"{side.value} foot contact must be boolean")
    for label, values in (
        ("joint position", feedback.joint_position_rad),
        ("joint velocity", feedback.joint_velocity_rad_s),
    ):
        missing = sorted(set(LEG_JOINTS) - set(values))
        if missing:
            raise LocomotionError(f"{label} feedback missing leg joints: {missing}")
        if not all(math.isfinite(float(values[name])) for name in LEG_JOINTS):
            raise LocomotionError(f"{label} feedback must be finite for every leg joint")


def _limit_targets(
    spec: ProductionRobotSpec,
    targets: Mapping[str, float],
    margin_rad: float,
) -> dict[str, float]:
    if not math.isfinite(margin_rad) or margin_rad < 0.0:
        raise LocomotionError("joint limit margin must be finite and nonnegative")
    limited: dict[str, float] = {}
    for name, raw_value in targets.items():
        value = float(raw_value)
        if not math.isfinite(value):
            raise LocomotionError(f"target for {name} must be finite")
        limit = spec.model.joint_limits[name]
        lower = limit.lower + margin_rad
        upper = limit.upper - margin_rad
        if lower > upper:
            raise LocomotionError(f"joint limit margin consumes the range for {name}")
        limited[name] = max(lower, min(upper, value))
    # Reuse the authoritative production-model validation after clamping.
    return spec.validate_targets(limited)
