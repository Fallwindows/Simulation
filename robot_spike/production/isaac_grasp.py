"""Isaac Sim 6.1 measured-contact adapter for the reviewed R4 controller.

Isaac imports are deliberately absent from this module.  The runtime harness
injects the experimental articulation, pose views, contact sensor, and
``PhysicsSchemaTools.intToSdfPath`` resolver.  Raw impulses, timestamps, and
step durations remain the authority; aggregate sensor force is diagnostic.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import numbers
from types import MappingProxyType
from typing import Callable, Mapping, Protocol, Sequence

from .arm_reach import ARM_DOF_NAMES, ArmReachPlanner, RightArmKinematics, ToolPose
from .isaac_feedback import FeedbackUnavailableError
from .physical_grasp import (
    BodyPose,
    ContactPair,
    GraspObservation,
    HAND_DOF_NAMES,
    LiftRequest,
    REQUIRED_CONTACT_LINKS,
    RobotContactBodyMap,
)


class ArticulationStateBackend(Protocol):
    @property
    def dof_names(self) -> Sequence[str]: ...

    @property
    def link_names(self) -> Sequence[str]: ...

    def get_dof_positions(self): ...


class PoseBackend(Protocol):
    def get_world_poses(self): ...


class ContactReading(Protocol):
    is_valid: bool
    in_contact: bool
    value: float
    time: float


class ContactSensorBackend(Protocol):
    def get_sensor_reading(self) -> ContactReading: ...

    def get_raw_data(self) -> Sequence[Mapping[str, object]]: ...


class PositionTargetController(Protocol):
    def command_joint_positions(self, targets: dict[str, float]) -> Sequence[str]: ...


def _plain(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return value


def _vector(value, length: int, label: str) -> tuple[float, ...]:
    value = _plain(value)
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise FeedbackUnavailableError(f"{label} must have shape ({length},)")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise FeedbackUnavailableError(f"{label} contains a nonfinite value")
    return result


def _single_row(value, columns: int, label: str) -> tuple[float, ...]:
    value = _plain(value)
    if not isinstance(value, (list, tuple)) or len(value) != 1:
        raise FeedbackUnavailableError(f"{label} must have shape (1, {columns})")
    return _vector(value[0], columns, f"{label} row")


def _xyz(value: object, label: str) -> tuple[float, float, float]:
    value = _plain(value)
    if isinstance(value, Mapping):
        try:
            value = (value["x"], value["y"], value["z"])
        except KeyError as exc:
            raise FeedbackUnavailableError(f"{label} lacks an xyz component") from exc
    return _vector(value, 3, label)  # type: ignore[return-value]


def _normalized_xyzw_from_wxyz(value: object, label: str) -> tuple[float, float, float, float]:
    w, x, y, z = _vector(value, 4, label)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        raise FeedbackUnavailableError(f"{label} has zero magnitude")
    return (x / norm, y / norm, z / norm, w / norm)


@dataclass(frozen=True)
class IsaacContactBindings:
    """Exact paths accepted from ``intToSdfPath`` for product contact."""

    robot_contacts: RobotContactBodyMap
    product_body_prim_path: str
    product_collider_prim_path: str
    pickup_support_prim_path: str

    def __post_init__(self) -> None:
        paths = (
            self.product_body_prim_path,
            self.product_collider_prim_path,
            self.pickup_support_prim_path,
        )
        if any(not isinstance(path, str) or not path.startswith("/") for path in paths):
            raise ValueError("contact bindings require absolute USD prim paths")
        if len(set(paths)) != len(paths):
            raise ValueError("product body, collider, and support paths must be distinct")

    @property
    def allowed_body_paths(self) -> frozenset[str]:
        return frozenset(
            (*self.robot_contacts.link_body_prim_paths.values(),
             self.product_body_prim_path,
             self.product_collider_prim_path,
             self.pickup_support_prim_path)
        )


class Isaac61GraspFeedbackAdapter:
    """Create one co-timed R4 observation solely from measured Isaac state."""

    def __init__(
        self,
        articulation: ArticulationStateBackend,
        palm_pose: PoseBackend,
        product_pose: PoseBackend,
        product_contact_sensor: ContactSensorBackend,
        timestamp_source: Callable[[], float],
        body_path_resolver: Callable[[int], str],
        bindings: IsaacContactBindings,
        *,
        maximum_contact_age_s: float,
    ) -> None:
        if not math.isfinite(maximum_contact_age_s) or maximum_contact_age_s < 0.0:
            raise ValueError("maximum contact age must be finite and nonnegative")
        self.articulation = articulation
        self.palm_pose = palm_pose
        self.product_pose = product_pose
        self.product_contact_sensor = product_contact_sensor
        self.timestamp_source = timestamp_source
        self.body_path_resolver = body_path_resolver
        self.bindings = bindings
        self.maximum_contact_age_s = float(maximum_contact_age_s)
        self._dof_names = tuple(str(name) for name in articulation.dof_names)
        self._link_names = tuple(str(name) for name in articulation.link_names)
        if len(set(self._dof_names)) != len(self._dof_names):
            raise FeedbackUnavailableError("articulation reports duplicate DOF names")
        if len(set(self._link_names)) != len(self._link_names):
            raise FeedbackUnavailableError("articulation reports duplicate link names")
        missing_dofs = sorted(set(HAND_DOF_NAMES) - set(self._dof_names))
        missing_links = sorted((set(REQUIRED_CONTACT_LINKS) | {"right_palm"}) - set(self._link_names))
        if missing_dofs or missing_links:
            raise FeedbackUnavailableError(
                f"production hand binding is incomplete; missing_dofs={missing_dofs}, "
                f"missing_links={missing_links}"
            )
        for link, path in bindings.robot_contacts.link_body_prim_paths.items():
            if link not in self._link_names or path.rsplit("/", 1)[-1] != link:
                raise FeedbackUnavailableError(f"contact body binding does not match link {link!r}")
        self._last_timestamp_s: float | None = None
        self._diagnostics: dict[str, object] = {}

    def diagnostics(self) -> dict[str, object]:
        return copy.deepcopy(self._diagnostics)

    @staticmethod
    def _pose(backend: PoseBackend, label: str) -> BodyPose:
        positions, orientations = backend.get_world_poses()
        position = _single_row(positions, 3, f"{label} positions")
        orientation = _normalized_xyzw_from_wxyz(
            _single_row(orientations, 4, f"{label} orientations"),
            f"{label} orientation",
        )
        return BodyPose(position, orientation)  # type: ignore[arg-type]

    def _resolve_body(self, value: object) -> str:
        value = _plain(value)
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            raise FeedbackUnavailableError(
                f"contact body handle must be an integer, got {value!r}"
            )
        try:
            path = str(self.body_path_resolver(value))
        except Exception as exc:
            raise FeedbackUnavailableError(f"cannot resolve contact body handle {value!r}") from exc
        if path not in self.bindings.allowed_body_paths:
            raise FeedbackUnavailableError(f"unbound contact body path {path!r}")
        return path

    def _contacts(self, timestamp_s: float) -> tuple[ContactPair, ...]:
        try:
            reading = self.product_contact_sensor.get_sensor_reading()
            valid = reading.is_valid
            in_contact = reading.in_contact
            aggregate_force = float(reading.value)
            reading_time = float(reading.time)
            raw = tuple(self.product_contact_sensor.get_raw_data())
        except Exception as exc:
            raise FeedbackUnavailableError(f"product contact read failed: {exc}") from exc
        if not isinstance(valid, bool) or not isinstance(in_contact, bool) or not valid:
            raise FeedbackUnavailableError("product contact sensor reading is invalid")
        if not math.isfinite(aggregate_force) or aggregate_force < 0.0:
            raise FeedbackUnavailableError("aggregate contact force is invalid")
        if not math.isfinite(reading_time) or abs(timestamp_s - reading_time) > self.maximum_contact_age_s:
            raise FeedbackUnavailableError("product contact reading is stale")
        if in_contact != bool(raw):
            raise FeedbackUnavailableError("aggregate contact state disagrees with raw records")

        forces: dict[tuple[str, str], float] = {}
        raw_times: list[float] = []
        for index, record in enumerate(raw):
            if not isinstance(record, Mapping):
                raise FeedbackUnavailableError(f"raw contact {index} is not a mapping")
            try:
                body0 = self._resolve_body(record["body0"])
                body1 = self._resolve_body(record["body1"])
                _xyz(record["position"], f"raw contact {index} position")
                normal = _xyz(record["normal"], f"raw contact {index} normal")
                impulse = _xyz(record["impulse"], f"raw contact {index} impulse")
                record_time = float(record["time"])
                dt = float(record["dt"])
            except KeyError as exc:
                raise FeedbackUnavailableError(f"raw contact {index} lacks {exc.args[0]}") from exc
            if body0 == body1:
                raise FeedbackUnavailableError("collapsed contact body pair")
            if math.sqrt(sum(component * component for component in normal)) <= 1.0e-12:
                raise FeedbackUnavailableError("raw contact normal has zero magnitude")
            product_paths = {self.bindings.product_body_prim_path, self.bindings.product_collider_prim_path}
            if not product_paths.intersection((body0, body1)):
                raise FeedbackUnavailableError("product sensor returned a contact without the product")
            other = body1 if body0 in product_paths else body0
            if other in product_paths:
                raise FeedbackUnavailableError("contact has product on both sides")
            if not math.isfinite(record_time) or abs(timestamp_s - record_time) > self.maximum_contact_age_s:
                raise FeedbackUnavailableError("raw contact record is stale")
            if not math.isfinite(dt) or dt <= 0.0:
                raise FeedbackUnavailableError("raw contact dt must be finite and positive")
            force = math.sqrt(sum(component * component for component in impulse)) / dt
            key = (self.bindings.product_body_prim_path, other)
            forces[key] = forces.get(key, 0.0) + force
            raw_times.append(record_time)

        path_to_link = {
            path: link for link, path in self.bindings.robot_contacts.link_body_prim_paths.items()
        }
        contacts = tuple(
            ContactPair(product, other, force, path_to_link.get(other))
            for (product, other), force in sorted(forces.items())
        )
        self._diagnostics = {
            "sample_time_s": timestamp_s,
            "reading_time_s": reading_time,
            "aggregate_force_n": aggregate_force,
            "in_contact": in_contact,
            "raw_contact_count": len(raw),
            "raw_times_s": raw_times,
            "normalized_contacts": [
                {"body0": item.body0_prim_path, "body1": item.body1_prim_path,
                 "normal_force_n": item.normal_force_n,
                 "robot_link_name": item.robot_link_name}
                for item in contacts
            ],
        }
        return contacts

    def read_observation(self) -> GraspObservation:
        timestamp_s = float(self.timestamp_source())
        if not math.isfinite(timestamp_s):
            raise FeedbackUnavailableError("simulation timestamp is nonfinite")
        if self._last_timestamp_s is not None and timestamp_s <= self._last_timestamp_s:
            raise FeedbackUnavailableError("simulation timestamp did not advance")
        positions = _single_row(
            self.articulation.get_dof_positions(), len(self._dof_names), "DOF positions"
        )
        joints = {name: positions[index] for index, name in enumerate(self._dof_names) if name in HAND_DOF_NAMES}
        contacts = self._contacts(timestamp_s)
        observation = GraspObservation(
            timestamp_s=timestamp_s,
            product_prim_path=self.bindings.product_body_prim_path,
            hand_joint_positions_rad=MappingProxyType(joints),
            palm_pose_world=self._pose(self.palm_pose, "right palm"),
            product_pose_world=self._pose(self.product_pose, "product"),
            contacts=contacts,
        )
        self._last_timestamp_s = timestamp_s
        return observation


class IsaacArmLiftPort:
    """Bounded right-arm-only lift queue using the reviewed R3 kinematics."""

    def __init__(
        self,
        controller: PositionTargetController,
        articulation: ArticulationStateBackend,
        kinematics: RightArmKinematics,
        timestamp_source: Callable[[], float],
        expected_product_prim_path: str,
        *,
        command_period_s: float,
    ) -> None:
        self.controller = controller
        self.articulation = articulation
        self.kinematics = kinematics
        self.timestamp_source = timestamp_source
        if not expected_product_prim_path.startswith("/"):
            raise ValueError("expected product path must be absolute")
        self.expected_product_prim_path = expected_product_prim_path
        self.planner = ArmReachPlanner(kinematics, command_period_s=command_period_s)
        self.command_period_s = float(command_period_s)
        self._dof_names = tuple(str(name) for name in articulation.dof_names)
        if not set(ARM_DOF_NAMES).issubset(self._dof_names):
            raise ValueError("articulation is missing a right-arm DOF")
        self._waypoints: list[dict[str, float]] = []
        self._deadline_s: float | None = None
        self.last_error: str | None = None

    def request_lift(self, request: LiftRequest) -> bool:
        if self._waypoints or self._deadline_s is not None:
            return False
        if request.product_prim_path != self.expected_product_prim_path:
            return False
        now = float(self.timestamp_source())
        if not all(math.isfinite(value) and value > 0.0 for value in (request.displacement_m, request.maximum_duration_s)):
            return False
        try:
            row = _single_row(self.articulation.get_dof_positions(), len(self._dof_names), "DOF positions")
            start = {name: row[self._dof_names.index(name)] for name in ARM_DOF_NAMES}
            pose = self.kinematics.forward(start)
            target = ToolPose(
                (pose.position_m[0], pose.position_m[1], pose.position_m[2] + request.displacement_m),
                pose.orientation_xyzw,
            )
            goal = self.kinematics.solve(target, start).as_mapping()
            steps = max(
                1,
                max(
                    math.ceil(
                        abs(goal[name] - start[name])
                        / self.planner.maximum_step_for_joint(name)
                    )
                    for name in ARM_DOF_NAMES
                ),
            )
            self._waypoints = [
                {
                    name: start[name] + (goal[name] - start[name]) * step / steps
                    for name in ARM_DOF_NAMES
                }
                for step in range(1, steps + 1)
            ]
        except Exception as exc:
            self.last_error = str(exc)
            self._waypoints = []
            return False
        if len(self._waypoints) * self.command_period_s > request.maximum_duration_s:
            self.last_error = "bounded arm plan cannot finish before lift deadline"
            self._waypoints = []
            return False
        self._deadline_s = now + request.maximum_duration_s
        return bool(self._waypoints)

    def advance(self) -> bool:
        if self._deadline_s is None:
            return False
        now = float(self.timestamp_source())
        if not math.isfinite(now) or now > self._deadline_s:
            self.last_error = "arm lift deadline expired"
            self._waypoints.clear()
            return False
        if not self._waypoints:
            return True
        targets = self._waypoints.pop(0)
        commanded = tuple(self.controller.command_joint_positions(targets))
        if commanded != ARM_DOF_NAMES:
            self.last_error = f"unexpected arm command set: {commanded}"
            self._waypoints.clear()
            return False
        return True
