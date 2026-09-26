"""Isaac Sim 6.1 measured-state adapter for the R2 locomotion policy.

This module deliberately has no Isaac imports.  Runtime objects are injected so
the normalization, frame transforms, COM calculation, and support geometry can
be tested on CPU.  The concrete methods used here are the Isaac Sim 6.1
experimental APIs documented in ``review/robot_manipulation/LOCOMOTION_CONTROL.md``.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
from typing import Callable, Protocol, Sequence

from .locomotion import (
    FootFeedback,
    FootPose,
    LEG_JOINTS,
    LocomotionFeedback,
    PlanarPose,
)


class FeedbackUnavailableError(RuntimeError):
    """Raised when a required physical reading is absent, stale, or malformed."""


class IsaacArticulationFeedbackBackend(Protocol):
    @property
    def dof_names(self) -> Sequence[str]: ...

    @property
    def link_names(self) -> Sequence[str]: ...

    def get_world_poses(self): ...

    def get_velocities(self): ...

    def get_dof_positions(self): ...

    def get_dof_velocities(self): ...

    def get_link_masses(self): ...

    def get_link_coms(self): ...


class IsaacLinkPoseBackend(Protocol):
    def get_world_poses(self): ...


class IsaacContactReading(Protocol):
    is_valid: bool
    in_contact: bool
    value: float
    time: float


class IsaacContactSensorBackend(Protocol):
    def get_sensor_reading(self) -> IsaacContactReading: ...

    def get_raw_data(self) -> Sequence[object]: ...


@dataclass(frozen=True)
class SoleGeometry:
    """Sole reference and support vertices in the ankle-roll link frame."""

    reference_m: tuple[float, float, float]
    support_vertices_m: tuple[tuple[float, float, float], ...]
    contact_sphere_radius_m: float

    def __post_init__(self) -> None:
        _finite_vector(self.reference_m, 3, "sole reference")
        if len(self.support_vertices_m) < 3:
            raise ValueError("sole geometry requires at least three support vertices")
        for vertex in self.support_vertices_m:
            _finite_vector(vertex, 3, "sole support vertex")
        if not math.isfinite(self.contact_sphere_radius_m) or self.contact_sphere_radius_m <= 0.0:
            raise ValueError("contact sphere radius must be finite and positive")


def _plain(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return value


def _finite_vector(value, length: int, label: str) -> tuple[float, ...]:
    value = _plain(value)
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise FeedbackUnavailableError(f"{label} must have shape ({length},)")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise FeedbackUnavailableError(f"{label} contains a nonfinite value")
    return result


def _matrix(value, rows: int, columns: int, label: str) -> tuple[tuple[float, ...], ...]:
    value = _plain(value)
    if not isinstance(value, (list, tuple)) or len(value) != rows:
        raise FeedbackUnavailableError(f"{label} must have shape ({rows}, {columns})")
    return tuple(_finite_vector(row, columns, f"{label} row") for row in value)


# The production URDF has four radius-5 mm contact spheres on each ankle-roll
# link.  These are the sphere bottom points and their centered sole reference.
ASIMOV_SOLE_GEOMETRY = SoleGeometry(
    reference_m=(0.0385, 0.0, -0.034),
    support_vertices_m=(
        (-0.045, -0.020, -0.034),
        (-0.045, 0.020, -0.034),
        (0.122, -0.028, -0.034),
        (0.122, 0.028, -0.034),
    ),
    contact_sphere_radius_m=0.005,
)


def _single_row(value, columns: int, label: str) -> tuple[float, ...]:
    return _matrix(value, 1, columns, label)[0]


def _normalized_quaternion_wxyz(value, label: str) -> tuple[float, float, float, float]:
    quaternion = _finite_vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1.0e-12:
        raise FeedbackUnavailableError(f"{label} has zero magnitude")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _rotate_wxyz(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    vx, vy, vz = vector
    # q * v * conjugate(q), expanded to avoid an Isaac or NumPy dependency.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _inverse_rotate_wxyz(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    return _rotate_wxyz((w, -x, -y, -z), vector)


def _roll_pitch_yaw_wxyz(quaternion: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _transform_point(
    position: Sequence[float], quaternion: Sequence[float], local_point: Sequence[float]
) -> tuple[float, float, float]:
    rotated = _rotate_wxyz(quaternion, local_point)
    return tuple(position[index] + rotated[index] for index in range(3))  # type: ignore[return-value]


def convex_hull_xy(points: Sequence[Sequence[float]]) -> tuple[tuple[float, float], ...]:
    """Return a deterministic counter-clockwise convex hull without repeats."""

    clean = sorted(
        {
            (float(point[0]), float(point[1]))
            for point in points
            if len(point) >= 2 and math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        }
    )
    if len(clean) < 3:
        raise FeedbackUnavailableError("support polygon requires three distinct finite points")

    def cross(origin, a, b) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in clean:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(clean):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = tuple(lower[:-1] + upper[:-1])
    if len(hull) < 3:
        raise FeedbackUnavailableError("support polygon is degenerate")
    return hull


def support_polygon_center_and_margin(
    points: Sequence[Sequence[float]], com_xy: Sequence[float]
) -> tuple[tuple[float, float], float]:
    """Compute area centroid and signed inward half-space margin in metres."""

    hull = convex_hull_xy(points)
    twice_area = 0.0
    centroid_x_numerator = 0.0
    centroid_y_numerator = 0.0
    margin = math.inf
    px, py = _finite_vector(com_xy, 2, "COM projection")
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        edge_x = end[0] - start[0]
        edge_y = end[1] - start[1]
        edge_length = math.hypot(edge_x, edge_y)
        if edge_length <= 1.0e-12:
            raise FeedbackUnavailableError("support polygon has a zero-length edge")
        edge_cross = start[0] * end[1] - end[0] * start[1]
        twice_area += edge_cross
        centroid_x_numerator += (start[0] + end[0]) * edge_cross
        centroid_y_numerator += (start[1] + end[1]) * edge_cross
        margin = min(margin, (edge_x * (py - start[1]) - edge_y * (px - start[0])) / edge_length)
    if twice_area <= 1.0e-12:
        raise FeedbackUnavailableError("support polygon has zero area")
    center = (
        centroid_x_numerator / (3.0 * twice_area),
        centroid_y_numerator / (3.0 * twice_area),
    )
    return center, margin


class Isaac61LocomotionFeedbackAdapter:
    """Normalize one measured Isaac articulation into ``LocomotionFeedback``.

    ``link_poses`` must wrap the articulation's link paths in exactly
    ``articulation.link_names`` order.  Contact validity and freshness are
    mandatory.  Missing readings raise instead of manufacturing a stable pose.
    """

    def __init__(
        self,
        articulation: IsaacArticulationFeedbackBackend,
        link_poses: IsaacLinkPoseBackend,
        left_contact_sensor: IsaacContactSensorBackend,
        right_contact_sensor: IsaacContactSensorBackend,
        timestamp_source: Callable[[], float],
        *,
        left_foot_link: str = "left_ankle_roll_link",
        right_foot_link: str = "right_ankle_roll_link",
        sole_geometry: SoleGeometry = ASIMOV_SOLE_GEOMETRY,
        maximum_contact_age_s: float | None = None,
        support_plane_tolerance_m: float = 0.0005,
        inferred_support_inset_fraction: float = 0.5,
    ) -> None:
        self.articulation = articulation
        self.link_poses = link_poses
        self.left_contact_sensor = left_contact_sensor
        self.right_contact_sensor = right_contact_sensor
        self.timestamp_source = timestamp_source
        self.sole_geometry = sole_geometry
        self.maximum_contact_age_s = maximum_contact_age_s
        self.support_plane_tolerance_m = float(support_plane_tolerance_m)
        self.inferred_support_inset_fraction = float(inferred_support_inset_fraction)
        if maximum_contact_age_s is not None and (
            not math.isfinite(maximum_contact_age_s) or maximum_contact_age_s < 0.0
        ):
            raise ValueError("maximum contact age must be finite and nonnegative")
        if not math.isfinite(self.support_plane_tolerance_m) or self.support_plane_tolerance_m <= 0.0:
            raise ValueError("support plane tolerance must be finite and positive")
        if not math.isfinite(self.inferred_support_inset_fraction) or not (
            0.0 < self.inferred_support_inset_fraction < 1.0
        ):
            raise ValueError("inferred support inset fraction must be between zero and one")

        self._dof_names = tuple(str(name) for name in articulation.dof_names)
        self._link_names = tuple(str(name) for name in articulation.link_names)
        if len(set(self._dof_names)) != len(self._dof_names):
            raise FeedbackUnavailableError("articulation reports duplicate DOF names")
        if len(set(self._link_names)) != len(self._link_names):
            raise FeedbackUnavailableError("articulation reports duplicate link names")
        missing_dofs = sorted(set(LEG_JOINTS) - set(self._dof_names))
        if missing_dofs:
            raise FeedbackUnavailableError(f"articulation is missing leg DOFs: {missing_dofs}")
        missing_links = sorted({left_foot_link, right_foot_link} - set(self._link_names))
        if missing_links:
            raise FeedbackUnavailableError(f"articulation is missing foot links: {missing_links}")
        self._left_link_index = self._link_names.index(left_foot_link)
        self._right_link_index = self._link_names.index(right_foot_link)
        self.last_contact_forces_n: dict[str, float] = {}
        self.last_contact_times_s: dict[str, float] = {}
        self.last_contact_point_counts: dict[str, int] = {}
        self.last_contact_diagnostics: dict[str, dict[str, object]] = {}
        self.last_support_diagnostics: dict[str, object] = {}

    @property
    def link_names(self) -> tuple[str, ...]:
        return self._link_names

    def diagnostics(self) -> dict[str, object]:
        """Return a JSON-safe snapshot of the most recent contact/support read."""

        return copy.deepcopy(
            {
                "contacts": self.last_contact_diagnostics,
                "support": self.last_support_diagnostics,
            }
        )

    def _contact(
        self, sensor: IsaacContactSensorBackend, side: str, timestamp_s: float
    ) -> tuple[bool, tuple[tuple[float, float, float], ...]]:
        diagnostic: dict[str, object] = {
            "side": side,
            "sample_time_s": timestamp_s,
            "is_valid": None,
            "in_contact": None,
            "force_n": None,
            "reading_time_s": None,
            "age_s": None,
            "raw_point_count": None,
            "raw_points_world_m": [],
        }
        self.last_contact_diagnostics[side] = diagnostic
        try:
            reading = sensor.get_sensor_reading()
            if not isinstance(reading.is_valid, bool) or not isinstance(reading.in_contact, bool):
                raise TypeError("contact validity/state are not booleans")
            valid = reading.is_valid
            in_contact = reading.in_contact
            force_n = float(reading.value)
            reading_time_s = float(reading.time)
        except Exception as exc:
            diagnostic["error"] = f"contact read failed: {exc}"
            raise FeedbackUnavailableError(f"{side} contact read failed: {exc}") from exc
        diagnostic.update(
            {
                "is_valid": valid,
                "in_contact": in_contact,
                "force_n": force_n if math.isfinite(force_n) else None,
                "reading_time_s": reading_time_s if math.isfinite(reading_time_s) else None,
                "age_s": (
                    abs(timestamp_s - reading_time_s)
                    if math.isfinite(reading_time_s)
                    else None
                ),
            }
        )
        if not valid:
            diagnostic["error"] = "contact sensor reading is invalid"
            raise FeedbackUnavailableError(f"{side} contact sensor reading is invalid")
        if not math.isfinite(force_n) or force_n < 0.0:
            diagnostic["error"] = "contact force is invalid"
            raise FeedbackUnavailableError(f"{side} contact force is invalid")
        if not math.isfinite(reading_time_s) or reading_time_s < 0.0:
            diagnostic["error"] = "contact timestamp is invalid"
            raise FeedbackUnavailableError(f"{side} contact timestamp is invalid")
        if self.maximum_contact_age_s is not None and abs(timestamp_s - reading_time_s) > self.maximum_contact_age_s:
            diagnostic["error"] = "contact reading is stale"
            raise FeedbackUnavailableError(
                f"{side} contact reading is stale by {abs(timestamp_s - reading_time_s):.9f}s"
            )
        self.last_contact_forces_n[side] = force_n
        self.last_contact_times_s[side] = reading_time_s
        try:
            raw_contacts = sensor.get_raw_data()
        except Exception as exc:
            diagnostic["error"] = f"raw contact read failed: {exc}"
            raise FeedbackUnavailableError(f"{side} raw contact read failed: {exc}") from exc
        if not isinstance(raw_contacts, (list, tuple)):
            diagnostic["error"] = "raw contacts are not a sequence"
            raise FeedbackUnavailableError(f"{side} raw contacts are not a sequence")
        try:
            points = tuple(self._raw_contact_position(contact, side) for contact in raw_contacts)
        except FeedbackUnavailableError as exc:
            diagnostic["error"] = str(exc)
            raise
        self.last_contact_point_counts[side] = len(points)
        diagnostic["raw_point_count"] = len(points)
        diagnostic["raw_points_world_m"] = [list(point) for point in points]
        if in_contact and not points:
            diagnostic["error"] = "contact reported without measured points"
            raise FeedbackUnavailableError(
                f"{side} reports contact without any measured contact points"
            )
        return in_contact, points if in_contact else ()

    def read_feedback(self) -> LocomotionFeedback:
        self.last_contact_diagnostics = {}
        self.last_support_diagnostics = {}
        timestamp_s = float(self.timestamp_source())
        if not math.isfinite(timestamp_s) or timestamp_s < 0.0:
            raise FeedbackUnavailableError("simulation timestamp is invalid")

        root_positions, root_orientations = self.articulation.get_world_poses()
        root_position = _single_row(root_positions, 3, "root positions")
        root_quaternion = _normalized_quaternion_wxyz(
            _single_row(root_orientations, 4, "root orientations"), "root orientation"
        )
        roll, pitch, yaw = _roll_pitch_yaw_wxyz(root_quaternion)

        linear_world, angular_world = self.articulation.get_velocities()
        linear_body = _inverse_rotate_wxyz(
            root_quaternion, _single_row(linear_world, 3, "root linear velocities")
        )
        angular_body = _inverse_rotate_wxyz(
            root_quaternion, _single_row(angular_world, 3, "root angular velocities")
        )

        dof_positions = _single_row(
            self.articulation.get_dof_positions(), len(self._dof_names), "DOF positions"
        )
        dof_velocities = _single_row(
            self.articulation.get_dof_velocities(), len(self._dof_names), "DOF velocities"
        )
        joint_position = dict(zip(self._dof_names, dof_positions))
        joint_velocity = dict(zip(self._dof_names, dof_velocities))

        link_positions_raw, link_orientations_raw = self.link_poses.get_world_poses()
        link_positions = _matrix(link_positions_raw, len(self._link_names), 3, "link positions")
        link_orientations = tuple(
            _normalized_quaternion_wxyz(row, f"link orientation {index}")
            for index, row in enumerate(
                _matrix(link_orientations_raw, len(self._link_names), 4, "link orientations")
            )
        )
        masses = self._read_link_masses()
        local_coms = self._read_link_coms()
        total_mass = sum(masses)
        if total_mass <= 1.0e-9:
            raise FeedbackUnavailableError("articulation link mass sum is not positive")
        world_coms = tuple(
            _transform_point(link_positions[index], link_orientations[index], local_coms[index])
            for index in range(len(self._link_names))
        )
        center_of_mass = tuple(
            sum(masses[index] * world_coms[index][axis] for index in range(len(masses))) / total_mass
            for axis in range(3)
        )

        left_foot = self._foot(
            self._left_link_index, link_positions, link_orientations
        )
        right_foot = self._foot(
            self._right_link_index, link_positions, link_orientations
        )
        left_contact, left_points = self._contact(
            self.left_contact_sensor, "left", timestamp_s
        )
        right_contact, right_points = self._contact(
            self.right_contact_sensor, "right", timestamp_s
        )
        raw_support_points: list[tuple[float, float, float]] = []
        if left_contact:
            raw_support_points.extend(left_points)
        if right_contact:
            raw_support_points.extend(right_points)
        self.last_support_diagnostics = {
            "raw_point_count": len(raw_support_points),
            "raw_points_world_m": [list(point) for point in raw_support_points],
            "raw_distinct_xy_count": len({(point[0], point[1]) for point in raw_support_points}),
            "source": None,
            "support_points_world_m": [],
        }
        if not raw_support_points:
            self.last_support_diagnostics["error"] = "neither foot reports contact"
            raise FeedbackUnavailableError("neither foot reports contact; support polygon unavailable")
        try:
            support_center_xy, support_margin = support_polygon_center_and_margin(
                raw_support_points, center_of_mass[:2]
            )
            support_points = raw_support_points
            self.last_support_diagnostics["source"] = "measured_raw_contact_hull"
        except FeedbackUnavailableError as raw_error:
            self.last_support_diagnostics["raw_hull_error"] = str(raw_error)
            try:
                support_points = self._contact_conditioned_support_points(
                    left_contact=left_contact,
                    left_points=left_points,
                    right_contact=right_contact,
                    right_points=right_points,
                    link_positions=link_positions,
                    link_orientations=link_orientations,
                )
                support_center_xy, support_margin = support_polygon_center_and_margin(
                    support_points, center_of_mass[:2]
                )
                self.last_support_diagnostics["source"] = "contact_conditioned_urdf_inset"
            except FeedbackUnavailableError as fallback_error:
                self.last_support_diagnostics["error"] = str(fallback_error)
                raise FeedbackUnavailableError(
                    f"measured support geometry is insufficient ({raw_error}); "
                    f"contact-conditioned URDF support is unavailable ({fallback_error})"
                ) from fallback_error
        self.last_support_diagnostics["support_points_world_m"] = [
            list(point) for point in support_points
        ]
        self.last_support_diagnostics["support_center_world_xy_m"] = list(support_center_xy)
        self.last_support_diagnostics["support_margin_m"] = support_margin
        support_center = (
            support_center_xy[0],
            support_center_xy[1],
            sum(point[2] for point in support_points) / len(support_points),
        )

        return LocomotionFeedback(
            timestamp_s=timestamp_s,
            root_pose=PlanarPose(root_position[0], root_position[1], yaw),
            root_height_m=root_position[2],
            root_tilt_roll_pitch_rad=(roll, pitch),
            root_linear_velocity_body_mps=linear_body,
            root_angular_velocity_body_rps=angular_body,
            left_foot=FootFeedback(left_foot, left_contact),
            right_foot=FootFeedback(right_foot, right_contact),
            joint_position_rad=joint_position,
            joint_velocity_rad_s=joint_velocity,
            com_position_world_m=center_of_mass,  # type: ignore[arg-type]
            support_center_world_m=support_center,
            support_margin_m=support_margin,
        )

    def _contact_conditioned_support_points(
        self,
        *,
        left_contact: bool,
        left_points: Sequence[Sequence[float]],
        right_contact: bool,
        right_points: Sequence[Sequence[float]],
        link_positions: Sequence[Sequence[float]],
        link_orientations: Sequence[Sequence[float]],
    ) -> list[tuple[float, float, float]]:
        """Infer a strict subset of a flat contacting sole's URDF footprint.

        This is used only when PhysX contact reduction yields too few raw points
        for a polygon.  A foot qualifies only when its positive-force contact
        has a raw point near an authored collision sphere and all four measured
        sphere bottoms lie on the raw contact plane within 0.5 mm.
        """

        feet = (
            ("left", left_contact, left_points, self._left_link_index),
            ("right", right_contact, right_points, self._right_link_index),
        )
        inferred: list[tuple[float, float, float]] = []
        foot_diagnostics: dict[str, object] = {}
        for side, in_contact, raw_points, link_index in feet:
            if not in_contact:
                continue
            force_n = self.last_contact_forces_n.get(side, 0.0)
            vertices = tuple(
                _transform_point(
                    link_positions[link_index], link_orientations[link_index], vertex
                )
                for vertex in self.sole_geometry.support_vertices_m
            )
            contact_plane_z = sum(point[2] for point in raw_points) / len(raw_points)
            raw_z_spread = max(point[2] for point in raw_points) - min(
                point[2] for point in raw_points
            )
            maximum_plane_error = max(
                abs(vertex[2] - contact_plane_z) for vertex in vertices
            )
            maximum_nearest_sphere_xy_distance = max(
                min(
                    math.hypot(point[0] - vertex[0], point[1] - vertex[1])
                    for vertex in vertices
                )
                for point in raw_points
            )
            gate = {
                "force_n": force_n,
                "raw_points_world_m": [list(point) for point in raw_points],
                "nominal_sphere_bottoms_world_m": [list(vertex) for vertex in vertices],
                "contact_plane_z_m": contact_plane_z,
                "raw_z_spread_m": raw_z_spread,
                "maximum_sphere_bottom_plane_error_m": maximum_plane_error,
                "maximum_raw_point_to_sphere_xy_distance_m": maximum_nearest_sphere_xy_distance,
                "plane_tolerance_m": self.support_plane_tolerance_m,
                "contact_sphere_radius_m": self.sole_geometry.contact_sphere_radius_m,
                "inset_fraction": self.inferred_support_inset_fraction,
            }
            foot_diagnostics[side] = gate
            if force_n <= 0.0:
                gate["error"] = "contact force is not positive"
                self.last_support_diagnostics["contact_conditioned_feet"] = foot_diagnostics
                raise FeedbackUnavailableError(f"{side} contact force is not positive")
            if raw_z_spread > self.support_plane_tolerance_m:
                gate["error"] = "raw points do not share one contact plane"
                self.last_support_diagnostics["contact_conditioned_feet"] = foot_diagnostics
                raise FeedbackUnavailableError(f"{side} raw contact plane is not flat")
            if maximum_plane_error > self.support_plane_tolerance_m:
                gate["error"] = "authored sphere bottoms are not on the measured contact plane"
                self.last_support_diagnostics["contact_conditioned_feet"] = foot_diagnostics
                raise FeedbackUnavailableError(f"{side} sole is not coplanar with contact")
            if maximum_nearest_sphere_xy_distance > (
                self.sole_geometry.contact_sphere_radius_m + self.support_plane_tolerance_m
            ):
                gate["error"] = "raw point does not match an authored contact sphere"
                self.last_support_diagnostics["contact_conditioned_feet"] = foot_diagnostics
                raise FeedbackUnavailableError(f"{side} raw contact is outside its sole spheres")

            center_xy, _margin = support_polygon_center_and_margin(vertices, vertices[0][:2])
            inset_vertices = [
                (
                    center_xy[0]
                    + self.inferred_support_inset_fraction * (vertex[0] - center_xy[0]),
                    center_xy[1]
                    + self.inferred_support_inset_fraction * (vertex[1] - center_xy[1]),
                    contact_plane_z,
                )
                for vertex in vertices
            ]
            gate["inferred_support_points_world_m"] = [
                list(point) for point in inset_vertices
            ]
            gate["status"] = "accepted"
            inferred.extend(inset_vertices)

        self.last_support_diagnostics["contact_conditioned_feet"] = foot_diagnostics
        if not inferred:
            raise FeedbackUnavailableError("no contacting foot passed support inference")
        return inferred

    def _read_link_masses(self) -> tuple[float, ...]:
        raw = _plain(self.articulation.get_link_masses())
        if not isinstance(raw, (list, tuple)) or len(raw) != 1:
            raise FeedbackUnavailableError(
                f"link masses must have shape (1, {len(self._link_names)})"
            )
        row = _plain(raw[0])
        if not isinstance(row, (list, tuple)) or len(row) != len(self._link_names):
            raise FeedbackUnavailableError(
                f"link masses must have shape (1, {len(self._link_names)})"
            )
        masses: list[float] = []
        for value in row:
            plain = _plain(value)
            if isinstance(plain, (list, tuple)):
                if len(plain) != 1:
                    raise FeedbackUnavailableError("link mass entry has invalid shape")
                plain = plain[0]
            mass = float(plain)
            if not math.isfinite(mass) or mass < 0.0:
                raise FeedbackUnavailableError("link masses must be finite and nonnegative")
            masses.append(mass)
        return tuple(masses)

    def _read_link_coms(self) -> tuple[tuple[float, float, float], ...]:
        positions, _orientations = self.articulation.get_link_coms()
        raw = _plain(positions)
        if not isinstance(raw, (list, tuple)) or len(raw) != 1:
            raise FeedbackUnavailableError(
                f"link COM positions must have shape (1, {len(self._link_names)}, 3)"
            )
        return _matrix(raw[0], len(self._link_names), 3, "link COM positions")

    def _foot(
        self,
        link_index: int,
        link_positions: Sequence[Sequence[float]],
        link_orientations: Sequence[Sequence[float]],
    ) -> FootPose:
        position = link_positions[link_index]
        orientation = link_orientations[link_index]
        sole_position = _transform_point(position, orientation, self.sole_geometry.reference_m)
        yaw = _roll_pitch_yaw_wxyz(orientation)[2]
        return FootPose(sole_position, yaw)

    @staticmethod
    def _raw_contact_position(contact: object, side: str) -> tuple[float, float, float]:
        if not isinstance(contact, dict) or "position" not in contact:
            raise FeedbackUnavailableError(f"{side} raw contact has no position")
        position = contact["position"]
        if isinstance(position, dict):
            try:
                position = [position[axis] for axis in ("x", "y", "z")]
            except KeyError as exc:
                raise FeedbackUnavailableError(
                    f"{side} raw contact position is missing {exc.args[0]}"
                ) from exc
        return _finite_vector(position, 3, f"{side} raw contact position")  # type: ignore[return-value]
