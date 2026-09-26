"""CPU-safe URDF validation and deterministic robot configuration loading.

This module deliberately has no Isaac Sim imports. It validates immutable
inputs before an importer or physics process starts and exposes a name-based
joint contract that does not depend on importer DOF ordering.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


PRODUCTION_ROOT = Path(__file__).resolve().parent


class ModelValidationError(ValueError):
    """The production robot description or configuration is inconsistent."""


class JointTargetError(ValueError):
    """A requested joint target cannot safely be sent to the articulation."""


def canonical_text_sha256(path: Path) -> str:
    """Hash text as UTF-8 with LF newlines, independent of checkout EOLs."""

    text = path.read_text(encoding="utf-8")
    data = text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _finite_float(value: str | int | float, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ModelValidationError(f"{label} must be numeric, got {value!r}") from exc
    if not math.isfinite(result):
        raise ModelValidationError(f"{label} must be finite, got {value!r}")
    return result


def _vector(values: Sequence[object], length: int, label: str) -> tuple[float, ...]:
    if len(values) != length:
        raise ModelValidationError(f"{label} must contain {length} values")
    return tuple(_finite_float(value, f"{label}[{index}]") for index, value in enumerate(values))


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    effort: float
    velocity: float

    def check(self, name: str, value: float) -> None:
        if not math.isfinite(value):
            raise JointTargetError(f"target for {name!r} must be finite")
        if value < self.lower or value > self.upper:
            raise JointTargetError(
                f"target for {name!r} is {value:.9g} rad; allowed range is "
                f"[{self.lower:.9g}, {self.upper:.9g}] rad"
            )


@dataclass(frozen=True)
class MeshReference:
    link: str
    role: str
    filename: str
    path: Path
    scale: tuple[float, float, float]


@dataclass(frozen=True)
class UrdfModel:
    path: Path
    robot_name: str
    root_link: str
    link_names: tuple[str, ...]
    joint_names: tuple[str, ...]
    dof_names: tuple[str, ...]
    joint_limits: Mapping[str, JointLimit]
    mesh_references: tuple[MeshReference, ...]
    links_without_collision: tuple[str, ...]
    zero_mass_links: tuple[str, ...]
    tiny_inertia_links: tuple[str, ...]

    @classmethod
    def load(cls, path: Path, asset_root: Path) -> "UrdfModel":
        try:
            robot = ET.parse(path).getroot()
        except (ET.ParseError, OSError) as exc:
            raise ModelValidationError(f"cannot parse URDF {path}: {exc}") from exc
        if robot.tag != "robot":
            raise ModelValidationError(f"URDF root must be <robot>, got <{robot.tag}>")

        links = robot.findall("link")
        joints = robot.findall("joint")
        link_names = tuple(link.attrib.get("name", "") for link in links)
        joint_names = tuple(joint.attrib.get("name", "") for joint in joints)
        _require_unique_nonempty(link_names, "link")
        _require_unique_nonempty(joint_names, "joint")
        link_set = set(link_names)

        child_to_joint: dict[str, str] = {}
        child_joint_types: dict[str, str] = {}
        adjacency: dict[str, list[str]] = {name: [] for name in link_names}
        limits: dict[str, JointLimit] = {}
        dof_names: list[str] = []
        for joint in joints:
            name = joint.attrib["name"]
            joint_type = joint.attrib.get("type")
            parent_element = joint.find("parent")
            child_element = joint.find("child")
            if parent_element is None or child_element is None:
                raise ModelValidationError(f"joint {name!r} lacks parent or child")
            parent = parent_element.attrib.get("link", "")
            child = child_element.attrib.get("link", "")
            if parent not in link_set or child not in link_set:
                raise ModelValidationError(f"joint {name!r} references an unknown link")
            if child in child_to_joint:
                raise ModelValidationError(
                    f"link {child!r} has multiple parents: {child_to_joint[child]!r} and {name!r}"
                )
            child_to_joint[child] = name
            child_joint_types[child] = str(joint_type)
            adjacency[parent].append(child)

            if joint_type == "fixed":
                continue
            if joint_type != "revolute":
                raise ModelValidationError(
                    f"joint {name!r} has unsupported production type {joint_type!r}; "
                    "finite revolute limits are required"
                )
            limit = joint.find("limit")
            if limit is None:
                raise ModelValidationError(f"revolute joint {name!r} has no limit")
            parsed = JointLimit(
                lower=_finite_float(limit.attrib.get("lower"), f"{name}.lower"),
                upper=_finite_float(limit.attrib.get("upper"), f"{name}.upper"),
                effort=_finite_float(limit.attrib.get("effort"), f"{name}.effort"),
                velocity=_finite_float(limit.attrib.get("velocity"), f"{name}.velocity"),
            )
            if parsed.lower > parsed.upper:
                raise ModelValidationError(f"joint {name!r} has lower limit above upper limit")
            if parsed.effort <= 0.0 or parsed.velocity <= 0.0:
                raise ModelValidationError(f"joint {name!r} must have positive effort and velocity limits")
            limits[name] = parsed
            dof_names.append(name)

        roots = sorted(link_set - set(child_to_joint))
        if len(roots) != 1:
            raise ModelValidationError(f"expected one URDF root link, found {roots}")
        visited: set[str] = set()
        pending = [roots[0]]
        while pending:
            link = pending.pop()
            if link in visited:
                raise ModelValidationError(f"cycle detected at link {link!r}")
            visited.add(link)
            pending.extend(adjacency[link])
        if visited != link_set:
            raise ModelValidationError(f"disconnected links: {sorted(link_set - visited)}")

        resolved_asset_root = asset_root.resolve()
        mesh_references: list[MeshReference] = []
        for link in links:
            link_name = link.attrib["name"]
            for role in ("visual", "collision"):
                mesh_elements = [
                    geometry
                    for element in link.findall(role)
                    for geometry in [element.find("geometry/mesh")]
                    if geometry is not None
                ]
                for mesh in mesh_elements:
                    filename = mesh.attrib.get("filename", "")
                    if not filename or filename.startswith("package://"):
                        raise ModelValidationError(
                            f"invalid mesh URI on {link_name!r}: {filename!r}"
                        )
                    mesh_path = (path.parent / filename).resolve()
                    if not mesh_path.is_relative_to(resolved_asset_root):
                        raise ModelValidationError(
                            f"mesh on {link_name!r} escapes asset root: {filename!r}"
                        )
                    if not mesh_path.is_file() or mesh_path.stat().st_size == 0:
                        raise ModelValidationError(
                            f"missing or empty mesh on {link_name!r}: {filename!r}"
                        )
                    scale_text = mesh.attrib.get("scale", "1 1 1")
                    scale = tuple(
                        _finite_float(value, f"mesh scale on {link_name}")
                        for value in scale_text.split()
                    )
                    if len(scale) != 3 or any(
                        value <= 0.0 or value > 1000.0 for value in scale
                    ):
                        raise ModelValidationError(
                            f"mesh scale on {link_name!r} must be three values in "
                            f"(0, 1000], got {scale_text!r}"
                        )
                    mesh_references.append(
                        MeshReference(link_name, role, filename, mesh_path, scale)
                    )

        links_without_collision = tuple(
            link.attrib["name"] for link in links if not link.findall("collision")
        )
        zero_mass_links: list[str] = []
        tiny_inertia_links: list[str] = []
        for link in links:
            name = link.attrib["name"]
            inertial = link.find("inertial")
            if inertial is None:
                raise ModelValidationError(f"link {name!r} has no inertial element")
            mass_element = inertial.find("mass")
            inertia = inertial.find("inertia")
            if mass_element is None or inertia is None:
                raise ModelValidationError(f"link {name!r} has an incomplete inertial element")
            mass = _finite_float(mass_element.attrib.get("value"), f"{name}.mass")
            ixx, ixy, ixz, iyy, iyz, izz = tuple(
                _finite_float(inertia.attrib.get(axis), f"{name}.{axis}")
                for axis in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            )
            diagonal = (ixx, iyy, izz)
            if mass < 0.0 or any(value < 0.0 for value in diagonal):
                raise ModelValidationError(f"link {name!r} has negative mass or diagonal inertia")
            if mass > 0.0:
                leading_minor = ixx * iyy - ixy * ixy
                determinant = (
                    ixx * (iyy * izz - iyz * iyz)
                    - ixy * (ixy * izz - iyz * ixz)
                    + ixz * (ixy * iyz - iyy * ixz)
                )
                if ixx <= 0.0 or leading_minor <= 0.0 or determinant <= 0.0:
                    raise ModelValidationError(
                        f"link {name!r} inertia tensor is not positive definite"
                    )
            if mass == 0.0:
                if (
                    child_joint_types.get(name) != "fixed"
                    or adjacency[name]
                    or link.findall("visual")
                    or link.findall("collision")
                ):
                    raise ModelValidationError(
                        f"zero-mass link {name!r} is not a fixed geometry-free leaf marker"
                    )
                zero_mass_links.append(name)
            elif mass <= 0.001 and max(diagonal) <= 1e-9:
                if (
                    child_joint_types.get(name) != "revolute"
                    or link.findall("visual")
                    or link.findall("collision")
                ):
                    raise ModelValidationError(
                        f"tiny-inertia link {name!r} is not a geometry-free movable joint frame"
                    )
                tiny_inertia_links.append(name)

        return cls(
            path=path.resolve(),
            robot_name=robot.attrib.get("name", ""),
            root_link=roots[0],
            link_names=link_names,
            joint_names=joint_names,
            dof_names=tuple(dof_names),
            joint_limits=limits,
            mesh_references=tuple(mesh_references),
            links_without_collision=links_without_collision,
            zero_mass_links=tuple(zero_mass_links),
            tiny_inertia_links=tuple(tiny_inertia_links),
        )


def _require_unique_nonempty(names: Iterable[str], kind: str) -> None:
    sequence = tuple(names)
    if any(not name for name in sequence):
        raise ModelValidationError(f"every {kind} must have a nonempty name")
    seen: set[str] = set()
    duplicates: set[str] = set()
    for name in sequence:
        if name in seen:
            duplicates.add(name)
        seen.add(name)
    if duplicates:
        raise ModelValidationError(f"duplicate {kind} names: {sorted(duplicates)}")


@dataclass(frozen=True)
class ProductionRobotSpec:
    root: Path
    model: UrdfModel
    canonical_dof_order: tuple[str, ...]
    root_position_m: tuple[float, float, float]
    root_orientation_wxyz: tuple[float, float, float, float]
    reset_joint_positions: Mapping[str, float]
    importer: Mapping[str, object]
    assumptions: Mapping[str, object]

    def validate_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        unknown = sorted(set(targets) - set(self.canonical_dof_order))
        if unknown:
            raise JointTargetError(f"unknown joint targets: {unknown}")
        checked: dict[str, float] = {}
        for name, raw_value in targets.items():
            try:
                value = float(raw_value)
            except (TypeError, ValueError) as exc:
                raise JointTargetError(f"target for {name!r} must be numeric") from exc
            self.model.joint_limits[name].check(name, value)
            checked[name] = value
        return checked

    def bind_runtime_dofs(self, runtime_dof_names: Sequence[str]) -> dict[str, int]:
        names = tuple(runtime_dof_names)
        if len(names) != len(set(names)):
            raise ModelValidationError("runtime articulation reports duplicate DOF names")
        expected = set(self.canonical_dof_order)
        actual = set(names)
        if expected != actual:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            raise ModelValidationError(
                f"runtime DOF set differs from production model; missing={missing}, extra={extra}"
            )
        return {name: index for index, name in enumerate(names)}

    def command_plan(
        self, runtime_dof_names: Sequence[str], targets: Mapping[str, float]
    ) -> tuple[tuple[int, ...], tuple[float, ...], tuple[str, ...]]:
        mapping = self.bind_runtime_dofs(runtime_dof_names)
        checked = self.validate_targets(targets)
        ordered_names = tuple(name for name in self.canonical_dof_order if name in checked)
        return (
            tuple(mapping[name] for name in ordered_names),
            tuple(checked[name] for name in ordered_names),
            ordered_names,
        )


def load_production_spec(root: Path | None = None) -> ProductionRobotSpec:
    root = (root or PRODUCTION_ROOT).resolve()
    config_path = root / "robot_config.json"
    manifest_path = root / "production_manifest.json"
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelValidationError(f"cannot load production robot metadata: {exc}") from exc

    source_root = root.parent
    source_path = (root / config["source_urdf"]).resolve()
    urdf_path = (root / config["production_urdf"]).resolve()
    expected_source = config["source_urdf_canonical_sha256"]
    if canonical_text_sha256(source_path) != expected_source:
        raise ModelValidationError("approved combined source URDF hash does not match configuration")
    if manifest.get("source_urdf_canonical_sha256") != expected_source:
        raise ModelValidationError("production manifest records a different source URDF hash")
    output_hash = canonical_text_sha256(urdf_path)
    if manifest.get("production_urdf_canonical_sha256") != output_hash:
        raise ModelValidationError("production URDF hash does not match production manifest")

    model = UrdfModel.load(urdf_path, source_root)
    canonical = tuple(config["canonical_dof_order"])
    _require_unique_nonempty(canonical, "canonical DOF")
    if set(canonical) != set(model.dof_names):
        raise ModelValidationError("canonical DOF order does not match URDF revolute joints")

    reset = config["reset"]
    root_position = _vector(reset["root_position_m"], 3, "reset.root_position_m")
    orientation = _vector(reset["root_orientation_wxyz"], 4, "reset.root_orientation_wxyz")
    if not math.isclose(sum(value * value for value in orientation), 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise ModelValidationError("reset root quaternion must have unit norm")
    joint_positions = {name: float(value) for name, value in reset["joint_positions_rad"].items()}

    spec = ProductionRobotSpec(
        root=root,
        model=model,
        canonical_dof_order=canonical,
        root_position_m=root_position,
        root_orientation_wxyz=orientation,
        reset_joint_positions=joint_positions,
        importer=dict(config["isaac_importer"]),
        assumptions=dict(config["model_assumptions"]),
    )
    if set(joint_positions) != set(canonical):
        raise ModelValidationError("reset joint map must cover every production DOF exactly once")
    spec.validate_targets(joint_positions)

    expected = config["expected_model"]
    actual_counts = {
        "link_count": len(model.link_names),
        "joint_count": len(model.joint_names),
        "dof_count": len(model.dof_names),
        "mesh_reference_count": len(model.mesh_references),
    }
    for key, actual in actual_counts.items():
        if int(expected[key]) != actual:
            raise ModelValidationError(f"{key} changed: expected {expected[key]}, got {actual}")
    required_links = set(expected["required_links"])
    if not required_links <= set(model.link_names):
        raise ModelValidationError(f"required robot links missing: {sorted(required_links - set(model.link_names))}")
    if model.root_link != expected["root_link"]:
        raise ModelValidationError(
            f"root link changed: expected {expected['root_link']!r}, got {model.root_link!r}"
        )

    assumptions = config["model_assumptions"]
    if model.zero_mass_links != tuple(assumptions["zero_mass_fixed_marker_links"]):
        raise ModelValidationError("zero-mass link classification differs from documented assumption")
    if model.tiny_inertia_links != tuple(assumptions["tiny_joint_frame_inertia_links"]):
        raise ModelValidationError("tiny-inertia link classification differs from documented assumption")
    if model.links_without_collision != tuple(assumptions["links_without_collision_geometry"]):
        raise ModelValidationError("collision-geometry gaps differ from documented assumption")
    if spec.importer.get("fix_base") is not False:
        raise ModelValidationError("production biped importer must use a non-fixed base")
    return spec


def validate_mesh_geometry(
    model: UrdfModel,
    *,
    minimum_extent_m: float = 1e-6,
    maximum_extent_m: float = 1.0,
) -> dict[str, tuple[float, float, float]]:
    """Load every unique STL and validate finite meter-scale geometry."""

    import numpy as np
    import trimesh

    extents: dict[str, tuple[float, float, float]] = {}
    for reference in model.mesh_references:
        key = str(reference.path)
        if key in extents:
            continue
        mesh = trimesh.load_mesh(reference.path, process=False)
        vertices = np.asarray(mesh.vertices, dtype=float)
        if vertices.size == 0 or not np.isfinite(vertices).all():
            raise ModelValidationError(f"mesh has empty or non-finite vertices: {reference.path}")
        scaled = np.asarray(mesh.extents, dtype=float) * np.asarray(reference.scale, dtype=float)
        if not np.isfinite(scaled).all():
            raise ModelValidationError(f"mesh has non-finite extents: {reference.path}")
        if float(scaled.max()) > maximum_extent_m or float(scaled.max()) < minimum_extent_m:
            raise ModelValidationError(
                f"mesh scale is outside [{minimum_extent_m}, {maximum_extent_m}] m: "
                f"{reference.path} -> {scaled.tolist()}"
            )
        extents[key] = tuple(float(value) for value in scaled)
    return extents
