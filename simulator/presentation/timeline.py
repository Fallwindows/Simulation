"""Validated presentation timing and input provenance contracts.

The storyboard images control composition, but they are never accepted as
render inputs.  A complete presentation must reference sensor-derived RGB,
LiDAR, pose, map, and object-reconstruction products through the role contracts
declared in the plan.  Diagnostic baselines may be shown only as explicitly
labelled fallbacks.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .provenance import (
    ROLE_SPECS,
    coherence_errors,
    storyboard_hashes,
    validate_diagnostic_identity,
    validate_role,
)


EXPECTED_SHOT_BOUNDARIES = (
    (0, 90),
    (90, 180),
    (180, 300),
    (300, 420),
    (420, 540),
    (540, 660),
    (660, 750),
    (750, 840),
    (840, 960),
    (960, 1080),
    (1080, 1200),
    (1200, 1350),
)


def _mapping(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON mapping")
    return value


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a positive integer")
    result = int(value)
    if result <= 0 or result != value:
        raise ValueError(f"{name} must be a positive integer")
    return result


@dataclass(frozen=True)
class RenderProfile:
    name: str
    width: int
    height: int
    fps: int
    frame_count: int
    label: str


@dataclass(frozen=True)
class RoleContract:
    name: str
    description: str
    schema_id: str
    producer_id: str
    required_artifacts: tuple[str, ...]


@dataclass(frozen=True)
class DiagnosticFallback:
    role: str
    source_start_frame: int


@dataclass(frozen=True)
class Shot:
    number: int
    slug: str
    title: str
    start_frame: int
    end_frame_exclusive: int
    render_role: str
    required_roles: tuple[str, ...]
    diagnostic_fallback: DiagnosticFallback | None

    @property
    def frame_count(self) -> int:
        return self.end_frame_exclusive - self.start_frame


@dataclass(frozen=True)
class PresentationPlan:
    path: Path
    fps: int
    duration_seconds: int
    frame_count: int
    profiles: dict[str, RenderProfile]
    role_contracts: dict[str, RoleContract]
    shots: tuple[Shot, ...]

    def shot_for_frame(self, frame_index: int) -> Shot:
        if frame_index < 0 or frame_index >= self.frame_count:
            raise IndexError(frame_index)
        for shot in self.shots:
            if shot.start_frame <= frame_index < shot.end_frame_exclusive:
                return shot
        raise RuntimeError(f"frame {frame_index} is not covered by the timeline")


@dataclass(frozen=True)
class RoleInput:
    name: str
    contract: str
    status: str
    provenance: str
    capture_id: str
    artifacts: dict[str, Path]
    artifact_sha256: dict[str, str]
    map_version: str | None
    object_state_version: str | None


@dataclass(frozen=True)
class InputReport:
    manifest_path: Path
    label: str
    roles: dict[str, RoleInput]
    ready_genuine_roles: frozenset[str]
    role_errors: dict[str, tuple[str, ...]]
    missing_by_shot: dict[int, tuple[str, ...]]
    storyboard_manifest_sha256: str
    provenance_validated: bool

    @property
    def complete(self) -> bool:
        return not any(self.missing_by_shot.values())


def load_plan(path: str | Path) -> PresentationPlan:
    source = Path(path).resolve()
    data = _mapping(source)
    if int(data.get("schema_version", -1)) != 1:
        raise ValueError("unsupported presentation plan schema_version")

    fps = _positive_int(data.get("fps"), "fps")
    duration = _positive_int(data.get("duration_seconds"), "duration_seconds")
    frame_count = _positive_int(data.get("frame_count"), "frame_count")
    if (fps, duration, frame_count) != (30, 45, 1350):
        raise ValueError("presentation must be exactly 30 fps, 45 seconds, and 1350 frames")
    if fps * duration != frame_count:
        raise ValueError("frame_count must equal fps * duration_seconds")

    profiles: dict[str, RenderProfile] = {}
    profiles_data = data.get("profiles")
    if not isinstance(profiles_data, dict):
        raise ValueError("profiles must be a mapping")
    for name, item in profiles_data.items():
        if not isinstance(item, dict):
            raise ValueError(f"profile {name} must be a mapping")
        profile = RenderProfile(
            str(name),
            _positive_int(item.get("width"), f"profiles.{name}.width"),
            _positive_int(item.get("height"), f"profiles.{name}.height"),
            _positive_int(item.get("fps"), f"profiles.{name}.fps"),
            _positive_int(item.get("frame_count"), f"profiles.{name}.frame_count"),
            str(item.get("label", "")),
        )
        if profile.fps != fps or profile.frame_count != frame_count:
            raise ValueError(f"profile {name} must use the plan fps and frame count")
        profiles[name] = profile
    required_profiles = {"preview": (1280, 720), "delivery": (1920, 1080)}
    for name, dimensions in required_profiles.items():
        profile = profiles.get(name)
        if profile is None or (profile.width, profile.height) != dimensions:
            raise ValueError(f"{name} profile must be {dimensions[0]}x{dimensions[1]}")

    contracts: dict[str, RoleContract] = {}
    contracts_data = data.get("role_contracts")
    if not isinstance(contracts_data, dict):
        raise ValueError("role_contracts must be a mapping")
    for name, item in contracts_data.items():
        if not isinstance(item, dict):
            raise ValueError(f"role contract {name} must be a mapping")
        spec = ROLE_SPECS.get(str(name))
        if spec is None:
            raise ValueError(f"unsupported role contract {name}")
        artifacts = item.get("required_artifacts")
        if not isinstance(artifacts, list) or not artifacts or not all(isinstance(value, str) and value for value in artifacts):
            raise ValueError(f"role contract {name} must list required_artifacts")
        expected_artifacts = tuple(artifact_name for artifact_name, _ in spec.required_artifacts)
        if tuple(artifacts) != expected_artifacts:
            raise ValueError(f"role contract {name} artifacts must match the built-in {spec.schema_id} schema")
        if item.get("schema_id") != spec.schema_id or item.get("producer_id") != spec.producer_id:
            raise ValueError(f"role contract {name} schema/producer does not match the repository stage")
        contracts[str(name)] = RoleContract(
            str(name),
            str(item.get("description", "")),
            spec.schema_id,
            spec.producer_id,
            expected_artifacts,
        )

    shots_data = data.get("shots")
    if not isinstance(shots_data, list) or len(shots_data) != 12:
        raise ValueError("the presentation must contain exactly 12 shots")
    shots: list[Shot] = []
    for index, item in enumerate(shots_data):
        if not isinstance(item, dict):
            raise ValueError(f"shot {index + 1} must be a mapping")
        number = _positive_int(item.get("number"), f"shots[{index}].number")
        if number != index + 1:
            raise ValueError("shots must be ordered and numbered 1 through 12")
        start = int(item.get("start_frame"))
        end = int(item.get("end_frame_exclusive"))
        if (start, end) != EXPECTED_SHOT_BOUNDARIES[index]:
            raise ValueError(f"shot {number:02d} must use boundary {EXPECTED_SHOT_BOUNDARIES[index]}")
        required_roles_value = item.get("required_roles")
        if not isinstance(required_roles_value, list) or not required_roles_value:
            raise ValueError(f"shot {number:02d} must declare required_roles")
        required_roles = tuple(str(value) for value in required_roles_value)
        unknown = sorted(set(required_roles) - set(contracts))
        if unknown:
            raise ValueError(f"shot {number:02d} uses unknown roles: {unknown}")
        render_role = str(item.get("render_role", ""))
        if render_role not in required_roles:
            raise ValueError(f"shot {number:02d} render_role must also be required")
        fallback_value = item.get("diagnostic_fallback")
        fallback = None
        if fallback_value is not None:
            if not isinstance(fallback_value, dict):
                raise ValueError(f"shot {number:02d} diagnostic_fallback must be a mapping")
            fallback = DiagnosticFallback(
                str(fallback_value["role"]),
                int(fallback_value.get("source_start_frame", start)),
            )
            if fallback.source_start_frame < 0:
                raise ValueError(f"shot {number:02d} fallback source frame must be nonnegative")
        shots.append(
            Shot(
                number,
                str(item.get("slug", "")),
                str(item.get("title", "")),
                start,
                end,
                render_role,
                required_roles,
                fallback,
            )
        )
    if sum(shot.frame_count for shot in shots) != frame_count:
        raise ValueError("shot frame counts do not total the presentation frame_count")
    return PresentationPlan(source, fps, duration, frame_count, profiles, contracts, tuple(shots))


def inspect_inputs(plan: PresentationPlan, path: str | Path) -> InputReport:
    source = Path(path).resolve()
    data = _mapping(source)
    if int(data.get("schema_version", -1)) != 2:
        raise ValueError("unsupported presentation input schema_version")
    if data.get("ground_truth_consumed") is not False:
        raise ValueError("presentation inputs must explicitly declare ground_truth_consumed=false")
    roles_data = data.get("roles")
    if not isinstance(roles_data, dict):
        raise ValueError("input roles must be a mapping")

    repo_root = plan.path.parents[2]
    if any(isinstance(item, dict) and item.get("provenance") == "diagnostic_baseline" for item in roles_data.values()):
        validate_diagnostic_identity(plan.path, source, repo_root)
    known_storyboard_hashes, storyboard_manifest_sha256 = storyboard_hashes(repo_root)
    validated_roles = {}
    for role_name, item in roles_data.items():
        if not isinstance(item, dict):
            raise ValueError(f"input role {role_name} must be a mapping")
        contract_name = str(item.get("contract", ""))
        if contract_name not in plan.role_contracts:
            raise ValueError(f"input role {role_name} uses unknown contract {contract_name}")
        validated_roles[str(role_name)] = validate_role(
            str(role_name), item, source, repo_root, known_storyboard_hashes
        )

    coherence = coherence_errors(validated_roles)
    roles: dict[str, RoleInput] = {}
    errors: dict[str, tuple[str, ...]] = {}
    ready: set[str] = set()
    for name, validated in validated_roles.items():
        combined_errors = tuple(validated.errors) + tuple(coherence.get(name, ()))
        role = RoleInput(
            validated.name,
            validated.contract,
            validated.status,
            validated.provenance,
            validated.capture_id,
            {artifact_name: artifact.path for artifact_name, artifact in validated.artifacts.items()},
            {artifact_name: artifact.sha256 for artifact_name, artifact in validated.artifacts.items()},
            validated.map_version,
            validated.object_state_version,
        )
        roles[name] = role
        errors[name] = combined_errors
        if not combined_errors:
            ready.add(validated.contract)

    missing_by_shot = {
        shot.number: tuple(role for role in shot.required_roles if role not in ready)
        for shot in plan.shots
    }
    return InputReport(
        source,
        str(data.get("label", "")),
        roles,
        frozenset(ready),
        errors,
        missing_by_shot,
        storyboard_manifest_sha256,
        True,
    )
