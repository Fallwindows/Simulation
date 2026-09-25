"""Validated presentation timing and input provenance contracts.

The storyboard images control composition, but they are never accepted as
render inputs.  A complete presentation must reference sensor-derived RGB,
LiDAR, pose, map, and object-reconstruction products through the role contracts
declared in the plan.  Diagnostic baselines may be shown only as explicitly
labelled fallbacks.
"""

from __future__ import annotations

import hashlib
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

SMOOTH_TRANSITION_BOUNDARIES = (540, 660, 750, 840, 960, 1080, 1200)
LEGACY_DIAGNOSTIC_PLAN_RELATIVE = "config/presentation/diagnostic_storyboard_legacy.yaml"
LEGACY_DIAGNOSTIC_PLAN_SHA256_LF = "5a7e98db32134ba8f6507e60534a056bf9e5e35f3290d61eb9fa6b684479bc5f"
REPLAY_DISCLOSURE_TEXT = "EARLIER SENSOR REPLAY · SOURCE t=02.00–05.90 s"
REPLAY_DISCLOSURE_SOURCE_RANGE_S = (2.0, 5.9)


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


def _lf_text_sha256(path: Path) -> str:
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


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
class TransitionSampling:
    mode: str
    frame_count: int
    source_frame_start: int | None
    source_frame_end_exclusive: int | None
    reason: str | None


@dataclass(frozen=True)
class Transition:
    from_shot: int
    to_shot: int
    boundary_frame: int
    duration_frames: int
    style: str
    easing: str
    intent: str
    outgoing_sampling: TransitionSampling
    incoming_sampling: TransitionSampling

    @property
    def half_duration_frames(self) -> int:
        return self.duration_frames // 2

    @property
    def start_frame(self) -> int:
        return self.boundary_frame - self.half_duration_frames

    @property
    def end_frame_exclusive(self) -> int:
        return self.boundary_frame + self.half_duration_frames


@dataclass(frozen=True)
class EditorialDisclosure:
    id: str
    text: str
    start_frame: int
    end_frame_exclusive: int
    source_time_range_s: tuple[float, float]
    source_time_basis: str


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
    transitions: tuple[Transition, ...]
    editorial_disclosures: tuple[EditorialDisclosure, ...]

    def shot_for_frame(self, frame_index: int) -> Shot:
        if frame_index < 0 or frame_index >= self.frame_count:
            raise IndexError(frame_index)
        for shot in self.shots:
            if shot.start_frame <= frame_index < shot.end_frame_exclusive:
                return shot
        raise RuntimeError(f"frame {frame_index} is not covered by the timeline")

    def transition_for_boundary(self, frame_index: int) -> Transition | None:
        return next((item for item in self.transitions if item.boundary_frame == frame_index), None)


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
class ShotRenderInput:
    shot_number: int
    video_path: Path
    video_sha256: str
    source_start_frame: int
    frame_count: int
    source_time_range_s: tuple[float, float]
    source_time_basis: str
    source_kind: str
    view_id: str
    receipt_path: Path
    receipt_sha256: str
    presentation_transform: dict[str, object] | None
    transition_boundary_frame: int | None
    transition_source_samples: tuple[tuple[int, float], ...]


@dataclass(frozen=True)
class InputReport:
    manifest_path: Path
    label: str
    presentation_classification: dict[str, str]
    roles: dict[str, RoleInput]
    ready_genuine_roles: frozenset[str]
    role_errors: dict[str, tuple[str, ...]]
    missing_by_shot: dict[int, tuple[str, ...]]
    storyboard_manifest_sha256: str
    provenance_validated: bool
    shot_sources: dict[int, ShotRenderInput]
    source_bindings: dict[str, Any]

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

    transition_policy = data.get("transition_policy")
    if transition_policy is None:
        normalized_source = source.as_posix().lower()
        if not normalized_source.endswith(LEGACY_DIAGNOSTIC_PLAN_RELATIVE.lower()) or _lf_text_sha256(source) != LEGACY_DIAGNOSTIC_PLAN_SHA256_LF:
            raise ValueError("production transition_policy is required; only the exact frozen legacy diagnostic plan may omit it")
        return PresentationPlan(source, fps, duration, frame_count, profiles, contracts, tuple(shots), (), ())
    if not isinstance(transition_policy, dict):
        raise ValueError("transition_policy must be a mapping")
    if transition_policy.get("mode") != "symmetric_boundary_blend_v1":
        raise ValueError("transition_policy mode must be symmetric_boundary_blend_v1")
    if transition_policy.get("frame_budget") != "transition windows straddle shot boundaries and consume no additional delivery frames":
        raise ValueError("transition_policy must preserve the exact delivery frame budget")
    transition_data = transition_policy.get("boundaries")
    if not isinstance(transition_data, list) or len(transition_data) != len(SMOOTH_TRANSITION_BOUNDARIES):
        raise ValueError("transition_policy must declare all seven technical boundaries")
    transitions: list[Transition] = []

    def transition_sampling(
        value: object,
        *,
        boundary_frame: int,
        half_duration_frames: int,
        side: str,
    ) -> TransitionSampling:
        if not isinstance(value, dict):
            raise ValueError(f"transition at frame {boundary_frame} must declare {side} source sampling")
        allowed = {"mode", "frame_count", "source_frame_start", "source_frame_end_exclusive", "reason"}
        if not set(value).issubset(allowed):
            raise ValueError(f"transition at frame {boundary_frame} {side} source sampling has unknown fields")
        mode = str(value.get("mode", ""))
        frame_count = _positive_int(
            value.get("frame_count"),
            f"transition at frame {boundary_frame} {side} source sampling frame_count",
        )
        if frame_count != half_duration_frames:
            raise ValueError(
                f"transition at frame {boundary_frame} {side} source sampling must cover one half-window"
            )
        reason_value = value.get("reason")
        reason = str(reason_value).strip() if reason_value is not None else None
        if mode == "edge_clone":
            if value.get("source_frame_start") is not None or value.get("source_frame_end_exclusive") is not None:
                raise ValueError("edge-clone sampling cannot declare a moving source range")
            return TransitionSampling(mode, frame_count, None, None, reason)
        if mode != "contiguous_postroll" or side != "outgoing":
            raise ValueError(
                f"transition at frame {boundary_frame} {side} source sampling mode is unsupported"
            )
        start = int(value.get("source_frame_start", -1))
        end = int(value.get("source_frame_end_exclusive", -1))
        if start != boundary_frame or end != boundary_frame + frame_count:
            raise ValueError(
                f"transition at frame {boundary_frame} contiguous post-roll must cover "
                f"source frames {boundary_frame}-{boundary_frame + frame_count - 1}"
            )
        return TransitionSampling(mode, frame_count, start, end, reason)

    for index, item in enumerate(transition_data):
        if not isinstance(item, dict):
            raise ValueError(f"transition {index} must be a mapping")
        from_shot = _positive_int(item.get("from_shot"), f"transitions[{index}].from_shot")
        to_shot = _positive_int(item.get("to_shot"), f"transitions[{index}].to_shot")
        boundary_frame = int(item.get("boundary_frame", -1))
        duration_frames = _positive_int(item.get("duration_frames"), f"transitions[{index}].duration_frames")
        expected_boundary = SMOOTH_TRANSITION_BOUNDARIES[index]
        if boundary_frame != expected_boundary:
            raise ValueError(f"transition {index} must use boundary {expected_boundary}")
        outgoing = shots[from_shot - 1] if 0 < from_shot <= len(shots) else None
        incoming = shots[to_shot - 1] if 0 < to_shot <= len(shots) else None
        if outgoing is None or incoming is None or to_shot != from_shot + 1:
            raise ValueError(f"transition at frame {boundary_frame} must join adjacent shots")
        if outgoing.end_frame_exclusive != boundary_frame or incoming.start_frame != boundary_frame:
            raise ValueError(f"transition at frame {boundary_frame} does not match its shot boundary")
        if duration_frames % 2 or duration_frames >= min(outgoing.frame_count, incoming.frame_count):
            raise ValueError(f"transition at frame {boundary_frame} must have a short positive even duration")
        style = str(item.get("style", ""))
        easing = str(item.get("easing", ""))
        intent = str(item.get("intent", "")).strip()
        if style != "smooth_crossfade" or easing != "smoothstep" or not intent:
            raise ValueError(f"transition at frame {boundary_frame} lacks the approved style, easing, or intent")
        sampling_value = item.get("source_sampling")
        if not isinstance(sampling_value, dict) or set(sampling_value) != {"outgoing", "incoming"}:
            raise ValueError(f"transition at frame {boundary_frame} must declare outgoing and incoming source sampling")
        outgoing_sampling = transition_sampling(
            sampling_value["outgoing"],
            boundary_frame=boundary_frame,
            half_duration_frames=duration_frames // 2,
            side="outgoing",
        )
        incoming_sampling = transition_sampling(
            sampling_value["incoming"],
            boundary_frame=boundary_frame,
            half_duration_frames=duration_frames // 2,
            side="incoming",
        )
        if boundary_frame == 540:
            if (
                outgoing_sampling.mode != "contiguous_postroll"
                or incoming_sampling.mode != "edge_clone"
                or not incoming_sampling.reason
            ):
                raise ValueError(
                    "transition at frame 540 requires moving RGB post-roll and a reasoned technical start clone"
                )
            if "earlier offline sensor replay" not in intent.lower() or "non-co-timed editorial blend" not in intent.lower():
                raise ValueError(
                    "transition at frame 540 must identify the earlier offline replay and non-co-timed editorial blend"
                )
        elif outgoing_sampling.mode != "edge_clone" or incoming_sampling.mode != "edge_clone":
            raise ValueError(f"transition at frame {boundary_frame} must retain symmetric edge clones")
        transitions.append(
            Transition(
                from_shot,
                to_shot,
                boundary_frame,
                duration_frames,
                style,
                easing,
                intent,
                outgoing_sampling,
                incoming_sampling,
            )
        )
    for previous, current in zip(transitions, transitions[1:]):
        if previous.end_frame_exclusive > current.start_frame:
            raise ValueError("transition windows must not overlap")

    disclosure_data = data.get("editorial_disclosures")
    if not isinstance(disclosure_data, list) or len(disclosure_data) != 1:
        raise ValueError("production plan must declare the earlier sensor replay disclosure")
    disclosure_value = disclosure_data[0]
    allowed_disclosure_fields = {
        "id", "text", "start_frame", "end_frame_exclusive", "source_time_range_s", "source_time_basis",
    }
    if not isinstance(disclosure_value, dict) or set(disclosure_value) != allowed_disclosure_fields:
        raise ValueError("earlier sensor replay disclosure fields do not match the presentation contract")
    source_range_value = disclosure_value.get("source_time_range_s")
    if not isinstance(source_range_value, list) or len(source_range_value) != 2:
        raise ValueError("earlier sensor replay disclosure source_time_range_s must contain two values")
    source_range = tuple(float(value) for value in source_range_value)
    first_transition = transitions[0]
    disclosure = EditorialDisclosure(
        id=str(disclosure_value.get("id", "")),
        text=str(disclosure_value.get("text", "")),
        start_frame=int(disclosure_value.get("start_frame", -1)),
        end_frame_exclusive=int(disclosure_value.get("end_frame_exclusive", -1)),
        source_time_range_s=(source_range[0], source_range[1]),
        source_time_basis=str(disclosure_value.get("source_time_basis", "")),
    )
    if (
        disclosure.id != "earlier_sensor_replay"
        or disclosure.text != REPLAY_DISCLOSURE_TEXT
        or disclosure.start_frame != first_transition.start_frame
        or disclosure.end_frame_exclusive != shots[5].end_frame_exclusive
        or disclosure.source_time_range_s != REPLAY_DISCLOSURE_SOURCE_RANGE_S
        or disclosure.source_time_basis != "capture-relative simulation sensor time"
    ):
        raise ValueError(
            "earlier sensor replay disclosure must cover film frames 531-659 and identify source t=02.00-05.90 s"
        )
    return PresentationPlan(
        source,
        fps,
        duration,
        frame_count,
        profiles,
        contracts,
        tuple(shots),
        tuple(transitions),
        (disclosure,),
    )


def inspect_inputs(
    plan: PresentationPlan,
    path: str | Path,
    rgb_capture_catalog: str | Path | None = None,
    technical_source_catalog: str | Path | None = None,
    ffprobe: str = "ffprobe",
) -> InputReport:
    source = Path(path).resolve()
    data = _mapping(source)
    schema_version = int(data.get("schema_version", -1))
    repo_root = plan.path.parents[2]
    if schema_version == 4:
        from .complete_bundle import validate_complete_bundle

        validated = validate_complete_bundle(
            plan,
            source,
            repo_root,
            ffprobe,
            Path(rgb_capture_catalog).resolve() if rgb_capture_catalog else None,
            Path(technical_source_catalog).resolve() if technical_source_catalog else None,
        )
        _, storyboard_manifest_sha256 = storyboard_hashes(repo_root)
        shot_sources = {
            item.shot_number: ShotRenderInput(
                item.shot_number,
                item.video_path,
                item.video_sha256,
                item.source_start_frame,
                item.frame_count,
                item.source_time_range_s,
                item.source_time_basis,
                item.source_kind,
                item.view_id,
                item.receipt_path,
                item.receipt_sha256,
                item.presentation_transform,
                item.transition_boundary_frame,
                item.transition_source_samples,
            )
            for item in validated.shots
        }
        return InputReport(
            source,
            str(data.get("label", "")),
            validated.presentation_classification,
            {},
            frozenset(plan.role_contracts),
            {},
            {shot.number: () for shot in plan.shots},
            storyboard_manifest_sha256,
            True,
            shot_sources,
            validated.source_bindings,
        )
    if schema_version != 2:
        raise ValueError("unsupported presentation input schema_version")
    if data.get("ground_truth_consumed") is not False:
        raise ValueError("presentation inputs must explicitly declare ground_truth_consumed=false")
    roles_data = data.get("roles")
    if not isinstance(roles_data, dict):
        raise ValueError("input roles must be a mapping")

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
            str(role_name),
            item,
            source,
            repo_root,
            known_storyboard_hashes,
            Path(rgb_capture_catalog).resolve() if rgb_capture_catalog else None,
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
        {"kind": "diagnostic_or_incomplete"},
        roles,
        frozenset(ready),
        errors,
        missing_by_shot,
        storyboard_manifest_sha256,
        True,
        {},
        {},
    )
