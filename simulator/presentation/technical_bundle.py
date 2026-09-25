"""Validate and adapt technical-view delivery receipts for presentation shots 6-12."""

from __future__ import annotations

import json
import hashlib
import math
import struct
import subprocess
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np

from .provenance import (
    SHA256_PATTERN,
    sha256_path,
    storyboard_hashes,
    validate_presentation_classification,
)


TECHNICAL_PRODUCER_ID = "grocery_sim.technical_views.cpu.v1"
TECHNICAL_VIEWS = (
    (6, "sensor_activation", "lidar", 120),
    (7, "lidar_environment", "lidar", 90),
    (8, "persistent_map", "map", 90),
    (9, "object_association", "reconstruction", 120),
    (10, "object_detail", "reconstruction", 120),
    (11, "observed_aisle_overview", "reconstruction", 120),
    (12, "final_technical_view", "reconstruction", 150),
)

SELECTIVE_SCAN_GOAL = {
    "status": "complete",
    "representation": "feature-specific selections from timestamped raw PointCloud2 returns",
    "spatial_registration": "recorded camera/LiDAR calibration and estimated map-frame pose interpolation",
    "temporal_policy": "current scans or bounded past-only scan history; future scans are rejected",
    "ground_truth_consumed": False,
}

RAW_SOURCE_ARTIFACTS = (
    "raw_lidar_bag", "bag_metadata", "camera_info", "sensor_transforms", "effective_config",
    "scene_manifest", "rgb_video", "rgb_frames", "frame_annotations",
)
IMPLEMENTATION_SOURCES = (
    "simulator/technical_lidar.py",
    "simulator/sensors/scan_projection.py",
    "simulator/sensors/feature_selection.py",
)
PER_RETURN_TIMING = "absent_in_point_fields; rigid_header_stamp_projection_without_deskew"
SCAN_TIME_MODEL = {
    "per_return_timing": "absent_in_bound_PointCloud2_fields",
    "deskew": "not_applied",
    "rigid_pose_time": "PointCloud2 header/database timestamp",
}
CAMERA_TRACE_BASIS = (
    "shots 6-7 audit direct recorded camera_optical poses without applying a presentation guide; "
    "shots 8-12 apply a C3 presentation guide fit to the declared sampling interval; "
    "exact interpolation-knot dependencies are derived from the hash-bound trajectory; "
    "both remain independent of rendered scan cutoff"
)
CAMERA_TRACE_FIELDS = {
    "global_frame", "view_id", "view_frame", "boundary_from_previous", "camera_motion_role",
    "camera_guide_pose_timestamp_s", "rendered_data_cutoff_s", "motion_phase", "eye_m", "target_m", "eye_velocity_mps",
    "eye_acceleration_mps2", "eye_jerk_mps3", "target_velocity_mps",
    "target_acceleration_mps2", "target_jerk_mps3",
}


@dataclass(frozen=True)
class TechnicalShotSource:
    shot_number: int
    view_id: str
    presentation_role: str
    video_path: Path
    video_sha256: str
    frame_count: int
    source_time_range_s: tuple[float, float]
    receipt_path: Path
    receipt_sha256: str


@dataclass(frozen=True)
class ValidatedTechnicalDelivery:
    manifest_path: Path
    manifest_sha256: str
    capture_id: str
    capture_sha256: str
    source_id: str
    source: dict[str, Any]
    presentation_classification: dict[str, str]
    shots: tuple[TechnicalShotSource, ...]


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON mapping")
    return value


def _sha256_lf_text(path: Path) -> str:
    normalized = path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def _safe_child(root: Path, value: object, field: str) -> Path:
    text = str(value)
    pure = PurePosixPath(text)
    if not text or pure.is_absolute() or "\\" in text or any(part in ("", ".", "..") for part in text.split("/")):
        raise ValueError(f"technical delivery {field} must be a canonical relative path")
    path = (root / Path(*pure.parts)).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"technical delivery {field} escapes its manifest directory")
    return path


def _probe_video(path: Path, ffprobe: str) -> dict[str, Any]:
    result = subprocess.run(
        [
            ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,r_frame_rate,nb_read_frames", "-of", "json", str(path),
        ],
        text=True,
        capture_output=True,
    )
    if result.returncode:
        raise ValueError(f"ffprobe could not read technical video {path}: {result.stderr.strip()}")
    streams = json.loads(result.stdout).get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"technical video must contain one video stream: {path}")
    stream = streams[0]
    rate = Fraction(str(stream["r_frame_rate"]))
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps": float(rate),
        "frame_count": int(stream["nb_read_frames"]),
    }


def _require_git_commit(repo_root: Path, revision: object, producer: str) -> str:
    value = str(revision)
    if len(value) != 40:
        raise ValueError(f"technical {producer} revision must be a full Git SHA")
    result = subprocess.run(
        ["git", "-C", str(repo_root), "cat-file", "-e", f"{value}^{{commit}}"],
        capture_output=True,
    )
    if result.returncode:
        raise ValueError(f"technical {producer} revision is not a repository commit")
    return value


def _plain_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _raw_indices_sha256(indices: tuple[int, ...]) -> str:
    digest = hashlib.sha256()
    for value in indices:
        digest.update(struct.pack("<q", value))
    return digest.hexdigest()


def _verified_source_artifact(
    delivery_directory: Path,
    catalog_source: dict[str, Any],
    artifact_name: str,
) -> Path:
    """Resolve one catalog artifact from the delivery's source-run ancestry.

    Technical delivery validation depends on the original paired capture.  The
    catalog path is relative to that run, while the technical output is stored
    below the same run.  A nearer conflicting file is an integrity failure; it
    must not be skipped in favor of a matching copy higher in the tree.
    """

    artifact = catalog_source.get("artifacts", {}).get(artifact_name)
    if not isinstance(artifact, dict) or not SHA256_PATTERN.fullmatch(str(artifact.get("sha256", ""))):
        raise ValueError(f"technical catalog {artifact_name} binding is invalid")
    roots = (delivery_directory, *delivery_directory.parents)
    for candidate_root in roots:
        candidate = _safe_child(candidate_root, artifact.get("path"), f"catalog {artifact_name}.path")
        if not candidate.exists():
            continue
        if not candidate.is_file() or sha256_path(candidate) != artifact["sha256"]:
            raise ValueError(f"technical bound {artifact_name} artifact hash mismatch")
        return candidate
    raise ValueError(f"technical bound {artifact_name} artifact is unavailable")


def _verified_rgb_frame_index(
    path: Path,
    *,
    camera_frame_id: str,
) -> dict[int, int]:
    frames: dict[int, int] = {}
    prior_timestamp_ns = -1
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError("technical bound RGB frame index row is invalid")
            frame_index = row.get("frame_index")
            stamp_s = row.get("stamp_s")
            width = row.get("width")
            height = row.get("height")
            if (
                not _plain_int(frame_index)
                or frame_index != len(frames)
                or not isinstance(stamp_s, (int, float))
                or isinstance(stamp_s, bool)
                or not math.isfinite(float(stamp_s))
                or float(stamp_s) < 0.0
                or not _plain_int(width)
                or not _plain_int(height)
                or width <= 0
                or height <= 0
                or row.get("frame_id") != camera_frame_id
            ):
                raise ValueError("technical bound RGB frame index row is invalid")
            timestamp_ns = int(round(float(stamp_s) * 1_000_000_000))
            if timestamp_ns <= prior_timestamp_ns:
                raise ValueError("technical bound RGB frame timestamps are not strictly ordered")
            frames[frame_index] = timestamp_ns
            prior_timestamp_ns = timestamp_ns
    if not frames:
        raise ValueError("technical bound RGB frame index is empty")
    return frames


def _finite_vector3(value: object, label: str) -> tuple[float, float, float]:
    if (
        not isinstance(value, list)
        or len(value) != 3
        or not all(isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(float(item)) for item in value)
    ):
        raise ValueError(f"technical camera trace {label} must be a finite 3-vector")
    return tuple(float(item) for item in value)


def _camera_trace_view_sha256(rows: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        bound = {
            "global_frame": row["global_frame"],
            "view_frame": row["view_frame"],
            "camera_guide_pose_timestamp_s": row["camera_guide_pose_timestamp_s"],
            "rendered_data_cutoff_s": row["rendered_data_cutoff_s"],
            "eye_m": row["eye_m"],
            "target_m": row["target_m"],
        }
        digest.update(json.dumps(bound, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        digest.update(b"\n")
    return digest.hexdigest()


def _validate_camera_motion_trace(
    trace_info: object,
    *,
    directory: Path,
    plan_views: dict[str, dict[str, Any]],
    expected_order: list[str],
    expected_counts: dict[str, int],
    simulation_window_s: tuple[float, float],
    fps: int,
    expected_rows: list[dict[str, Any]],
    expected_dependency_windows_s: dict[str, list[float]],
) -> dict[str, list[dict[str, Any]]]:
    if not isinstance(trace_info, dict) or set(trace_info) != {
        "path", "sha256", "frame_count", "basis", "camera_pose_dependency_windows_s",
    }:
        raise ValueError("technical delivery camera motion trace receipt is missing or malformed")
    if trace_info.get("path") != "technical_camera_trace.jsonl":
        raise ValueError("technical delivery camera motion trace path is not canonical")
    trace_path = _safe_child(directory, trace_info.get("path"), "camera_motion_trace.path")
    if not trace_path.is_file() or sha256_path(trace_path) != trace_info.get("sha256"):
        raise ValueError("technical delivery camera motion trace hash mismatch")
    if trace_info.get("frame_count") != sum(expected_counts.values()) or trace_info.get("basis") != CAMERA_TRACE_BASIS:
        raise ValueError("technical delivery camera motion trace contract is invalid")
    if trace_info.get("camera_pose_dependency_windows_s") != expected_dependency_windows_s:
        raise ValueError("technical delivery camera motion trace dependency binding is invalid")
    rows: list[dict[str, Any]] = []
    with trace_path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict) or set(value) != CAMERA_TRACE_FIELDS:
                raise ValueError("technical camera motion trace row schema is invalid")
            rows.append(value)
    if len(rows) != trace_info["frame_count"]:
        raise ValueError("technical camera motion trace row count is invalid")
    if rows != expected_rows:
        raise ValueError("technical camera motion trace does not match deterministic source-derived camera motion")

    expected_sequence: list[tuple[str, int]] = []
    for view_id in expected_order:
        expected_sequence.extend((view_id, index) for index in range(expected_counts[view_id]))
    vectors: dict[str, list[tuple[float, float, float]]] = {
        name: [] for name in (
            "eye_m", "target_m", "eye_velocity_mps", "eye_acceleration_mps2", "eye_jerk_mps3",
            "target_velocity_mps", "target_acceleration_mps2", "target_jerk_mps3",
        )
    }
    by_view: dict[str, list[dict[str, Any]]] = {view_id: [] for view_id in expected_order}
    previous_timestamp = -math.inf
    previous_phase = -math.inf
    for global_frame, (row, (view_id, view_frame)) in enumerate(zip(rows, expected_sequence)):
        plan = plan_views.get(view_id, {})
        role = plan.get("camera_motion_role")
        guide_window = plan.get("camera_guide_timestamp_window_s")
        sampling_window = plan.get("camera_pose_sampling_window_s")
        timestamp = row.get("camera_guide_pose_timestamp_s")
        cutoff = row.get("rendered_data_cutoff_s")
        phase = row.get("motion_phase")
        display_window = plan.get("display_window_s")
        if plan.get("temporal_mode") == "current_window":
            expected_cutoff = float(display_window[0]) + (
                float(display_window[1]) - float(display_window[0])
            ) * (view_frame / expected_counts[view_id])
        elif plan.get("temporal_mode") == "past_only_reveal":
            progress = 0.0 if expected_counts[view_id] == 1 else view_frame / (expected_counts[view_id] - 1)
            eased = progress * progress * (3.0 - 2.0 * progress)
            expected_cutoff = float(display_window[0]) + (
                float(display_window[1]) - float(display_window[0])
            ) * eased
        else:
            expected_cutoff = float(display_window[1])
        if (
            row.get("global_frame") != global_frame
            or row.get("view_id") != view_id
            or row.get("view_frame") != view_frame
            or row.get("boundary_from_previous") is not (view_frame == 0 and global_frame > 0)
            or row.get("camera_motion_role") != role
            or not isinstance(guide_window, list)
            or len(guide_window) != 2
            or not isinstance(sampling_window, list)
            or len(sampling_window) != 2
            or not isinstance(timestamp, (int, float))
            or isinstance(timestamp, bool)
            or not math.isfinite(float(timestamp))
            or not isinstance(cutoff, (int, float))
            or isinstance(cutoff, bool)
            or not math.isfinite(float(cutoff))
            or not math.isclose(float(cutoff), expected_cutoff, abs_tol=1e-8)
            or not isinstance(phase, (int, float))
            or isinstance(phase, bool)
            or not math.isfinite(float(phase))
            or not (float(guide_window[0]) - 1e-9 <= float(timestamp) <= float(guide_window[1]) + 1e-9)
            or not (float(sampling_window[0]) - 1e-9 <= float(timestamp) <= float(sampling_window[1]) + 1e-9)
            or not (simulation_window_s[0] - 1e-9 <= float(timestamp) <= simulation_window_s[1] + 1e-9)
            or float(timestamp) < previous_timestamp - 1e-9
            or float(phase) < previous_phase - 1e-9
        ):
            raise ValueError(f"technical camera motion trace identity/time binding is invalid at frame {global_frame}")
        for name in vectors:
            vectors[name].append(_finite_vector3(row.get(name), name))
        if math.dist(vectors["eye_m"][-1], vectors["target_m"][-1]) <= 1e-6:
            raise ValueError(f"technical camera motion trace eye and target coincide at frame {global_frame}")
        by_view[view_id].append(row)
        previous_timestamp = float(timestamp)
        previous_phase = float(phase)

    def differences(values: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        result = [(0.0, 0.0, 0.0)]
        result.extend(tuple((right[axis] - left[axis]) * fps for axis in range(3)) for left, right in zip(values, values[1:]))
        return result

    expected_velocity = differences(vectors["eye_m"])
    expected_acceleration = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), *differences(expected_velocity)[2:]]
    expected_jerk = [
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        *differences(expected_acceleration)[3:],
    ]
    expected_target_velocity = differences(vectors["target_m"])
    expected_target_acceleration = [
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), *differences(expected_target_velocity)[2:],
    ]
    expected_target_jerk = [
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        *differences(expected_target_acceleration)[3:],
    ]
    for label, expected, recorded in (
        ("eye velocity", expected_velocity, vectors["eye_velocity_mps"]),
        ("eye acceleration", expected_acceleration, vectors["eye_acceleration_mps2"]),
        ("eye jerk", expected_jerk, vectors["eye_jerk_mps3"]),
        ("target velocity", expected_target_velocity, vectors["target_velocity_mps"]),
        ("target acceleration", expected_target_acceleration, vectors["target_acceleration_mps2"]),
        ("target jerk", expected_target_jerk, vectors["target_jerk_mps3"]),
    ):
        if any(math.dist(left, right) > 2e-3 for left, right in zip(expected, recorded)):
            raise ValueError(f"technical camera motion trace {label} is inconsistent with eye/target samples")

    final_plan = plan_views[expected_order[-1]]
    hold = final_plan.get("final_hold_frames")
    final_rows = by_view[expected_order[-1]]
    if not _plain_int(hold) or hold <= 0 or len(final_rows) < hold:
        raise ValueError("technical final camera hold contract is invalid")
    held = final_rows[-hold:]
    if any(row["eye_m"] != held[0]["eye_m"] or row["target_m"] != held[0]["target_m"] for row in held):
        raise ValueError("technical final camera hold is not stable")
    preceding = final_rows[-hold - 1]
    if preceding["eye_m"] == held[0]["eye_m"] and preceding["target_m"] == held[0]["target_m"]:
        raise ValueError("technical final camera hold exceeds the exact declared frame count")
    return by_view


def _expected_camera_motion(
    plan: dict[str, Any],
    rows: list[dict[str, Any]],
    focus_inventory: object,
    dependency_window_s: tuple[float, float],
) -> dict[str, Any]:
    timestamps = [float(row["camera_guide_pose_timestamp_s"]) for row in rows]
    cutoffs = [float(row["rendered_data_cutoff_s"]) for row in rows]
    path = (
        "direct recorded camera_optical pose audit; no presentation camera applied"
        if plan.get("camera_motion_role") == "recorded_camera_optical"
        else "continuous presentation orbit fitted to estimated map poses"
    )
    return {
        "role": plan.get("camera_motion_role"),
        "path": path,
        "render_application": (
            "not_applied; trace audits recorded camera pose for the timestamped sensor replay"
            if plan.get("camera_motion_role") == "recorded_camera_optical"
            else "applied to map-view projection"
        ),
        "pose_time_basis": CAMERA_TRACE_BASIS,
        "camera_guide_timestamp_window_s": plan.get("camera_guide_timestamp_window_s"),
        "camera_pose_sampling_window_s": plan.get("camera_pose_sampling_window_s"),
        "camera_pose_dependency_window_s": list(dependency_window_s),
        "camera_guide_pose_timestamp_range_s": [min(timestamps), max(timestamps)],
        "rendered_data_cutoff_range_s": [min(cutoffs), max(cutoffs)],
        "trace": "technical_camera_trace.jsonl",
        "trace_global_frame_range": [rows[0]["global_frame"], rows[-1]["global_frame"]],
        "trace_eye_target_sha256": _camera_trace_view_sha256(rows),
        "final_hold_frames": plan.get("final_hold_frames"),
        "map_focus": (
            None
            if plan.get("camera_motion_role") == "recorded_camera_optical"
            else {
                "track_id": int(getattr(focus_inventory, "track_id")),
                "position_m": [float(value) for value in getattr(focus_inventory, "position")],
                "source": "first deterministic selected row in hash-bound estimated inventory",
            }
        ),
    }


def _validate_scan_selection(
    selection: object,
    *,
    view_id: str,
    label: str,
    expected_mode: str,
    source_window_s: tuple[float, float],
    lidar_frame_id: str,
) -> tuple[dict[str, Any], ...]:
    if not isinstance(selection, dict) or set(selection) != {
        "scan_count", "time_range_s", "selected_return_count", "scans",
    }:
        raise ValueError(f"technical {label} schema is invalid for {view_id}")
    rows = selection.get("scans")
    if (
        not isinstance(rows, list)
        or not rows
        or not _plain_int(selection.get("scan_count"))
        or selection["scan_count"] != len(rows)
    ):
        raise ValueError(f"technical {label} scan list is invalid for {view_id}")
    validated: list[dict[str, Any]] = []
    prior_timestamp = -1
    message_ids: set[int] = set()
    timing_fields = {"time", "t", "timestamp", "offset_time", "time_offset", "timestamp_ns"}
    for row in rows:
        if not isinstance(row, dict) or set(row) != {
            "message_id", "timestamp_ns", "raw_point_count", "selected_return_count",
            "selected_raw_indices_sha256", "selection_mode", "source_frame_id", "point_fields",
            "per_return_timing",
        }:
            raise ValueError(f"technical {label} selected-return receipt row schema is invalid for {view_id}")
        fields = row.get("point_fields")
        message_id = row.get("message_id")
        timestamp_ns = row.get("timestamp_ns")
        raw_count = row.get("raw_point_count")
        selected_count = row.get("selected_return_count")
        if (
            not all(_plain_int(value) for value in (message_id, timestamp_ns, raw_count, selected_count))
            or message_id <= 0
            or timestamp_ns <= prior_timestamp
            or message_id in message_ids
            or raw_count <= 0
            or selected_count <= 0
            or selected_count > raw_count
            or not SHA256_PATTERN.fullmatch(str(row.get("selected_raw_indices_sha256", "")))
            or row.get("selection_mode") != expected_mode
            or row.get("source_frame_id") != lidar_frame_id
            or not isinstance(fields, list)
            or not all(isinstance(name, str) and name for name in fields)
            or len(fields) != len(set(fields))
            or not {"x", "y", "z"}.issubset(fields)
            or any(name in timing_fields for name in fields)
            or row.get("per_return_timing") != PER_RETURN_TIMING
        ):
            raise ValueError(f"technical {label} selected-return receipt row is invalid for {view_id}")
        timestamp_s = timestamp_ns / 1_000_000_000.0
        if not (source_window_s[0] - 1e-9 <= timestamp_s <= source_window_s[1] + 1e-9):
            raise ValueError(f"technical {label} scan timestamp lies outside the source window for {view_id}")
        prior_timestamp = timestamp_ns
        message_ids.add(message_id)
        validated.append(row)
    expected_count = sum(int(row["selected_return_count"]) for row in validated)
    if selection.get("selected_return_count") != expected_count:
        raise ValueError(f"technical {label} aggregate selected-return count is invalid for {view_id}")
    expected_range = [validated[0]["timestamp_ns"] / 1_000_000_000.0, validated[-1]["timestamp_ns"] / 1_000_000_000.0]
    if selection.get("time_range_s") != expected_range:
        raise ValueError(f"technical {label} aggregate time range is invalid for {view_id}")
    return tuple(validated)


def _catalog_source(catalog_path: Path, capture_id: str, source_id: str) -> dict[str, Any]:
    catalog = _json(catalog_path)
    if catalog.get("schema_version") != 1 or catalog.get("status") != "reviewed_source_catalog":
        raise ValueError("technical source catalog is not reviewed schema v1")
    matches = [
        item for item in catalog.get("sources", [])
        if isinstance(item, dict) and item.get("capture_id") == capture_id and item.get("source_id") == source_id
    ]
    if len(matches) != 1:
        raise ValueError("technical source is not present exactly once in the reviewed source catalog")
    return matches[0]


def _validate_source_binding(
    source: dict[str, Any],
    manifest: dict[str, Any],
    catalog_source: dict[str, Any],
    catalog_hash: str,
    repo_root: Path,
) -> None:
    if source.get("capture_id") != catalog_source.get("capture_id"):
        raise ValueError("technical source capture_id does not match its reviewed catalog")
    if source.get("source_id") != catalog_source.get("source_id"):
        raise ValueError("technical source_id does not match its reviewed catalog")
    revisions = source.get("producer_revisions")
    if revisions != catalog_source.get("producer_revisions"):
        raise ValueError("technical producer revisions do not match the reviewed catalog")
    if not isinstance(revisions, dict):
        raise ValueError("technical producer revisions are missing")
    for producer in ("capture", "slam", "perception"):
        _require_git_commit(repo_root, revisions.get(producer), producer)
    catalog_time = catalog_source.get("simulation_time", {})
    expected_time = {
        "basis": catalog_time.get("source"),
        "start_s": catalog_time.get("start_s"),
        "end_s": catalog_time.get("end_s"),
    }
    if source.get("simulation_time") != expected_time:
        raise ValueError("technical simulation-time range does not match the reviewed catalog")
    artifacts = source.get("artifacts")
    catalog_artifacts = catalog_source.get("artifacts", {})
    catalog_manifests = catalog_source.get("producer_manifests", {})
    if not isinstance(artifacts, dict):
        raise ValueError("technical source artifact bindings are missing")
    expected_hashes = {
        "map": catalog_artifacts.get("map", {}).get("sha256"),
        "trajectory": catalog_artifacts.get("trajectory", {}).get("sha256"),
        "inventory": catalog_artifacts.get("inventory", {}).get("sha256"),
        "capture_manifest": catalog_manifests.get("capture", {}).get("sha256"),
        "slam_manifest": catalog_manifests.get("slam", {}).get("sha256"),
        "perception_manifest": catalog_manifests.get("perception", {}).get("sha256"),
        "source_catalog": catalog_hash,
        **{name: catalog_artifacts.get(name, {}).get("sha256") for name in RAW_SOURCE_ARTIFACTS},
    }
    for name, expected in expected_hashes.items():
        if artifacts.get(name, {}).get("sha256") != expected:
            raise ValueError(f"technical {name} hash does not match the reviewed source catalog")
    expected_states = {
        "map_state": ("map", "slam"),
        "trajectory_state": ("trajectory", "slam"),
        "object_state": ("inventory", "perception"),
    }
    for state_name, (artifact_name, producer) in expected_states.items():
        state = source.get(state_name)
        catalog_artifact = catalog_artifacts.get(artifact_name, {})
        if not isinstance(state, dict):
            raise ValueError(f"technical {state_name} is missing")
        expected = {
            "version": catalog_artifact.get("version"),
            "sha256": catalog_artifact.get("sha256"),
            "producer_manifest_sha256": catalog_manifests.get(producer, {}).get("sha256"),
            "producer_revision": revisions.get(producer),
        }
        for key, value in expected.items():
            if state.get(key) != value:
                raise ValueError(f"technical {state_name}.{key} does not match the reviewed source catalog")
    object_state = source["object_state"]
    if object_state.get("ground_truth_consumed") is not False:
        raise ValueError("technical object state must be ground-truth-free")
    if object_state.get("depth_sources") != ["lidar_projected_with_slam_pose"]:
        raise ValueError("technical object state must use the reviewed estimated depth source")
    compatibility = catalog_source.get("presentation_compatibility")
    paired = source.get("paired_capture_state")
    if not isinstance(compatibility, dict) or not isinstance(paired, dict):
        raise ValueError("technical paired RGB/LiDAR compatibility binding is missing")
    expected_status = compatibility.get("status")
    expected_eligible = compatibility.get("delivery_eligible")
    if expected_status not in ("current_goal_paired_capture", "generated_test_fixture") or expected_eligible is not True:
        raise ValueError("technical delivery source is not a current-goal paired RGB/LiDAR capture")
    sensor_contract = catalog_source.get("sensor_contract")
    if not isinstance(sensor_contract, dict) or set(sensor_contract) != {
        "lidar_topic", "camera_info_topic", "lidar_frame_id", "camera_frame_id",
        "configured_lidar_hz", "maximum_current_age_periods", "maximum_rgb_skew_ns",
    }:
        raise ValueError("technical source catalog lacks its exact sensor contract")
    if (
        not all(isinstance(sensor_contract[name], str) and sensor_contract[name] for name in (
            "lidar_topic", "camera_info_topic", "lidar_frame_id", "camera_frame_id",
        ))
        or float(sensor_contract["configured_lidar_hz"]) <= 0.0
        or sensor_contract["maximum_current_age_periods"] != 2.0
        or sensor_contract["maximum_rgb_skew_ns"] != 17_000_001
    ):
        raise ValueError("technical source catalog sensor contract is invalid")
    expected_paired = {
        "compatibility_status": expected_status,
        "delivery_eligible": True,
        "raw_lidar_bag_sha256": catalog_artifacts.get("raw_lidar_bag", {}).get("sha256"),
        "rgb_video_sha256": catalog_artifacts.get("rgb_video", {}).get("sha256"),
        "rgb_frames_sha256": catalog_artifacts.get("rgb_frames", {}).get("sha256"),
        "camera_info_sha256": catalog_artifacts.get("camera_info", {}).get("sha256"),
        "sensor_transforms_sha256": catalog_artifacts.get("sensor_transforms", {}).get("sha256"),
        "scene_manifest_sha256": catalog_artifacts.get("scene_manifest", {}).get("sha256"),
        "effective_config_sha256": catalog_artifacts.get("effective_config", {}).get("sha256"),
        "trajectory_sha256": catalog_artifacts.get("trajectory", {}).get("sha256"),
        "capture_manifest_sha256": catalog_manifests.get("capture", {}).get("sha256"),
        **sensor_contract,
    }
    if paired != expected_paired:
        raise ValueError("technical paired capture state does not match the reviewed source catalog")
    if manifest.get("capture_id") != source.get("capture_id"):
        raise ValueError("technical delivery capture_id does not match its receipt source")
    if manifest.get("simulation_time") != {
        "start_s": expected_time["start_s"], "end_s": expected_time["end_s"]
    }:
        raise ValueError("technical delivery simulation time does not match its source")


def validate_technical_delivery(
    manifest_path: str | Path,
    repo_root: str | Path,
    ffprobe: str,
    source_catalog_path: str | Path | None = None,
) -> ValidatedTechnicalDelivery:
    """Validate seven distinct 1080p technical views and their derivation receipts."""

    root = Path(repo_root).resolve()
    path = Path(manifest_path).resolve()
    directory = path.parent
    manifest = _json(path)
    expected_order = [view_id for _, view_id, _, _ in TECHNICAL_VIEWS]
    expected_counts = {view_id: count for _, view_id, _, count in TECHNICAL_VIEWS}
    if manifest.get("schema_version") != 2 or manifest.get("status") != "complete":
        raise ValueError("technical delivery manifest is incomplete")
    if manifest.get("producer") != TECHNICAL_PRODUCER_ID or manifest.get("profile") != "delivery":
        raise ValueError("technical delivery producer/profile is invalid")
    if (manifest.get("width"), manifest.get("height"), manifest.get("fps")) != (1920, 1080, 30):
        raise ValueError("technical delivery must be native 1920x1080 at 30 fps")
    if manifest.get("ordered_views") != expected_order or manifest.get("view_frame_counts") != expected_counts:
        raise ValueError("technical delivery view order/frame counts are invalid")
    if manifest.get("total_frames") != sum(expected_counts.values()):
        raise ValueError("technical delivery total frame count is invalid")
    if manifest.get("storyboard_content_used") is not False:
        raise ValueError("technical delivery must exclude storyboard content")
    if manifest.get("selective_current_scan_goal") != SELECTIVE_SCAN_GOAL:
        raise ValueError("technical delivery must use the completed selective real-scan contract")
    renderer_hash = _sha256_lf_text(root / "simulator" / "technical_views.py")
    implementation_hashes = {name: _sha256_lf_text(root / name) for name in IMPLEMENTATION_SOURCES}
    plan_hash = _sha256_lf_text(root / "config" / "technical_views.json")
    if manifest.get("renderer", {}).get("sha256") != renderer_hash:
        raise ValueError("technical delivery renderer hash does not match repository code")
    if manifest.get("implementation_sha256") != implementation_hashes:
        raise ValueError("technical delivery LiDAR implementation hashes do not match repository code")
    if manifest.get("plan", {}).get("sha256") != plan_hash:
        raise ValueError("technical delivery plan hash does not match repository configuration")
    plan_payload = _json(root / "config" / "technical_views.json")
    plan_views = {str(item.get("id")): item for item in plan_payload.get("views", []) if isinstance(item, dict)}
    prior_guide_end: float | None = None
    for view_id in expected_order:
        guide_window = plan_views.get(view_id, {}).get("camera_guide_timestamp_window_s")
        sampling_window = plan_views.get(view_id, {}).get("camera_pose_sampling_window_s")
        if (
            not isinstance(guide_window, list)
            or len(guide_window) != 2
            or not isinstance(sampling_window, list)
            or len(sampling_window) != 2
            or not all(
                isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
                for value in (*guide_window, *sampling_window)
            )
            or float(guide_window[0]) >= float(guide_window[1])
            or float(sampling_window[0]) >= float(sampling_window[1])
            or float(sampling_window[0]) > float(guide_window[0])
            or float(guide_window[1]) > float(sampling_window[1])
            or (
                prior_guide_end is not None
                and not math.isclose(prior_guide_end, float(guide_window[0]), abs_tol=1e-9)
            )
        ):
            raise ValueError(f"technical plan camera guide/sampling window is invalid for {view_id}")
        prior_guide_end = float(guide_window[1])
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise ValueError("technical delivery source receipt is missing")
    capture_id = str(source.get("capture_id", ""))
    source_id = str(source.get("source_id", ""))
    catalog_path = Path(source_catalog_path).resolve() if source_catalog_path else root / "config" / "technical_source_catalog.json"
    catalog_hash = _sha256_lf_text(catalog_path)
    catalog_source = _catalog_source(catalog_path, capture_id, source_id)
    _validate_source_binding(source, manifest, catalog_source, catalog_hash, root)
    simulation = manifest.get("simulation_time")
    if not isinstance(simulation, dict):
        raise ValueError("technical delivery simulation window is missing")
    simulation_window = (float(simulation.get("start_s", math.nan)), float(simulation.get("end_s", math.nan)))
    if not all(math.isfinite(value) for value in simulation_window) or simulation_window[0] > simulation_window[1]:
        raise ValueError("technical delivery simulation window is invalid")
    from simulator.sensors.scan_projection import resolve_transform
    from simulator.technical_lidar import EstimatedTrajectory
    from simulator.technical_views import TechnicalCameraPath, load_inventory, load_plan, select_inventory

    trajectory_path = _verified_source_artifact(directory, catalog_source, "trajectory")
    inventory_path = _verified_source_artifact(directory, catalog_source, "inventory")
    transforms_path = _verified_source_artifact(directory, catalog_source, "sensor_transforms")
    transforms = _json(transforms_path)
    frames = transforms.get("frames")
    transform_rows = transforms.get("transforms")
    if not isinstance(frames, dict) or not isinstance(transform_rows, list):
        raise ValueError("technical camera motion source transform graph is invalid")
    rig_from_optical = resolve_transform(
        transform_rows,
        source_frame=str(frames.get("camera_optical", "")),
        target_frame=str(frames.get("sensor_rig", "")),
    )
    _profiles, view_specs = load_plan(root / "config" / "technical_views.json")
    view_specs_by_id = {spec.id: spec for spec in view_specs}
    focus_inventory = select_inventory(load_inventory(inventory_path))[0]
    expected_camera_path = TechnicalCameraPath(
        view_specs,
        EstimatedTrajectory.from_csv(trajectory_path),
        rig_from_optical,
        np.asarray(focus_inventory.position, dtype=np.float64),
    )
    expected_trace_rows = expected_camera_path.trace_rows(30)
    expected_dependency_windows_s = expected_camera_path.camera_pose_dependency_windows_receipt()
    expected_anchor = {
        "track_id": focus_inventory.track_id,
        "position_m": list(focus_inventory.position),
        "source": "first deterministic selected row in hash-bound estimated inventory",
    }
    if manifest.get("camera_motion_anchor") != expected_anchor:
        raise ValueError("technical camera motion anchor is not deterministic from the bound inventory")
    trace_by_view = _validate_camera_motion_trace(
        manifest.get("camera_motion_trace"),
        directory=directory,
        plan_views=plan_views,
        expected_order=expected_order,
        expected_counts=expected_counts,
        simulation_window_s=simulation_window,
        fps=30,
        expected_rows=expected_trace_rows,
        expected_dependency_windows_s=expected_dependency_windows_s,
    )
    manifest_derivations = manifest.get("view_derivations")
    if not isinstance(manifest_derivations, dict) or set(manifest_derivations) != set(expected_order):
        raise ValueError("technical delivery manifest view derivations are missing or incomplete")
    rgb_index_path = _verified_source_artifact(directory, catalog_source, "rgb_frames")
    rgb_frame_timestamps_ns = _verified_rgb_frame_index(
        rgb_index_path,
        camera_frame_id=str(source["paired_capture_state"]["camera_frame_id"]),
    )
    if manifest.get("input_snapshot_verification") != "post_render_sha256_match":
        raise ValueError("technical delivery lacks post-render input snapshot verification")
    presentation_classification = validate_presentation_classification(
        catalog_source.get("presentation_classification"),
        allow_legacy_production=True,
    )
    if source.get("presentation_classification", {"kind": "reviewed_production"}) != presentation_classification:
        raise ValueError("technical source presentation classification does not match the reviewed catalog")
    outputs = manifest.get("outputs")
    if not isinstance(outputs, list) or len(outputs) != len(TECHNICAL_VIEWS):
        raise ValueError("technical delivery must contain seven output entries")
    known_storyboards, _ = storyboard_hashes(root)
    seen_video_hashes: set[str] = set()
    shots: list[TechnicalShotSource] = []
    for output, (shot_number, view_id, role, frame_count) in zip(outputs, TECHNICAL_VIEWS):
        if not isinstance(output, dict) or output.get("view_ids") != [view_id]:
            raise ValueError(f"technical output does not match view {view_id}")
        if output.get("presentation_role") != role or output.get("frames") != frame_count:
            raise ValueError(f"technical output role/frame count is invalid for {view_id}")
        video_path = _safe_child(directory, output.get("path"), f"outputs[{view_id}].path")
        receipt_info = output.get("receipt")
        if not isinstance(receipt_info, dict):
            raise ValueError(f"technical output receipt is missing for {view_id}")
        receipt_path = _safe_child(directory, receipt_info.get("path"), f"outputs[{view_id}].receipt.path")
        if not video_path.is_file() or not receipt_path.is_file():
            raise ValueError(f"technical output files are missing for {view_id}")
        video_hash = sha256_path(video_path)
        receipt_hash = sha256_path(receipt_path)
        if video_hash != output.get("sha256") or receipt_hash != receipt_info.get("sha256"):
            raise ValueError(f"technical output hash mismatch for {view_id}")
        if video_hash in known_storyboards or video_hash in seen_video_hashes:
            raise ValueError("technical delivery reuses a storyboard or another shot video")
        seen_video_hashes.add(video_hash)
        probe = _probe_video(video_path, ffprobe)
        if probe != {"width": 1920, "height": 1080, "fps": 30.0, "frame_count": frame_count}:
            raise ValueError(f"technical output probe mismatch for {view_id}: {probe}")
        if output.get("probe") != probe:
            raise ValueError(f"technical manifest probe is forged for {view_id}")
        receipt = _json(receipt_path)
        if (
            receipt.get("schema_version") != 2
            or receipt.get("artifact_type") != "technical_source_view_receipt"
            or receipt.get("status") != "complete"
            or receipt.get("view_id") != view_id
            or receipt.get("presentation_role") != role
        ):
            raise ValueError(f"technical receipt identity is invalid for {view_id}")
        producer = receipt.get("producer", {})
        if producer != {
            "id": TECHNICAL_PRODUCER_ID,
            "renderer_sha256": renderer_hash,
            "implementation_sha256": implementation_hashes,
            "plan_sha256": plan_hash,
        }:
            raise ValueError(f"technical receipt producer binding is invalid for {view_id}")
        if receipt.get("source") != source:
            raise ValueError(f"technical receipt source binding differs for {view_id}")
        video_receipt = receipt.get("video", {})
        if video_receipt != {"path": video_path.name, "sha256": video_hash, **probe}:
            raise ValueError(f"technical receipt video binding is invalid for {view_id}")
        derivation = receipt.get("derivation", {})
        if manifest_derivations.get(view_id) != derivation:
            raise ValueError(f"technical manifest/receipt derivations differ for {view_id}")
        if derivation.get("storyboard_pixels_consumed") is not False:
            raise ValueError(f"technical receipt lacks storyboard exclusion for {view_id}")
        expected_plan = plan_views.get(view_id, {})
        for field in ("selection_mode", "temporal_mode"):
            if derivation.get(field) != expected_plan.get(field):
                raise ValueError(f"technical receipt {field} is invalid for {view_id}")
        expected_window = expected_plan.get("source_window_s")
        if derivation.get("source_window_s") != expected_window:
            raise ValueError(f"technical receipt source window is invalid for {view_id}")
        expected_display_window = expected_plan.get("display_window_s")
        if derivation.get("display_window_s") != expected_display_window:
            raise ValueError(f"technical receipt display window is invalid for {view_id}")
        expected_camera_motion = _expected_camera_motion(
            expected_plan,
            trace_by_view[view_id],
            focus_inventory,
            expected_camera_path.camera_pose_dependency_window_s(view_specs_by_id[view_id]),
        )
        if derivation.get("camera_motion") != expected_camera_motion:
            raise ValueError(f"technical receipt camera motion binding is invalid for {view_id}")
        if derivation.get("rendered_context_point_budget") != expected_plan.get("maximum_points"):
            raise ValueError(f"technical receipt context point budget is invalid for {view_id}")
        is_roi_view = expected_plan.get("selection_mode") == "estimated_roi_front_surfaces"
        expected_context_treatment = (
            "local sparse structural silhouette around the estimated ROI"
            if is_roi_view else "bounded structural or current-return subset"
        )
        if derivation.get("context_treatment") != expected_context_treatment:
            raise ValueError(f"technical receipt context treatment is invalid for {view_id}")
        if (
            derivation.get("selective_current_scan_status") != "complete"
            or derivation.get("future_returns_consumed") is not False
            or derivation.get("simulator_truth_consumed") is not False
            or derivation.get("scene_or_asset_metadata_consumed") is not False
        ):
            raise ValueError(f"technical receipt violates selective real-scan isolation for {view_id}")
        if derivation.get("scan_time_model") != SCAN_TIME_MODEL:
            raise ValueError(f"technical receipt lacks exact rigid scan-time disclosure for {view_id}")
        if derivation.get("causal_display_policy") != (
            "latest and previous scans only; maximum current age is 2 configured scan periods"
        ):
            raise ValueError(f"technical receipt lacks bounded causal scan policy for {view_id}")
        calibration = derivation.get("camera_calibration")
        expected_camera_topic = source["paired_capture_state"]["camera_info_topic"]
        if not isinstance(calibration, dict) or calibration.get("topic") != expected_camera_topic:
            raise ValueError(f"technical camera calibration receipt is invalid for {view_id}")
        if calibration.get("representation") == "configured_intrinsics":
            if calibration != {
                "representation": "configured_intrinsics",
                "observed_ros_message": False,
                "distortion_handling": "ideal_pinhole_declared_by_capture",
                "topic": expected_camera_topic,
            }:
                raise ValueError(f"technical configured calibration disclosure is invalid for {view_id}")
        elif calibration.get("representation") == "observed_ros_camera_info":
            if (
                calibration.get("observed_ros_message") is not True
                or calibration.get("distortion_handling") != "zero_coefficients_no_rectification_required"
                or not isinstance(calibration.get("stamp_s"), (int, float))
            ):
                raise ValueError(f"technical observed calibration disclosure is invalid for {view_id}")
        else:
            raise ValueError(f"technical camera calibration provenance is unsupported for {view_id}")
        source_window = tuple(float(value) for value in expected_window)
        display_window = tuple(float(value) for value in expected_display_window)
        primary_window = display_window if (
            expected_plan.get("temporal_mode") == "current_window" or is_roi_view
        ) else source_window
        scan_rows = _validate_scan_selection(
            derivation.get("scan_selection"),
            view_id=view_id,
            label="primary scan selection",
            expected_mode=str(expected_plan.get("selection_mode")),
            source_window_s=primary_window,
            lidar_frame_id=str(source["paired_capture_state"]["lidar_frame_id"]),
        )
        scan_by_timestamp = {int(row["timestamp_ns"]): row for row in scan_rows}
        scan_by_identity = {(int(row["message_id"]), int(row["timestamp_ns"])): row for row in scan_rows}
        if is_roi_view:
            _validate_scan_selection(
                derivation.get("context_scan_selection"),
                view_id=view_id,
                label="ROI context scan selection",
                expected_mode="past_structural_history",
                source_window_s=source_window,
                lidar_frame_id=str(source["paired_capture_state"]["lidar_frame_id"]),
            )
        elif "context_scan_selection" in derivation:
            raise ValueError(f"technical non-ROI receipt has unexpected context scan selection for {view_id}")
        if bool(expected_plan.get("rgb_context")):
            pairs = derivation.get("cotimed_rgb_pairs")
            pair_rows = pairs.get("pairs") if isinstance(pairs, dict) else None
            if not isinstance(pairs, dict) or set(pairs) != {
                "pair_count", "maximum_absolute_skew_ns", "pairs",
            } or not isinstance(pair_rows, list) or not pair_rows:
                raise ValueError(f"technical current scan lacks co-timed RGB binding for {view_id}")
            seen_pairs: set[tuple[int, int]] = set()
            skews: list[int] = []
            paired_scan_timestamps: set[int] = set()
            for pair in pair_rows:
                if not isinstance(pair, dict) or set(pair) != {
                    "scan_timestamp_ns", "rgb_frame_index", "absolute_skew_ns",
                }:
                    raise ValueError(f"technical RGB pair schema is invalid for {view_id}")
                scan_timestamp = pair.get("scan_timestamp_ns")
                frame_index = pair.get("rgb_frame_index")
                skew_ns = pair.get("absolute_skew_ns")
                identity = (scan_timestamp, frame_index)
                bound_rgb_timestamp_ns = (
                    rgb_frame_timestamps_ns.get(frame_index) if _plain_int(frame_index) else None
                )
                expected_skew_ns = (
                    abs(bound_rgb_timestamp_ns - scan_timestamp)
                    if bound_rgb_timestamp_ns is not None and _plain_int(scan_timestamp)
                    else None
                )
                if (
                    not all(_plain_int(value) for value in (scan_timestamp, frame_index, skew_ns))
                    or scan_timestamp not in scan_by_timestamp
                    or frame_index < 0
                    or skew_ns < 0
                    or skew_ns > source["paired_capture_state"]["maximum_rgb_skew_ns"]
                    or expected_skew_ns != skew_ns
                    or identity in seen_pairs
                ):
                    raise ValueError(
                        f"technical RGB pair does not match the bound RGB frame index or selected current scan for {view_id}"
                    )
                seen_pairs.add(identity)
                skews.append(skew_ns)
                paired_scan_timestamps.add(scan_timestamp)
            if (
                pairs.get("pair_count") != len(pair_rows)
                or pairs.get("maximum_absolute_skew_ns") != max(skews)
                or paired_scan_timestamps != set(scan_by_timestamp)
            ):
                raise ValueError(f"technical RGB pair aggregates are invalid for {view_id}")
        elif "cotimed_rgb_pairs" in derivation:
            raise ValueError(f"technical non-RGB receipt has unexpected co-timed RGB pairs for {view_id}")
        if is_roi_view:
            rois = derivation.get("estimated_roi_selection")
            roi_rows = rois.get("rois") if isinstance(rois, dict) else None
            if (
                not isinstance(rois, dict)
                or not _plain_int(rois.get("roi_count"))
                or rois["roi_count"] <= 0
                or not isinstance(roi_rows, list)
                or len(roi_rows) != rois["roi_count"]
            ):
                raise ValueError(f"technical estimated ROI support is missing for {view_id}")
            seen_rois: set[tuple[object, ...]] = set()
            referenced_scans: set[tuple[int, int]] = set()
            roi_indices_by_scan: dict[tuple[int, int], set[int]] = {}
            for roi in roi_rows:
                if not isinstance(roi, dict) or set(roi) != {
                    "scan_message_id", "scan_timestamp_ns", "rgb_frame_index", "rgb_timestamp_s",
                    "absolute_rgb_skew_ns", "track_id", "raw_track_id", "bbox_xyxy",
                    "selected_return_count", "selected_raw_indices", "selected_raw_indices_sha256", "selection",
                }:
                    raise ValueError(f"technical ROI method/index receipt is invalid for {view_id}")
                message_id = roi.get("scan_message_id")
                timestamp_ns = roi.get("scan_timestamp_ns")
                frame_index = roi.get("rgb_frame_index")
                rgb_timestamp_s = roi.get("rgb_timestamp_s")
                skew_ns = roi.get("absolute_rgb_skew_ns")
                track_id = roi.get("track_id")
                raw_track_id = roi.get("raw_track_id")
                count = roi.get("selected_return_count")
                bbox = roi.get("bbox_xyxy")
                scan_identity = (message_id, timestamp_ns)
                raw_indices_value = roi.get("selected_raw_indices")
                raw_indices = (
                    tuple(raw_indices_value)
                    if isinstance(raw_indices_value, list)
                    and all(_plain_int(value) for value in raw_indices_value)
                    else ()
                )
                bound_rgb_timestamp_ns = (
                    rgb_frame_timestamps_ns.get(frame_index) if _plain_int(frame_index) else None
                )
                if (
                    not all(_plain_int(value) for value in (message_id, timestamp_ns, frame_index, skew_ns, track_id, count))
                    or (raw_track_id is not None and not _plain_int(raw_track_id))
                    or (view_id == "object_detail" and track_id != focus_inventory.track_id)
                    or scan_identity not in scan_by_identity
                    or frame_index < 0
                    or count <= 0
                    or count > int(scan_by_identity[scan_identity]["selected_return_count"])
                    or len(raw_indices) != count
                    or any(
                        value < 0 or value >= int(scan_by_identity[scan_identity]["raw_point_count"])
                        for value in raw_indices
                    )
                    or any(second <= first for first, second in zip(raw_indices, raw_indices[1:]))
                    or roi.get("selection") != "bbox_nearest_front_surface"
                    or roi.get("selected_raw_indices_sha256") != _raw_indices_sha256(raw_indices)
                    or not isinstance(rgb_timestamp_s, (int, float))
                    or isinstance(rgb_timestamp_s, bool)
                    or not math.isfinite(float(rgb_timestamp_s))
                    or bound_rgb_timestamp_ns is None
                    or int(round(float(rgb_timestamp_s) * 1_000_000_000)) != bound_rgb_timestamp_ns
                    or not isinstance(bbox, list)
                    or len(bbox) != 4
                    or not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)) for value in bbox)
                    or float(bbox[2]) < float(bbox[0])
                    or float(bbox[3]) < float(bbox[1])
                    or skew_ns < 0
                    or skew_ns > source["paired_capture_state"]["maximum_rgb_skew_ns"]
                    or abs(bound_rgb_timestamp_ns - timestamp_ns) != skew_ns
                    or not (
                        display_window[0] - skew_ns / 1_000_000_000.0 - 1e-9
                        <= float(rgb_timestamp_s)
                        <= display_window[1] + skew_ns / 1_000_000_000.0 + 1e-9
                    )
                ):
                    raise ValueError(
                        f"technical ROI row is not bound to a valid in-window selected scan or co-timed RGB frame for {view_id}"
                    )
                identity = (message_id, timestamp_ns, track_id, tuple(float(value) for value in bbox), roi["selected_raw_indices_sha256"])
                if identity in seen_rois:
                    raise ValueError(f"technical ROI rows are not unique for {view_id}")
                seen_rois.add(identity)
                referenced_scans.add(scan_identity)
                roi_indices_by_scan.setdefault(scan_identity, set()).update(raw_indices)
            exact_roi_union = referenced_scans == set(scan_by_identity)
            for scan_identity, scan in scan_by_identity.items():
                indices = tuple(sorted(roi_indices_by_scan.get(scan_identity, set())))
                if (
                    len(indices) != int(scan["selected_return_count"])
                    or _raw_indices_sha256(indices) != scan["selected_raw_indices_sha256"]
                ):
                    exact_roi_union = False
            if not exact_roi_union:
                raise ValueError(f"technical ROI aggregates do not cover the selected ROI scans for {view_id}")
        elif "estimated_roi_selection" in derivation:
            raise ValueError(f"technical non-ROI receipt has unexpected ROI selection for {view_id}")
        shots.append(
            TechnicalShotSource(
                shot_number,
                view_id,
                role,
                video_path,
                video_hash,
                frame_count,
                source_window,
                receipt_path,
                receipt_hash,
            )
        )
    return ValidatedTechnicalDelivery(
        path,
        sha256_path(path),
        capture_id,
        str(catalog_source.get("capture_sha256", "")),
        source_id,
        source,
        presentation_classification,
        tuple(shots),
    )
