"""Validate and adapt technical-view delivery receipts for presentation shots 6-12."""

from __future__ import annotations

import json
import hashlib
import subprocess
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path, PurePosixPath
from typing import Any

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
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise ValueError("technical delivery source receipt is missing")
    capture_id = str(source.get("capture_id", ""))
    source_id = str(source.get("source_id", ""))
    catalog_path = Path(source_catalog_path).resolve() if source_catalog_path else root / "config" / "technical_source_catalog.json"
    catalog_hash = _sha256_lf_text(catalog_path)
    catalog_source = _catalog_source(catalog_path, capture_id, source_id)
    _validate_source_binding(source, manifest, catalog_source, catalog_hash, root)
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
        if derivation.get("storyboard_pixels_consumed") is not False:
            raise ValueError(f"technical receipt lacks storyboard exclusion for {view_id}")
        expected_plan = plan_views.get(view_id, {})
        for field in ("selection_mode", "temporal_mode"):
            if derivation.get(field) != expected_plan.get(field):
                raise ValueError(f"technical receipt {field} is invalid for {view_id}")
        expected_window = expected_plan.get("source_window_s")
        if derivation.get("source_window_s") != expected_window:
            raise ValueError(f"technical receipt source window is invalid for {view_id}")
        if derivation.get("rendered_context_point_budget") != expected_plan.get("maximum_points"):
            raise ValueError(f"technical receipt context point budget is invalid for {view_id}")
        if derivation.get("context_treatment") not in {
            "local sparse structural silhouette around the estimated ROI",
            "bounded structural or current-return subset",
        }:
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
        scan_selection = derivation.get("scan_selection")
        if not isinstance(scan_selection, dict) or int(scan_selection.get("scan_count", 0)) <= 0:
            raise ValueError(f"technical receipt has no selected raw scans for {view_id}")
        scan_rows = scan_selection.get("scans")
        if not isinstance(scan_rows, list) or len(scan_rows) != scan_selection.get("scan_count"):
            raise ValueError(f"technical scan receipt list is invalid for {view_id}")
        for scan in scan_rows:
            fields = scan.get("point_fields") if isinstance(scan, dict) else None
            if (
                not isinstance(scan, dict)
                or int(scan.get("raw_point_count", 0)) <= 0
                or int(scan.get("selected_return_count", 0)) <= 0
                or not SHA256_PATTERN.fullmatch(str(scan.get("selected_raw_indices_sha256", "")))
                or scan.get("source_frame_id") != source["paired_capture_state"]["lidar_frame_id"]
                or not isinstance(fields, list)
                or not {"x", "y", "z"}.issubset(fields)
                or any(name in {"time", "t", "timestamp", "offset_time", "time_offset", "timestamp_ns"} for name in fields)
                or scan.get("per_return_timing") != PER_RETURN_TIMING
            ):
                raise ValueError(f"technical selected-return receipt is invalid for {view_id}")
        if bool(expected_plan.get("rgb_context")):
            pairs = derivation.get("cotimed_rgb_pairs")
            if (
                not isinstance(pairs, dict)
                or int(pairs.get("pair_count", 0)) <= 0
                or int(pairs.get("maximum_absolute_skew_ns", 99_999_999)) > 17_000_001
            ):
                raise ValueError(f"technical current scan lacks co-timed RGB binding for {view_id}")
        if expected_plan.get("selection_mode") == "estimated_roi_front_surfaces":
            rois = derivation.get("estimated_roi_selection")
            roi_rows = rois.get("rois") if isinstance(rois, dict) else None
            if (
                not isinstance(rois, dict)
                or int(rois.get("roi_count", 0)) <= 0
                or not isinstance(roi_rows, list)
                or len(roi_rows) != int(rois.get("roi_count", 0))
            ):
                raise ValueError(f"technical estimated ROI support is missing for {view_id}")
            if any(
                not isinstance(roi, dict) or int(roi.get("absolute_rgb_skew_ns", 99_999_999)) > 17_000_001
                for roi in roi_rows
            ):
                raise ValueError(f"technical estimated ROI lacks co-timed RGB binding for {view_id}")
        source_window = tuple(float(value) for value in expected_window)
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
