"""Fail-closed provenance validation for presentation input bundles.

The input manifest is evidence routing, not authority.  Genuine roles are
accepted only when every artifact has a verified hash, the files match the
machine-readable formats emitted by the named repository stage, capture and
map associations agree, and a derived-view receipt binds the view video to the
exact source hashes. Technical delivery views are validated through their
repository renderer receipts and reviewed source catalog by technical_bundle.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from simulator.perception.provenance import (
    MINIMUM_PRESENTATION_SOURCE_FRAMES,
    validate_perception_frame_coverage,
    validate_perception_manifest_bindings,
)


SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
REQUIRED_CAPTURE_TOPICS = (
    "/clock",
    "/sim/camera/rgb/image_raw",
    "/sim/camera/rgb/camera_info",
    "/sim/lidar/points",
    "/tf",
    "/tf_static",
)
RGB_PRESENTATION_FRAME_COUNT = 540
RGB_TIMESTAMP_PERIOD_S = 1.0 / 30.0
RGB_TIMESTAMP_TOLERANCE_S = 0.002
DEFAULT_RGB_CAPTURE_CATALOG = "config/presentation/accepted_rgb_captures.json"
GENERATED_TEST_FIXTURE_MARKER_ID = "simulator.presentation.generated-test-fixture.v1"
GENERATED_TEST_FIXTURE_MARKER_SHA256 = hashlib.sha256(
    (GENERATED_TEST_FIXTURE_MARKER_ID + "\n").encode("utf-8")
).hexdigest()
REVIEWED_PRODUCTION_CLASSIFICATION = {"kind": "reviewed_production"}
GENERATED_TEST_FIXTURE_CLASSIFICATION = {
    "kind": "generated_test_fixture",
    "marker_id": GENERATED_TEST_FIXTURE_MARKER_ID,
    "marker_sha256": GENERATED_TEST_FIXTURE_MARKER_SHA256,
}
PRESENTATION_TRANSFORM_NONE = {
    "operation": "none",
    "reason": "independent review confirmed the raw FRESH MARKET sign reads left-to-right; no global flip is permitted",
    "raw_capture_bytes_modified": False,
}
PRESENTATION_TRANSFORM_HFLIP = {
    "operation": "hflip",
    "reason": "native simulator RGB is mirrored horizontally; presentation text must read normally",
    "raw_capture_bytes_modified": False,
}
PRESENTATION_TRANSFORMS = {
    "none": PRESENTATION_TRANSFORM_NONE,
    "hflip": PRESENTATION_TRANSFORM_HFLIP,
}

DIAGNOSTIC_BASELINE_IDENTITY = {
    "plan_path": "config/presentation/diagnostic_storyboard_legacy.yaml",
    "plan_sha256_lf": "5a7e98db32134ba8f6507e60534a056bf9e5e35f3290d61eb9fa6b684479bc5f",
    "manifest_path": "config/presentation/diagnostic_baseline_inputs.json",
    "manifest_sha256_lf": "c18e9139feb34a59e9582aea331a83c2f1e0a8e1d77b2490dd3892eb21e8a9cf",
    "video_path": "demo/walking_aisle_final_hifi.mp4",
    "video_sha256": "c8a0032862b1397889d187295cc4132ad1d45caad4697fad3fd6f320384e4096",
    "video_git_blob": "57628e29ab73fc83a9e7ad97167a01df9b66e472",
    "introduced_commit": "d5e825c8f6dab77aa6a1007c9731c226b588dfcf",
    "storyboard_manifest_commit": "d315aa9c21fa1bfb3aec284687c6ed4a49151941",
}


@dataclass(frozen=True)
class RoleSpec:
    schema_id: str
    producer_id: str
    producer_source_path: str
    required_artifacts: tuple[tuple[str, str], ...]
    view_producer_id: str | None
    view_producer_source_path: str | None
    view_producer_available: bool


ROLE_SPECS = {
    "rgb": RoleSpec(
        "simulation.presentation.rgb_bundle.v1",
        "scripts.capture_simulation.v1",
        "scripts/capture_simulation.ps1",
        (
            ("view_video", "video"),
            ("frame_index", "rgb_frame_index"),
            ("camera_info", "camera_info"),
            ("rgb_metadata", "rgb_metadata"),
            ("capture_manifest", "capture_manifest"),
            ("view_manifest", "view_manifest"),
        ),
        "simulator.presentation.rgb_bundle_emitter.v1",
        "simulator/presentation/rgb_bundle.py",
        True,
    ),
    "lidar": RoleSpec(
        "simulation.presentation.lidar_bundle.v1",
        "simulator.capture.rosbag_capture.v1",
        "simulator/capture/rosbag_capture.py",
        (
            ("view_video", "video"),
            ("scan_index", "bag_metadata"),
            ("returns", "rosbag2"),
            ("calibration", "sensor_transforms"),
            ("capture_manifest", "capture_manifest"),
            ("view_manifest", "view_manifest"),
        ),
        "simulator.presentation.lidar_view.v1",
        None,
        False,
    ),
    "pose": RoleSpec(
        "simulation.presentation.pose_bundle.v1",
        "scripts.run_slam_offline.v1",
        "scripts/run_slam_offline.ps1",
        (
            ("trajectory", "slam_poses"),
            ("frame_contract", "frame_contract"),
            ("capture_manifest", "capture_manifest"),
            ("slam_manifest", "slam_manifest"),
        ),
        None,
        None,
        True,
    ),
    "map": RoleSpec(
        "simulation.presentation.map_bundle.v1",
        "scripts.run_slam_offline.v1",
        "scripts/run_slam_offline.ps1",
        (
            ("view_video", "video"),
            ("snapshot_index", "map_snapshot_index"),
            ("map_states", "pcd"),
            ("pose_association", "slam_poses"),
            ("capture_manifest", "capture_manifest"),
            ("slam_manifest", "slam_manifest"),
            ("view_manifest", "view_manifest"),
        ),
        "simulator.presentation.map_view.v1",
        None,
        False,
    ),
    "reconstruction": RoleSpec(
        "simulation.presentation.reconstruction_bundle.v1",
        "simulator.perception.rgb_tracking.v1",
        "simulator/perception/rgb_tracking.py",
        (
            ("view_video", "video"),
            ("object_records", "object_records"),
            ("observation_links", "observation_links"),
            ("capture_manifest", "capture_manifest"),
            ("slam_manifest", "slam_manifest"),
            ("perception_manifest", "perception_manifest"),
            ("view_manifest", "view_manifest"),
        ),
        "simulator.presentation.reconstruction_view.v1",
        None,
        False,
    ),
}


def schema_catalog() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "input_manifest_schema_versions": {
            "diagnostic_or_single_role": 2,
            "complete_shot_bundle": 3,
        },
        "rgb_capture_acceptance": {
            "mechanism": "reviewed_exact_capture_allowlist",
            "catalog_path": DEFAULT_RGB_CAPTURE_CATALOG,
            "cryptographic_execution_attestation": False,
            "presentation_classifications": [
                REVIEWED_PRODUCTION_CLASSIFICATION,
                GENERATED_TEST_FIXTURE_CLASSIFICATION,
            ],
            "presentation_transforms": list(PRESENTATION_TRANSFORMS.values()),
        },
        "repaired_source_contract": {
            "capture_manifest_version": 1,
            "capture_topics": list(REQUIRED_CAPTURE_TOPICS),
            "camera_info_provenance": "observed_ros_message",
            "slam_trajectory": "slam/slam_map_poses.csv",
            "slam_trajectory_role": "dense_corrected_trajectory",
            "slam_frame_id": "map",
            "slam_optimized": True,
            "perception_slam_artifact": "slam/slam_map.pcd",
            "perception_ground_truth_consumed": False,
        },
        "diagnostic_baseline": {
            "schema_id": "simulation.presentation.diagnostic_baseline.v1",
            "producer_id": "repository.demo.baseline.v1",
            "artifacts": {"view_video": "video"},
            "immutable_identity": DIAGNOSTIC_BASELINE_IDENTITY,
        },
        "roles": {
            name: {
                "schema_id": spec.schema_id,
                "producer_id": spec.producer_id,
                "producer_source_path": spec.producer_source_path,
                "artifacts": dict(spec.required_artifacts),
                "view_producer_id": spec.view_producer_id,
                "view_producer_source_path": spec.view_producer_source_path,
                "view_producer_available": spec.view_producer_available,
            }
            for name, spec in ROLE_SPECS.items()
        },
        "view_derivation": {
            "schema_id": "simulation.presentation.view_derivation.v1",
            "required_fields": [
                "producer_id",
                "producer_source_sha256",
                "role",
                "capture_id",
                "output_video_sha256",
                "source_artifact_sha256",
                "source_time_range_s",
                "presentation_transform",
                "map_version",
                "object_state_version",
            ],
        },
        "complete_bundle": {
            "schema_id": "simulation.presentation.complete_bundle.v3",
            "producer_id": "simulator.presentation.complete_bundle_emitter.v1",
            "technical_producer_id": "grocery_sim.technical_views.cpu.v1",
            "technical_views": [
                "sensor_activation", "lidar_environment", "persistent_map", "object_association",
                "object_detail", "observed_aisle_overview", "final_technical_view",
            ],
        },
    }


def validate_presentation_transform(value: object) -> dict[str, object]:
    """Accept only an exact reviewed RGB orientation operation and reason."""

    if not isinstance(value, dict):
        raise ValueError("RGB presentation_transform must be a mapping")
    operation = value.get("operation")
    expected = PRESENTATION_TRANSFORMS.get(str(operation))
    if expected is None or value != expected:
        raise ValueError("RGB presentation_transform is not an exact supported reviewed transform")
    return dict(expected)


@dataclass(frozen=True)
class Artifact:
    name: str
    path: Path
    sha256: str
    kind: str


@dataclass(frozen=True)
class ValidatedRole:
    name: str
    contract: str
    status: str
    provenance: str
    schema_id: str
    producer_id: str
    capture_id: str
    map_version: str | None
    object_state_version: str | None
    artifacts: dict[str, Artifact]
    errors: tuple[str, ...]


def sha256_path(path: Path) -> str:
    if path.is_file():
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()
    if path.is_dir():
        digest = hashlib.sha256()
        files = sorted((item for item in path.rglob("*") if item.is_file()), key=lambda item: item.relative_to(path).as_posix())
        for item in files:
            relative = item.relative_to(path).as_posix().encode("utf-8")
            digest.update(len(relative).to_bytes(4, "big"))
            digest.update(relative)
            digest.update(bytes.fromhex(sha256_path(item)))
        return digest.hexdigest()
    raise FileNotFoundError(path)


def source_text_sha256(path: Path) -> str:
    normalized = path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def validate_presentation_classification(
    value: object,
    *,
    allow_legacy_production: bool = False,
) -> dict[str, str]:
    """Return the exact classification carried by a reviewed source catalog."""

    if value is None and allow_legacy_production:
        return dict(REVIEWED_PRODUCTION_CLASSIFICATION)
    if value == REVIEWED_PRODUCTION_CLASSIFICATION:
        return dict(REVIEWED_PRODUCTION_CLASSIFICATION)
    if value == GENERATED_TEST_FIXTURE_CLASSIFICATION:
        return dict(GENERATED_TEST_FIXTURE_CLASSIFICATION)
    raise ValueError("reviewed source presentation_classification is invalid")


def _git_blob_oid(path: Path) -> str:
    content = path.read_bytes()
    header = f"blob {len(content)}\0".encode("ascii")
    return hashlib.sha1(header + content).hexdigest()


def validate_diagnostic_identity(plan_path: Path, manifest_path: Path, repo_root: Path) -> None:
    expected_plan = (repo_root / DIAGNOSTIC_BASELINE_IDENTITY["plan_path"]).resolve()
    expected_manifest = (repo_root / DIAGNOSTIC_BASELINE_IDENTITY["manifest_path"]).resolve()
    if plan_path.resolve() != expected_plan or source_text_sha256(plan_path) != DIAGNOSTIC_BASELINE_IDENTITY["plan_sha256_lf"]:
        raise ValueError("diagnostic rendering requires the canonical immutable presentation plan")
    if manifest_path.resolve() != expected_manifest or source_text_sha256(manifest_path) != DIAGNOSTIC_BASELINE_IDENTITY["manifest_sha256_lf"]:
        raise ValueError("diagnostic rendering requires the canonical immutable input manifest")


def storyboard_hashes(repo_root: Path) -> tuple[frozenset[str], str]:
    manifest_path = repo_root / "references" / "manifest.json"
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    hashes = frozenset(str(item["sha256"]).lower() for item in data.get("references", []))
    if len(hashes) != 12 or not all(SHA256_PATTERN.fullmatch(value) for value in hashes):
        raise ValueError("references/manifest.json must declare 12 valid storyboard hashes")
    return hashes, sha256_path(manifest_path)


def _json(path: Path, expected: type = dict) -> Any:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, expected):
        raise ValueError(f"{path.name} must contain {expected.__name__}")
    return value


def _jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"{path.name}:{line_number} must contain a JSON object")
            rows.append(value)
    if not rows:
        raise ValueError(f"{path.name} must not be empty")
    return rows


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _validate_video(path: Path) -> None:
    if path.stat().st_size < 32:
        raise ValueError("video is too small")
    with path.open("rb") as handle:
        header = handle.read(32)
    if b"ftyp" not in header:
        raise ValueError("video does not have an ISO BMFF/MP4 header")


def _validate_rgb_frame_index(path: Path) -> None:
    rows = _jsonl(path)
    if len(rows) < RGB_PRESENTATION_FRAME_COUNT:
        raise ValueError("RGB frame index must contain at least 540 frames for presentation shots 1-5")
    last_stamp = -math.inf
    for expected, row in enumerate(rows):
        if int(row.get("frame_index", -1)) != expected:
            raise ValueError("RGB frame indices must be contiguous from zero")
        stamp = _finite(row.get("stamp_s"), "RGB stamp_s")
        if expected and abs((stamp - last_stamp) - RGB_TIMESTAMP_PERIOD_S) > RGB_TIMESTAMP_TOLERANCE_S:
            raise ValueError("RGB timestamps must form a contiguous 30 fps source-frame mapping")
        last_stamp = stamp
        if int(row.get("width", 0)) <= 0 or int(row.get("height", 0)) <= 0 or not str(row.get("frame_id", "")):
            raise ValueError("RGB frame record is incomplete")


def _validate_camera_info(path: Path) -> None:
    data = _json(path)
    if data.get("topic") != "/sim/camera/rgb/camera_info" or not str(data.get("frame_id", "")):
        raise ValueError("camera_info topic/frame_id is invalid")
    if data.get("provenance") == "configured_intrinsics":
        expected_keys = {
            "schema_version", "provenance", "observed_ros_message", "source", "topic",
            "frame_id", "model", "width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px",
        }
        if set(data) != expected_keys:
            raise ValueError("configured camera_info fields do not match the canonical export schema")
        if data.get("schema_version") != 1 or data.get("observed_ros_message") is not False:
            raise ValueError("configured camera_info provenance flags are invalid")
        if data.get("source") != "sensor_transforms.json" or data.get("model") != "ideal_pinhole":
            raise ValueError("configured camera_info source/model is invalid")
        width, height = int(data.get("width_px", 0)), int(data.get("height_px", 0))
        fx = _finite(data.get("fx_px"), "camera fx_px")
        fy = _finite(data.get("fy_px"), "camera fy_px")
        cx = _finite(data.get("cx_px"), "camera cx_px")
        cy = _finite(data.get("cy_px"), "camera cy_px")
        if width <= 0 or height <= 0 or fx <= 0 or fy <= 0 or not (0 <= cx < width) or not (0 <= cy < height):
            raise ValueError("configured camera_info intrinsics are invalid")
        return
    if int(data.get("width", 0)) <= 0 or int(data.get("height", 0)) <= 0:
        raise ValueError("camera_info dimensions are invalid")
    for key, length in (("k", 9), ("r", 9), ("p", 12)):
        values = data.get(key)
        if not isinstance(values, list) or len(values) != length or not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"camera_info {key} is invalid")


def _validate_rgb_metadata(path: Path) -> None:
    data = _json(path)
    if data.get("status") != "complete" or int(data.get("frame_count", 0)) < RGB_PRESENTATION_FRAME_COUNT:
        raise ValueError("RGB metadata is incomplete or shorter than the 540-frame presentation source range")
    first = _finite(data.get("first_image_stamp_s"), "first_image_stamp_s")
    last = _finite(data.get("last_image_stamp_s"), "last_image_stamp_s")
    minimum_span = (RGB_PRESENTATION_FRAME_COUNT - 1) * RGB_TIMESTAMP_PERIOD_S - RGB_TIMESTAMP_TOLERANCE_S
    if last <= first or last - first < minimum_span or float(data.get("nominal_fps", 0.0)) != 30.0:
        raise ValueError("RGB metadata does not cover source frames 0-539 at 30 fps")


def _validate_capture_manifest(path: Path) -> None:
    data = _json(path)
    from simulator.capture.manifest import capture_hash, validate_capture_for_slam

    if int(data.get("manifest_version", -1)) != 1:
        raise ValueError("presentation inputs must use the repaired capture manifest v1 contract")
    validate_capture_for_slam(path.parent, data)
    declared_capture_sha = str(data.get("capture_sha256", "")).lower()
    if not SHA256_PATTERN.fullmatch(declared_capture_sha):
        raise ValueError("capture manifest lacks a valid declared capture_sha256")
    if capture_hash(data) != declared_capture_sha:
        raise ValueError("capture manifest declared checksum does not match its repaired-v1 payload")
    bag = data.get("bag")
    if not isinstance(bag, dict) or bag.get("topics") != list(REQUIRED_CAPTURE_TOPICS):
        raise ValueError("capture manifest must contain the exact six repaired-v1 raw topics")
    if int(data.get("rgb", {}).get("frame_count", 0)) < RGB_PRESENTATION_FRAME_COUNT:
        raise ValueError("capture manifest RGB sequence is shorter than source frames 0-539")
    files = data.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("capture manifest file inventory is missing")
    for item in files:
        if not isinstance(item, dict) or not str(item.get("path", "")) or not SHA256_PATTERN.fullmatch(str(item.get("sha256", "")).lower()):
            raise ValueError("capture manifest contains an invalid file inventory entry")


def _validate_bag_metadata(path: Path) -> None:
    data = _json(path)
    if data.get("status") != "complete" or data.get("topics") != list(REQUIRED_CAPTURE_TOPICS):
        raise ValueError("bag metadata is incomplete")
    counts = data.get("counts", {})
    if int(counts.get("/sim/lidar/points", 0)) <= 0:
        raise ValueError("bag metadata has no LiDAR scans")
    first = _finite(data.get("first_stamp_s", {}).get("/sim/lidar/points"), "first LiDAR stamp")
    last = _finite(data.get("last_stamp_s", {}).get("/sim/lidar/points"), "last LiDAR stamp")
    if last <= first:
        raise ValueError("LiDAR timestamp range is invalid")


def _validate_rosbag2(path: Path) -> None:
    if not path.is_dir() or not (path / "metadata.yaml").is_file():
        raise ValueError("ROS bag must be a directory containing metadata.yaml")
    databases = [item for item in path.glob("*.db3") if item.stat().st_size > 0]
    if not databases:
        raise ValueError("ROS bag has no nonempty sqlite3 database")


def _validate_sensor_transforms(path: Path) -> None:
    data = _json(path)
    if data.get("units") != "m" or data.get("rotation_order") != "xyzw_ros":
        raise ValueError("sensor transform units/rotation convention is invalid")
    if not isinstance(data.get("transforms"), list) or not data["transforms"]:
        raise ValueError("sensor transforms are missing")
    topics = data.get("topics", {})
    if topics.get("lidar_points") != "/sim/lidar/points":
        raise ValueError("sensor transform LiDAR topic is invalid")


def _slam_pose_time_range(path: Path) -> tuple[float, float]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "quaternion_valid"}
    if len(rows) < 2 or not rows or not required.issubset(rows[0]):
        raise ValueError("SLAM trajectory schema is invalid")
    last = -math.inf
    for row in rows:
        stamp = _finite(row["timestamp_s"], "SLAM timestamp")
        if stamp <= last or float(row["quaternion_valid"]) != 1.0:
            raise ValueError("SLAM trajectory timestamps/quaternions are invalid")
        quaternion = [_finite(row[key], key) for key in ("qx", "qy", "qz", "qw")]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("SLAM quaternion is not normalized")
        last = stamp
    return float(rows[0]["timestamp_s"]), float(rows[-1]["timestamp_s"])


def _validate_slam_poses(path: Path) -> None:
    _slam_pose_time_range(path)


def _validate_capture_time_coverage(
    actual_start: float, actual_end: float, capture: dict[str, Any], label: str
) -> None:
    bag = capture.get("bag", {})
    capture_start = _finite(bag.get("first_clock_s"), "capture first clock")
    capture_end = _finite(bag.get("last_clock_s"), "capture last clock")
    if capture_end <= capture_start:
        raise ValueError("capture simulation-time range is invalid")
    tolerance = 1e-6
    if actual_start < capture_start - tolerance or actual_end > capture_end + tolerance:
        raise ValueError(f"{label} lies outside the capture simulation-time range")
    edge_allowance = max(1.0, (capture_end - capture_start) * 0.1)
    if actual_start - capture_start > edge_allowance + tolerance or capture_end - actual_end > edge_allowance + tolerance:
        raise ValueError(f"{label} does not cover the capture simulation-time range")


def _validate_slam_pose_coverage(path: Path, capture: dict[str, Any], slam: dict[str, Any]) -> None:
    """Bind estimator pose coverage to the source capture's simulation clock."""

    actual_start, actual_end = _slam_pose_time_range(path)
    _validate_capture_time_coverage(actual_start, actual_end, capture, "SLAM trajectory")
    if slam.get("producer_mode") == "rtabmap_database_export":
        producer = slam.get("producer", {})
        declared_start = _finite(producer.get("first_pose_timestamp_s"), "producer first pose timestamp")
        declared_end = _finite(producer.get("last_pose_timestamp_s"), "producer last pose timestamp")
        if not math.isclose(actual_start, declared_start, abs_tol=1e-6) or not math.isclose(
            actual_end, declared_end, abs_tol=1e-6
        ):
            raise ValueError("SLAM trajectory range does not match database-export provenance")


def _validate_frame_contract(path: Path) -> None:
    data = _json(path)
    frames = data.get("frames", {})
    for key in ("map", "odom", "sensor_rig", "camera_optical", "lidar_link"):
        if not str(frames.get(key, "")):
            raise ValueError(f"frame contract is missing {key}")


def _validate_slam_manifest(path: Path) -> None:
    data = _json(path)
    if data.get("status") != "complete" or not str(data.get("capture_id", "")):
        raise ValueError("SLAM manifest is incomplete")
    if not SHA256_PATTERN.fullmatch(str(data.get("capture_sha256", "")).lower()):
        raise ValueError("SLAM capture_sha256 is invalid")
    producer_mode_declared = "producer_mode" in data
    producer_mode = data.get("producer_mode")
    if producer_mode_declared and producer_mode != "rtabmap_database_export":
        raise ValueError("unsupported explicit SLAM producer mode")
    if producer_mode == "rtabmap_database_export":
        producer = data.get("producer", {})
        if not isinstance(producer, dict) or producer.get("status") != "complete":
            raise ValueError("SLAM database-export producer metadata is incomplete")
        if producer.get("producer_mode") != "rtabmap_database_export":
            raise ValueError("SLAM database-export producer mode is inconsistent")
        if producer.get("ground_truth_subscribed") is not False or producer.get("ground_truth_consumed") is not False:
            raise ValueError("SLAM database export ground-truth provenance is invalid")
        if int(producer.get("pose_count", 0)) < 2 or int(producer.get("map_point_count", 0)) <= 0:
            raise ValueError("SLAM database export is empty")
        producer_path = path.with_name("slam_producer.json")
        if not producer_path.is_file() or _json(producer_path) != producer:
            raise ValueError("SLAM database-export producer metadata does not match slam_producer.json")
        database = producer.get("database", {})
        if not isinstance(database, dict) or database.get("path") != "rtabmap.db" or not SHA256_PATTERN.fullmatch(str(database.get("sha256", "")).lower()):
            raise ValueError("SLAM database export database hash is invalid")
        database_path = path.with_name("rtabmap.db")
        declared_database_path = Path(str(data.get("database_path", "")))
        if not declared_database_path.is_absolute() or declared_database_path.resolve() != database_path.resolve():
            raise ValueError("SLAM manifest database path does not match the sibling database")
        if not database_path.is_file() or sha256_path(database_path) != str(database["sha256"]).lower():
            raise ValueError("SLAM database export database bytes do not match provenance")
        artifacts = producer.get("artifacts", {})
        for name in ("slam_poses.csv", "slam_map.ply", "slam_map.pcd"):
            artifact = artifacts.get(name, {}) if isinstance(artifacts, dict) else {}
            if artifact.get("path") != name or not SHA256_PATTERN.fullmatch(str(artifact.get("sha256", "")).lower()):
                raise ValueError(f"SLAM database export {name} provenance is invalid")
            artifact_path = path.with_name(name)
            if not artifact_path.is_file() or sha256_path(artifact_path) != str(artifact["sha256"]).lower():
                raise ValueError(f"SLAM database export {name} bytes do not match provenance")
        optimize_command = producer.get("optimization", {}).get("command", [])
        export_command = producer.get("map_export", {}).get("command", [])
        native_export = path.parent / "native_export"
        expected_optimize = [
            "rtabmap-export.exe", "--poses", "--poses_format", "10", "--opt", "0", "--save_in_db",
            "--output", "slam_optimized", "--output_dir", str(native_export), str(database_path),
        ]
        expected_export = [
            "rtabmap-export.exe", "--cloud", "--scan", "--poses", "--poses_format", "10", "--ascii",
            "--opt", "2", "--max_range", "100", "--voxel", "0.03", "--output", "slam_map",
            "--output_dir", str(native_export), str(database_path),
        ]
        if optimize_command != expected_optimize:
            raise ValueError("SLAM database optimization provenance is invalid")
        if export_command != expected_export:
            raise ValueError("SLAM database map-export provenance is invalid")
        tool = producer.get("tool", {})
        if tool.get("name") != "rtabmap-export" or not str(tool.get("version", "")):
            raise ValueError("SLAM database export tool provenance is invalid")
        return
    observer = data.get("observer", {})
    if not observer.get("fresh_odom") or not observer.get("fresh_map") or observer.get("ground_truth_subscribed") is not False:
        raise ValueError("SLAM observer provenance is invalid")
    if int(observer.get("map_point_count", 0)) <= 0 or int(observer.get("odom_sample_count", 0)) < 2:
        raise ValueError("SLAM observer output is empty")


def _map_snapshot_time_range(path: Path) -> tuple[float, float]:
    data = _json(path)
    snapshots = data.get("snapshots")
    if data.get("status") != "complete" or not isinstance(snapshots, list) or not snapshots:
        raise ValueError("map snapshot index is incomplete")
    cutoffs = [_finite(item.get("measurement_cutoff_s"), "map cutoff") for item in snapshots if isinstance(item, dict)]
    if len(cutoffs) < 2 or len(cutoffs) != len(snapshots) or any(right <= left for left, right in zip(cutoffs, cutoffs[1:])):
        raise ValueError("map snapshot cutoffs are invalid or non-increasing")
    return cutoffs[0], cutoffs[-1]


def _validate_map_snapshot_index(path: Path) -> None:
    _map_snapshot_time_range(path)


def _validate_map_snapshot_coverage(path: Path, capture: dict[str, Any]) -> None:
    start, end = _map_snapshot_time_range(path)
    _validate_capture_time_coverage(start, end, capture, "map snapshots")


def _validate_pcd(path: Path) -> None:
    lines = path.read_text(encoding="ascii", errors="strict").splitlines()
    if not lines or not lines[0].startswith("# .PCD"):
        raise ValueError("map state is not a PCD file")
    points = [line for line in lines if line.startswith("POINTS ")]
    data = [line for line in lines if line.startswith("DATA ")]
    if len(points) != 1 or int(points[0].split()[1]) <= 0 or data != ["DATA ascii"]:
        raise ValueError("PCD header or point count is invalid")


def _validate_object_records(path: Path) -> None:
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    else:
        rows = _json(path, list)
    if not rows:
        raise ValueError("object records are empty")
    identifiers = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("object record must be a mapping")
        track_id = int(row.get("track_id", -1))
        if track_id < 0 or track_id in identifiers:
            raise ValueError("object track IDs must be unique nonnegative integers")
        identifiers.add(track_id)
        for key in ("estimated_x_m", "estimated_y_m", "estimated_z_m"):
            _finite(row.get(key), key)
        if not str(row.get("depth_source", "")).startswith("lidar_projected_with_slam_pose"):
            raise ValueError("object record lacks LiDAR+SLAM provenance")


def _validate_observation_links(path: Path) -> None:
    rows = _jsonl(path)
    indices = []
    detections = 0
    for row in rows:
        indices.append(int(row.get("frame_index", -1)))
        values = row.get("detections")
        if not isinstance(values, list):
            raise ValueError("observation detections must be a list")
        detections += len(values)
    if indices != list(range(len(indices))) or detections <= 0:
        raise ValueError("observation links are noncontiguous or empty")


def _validate_perception_manifest(path: Path) -> None:
    data = _json(path)
    if data.get("status") != "complete" or data.get("ground_truth_consumed") is not False:
        raise ValueError("perception manifest provenance is invalid")
    if data.get("lidar_consumed_for_estimation") is not True or data.get("slam_consumed_for_estimation") is not True:
        raise ValueError("perception manifest did not consume LiDAR and SLAM")
    if int(data.get("track_count", 0)) <= 0 or int(data.get("frame_count", 0)) < MINIMUM_PRESENTATION_SOURCE_FRAMES:
        raise ValueError("perception manifest is empty or shorter than the 540-frame shot source contract")
    if not isinstance(data.get("slam_manifest_sha256"), str) or not SHA256_PATTERN.fullmatch(data["slam_manifest_sha256"]):
        raise ValueError("perception manifest lacks its exact G02 SLAM manifest binding")
    if Path(str(data.get("slam_artifact", ""))).name != "slam_map.pcd":
        raise ValueError("perception manifest must identify the G02 SLAM map artifact")
    pose = data.get("slam_pose_provenance")
    if not isinstance(pose, dict) or pose.get("slam_manifest_sha256") != data.get("slam_manifest_sha256"):
        raise ValueError("perception manifest lacks coherent G02 pose provenance")


CONTENT_VALIDATORS = {
    "video": _validate_video,
    "rgb_frame_index": _validate_rgb_frame_index,
    "camera_info": _validate_camera_info,
    "rgb_metadata": _validate_rgb_metadata,
    "capture_manifest": _validate_capture_manifest,
    "bag_metadata": _validate_bag_metadata,
    "rosbag2": _validate_rosbag2,
    "sensor_transforms": _validate_sensor_transforms,
    "slam_poses": _validate_slam_poses,
    "frame_contract": _validate_frame_contract,
    "slam_manifest": _validate_slam_manifest,
    "map_snapshot_index": _validate_map_snapshot_index,
    "pcd": _validate_pcd,
    "object_records": _validate_object_records,
    "observation_links": _validate_observation_links,
    "perception_manifest": _validate_perception_manifest,
}


def _git_output(repo_root: Path, *arguments: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), *arguments],
        text=True,
        capture_output=True,
    )
    if completed.returncode:
        raise ValueError(f"capture git revision validation failed: {' '.join(arguments)}")
    return completed.stdout.strip()


def _review_evidence_path(repo_root: Path, declared_path: object) -> Path:
    """Resolve a canonical repository-relative review artifact, rejecting escapes."""

    if not isinstance(declared_path, str) or not declared_path or "\\" in declared_path:
        raise ValueError("reviewed RGB capture entry has an invalid verdict path")
    relative = PurePosixPath(declared_path)
    if (
        relative.is_absolute()
        or relative.as_posix() != declared_path
        or not relative.parts
        or relative.parts[0].endswith(":")
        or any(part in ("", ".", "..") for part in relative.parts)
    ):
        raise ValueError("reviewed RGB capture entry has an invalid verdict path")

    root = repo_root.resolve(strict=True)
    try:
        evidence = root.joinpath(*relative.parts).resolve(strict=True)
        evidence.relative_to(root)
    except (FileNotFoundError, OSError, RuntimeError, ValueError):
        raise ValueError("reviewed RGB verdict must be an existing file inside the repository") from None
    if not evidence.is_file():
        raise ValueError("reviewed RGB verdict must be an existing regular file")
    return evidence


def validate_rgb_capture_acceptance(
    capture: dict[str, Any],
    artifact_hashes: dict[str, str],
    repo_root: Path,
    catalog_path: Path | None = None,
) -> dict[str, Any]:
    """Require exact membership in the reviewed production-capture catalog."""

    git_sha = str(capture.get("git_sha", "")).lower()
    if not re.fullmatch(r"[0-9a-f]{40}", git_sha):
        raise ValueError("capture git_sha must be a full 40-character commit hash")
    resolved = _git_output(repo_root, "rev-parse", "--verify", f"{git_sha}^{{commit}}")
    if resolved.lower() != git_sha:
        raise ValueError("capture git_sha does not resolve to the declared commit")
    producer_path = ROLE_SPECS["rgb"].producer_source_path
    producer_blob = _git_output(repo_root, "rev-parse", f"{git_sha}:{producer_path}").lower()
    current_blob = _git_output(repo_root, "hash-object", producer_path).lower()

    source = catalog_path.resolve() if catalog_path else (repo_root / DEFAULT_RGB_CAPTURE_CATALOG).resolve()
    catalog = _json(source)
    if catalog.get("schema_version") != 1 or catalog.get("mechanism") != "reviewed_exact_capture_allowlist":
        raise ValueError("RGB capture acceptance catalog schema/mechanism is invalid")
    entries = catalog.get("captures")
    if not isinstance(entries, list):
        raise ValueError("RGB capture acceptance catalog must contain a captures list")
    matches = [
        entry
        for entry in entries
        if isinstance(entry, dict)
        and entry.get("capture_sha256") == capture.get("capture_sha256")
        and entry.get("git_sha") == git_sha
    ]
    if len(matches) != 1:
        raise ValueError("capture is not present exactly once in the reviewed RGB capture allowlist")
    entry = matches[0]
    expected_keys = {
        "capture_id", "capture_sha256", "git_sha", "producer_source_path",
        "producer_blob_sha1", "required_artifact_sha256", "review",
        "presentation_classification", "presentation_transform",
    }
    if set(entry) != expected_keys:
        raise ValueError("reviewed RGB capture entry fields are invalid")
    if entry.get("capture_id") != capture.get("capture_id"):
        raise ValueError("reviewed RGB capture_id does not match")
    if entry.get("producer_source_path") != producer_path or entry.get("producer_blob_sha1") != producer_blob:
        raise ValueError("reviewed RGB producer source/blob identity does not match")
    if entry.get("required_artifact_sha256") != artifact_hashes:
        raise ValueError("reviewed RGB artifact hashes do not match the capture bundle")
    review = entry.get("review")
    if not isinstance(review, dict) or set(review) != {
        "status", "reviewer", "reviewed_utc", "verdict_path", "verdict_sha256",
    } or review.get("status") != "accepted":
        raise ValueError("reviewed RGB capture entry has no accepted review record")
    verdict_sha256 = str(review.get("verdict_sha256", ""))
    if not SHA256_PATTERN.fullmatch(verdict_sha256):
        raise ValueError("reviewed RGB capture entry has an invalid verdict binding")
    verdict = _review_evidence_path(repo_root, review.get("verdict_path"))
    classification = validate_presentation_classification(entry.get("presentation_classification"))
    verdict_digest = (
        source_text_sha256(verdict)
        if classification == GENERATED_TEST_FIXTURE_CLASSIFICATION
        else sha256_path(verdict)
    )
    if verdict_digest != verdict_sha256:
        raise ValueError("reviewed RGB verdict hash does not match the checked-in evidence")
    if not str(review.get("reviewer", "")).strip() or not str(review.get("reviewed_utc", "")).endswith("Z"):
        raise ValueError("reviewed RGB capture entry review identity/time is invalid")
    if producer_blob != current_blob:
        raise ValueError("capture producer blob differs from the current repaired-base producer source")
    presentation_transform = validate_presentation_transform(entry.get("presentation_transform"))
    return {
        "catalog_path": str(source),
        "catalog_sha256": sha256_path(source),
        "entry": entry,
        "presentation_classification": classification,
        "presentation_transform": presentation_transform,
    }


def _artifact_inventory(capture_manifest: dict[str, Any]) -> dict[str, str]:
    return {str(item["path"]): str(item["sha256"]).lower() for item in capture_manifest.get("files", [])}


def _require_capture_artifact(capture_path: Path, capture: dict[str, Any], artifact: Artifact, relative: str) -> None:
    expected_path = (capture_path.parent / relative).resolve()
    if artifact.path != expected_path:
        raise ValueError(f"{artifact.name} does not match capture manifest path {relative}")
    if _artifact_inventory(capture).get(Path(relative).as_posix()) != artifact.sha256:
        raise ValueError(f"{artifact.name} hash is not recorded by the capture producer")


def _require_capture_directory(capture_path: Path, capture: dict[str, Any], artifact: Artifact) -> None:
    inventory = _artifact_inventory(capture)
    capture_root = capture_path.parent
    files = [item for item in artifact.path.rglob("*") if item.is_file()]
    if not files:
        raise ValueError(f"{artifact.name} directory is empty")
    for path in files:
        relative = path.relative_to(capture_root).as_posix()
        if inventory.get(relative) != sha256_path(path):
            raise ValueError(f"{artifact.name} file is not recorded by the capture producer: {relative}")


def _validate_view_manifest(
    role: str,
    item: dict[str, Any],
    artifacts: dict[str, Artifact],
    spec: RoleSpec,
    repo_root: Path,
) -> None:
    if spec.view_producer_id is None:
        return
    data = _json(artifacts["view_manifest"].path)
    if data.get("schema_id") != "simulation.presentation.view_derivation.v1":
        raise ValueError("view manifest schema_id is invalid")
    if data.get("producer_id") != spec.view_producer_id or data.get("role") != role:
        raise ValueError("view manifest producer/role is invalid")
    if not spec.view_producer_available or spec.view_producer_source_path is None:
        raise ValueError(f"validated technical view producer is unavailable: {spec.view_producer_id}")
    expected_producer_hash = source_text_sha256(repo_root / spec.view_producer_source_path)
    if data.get("producer_source_sha256") != expected_producer_hash:
        raise ValueError("view manifest producer source hash mismatch")
    if data.get("capture_id") != item.get("capture_id"):
        raise ValueError("view manifest capture_id mismatch")
    if data.get("output_video_sha256") != artifacts["view_video"].sha256:
        raise ValueError("view manifest does not bind the rendered video hash")
    expected_sources = {
        name: artifact.sha256
        for name, artifact in artifacts.items()
        if name not in {"view_video", "view_manifest"}
    }
    if data.get("source_artifact_sha256") != expected_sources:
        raise ValueError("view manifest source hashes do not match the role artifacts")
    time_range = data.get("source_time_range_s")
    if not isinstance(time_range, list) or len(time_range) != 2:
        raise ValueError("view manifest source_time_range_s is invalid")
    start, end = (_finite(time_range[0], "view start"), _finite(time_range[1], "view end"))
    if end <= start:
        raise ValueError("view manifest source_time_range_s must be increasing")
    if role == "rgb":
        rows = _jsonl(artifacts["frame_index"].path)
        actual = (float(rows[0]["stamp_s"]), float(rows[-1]["stamp_s"]))
        if any(abs(observed - expected) > 1e-6 for observed, expected in zip((start, end), actual)):
            raise ValueError("RGB view manifest source_time_range_s must equal the actual frame-index stamps")
        capture_directory = artifacts["capture_manifest"].path.parent
        sensor_transforms = capture_directory / "sensor_transforms.json"
        from simulator.technical_lidar import load_camera_head_transform_artifact

        camera_head_trajectory, expected_camera_head_receipt = load_camera_head_transform_artifact(
            sensor_transforms
        )
        if camera_head_trajectory is not None:
            camera_head_trajectory.validate_image_timestamps(float(row["stamp_s"]) for row in rows)
        camera_head_receipt = data.get("camera_head_transform")
        if (
            expected_camera_head_receipt.get("artifact_declared") is True
            or camera_head_receipt is not None
        ) and camera_head_receipt != expected_camera_head_receipt:
            raise ValueError("RGB view manifest camera head transform binding is invalid")
        validate_presentation_transform(data.get("presentation_transform"))
    if data.get("map_version") != item.get("map_version") or data.get("object_state_version") != item.get("object_state_version"):
        raise ValueError("view manifest map/object version mismatch")


def _role_associations(role: str, item: dict[str, Any], artifacts: dict[str, Artifact]) -> None:
    capture = _json(artifacts["capture_manifest"].path)
    if capture.get("capture_id") != item.get("capture_id"):
        raise ValueError("capture_id does not match capture_manifest")
    run_root = artifacts["capture_manifest"].path.parent.parent
    if role == "rgb":
        rgb = capture["rgb"]
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["view_video"], str(rgb["video"]))
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["frame_index"], str(rgb["timestamp_index"]))
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["camera_info"], str(rgb["camera_info"]))
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["rgb_metadata"], str(rgb["metadata"]))
    elif role == "lidar":
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["scan_index"], "bag_metadata.json")
        expected_bag = (artifacts["capture_manifest"].path.parent / str(capture["bag"]["uri"])).resolve()
        if artifacts["returns"].path != expected_bag:
            raise ValueError("LiDAR returns directory does not match capture manifest")
        _require_capture_directory(artifacts["capture_manifest"].path, capture, artifacts["returns"])
        _require_capture_artifact(artifacts["capture_manifest"].path, capture, artifacts["calibration"], "sensor_transforms.json")
    if "slam_manifest" in artifacts:
        slam = _json(artifacts["slam_manifest"].path)
        if slam.get("capture_id") != item.get("capture_id") or slam.get("capture_sha256") != capture.get("capture_sha256"):
            raise ValueError("SLAM manifest capture association mismatch")
        try:
            slam_relative = artifacts["slam_manifest"].path.relative_to(run_root)
        except ValueError as exc:
            raise ValueError("SLAM manifest is outside the capture run") from exc
        if not slam_relative.parts or slam_relative.parts[0] != "slam":
            raise ValueError("SLAM manifest is not in the capture run's slam output")
        bag_path = Path(str(slam.get("bag_replayed", ""))).resolve()
        expected_bag = (artifacts["capture_manifest"].path.parent / str(capture["bag"]["uri"])).resolve()
        if bag_path != expected_bag:
            raise ValueError("SLAM manifest was not produced from the declared capture bag")
        if role == "pose" and artifacts["trajectory"].path != artifacts["slam_manifest"].path.with_name("slam_map_poses.csv"):
            raise ValueError("pose trajectory is not the repaired SLAM producer's slam_map_poses.csv")
        if role == "pose":
            _validate_slam_pose_coverage(artifacts["trajectory"].path, capture, slam)
        if role == "map":
            if artifacts["pose_association"].path != artifacts["slam_manifest"].path.with_name("slam_map_poses.csv"):
                raise ValueError("map pose association is not the repaired SLAM producer's slam_map_poses.csv")
            if artifacts["map_states"].path != artifacts["slam_manifest"].path.with_name("slam_map.pcd"):
                raise ValueError("map state is not the SLAM producer's slam_map.pcd")
            _validate_slam_pose_coverage(artifacts["pose_association"].path, capture, slam)
            _validate_map_snapshot_coverage(artifacts["snapshot_index"].path, capture)
        expected_map_version = artifacts["slam_manifest"].sha256
        if item.get("map_version") != expected_map_version:
            raise ValueError("map_version must equal the SLAM manifest hash")
    if role == "reconstruction":
        perception = _json(artifacts["perception_manifest"].path)
        try:
            perception_relative = artifacts["perception_manifest"].path.relative_to(run_root)
        except ValueError as exc:
            raise ValueError("perception manifest is outside the capture run") from exc
        if not perception_relative.parts or perception_relative.parts[0] != "perception":
            raise ValueError("perception manifest is not in the capture run's perception output")
        validate_perception_manifest_bindings(
            perception,
            artifacts["capture_manifest"].path.parent,
            artifacts["slam_manifest"].path.parent,
            artifacts["perception_manifest"].path.parent,
        )
        expected_records = (artifacts["perception_manifest"].path.parent / str(perception.get("estimated_inventory", ""))).resolve()
        expected_observations = (artifacts["perception_manifest"].path.parent / str(perception.get("annotations", ""))).resolve()
        if artifacts["object_records"].path != expected_records:
            raise ValueError("object records do not match the perception producer manifest")
        if artifacts["observation_links"].path != expected_observations:
            raise ValueError("observation links do not match the perception producer manifest")
        validate_perception_frame_coverage(
            perception,
            artifacts["capture_manifest"].path.parent,
            artifacts["observation_links"].path,
        )
        expected_rgb = (artifacts["perception_manifest"].path.parent / str(perception.get("video", ""))).resolve()
        expected_frames = (artifacts["perception_manifest"].path.parent / str(perception.get("frames", ""))).resolve()
        capture_rgb = capture["rgb"]
        if expected_rgb != (artifacts["capture_manifest"].path.parent / str(capture_rgb["video"])).resolve():
            raise ValueError("perception manifest RGB video does not match the capture")
        if expected_frames != (artifacts["capture_manifest"].path.parent / str(capture_rgb["timestamp_index"])).resolve():
            raise ValueError("perception manifest RGB frame index does not match the capture")
        expected_object_version = artifacts["perception_manifest"].sha256
        if item.get("object_state_version") != expected_object_version:
            raise ValueError("object_state_version must equal the perception manifest hash")


def validate_role(
    role_name: str,
    item: dict[str, Any],
    manifest_path: Path,
    repo_root: Path,
    known_storyboard_hashes: frozenset[str],
    rgb_capture_catalog: Path | None = None,
) -> ValidatedRole:
    contract = str(item.get("contract", ""))
    status = str(item.get("status", ""))
    provenance = str(item.get("provenance", ""))
    schema_id = str(item.get("schema_id", ""))
    producer_id = str(item.get("producer_id", ""))
    capture_id = str(item.get("capture_id", ""))
    map_version = item.get("map_version")
    object_state_version = item.get("object_state_version")
    errors: list[str] = []

    if provenance == "diagnostic_baseline":
        artifacts_value = item.get("artifacts")
        artifacts: dict[str, Artifact] = {}
        if schema_id != "simulation.presentation.diagnostic_baseline.v1" or producer_id != "repository.demo.baseline.v1":
            errors.append("diagnostic baseline schema/producer is invalid")
        if not isinstance(artifacts_value, dict) or set(artifacts_value) != {"view_video"}:
            errors.append("diagnostic baseline must declare only view_video")
        else:
            descriptor = artifacts_value["view_video"]
            if not isinstance(descriptor, dict):
                errors.append("diagnostic view_video descriptor is invalid")
            else:
                path = (manifest_path.parent / str(descriptor.get("path", ""))).resolve()
                declared = str(descriptor.get("sha256", "")).lower()
                if not path.is_file() or not SHA256_PATTERN.fullmatch(declared):
                    errors.append("diagnostic view_video path/hash is invalid")
                else:
                    actual = sha256_path(path)
                    if actual != declared:
                        errors.append("diagnostic view_video hash mismatch")
                    expected_path = (repo_root / DIAGNOSTIC_BASELINE_IDENTITY["video_path"]).resolve()
                    if path != expected_path or declared != DIAGNOSTIC_BASELINE_IDENTITY["video_sha256"]:
                        raise ValueError("diagnostic source is not the pinned repository baseline")
                    if _git_blob_oid(path) != DIAGNOSTIC_BASELINE_IDENTITY["video_git_blob"]:
                        raise ValueError("diagnostic source does not match the pinned repository blob")
                    if actual in known_storyboard_hashes:
                        raise ValueError(f"known storyboard content is forbidden as an input: {path}")
                    try:
                        _validate_video(path)
                    except (OSError, ValueError, json.JSONDecodeError) as exc:
                        errors.append(f"view_video: {exc}")
                    artifacts["view_video"] = Artifact("view_video", path, declared, "video")
        if status != "complete":
            errors.append("diagnostic baseline status is not complete")
        errors.append("provenance is diagnostic_baseline")
        return ValidatedRole(role_name, contract, status, provenance, schema_id, producer_id, capture_id, None, None, artifacts, tuple(errors))

    spec = ROLE_SPECS.get(contract)
    artifacts = {}
    if spec is None:
        errors.append(f"unknown role contract {contract}")
    else:
        if role_name != contract:
            errors.append("genuine role name must match its contract name")
        if schema_id != spec.schema_id:
            errors.append(f"schema_id must be {spec.schema_id}")
        if producer_id != spec.producer_id:
            errors.append(f"producer_id must be {spec.producer_id}")
        expected_producer_hash = source_text_sha256(repo_root / spec.producer_source_path)
        if item.get("producer_source_sha256") != expected_producer_hash:
            errors.append("producer source hash does not match the repository stage")
        if status != "complete":
            errors.append("status is not complete")
        if provenance != "genuine":
            errors.append("provenance is not genuine")
        if not capture_id:
            errors.append("capture_id is missing")
        values = item.get("artifacts")
        expected = {name for name, _ in spec.required_artifacts}
        if not isinstance(values, dict):
            errors.append("artifacts must be a mapping")
        elif set(values) != expected:
            errors.append(f"artifacts must exactly match {sorted(expected)}")
        else:
            for artifact_name, kind in spec.required_artifacts:
                descriptor = values[artifact_name]
                if not isinstance(descriptor, dict):
                    errors.append(f"artifact {artifact_name} descriptor must be a mapping")
                    continue
                path = (manifest_path.parent / str(descriptor.get("path", ""))).resolve()
                declared = str(descriptor.get("sha256", "")).lower()
                if not path.exists():
                    errors.append(f"artifact {artifact_name} is missing")
                    continue
                if not SHA256_PATTERN.fullmatch(declared):
                    errors.append(f"artifact {artifact_name} must declare a SHA-256 hash")
                    continue
                actual = sha256_path(path)
                if actual != declared:
                    errors.append(f"artifact {artifact_name} hash mismatch")
                    continue
                if actual in known_storyboard_hashes:
                    raise ValueError(f"known storyboard content is forbidden as an input: {path}")
                artifact = Artifact(artifact_name, path, actual, kind)
                artifacts[artifact_name] = artifact
                if kind != "view_manifest":
                    try:
                        CONTENT_VALIDATORS[kind](path)
                    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                        errors.append(f"artifact {artifact_name}: {exc}")
        if len(artifacts) == len(spec.required_artifacts):
            try:
                _role_associations(contract, item, artifacts)
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                errors.append(str(exc))
            try:
                _validate_view_manifest(contract, item, artifacts, spec, repo_root)
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                errors.append(str(exc))
            if contract == "rgb":
                try:
                    capture = _json(artifacts["capture_manifest"].path)
                    camera_info = _json(artifacts["camera_info"].path)
                    sensor_transforms = (
                        artifacts["camera_info"].path.parent
                        / str(camera_info.get("source", "sensor_transforms.json"))
                    ).resolve()
                    acceptance_hashes = {
                        name: artifacts[name].sha256
                        for name in ("view_video", "frame_index", "camera_info", "rgb_metadata", "capture_manifest")
                    }
                    acceptance_hashes["sensor_transforms"] = sha256_path(sensor_transforms)
                    acceptance = validate_rgb_capture_acceptance(
                        capture,
                        acceptance_hashes,
                        repo_root,
                        rgb_capture_catalog,
                    )
                    expected_acceptance = {
                        "mechanism": "reviewed_exact_capture_allowlist",
                        "catalog_sha256": acceptance["catalog_sha256"],
                        "presentation_classification": acceptance["presentation_classification"],
                        "presentation_transform": acceptance["presentation_transform"],
                        "cryptographic_execution_attestation": False,
                    }
                    if item.get("acceptance") != expected_acceptance:
                        raise ValueError("RGB role acceptance does not match its reviewed catalog")
                    receipt = _json(artifacts["view_manifest"].path)
                    if receipt.get("capture_acceptance") != expected_acceptance:
                        raise ValueError("RGB view receipt acceptance does not match its reviewed catalog")
                    if receipt.get("presentation_transform") != acceptance["presentation_transform"]:
                        raise ValueError("RGB view receipt transform does not match its reviewed catalog")
                except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    errors.append(str(exc))
    return ValidatedRole(
        role_name,
        contract,
        status,
        provenance,
        schema_id,
        producer_id,
        capture_id,
        str(map_version) if map_version is not None else None,
        str(object_state_version) if object_state_version is not None else None,
        artifacts,
        tuple(errors),
    )


def coherence_errors(roles: dict[str, ValidatedRole]) -> dict[str, list[str]]:
    errors = {name: [] for name in roles}
    genuine = [role for role in roles.values() if role.provenance == "genuine"]
    capture_ids = {role.capture_id for role in genuine if role.capture_id}
    if len(capture_ids) > 1:
        for role in genuine:
            errors[role.name].append("genuine roles do not share one capture_id")
    capture_hashes = {
        role.artifacts["capture_manifest"].sha256
        for role in genuine
        if "capture_manifest" in role.artifacts
    }
    if len(capture_hashes) > 1:
        for role in genuine:
            errors[role.name].append("genuine roles reference different capture manifests")
    slam_hashes = {
        role.artifacts["slam_manifest"].sha256
        for role in genuine
        if "slam_manifest" in role.artifacts
    }
    if len(slam_hashes) > 1:
        for role in genuine:
            errors[role.name].append("map-associated roles reference different SLAM manifests")
    view_hashes: dict[str, list[str]] = {}
    for role in genuine:
        if "view_video" in role.artifacts:
            view_hashes.setdefault(role.artifacts["view_video"].sha256, []).append(role.name)
    for owners in view_hashes.values():
        if len(owners) > 1:
            for owner in owners:
                errors[owner].append(f"view video hash is reused across roles: {', '.join(sorted(owners))}")
    return errors
