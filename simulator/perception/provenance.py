"""Fail-closed provenance and frame-coverage checks for RGB tracking."""

from __future__ import annotations

import json
import math
from pathlib import Path, PurePosixPath
import re
from typing import Any

from simulator.capture.manifest import (
    REQUIRED_CAPTURE_TOPIC_TYPES,
    _require_canonical_windows_relative_path,
    _safe_relative_path,
    _windows_long_path,
    sha256_file,
    validate_capture_manifest_hash,
)


LIDAR_TOPIC = "/sim/lidar/points"
MINIMUM_PRESENTATION_SOURCE_FRAMES = 540
RGB_FPS = 30.0
RGB_PERIOD_S = 1.0 / RGB_FPS
RGB_TIMESTAMP_TOLERANCE_S = 0.001
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _same_run(capture: Path, slam: Path, output: Path) -> Path:
    if capture.name != "capture" or slam.name != "slam" or output.name != "perception":
        raise ValueError("perception inputs must use sibling capture, slam, and perception directories")
    if capture.parent != slam.parent or capture.parent != output.parent:
        raise ValueError("perception capture, SLAM, and output must belong to the same run")
    return capture.parent


def _inventory(capture: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["path"]): item for item in capture.get("files", []) if isinstance(item, dict)}


def _capture_file_binding(
    capture_root: Path,
    inventory: dict[str, dict[str, Any]],
    relative: str,
) -> dict[str, object]:
    relative = _safe_relative_path(relative, "perception sensor input")
    item = inventory.get(relative)
    path = capture_root / relative
    try:
        resolved = path.resolve(strict=True)
    except (FileNotFoundError, OSError) as error:
        raise ValueError(f"capture provenance is missing required artifact: {relative}") from error
    if not isinstance(item, dict) or not resolved.is_file() or not resolved.is_relative_to(capture_root):
        raise ValueError(f"capture provenance is missing required artifact: {relative}")
    _require_canonical_windows_relative_path(
        root_long_path=_windows_long_path(capture_root),
        candidate=path,
        declared_path=relative,
        field="perception sensor input",
    )
    digest = str(item.get("sha256", "")).lower()
    if not SHA256_PATTERN.fullmatch(digest) or sha256_file(resolved) != digest:
        raise ValueError(f"capture artifact hash mismatch: {relative}")
    if int(item.get("size_bytes", -1)) != resolved.stat().st_size:
        raise ValueError(f"capture artifact size mismatch: {relative}")
    return {"path": f"../capture/{relative}", "sha256": digest, "size_bytes": resolved.stat().st_size}


def _capture_bag_bindings(
    capture_root: Path,
    inventory: dict[str, dict[str, Any]],
    bag_uri: str,
) -> list[dict[str, object]]:
    bag_uri = _safe_relative_path(bag_uri, "perception raw LiDAR bag")
    bag_candidate = capture_root / bag_uri
    try:
        bag_root = bag_candidate.resolve(strict=True)
    except (FileNotFoundError, OSError) as error:
        raise ValueError("capture provenance is missing the raw LiDAR bag") from error
    if not bag_root.is_dir() or not bag_root.is_relative_to(capture_root):
        raise ValueError("capture raw LiDAR bag must be a directory under the capture root")
    root_long_path = _windows_long_path(capture_root)
    _require_canonical_windows_relative_path(
        root_long_path=root_long_path,
        candidate=bag_candidate,
        declared_path=bag_uri,
        field="perception raw LiDAR bag",
    )
    actual_files: list[str] = []
    for path in bag_root.rglob("*"):
        if not path.is_file():
            continue
        resolved = path.resolve(strict=True)
        if not resolved.is_relative_to(bag_root):
            raise ValueError("capture raw LiDAR bag file escapes its directory")
        relative = resolved.relative_to(capture_root).as_posix()
        _require_canonical_windows_relative_path(
            root_long_path=root_long_path,
            candidate=path,
            declared_path=relative,
            field="perception raw LiDAR bag file",
        )
        actual_files.append(relative)
    inventoried_files = sorted(relative for relative in inventory if relative.startswith(f"{bag_uri}/"))
    if sorted(actual_files) != inventoried_files:
        raise ValueError("capture raw LiDAR bag files do not exactly match the capture manifest inventory")
    bag_path = PurePosixPath(bag_uri)
    if any(PurePosixPath(relative).parent != bag_path for relative in inventoried_files):
        raise ValueError("capture raw LiDAR bag cannot contain nested storage files")
    metadata_path = f"{bag_uri}/metadata.yaml"
    database_paths = [relative for relative in inventoried_files if PurePosixPath(relative).suffix.casefold() == ".db3"]
    if metadata_path not in inventoried_files or not database_paths:
        raise ValueError("capture manifest must bind root-level bag metadata and SQLite storage")
    if set(inventoried_files) != {metadata_path, *database_paths}:
        raise ValueError("capture raw LiDAR bag can contain only root metadata.yaml and SQLite .db3 storage")
    bag_files = [_capture_file_binding(capture_root, inventory, relative) for relative in inventoried_files]
    return bag_files


def build_perception_input_bindings(
    capture_dir: str | Path,
    slam_dir: str | Path,
    output_dir: str | Path,
) -> dict[str, object]:
    """Validate producer inputs and return their exact manifest representation."""

    capture_root = Path(capture_dir).resolve()
    slam_root = Path(slam_dir).resolve()
    output_root = Path(output_dir).resolve()
    _same_run(capture_root, slam_root, output_root)

    capture_manifest_path = capture_root / "capture_manifest.json"
    slam_manifest_path = slam_root / "slam_manifest.json"
    if not capture_manifest_path.is_file() or not slam_manifest_path.is_file():
        raise ValueError("perception requires capture_manifest.json and slam_manifest.json")
    capture = _json(capture_manifest_path)
    canonical_capture_sha = validate_capture_manifest_hash(capture)
    capture_id = str(capture.get("capture_id", ""))

    slam = _json(slam_manifest_path)
    if slam.get("status") != "complete":
        raise ValueError("SLAM manifest is not complete")
    if str(slam.get("capture_id", "")) != capture_id:
        raise ValueError("capture and SLAM capture_id do not match")
    if str(slam.get("capture_sha256", "")).lower() != canonical_capture_sha:
        raise ValueError("SLAM manifest capture_sha256 does not match the canonical capture")
    expected_bag = (capture_root / str(capture["bag"]["uri"])).resolve()
    if Path(str(slam.get("bag_replayed", ""))).resolve() != expected_bag:
        raise ValueError("SLAM manifest was not produced from the capture bag")

    producer = slam.get("producer")
    artifacts = producer.get("artifacts") if isinstance(producer, dict) else None
    trajectory_binding = artifacts.get("slam_poses.csv") if isinstance(artifacts, dict) else None
    trajectory_path = slam_root / "slam_poses.csv"
    if not isinstance(trajectory_binding, dict) or trajectory_binding.get("path") != "slam_poses.csv":
        raise ValueError("SLAM manifest does not bind the consumed slam_poses.csv trajectory")
    trajectory_sha = str(trajectory_binding.get("sha256", "")).lower()
    if not SHA256_PATTERN.fullmatch(trajectory_sha) or not trajectory_path.is_file():
        raise ValueError("SLAM trajectory provenance is invalid")
    if sha256_file(trajectory_path) != trajectory_sha:
        raise ValueError("SLAM trajectory bytes do not match its producer manifest")

    inventory = _inventory(capture)
    rgb = capture["rgb"]
    video_binding = _capture_file_binding(capture_root, inventory, str(rgb["video"]))
    frames_binding = _capture_file_binding(capture_root, inventory, str(rgb["timestamp_index"]))
    metadata_binding = _capture_file_binding(capture_root, inventory, str(rgb["metadata"]))
    transforms_binding = _capture_file_binding(capture_root, inventory, "sensor_transforms.json")
    bag_uri = str(capture["bag"]["uri"])
    bag_files = _capture_bag_bindings(capture_root, inventory, bag_uri)

    return {
        "capture": {
            "capture_id": capture_id,
            "capture_sha256": canonical_capture_sha,
            "manifest": {
                "path": "../capture/capture_manifest.json",
                "sha256": sha256_file(capture_manifest_path),
            },
        },
        "rgb": {
            "video": video_binding,
            "frames": frames_binding,
            "metadata": metadata_binding,
            "sensor_transforms": transforms_binding,
        },
        "slam": {
            "manifest": {
                "path": "../slam/slam_manifest.json",
                "sha256": sha256_file(slam_manifest_path),
            },
            "trajectory": {
                "path": "../slam/slam_poses.csv",
                "sha256": trajectory_sha,
                "producer_path": "slam_poses.csv",
            },
        },
        "raw_lidar": {
            "topic": LIDAR_TOPIC,
            "type": REQUIRED_CAPTURE_TOPIC_TYPES[LIDAR_TOPIC],
            "message_count": int(capture["bag"]["counts"][LIDAR_TOPIC]),
            "bag": {
                "path": f"../capture/{bag_uri}",
                "uri": bag_uri,
                "storage_id": capture["bag"]["storage_id"],
                "files": bag_files,
            },
        },
    }


def validate_perception_manifest_bindings(
    perception: dict[str, Any],
    capture_dir: str | Path,
    slam_dir: str | Path,
    output_dir: str | Path,
) -> dict[str, object]:
    expected = build_perception_input_bindings(capture_dir, slam_dir, output_dir)
    capture = expected["capture"]
    slam = expected["slam"]
    if perception.get("capture_id") != capture["capture_id"]:
        raise ValueError("perception capture_id does not match its capture")
    if perception.get("capture_sha256") != capture["capture_sha256"]:
        raise ValueError("perception capture_sha256 does not match its canonical capture")
    if perception.get("capture_manifest_sha256") != capture["manifest"]["sha256"]:
        raise ValueError("perception capture manifest hash does not match")
    if perception.get("slam_manifest_sha256") != slam["manifest"]["sha256"]:
        raise ValueError("perception SLAM manifest hash does not match")
    if perception.get("inputs") != expected:
        raise ValueError("perception input bindings do not match the consumed capture, bag, RGB, and SLAM bytes")
    if "slam_artifact" in perception:
        raise ValueError("perception manifest must name the consumed SLAM trajectory, not a map artifact")
    return expected


def _jsonl(path: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    if not all(isinstance(row, dict) for row in rows):
        raise ValueError(f"JSONL objects required: {path}")
    return rows


def validate_perception_frame_coverage(
    perception: dict[str, Any],
    capture_dir: str | Path,
    annotations_path: str | Path,
) -> dict[str, object]:
    """Bind annotations to every captured RGB row and the 540-frame shot minimum."""

    capture_root = Path(capture_dir).resolve()
    capture = _json(capture_root / "capture_manifest.json")
    frames_path = capture_root / str(capture["rgb"]["timestamp_index"])
    metadata_path = capture_root / str(capture["rgb"]["metadata"])
    frames = _jsonl(frames_path)
    annotations = _jsonl(Path(annotations_path))
    metadata = _json(metadata_path)
    frame_count = len(frames)
    if frame_count < MINIMUM_PRESENTATION_SOURCE_FRAMES:
        raise ValueError("capture RGB index is shorter than the required contiguous source frames 0-539")
    if int(capture["rgb"].get("frame_count", -1)) != frame_count:
        raise ValueError("capture RGB manifest frame count does not match its timestamp index")
    if metadata.get("status") != "complete" or int(metadata.get("frame_count", -1)) != frame_count:
        raise ValueError("capture RGB metadata frame count does not match its timestamp index")
    if not math.isclose(float(metadata.get("nominal_fps", 0.0)), RGB_FPS, abs_tol=1e-9):
        raise ValueError("capture RGB metadata is not 30 fps")
    if int(perception.get("frame_count", -1)) != frame_count or len(annotations) != frame_count:
        raise ValueError("perception frame count must exactly match the associated capture RGB index")

    expected_indices = list(range(frame_count))
    frame_indices = [int(row.get("frame_index", -1)) for row in frames]
    annotation_indices = [int(row.get("frame_index", -1)) for row in annotations]
    if frame_indices != expected_indices or annotation_indices != expected_indices:
        raise ValueError("capture and perception frame indices must be contiguous from zero")
    stamps = [float(row.get("stamp_s", math.nan)) for row in frames]
    if not all(math.isfinite(stamp) for stamp in stamps):
        raise ValueError("capture RGB timestamps must be finite")
    if any(
        abs((later - earlier) - RGB_PERIOD_S) > RGB_TIMESTAMP_TOLERANCE_S
        for earlier, later in zip(stamps, stamps[1:])
    ):
        raise ValueError("capture RGB timestamps must be contiguous at 30 fps")
    if not math.isclose(float(capture["rgb"].get("first_stamp_s", math.nan)), stamps[0], abs_tol=1e-9) or not math.isclose(
        float(capture["rgb"].get("last_stamp_s", math.nan)), stamps[-1], abs_tol=1e-9
    ):
        raise ValueError("capture RGB manifest timestamp range does not match its timestamp index")
    if not math.isclose(float(metadata.get("first_image_stamp_s", math.nan)), stamps[0], abs_tol=1e-9) or not math.isclose(
        float(metadata.get("last_image_stamp_s", math.nan)), stamps[-1], abs_tol=1e-9
    ):
        raise ValueError("capture RGB metadata timestamp range does not match its timestamp index")
    dimensions = {(int(row.get("width", -1)), int(row.get("height", -1))) for row in frames}
    if len(dimensions) != 1 or dimensions != {(int(metadata.get("width", -2)), int(metadata.get("height", -2)))}:
        raise ValueError("capture RGB metadata dimensions do not match its timestamp index")
    for frame, annotation in zip(frames, annotations):
        if not math.isclose(float(annotation.get("stamp_s", math.nan)), float(frame["stamp_s"]), abs_tol=1e-9):
            raise ValueError("perception annotation timestamps do not match the capture RGB index")
        if int(annotation.get("width", -1)) != int(frame.get("width", -2)) or int(
            annotation.get("height", -1)
        ) != int(frame.get("height", -2)):
            raise ValueError("perception annotation dimensions do not match the capture RGB index")
    return {
        "status": "complete",
        "frame_count": frame_count,
        "first_frame_index": 0,
        "last_frame_index": frame_count - 1,
        "minimum_required_frames": MINIMUM_PRESENTATION_SOURCE_FRAMES,
    }
