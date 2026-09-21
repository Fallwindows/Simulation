"""Canonical experiment/capture manifests and compatibility validation.

The hash inputs intentionally separate physical geometry and sensor/trajectory
configuration from appearance assets.  Offline SLAM consumes the former and
the immutable sensor bag; changing a texture must not invalidate that result.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
from typing import Any, Iterable

CAPTURE_MANIFEST_VERSION = 2
REQUIRED_CAPTURE_TOPICS = (
    "/clock",
    "/sim/camera/rgb/image_raw",
    "/sim/lidar/points",
    "/tf",
    "/tf_static",
)
REQUIRED_CAPTURE_TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
    "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
_HEX_40 = re.compile(r"^[0-9a-fA-F]{40}$")
_HEX_64 = re.compile(r"^[0-9a-fA-F]{64}$")


def _jsonable(value: Any) -> Any:
    if dataclasses.is_dataclass(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, dict):
        return {str(key): _jsonable(value[key]) for key in sorted(value)}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, float):
        # JSON's representation is stable enough for the finite config values
        # used by this project, while rejecting accidental NaN/Inf inputs.
        if not __import__("math").isfinite(value):
            raise ValueError("non-finite values cannot be hashed")
        return value
    return value


def canonical_json(value: Any) -> bytes:
    return (json.dumps(_jsonable(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_sha(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _relative_file_hashes(root: Path, paths: Iterable[Path]) -> list[dict[str, str]]:
    entries: list[dict[str, str]] = []
    for path in sorted((Path(item).resolve() for item in paths), key=lambda item: item.as_posix()):
        if not path.is_file():
            continue
        try:
            relative = path.relative_to(root.resolve()).as_posix()
        except ValueError:
            relative = path.name
        entries.append({"path": relative, "sha256": sha256_file(path)})
    return entries


def build_experiment_hashes(scenario_path: str | Path, repo_root: str | Path | None = None) -> dict[str, Any]:
    """Build stable physical/sensor/trajectory/appearance hashes for a scenario."""

    from simulator.config.loader import load_scenario
    from simulator.environment.aisle_builder import build_aisle_layout
    from simulator.environment.retail_catalog import load_retail_catalog

    scenario_file = Path(scenario_path).resolve()
    root = Path(repo_root).resolve() if repo_root else scenario_file.parents[1]
    scenario = load_scenario(scenario_file)
    layout = build_aisle_layout(scenario.environment)
    catalog = load_retail_catalog(scenario.environment.asset_manifest_path)
    records = {asset.asset_key: asset for asset in catalog.assets}
    inventory = []
    for asset in layout.assets:
        record = records[asset.asset_key]
        inventory.append({
            "semantic_id": asset.semantic_id,
            "asset_key": asset.asset_key,
            "category": asset.category,
            "position_m": list(asset.position_m),
            "rotation_rpy_deg": list(asset.rotation_rpy_deg),
            "scale_xyz": list(asset.scale_xyz),
            "dimensions_m": list(record.dimensions_m),
        })
    primitives = [
        {"name": item.name, "center_m": list(item.center_m), "size_m": list(item.size_m), "kind": item.kind}
        for item in layout.primitives
    ]
    geometry_payload = {"primitives": primitives, "assets": inventory, "seed": layout.seed}
    sensor_payload = {
        "camera": dataclasses.asdict(scenario.camera),
        "lidar": dataclasses.asdict(scenario.lidar),
        "sensor_overrides": scenario.sensor_overrides,
    }
    trajectory_payload = dataclasses.asdict(scenario.trajectory)
    appearance_files = _relative_file_hashes(
        root,
        list((root / "assets" / "retail" / "usd").rglob("*")) + list((root / "assets" / "retail" / "textures").rglob("*")),
    )
    return {
        "geometry_sha256": sha256_json(geometry_payload),
        "inventory_sha256": sha256_json(inventory),
        "trajectory_sha256": sha256_json(trajectory_payload),
        "sensor_sha256": sha256_json(sensor_payload),
        "appearance_sha256": sha256_json(appearance_files),
        "inputs": {
            "scenario": scenario_file.relative_to(root).as_posix() if scenario_file.is_relative_to(root) else scenario_file.as_posix(),
            "asset_manifest": str(Path(scenario.environment.asset_manifest_path).resolve().relative_to(root)).replace("\\", "/")
            if Path(scenario.environment.asset_manifest_path).resolve().is_relative_to(root)
            else str(Path(scenario.environment.asset_manifest_path).resolve()),
            "appearance_files": appearance_files,
        },
        "git_sha": _git_sha(root),
    }


def capture_hash(manifest_without_hash: dict[str, Any]) -> str:
    payload = dict(manifest_without_hash)
    payload.pop("capture_sha256", None)
    return sha256_json(payload)


def _mapping(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"capture manifest {field} must be a mapping")
    return value


def _nonempty_string(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"capture manifest {field} must be a nonempty string")
    return value


def _finite_number(value: object, field: str, *, positive: bool = False, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"capture manifest {field} must be a finite number")
    number = float(value)
    if positive and number <= 0.0:
        raise ValueError(f"capture manifest {field} must be positive")
    if nonnegative and number < 0.0:
        raise ValueError(f"capture manifest {field} must be nonnegative")
    return number


def _nonnegative_integer(value: object, field: str, *, positive: bool = False) -> int:
    if type(value) is not int or value < (1 if positive else 0):
        qualifier = "positive" if positive else "nonnegative"
        raise ValueError(f"capture manifest {field} must be a {qualifier} integer")
    return value


def _safe_relative_path(value: object, field: str) -> str:
    path_text = _nonempty_string(value, field)
    path = PurePosixPath(path_text)
    if "\\" in path_text or path.is_absolute() or ":" in path_text or any(part in ("", ".", "..") for part in path.parts):
        raise ValueError(f"capture manifest {field} must be a safe POSIX relative path")
    return path_text


def validate_capture_manifest_v2(manifest: dict[str, Any], *, require_hash: bool) -> None:
    """Validate values and nested types of the production manifest-v2 contract."""

    if not isinstance(manifest, dict):
        raise ValueError("capture manifest must be a mapping")
    if type(manifest.get("manifest_version")) is not int or manifest["manifest_version"] != CAPTURE_MANIFEST_VERSION:
        raise ValueError("unsupported capture manifest version")
    if manifest.get("status") != "complete":
        raise ValueError("capture manifest is not complete")
    _nonempty_string(manifest.get("capture_id"), "capture_id")
    git_sha = _nonempty_string(manifest.get("git_sha"), "git_sha")
    if not _HEX_40.fullmatch(git_sha):
        raise ValueError("capture manifest git_sha must be 40 hexadecimal characters")
    _nonempty_string(manifest.get("scenario"), "scenario")
    _nonempty_string(manifest.get("rmw_implementation"), "rmw_implementation")
    _nonnegative_integer(manifest.get("ros_domain_id"), "ros_domain_id")
    _finite_number(manifest.get("duration_s"), "duration_s", positive=True)

    bag = _mapping(manifest.get("bag"), "bag")
    _safe_relative_path(bag.get("uri"), "bag.uri")
    if bag.get("storage_id") != "sqlite3":
        raise ValueError("capture manifest bag.storage_id must be sqlite3")
    topics = bag.get("topics")
    if not isinstance(topics, list) or topics != list(REQUIRED_CAPTURE_TOPICS):
        raise ValueError("capture manifest bag.topics must be the exact ordered v2 raw-topic list")
    topic_types = _mapping(bag.get("topic_types"), "bag.topic_types")
    if topic_types != REQUIRED_CAPTURE_TOPIC_TYPES:
        raise ValueError("capture manifest bag.topic_types must match the v2 raw-topic types")
    counts = _mapping(bag.get("counts"), "bag.counts")
    if set(counts) != set(REQUIRED_CAPTURE_TOPICS):
        raise ValueError("capture manifest bag.counts must contain exactly the v2 raw topics")
    for topic in REQUIRED_CAPTURE_TOPICS:
        _nonnegative_integer(counts.get(topic), f"bag.counts[{topic}]")
    first_clock = _finite_number(bag.get("first_clock_s"), "bag.first_clock_s", nonnegative=True)
    last_clock = _finite_number(bag.get("last_clock_s"), "bag.last_clock_s", nonnegative=True)
    if last_clock < first_clock:
        raise ValueError("capture manifest bag clock range is reversed")

    rgb = _mapping(manifest.get("rgb"), "rgb")
    for field in ("video", "timestamp_index", "camera_info", "metadata"):
        _safe_relative_path(rgb.get(field), f"rgb.{field}")
    if rgb.get("camera_info_provenance") != "configured_intrinsics":
        raise ValueError("capture manifest rgb.camera_info_provenance must be configured_intrinsics")
    _nonnegative_integer(rgb.get("frame_count"), "rgb.frame_count", positive=True)
    first_stamp = _finite_number(rgb.get("first_stamp_s"), "rgb.first_stamp_s", nonnegative=True)
    last_stamp = _finite_number(rgb.get("last_stamp_s"), "rgb.last_stamp_s", nonnegative=True)
    if last_stamp < first_stamp:
        raise ValueError("capture manifest RGB timestamp range is reversed")

    ground_truth = _mapping(manifest.get("ground_truth"), "ground_truth")
    _safe_relative_path(ground_truth.get("inventory_csv"), "ground_truth.inventory_csv")
    _safe_relative_path(ground_truth.get("inventory_json"), "ground_truth.inventory_json")
    if ground_truth.get("pose_topic") != "/sim/ground_truth/pose" or ground_truth.get("evaluation_only") is not True:
        raise ValueError("capture manifest ground_truth boundary is invalid")

    hashes = _mapping(manifest.get("hashes"), "hashes")
    for field in ("geometry_sha256", "inventory_sha256", "trajectory_sha256", "sensor_sha256", "appearance_sha256"):
        value = hashes.get(field)
        if not isinstance(value, str) or not _HEX_64.fullmatch(value):
            raise ValueError(f"capture manifest hashes.{field} must be 64 hexadecimal characters")
    _mapping(hashes.get("inputs"), "hashes.inputs")
    hashes_git = hashes.get("git_sha")
    if not isinstance(hashes_git, str) or not _HEX_40.fullmatch(hashes_git):
        raise ValueError("capture manifest hashes.git_sha must be 40 hexadecimal characters")
    _nonempty_string(manifest.get("software_versions"), "software_versions")

    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("capture manifest files must be a nonempty list")
    seen_paths: set[str] = set()
    for index, item in enumerate(files):
        entry = _mapping(item, f"files[{index}]")
        path_text = _safe_relative_path(entry.get("path"), f"files[{index}].path")
        normalized = path_text.casefold()
        if normalized == "capture_manifest.json":
            raise ValueError("capture manifest files cannot include capture_manifest.json")
        if normalized in seen_paths:
            raise ValueError(f"capture manifest files contains duplicate path: {path_text}")
        seen_paths.add(normalized)
        digest = entry.get("sha256")
        if not isinstance(digest, str) or not _HEX_64.fullmatch(digest):
            raise ValueError(f"capture manifest files[{index}].sha256 must be 64 hexadecimal characters")
        _nonnegative_integer(entry.get("size_bytes"), f"files[{index}].size_bytes")

    if require_hash:
        declared = manifest.get("capture_sha256")
        if not isinstance(declared, str) or not _HEX_64.fullmatch(declared):
            raise ValueError("capture manifest lacks a full capture_sha256")


def validate_capture_manifest_hash(manifest: dict[str, Any]) -> str:
    """Return the canonical capture hash or reject a malformed/mismatched manifest."""

    validate_capture_manifest_v2(manifest, require_hash=True)
    declared = manifest.get("capture_sha256")
    expected = capture_hash(manifest)
    if declared.lower() != expected:
        raise ValueError(f"capture manifest hash mismatch: declared {declared}, canonical {expected}")
    return expected


def finalize_capture_manifest(staging_path: str | Path, output_path: str | Path) -> dict[str, Any]:
    """Canonicalize an unhashed v2 staging manifest and write BOM-free UTF-8.

    PowerShell 5.1 staging files may contain a UTF-8 BOM, so only this staging
    boundary accepts ``utf-8-sig``. The final file is emitted through
    :func:`write_json` and is immediately re-read as strict UTF-8.
    """

    staging = Path(staging_path)
    output = Path(output_path)
    value = json.loads(staging.read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError("capture manifest staging input must be a JSON object")
    validate_capture_manifest_v2(value, require_hash=False)
    finalized = dict(value)
    finalized.pop("capture_sha256", None)
    finalized["capture_sha256"] = capture_hash(finalized)
    validate_capture_manifest_hash(finalized)
    temporary = output.with_name(f"{output.name}.tmp")
    try:
        write_json(temporary, finalized)
        decoded = json.loads(temporary.read_text(encoding="utf-8"))
        validate_capture_manifest_hash(decoded)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return finalized


def validate_capture_for_slam(capture_dir: str | Path, manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    """Validate the sensor-only interface required by offline SLAM.

    Ground-truth inventory and pose files are deliberately not required here;
    this is the isolation test that prevents evaluation data leakage into SLAM.
    """

    root = Path(capture_dir).resolve()
    manifest_path = root / "capture_manifest.json"
    data = manifest if manifest is not None else json.loads(manifest_path.read_text(encoding="utf-8"))
    if data.get("status") != "complete":
        raise ValueError("capture manifest is not complete")
    if int(data.get("manifest_version", -1)) != CAPTURE_MANIFEST_VERSION:
        raise ValueError("unsupported capture manifest version")
    missing = [topic for topic in REQUIRED_CAPTURE_TOPICS if topic not in data.get("bag", {}).get("topics", [])]
    if missing:
        raise ValueError(f"capture bag is missing required sensor topics: {missing}")
    rgb = data.get("rgb", {})
    if rgb.get("camera_info_provenance") != "configured_intrinsics":
        raise ValueError("capture must identify camera intrinsics as configured_intrinsics")
    camera_info_path = root / str(rgb.get("camera_info", ""))
    if not camera_info_path.is_file():
        raise ValueError("configured camera intrinsics artifact is missing")
    camera_info = json.loads(camera_info_path.read_text(encoding="utf-8"))
    if camera_info.get("provenance") != "configured_intrinsics" or camera_info.get("observed_ros_message") is not False:
        raise ValueError("camera intrinsics artifact has invalid provenance")
    bag_uri = root / str(data.get("bag", {}).get("uri", ""))
    if not bag_uri.is_dir():
        raise ValueError(f"capture bag does not exist: {bag_uri}")
    if not (root / "rgb_camera.mp4").is_file():
        raise ValueError("capture RGB video is missing")
    if not (root / "rgb_frames.jsonl").is_file():
        raise ValueError("capture RGB timestamp index is missing")
    return {"status": "valid", "capture_dir": str(root), "gt_required": False, "topics": list(data["bag"]["topics"])}


def write_json(path: str | Path, value: Any) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(canonical_json(value))
