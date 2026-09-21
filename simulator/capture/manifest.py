"""Canonical experiment/capture manifests and compatibility validation.

The hash inputs intentionally separate physical geometry and sensor/trajectory
configuration from appearance assets.  Offline SLAM consumes the former and
the immutable sensor bag; changing a texture must not invalidate that result.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
import subprocess
from pathlib import Path
from typing import Any, Iterable

CAPTURE_MANIFEST_VERSION = 2
REQUIRED_CAPTURE_TOPICS = (
    "/clock",
    "/sim/camera/rgb/image_raw",
    "/sim/lidar/points",
    "/tf",
    "/tf_static",
)


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


def validate_capture_manifest_hash(manifest: dict[str, Any]) -> str:
    """Return the canonical capture hash or reject a malformed/mismatched manifest."""

    if int(manifest.get("manifest_version", -1)) != CAPTURE_MANIFEST_VERSION:
        raise ValueError("unsupported capture manifest version")
    if manifest.get("status") != "complete":
        raise ValueError("capture manifest is not complete")
    for key in ("capture_id", "git_sha", "bag", "rgb", "files"):
        if key not in manifest:
            raise ValueError(f"capture manifest is missing required field: {key}")
    declared = manifest.get("capture_sha256")
    if not isinstance(declared, str) or len(declared) != 64:
        raise ValueError("capture manifest lacks a full capture_sha256")
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
    if int(value.get("manifest_version", -1)) != CAPTURE_MANIFEST_VERSION:
        raise ValueError("unsupported capture manifest version")
    if value.get("status") != "complete":
        raise ValueError("capture manifest staging input is not complete")
    finalized = dict(value)
    finalized.pop("capture_sha256", None)
    finalized["capture_sha256"] = capture_hash(finalized)
    temporary = output.with_name(f"{output.name}.tmp")
    try:
        write_json(temporary, finalized)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    decoded = json.loads(output.read_text(encoding="utf-8"))
    validate_capture_manifest_hash(decoded)
    return decoded


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
