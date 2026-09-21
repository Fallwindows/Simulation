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
    if manifest.get("schema_version") != 1 or manifest.get("status") != "complete":
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
    renderer_hash = _sha256_lf_text(root / "simulator" / "technical_views.py")
    plan_hash = _sha256_lf_text(root / "config" / "technical_views.json")
    if manifest.get("renderer", {}).get("sha256") != renderer_hash:
        raise ValueError("technical delivery renderer hash does not match repository code")
    if manifest.get("plan", {}).get("sha256") != plan_hash:
        raise ValueError("technical delivery plan hash does not match repository configuration")
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise ValueError("technical delivery source receipt is missing")
    capture_id = str(source.get("capture_id", ""))
    source_id = str(source.get("source_id", ""))
    catalog_path = Path(source_catalog_path).resolve() if source_catalog_path else root / "config" / "technical_source_catalog.json"
    catalog_hash = _sha256_lf_text(catalog_path)
    catalog_source = _catalog_source(catalog_path, capture_id, source_id)
    _validate_source_binding(source, manifest, catalog_source, catalog_hash, root)
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
            receipt.get("schema_version") != 1
            or receipt.get("artifact_type") != "technical_source_view_receipt"
            or receipt.get("status") != "complete"
            or receipt.get("view_id") != view_id
            or receipt.get("presentation_role") != role
        ):
            raise ValueError(f"technical receipt identity is invalid for {view_id}")
        producer = receipt.get("producer", {})
        if producer != {"id": TECHNICAL_PRODUCER_ID, "renderer_sha256": renderer_hash, "plan_sha256": plan_hash}:
            raise ValueError(f"technical receipt producer binding is invalid for {view_id}")
        if receipt.get("source") != source:
            raise ValueError(f"technical receipt source binding differs for {view_id}")
        video_receipt = receipt.get("video", {})
        if video_receipt != {"path": video_path.name, "sha256": video_hash, **probe}:
            raise ValueError(f"technical receipt video binding is invalid for {view_id}")
        if receipt.get("derivation", {}).get("storyboard_pixels_consumed") is not False:
            raise ValueError(f"technical receipt lacks storyboard exclusion for {view_id}")
        source_time = source["simulation_time"]
        shots.append(
            TechnicalShotSource(
                shot_number,
                view_id,
                role,
                video_path,
                video_hash,
                frame_count,
                (float(source_time["start_s"]), float(source_time["end_s"])),
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
