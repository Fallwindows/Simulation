"""Extract a bounded, hash-bound real-LiDAR target-support receipt.

This command reads rosbag2 SQLite/CDR directly and is intentionally CPU only.
Extraction requires an independently supplied aggregate input-binding digest;
``--inspect-binding`` exists only to inventory a candidate source set before it
is approved for extraction.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simulator.sensors.feature_selection import select_bbox_front_surface
from simulator.sensors.scan_projection import (
    CameraIntrinsics,
    project_lidar_scan,
    read_pointcloud2_sqlite,
    resolve_transform,
)


SOURCE_RELATIVE_PATHS = (
    "capture/capture_manifest.json",
    "capture/bag_metadata.json",
    "capture/effective_config.json",
    "capture/camera_info.json",
    "capture/sensor_transforms.json",
    "capture/rgb_frames.jsonl",
    "capture/sensors_bag/metadata.yaml",
    "capture/sensors_bag/sensors_bag_0.db3",
    "perception/perception_manifest.json",
    "perception/frame_annotations.jsonl",
    "perception/tracks.csv",
    "slam/slam_manifest.json",
    "slam/slam_poses.csv",
    "slam/native_export/slam_map_poses.txt",
)


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _hash_file_stable(path: Path) -> dict[str, object]:
    before = path.stat()
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"source must be a regular non-symlink file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    after = path.stat()
    identity_before = (before.st_size, before.st_mtime_ns)
    identity_after = (after.st_size, after.st_mtime_ns)
    if identity_before != identity_after:
        raise ValueError(f"source changed while hashing: {path}")
    return {
        "size_bytes": before.st_size,
        "sha256": digest.hexdigest(),
        "mtime_ns_at_read": before.st_mtime_ns,
    }


def _source_inventory(run_dir: Path) -> tuple[dict[str, dict[str, object]], str]:
    inventory: dict[str, dict[str, object]] = {}
    for relative in SOURCE_RELATIVE_PATHS:
        path = run_dir / relative
        if not path.is_file():
            raise ValueError(f"required source input is missing: {relative}")
        inventory[relative] = _hash_file_stable(path)
    binding_payload = {
        relative: {
            "size_bytes": record["size_bytes"],
            "sha256": record["sha256"],
        }
        for relative, record in inventory.items()
    }
    return inventory, hashlib.sha256(_canonical_json_bytes(binding_payload)).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _manifest_file_map(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records = manifest.get("files")
    if not isinstance(records, list):
        raise ValueError("capture manifest is missing its files list")
    result: dict[str, dict[str, Any]] = {}
    for record in records:
        if not isinstance(record, dict) or not isinstance(record.get("path"), str):
            raise ValueError("capture manifest contains a malformed file record")
        path = record["path"]
        if path in result:
            raise ValueError(f"capture manifest contains duplicate file record {path!r}")
        result[path] = record
    return result


def _require_hash_record(
    inventory: dict[str, dict[str, object]], relative: str, expected: dict[str, Any], label: str
) -> None:
    observed = inventory[relative]
    if observed["sha256"] != expected.get("sha256") or observed["size_bytes"] != expected.get("size_bytes"):
        raise ValueError(f"{label} does not match its authoritative hash/size record")


def _validate_manifest_chain(run_dir: Path, inventory: dict[str, dict[str, object]]) -> tuple[dict[str, Any], dict[str, Any]]:
    capture_manifest = _read_json(run_dir / "capture/capture_manifest.json")
    perception_manifest = _read_json(run_dir / "perception/perception_manifest.json")
    if capture_manifest.get("status") != "complete" or perception_manifest.get("status") != "complete":
        raise ValueError("capture and perception manifests must both be complete")
    capture_id = capture_manifest.get("capture_id")
    if capture_id != run_dir.name or perception_manifest.get("capture_id") != capture_id:
        raise ValueError("run directory, capture manifest, and perception manifest capture IDs disagree")
    files = _manifest_file_map(capture_manifest)
    for relative, manifest_name in (
        ("capture/bag_metadata.json", "bag_metadata.json"),
        ("capture/effective_config.json", "effective_config.json"),
        ("capture/camera_info.json", "camera_info.json"),
        ("capture/sensor_transforms.json", "sensor_transforms.json"),
        ("capture/rgb_frames.jsonl", "rgb_frames.jsonl"),
        ("capture/sensors_bag/metadata.yaml", "sensors_bag/metadata.yaml"),
        ("capture/sensors_bag/sensors_bag_0.db3", "sensors_bag/sensors_bag_0.db3"),
    ):
        if manifest_name not in files:
            raise ValueError(f"capture manifest is missing {manifest_name}")
        _require_hash_record(inventory, relative, files[manifest_name], manifest_name)

    capture_manifest_sha = inventory["capture/capture_manifest.json"]["sha256"]
    if perception_manifest.get("capture_manifest_sha256") != capture_manifest_sha:
        raise ValueError("perception manifest does not bind the current capture manifest")
    inputs = perception_manifest.get("inputs")
    if not isinstance(inputs, dict):
        raise ValueError("perception manifest is missing input provenance")
    try:
        capture_input = inputs["capture"]["manifest"]
        bag_files = inputs["raw_lidar"]["bag"]["files"]
        rgb_input = inputs["rgb"]
        slam_input = inputs["slam"]
    except (KeyError, TypeError) as exc:
        raise ValueError("perception manifest input provenance is malformed") from exc
    if capture_input.get("sha256") != capture_manifest_sha:
        raise ValueError("perception capture input hash is inconsistent")
    expected_bag = next(
        (record for record in bag_files if str(record.get("path", "")).endswith("sensors_bag_0.db3")), None
    )
    if not isinstance(expected_bag, dict):
        raise ValueError("perception manifest is missing its raw LiDAR database record")
    _require_hash_record(inventory, "capture/sensors_bag/sensors_bag_0.db3", expected_bag, "raw LiDAR database")
    _require_hash_record(inventory, "capture/rgb_frames.jsonl", rgb_input["frames"], "RGB timestamp index")
    _require_hash_record(inventory, "capture/sensor_transforms.json", rgb_input["sensor_transforms"], "sensor transforms")
    if slam_input["manifest"].get("sha256") != inventory["slam/slam_manifest.json"]["sha256"]:
        raise ValueError("perception manifest does not bind the current SLAM manifest")
    if slam_input["trajectory"].get("sha256") != inventory["slam/slam_poses.csv"]["sha256"]:
        raise ValueError("perception manifest does not bind the current legacy SLAM trajectory")
    if (run_dir / "slam/slam_map_poses.csv").exists():
        raise ValueError("legacy diagnostic run unexpectedly contains canonical slam_map_poses.csv")
    return capture_manifest, perception_manifest


def _find_jsonl_record(path: Path, key: str, expected: int) -> dict[str, Any]:
    matches: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get(key) == expected:
                matches.append(record)
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {path.name} record with {key}={expected}")
    return matches[0]


def _find_track(path: Path, track_id: int) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        matches = [row for row in csv.DictReader(handle) if row.get("track_id") == str(track_id)]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one tracks.csv row for track {track_id}")
    return matches[0]


def _legacy_pose_at(path: Path, timestamp_s: float, *, native_text: bool) -> dict[str, float]:
    records: list[dict[str, float]] = []
    if native_text:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip() or line.startswith("#"):
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 8:
                raise ValueError("legacy native pose row must contain eight values")
            records.append(dict(zip(("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"), values)))
    else:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                records.append({key: float(row[key]) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")})
    matches = [row for row in records if abs(row["timestamp_s"] - timestamp_s) <= 5e-10]
    if len(matches) != 1:
        raise ValueError(f"legacy pose artifact lacks one exact pose at {timestamp_s:.9f} s")
    if not np.isfinite(np.asarray(list(matches[0].values()), dtype=np.float64)).all():
        raise ValueError("legacy pose contains non-finite values")
    return matches[0]


def _indices_digest(indices: np.ndarray) -> str:
    canonical = np.asarray(indices, dtype="<i8")
    return hashlib.sha256(canonical.tobytes(order="C")).hexdigest()


def _stage_receipt(stages: dict[str, np.ndarray]) -> dict[str, dict[str, object]]:
    return {
        name: {"count": int(len(indices)), "raw_indices_sha256": _indices_digest(indices)}
        for name, indices in stages.items()
    }


def _git_identity(repo_root: Path) -> dict[str, object]:
    def run(*args: str) -> str:
        result = subprocess.run(
            ["git", "-C", str(repo_root), *args], check=True, capture_output=True, text=True
        )
        return result.stdout.strip()

    return {
        "git_sha": run("rev-parse", "HEAD"),
        "git_tree": run("show", "-s", "--format=%T", "HEAD"),
        "source_files": {
            str(path.relative_to(repo_root)).replace("\\", "/"): {
                "size_bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for path in (
                repo_root / "simulator/sensors/scan_projection.py",
                repo_root / "simulator/sensors/feature_selection.py",
                repo_root / "scripts/extract_scan_receipt.py",
            )
        },
    }


def _recheck_inventory(run_dir: Path, inventory: dict[str, dict[str, object]]) -> None:
    for relative, record in inventory.items():
        stat = (run_dir / relative).stat()
        if (stat.st_size, stat.st_mtime_ns) != (record["size_bytes"], record["mtime_ns_at_read"]):
            raise ValueError(f"source changed after hashing: {relative}")


def extract_receipt(args: argparse.Namespace) -> tuple[Path, str, dict[str, Any]]:
    run_dir = Path(args.run_dir).resolve()
    if not run_dir.is_dir():
        raise ValueError(f"run directory is missing: {run_dir}")
    output = Path(args.output).resolve()
    try:
        output.relative_to(run_dir)
    except ValueError as exc:
        raise ValueError("receipt output must remain under the selected run directory") from exc

    inventory, binding_sha256 = _source_inventory(run_dir)
    if binding_sha256 != args.expected_binding_sha256.lower():
        raise ValueError(
            f"input binding mismatch: observed {binding_sha256}, expected {args.expected_binding_sha256.lower()}"
        )
    capture_manifest, perception_manifest = _validate_manifest_chain(run_dir, inventory)

    camera_info = _read_json(run_dir / "capture/camera_info.json")
    transform_config = _read_json(run_dir / "capture/sensor_transforms.json")
    effective_config = _read_json(run_dir / "capture/effective_config.json")
    bag_metadata = _read_json(run_dir / "capture/bag_metadata.json")
    frames = transform_config.get("frames")
    transforms = transform_config.get("transforms")
    if not isinstance(frames, dict) or not isinstance(transforms, list):
        raise ValueError("sensor transform file is missing frames or transforms")
    if camera_info.get("observed_ros_message") is not False or camera_info.get("provenance") != "configured_intrinsics":
        raise ValueError("legacy camera calibration must be explicitly configured, not claimed as observed")
    expected_camera = frames.get("camera_optical")
    expected_lidar = frames.get("lidar_link")
    if camera_info.get("frame_id") != expected_camera:
        raise ValueError("camera calibration frame does not match the transform contract")
    intrinsics = CameraIntrinsics(
        width_px=int(camera_info["width_px"]),
        height_px=int(camera_info["height_px"]),
        fx_px=float(camera_info["fx_px"]),
        fy_px=float(camera_info["fy_px"]),
        cx_px=float(camera_info["cx_px"]),
        cy_px=float(camera_info["cy_px"]),
    )
    declared_intrinsics = transform_config.get("intrinsics")
    if not isinstance(declared_intrinsics, dict) or any(
        float(declared_intrinsics[key]) != float(camera_info[key])
        for key in ("width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px")
    ):
        raise ValueError("camera_info.json and sensor_transforms.json intrinsics disagree")
    lidar_config = effective_config.get("lidar")
    if not isinstance(lidar_config, dict):
        raise ValueError("effective config is missing the LiDAR profile")
    minimum_depth_m = float(lidar_config["min_range_m"])
    maximum_depth_m = float(lidar_config["max_range_m"])

    frame = _find_jsonl_record(run_dir / "capture/rgb_frames.jsonl", "frame_index", args.frame_index)
    annotation = _find_jsonl_record(run_dir / "perception/frame_annotations.jsonl", "frame_index", args.frame_index)
    if any(annotation.get(key) != frame.get(key) for key in ("frame_index", "stamp_s", "width", "height")):
        raise ValueError("annotation and RGB timestamp index frame metadata disagree")
    if frame.get("frame_id") != expected_camera or frame.get("width") != intrinsics.width_px or frame.get("height") != intrinsics.height_px:
        raise ValueError("RGB frame does not match camera frame or calibration dimensions")
    detections = annotation.get("detections")
    if not isinstance(detections, list):
        raise ValueError("annotation record has no detections list")
    detection_matches = [
        detection
        for detection in detections
        if detection.get("track_id") == args.track_id and detection.get("raw_track_id") == args.raw_track_id
    ]
    if len(detection_matches) != 1:
        raise ValueError("selected frame does not contain exactly one matching persistent/raw track observation")
    detection = detection_matches[0]
    bbox = detection.get("bbox_xyxy")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError("selected detection has no valid bbox_xyxy")
    if bbox != args.expected_bbox_xyxy:
        raise ValueError(f"selected detection bbox {bbox} does not match expected bbox {args.expected_bbox_xyxy}")
    track = _find_track(run_dir / "perception/tracks.csv", args.track_id)
    if track.get("source") != "rgb_lidar_slam_consolidated":
        raise ValueError("selected persistent track is not the expected legacy consolidated track type")

    frame_stamp_ns = round(float(frame["stamp_s"]) * 1_000_000_000)
    topics = transform_config.get("topics")
    if not isinstance(topics, dict) or not isinstance(topics.get("lidar_points"), str):
        raise ValueError("sensor transform file has no LiDAR topic contract")
    bag_counts = bag_metadata.get("counts")
    first_stamps = bag_metadata.get("first_stamp_s")
    if not isinstance(bag_counts, dict) or not isinstance(first_stamps, dict):
        raise ValueError("bag metadata is missing counts or first stamps")
    lidar_topic = topics["lidar_points"]
    if bag_counts.get(lidar_topic) != perception_manifest["inputs"]["raw_lidar"]["message_count"]:
        raise ValueError("bag and perception manifests disagree on LiDAR message count")
    if abs(float(first_stamps.get(lidar_topic)) - float(frame["stamp_s"])) > 5e-10:
        raise ValueError("selected legacy receipt is expected to use the first co-timed LiDAR scan")

    bag_record = read_pointcloud2_sqlite(
        run_dir / "capture/sensors_bag/sensors_bag_0.db3",
        topic=lidar_topic,
        timestamp_ns=frame_stamp_ns,
        expected_message_id=args.expected_message_id,
    )
    cloud = bag_record.cloud
    if cloud.frame_id != expected_lidar:
        raise ValueError("PointCloud2 frame does not match the calibration transform graph")
    if len(cloud.xyz_m) != args.expected_raw_point_count:
        raise ValueError(
            f"raw scan point count {len(cloud.xyz_m)} does not match expected {args.expected_raw_point_count}"
        )
    optical_from_lidar = resolve_transform(
        transforms, source_frame=str(expected_lidar), target_frame=str(expected_camera)
    )
    projected = project_lidar_scan(
        cloud.xyz_m,
        cloud.raw_point_indices,
        optical_from_lidar=optical_from_lidar,
        intrinsics=intrinsics,
        minimum_depth_m=minimum_depth_m,
        maximum_depth_m=maximum_depth_m,
    )
    support = select_bbox_front_surface(
        projected,
        tuple(float(value) for value in bbox),
        front_surface_band_m=args.front_surface_band_m,
        maximum_selected_points=args.maximum_selected_points,
    )
    if len(support.raw_point_indices) < args.minimum_selected_points:
        raise ValueError(
            f"target support has {len(support.raw_point_indices)} points; minimum is {args.minimum_selected_points}"
        )

    legacy_pose = _legacy_pose_at(run_dir / "slam/slam_poses.csv", float(frame["stamp_s"]), native_text=False)
    native_pose = _legacy_pose_at(
        run_dir / "slam/native_export/slam_map_poses.txt", float(frame["stamp_s"]), native_text=True
    )
    for key in legacy_pose:
        if not np.isclose(legacy_pose[key], native_pose[key], rtol=0.0, atol=5e-7):
            raise ValueError("legacy SLAM CSV and native export disagree at the selected timestamp")

    points = [
        {
            "raw_point_index": int(raw_index),
            "raw_xyz_m": [float(value) for value in raw_xyz],
            "optical_xyz_m": [float(value) for value in optical_xyz],
            "u_px": float(u),
            "v_px": float(v),
            "depth_m": float(depth),
        }
        for raw_index, raw_xyz, optical_xyz, u, v, depth in zip(
            support.raw_point_indices,
            support.raw_xyz_m,
            support.optical_xyz_m,
            support.u_px,
            support.v_px,
            support.depth_m,
        )
    ]
    receipt: dict[str, Any] = {
        "schema_version": 1,
        "status": "diagnostic_legacy_source",
        "acceptance_scope": {
            "production_acceptance": False,
            "reason": "source run predates the repaired canonical slam_map_poses.csv contract",
            "permitted_use": "auditable real-scan projection and target-support diagnostic for later overlay development",
            "prohibited_inferences": [
                "product class or SKU",
                "full object extent",
                "true object geometry",
                "final production acceptance",
            ],
        },
        "capture": {
            "capture_id": capture_manifest["capture_id"],
            "capture_sha256": capture_manifest["capture_sha256"],
            "input_binding_sha256": binding_sha256,
            "inputs": inventory,
        },
        "scan": {
            "scan_id": f"sqlite_message_id:{bag_record.message_id}",
            "sqlite_message_id": bag_record.message_id,
            "sqlite_topic_id": bag_record.topic_id,
            "topic": bag_record.topic,
            "type": bag_record.type_name,
            "serialization_format": bag_record.serialization_format,
            "timestamp_ns": bag_record.database_timestamp_ns,
            "timestamp_s": bag_record.database_timestamp_ns / 1_000_000_000.0,
            "frame_id": cloud.frame_id,
            "height": cloud.height,
            "width": cloud.width,
            "raw_point_count": len(cloud.xyz_m),
            "point_step": cloud.point_step,
            "row_step": cloud.row_step,
            "is_bigendian": cloud.is_bigendian,
            "is_dense": cloud.is_dense,
            "fields": [field.__dict__ for field in cloud.fields],
            "point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
        },
        "rgb_frame": {
            "frame_id": int(frame["frame_index"]),
            "timestamp_ns": frame_stamp_ns,
            "timestamp_s": float(frame["stamp_s"]),
            "camera_frame": frame["frame_id"],
            "width_px": int(frame["width"]),
            "height_px": int(frame["height"]),
            "scan_time_delta_s": (bag_record.database_timestamp_ns - frame_stamp_ns) / 1_000_000_000.0,
        },
        "target_observation": {
            "persistent_track_id": args.track_id,
            "raw_track_id": args.raw_track_id,
            "bbox_xyxy": bbox,
            "detector_class": detection.get("class"),
            "detector_source": detection.get("source"),
            "source_claim_limit": "RGB component observation; class is unknown and bbox is not a segmentation mask",
            "legacy_annotation_depth_point_count": detection.get("depth_point_count"),
            "selected_support_count": len(points),
            "selected_raw_point_indices": [int(value) for value in support.raw_point_indices],
            "points": points,
        },
        "projection": {
            "transform_notation": "T_A_from_B maps coordinates in frame B into frame A",
            "matrix_name": f"T_{expected_camera}_from_{expected_lidar}",
            "optical_from_lidar_row_major": optical_from_lidar.tolist(),
            "intrinsics": {
                "model": camera_info["model"],
                "provenance": camera_info["provenance"],
                "observed_ros_message": camera_info["observed_ros_message"],
                "fx_px": intrinsics.fx_px,
                "fy_px": intrinsics.fy_px,
                "cx_px": intrinsics.cx_px,
                "cy_px": intrinsics.cy_px,
                "width_px": intrinsics.width_px,
                "height_px": intrinsics.height_px,
            },
            "depth_limits_m": {"minimum": minimum_depth_m, "maximum": maximum_depth_m},
            "selection": {
                "method": "inclusive_bbox_then_nearest_depth_band_then_raw_index_ordered_bounded_decimation",
                "front_surface_band_m": args.front_surface_band_m,
                "maximum_selected_points": args.maximum_selected_points,
                "nearest_depth_m": support.nearest_depth_m,
                "front_surface_max_depth_m": support.front_surface_max_depth_m,
            },
            "stages": {
                **_stage_receipt(projected.stage_raw_indices),
                **_stage_receipt(support.stage_raw_indices),
            },
        },
        "legacy_slam_diagnostic": {
            "used_for_projection": False,
            "reason_not_used": "co-timed rigid LiDAR-to-camera projection needs only calibrated rig extrinsics",
            "frame": "map",
            "pose_at_frame_timestamp": legacy_pose,
            "native_export_pose_at_frame_timestamp": native_pose,
            "canonical_slam_map_poses_csv_present": False,
        },
        "implementation": _git_identity(REPO_ROOT),
    }
    _recheck_inventory(run_dir, inventory)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(receipt, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = output.with_name(output.name + ".tmp")
    temporary.write_text(payload, encoding="utf-8", newline="\n")
    os.replace(temporary, output)
    receipt_sha256 = hashlib.sha256(output.read_bytes()).hexdigest()
    return output, receipt_sha256, receipt


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output")
    parser.add_argument("--inspect-binding", action="store_true")
    parser.add_argument("--expected-binding-sha256")
    parser.add_argument("--frame-index", type=int, default=3)
    parser.add_argument("--track-id", type=int, default=429)
    parser.add_argument("--raw-track-id", type=int, default=63)
    parser.add_argument("--expected-bbox-xyxy", type=int, nargs=4, default=[1793, 998, 1851, 1047])
    parser.add_argument("--expected-message-id", type=int, default=24)
    parser.add_argument("--expected-raw-point-count", type=int, default=177241)
    parser.add_argument("--front-surface-band-m", type=float, default=0.12)
    parser.add_argument("--maximum-selected-points", type=int, default=64)
    parser.add_argument("--minimum-selected-points", type=int, default=1)
    args = parser.parse_args()
    if args.inspect_binding:
        if args.output or args.expected_binding_sha256:
            parser.error("--inspect-binding cannot be combined with extraction output or expected binding")
    else:
        if not args.output or not args.expected_binding_sha256:
            parser.error("extraction requires --output and --expected-binding-sha256")
        digest = args.expected_binding_sha256.lower()
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            parser.error("--expected-binding-sha256 must be a lowercase or uppercase SHA-256 hex digest")
    return args


def main() -> None:
    args = _parse_args()
    run_dir = Path(args.run_dir).resolve()
    if args.inspect_binding:
        inventory, binding = _source_inventory(run_dir)
        print(json.dumps({"input_binding_sha256": binding, "inputs": inventory}, indent=2, sort_keys=True))
        return
    output, receipt_sha256, receipt = extract_receipt(args)
    print(
        json.dumps(
            {
                "status": receipt["status"],
                "output": str(output),
                "sha256": receipt_sha256,
                "selected_support_count": receipt["target_observation"]["selected_support_count"],
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
