"""Post-estimation inventory matching and artifact export.

This module is intentionally separate from :mod:`rgb_tracking`: it is the
first stage allowed to open ``inventory_ground_truth.csv``.  The estimator can
therefore be run on a capture with the evaluation-only files removed.
"""

from __future__ import annotations

import argparse
import csv
import html
import hashlib
import io
import json
import math
import subprocess
import unicodedata
import zipfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Iterable

import numpy as np

from evaluation.metrics import PoseSample, interpolate_pose, safe_quaternion


INVENTORY_FIELDS = [
    "estimated_id", "estimated_sku", "estimated_category", "relative_x_m", "relative_y_m", "relative_z_m",
    "map_x_m", "map_y_m", "map_z_m", "observations", "confidence", "spatially_associated_gt_id",
    "gt_x_m", "gt_y_m", "gt_z_m", "spatial_association_error_m", "category_agreement",
]

SLAM_ARTIFACT_NAMES = frozenset({
    "slam_map_poses.csv", "slam_map_keyframes.csv", "slam_odom_poses.csv", "map_to_odom.csv",
    "slam_poses.csv", "slam_map.pcd", "slam_map.ply",
})


def _git_sha(repo_root: Path) -> str | None:
    try:
        return subprocess.run(["git", "-C", str(repo_root), "rev-parse", "HEAD"], check=True, capture_output=True, text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _quat_to_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _interpolate_truth_start_pose(samples: list[PoseSample], timestamp_s: float) -> PoseSample:
    samples = sorted(samples, key=lambda sample: sample.timestamp_s)
    pose = interpolate_pose(samples, timestamp_s, max_gap_s=0.1)
    if pose is None:
        raise RuntimeError(f"No truth_sensor_rig pose brackets estimator start timestamp {timestamp_s:.9f}s")
    return pose


def _read_truth_start_pose(capture_dir: Path, timestamp_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate evaluation truth at the estimator's actual start timestamp."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str((capture_dir / "sensors_bag").resolve()).replace("\\", "/"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    samples: list[PoseSample] = []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/tf":
            continue
        message = deserialize_message(serialized, TFMessage)
        for transform in message.transforms:
            if transform.child_frame_id != "truth_sensor_rig":
                continue
            stamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) / 1_000_000_000.0
            q = safe_quaternion((transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w))
            position = tuple(float(value) for value in (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ))
            if q is None or not math.isfinite(stamp) or not all(math.isfinite(value) for value in position):
                continue
            samples.append(PoseSample(stamp, position, q))
    pose = _interpolate_truth_start_pose(samples, timestamp_s)
    return np.asarray(pose.position_m, dtype=np.float64), _quat_to_matrix(pose.orientation_xyzw)


def _start_relative_truth_rows(
    truth_rows: list[dict[str, str]], start_translation: np.ndarray, start_rotation: np.ndarray,
) -> list[dict[str, object]]:
    """Transform physical truth centers into the estimator's start-relative frame."""
    rows: list[dict[str, object]] = []
    for row in truth_rows:
        coordinates = [_float_or_none(row.get(key)) for key in ("x_m", "y_m", "z_m")]
        if any(value is None for value in coordinates):
            raise ValueError(f"Ground-truth item {row.get('semantic_id', '<unknown>')} has invalid coordinates")
        world = np.asarray(coordinates, dtype=np.float64)
        relative = (world - start_translation) @ start_rotation
        rows.append({
            "semantic_id": row["semantic_id"],
            "asset_key": row.get("asset_key", ""),
            "category": row.get("category", ""),
            "relative": relative,
            "world": world,
        })
    return rows


def _gt_start_relative(capture_dir: Path, timestamp_s: float) -> list[dict[str, object]]:
    start_t, start_r = _read_truth_start_pose(capture_dir, timestamp_s)
    return _start_relative_truth_rows(_load_csv(capture_dir / "inventory_ground_truth.csv"), start_t, start_r)

def _load_csv(path: Path) -> list[dict[str, str]]:
    _, rows = _load_csv_with_fields(path)
    return rows


def _load_csv_bytes(name: str, content: bytes) -> tuple[list[str], list[dict[str, str]]]:
    try:
        text = content.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{name} is not valid UTF-8 CSV") from exc
    with io.StringIO(text, newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or [])
        if len(fields) != len(set(fields)):
            raise ValueError(f"{name} contains duplicate CSV column names")
        return fields, list(reader)


def _load_csv_with_fields(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    return _load_csv_bytes(path.name, path.read_bytes())


def _sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _valid_sha256(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(character in "0123456789abcdef" for character in value)


def _read_complete_manifest(path: Path, name: str) -> tuple[dict[str, object], bytes]:
    try:
        content = path.read_bytes()
        value = json.loads(content.decode("utf-8-sig"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{name} is missing or invalid") from exc
    if not isinstance(value, dict) or value.get("status") != "complete":
        raise ValueError(f"{name} is not complete")
    return value, content


def _canonical_artifact_identity(path: str) -> str:
    """Return a Windows-aware identity for one canonical relative artifact path."""
    posix_path = PurePosixPath(path)
    windows_path = PureWindowsPath(path)
    segments = path.split("/")
    if (
        not path
        or "\\" in path
        or posix_path.is_absolute()
        or windows_path.is_absolute()
        or windows_path.drive
        or posix_path.as_posix() != path
        or any(part in {"", ".", ".."} for part in segments)
    ):
        raise ValueError(f"SLAM manifest contains an unsafe or noncanonical artifact path: {path!r}")
    normalized = unicodedata.normalize("NFC", path)
    if normalized != path:
        raise ValueError(f"SLAM manifest artifact path is not Unicode-normalized: {path!r}")
    for segment in segments:
        if segment.endswith((".", " ")) or any(char in '<>:"|?*' or ord(char) < 32 for char in segment):
            raise ValueError(f"SLAM manifest contains a noncanonical Windows artifact segment: {segment!r}")
        device_stem = segment.split(".", 1)[0].upper()
        if device_stem in {"CON", "PRN", "AUX", "NUL", *(f"COM{i}" for i in range(1, 10)), *(f"LPT{i}" for i in range(1, 10))}:
            raise ValueError(f"SLAM manifest contains a reserved Windows device name: {segment!r}")
    if path not in SLAM_ARTIFACT_NAMES:
        raise ValueError(f"SLAM manifest contains an unlisted artifact path: {path!r}")
    return normalized.casefold()


def _validate_map_provenance(slam_dir: Path, perception_dir: Path) -> tuple[dict[str, object], bytes, bytes]:
    """Verify optimized map inputs and retain validated cloud and pose bytes."""
    slam_manifest_path = slam_dir / "slam_manifest.json"
    perception_manifest_path = perception_dir / "perception_manifest.json"
    slam_manifest, slam_bytes = _read_complete_manifest(slam_manifest_path, "slam_manifest.json")
    perception_manifest, perception_bytes = _read_complete_manifest(perception_manifest_path, "perception_manifest.json")

    map_version = slam_manifest.get("map_version")
    if not _valid_sha256(map_version):
        raise ValueError("SLAM manifest has an invalid map_version")
    if perception_manifest.get("map_version") != map_version:
        raise ValueError("perception manifest map_version does not match the SLAM map")
    if perception_manifest.get("slam_manifest_sha256") != _sha256(slam_bytes):
        raise ValueError("perception manifest was produced from a different slam_manifest.json")
    recorded_manifest_size = perception_manifest.get("slam_manifest_size_bytes")
    if not isinstance(recorded_manifest_size, int) or isinstance(recorded_manifest_size, bool) or recorded_manifest_size != len(slam_bytes):
        raise ValueError("perception manifest slam_manifest_size_bytes does not match slam_manifest.json")

    if slam_manifest.get("map_frame_id") != "map" or slam_manifest.get("optimized") is not True:
        raise ValueError("SLAM manifest does not declare a complete optimized map-frame output")
    graph_version = slam_manifest.get("graph_pose_version")
    if not _valid_sha256(graph_version) or slam_manifest.get("pre_publish_graph_version") != graph_version:
        raise ValueError("SLAM manifest optimized graph versions are invalid or disagree")
    observer = slam_manifest.get("observer")
    if not isinstance(observer, dict) or observer.get("status") != "complete":
        raise ValueError("SLAM manifest does not embed a complete observer record")
    observer_fields = {
        "map_version": "map_version",
        "graph_pose_version": "graph_pose_version",
        "pre_publish_graph_version": "pre_publish_graph_version",
        "map_pose_frame_id": "map_frame_id",
    }
    for observer_field, manifest_field in observer_fields.items():
        if observer.get(observer_field) != slam_manifest.get(manifest_field):
            raise ValueError(f"SLAM manifest observer disagrees on {observer_field}")
    if observer.get("map_graph_matches_final_cloud") is not True or observer.get("optimized_pose_graph_complete") is not True or slam_manifest.get("optimized") is not True:
        raise ValueError("SLAM observer does not confirm the final optimized graph/cloud")

    observer_record = slam_manifest.get("observer_artifact")
    observer_path = slam_dir / "slam_observer.json"
    if not isinstance(observer_record, dict) or observer_record.get("path") != "slam_observer.json" or not observer_path.is_file():
        raise ValueError("SLAM manifest is missing its observer artifact record")
    observer_bytes = observer_path.read_bytes()
    observer_size = observer_record.get("size_bytes")
    observer_digest = observer_record.get("sha256")
    if not isinstance(observer_size, int) or isinstance(observer_size, bool) or observer_size != len(observer_bytes) or not _valid_sha256(observer_digest) or observer_digest != _sha256(observer_bytes):
        raise ValueError("slam_observer.json does not match its SLAM manifest integrity record")
    try:
        observer_json = json.loads(observer_bytes.decode("utf-8-sig"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("slam_observer.json is invalid") from exc
    if observer_json != observer:
        raise ValueError("slam_observer.json content differs from the observer embedded in slam_manifest.json")

    artifacts = slam_manifest.get("artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("SLAM manifest is missing its artifact list")
    paths: set[str] = set()
    for record in artifacts:
        if not isinstance(record, dict) or not isinstance(record.get("path"), str) or not record["path"].strip():
            raise ValueError("SLAM manifest contains a malformed artifact record")
        path_identity = _canonical_artifact_identity(record["path"])
        if path_identity in paths:
            raise ValueError(f"SLAM manifest contains a duplicate artifact path: {record['path']}")
        paths.add(path_identity)
    expected_paths = {name.casefold() for name in SLAM_ARTIFACT_NAMES}
    if paths != expected_paths:
        missing_paths = sorted(expected_paths - paths)
        extra_paths = sorted(paths - expected_paths)
        raise ValueError(f"SLAM manifest artifact set is incomplete or unexpected; missing={missing_paths}, extra={extra_paths}")

    cloud_records = [record for record in artifacts if record.get("path") == "slam_map.ply"]
    if len(cloud_records) != 1:
        raise ValueError("SLAM manifest must contain exactly one artifact record for slam_map.ply")
    cloud_record = cloud_records[0]
    if cloud_record.get("role") != "final_optimized_cloud" or cloud_record.get("frame_id") != "map" or cloud_record.get("optimized") is not True or cloud_record.get("map_version") != map_version:
        raise ValueError("slam_map.ply is not declared as the final optimized cloud for this map_version")
    cloud_path = slam_dir / "slam_map.ply"
    if not cloud_path.is_file():
        raise ValueError("SLAM final map cloud is missing: slam_map.ply")
    cloud_bytes = cloud_path.read_bytes()
    expected_cloud_size = cloud_record.get("size_bytes")
    expected_cloud_hash = cloud_record.get("sha256")
    if not isinstance(expected_cloud_size, int) or isinstance(expected_cloud_size, bool) or expected_cloud_size != len(cloud_bytes) or not _valid_sha256(expected_cloud_hash) or expected_cloud_hash != _sha256(cloud_bytes):
        raise ValueError("slam_map.ply size or SHA-256 does not match its SLAM manifest record")
    if perception_manifest.get("slam_cloud_ply_sha256") != expected_cloud_hash:
        raise ValueError("perception manifest slam cloud SHA-256 does not match slam_map.ply")
    perception_cloud_size = perception_manifest.get("slam_cloud_ply_size_bytes")
    if not isinstance(perception_cloud_size, int) or isinstance(perception_cloud_size, bool) or perception_cloud_size != expected_cloud_size:
        raise ValueError("perception manifest slam cloud size does not match slam_map.ply")

    pose_records = [record for record in artifacts if record.get("path") == "slam_poses.csv"]
    if len(pose_records) != 1:
        raise ValueError("SLAM manifest must contain exactly one artifact record for slam_poses.csv")
    pose_record = pose_records[0]
    if pose_record.get("role") != "legacy_map_trajectory" or pose_record.get("frame_id") != "map" or pose_record.get("optimized") is not True or pose_record.get("map_version") != map_version:
        raise ValueError("slam_poses.csv is not declared as the optimized map trajectory for this map_version")
    pose_path = slam_dir / "slam_poses.csv"
    if not pose_path.is_file():
        raise ValueError("SLAM estimator pose stream is missing: slam_poses.csv")
    pose_bytes = pose_path.read_bytes()
    expected_pose_size = pose_record.get("size_bytes")
    expected_pose_hash = pose_record.get("sha256")
    if not isinstance(expected_pose_size, int) or isinstance(expected_pose_size, bool) or expected_pose_size != len(pose_bytes) or not _valid_sha256(expected_pose_hash) or expected_pose_hash != _sha256(pose_bytes):
        raise ValueError("slam_poses.csv size or SHA-256 does not match its SLAM manifest record")

    provenance = {
        "map_version": map_version,
        "slam_manifest": {"path": "../slam/slam_manifest.json", "size_bytes": len(slam_bytes), "sha256": _sha256(slam_bytes)},
        "perception_manifest": {"path": "perception_manifest.json", "size_bytes": len(perception_bytes), "sha256": _sha256(perception_bytes)},
        "slam_map_cloud": {"path": "../slam/slam_map.ply", "size_bytes": len(cloud_bytes), "sha256": _sha256(cloud_bytes), "frame_id": "map", "role": "final_optimized_cloud"},
        "slam_poses": {"path": "../slam/slam_poses.csv", "size_bytes": len(pose_bytes), "sha256": _sha256(pose_bytes), "frame_id": "map", "role": "legacy_map_trajectory"},
    }
    return provenance, cloud_bytes, pose_bytes


def _validate_estimate_map_versions(fields: list[str], estimates: list[dict[str, str]], map_version: str) -> None:
    if "map_version" not in fields:
        raise ValueError("estimated_inventory.csv is missing the required map_version column")
    if any(row.get("map_version") != map_version for row in estimates):
        raise ValueError("estimated inventory contains a missing or inconsistent map_version")


def _float_or_none(value: object) -> float | None:
    if value is None or str(value).strip() in {"", "None", "nan"}:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _coordinate_triplet(row: dict[str, str], keys: tuple[str, str, str]) -> np.ndarray | None:
    values = [_float_or_none(row.get(key)) for key in keys]
    if any(value is None for value in values):
        return None
    return np.asarray(values, dtype=np.float64)


def _minimum_cost_maximum_cardinality_associations(
    estimate_points: list[np.ndarray | None],
    truth_rows: list[dict[str, object]],
    threshold_m: float,
    eligible_truth_ids: set[str] | None = None,
) -> dict[int, tuple[int, float]]:
    """Find a maximum-cardinality gated assignment, minimizing total distance.

    Successive shortest augmenting paths over a residual bipartite flow graph
    permit reassignment of earlier pairs when that increases final match count.
    """
    estimate_count = len(estimate_points)
    truth_count = len(truth_rows)
    source = 0
    estimate_base = 1
    truth_base = estimate_base + estimate_count
    sink = truth_base + truth_count
    graph: list[list[list[float | int]]] = [[] for _ in range(sink + 1)]

    def add_edge(start: int, end: int, capacity: int, cost: float) -> list[float | int]:
        forward: list[float | int] = [end, len(graph[end]), capacity, cost]
        reverse: list[float | int] = [start, len(graph[start]), 0, -cost]
        graph[start].append(forward)
        graph[end].append(reverse)
        return forward

    for estimate_index in range(estimate_count):
        add_edge(source, estimate_base + estimate_index, 1, 0.0)
    for truth_index in range(truth_count):
        add_edge(truth_base + truth_index, sink, 1, 0.0)

    assignment_edges: list[tuple[int, int, list[float | int], float]] = []
    for estimate_index, estimate in enumerate(estimate_points):
        if estimate is None:
            continue
        for truth_index, truth in enumerate(truth_rows):
            if eligible_truth_ids is not None and str(truth["semantic_id"]) not in eligible_truth_ids:
                continue
            point = truth.get("relative")
            if point is None:
                continue
            distance = float(np.linalg.norm(estimate - np.asarray(point, dtype=np.float64)))
            if distance <= threshold_m:
                edge = add_edge(estimate_base + estimate_index, truth_base + truth_index, 1, distance)
                assignment_edges.append((estimate_index, truth_index, edge, distance))

    import heapq

    potentials = [0.0] * len(graph)
    while True:
        distances = [math.inf] * len(graph)
        previous: list[tuple[int, int] | None] = [None] * len(graph)
        distances[source] = 0.0
        queue = [(0.0, source)]
        while queue:
            distance_here, node = heapq.heappop(queue)
            if distance_here > distances[node] + 1e-12:
                continue
            for edge_index, edge in enumerate(graph[node]):
                target = int(edge[0])
                if int(edge[2]) <= 0:
                    continue
                reduced_cost = float(edge[3]) + potentials[node] - potentials[target]
                if reduced_cost < 0.0 and reduced_cost > -1e-10:
                    reduced_cost = 0.0
                candidate_distance = distance_here + reduced_cost
                if candidate_distance < distances[target] - 1e-12:
                    distances[target] = candidate_distance
                    previous[target] = (node, edge_index)
                    heapq.heappush(queue, (candidate_distance, target))
        if not math.isfinite(distances[sink]):
            break
        for node, distance_value in enumerate(distances):
            if math.isfinite(distance_value):
                potentials[node] += distance_value
        node = sink
        while node != source:
            step = previous[node]
            if step is None:
                raise RuntimeError("assignment residual path is incomplete")
            parent, edge_index = step
            edge = graph[parent][edge_index]
            reverse_index = int(edge[1])
            edge[2] = int(edge[2]) - 1
            graph[node][reverse_index][2] = int(graph[node][reverse_index][2]) + 1
            node = parent

    return {
        estimate_index: (truth_index, distance)
        for estimate_index, truth_index, edge, distance in assignment_edges
        if int(edge[2]) == 0
    }


def _match_rows(
    estimate_rows: list[dict[str, str]],
    truth_rows: list[dict[str, object]],
    threshold_m: float = 0.35,
    eligible_truth_ids: set[str] | None = None,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    """Associate coordinates spatially; this is not object-identity matching.

    Visibility eligibility is an independent input. When it is unavailable, this
    function reports unconditioned spatial associations and leaves visible recall
    unavailable instead of treating all authored truth as visible.
    """
    if not math.isfinite(threshold_m) or threshold_m <= 0.0:
        raise ValueError("threshold_m must be a positive finite distance")
    truth_ids = {str(item["semantic_id"]) for item in truth_rows}
    if eligible_truth_ids is not None and not eligible_truth_ids.issubset(truth_ids):
        raise ValueError("eligible_truth_ids contains IDs absent from ground truth")

    estimate_points = [_coordinate_triplet(row, ("estimated_x_m", "estimated_y_m", "estimated_z_m")) for row in estimate_rows]
    associations = _minimum_cost_maximum_cardinality_associations(
        estimate_points, truth_rows, threshold_m,
    )
    eligible_associations = (
        len(_minimum_cost_maximum_cardinality_associations(
            estimate_points, truth_rows, threshold_m, eligible_truth_ids,
        ))
        if eligible_truth_ids is not None else 0
    )

    output: list[dict[str, object]] = []
    errors: list[float] = []
    invalid_coordinate_count = 0
    for estimate_index, row in enumerate(estimate_rows):
        estimate = estimate_points[estimate_index]
        if estimate is None:
            invalid_coordinate_count += 1
        association = associations.get(estimate_index)
        result: dict[str, object] = {
            "estimated_id": row.get("track_id", ""),
            "estimated_sku": "unknown_product",
            "estimated_category": row.get("class", "unknown_product"),
            "relative_x_m": None if estimate is None else float(estimate[0]),
            "relative_y_m": None if estimate is None else float(estimate[1]),
            "relative_z_m": None if estimate is None else float(estimate[2]),
            "map_x_m": _float_or_none(row.get("map_x_m")),
            "map_y_m": _float_or_none(row.get("map_y_m")),
            "map_z_m": _float_or_none(row.get("map_z_m")),
            "observations": int(row.get("3d_observation_count", 0)),
            "confidence": None,
            "spatially_associated_gt_id": "",
            "gt_x_m": "", "gt_y_m": "", "gt_z_m": "", "spatial_association_error_m": "",
            "category_agreement": None,
        }
        if association is not None:
            truth_index, distance = association
            truth = truth_rows[truth_index]
            truth_id = str(truth["semantic_id"])
            errors.append(distance)
            result.update({
                "spatially_associated_gt_id": truth_id,
                "gt_x_m": round(float(truth["relative"][0]), 4),
                "gt_y_m": round(float(truth["relative"][1]), 4),
                "gt_z_m": round(float(truth["relative"][2]), 4),
                "spatial_association_error_m": round(distance, 4),
            })
            predicted_category = str(row.get("class", "")).strip()
            truth_category = str(truth.get("category", "")).strip()
            if predicted_category and predicted_category != "unknown_product" and truth_category:
                result["category_agreement"] = predicted_category == truth_category
        output.append(result)

    errors.sort()
    if errors:
        middle = len(errors) // 2
        median = errors[middle] if len(errors) % 2 else (errors[middle - 1] + errors[middle]) / 2.0
        p95 = errors[min(len(errors) - 1, max(0, int(math.ceil(len(errors) * 0.95)) - 1))]
    else:
        median = p95 = None
    metrics: dict[str, object] = {
        "matching_threshold_m": threshold_m,
        "association_semantics": "one_to_one_min_cost_max_cardinality_spatial_association_not_identity_or_detection_match",
        "estimated_item_count": len(output),
        "invalid_estimate_coordinate_count": invalid_coordinate_count,
        "spatially_associated_estimate_count": len(associations),
        "ground_truth_item_count": len(truth_rows),
        "eligible_ground_truth_count": None if eligible_truth_ids is None else len(eligible_truth_ids),
        "eligible_ground_truth_association_count": None if eligible_truth_ids is None else eligible_associations,
        "eligible_recall": None if eligible_truth_ids is None or not eligible_truth_ids else eligible_associations / len(eligible_truth_ids),
        "eligible_precision": None,
        "eligibility_status": "unavailable_no_independent_visibility_labels" if eligible_truth_ids is None else "provided_by_independent_evaluation_input",
        "median_spatial_association_error_m": median,
        "p95_spatial_association_error_m": p95,
        "category_agreement_count": sum(row["category_agreement"] is True for row in output),
        "category_agreement_evaluated_count": sum(row["category_agreement"] is not None for row in output),
    }
    return output, metrics


def _write_csv(path: Path, rows: Iterable[dict[str, object]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _column_name(index: int) -> str:
    name = ""
    while index:
        index, remainder = divmod(index - 1, 26)
        name = chr(65 + remainder) + name
    return name


def _cell(value: object, reference: str) -> str:
    if value is None or value == "":
        return f'<c r="{reference}"/>'
    if isinstance(value, bool):
        return f'<c r="{reference}" t="b"><v>{1 if value else 0}</v></c>'
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f'<c r="{reference}"><v>{value}</v></c>'
    return f'<c r="{reference}" t="inlineStr"><is><t>{html.escape(str(value))}</t></is></c>'


def _write_xlsx(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    """Write a dependency-free XLSX that opens in Excel/LibreOffice."""
    data = [fields] + [[row.get(field, "") for field in fields] for row in rows]
    sheet_rows: list[str] = []
    for row_number, values in enumerate(data, start=1):
        cells = "".join(_cell(value, f"{_column_name(column_number)}{row_number}") for column_number, value in enumerate(values, start=1))
        sheet_rows.append(f'<row r="{row_number}">{cells}</row>')
    worksheet = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetViews><sheetView workbookViewId="0"/></sheetViews><sheetData>' + "".join(sheet_rows) + '</sheetData></worksheet>'
    workbook = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets><sheet name="Inventory" sheetId="1" r:id="rId1"/></sheets></workbook>'
    relationships = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>'
    workbook_relationships = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/></Relationships>'
    content_types = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/><Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/></Types>'
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", relationships)
        archive.writestr("xl/workbook.xml", workbook)
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_relationships)
        archive.writestr("xl/worksheets/sheet1.xml", worksheet)


def _write_inventory_map(path: Path, slam_map_bytes: bytes, rows: list[dict[str, object]]) -> int:
    lines = slam_map_bytes.decode("utf-8").splitlines()
    end_header = next(index for index, line in enumerate(lines) if line == "end_header")
    header = lines[:end_header + 1]
    original_count = next(int(line.split()[2]) for line in header if line.startswith("element vertex "))
    centers = [row for row in rows if row.get("map_x_m") is not None and row.get("map_y_m") is not None and row.get("map_z_m") is not None]
    header = [line.replace(f"element vertex {original_count}", f"element vertex {original_count + len(centers)}") for line in header]
    header = [line for line in header if not line.startswith("property uchar")]
    insert_at = next(index for index, line in enumerate(header) if line == "end_header")
    header[insert_at:insert_at] = ["property uchar red", "property uchar green", "property uchar blue"]
    output_lines = header
    output_lines.extend(f"{line} 150 150 150" for line in lines[end_header + 1:] if line.strip())
    output_lines.extend(f"{row['map_x_m']} {row['map_y_m']} {row['map_z_m']} 235 45 45" for row in centers)
    path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")
    return len(centers)


def _slam_start_timestamp(slam_dir: Path, pose_csv_bytes: bytes | None = None) -> float:
    """Use the estimator's pose ordering, then reject an invalid selected start."""
    poses: list[PoseSample] = []
    pose_rows = (
        _load_csv_bytes("slam_poses.csv", pose_csv_bytes)[1]
        if pose_csv_bytes is not None
        else _load_csv(slam_dir / "slam_poses.csv")
    )
    for row in pose_rows:
        orientation = safe_quaternion(tuple(float(row[key]) for key in ("qx", "qy", "qz", "qw")))
        if orientation is None:
            continue
        poses.append(PoseSample(
            float(row["timestamp_s"]),
            tuple(float(row[key]) for key in ("x_m", "y_m", "z_m")),
            orientation,
        ))
    poses.sort(key=lambda item: item.timestamp_s)
    if not poses:
        raise RuntimeError("SLAM pose stream has no valid orientations for evaluation alignment")
    selected = poses[0]
    if not math.isfinite(selected.timestamp_s) or not all(math.isfinite(value) for value in selected.position_m):
        raise RuntimeError("SLAM estimator's first accepted pose has a nonfinite timestamp or position")
    return selected.timestamp_s


def _validate_eligibility_manifest(
    manifest: dict[str, object],
    source: dict[str, object],
    manifest_sha256: str,
    source_sha256: str,
    manifest_path: Path,
    capture_id: str,
    truth_ids: set[str],
    evaluation_started_at: datetime,
) -> tuple[set[str], dict[str, object]]:
    """Validate frozen, exhaustive eligibility labels and preserve provenance."""
    if manifest.get("schema_version") != 1 or source.get("schema_version") != 1:
        raise ValueError("eligibility manifest and label source must use schema_version 1")
    if manifest.get("capture_id") != capture_id or source.get("capture_id") != capture_id:
        raise ValueError("eligibility manifest/source capture_id does not match the evaluated capture")
    method = manifest.get("method")
    rule = manifest.get("rule")
    if not isinstance(method, str) or not method.strip() or not isinstance(rule, str) or not rule.strip():
        raise ValueError("eligibility manifest must declare non-empty method and rule")
    frozen_value = manifest.get("frozen_at_utc")
    if not isinstance(frozen_value, str):
        raise ValueError("eligibility manifest must declare frozen_at_utc")
    try:
        frozen_at = datetime.fromisoformat(frozen_value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("eligibility frozen_at_utc must be an ISO-8601 timestamp") from exc
    if frozen_at.tzinfo is None:
        raise ValueError("eligibility frozen_at_utc must include a timezone")
    if frozen_at.astimezone(timezone.utc) > evaluation_started_at.astimezone(timezone.utc):
        raise ValueError("eligibility labels must be frozen before evaluation starts")

    source_ref = manifest.get("source")
    if not isinstance(source_ref, dict) or not isinstance(source_ref.get("path"), str):
        raise ValueError("eligibility manifest must identify its source file")
    declared_source_hash = source_ref.get("sha256")
    if not isinstance(declared_source_hash, str) or declared_source_hash.lower() != source_sha256.lower():
        raise ValueError("eligibility source hash does not match the file bytes")
    labels = source.get("labels")
    if not isinstance(labels, dict) or any(
        not isinstance(value, str) or value not in {"eligible", "ineligible"}
        for value in labels.values()
    ):
        raise ValueError("eligibility source labels must map each semantic ID to eligible or ineligible")
    if set(labels) != truth_ids:
        raise ValueError("eligibility source must label every and only ground-truth semantic ID")
    eligible = {str(key) for key, value in labels.items() if value == "eligible"}
    ineligible = {str(key) for key, value in labels.items() if value == "ineligible"}
    manifest_eligible = manifest.get("eligible_truth_ids")
    manifest_ineligible = manifest.get("ineligible_truth_ids")
    if not isinstance(manifest_eligible, list) or not isinstance(manifest_ineligible, list):
        raise ValueError("eligibility manifest must include eligible_truth_ids and ineligible_truth_ids lists")
    if any(not isinstance(item, str) for item in manifest_eligible + manifest_ineligible):
        raise ValueError("eligibility manifest IDs must be strings")
    if len(manifest_eligible) != len(set(manifest_eligible)) or len(manifest_ineligible) != len(set(manifest_ineligible)):
        raise ValueError("eligibility manifest ID lists must not contain duplicates")
    if set(manifest_eligible) != eligible or set(manifest_ineligible) != ineligible:
        raise ValueError("eligibility manifest ID lists do not match the hash-bound source labels")
    if eligible & ineligible or eligible | ineligible != truth_ids:
        raise ValueError("eligibility IDs must be a disjoint exhaustive partition of ground truth")

    evidence = {
        "schema_version": 1,
        "capture_id": capture_id,
        "manifest_path": str(manifest_path),
        "manifest_sha256": manifest_sha256,
        "source_path": str(source_ref["path"]),
        "source_sha256": source_sha256,
        "method": method.strip(),
        "rule": rule.strip(),
        "frozen_at_utc": frozen_at.astimezone(timezone.utc).isoformat(),
        "eligible_truth_ids": sorted(eligible),
        "ineligible_truth_ids": sorted(ineligible),
    }
    return eligible, evidence


def _load_eligibility_manifest(
    path: Path,
    capture_id: str,
    truth_ids: set[str],
    evaluation_started_at: datetime,
) -> tuple[set[str], dict[str, object]]:
    manifest_bytes = path.read_bytes()
    manifest = json.loads(manifest_bytes)
    if not isinstance(manifest, dict):
        raise ValueError("eligibility manifest root must be a JSON object")
    source_ref = manifest.get("source")
    if not isinstance(source_ref, dict) or not isinstance(source_ref.get("path"), str):
        raise ValueError("eligibility manifest must identify its source file")
    source_relative = Path(str(source_ref["path"]))
    if source_relative.is_absolute() or ".." in source_relative.parts:
        raise ValueError("eligibility source path must be relative to the manifest and stay within its directory")
    manifest_directory = path.parent.resolve()
    source_path = (manifest_directory / source_relative).resolve()
    if not source_path.is_relative_to(manifest_directory):
        raise ValueError("eligibility source must remain within the manifest directory")
    source_bytes = source_path.read_bytes()
    source = json.loads(source_bytes)
    if not isinstance(source, dict):
        raise ValueError("eligibility source root must be a JSON object")
    return _validate_eligibility_manifest(
        manifest, source, hashlib.sha256(manifest_bytes).hexdigest(),
        hashlib.sha256(source_bytes).hexdigest(), path, capture_id, truth_ids,
        evaluation_started_at,
    )


def evaluate_inventory(
    capture_dir: str | Path,
    slam_dir: str | Path,
    perception_dir: str | Path,
    repo_root: str | Path | None = None,
    eligibility_manifest_path: str | Path | None = None,
) -> dict[str, object]:
    evaluation_started_at = datetime.now(timezone.utc)
    capture = Path(capture_dir).resolve()
    slam = Path(slam_dir).resolve()
    perception = Path(perception_dir).resolve()
    input_provenance, validated_cloud_bytes, validated_pose_bytes = _validate_map_provenance(slam, perception)
    estimate_fields, estimates = _load_csv_with_fields(perception / "estimated_inventory.csv")
    _validate_estimate_map_versions(estimate_fields, estimates, str(input_provenance["map_version"]))
    start_timestamp_s = _slam_start_timestamp(slam, validated_pose_bytes)
    truth = _gt_start_relative(capture, start_timestamp_s)
    eligibility_evidence = None
    if eligibility_manifest_path is None:
        rows, metrics = _match_rows(estimates, truth)
    else:
        eligible_ids, eligibility_evidence = _load_eligibility_manifest(
            Path(eligibility_manifest_path).resolve(), capture.parent.name,
            {str(item["semantic_id"]) for item in truth}, evaluation_started_at,
        )
        rows, metrics = _match_rows(estimates, truth, eligible_truth_ids=eligible_ids)
    _write_csv(perception / "inventory.csv", rows, INVENTORY_FIELDS)
    _write_xlsx(perception / "inventory.xlsx", rows, INVENTORY_FIELDS)
    map_center_count = _write_inventory_map(slam / "slam_map_with_inventory.ply", validated_cloud_bytes, rows)
    result = {
        "status": "complete",
        "ground_truth_consumed": True,
        "estimator_completed_before_ground_truth": True,
        "capture_id": capture.parent.name,
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "evaluation_start_timestamp_s": start_timestamp_s,
        "evaluation_started_at_utc": evaluation_started_at.isoformat(),
        "map_version": input_provenance["map_version"],
        "input_provenance": input_provenance,
        "eligibility_evidence": eligibility_evidence,
        "metrics": metrics,
        "artifacts": {
            "inventory_csv": "inventory.csv",
            "inventory_xlsx": "inventory.xlsx",
            "slam_map_with_inventory": str((slam / "slam_map_with_inventory.ply").relative_to(perception.parent)).replace("\\", "/"),
        },
        "map_inventory_center_count": map_center_count,
        "limitations": [
            "The standard offline runner supplies no independent visibility labels; eligible recall remains null unless a validated frozen eligibility manifest is provided.",
            "Spatial associations are proximity-based candidates and do not establish persistent identity or calibrated detection precision.",
        ],
    }
    (perception / "inventory_evaluation.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--slam-dir", required=True)
    parser.add_argument("--perception-dir", required=True)
    parser.add_argument("--repo-root", default="")
    parser.add_argument("--eligibility-manifest", default="", help="Optional versioned, hash-bound visibility eligibility manifest")
    args = parser.parse_args()
    print(json.dumps(evaluate_inventory(
        args.capture_dir, args.slam_dir, args.perception_dir, args.repo_root or None,
        args.eligibility_manifest or None,
    ), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
