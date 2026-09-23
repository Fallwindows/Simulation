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
import json
import math
import subprocess
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np

from evaluation.metrics import PoseSample, interpolate_pose, safe_quaternion


INVENTORY_FIELDS = [
    "estimated_id", "estimated_sku", "estimated_category", "relative_x_m", "relative_y_m", "relative_z_m",
    "map_x_m", "map_y_m", "map_z_m", "observations", "confidence", "spatially_associated_gt_id",
    "gt_x_m", "gt_y_m", "gt_z_m", "spatial_association_error_m", "category_agreement",
]


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
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


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


def _write_inventory_map(path: Path, slam_map: Path, rows: list[dict[str, object]]) -> int:
    lines = slam_map.read_text(encoding="utf-8").splitlines()
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


def _slam_start_timestamp(slam_dir: Path) -> float:
    """Use the estimator's pose ordering, then reject an invalid selected start."""
    poses: list[PoseSample] = []
    for row in _load_csv(slam_dir / "slam_poses.csv"):
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
    estimates = _load_csv(perception / "estimated_inventory.csv")
    start_timestamp_s = _slam_start_timestamp(slam)
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
    map_center_count = _write_inventory_map(slam / "slam_map_with_inventory.ply", slam / "slam_map.ply", rows)
    result = {
        "status": "complete",
        "ground_truth_consumed": True,
        "estimator_completed_before_ground_truth": True,
        "capture_id": capture.parent.name,
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "evaluation_start_timestamp_s": start_timestamp_s,
        "evaluation_started_at_utc": evaluation_started_at.isoformat(),
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
