"""Post-estimation inventory matching and artifact export.

This module is intentionally separate from :mod:`rgb_tracking`: it is the
first stage allowed to open ``inventory_ground_truth.csv``.  The estimator can
therefore be run on a capture with the evaluation-only files removed.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import subprocess
import zipfile
from pathlib import Path
from typing import Iterable

import numpy as np

from evaluation.metrics import safe_quaternion
from simulator.perception.rgb_tracking import _quat_to_matrix


INVENTORY_FIELDS = [
    "estimated_id", "estimated_sku", "estimated_category", "relative_x_m", "relative_y_m", "relative_z_m",
    "map_x_m", "map_y_m", "map_z_m", "observations", "confidence", "matched_gt_id",
    "gt_x_m", "gt_y_m", "gt_z_m", "center_error_m", "correct_sku_category",
]


def _git_sha(repo_root: Path) -> str | None:
    try:
        return subprocess.run(["git", "-C", str(repo_root), "rev-parse", "HEAD"], check=True, capture_output=True, text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _read_truth_start_pose(capture_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read the first truth TF pose; this is evaluation-only and never estimator input."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str((capture_dir / "sensors_bag").resolve()).replace("\\", "/"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    selected: tuple[float, np.ndarray, np.ndarray] | None = None
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
            if q is None:
                continue
            position = np.asarray([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ], dtype=np.float64)
            rotation = _quat_to_matrix(q)
            candidate = (stamp, position, rotation)
            if selected is None or stamp < selected[0]:
                selected = candidate
    if selected is None:
        raise RuntimeError("No valid truth_sensor_rig TF was found for evaluation alignment")
    return selected[1], selected[2]


def _load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _gt_start_relative(capture_dir: Path) -> list[dict[str, object]]:
    start_t, start_r = _read_truth_start_pose(capture_dir)
    rows: list[dict[str, object]] = []
    for row in _load_csv(capture_dir / "inventory_ground_truth.csv"):
        world = np.asarray([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])], dtype=np.float64)
        relative = (world - start_t) @ start_r
        rows.append({
            "semantic_id": row["semantic_id"],
            "asset_key": row["asset_key"],
            "category": row["category"],
            "relative": relative,
            "world": world,
        })
    return rows


def _float_or_none(value: object) -> float | None:
    if value is None or str(value).strip() in {"", "None", "nan"}:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _match_rows(estimate_rows: list[dict[str, str]], truth_rows: list[dict[str, object]], threshold_m: float = 0.35) -> tuple[list[dict[str, object]], dict[str, object]]:
    candidates: list[tuple[float, int, int]] = []
    for estimate_index, row in enumerate(estimate_rows):
        estimate = np.asarray([_float_or_none(row.get("estimated_x_m")) or 0.0, _float_or_none(row.get("estimated_y_m")) or 0.0, _float_or_none(row.get("estimated_z_m")) or 0.0], dtype=np.float64)
        for truth_index, truth in enumerate(truth_rows):
            distance = float(np.linalg.norm(estimate - truth["relative"]))
            if distance <= threshold_m:
                candidates.append((distance, estimate_index, truth_index))
    candidates.sort()
    used_estimates: set[int] = set()
    used_truth: set[int] = set()
    matches: dict[int, tuple[int, float]] = {}
    for distance, estimate_index, truth_index in candidates:
        if estimate_index in used_estimates or truth_index in used_truth:
            continue
        used_estimates.add(estimate_index)
        used_truth.add(truth_index)
        matches[estimate_index] = (truth_index, distance)

    output: list[dict[str, object]] = []
    for estimate_index, row in enumerate(estimate_rows):
        match = matches.get(estimate_index)
        result: dict[str, object] = {
            "estimated_id": int(row["track_id"]),
            "estimated_sku": "unknown_product",
            "estimated_category": "unknown_product",
            "relative_x_m": _float_or_none(row.get("estimated_x_m")),
            "relative_y_m": _float_or_none(row.get("estimated_y_m")),
            "relative_z_m": _float_or_none(row.get("estimated_z_m")),
            "map_x_m": _float_or_none(row.get("map_x_m")),
            "map_y_m": _float_or_none(row.get("map_y_m")),
            "map_z_m": _float_or_none(row.get("map_z_m")),
            "observations": int(row.get("3d_observation_count", 0)),
            "confidence": round(min(1.0, float(row.get("detection_count", 0)) / 24.0), 4),
            "matched_gt_id": "",
            "gt_x_m": "", "gt_y_m": "", "gt_z_m": "", "center_error_m": "",
            "correct_sku_category": False,
        }
        if match is not None:
            truth_index, distance = match
            truth = truth_rows[truth_index]
            result.update({
                "matched_gt_id": truth["semantic_id"],
                "gt_x_m": round(float(truth["relative"][0]), 4),
                "gt_y_m": round(float(truth["relative"][1]), 4),
                "gt_z_m": round(float(truth["relative"][2]), 4),
                "center_error_m": round(distance, 4),
                "correct_sku_category": str(row.get("class", "unknown_product")) == str(truth["category"]),
            })
        output.append(result)

    errors = [float(row["center_error_m"]) for row in output if row["center_error_m"] != ""]
    errors.sort()
    median = errors[len(errors) // 2] if errors else None
    p95 = errors[min(len(errors) - 1, max(0, int(math.ceil(len(errors) * 0.95)) - 1))] if errors else None
    matched = len(errors)
    metrics = {
        "matching_threshold_m": threshold_m,
        "estimated_persistent_item_count": len(output),
        "matched_visible_gt_count": matched,
        "ground_truth_item_count": len(truth_rows),
        "precision": matched / len(output) if output else 0.0,
        "visible_recall": matched / len(truth_rows) if truth_rows else 0.0,
        "median_center_error_m": median,
        "p95_center_error_m": p95,
        "correct_sku_category_count": sum(1 for row in output if row["correct_sku_category"]),
        "visibility_definition": "GT items with one-to-one localized estimate within matching_threshold_m",
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


def evaluate_inventory(capture_dir: str | Path, slam_dir: str | Path, perception_dir: str | Path, repo_root: str | Path | None = None) -> dict[str, object]:
    capture = Path(capture_dir).resolve()
    slam = Path(slam_dir).resolve()
    perception = Path(perception_dir).resolve()
    estimates = _load_csv(perception / "estimated_inventory.csv")
    truth = _gt_start_relative(capture)
    rows, metrics = _match_rows(estimates, truth)
    _write_csv(perception / "inventory.csv", rows, INVENTORY_FIELDS)
    _write_xlsx(perception / "inventory.xlsx", rows, INVENTORY_FIELDS)
    map_center_count = _write_inventory_map(slam / "slam_map_with_inventory.ply", slam / "slam_map.ply", rows)
    result = {
        "status": "complete",
        "ground_truth_consumed": True,
        "estimator_completed_before_ground_truth": True,
        "capture_id": capture.parent.name,
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "metrics": metrics,
        "artifacts": {
            "inventory_csv": "inventory.csv",
            "inventory_xlsx": "inventory.xlsx",
            "slam_map_with_inventory": str((slam / "slam_map_with_inventory.ply").relative_to(perception.parent)).replace("\\", "/"),
        },
        "map_inventory_center_count": map_center_count,
    }
    (perception / "inventory_evaluation.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--slam-dir", required=True)
    parser.add_argument("--perception-dir", required=True)
    parser.add_argument("--repo-root", default="")
    args = parser.parse_args()
    print(json.dumps(evaluate_inventory(args.capture_dir, args.slam_dir, args.perception_dir, args.repo_root or None), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
