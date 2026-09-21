"""Validate and normalize native RTAB-Map database exports.

This module deliberately uses only the Python standard library.  The native
``rtabmap-export`` process reads the ROS data and this process turns its ASCII
outputs into the stable artifacts consumed by the rest of the project.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import TextIO


POSE_FIELDS = (
    "timestamp_s",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
    "quaternion_valid",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: str, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _convert_poses(source: Path, destination: Path) -> tuple[int, float, float]:
    rows: list[list[float]] = []
    with source.open(encoding="ascii") as handle:
        for line_number, line in enumerate(handle, 1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            if len(parts) != 8:
                raise ValueError(f"pose line {line_number} must contain 8 values")
            values = [_finite(value, f"pose line {line_number}") for value in parts]
            quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
            if abs(quaternion_norm - 1.0) > 1e-3:
                raise ValueError(f"pose line {line_number} quaternion is not normalized")
            if rows and values[0] <= rows[-1][0]:
                raise ValueError("pose timestamps must be strictly increasing")
            rows.append(values)
    if len(rows) < 2:
        raise ValueError("native export must contain at least two poses")

    with destination.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(POSE_FIELDS)
        for values in rows:
            writer.writerow([*(f"{value:.9g}" for value in values), "1"])
    return len(rows), rows[0][0], rows[-1][0]


def _read_ply_header(handle: TextIO) -> tuple[int, list[str]]:
    if handle.readline().strip() != "ply":
        raise ValueError("native cloud is not a PLY file")
    if handle.readline().strip() != "format ascii 1.0":
        raise ValueError("native cloud must be ASCII PLY 1.0")
    vertex_count: int | None = None
    vertex_properties: list[str] = []
    current_element = ""
    for line_number in range(3, 10_001):
        line = handle.readline()
        if not line:
            raise ValueError("native cloud PLY header is incomplete")
        text = line.strip()
        if text == "end_header":
            break
        parts = text.split()
        if len(parts) == 3 and parts[0] == "element":
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts and parts[0] == "property" and current_element == "vertex":
            if len(parts) != 3 or parts[1] == "list":
                raise ValueError("native cloud has an unsupported vertex property")
            vertex_properties.append(parts[2])
    else:
        raise ValueError("native cloud PLY header is too long")
    if vertex_count is None or vertex_count <= 0:
        raise ValueError("native cloud has no vertices")
    if not {"x", "y", "z"}.issubset(vertex_properties):
        raise ValueError("native cloud does not contain x/y/z properties")
    return vertex_count, vertex_properties


def _convert_cloud(source: Path, ply_destination: Path, pcd_destination: Path) -> int:
    with source.open(encoding="ascii", errors="strict") as input_handle:
        vertex_count, properties = _read_ply_header(input_handle)
        indices = [properties.index(axis) for axis in ("x", "y", "z")]
        with ply_destination.open("w", encoding="ascii", newline="\n") as ply_handle, pcd_destination.open(
            "w", encoding="ascii", newline="\n"
        ) as pcd_handle:
            ply_handle.write(
                "ply\nformat ascii 1.0\ncomment normalized from native RTAB-Map export\n"
                f"element vertex {vertex_count}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
            )
            pcd_handle.write(
                "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {vertex_count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {vertex_count}\nDATA ascii\n"
            )
            for vertex_index in range(vertex_count):
                line = input_handle.readline()
                if not line:
                    raise ValueError(f"native cloud ended after {vertex_index} vertices")
                parts = line.split()
                if len(parts) < len(properties):
                    raise ValueError(f"native cloud vertex {vertex_index} is incomplete")
                xyz = [_finite(parts[index], f"cloud vertex {vertex_index}") for index in indices]
                normalized = " ".join(f"{value:.9g}" for value in xyz) + "\n"
                ply_handle.write(normalized)
                pcd_handle.write(normalized)
    return vertex_count


def finalize(
    poses_path: Path,
    cloud_path: Path,
    database_path: Path,
    output_dir: Path,
    exporter_version_path: Path,
    optimize_command: list[str],
    export_command: list[str],
) -> dict[str, object]:
    for label, path in (
        ("pose export", poses_path),
        ("cloud export", cloud_path),
        ("RTAB-Map database", database_path),
        ("exporter version log", exporter_version_path),
    ):
        if not path.is_file() or path.stat().st_size <= 0:
            raise ValueError(f"{label} is missing or empty: {path}")
    expected_optimize = [
        "rtabmap-export.exe", "--poses", "--poses_format", "10", "--opt", "0", "--save_in_db",
        "--output", "slam_optimized", "--output_dir", str(cloud_path.parent), str(database_path),
    ]
    expected_export = [
        "rtabmap-export.exe", "--cloud", "--scan", "--poses", "--poses_format", "10", "--ascii",
        "--opt", "2", "--max_range", "100", "--voxel", "0.03", "--output", "slam_map",
        "--output_dir", str(cloud_path.parent), str(database_path),
    ]
    if optimize_command != expected_optimize:
        raise ValueError("optimization command provenance is incomplete")
    if export_command != expected_export:
        raise ValueError("cloud export command provenance is incomplete")

    version_output = exporter_version_path.read_text(encoding="utf-8", errors="strict")
    version_line = next((line.strip() for line in version_output.splitlines() if line.startswith("RTAB-Map:")), "")
    if not version_line:
        raise ValueError("RTAB-Map exporter version is missing")

    output_dir.mkdir(parents=True, exist_ok=True)
    pose_count, first_stamp, last_stamp = _convert_poses(poses_path, output_dir / "slam_poses.csv")
    point_count = _convert_cloud(cloud_path, output_dir / "slam_map.ply", output_dir / "slam_map.pcd")
    artifacts = {
        name: {"path": name, "sha256": _sha256(output_dir / name)}
        for name in ("slam_poses.csv", "slam_map.ply", "slam_map.pcd")
    }
    result: dict[str, object] = {
        "status": "complete",
        "producer_mode": "rtabmap_database_export",
        "ground_truth_subscribed": False,
        "ground_truth_consumed": False,
        "pose_count": pose_count,
        "map_point_count": point_count,
        "first_pose_timestamp_s": first_stamp,
        "last_pose_timestamp_s": last_stamp,
        "database": {"path": database_path.name, "sha256": _sha256(database_path)},
        "artifacts": artifacts,
        "optimization": {
            "method": "global graph optimization persisted in RTAB-Map database",
            "command": optimize_command,
        },
        "map_export": {
            "method": "saved optimized poses with assembled LiDAR scan clouds",
            "command": export_command,
        },
        "tool": {"name": "rtabmap-export", "version": version_line.split(":", 1)[1].strip()},
    }
    (output_dir / "slam_producer.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--cloud", required=True, type=Path)
    parser.add_argument("--database", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--exporter-version-log", required=True, type=Path)
    parser.add_argument("--commands-json", required=True, type=Path)
    args = parser.parse_args()
    commands = json.loads(args.commands_json.read_text(encoding="utf-8"))
    result = finalize(
        args.poses,
        args.cloud,
        args.database,
        args.output_dir,
        args.exporter_version_log,
        commands["optimize"],
        commands["export"],
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
