"""A deterministic, capture-only RGB product proposal and tracker.

This is deliberately an estimator, not a ground-truth renderer.  It proposes
visible product-like regions from the captured RGB stream using colour and
connected-component evidence, then maintains IDs with frame-to-frame motion
and appearance association.  Ground-truth inventory is never opened here.

The RGB proposal/tracking stage is followed by a sensor-only association step:
raw LiDAR returns are projected through interpolated SLAM poses and camera
calibration to obtain start-relative 3D estimates.  Ground-truth inventory is
never opened here.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import json
import math
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from evaluation.metrics import PoseSample, interpolate_pose, safe_quaternion


@dataclass(frozen=True)
class Detection:
    bbox_xyxy: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: int
    mean_hue: float
    mean_saturation: float
    confidence: float


@dataclass
class Track:
    track_id: int
    first_frame_index: int
    last_frame_index: int
    detection_count: int = 0
    missed_frames: int = 0
    center_px: tuple[float, float] = (0.0, 0.0)
    velocity_px: tuple[float, float] = (0.0, 0.0)
    area_px: float = 0.0
    mean_hue: float = 0.0
    mean_saturation: float = 0.0
    detections: list[dict[str, object]] = field(default_factory=list)

    def update(self, frame_index: int, detection: Detection) -> None:
        previous = self.center_px
        alpha = 0.65
        self.velocity_px = (
            0.7 * self.velocity_px[0] + 0.3 * (detection.center_px[0] - previous[0]),
            0.7 * self.velocity_px[1] + 0.3 * (detection.center_px[1] - previous[1]),
        )
        self.center_px = (
            alpha * detection.center_px[0] + (1.0 - alpha) * previous[0],
            alpha * detection.center_px[1] + (1.0 - alpha) * previous[1],
        )
        self.area_px = alpha * detection.area_px + (1.0 - alpha) * self.area_px
        self.mean_hue = alpha * detection.mean_hue + (1.0 - alpha) * self.mean_hue
        self.mean_saturation = alpha * detection.mean_saturation + (1.0 - alpha) * self.mean_saturation
        self.last_frame_index = frame_index
        self.detection_count += 1
        self.missed_frames = 0
        self.detections.append(_detection_record(self.track_id, detection))


def _detection_record(track_id: int, detection: Detection) -> dict[str, object]:
    x0, y0, x1, y1 = detection.bbox_xyxy
    return {
        "track_id": track_id,
        "bbox_xyxy": [x0, y0, x1, y1],
        "center_px": [round(detection.center_px[0], 3), round(detection.center_px[1], 3)],
        "area_px": detection.area_px,
        "confidence": round(detection.confidence, 4),
        "class": "unknown_product",
        "source": "rgb_color_connected_component",
    }


def detect_product_blobs(frame_bgr: np.ndarray, max_detections: int = 160) -> list[Detection]:
    """Detect visible saturated product faces without scene metadata.

    Grey shelving, floor, and walls are suppressed by the saturation gate.
    Component shape/area limits prevent aisle-sized regions from becoming
    false products.  The returned center is the image centroid of the actual
    connected component, not a hand-authored or ground-truth location.
    """

    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError("frame_bgr must be an HxWx3 image")
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    mask = ((saturation >= 52) & (value >= 42)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    detections: list[Detection] = []
    height, width = mask.shape
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area < 70 or area > 9000 or w < 5 or h < 5:
            continue
        if w > width * 0.35 or h > height * 0.45:
            continue
        aspect = w / max(1.0, float(h))
        if aspect < 0.08 or aspect > 7.0:
            continue
        fill = area / max(1.0, float(w * h))
        if fill < 0.16:
            continue
        cx, cy = (float(value) for value in centroids[label])
        component_hsv = hsv[labels == label]
        mean_hue = float(np.mean(component_hsv[:, 0]))
        mean_sat = float(np.mean(component_hsv[:, 1]))
        # Confidence is evidence quality, not a learned probability.
        confidence = min(1.0, 0.35 + 0.35 * min(1.0, fill) + 0.30 * min(1.0, area / 1500.0))
        detections.append(Detection((x, y, x + w - 1, y + h - 1), (cx, cy), area, mean_hue, mean_sat, confidence))
    detections.sort(key=lambda item: (item.center_px[1], item.center_px[0]))
    return detections[:max_detections]


class BlobTracker:
    def __init__(self, max_match_distance_px: float = 85.0, max_missed_frames: int = 8):
        self.max_match_distance_px = float(max_match_distance_px)
        self.max_missed_frames = int(max_missed_frames)
        self.next_track_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, frame_index: int, detections: Iterable[Detection]) -> list[dict[str, object]]:
        detections = list(detections)
        candidates: list[tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            if track.missed_frames > self.max_missed_frames:
                continue
            predicted = (track.center_px[0] + track.velocity_px[0], track.center_px[1] + track.velocity_px[1])
            for detection_index, detection in enumerate(detections):
                distance = math.hypot(predicted[0] - detection.center_px[0], predicted[1] - detection.center_px[1])
                area_ratio = max(detection.area_px, 1) / max(track.area_px, 1.0)
                hue_distance = abs(detection.mean_hue - track.mean_hue)
                hue_distance = min(hue_distance, 180.0 - hue_distance)
                cost = distance + 10.0 * abs(math.log(area_ratio)) + 0.20 * hue_distance
                if distance <= self.max_match_distance_px and area_ratio < 6.0 and area_ratio > (1.0 / 6.0):
                    candidates.append((cost, track_id, detection_index))
        candidates.sort()
        assigned_tracks: set[int] = set()
        assigned_detections: set[int] = set()
        for _, track_id, detection_index in candidates:
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            self.tracks[track_id].update(frame_index, detections[detection_index])
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)

        for track_id, track in self.tracks.items():
            if track_id not in assigned_tracks:
                track.missed_frames += 1
        for detection_index, detection in enumerate(detections):
            if detection_index in assigned_detections:
                continue
            track_id = self.next_track_id
            self.next_track_id += 1
            track = Track(track_id, frame_index, frame_index)
            track.update(frame_index, detection)
            self.tracks[track_id] = track

        return [
            record
            for track in sorted(self.tracks.values(), key=lambda item: item.track_id)
            for record in (track.detections[-1],)
            if track.last_frame_index == frame_index
        ]


def _read_timestamp_index(path: Path) -> list[dict[str, object]]:
    records = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                records.append(json.loads(line))
    records.sort(key=lambda item: int(item["frame_index"]))
    return records


def _git_sha(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(["git", "-C", str(repo_root), "rev-parse", "HEAD"], check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _quat_to_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = safe_quaternion(q) or ((0.0, 0.0, 0.0, 1.0))
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _load_slam_poses(path: Path) -> list[PoseSample]:
    poses: list[PoseSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            orientation = safe_quaternion(tuple(float(row[key]) for key in ("qx", "qy", "qz", "qw")))
            if orientation is None:
                continue
            poses.append(PoseSample(
                float(row["timestamp_s"]),
                (float(row["x_m"]), float(row["y_m"]), float(row["z_m"])),
                orientation,
            ))
    return sorted(poses, key=lambda item: item.timestamp_s)


def _load_sensor_geometry(path: Path) -> dict[str, object]:
    data = json.loads(path.read_text(encoding="utf-8"))
    transforms = {item["child"]: item for item in data["transforms"]}
    return {
        "rig_lidar_t": np.asarray(transforms["lidar_link"]["translation_m"], dtype=np.float64),
        "rig_lidar_r": _quat_to_matrix(tuple(transforms["lidar_link"]["rotation_xyzw"])),
        "rig_camera_t": np.asarray(transforms["camera_link"]["translation_m"], dtype=np.float64),
        "rig_camera_r": _quat_to_matrix(tuple(transforms["camera_link"]["rotation_xyzw"])),
        "link_optical_r": _quat_to_matrix(tuple(transforms["camera_optical_frame"]["rotation_xyzw"])),
        "fx": float(data["intrinsics"]["fx_px"]),
        "fy": float(data["intrinsics"]["fy_px"]),
        "cx": float(data["intrinsics"]["cx_px"]),
        "cy": float(data["intrinsics"]["cy_px"]),
    }


def _read_lidar_scans(bag_dir: Path):
    """Yield decimated raw PointCloud2 xyz arrays without using truth topics."""

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir).replace("\\", "/"), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/sim/lidar/points":
            continue
        message = deserialize_message(serialized, PointCloud2)
        fields = {field.name: field for field in message.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            continue
        point_step = int(message.point_step)
        raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
        if point_step <= 0 or raw.size < point_step:
            continue
        rows = raw[: (raw.size // point_step) * point_step].reshape((-1, point_step))
        try:
            points = np.stack([
                rows[:, int(fields[name].offset):int(fields[name].offset) + 4].copy().view("<f4").reshape(-1)
                for name in ("x", "y", "z")
            ], axis=1).astype(np.float64, copy=False)
        except (TypeError, ValueError):
            continue
        points = points[::8]
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if len(points):
            stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0
            yield stamp, points


def _nearest_frame_index(frames: list[dict[str, object]], timestamps: list[float], timestamp_s: float) -> int | None:
    index = bisect_left(timestamps, timestamp_s)
    candidates = [index]
    if index > 0:
        candidates.append(index - 1)
    candidates = [candidate for candidate in candidates if 0 <= candidate < len(frames)]
    if not candidates:
        return None
    selected = min(candidates, key=lambda candidate: abs(timestamps[candidate] - timestamp_s))
    return selected if abs(timestamps[selected] - timestamp_s) <= 0.08 else None


def _augment_with_lidar_estimates(
    capture: Path,
    slam: Path,
    frames: list[dict[str, object]],
    frame_annotations: list[dict[str, object]],
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray], dict[str, object]]:
    """Associate projected LiDAR returns with RGB proposals and make start-relative estimates."""

    poses = _load_slam_poses(slam / "slam_poses.csv")
    if not poses:
        return {}, {}, {"status": "unavailable", "reason": "no_valid_slam_poses"}
    geometry = _load_sensor_geometry(capture / "sensor_transforms.json")
    start_t = np.asarray(poses[0].position_m, dtype=np.float64)
    start_r = _quat_to_matrix(poses[0].orientation_xyzw)
    frame_timestamps = [float(frame["stamp_s"]) for frame in frames]
    annotations_by_frame = {int(record["frame_index"]): record for record in frame_annotations}
    observations_start: dict[int, list[np.ndarray]] = {}
    observations_map: dict[int, list[np.ndarray]] = {}
    scan_count = 0
    projected_point_count = 0

    for stamp_s, lidar_points in _read_lidar_scans(capture / "sensors_bag"):
        pose = interpolate_pose(poses, stamp_s, max_gap_s=0.5)
        if pose is None:
            continue
        frame_index = _nearest_frame_index(frames, frame_timestamps, stamp_s)
        if frame_index is None:
            continue
        frame_record = annotations_by_frame.get(int(frames[frame_index]["frame_index"]))
        if frame_record is None or not frame_record["detections"]:
            continue
        rig_lidar_t = geometry["rig_lidar_t"]
        rig_lidar_r = geometry["rig_lidar_r"]
        rig_camera_t = geometry["rig_camera_t"]
        rig_camera_r = geometry["rig_camera_r"]
        link_optical_r = geometry["link_optical_r"]
        points_rig = lidar_points @ rig_lidar_r.T + rig_lidar_t
        points_link = (points_rig - rig_camera_t) @ rig_camera_r
        points_optical = points_link @ link_optical_r
        positive = (points_optical[:, 2] > 0.25) & (points_optical[:, 2] < 45.0)
        points_rig = points_rig[positive]
        points_optical = points_optical[positive]
        if not len(points_optical):
            continue
        u = geometry["fx"] * points_optical[:, 0] / points_optical[:, 2] + geometry["cx"]
        v = geometry["fy"] * points_optical[:, 1] / points_optical[:, 2] + geometry["cy"]
        in_image = (u >= 0.0) & (u < float(frames[frame_index]["width"])) & (v >= 0.0) & (v < float(frames[frame_index]["height"]))
        points_rig = points_rig[in_image]
        points_optical = points_optical[in_image]
        u = u[in_image]
        v = v[in_image]
        if not len(points_optical):
            continue
        projected_point_count += len(points_optical)
        world_r = _quat_to_matrix(pose.orientation_xyzw)
        pose_t = np.asarray(pose.position_m, dtype=np.float64)
        points_world = points_rig @ world_r.T + pose_t
        points_start = (points_world - start_t) @ start_r
        # Index detections into coarse image cells first.  This avoids creating
        # a full boolean cloud mask for every box on every LiDAR scan.
        cell_size = 32
        detection_grid: dict[tuple[int, int], list[int]] = {}
        detections = frame_record["detections"]
        for detection_index, detection in enumerate(detections):
            x0, y0, x1, y1 = [float(value) for value in detection["bbox_xyxy"]]
            gx0, gy0 = int(x0 // cell_size), int(y0 // cell_size)
            gx1, gy1 = int(x1 // cell_size), int(y1 // cell_size)
            for gx in range(gx0, gx1 + 1):
                for gy in range(gy0, gy1 + 1):
                    detection_grid.setdefault((gx, gy), []).append(detection_index)
        point_indices: list[list[int]] = [[] for _ in detections]
        for point_index, (point_u, point_v) in enumerate(zip(u, v)):
            for detection_index in detection_grid.get((int(point_u // cell_size), int(point_v // cell_size)), ()):
                x0, y0, x1, y1 = [float(value) for value in detections[detection_index]["bbox_xyxy"]]
                if x0 <= point_u <= x1 and y0 <= point_v <= y1:
                    point_indices[detection_index].append(point_index)
        for detection, indices in zip(detections, point_indices):
            if len(indices) < 3:
                continue
            index_array = np.asarray(indices, dtype=np.int64)
            depths = points_optical[index_array, 2]
            # Prefer the front surface of a proposed product over returns from
            # the shelf behind it, while retaining several returns for a stable
            # robust center estimate.
            front_cutoff = float(np.percentile(depths, 45.0))
            selected_indices = index_array[depths <= front_cutoff]
            if len(selected_indices) < 3:
                selected_indices = index_array
            estimate = np.median(points_start[selected_indices], axis=0)
            track_id = int(detection["track_id"])
            observations_start.setdefault(track_id, []).append(estimate)
            observations_map.setdefault(track_id, []).append(points_world[selected_indices].mean(axis=0))
            detection["depth_point_count"] = int(len(selected_indices))

        scan_count += 1

    track_estimates = {track_id: np.median(np.stack(values), axis=0) for track_id, values in observations_start.items() if values}
    map_estimates = {track_id: np.median(np.stack(values), axis=0) for track_id, values in observations_map.items() if values}
    for frame_record in frame_annotations:
        for detection in frame_record["detections"]:
            estimate = track_estimates.get(int(detection["track_id"]))
            if estimate is None:
                detection["coordinate_source"] = "rgb_only_no_lidar_association"
                continue
            rounded = [round(float(value), 3) for value in estimate]
            detection["estimated_center_start_relative_m"] = rounded
            detection["coordinate_source"] = "lidar_projected_with_slam_pose"
    for frame_record in frame_annotations:
        for detection in frame_record["detections"]:
            estimate = map_estimates.get(int(detection["track_id"]))
            if estimate is not None:
                detection["estimated_center_map_m"] = [round(float(value), 3) for value in estimate]
    return track_estimates, map_estimates, {
        "status": "complete",
        "start_pose_world_m": [round(float(value), 6) for value in start_t],
        "start_pose_orientation_xyzw": [round(float(value), 8) for value in poses[0].orientation_xyzw],
        "coordinate_frame": "start-relative sensor-rig frame; x forward, y left, z up",
        "scan_count_used": scan_count,
        "projected_point_count": projected_point_count,
        "track_count_with_3d_estimate": len(track_estimates),
    }


def _consolidate_track_estimates(
    track_estimates: dict[int, np.ndarray],
    merge_radius_m: float = 0.21,
) -> tuple[dict[int, int], dict[int, np.ndarray], dict[int, list[int]]]:
    """Merge RGB track fragments using estimated 3D position only.

    RGB components can split when a product crosses a shelf edge or changes
    apparent size.  The canonical ID is therefore assigned from the estimated
    start-relative center, never from simulator identity or ground truth.
    """

    clusters: list[dict[str, object]] = []
    grid: dict[tuple[int, int, int], list[int]] = {}
    for raw_track_id in sorted(track_estimates):
        point = np.asarray(track_estimates[raw_track_id], dtype=np.float64)
        cell = tuple(np.floor(point / merge_radius_m).astype(int))
        candidates: list[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    candidates.extend(grid.get((cell[0] + dx, cell[1] + dy, cell[2] + dz), []))
        selected = None
        selected_distance = float("inf")
        for cluster_index in candidates:
            center = np.asarray(clusters[cluster_index]["center"], dtype=np.float64)
            distance = float(np.linalg.norm(center - point))
            if distance <= merge_radius_m and distance < selected_distance:
                selected = cluster_index
                selected_distance = distance
        if selected is None:
            selected = len(clusters)
            clusters.append({"center": point.copy(), "count": 1, "members": [raw_track_id]})
            grid.setdefault(cell, []).append(selected)
        else:
            cluster = clusters[selected]
            count = int(cluster["count"]) + 1
            cluster["center"] = (np.asarray(cluster["center"]) * (count - 1) + point) / count
            cluster["count"] = count
            cluster["members"].append(raw_track_id)

    order = sorted(range(len(clusters)), key=lambda index: tuple(float(value) for value in clusters[index]["center"]))
    raw_to_canonical: dict[int, int] = {}
    canonical_estimates: dict[int, np.ndarray] = {}
    canonical_members: dict[int, list[int]] = {}
    for canonical_id, cluster_index in enumerate(order, start=1):
        cluster = clusters[cluster_index]
        canonical_estimates[canonical_id] = np.asarray(cluster["center"], dtype=np.float64)
        members = [int(value) for value in cluster["members"]]
        canonical_members[canonical_id] = members
        for raw_track_id in members:
            raw_to_canonical[raw_track_id] = canonical_id
    return raw_to_canonical, canonical_estimates, canonical_members


def run_rgb_tracking(capture_dir: str | Path, slam_dir: str | Path, output_dir: str | Path, repo_root: str | Path | None = None) -> dict[str, object]:
    """Run RGB tracking plus sensor-only 3D localization."""

    capture = Path(capture_dir).resolve()
    slam = Path(slam_dir).resolve()
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    frames_index = _read_timestamp_index(capture / "rgb_frames.jsonl")
    if not frames_index:
        raise ValueError("RGB timestamp index is empty")

    import av

    tracker = BlobTracker()
    frame_annotations: list[dict[str, object]] = []
    with av.open(str(capture / "rgb_camera.mp4")) as container:
        decoded = 0
        for frame in container.decode(video=0):
            if decoded >= len(frames_index):
                break
            image = frame.to_ndarray(format="bgr24")
            detections = detect_product_blobs(image)
            records = tracker.update(int(frames_index[decoded]["frame_index"]), detections)
            frame_annotations.append({
                "frame_index": int(frames_index[decoded]["frame_index"]),
                "stamp_s": float(frames_index[decoded]["stamp_s"]),
                "width": int(frames_index[decoded]["width"]),
                "height": int(frames_index[decoded]["height"]),
                "detections": records,
            })
            decoded += 1
    if decoded != len(frames_index):
        raise RuntimeError(f"RGB frame/index mismatch: decoded={decoded}, indexed={len(frames_index)}")

    track_estimates, map_estimates, localization = _augment_with_lidar_estimates(capture, slam, frames_index, frame_annotations)
    raw_to_canonical, canonical_estimates, canonical_members = _consolidate_track_estimates(track_estimates)
    canonical_map_estimates: dict[int, np.ndarray] = {}
    for canonical_id, raw_members in canonical_members.items():
        values = [map_estimates[raw_id] for raw_id in raw_members if raw_id in map_estimates]
        if values:
            canonical_map_estimates[canonical_id] = np.median(np.stack(values), axis=0)
    localization["canonical_track_count"] = len(canonical_estimates)
    localization["canonicalization_radius_m"] = 0.21

    observation_counts: dict[int, int] = {}
    for frame in frame_annotations:
        for detection in frame["detections"]:
            if detection.get("coordinate_source") != "lidar_projected_with_slam_pose":
                continue
            track_id = int(detection["track_id"])
            observation_counts[track_id] = observation_counts.get(track_id, 0) + 1

    # Replace fragment IDs in the output stream with stable 3D-associated IDs.
    for frame in frame_annotations:
        for detection in frame["detections"]:
            raw_track_id = int(detection["track_id"])
            canonical_id = raw_to_canonical.get(raw_track_id)
            if canonical_id is None:
                continue
            detection["raw_track_id"] = raw_track_id
            detection["track_id"] = canonical_id
            detection["estimated_center_start_relative_m"] = [
                round(float(value), 3) for value in canonical_estimates[canonical_id]
            ]
            if canonical_id in canonical_map_estimates:
                detection["estimated_center_map_m"] = [
                    round(float(value), 3) for value in canonical_map_estimates[canonical_id]
                ]
            detection["coordinate_source"] = "lidar_projected_with_slam_pose_consolidated"

    track_rows = []
    for canonical_id in sorted(canonical_estimates):
        raw_members = canonical_members[canonical_id]
        member_tracks = [tracker.tracks[raw_id] for raw_id in raw_members if raw_id in tracker.tracks]
        if not member_tracks:
            continue
        total_detections = sum(track.detection_count for track in member_tracks)
        weighted_u = sum(track.center_px[0] * track.detection_count for track in member_tracks) / max(1, total_detections)
        weighted_v = sum(track.center_px[1] * track.detection_count for track in member_tracks) / max(1, total_detections)
        track_rows.append({
            "track_id": canonical_id,
            "class": "unknown_product",
            "source": "rgb_lidar_slam_consolidated",
            "first_frame_index": min(track.first_frame_index for track in member_tracks),
            "last_frame_index": max(track.last_frame_index for track in member_tracks),
            "detection_count": total_detections,
            "center_u_px": round(weighted_u, 3),
            "center_v_px": round(weighted_v, 3),
            "estimated_x_m": round(float(canonical_estimates[canonical_id][0]), 3),
            "estimated_y_m": round(float(canonical_estimates[canonical_id][1]), 3),
            "estimated_z_m": round(float(canonical_estimates[canonical_id][2]), 3),
            "map_x_m": round(float(canonical_map_estimates[canonical_id][0]), 3) if canonical_id in canonical_map_estimates else None,
            "map_y_m": round(float(canonical_map_estimates[canonical_id][1]), 3) if canonical_id in canonical_map_estimates else None,
            "map_z_m": round(float(canonical_map_estimates[canonical_id][2]), 3) if canonical_id in canonical_map_estimates else None,
            "depth_source": "lidar_projected_with_slam_pose",
            "3d_observation_count": sum(observation_counts.get(raw_id, 0) for raw_id in raw_members),
            "raw_track_count": len(raw_members),
        })

    annotation_path = output / "frame_annotations.jsonl"
    with annotation_path.open("w", encoding="utf-8", newline="\n") as handle:
        for record in frame_annotations:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    for row in track_rows:
        row["3d_observation_count"] = int(row["3d_observation_count"])
    fields = ["track_id", "class", "source", "first_frame_index", "last_frame_index", "detection_count", "center_u_px", "center_v_px", "estimated_x_m", "estimated_y_m", "estimated_z_m", "map_x_m", "map_y_m", "map_z_m", "depth_source", "3d_observation_count", "raw_track_count"]
    _write_csv(output / "estimated_inventory.csv", track_rows, fields)
    _write_csv(output / "tracks.csv", track_rows, fields)
    (output / "estimated_inventory.json").write_text(json.dumps(track_rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    summary = {
        "status": "complete",
        "detector_status": "rgb_color_connected_component_baseline",
        "tracker_status": "nearest_prediction_with_appearance_gate",
        "capture_only": True,
        "ground_truth_consumed": False,
        "ground_truth_required": False,
        "slam_consumed_for_estimation": True,
        "lidar_consumed_for_estimation": True,
        "localization": localization,
        "slam_artifact": str((slam / "slam_map.pcd").relative_to(output.parent)).replace("\\", "/") if (slam / "slam_map.pcd").exists() else None,
        "frame_count": len(frame_annotations),
        "detection_count": sum(len(item["detections"]) for item in frame_annotations),
        "track_count": len(track_rows),
        "raw_rgb_track_count": len(tracker.tracks),
        "video": "../capture/rgb_camera.mp4",
        "frames": "../capture/rgb_frames.jsonl",
        "annotations": "frame_annotations.jsonl",
        "estimated_inventory": "estimated_inventory.csv",
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "notes": "Boxes originate from RGB components; start-relative 3D centers are robust medians of projected LiDAR returns using interpolated SLAM pose.",
    }
    (output / "perception_manifest.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--slam-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--repo-root", default="")
    args = parser.parse_args()
    result = run_rgb_tracking(args.capture_dir, args.slam_dir, args.output_dir, args.repo_root or None)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
