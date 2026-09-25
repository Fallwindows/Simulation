"""Selective, timestamped LiDAR sources for the technical-view renderer.

This module reads the recorded ``PointCloud2`` bag directly, keeps immutable
raw return indices through every selection, and supplies either camera-aligned
current returns or past-only map-frame history.  It intentionally has no scene,
asset, or simulator-truth input.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
import shutil
import sqlite3
import subprocess
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from simulator.sensors.feature_selection import select_bbox_front_surface
from simulator.sensors.scan_projection import (
    CameraIntrinsics,
    ProjectedScan,
    project_lidar_scan,
    quaternion_xyzw_matrix,
    read_pointcloud2_sqlite,
    resolve_transform,
)


LIDAR_TOPIC = "/sim/lidar/points"
LIDAR_TYPE = "sensor_msgs/msg/PointCloud2"


@dataclass(frozen=True)
class ScanRecord:
    message_id: int
    timestamp_ns: int

    @property
    def timestamp_s(self) -> float:
        return self.timestamp_ns / 1_000_000_000.0


@dataclass(frozen=True)
class RgbFrameRecord:
    frame_index: int
    timestamp_s: float
    width: int
    height: int
    frame_id: str


@dataclass(frozen=True)
class DetectionRegion:
    track_id: int
    raw_track_id: int | None
    bbox_xyxy: tuple[float, float, float, float]
    estimated_center_map_m: tuple[float, float, float] | None


@dataclass(frozen=True)
class PreparedScan:
    record: ScanRecord
    raw_xyz_m: np.ndarray
    optical_xyz_m: np.ndarray
    map_xyz_m: np.ndarray
    u_px: np.ndarray
    v_px: np.ndarray
    depth_m: np.ndarray
    raw_point_indices: np.ndarray
    selection_mode: str
    raw_point_count: int
    source_frame_id: str
    point_fields: tuple[str, ...]
    per_return_timing: str


@dataclass(frozen=True)
class RoiObservation:
    record: ScanRecord
    rgb_frame: RgbFrameRecord
    detection: DetectionRegion
    optical_xyz_m: np.ndarray
    map_xyz_m: np.ndarray
    u_px: np.ndarray
    v_px: np.ndarray
    depth_m: np.ndarray
    raw_point_indices: np.ndarray
    absolute_rgb_skew_ns: int


def _indices_sha256(indices: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(indices, dtype="<i8").tobytes(order="C")).hexdigest()


def _even_positions(count: int, limit: int) -> np.ndarray:
    if limit <= 0:
        raise ValueError("point limit must be positive")
    if count <= limit:
        return np.arange(count, dtype=np.int64)
    if limit == 1:
        return np.asarray([0], dtype=np.int64)
    return (np.arange(limit, dtype=np.int64) * (count - 1)) // (limit - 1)


def _front_cells(projected: ProjectedScan, cell_px: int, limit: int, mask: np.ndarray | None = None) -> np.ndarray:
    """Choose the nearest measured surface in each image cell deterministically."""

    if cell_px <= 0:
        raise ValueError("cell size must be positive")
    positions = np.arange(len(projected.depth_m), dtype=np.int64)
    if mask is not None:
        candidate = np.asarray(mask, dtype=bool)
        if candidate.shape != positions.shape:
            raise ValueError("selection mask does not match projected points")
        positions = positions[candidate]
    if not len(positions):
        return positions
    columns = np.floor(projected.u_px[positions] / cell_px).astype(np.int64)
    rows = np.floor(projected.v_px[positions] / cell_px).astype(np.int64)
    cells = rows * (1 + int(np.max(columns))) + columns
    order = np.lexsort((projected.raw_point_indices[positions], projected.depth_m[positions], cells))
    ordered = positions[order]
    ordered_cells = cells[order]
    first = np.r_[True, ordered_cells[1:] != ordered_cells[:-1]]
    selected = ordered[first]
    raw_order = np.argsort(projected.raw_point_indices[selected], kind="stable")
    selected = selected[raw_order]
    return selected[_even_positions(len(selected), limit)]


def _voxel_positions(points: np.ndarray, raw_indices: np.ndarray, voxel_m: float, limit: int) -> np.ndarray:
    if voxel_m <= 0.0:
        raise ValueError("voxel size must be positive")
    if not len(points):
        return np.empty(0, dtype=np.int64)
    voxels = np.floor(points / voxel_m).astype(np.int64)
    order = np.lexsort((raw_indices, voxels[:, 2], voxels[:, 1], voxels[:, 0]))
    ordered_voxels = voxels[order]
    first = np.r_[True, np.any(ordered_voxels[1:] != ordered_voxels[:-1], axis=1)]
    selected = order[first]
    selected = selected[np.argsort(raw_indices[selected], kind="stable")]
    return selected[_even_positions(len(selected), limit)]


def _slerp_xyzw(first: np.ndarray, second: np.ndarray, alpha: float) -> np.ndarray:
    a = np.asarray(first, dtype=np.float64)
    b = np.asarray(second, dtype=np.float64)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = a + alpha * (b - a)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (math.sin((1.0 - alpha) * angle) / sine) * a + (math.sin(alpha * angle) / sine) * b


class EstimatedTrajectory:
    """Time-ordered estimated sensor-rig poses in the SLAM map frame."""

    def __init__(self, timestamps_s: np.ndarray, positions_m: np.ndarray, quaternions_xyzw: np.ndarray):
        self.timestamps_s = np.asarray(timestamps_s, dtype=np.float64)
        self.positions_m = np.asarray(positions_m, dtype=np.float64)
        self.quaternions_xyzw = np.asarray(quaternions_xyzw, dtype=np.float64)
        if (
            self.timestamps_s.ndim != 1
            or len(self.timestamps_s) < 2
            or self.positions_m.shape != (len(self.timestamps_s), 3)
            or self.quaternions_xyzw.shape != (len(self.timestamps_s), 4)
            or not np.isfinite(self.timestamps_s).all()
            or not np.isfinite(self.positions_m).all()
            or not np.isfinite(self.quaternions_xyzw).all()
            or np.any(np.diff(self.timestamps_s) <= 0.0)
        ):
            raise ValueError("estimated trajectory must contain finite strictly ordered poses")
        norms = np.linalg.norm(self.quaternions_xyzw, axis=1)
        if not np.allclose(norms, 1.0, rtol=0.0, atol=1e-5):
            raise ValueError("estimated trajectory contains a non-unit quaternion")

    @classmethod
    def from_csv(cls, path: str | Path) -> "EstimatedTrajectory":
        timestamps: list[float] = []
        positions: list[tuple[float, float, float]] = []
        quaternions: list[tuple[float, float, float, float]] = []
        with Path(path).open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            required = {"timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError("trajectory CSV lacks timestamp, position, or quaternion fields")
            for row in reader:
                timestamps.append(float(row["timestamp_s"]))
                positions.append(tuple(float(row[name]) for name in ("x_m", "y_m", "z_m")))
                quaternions.append(tuple(float(row[name]) for name in ("qx", "qy", "qz", "qw")))
        return cls(np.asarray(timestamps), np.asarray(positions), np.asarray(quaternions))

    def map_from_sensor_rig(self, timestamp_s: float) -> np.ndarray:
        if timestamp_s < self.timestamps_s[0] - 1e-9 or timestamp_s > self.timestamps_s[-1] + 1e-9:
            raise ValueError("scan timestamp lies outside the estimated trajectory")
        upper = int(np.searchsorted(self.timestamps_s, timestamp_s, side="right"))
        if upper == 0:
            lower = upper = 0
        elif upper >= len(self.timestamps_s):
            lower = upper = len(self.timestamps_s) - 1
        else:
            lower = upper - 1
        if lower == upper:
            alpha = 0.0
        else:
            span = self.timestamps_s[upper] - self.timestamps_s[lower]
            alpha = float((timestamp_s - self.timestamps_s[lower]) / span)
        position = (1.0 - alpha) * self.positions_m[lower] + alpha * self.positions_m[upper]
        quaternion = _slerp_xyzw(self.quaternions_xyzw[lower], self.quaternions_xyzw[upper], alpha)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = quaternion_xyzw_matrix(quaternion)
        matrix[:3, 3] = position
        return matrix


def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64)
    values = np.asarray(points, dtype=np.float64)
    return values @ matrix[:3, :3].T + matrix[:3, 3]


def _scan_index(database_path: Path) -> tuple[ScanRecord, ...]:
    uri = database_path.resolve().as_uri() + "?mode=ro&immutable=1"
    with sqlite3.connect(uri, uri=True) as connection:
        connection.execute("PRAGMA query_only = ON")
        topics = connection.execute(
            "SELECT id, type, serialization_format FROM topics WHERE name = ?", (LIDAR_TOPIC,)
        ).fetchall()
        if len(topics) != 1 or topics[0][1:] != (LIDAR_TYPE, "cdr"):
            raise ValueError("recorded LiDAR topic is not a unique CDR PointCloud2 stream")
        rows = connection.execute(
            "SELECT id, timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp, id", (topics[0][0],)
        ).fetchall()
    records = tuple(ScanRecord(int(message_id), int(timestamp_ns)) for message_id, timestamp_ns in rows)
    if len(records) < 2 or any(b.timestamp_ns <= a.timestamp_ns for a, b in zip(records, records[1:])):
        raise ValueError("recorded LiDAR scan timestamps must be unique and ordered")
    return records


def _rgb_frame_index(path: Path) -> tuple[RgbFrameRecord, ...]:
    records: list[RgbFrameRecord] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            item = json.loads(line)
            records.append(
                RgbFrameRecord(
                    int(item["frame_index"]), float(item["stamp_s"]), int(item["width"]),
                    int(item["height"]), str(item["frame_id"]),
                )
            )
    if not records or any(record.frame_index != index for index, record in enumerate(records)):
        raise ValueError("RGB frame index must be contiguous and zero based")
    if any(b.timestamp_s <= a.timestamp_s for a, b in zip(records, records[1:])):
        raise ValueError("RGB frame timestamps must be strictly ordered")
    return tuple(records)


def _camera_intrinsics(camera: dict[str, object]) -> tuple[CameraIntrinsics, dict[str, object]]:
    """Read either the capture's configured pinhole record or observed ROS CameraInfo."""

    topic = camera.get("topic")
    if topic != "/sim/camera/rgb/camera_info":
        raise ValueError("camera calibration topic is not the recorded RGB CameraInfo topic")
    configured_keys = {"width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px"}
    ros_keys = {"width", "height", "k", "d", "distortion_model"}
    if configured_keys.issubset(camera):
        if camera.get("observed_ros_message") is not False or camera.get("provenance") != "configured_intrinsics":
            raise ValueError("configured camera calibration lacks explicit configured-intrinsics provenance")
        if camera.get("model") != "ideal_pinhole":
            raise ValueError("configured camera calibration is not an ideal pinhole model")
        intrinsics = CameraIntrinsics(
            int(camera["width_px"]), int(camera["height_px"]), float(camera["fx_px"]),
            float(camera["fy_px"]), float(camera["cx_px"]), float(camera["cy_px"]),
        )
        receipt = {
            "representation": "configured_intrinsics",
            "observed_ros_message": False,
            "distortion_handling": "ideal_pinhole_declared_by_capture",
            "topic": topic,
        }
    elif ros_keys.issubset(camera):
        matrix = camera.get("k")
        distortion = camera.get("d")
        if not isinstance(matrix, list) or len(matrix) != 9 or not all(np.isfinite(float(value)) for value in matrix):
            raise ValueError("observed CameraInfo K matrix is invalid")
        if not isinstance(distortion, list) or not all(np.isfinite(float(value)) for value in distortion):
            raise ValueError("observed CameraInfo distortion vector is invalid")
        if any(abs(float(value)) > 1e-12 for value in distortion):
            raise ValueError("nonzero CameraInfo distortion is unsupported without rectification")
        if camera.get("distortion_model") not in ("plumb_bob", ""):
            raise ValueError("observed CameraInfo distortion model is unsupported")
        stamp_s = float(camera.get("stamp_s", -1.0))
        if not np.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError("observed CameraInfo stamp is invalid")
        intrinsics = CameraIntrinsics(
            int(camera["width"]), int(camera["height"]), float(matrix[0]),
            float(matrix[4]), float(matrix[2]), float(matrix[5]),
        )
        receipt = {
            "representation": "observed_ros_camera_info",
            "observed_ros_message": True,
            "stamp_s": stamp_s,
            "distortion_model": camera["distortion_model"],
            "distortion_handling": "zero_coefficients_no_rectification_required",
            "topic": topic,
        }
    else:
        raise ValueError("camera calibration is neither configured pinhole nor observed ROS CameraInfo")
    intrinsics.validate()
    return intrinsics, receipt


def _validate_pointcloud_contract(frame_id: str, field_names: Iterable[str], expected_frame_id: str) -> tuple[str, ...]:
    fields = tuple(str(value) for value in field_names)
    if frame_id != expected_frame_id:
        raise ValueError(
            f"PointCloud2 frame_id {frame_id!r} does not match recorded LiDAR frame {expected_frame_id!r}"
        )
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PointCloud2 lacks required x/y/z fields")
    timing_fields = {"time", "t", "timestamp", "offset_time", "time_offset", "timestamp_ns"}
    present_timing = sorted(set(fields) & timing_fields)
    if present_timing:
        raise ValueError(
            "PointCloud2 contains per-return timing fields but this renderer has no verified deskew "
            f"implementation: {present_timing}"
        )
    return fields


def _load_detection_frames(path: Path, frame_indices: set[int]) -> dict[int, tuple[DetectionRegion, ...]]:
    result: dict[int, tuple[DetectionRegion, ...]] = {}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            item = json.loads(line)
            frame_index = int(item["frame_index"])
            if frame_index not in frame_indices:
                continue
            detections: list[DetectionRegion] = []
            for detection in item.get("detections", []):
                bbox = detection.get("bbox_xyxy")
                if not isinstance(bbox, list) or len(bbox) != 4:
                    continue
                coordinate_source = str(detection.get("coordinate_source", ""))
                if "lidar_projected_with_slam_pose" not in coordinate_source:
                    continue
                center = detection.get("estimated_center_map_m")
                detections.append(
                    DetectionRegion(
                        int(detection["track_id"]),
                        int(detection["raw_track_id"]) if detection.get("raw_track_id") is not None else None,
                        tuple(float(value) for value in bbox),
                        tuple(float(value) for value in center) if isinstance(center, list) and len(center) == 3 else None,
                    )
                )
            result[frame_index] = tuple(detections)
    return result


class SelectiveLidarSource:
    """Bounded real-scan decoder and feature selector for one coherent capture."""

    def __init__(
        self,
        *,
        database_path: str | Path,
        trajectory_path: str | Path,
        camera_info_path: str | Path,
        sensor_transforms_path: str | Path,
        effective_config_path: str | Path,
        rgb_video_path: str | Path,
        rgb_frames_path: str | Path,
        annotations_path: str | Path,
        ffmpeg: str | None = None,
    ):
        self.database_path = Path(database_path)
        self.rgb_video_path = Path(rgb_video_path)
        self.scan_records = _scan_index(self.database_path)
        self.scan_timestamps_s = np.asarray([item.timestamp_s for item in self.scan_records])
        self.rgb_frames = _rgb_frame_index(Path(rgb_frames_path))
        self.rgb_timestamps_s = np.asarray([item.timestamp_s for item in self.rgb_frames])
        self.trajectory = EstimatedTrajectory.from_csv(trajectory_path)
        camera = json.loads(Path(camera_info_path).read_text(encoding="utf-8"))
        transforms = json.loads(Path(sensor_transforms_path).read_text(encoding="utf-8"))
        effective = json.loads(Path(effective_config_path).read_text(encoding="utf-8"))
        frames = transforms.get("frames", {})
        camera_frame = str(frames.get("camera_optical", ""))
        if not camera_frame or camera.get("frame_id") != camera_frame:
            raise ValueError("camera calibration frame does not match the recorded transform graph")
        self.intrinsics, self.camera_calibration_receipt = _camera_intrinsics(camera)
        self.camera_frame_id = camera_frame
        for frame in self.rgb_frames:
            if frame.frame_id != camera_frame:
                raise ValueError("RGB index frame_id does not match the recorded camera optical frame")
            if (frame.width, frame.height) != (self.intrinsics.width_px, self.intrinsics.height_px):
                raise ValueError("RGB index dimensions do not match the bound camera calibration")
        transform_records = transforms.get("transforms")
        if not isinstance(transform_records, list):
            raise ValueError("recorded sensor transform graph is missing")
        self.optical_from_lidar = resolve_transform(
            transform_records, source_frame=str(frames["lidar_link"]), target_frame=str(frames["camera_optical"])
        )
        self.rig_from_lidar = resolve_transform(
            transform_records, source_frame=str(frames["lidar_link"]), target_frame=str(frames["sensor_rig"])
        )
        self.lidar_frame_id = str(frames["lidar_link"])
        lidar_config = effective.get("lidar")
        if not isinstance(lidar_config, dict):
            raise ValueError("effective capture configuration has no LiDAR profile")
        self.minimum_depth_m = float(lidar_config["min_range_m"])
        self.maximum_depth_m = float(lidar_config["max_range_m"])
        if not (0.0 < self.minimum_depth_m < self.maximum_depth_m):
            raise ValueError("effective LiDAR range is invalid")
        self.expected_scan_period_s = 1.0 / float(lidar_config["hz"])
        if not np.isfinite(self.expected_scan_period_s) or self.expected_scan_period_s <= 0.0:
            raise ValueError("effective LiDAR cadence is invalid")
        intervals = np.diff(self.scan_timestamps_s)
        if np.any(intervals > self.expected_scan_period_s * 2.0 + 1e-9):
            raise ValueError("recorded LiDAR scan stream contains a gap beyond the configured cadence")
        self.maximum_current_scan_age_s = self.expected_scan_period_s * 2.0
        nearest_frame_indices = {self.rgb_at(record.timestamp_s).frame_index for record in self.scan_records}
        self.detections = _load_detection_frames(Path(annotations_path), nearest_frame_indices)
        self._projected_cache: OrderedDict[int, tuple[ProjectedScan, int, tuple[str, ...]]] = OrderedDict()
        self._selection_cache: OrderedDict[tuple[int, str, int], PreparedScan] = OrderedDict()
        self._roi_cache: dict[tuple[float, float, int], tuple[RoiObservation, ...]] = {}
        self.ffmpeg = ffmpeg or shutil.which("ffmpeg")
        if not self.ffmpeg:
            raise ValueError("trusted FFmpeg executable is required for deterministic RGB frame decode")
        self._decoder: subprocess.Popen[bytes] | None = None
        self._decoder_size: tuple[int, int] | None = None
        self._decoded_frame_index = -1
        self._decoded_frame: np.ndarray | None = None

    def close(self) -> None:
        if self._decoder is not None:
            self._decoder.terminate()
            try:
                self._decoder.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._decoder.kill()
                self._decoder.wait()
            self._decoder = None

    def _restart_decoder(self, width: int, height: int) -> None:
        self.close()
        self._decoder = subprocess.Popen(
            [
                str(self.ffmpeg), "-hide_banner", "-loglevel", "error", "-i", str(self.rgb_video_path),
                "-an", "-vf", f"scale={width}:{height}:flags=area", "-f", "rawvideo", "-pix_fmt", "bgr24", "-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self._decoder_size = (width, height)
        self._decoded_frame_index = -1
        self._decoded_frame = None

    def _read_decoder_bytes(self, size: int) -> bytes:
        if self._decoder is None or self._decoder.stdout is None:
            raise ValueError("RGB FFmpeg decoder is unavailable")
        chunks: list[bytes] = []
        remaining = size
        while remaining:
            chunk = self._decoder.stdout.read(remaining)
            if not chunk:
                stderr = self._decoder.stderr.read().decode("utf-8", errors="replace") if self._decoder.stderr else ""
                raise ValueError(f"recorded RGB frame decode ended early: {stderr.strip()}")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def rgb_at(self, timestamp_s: float) -> RgbFrameRecord:
        position = int(np.searchsorted(self.rgb_timestamps_s, timestamp_s))
        candidates = [index for index in (position - 1, position) if 0 <= index < len(self.rgb_frames)]
        record = min(candidates, key=lambda index: (abs(self.rgb_frames[index].timestamp_s - timestamp_s), index))
        return self.rgb_frames[record]

    def causal_scan_pair(self, timestamp_s: float) -> tuple[ScanRecord, ScanRecord, float]:
        """Return two scans no later than ``timestamp_s`` with a causal fade.

        A newly arrived scan fades in during the following scan interval.  At
        the next arrival the prior scan is already fully visible, so there is
        no point-set snap and no future measurement is consumed.
        """

        if timestamp_s < self.scan_records[0].timestamp_s - 1e-9:
            raise ValueError("current-view timestamp precedes the first recorded LiDAR scan")
        latest_index = bisect.bisect_right(self.scan_timestamps_s, timestamp_s) - 1
        latest_index = min(len(self.scan_records) - 1, latest_index)
        previous_index = max(0, latest_index - 1)
        previous = self.scan_records[previous_index]
        latest = self.scan_records[latest_index]
        if timestamp_s - latest.timestamp_s > self.maximum_current_scan_age_s + 1e-9:
            raise ValueError("current-view timestamp has no LiDAR scan within the configured cadence")
        if previous_index == latest_index:
            alpha = 1.0
        else:
            interval = latest.timestamp_s - previous.timestamp_s
            alpha = float(np.clip((timestamp_s - latest.timestamp_s) / max(interval, 1e-9), 0.0, 1.0))
            alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        return previous, latest, alpha

    def _projected(self, record: ScanRecord) -> tuple[ProjectedScan, int, tuple[str, ...]]:
        cached = self._projected_cache.get(record.timestamp_ns)
        if cached is not None:
            self._projected_cache.move_to_end(record.timestamp_ns)
            return cached
        bag_record = read_pointcloud2_sqlite(
            self.database_path, topic=LIDAR_TOPIC, timestamp_ns=record.timestamp_ns,
            expected_message_id=record.message_id,
        )
        field_names = _validate_pointcloud_contract(
            bag_record.cloud.frame_id,
            (field.name for field in bag_record.cloud.fields),
            self.lidar_frame_id,
        )
        projected = project_lidar_scan(
            bag_record.cloud.xyz_m,
            bag_record.cloud.raw_point_indices,
            optical_from_lidar=self.optical_from_lidar,
            intrinsics=self.intrinsics,
            minimum_depth_m=self.minimum_depth_m,
            maximum_depth_m=self.maximum_depth_m,
        )
        cached = (projected, len(bag_record.cloud.xyz_m), field_names)
        self._projected_cache[record.timestamp_ns] = cached
        self._projected_cache.move_to_end(record.timestamp_ns)
        while len(self._projected_cache) > 4:
            self._projected_cache.popitem(last=False)
        return cached

    def _map_from_lidar(self, timestamp_s: float) -> np.ndarray:
        return self.trajectory.map_from_sensor_rig(timestamp_s) @ self.rig_from_lidar

    def prepare_scan(self, record: ScanRecord, selection_mode: str, limit: int) -> PreparedScan:
        key = (record.timestamp_ns, selection_mode, limit)
        cached = self._selection_cache.get(key)
        if cached is not None:
            self._selection_cache.move_to_end(key)
            return cached
        projected, raw_count, field_names = self._projected(record)
        if selection_mode == "camera_occlusion_surfaces":
            raw = projected.raw_xyz_m
            planar_range = np.linalg.norm(raw[:, :2], axis=1)
            mask = (
                (raw[:, 0] >= 0.45)
                & (raw[:, 0] <= 12.0)
                & (raw[:, 2] >= -1.45)
                & (raw[:, 2] <= 1.10)
                & ((np.abs(raw[:, 1]) >= 0.82) | (planar_range <= 4.5))
            )
            positions = _front_cells(projected, 48, limit, mask)
        elif selection_mode == "navigation_boundaries":
            raw = projected.raw_xyz_m
            planar_range = np.linalg.norm(raw[:, :2], axis=1)
            mask = (
                (raw[:, 0] >= 0.45)
                & (raw[:, 0] <= 11.0)
                & (raw[:, 2] >= -1.50)
                & (raw[:, 2] <= 1.15)
                & ((np.abs(raw[:, 1]) >= 0.85) | ((np.abs(raw[:, 1]) < 0.85) & (planar_range <= 5.5)))
            )
            positions = _front_cells(projected, 26, limit, mask)
        elif selection_mode == "past_structural_history":
            raw = projected.raw_xyz_m
            mask = (
                (raw[:, 0] >= 0.35)
                & (raw[:, 0] <= 12.0)
                & (raw[:, 2] >= -1.55)
                & (raw[:, 2] <= 1.20)
                & (
                    ((np.abs(raw[:, 1]) >= 0.95) & (raw[:, 2] >= -1.42))
                    | ((raw[:, 2] <= -1.34) & (np.abs(raw[:, 1]) >= 0.72))
                )
            )
            candidates = np.flatnonzero(mask)
            kept = _voxel_positions(raw[candidates], projected.raw_point_indices[candidates], 0.13, limit)
            positions = candidates[kept]
        else:
            raise ValueError(f"unsupported LiDAR selection mode: {selection_mode}")
        raw_xyz = projected.raw_xyz_m[positions]
        map_xyz = _transform_points(self._map_from_lidar(record.timestamp_s), raw_xyz)
        cached = PreparedScan(
            record,
            raw_xyz,
            projected.optical_xyz_m[positions],
            map_xyz,
            projected.u_px[positions],
            projected.v_px[positions],
            projected.depth_m[positions],
            projected.raw_point_indices[positions],
            selection_mode,
            raw_count,
            self.lidar_frame_id,
            field_names,
            "absent_in_point_fields; rigid_header_stamp_projection_without_deskew",
        )
        self._selection_cache[key] = cached
        self._selection_cache.move_to_end(key)
        while len(self._selection_cache) > 256:
            self._selection_cache.popitem(last=False)
        return cached

    def history_scans(self, cutoff_s: float, stride: int, limit_per_scan: int) -> tuple[PreparedScan, ...]:
        if stride <= 0:
            raise ValueError("history stride must be positive")
        eligible = [
            item for item in self.scan_records
            if self.trajectory.timestamps_s[0] - 1e-9 <= item.timestamp_s <= min(cutoff_s, self.trajectory.timestamps_s[-1]) + 1e-9
        ]
        return tuple(
            self.prepare_scan(record, "past_structural_history", limit_per_scan)
            for record in eligible[::stride]
        )

    def roi_observations(self, start_s: float, end_s: float, limit: int = 6) -> tuple[RoiObservation, ...]:
        key = (start_s, end_s, limit)
        cached = self._roi_cache.get(key)
        if cached is not None:
            return cached
        candidates: list[tuple[int, float, ScanRecord, RgbFrameRecord, tuple[DetectionRegion, ...]]] = []
        midpoint = 0.5 * (start_s + end_s)
        for record in self.scan_records:
            if not (start_s <= record.timestamp_s <= end_s):
                continue
            frame = self.rgb_at(record.timestamp_s)
            skew_ns = int(round(abs(frame.timestamp_s - record.timestamp_s) * 1_000_000_000))
            if skew_ns > 17_000_001:
                continue
            detections = self.detections.get(frame.frame_index, ())
            if detections:
                candidates.append((-len(detections), abs(record.timestamp_s - midpoint), record, frame, detections))
        candidates.sort(key=lambda item: (item[0], item[1], item[2].timestamp_ns))
        result: list[RoiObservation] = []
        for _negative_count, _distance, record, frame, detections in candidates[:8]:
            projected, _raw_count, _field_names = self._projected(record)
            ranked = sorted(
                detections,
                key=lambda item: -((item.bbox_xyxy[2] - item.bbox_xyxy[0] + 1.0) * (item.bbox_xyxy[3] - item.bbox_xyxy[1] + 1.0)),
            )
            for detection in ranked:
                support = select_bbox_front_surface(
                    projected, detection.bbox_xyxy, front_surface_band_m=0.16, maximum_selected_points=320,
                )
                if len(support.raw_point_indices) < 3:
                    continue
                result.append(
                    RoiObservation(
                        record,
                        frame,
                        detection,
                        support.optical_xyz_m,
                        _transform_points(self._map_from_lidar(record.timestamp_s), support.raw_xyz_m),
                        support.u_px,
                        support.v_px,
                        support.depth_m,
                        support.raw_point_indices,
                        int(round(abs(frame.timestamp_s - record.timestamp_s) * 1_000_000_000)),
                    )
                )
                if len(result) >= limit:
                    break
            if result:
                break
        if not result:
            raise ValueError("selected source window has no LiDAR-supported estimated ROI")
        cached = tuple(result)
        self._roi_cache[key] = cached
        return cached

    def read_rgb_frame(self, record: RgbFrameRecord, width: int, height: int) -> np.ndarray:
        if self._decoder_size != (width, height) or record.frame_index < self._decoded_frame_index:
            self._restart_decoder(width, height)
        frame_bytes = width * height * 3
        while self._decoded_frame_index < record.frame_index:
            payload = self._read_decoder_bytes(frame_bytes)
            self._decoded_frame = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3).copy()
            self._decoded_frame_index += 1
        if self._decoded_frame is None:
            raise ValueError(f"recorded RGB frame {record.frame_index} could not be decoded")
        return self._decoded_frame.copy()

    @staticmethod
    def scan_summary(scans: Iterable[PreparedScan]) -> dict[str, object]:
        values = tuple(scans)
        if not values:
            return {"scan_count": 0, "selected_return_count": 0, "scans": []}
        return {
            "scan_count": len(values),
            "time_range_s": [values[0].record.timestamp_s, values[-1].record.timestamp_s],
            "selected_return_count": sum(len(item.raw_point_indices) for item in values),
            "scans": [
                {
                    "message_id": item.record.message_id,
                    "timestamp_ns": item.record.timestamp_ns,
                    "raw_point_count": item.raw_point_count,
                    "selected_return_count": len(item.raw_point_indices),
                    "selected_raw_indices_sha256": _indices_sha256(item.raw_point_indices),
                    "selection_mode": item.selection_mode,
                    "source_frame_id": item.source_frame_id,
                    "point_fields": list(item.point_fields),
                    "per_return_timing": item.per_return_timing,
                }
                for item in values
            ],
        }

    @staticmethod
    def roi_summary(observations: Iterable[RoiObservation]) -> dict[str, object]:
        values = tuple(observations)
        return {
            "roi_count": len(values),
            "rois": [
                {
                    "scan_message_id": item.record.message_id,
                    "scan_timestamp_ns": item.record.timestamp_ns,
                    "rgb_frame_index": item.rgb_frame.frame_index,
                    "rgb_timestamp_s": item.rgb_frame.timestamp_s,
                    "absolute_rgb_skew_ns": item.absolute_rgb_skew_ns,
                    "track_id": item.detection.track_id,
                    "raw_track_id": item.detection.raw_track_id,
                    "bbox_xyxy": list(item.detection.bbox_xyxy),
                    "selected_return_count": len(item.raw_point_indices),
                    "selected_raw_indices_sha256": _indices_sha256(item.raw_point_indices),
                    "selection": "bbox_nearest_front_surface",
                }
                for item in values
            ],
        }
