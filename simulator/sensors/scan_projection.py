"""Auditable CPU-only PointCloud2 decoding and camera projection.

The public functions in this module deliberately do not depend on ``rclpy`` or
``rosbag2_py``.  That keeps forensic extraction usable when Windows application
control blocks ROS Python extension loading, while still validating the CDR and
PointCloud2 contracts rather than guessing a packed XYZ layout.
"""

from __future__ import annotations

import math
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


_POINT_FIELD_DTYPES = {7: "f4", 8: "f8"}  # sensor_msgs/PointField FLOAT32/FLOAT64


@dataclass(frozen=True)
class PointField:
    name: str
    offset: int
    datatype: int
    count: int


@dataclass(frozen=True)
class DecodedPointCloud2:
    stamp_ns: int
    frame_id: str
    height: int
    width: int
    fields: tuple[PointField, ...]
    is_bigendian: bool
    point_step: int
    row_step: int
    is_dense: bool
    xyz_m: np.ndarray
    raw_point_indices: np.ndarray


@dataclass(frozen=True)
class BagPointCloud2Record:
    message_id: int
    topic_id: int
    topic: str
    type_name: str
    serialization_format: str
    database_timestamp_ns: int
    cloud: DecodedPointCloud2


@dataclass(frozen=True)
class CameraIntrinsics:
    width_px: int
    height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def validate(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px)
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("camera dimensions must be positive")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx_px <= 0.0 or self.fy_px <= 0.0:
            raise ValueError("camera focal lengths must be positive")


@dataclass(frozen=True)
class ProjectedScan:
    raw_xyz_m: np.ndarray
    optical_xyz_m: np.ndarray
    u_px: np.ndarray
    v_px: np.ndarray
    depth_m: np.ndarray
    raw_point_indices: np.ndarray
    stage_raw_indices: dict[str, np.ndarray]


class _CdrReader:
    def __init__(self, serialized: bytes) -> None:
        if len(serialized) < 4:
            raise ValueError("serialized CDR message is shorter than its encapsulation header")
        representation = serialized[:2]
        if representation == b"\x00\x01":
            self.endian = "<"
        elif representation == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError(f"unsupported CDR representation identifier {representation.hex()}")
        if serialized[2:4] != b"\x00\x00":
            raise ValueError("CDR encapsulation options must be zero")
        self.data = memoryview(serialized)
        self.offset = 4

    def _align(self, alignment: int) -> None:
        padding = (-self.offset) % alignment
        if self.offset + padding > len(self.data):
            raise ValueError("truncated CDR alignment padding")
        self.offset += padding

    def _unpack(self, code: str, alignment: int) -> int:
        self._align(alignment)
        size = struct.calcsize(code)
        if self.offset + size > len(self.data):
            raise ValueError("truncated CDR primitive")
        value = struct.unpack_from(self.endian + code, self.data, self.offset)[0]
        self.offset += size
        return int(value)

    def u8(self) -> int:
        return self._unpack("B", 1)

    def i32(self) -> int:
        return self._unpack("i", 4)

    def u32(self) -> int:
        return self._unpack("I", 4)

    def string(self) -> str:
        length = self.u32()
        if length < 1 or self.offset + length > len(self.data):
            raise ValueError("invalid or truncated CDR string")
        raw = bytes(self.data[self.offset : self.offset + length])
        self.offset += length
        if raw[-1:] != b"\x00":
            raise ValueError("CDR string is not NUL terminated")
        try:
            return raw[:-1].decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("CDR string is not valid UTF-8") from exc

    def octet_sequence(self) -> bytes:
        length = self.u32()
        if self.offset + length > len(self.data):
            raise ValueError("truncated CDR octet sequence")
        raw = bytes(self.data[self.offset : self.offset + length])
        self.offset += length
        return raw

    def finish(self) -> None:
        trailing = bytes(self.data[self.offset :])
        if len(trailing) > 3 or any(trailing):
            raise ValueError("unexpected trailing bytes in PointCloud2 CDR message")


def _decode_xyz(
    data: bytes,
    *,
    fields: Iterable[PointField],
    height: int,
    width: int,
    point_step: int,
    row_step: int,
    is_bigendian: bool,
) -> np.ndarray:
    fields = tuple(fields)
    field_by_name = {field.name: field for field in fields}
    if len(field_by_name) != len(fields):
        raise ValueError("PointCloud2 field names must be unique")
    if any(name not in field_by_name for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 is missing one or more XYZ fields")
    endian = ">" if is_bigendian else "<"
    columns: list[np.ndarray] = []
    for name in ("x", "y", "z"):
        field = field_by_name[name]
        if field.datatype not in _POINT_FIELD_DTYPES:
            raise ValueError(f"unsupported PointCloud2 datatype {field.datatype} for {name}")
        if field.count != 1:
            raise ValueError(f"PointCloud2 field {name} must have count=1")
        dtype = np.dtype(endian + _POINT_FIELD_DTYPES[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > point_step:
            raise ValueError(f"PointCloud2 field {name} falls outside point_step")
        column = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=data,
            offset=field.offset,
            strides=(row_step, point_step),
        )
        columns.append(np.asarray(column, dtype=np.float64).reshape(-1))
    return np.column_stack(columns)


def decode_pointcloud2_cdr(serialized: bytes) -> DecodedPointCloud2:
    """Decode one CDR ``sensor_msgs/msg/PointCloud2`` message strictly."""

    reader = _CdrReader(serialized)
    seconds = reader.i32()
    nanoseconds = reader.u32()
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("PointCloud2 header has an invalid timestamp")
    frame_id = reader.string()
    if not frame_id:
        raise ValueError("PointCloud2 header frame_id is empty")
    height = reader.u32()
    width = reader.u32()
    if height <= 0 or width <= 0:
        raise ValueError("PointCloud2 width and height must be positive")
    field_count = reader.u32()
    if field_count <= 0 or field_count > 128:
        raise ValueError("PointCloud2 field count is invalid")
    fields = tuple(
        PointField(reader.string(), reader.u32(), reader.u8(), reader.u32())
        for _ in range(field_count)
    )
    is_bigendian_raw = reader.u8()
    if is_bigendian_raw not in (0, 1):
        raise ValueError("PointCloud2 is_bigendian is not a valid CDR boolean")
    point_step = reader.u32()
    row_step = reader.u32()
    if point_step <= 0 or row_step < width * point_step:
        raise ValueError("PointCloud2 point_step or row_step is inconsistent")
    point_data = reader.octet_sequence()
    if len(point_data) != height * row_step:
        raise ValueError("PointCloud2 data length does not equal height * row_step")
    is_dense_raw = reader.u8()
    if is_dense_raw not in (0, 1):
        raise ValueError("PointCloud2 is_dense is not a valid CDR boolean")
    reader.finish()
    xyz = _decode_xyz(
        point_data,
        fields=fields,
        height=height,
        width=width,
        point_step=point_step,
        row_step=row_step,
        is_bigendian=bool(is_bigendian_raw),
    )
    count = height * width
    if xyz.shape != (count, 3):
        raise ValueError("decoded PointCloud2 XYZ shape is inconsistent")
    return DecodedPointCloud2(
        stamp_ns=seconds * 1_000_000_000 + nanoseconds,
        frame_id=frame_id,
        height=height,
        width=width,
        fields=fields,
        is_bigendian=bool(is_bigendian_raw),
        point_step=point_step,
        row_step=row_step,
        is_dense=bool(is_dense_raw),
        xyz_m=xyz,
        raw_point_indices=np.arange(count, dtype=np.int64),
    )


def read_pointcloud2_sqlite(
    database_path: str | Path,
    *,
    topic: str,
    timestamp_ns: int,
    expected_message_id: int | None = None,
) -> BagPointCloud2Record:
    """Read one exact-timestamp PointCloud2 record through SQLite read-only mode."""

    database = Path(database_path).resolve()
    if not database.is_file():
        raise ValueError(f"bag database is missing: {database}")
    uri = database.as_uri() + "?mode=ro&immutable=1"
    with sqlite3.connect(uri, uri=True) as connection:
        connection.execute("PRAGMA query_only = ON")
        topic_rows = connection.execute(
            "SELECT id, type, serialization_format FROM topics WHERE name = ?", (topic,)
        ).fetchall()
        if len(topic_rows) != 1:
            raise ValueError(f"expected exactly one topic record for {topic!r}")
        topic_id, type_name, serialization_format = topic_rows[0]
        if type_name != "sensor_msgs/msg/PointCloud2" or serialization_format != "cdr":
            raise ValueError("LiDAR topic is not CDR sensor_msgs/msg/PointCloud2")
        message_rows = connection.execute(
            "SELECT id, timestamp, data FROM messages WHERE topic_id = ? AND timestamp = ? ORDER BY id",
            (topic_id, int(timestamp_ns)),
        ).fetchall()
    if len(message_rows) != 1:
        raise ValueError(
            f"expected exactly one {topic!r} message at {timestamp_ns} ns, found {len(message_rows)}"
        )
    message_id, database_timestamp_ns, serialized = message_rows[0]
    if expected_message_id is not None and message_id != expected_message_id:
        raise ValueError(f"bag message id {message_id} does not match expected id {expected_message_id}")
    cloud = decode_pointcloud2_cdr(serialized)
    if cloud.stamp_ns != database_timestamp_ns:
        raise ValueError("PointCloud2 header timestamp does not match rosbag database timestamp")
    return BagPointCloud2Record(
        message_id=int(message_id),
        topic_id=int(topic_id),
        topic=topic,
        type_name=str(type_name),
        serialization_format=str(serialization_format),
        database_timestamp_ns=int(database_timestamp_ns),
        cloud=cloud,
    )


def quaternion_xyzw_matrix(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    values = np.asarray(tuple(quaternion_xyzw), dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("rotation_xyzw must contain four finite values")
    norm = math.sqrt(sum(float(value) * float(value) for value in values))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"rotation_xyzw must be unit length, got norm {norm}")
    x, y, z, w = values
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation_m: Iterable[float], rotation_xyzw: Iterable[float]) -> np.ndarray:
    translation = np.asarray(tuple(translation_m), dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("translation_m must contain three finite values")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_xyzw_matrix(rotation_xyzw)
    matrix[:3, 3] = translation
    return matrix


def _invert_rigid(matrix: np.ndarray) -> np.ndarray:
    """Invert a rigid matrix without invoking a platform BLAS backend."""

    result = np.eye(4, dtype=np.float64)
    rotation = matrix[:3, :3]
    result[:3, :3] = rotation.T
    translation = matrix[:3, 3]
    result[:3, 3] = -np.asarray(
        [
            rotation[0, 0] * translation[0] + rotation[1, 0] * translation[1] + rotation[2, 0] * translation[2],
            rotation[0, 1] * translation[0] + rotation[1, 1] * translation[1] + rotation[2, 1] * translation[2],
            rotation[0, 2] * translation[0] + rotation[1, 2] * translation[1] + rotation[2, 2] * translation[2],
        ],
        dtype=np.float64,
    )
    return result


def _compose_rigid(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Return ``left * right`` using explicit rigid-transform arithmetic."""

    result = np.eye(4, dtype=np.float64)
    for row in range(3):
        for column in range(3):
            result[row, column] = sum(
                left[row, inner] * right[inner, column] for inner in range(3)
            )
        result[row, 3] = left[row, 3] + sum(
            left[row, inner] * right[inner, 3] for inner in range(3)
        )
    return result


def resolve_transform(
    transforms: Iterable[dict[str, object]], *, source_frame: str, target_frame: str
) -> np.ndarray:
    """Resolve ``T_target_from_source`` from parent/child mount records."""

    if not source_frame or not target_frame:
        raise ValueError("source and target frame names must be nonempty")
    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {}
    seen_edges: set[tuple[str, str]] = set()
    for record in transforms:
        parent = record.get("parent")
        child = record.get("child")
        if not isinstance(parent, str) or not isinstance(child, str) or not parent or not child:
            raise ValueError("transform record has invalid parent or child")
        if parent == child or (parent, child) in seen_edges:
            raise ValueError("transform tree contains a self edge or duplicate edge")
        seen_edges.add((parent, child))
        parent_from_child = transform_matrix(record.get("translation_m", ()), record.get("rotation_xyzw", ()))
        child_from_parent = _invert_rigid(parent_from_child)
        adjacency.setdefault(child, []).append((parent, parent_from_child))
        adjacency.setdefault(parent, []).append((child, child_from_parent))
    if source_frame not in adjacency or target_frame not in adjacency:
        raise ValueError("source or target frame is absent from the transform graph")
    queue: list[tuple[str, np.ndarray]] = [(source_frame, np.eye(4, dtype=np.float64))]
    visited = {source_frame}
    for current, current_from_source in queue:
        if current == target_frame:
            return current_from_source
        for neighbor, neighbor_from_current in adjacency[current]:
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append((neighbor, _compose_rigid(neighbor_from_current, current_from_source)))
    raise ValueError(f"no transform path from {source_frame!r} to {target_frame!r}")


def project_lidar_scan(
    xyz_m: np.ndarray,
    raw_point_indices: np.ndarray,
    *,
    optical_from_lidar: np.ndarray,
    intrinsics: CameraIntrinsics,
    minimum_depth_m: float = 0.05,
    maximum_depth_m: float | None = None,
) -> ProjectedScan:
    """Filter and project raw LiDAR points while preserving source indices."""

    intrinsics.validate()
    points = np.asarray(xyz_m, dtype=np.float64)
    indices = np.asarray(raw_point_indices, dtype=np.int64)
    transform = np.asarray(optical_from_lidar, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz_m must have shape (N, 3)")
    if indices.shape != (len(points),) or len(np.unique(indices)) != len(indices):
        raise ValueError("raw_point_indices must be a unique length-N vector")
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("optical_from_lidar must be a finite 4x4 matrix")
    if not math.isfinite(minimum_depth_m) or minimum_depth_m <= 0.0:
        raise ValueError("minimum_depth_m must be finite and positive")
    if maximum_depth_m is not None and (
        not math.isfinite(maximum_depth_m) or maximum_depth_m <= minimum_depth_m
    ):
        raise ValueError("maximum_depth_m must exceed minimum_depth_m")

    stages: dict[str, np.ndarray] = {"raw": indices.copy()}
    finite_mask = np.isfinite(points).all(axis=1)
    points = points[finite_mask]
    indices = indices[finite_mask]
    stages["finite"] = indices.copy()

    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    optical = np.column_stack(
        tuple(
            rotation[row, 0] * points[:, 0]
            + rotation[row, 1] * points[:, 1]
            + rotation[row, 2] * points[:, 2]
            + translation[row]
            for row in range(3)
        )
    )
    forward_mask = optical[:, 2] > minimum_depth_m
    if maximum_depth_m is not None:
        forward_mask &= optical[:, 2] <= maximum_depth_m
    points = points[forward_mask]
    optical = optical[forward_mask]
    indices = indices[forward_mask]
    stages["forward"] = indices.copy()

    depth = optical[:, 2]
    u = intrinsics.fx_px * optical[:, 0] / depth + intrinsics.cx_px
    v = intrinsics.fy_px * optical[:, 1] / depth + intrinsics.cy_px
    image_mask = (
        np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0.0)
        & (u < intrinsics.width_px)
        & (v >= 0.0)
        & (v < intrinsics.height_px)
    )
    stages["in_image"] = indices[image_mask].copy()
    return ProjectedScan(
        raw_xyz_m=points[image_mask],
        optical_xyz_m=optical[image_mask],
        u_px=u[image_mask],
        v_px=v[image_mask],
        depth_m=depth[image_mask],
        raw_point_indices=indices[image_mask],
        stage_raw_indices=stages,
    )
