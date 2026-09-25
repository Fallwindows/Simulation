"""Feature-specific selection over an auditable projected LiDAR scan."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from simulator.sensors.scan_projection import ProjectedScan


@dataclass(frozen=True)
class TargetSupport:
    raw_xyz_m: np.ndarray
    optical_xyz_m: np.ndarray
    u_px: np.ndarray
    v_px: np.ndarray
    depth_m: np.ndarray
    raw_point_indices: np.ndarray
    nearest_depth_m: float | None
    front_surface_max_depth_m: float | None
    stage_raw_indices: dict[str, np.ndarray]


def _evenly_spaced_positions(count: int, limit: int) -> np.ndarray:
    if limit <= 0:
        raise ValueError("decimation limit must be positive")
    if count <= limit:
        return np.arange(count, dtype=np.int64)
    # Integer arithmetic includes both ends, is stable across NumPy versions,
    # and cannot produce duplicates while count > limit >= 2.
    if limit == 1:
        return np.asarray([0], dtype=np.int64)
    return (np.arange(limit, dtype=np.int64) * (count - 1)) // (limit - 1)


def select_bbox_front_surface(
    projected: ProjectedScan,
    bbox_xyxy: tuple[float, float, float, float],
    *,
    front_surface_band_m: float = 0.12,
    maximum_selected_points: int = 64,
) -> TargetSupport:
    """Select nearest visible support within a 2D bbox, then bound the receipt.

    The rectangle is inclusive because the existing detector records OpenCV
    component extrema.  Selection first finds all projected returns inside the
    observation, retains the nearest depth band, sorts by immutable raw point
    index, and finally applies deterministic bounded decimation.
    """

    bbox = np.asarray(bbox_xyxy, dtype=np.float64)
    if bbox.shape != (4,) or not np.isfinite(bbox).all():
        raise ValueError("bbox_xyxy must contain four finite values")
    x0, y0, x1, y1 = (float(value) for value in bbox)
    if x1 < x0 or y1 < y0:
        raise ValueError("bbox_xyxy has negative extent")
    if not math.isfinite(front_surface_band_m) or front_surface_band_m < 0.0:
        raise ValueError("front_surface_band_m must be finite and nonnegative")
    if maximum_selected_points <= 0:
        raise ValueError("maximum_selected_points must be positive")

    in_bbox = (
        (projected.u_px >= x0)
        & (projected.u_px <= x1)
        & (projected.v_px >= y0)
        & (projected.v_px <= y1)
    )
    candidate_positions = np.flatnonzero(in_bbox)
    bbox_indices = projected.raw_point_indices[candidate_positions]
    if not len(candidate_positions):
        empty_xyz = np.empty((0, 3), dtype=np.float64)
        empty = np.empty(0, dtype=np.float64)
        empty_indices = np.empty(0, dtype=np.int64)
        return TargetSupport(
            empty_xyz,
            empty_xyz.copy(),
            empty,
            empty.copy(),
            empty.copy(),
            empty_indices,
            None,
            None,
            {"bbox": empty_indices.copy(), "front_surface": empty_indices.copy(), "decimated": empty_indices.copy()},
        )

    candidate_depths = projected.depth_m[candidate_positions]
    nearest = float(np.min(candidate_depths))
    front_max = nearest + front_surface_band_m
    front_positions = candidate_positions[candidate_depths <= front_max]
    order = np.argsort(projected.raw_point_indices[front_positions], kind="stable")
    front_positions = front_positions[order]
    front_indices = projected.raw_point_indices[front_positions]
    keep = _evenly_spaced_positions(len(front_positions), maximum_selected_points)
    selected_positions = front_positions[keep]
    selected_indices = projected.raw_point_indices[selected_positions]
    return TargetSupport(
        raw_xyz_m=projected.raw_xyz_m[selected_positions],
        optical_xyz_m=projected.optical_xyz_m[selected_positions],
        u_px=projected.u_px[selected_positions],
        v_px=projected.v_px[selected_positions],
        depth_m=projected.depth_m[selected_positions],
        raw_point_indices=selected_indices,
        nearest_depth_m=nearest,
        front_surface_max_depth_m=front_max,
        stage_raw_indices={
            "bbox": bbox_indices.copy(),
            "front_surface": front_indices.copy(),
            "decimated": selected_indices.copy(),
        },
    )
