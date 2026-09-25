"""CPU renderer for selective technical views of recorded RGB/LiDAR data.

The renderer consumes one exact capture/SLAM/perception bundle.  Current views
project real timestamped PointCloud2 returns into their co-timed recorded RGB
camera, while map views transform bounded past-only return subsets through the
estimated trajectory.  It never reads scene geometry, assets, simulator truth,
or storyboard pixels.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from simulator.perception.provenance import (
    validate_perception_frame_coverage,
    validate_perception_manifest_bindings,
)
PRODUCER_ID = "grocery_sim.technical_views.cpu.v1"
ALLOWED_ESTIMATED_DEPTH_SOURCES = frozenset({"lidar_projected_with_slam_pose"})
VIEW_ORDER = (
    "sensor_activation",
    "lidar_environment",
    "persistent_map",
    "object_association",
    "object_detail",
    "observed_aisle_overview",
    "final_technical_view",
)
FONT_REGULAR = Path("C:/Windows/Fonts/segoeui.ttf")
FONT_BOLD = Path("C:/Windows/Fonts/segoeuib.ttf")
IMPLEMENTATION_SOURCES = (
    "simulator/technical_lidar.py",
    "simulator/sensors/scan_projection.py",
    "simulator/sensors/feature_selection.py",
)
CAMERA_TRACE_BASIS = (
    "shots 6-7 audit direct recorded camera_optical poses without applying a presentation guide; "
    "shots 8-12 apply a C3 presentation guide fit to the disclosed estimated-trajectory dependency; "
    "both remain independent of rendered scan cutoff"
)
CAMERA_TRACE_POSITION_DECIMALS = 9
CAMERA_TRACE_DERIVATIVE_DECIMALS = 7


def _opencv():
    """Load OpenCV only in render paths; provenance inspection stays native-only."""

    import cv2

    return cv2


class _LazyOpenCv:
    def __getattr__(self, name: str):
        return getattr(_opencv(), name)


cv2 = _LazyOpenCv()


@dataclass(frozen=True)
class RenderProfile:
    name: str
    width: int
    height: int
    fps: int
    crf: int
    preset: str


@dataclass(frozen=True)
class ViewSpec:
    id: str
    presentation_role: str
    frames: int
    title: str
    subtitle: str
    capability_focus: str
    point_source: str
    selective_current_scan_status: str
    selection_mode: str
    temporal_mode: str
    source_window_s: tuple[float, float]
    display_window_s: tuple[float, float]
    rgb_context: bool
    maximum_points: int
    history_stride_scans: int
    camera_motion_role: str
    camera_guide_timestamp_window_s: tuple[float, float]
    camera_pose_dependency_window_s: tuple[float, float]
    final_hold_frames: int


@dataclass(frozen=True)
class CameraPose:
    eye_m: np.ndarray
    target_m: np.ndarray
    source_timestamp_s: float
    motion_phase: float


def _smootherstep(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


def _smootherstep_c3(value: float) -> float:
    """Seventh-order smoothstep with zero derivatives through jerk at both ends."""

    x = float(np.clip(value, 0.0, 1.0))
    return x**4 * (35.0 - 84.0 * x + 70.0 * x**2 - 20.0 * x**3)


def _smooth_window(value: float, rise_start: float, rise_end: float, fall_start: float, fall_end: float) -> float:
    rise = _smootherstep_c3((value - rise_start) / (rise_end - rise_start))
    fall = 1.0 - _smootherstep_c3((value - fall_start) / (fall_end - fall_start))
    return float(np.clip(rise * fall, 0.0, 1.0))


def _activation_ray_opacities(progress: float, count: int = 12) -> np.ndarray:
    """Return continuous, staggered ray opacity without whole-line pops."""

    if count <= 0:
        raise ValueError("activation ray count must be positive")
    activation = float(np.clip(progress / 0.38, 0.0, 1.0))
    starts = np.linspace(0.0, 0.64, count, dtype=np.float64)
    return np.asarray([_smootherstep((activation - start) / 0.36) for start in starts], dtype=np.float32)


class _C3TimestampCurve:
    """Monotone piecewise septic interpolation with shared C3 knot state."""

    _END_MATRIX_INVERSE = np.asarray([
        [35.0, -15.0, 2.5, -1.0 / 6.0],
        [-84.0, 39.0, -7.0, 0.5],
        [70.0, -34.0, 6.5, -0.5],
        [-20.0, 10.0, -2.0, 1.0 / 6.0],
    ], dtype=np.float64)

    def __init__(self, frames: Iterable[int], timestamps_s: Iterable[float]):
        self.frames = np.asarray(tuple(frames), dtype=np.float64)
        self.timestamps_s = np.asarray(tuple(timestamps_s), dtype=np.float64)
        if (
            len(self.frames) < 2
            or self.frames.shape != self.timestamps_s.shape
            or np.any(np.diff(self.frames) <= 0.0)
            or np.any(np.diff(self.timestamps_s) <= 0.0)
        ):
            raise ValueError("technical camera timestamp knots must be strictly ordered")
        spans = np.diff(self.frames)
        secants = np.diff(self.timestamps_s) / spans
        slopes = np.zeros_like(self.timestamps_s)
        for index in range(1, len(slopes) - 1):
            left = secants[index - 1]
            right = secants[index]
            if left > 0.0 and right > 0.0:
                left_span = spans[index - 1]
                right_span = spans[index]
                w1 = 2.0 * right_span + left_span
                w2 = right_span + 2.0 * left_span
                slopes[index] = (w1 + w2) / (w1 / left + w2 / right)
        # The guide begins and enters its final hold with zero velocity.  All
        # interior knots retain a nonzero shared velocity and zero shared
        # acceleration/jerk, avoiding per-shot stop/start pulses.
        slopes[0] = 0.0
        slopes[-1] = 0.0
        coefficients: list[np.ndarray] = []
        for index, span in enumerate(spans):
            c0 = self.timestamps_s[index]
            c1 = slopes[index] * span
            known_end = c0 + c1
            rhs = np.asarray([
                self.timestamps_s[index + 1] - known_end,
                slopes[index + 1] * span - c1,
                0.0,
                0.0,
            ], dtype=np.float64)
            high = self._END_MATRIX_INVERSE @ rhs
            coefficients.append(np.concatenate((np.asarray([c0, c1, 0.0, 0.0]), high)))
        self.coefficients = tuple(coefficients)

        probes = np.linspace(self.frames[0], self.frames[-1], 4097)
        sampled = np.asarray([self.evaluate(value) for value in probes])
        if np.any(np.diff(sampled) < -1e-10):
            raise ValueError("technical camera timestamp schedule is not monotone")

    def evaluate(self, frame: float) -> float:
        value = float(np.clip(frame, self.frames[0], self.frames[-1]))
        segment = int(np.searchsorted(self.frames, value, side="right") - 1)
        segment = min(max(segment, 0), len(self.coefficients) - 1)
        span = self.frames[segment + 1] - self.frames[segment]
        x = (value - self.frames[segment]) / span
        return float(np.polynomial.polynomial.polyval(x, self.coefficients[segment]))


class _EstimatedPoseGuide:
    """Global smooth Bezier guide sampled only from the estimated trajectory."""

    def __init__(
        self,
        trajectory: object,
        rig_from_optical: np.ndarray,
        start_s: float,
        end_s: float,
        sample_count: int = 12,
    ):
        self.start_s = float(start_s)
        self.end_s = float(end_s)
        timestamps = np.linspace(self.start_s, self.end_s, sample_count)
        rig_positions: list[np.ndarray] = []
        optical_positions: list[np.ndarray] = []
        forwards: list[np.ndarray] = []
        for timestamp_s in timestamps:
            map_from_rig = trajectory.map_from_sensor_rig(float(timestamp_s))
            rotation = np.asarray(map_from_rig[:3, :3], dtype=np.float64)
            rig_position = np.asarray(map_from_rig[:3, 3], dtype=np.float64)
            optical_offset = np.asarray(rig_from_optical[:3, 3], dtype=np.float64)
            optical_axis = np.asarray(rig_from_optical[:3, 2], dtype=np.float64)
            # Explicit products avoid dispatching a threaded BLAS kernel for
            # twelve tiny 3x3 transforms on constrained render workers.
            optical_position = rig_position + np.sum(rotation * optical_offset[None, :], axis=1)
            optical_forward = np.sum(rotation * optical_axis[None, :], axis=1)
            rig_positions.append(rig_position)
            optical_positions.append(optical_position)
            forwards.append(optical_forward)
        self._rig_controls = np.stack(rig_positions)
        self._optical_controls = np.stack(optical_positions)
        self._forward_controls = np.stack(forwards)

    def _normalized(self, timestamp_s: float | np.ndarray) -> float | np.ndarray:
        return 2.0 * (np.asarray(timestamp_s) - self.start_s) / (self.end_s - self.start_s) - 1.0

    def evaluate(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if timestamp_s < self.start_s - 1e-9 or timestamp_s > self.end_s + 1e-9:
            raise ValueError("technical camera pose timestamp lies outside its estimated-pose fit")
        x = float((timestamp_s - self.start_s) / (self.end_s - self.start_s))

        def bezier(controls: np.ndarray) -> np.ndarray:
            work = controls.copy()
            for count in range(len(work) - 1, 0, -1):
                work[:count] = (1.0 - x) * work[:count] + x * work[1:count + 1]
            return work[0]

        rig = bezier(self._rig_controls)
        optical = bezier(self._optical_controls)
        forward = bezier(self._forward_controls)
        norm = float(np.linalg.norm(forward))
        if norm <= 1e-9:
            raise ValueError("technical camera estimated optical direction is degenerate")
        return rig, optical, forward / norm


class TechnicalCameraPath:
    """Direct replay-pose audit followed by a continuous map-camera guide.

    Shots 6-7 report the direct recorded optical pose but do not apply it as a
    presentation camera. ``EstimatedTrajectory`` intentionally interpolates its
    source samples piecewise. The map camera samples only that trajectory,
    builds a smooth Bezier guide from the disclosed global dependency, and
    evaluates it on a C3 timestamp schedule. It does not replace scan
    projection or alter any measured point.
    """

    def __init__(
        self,
        views: tuple[ViewSpec, ...],
        trajectory: object,
        rig_from_optical: np.ndarray,
        roi_focus_m: np.ndarray,
    ):
        self.views = views
        self.trajectory = trajectory
        self.rig_from_optical = np.asarray(rig_from_optical, dtype=np.float64)
        self.roi_focus_m = np.asarray(roi_focus_m, dtype=np.float64)
        if self.rig_from_optical.shape != (4, 4) or self.roi_focus_m.shape != (3,):
            raise ValueError("technical camera path inputs have invalid dimensions")
        self.offsets: dict[str, int] = {}
        offset = 0
        for view in views:
            self.offsets[view.id] = offset
            offset += view.frames
        self.total_frames = offset
        self.map_views = tuple(view for view in views if view.camera_motion_role == "continuous_estimated_map_path")
        if not self.map_views:
            raise ValueError("technical camera path has no map views")
        self.map_start_global = self.offsets[self.map_views[0].id]
        self.map_total_frames = sum(view.frames for view in self.map_views)
        self.final_hold_frames = self.map_views[-1].final_hold_frames
        self.motion_end_global = self.total_frames - self.final_hold_frames
        self.map_motion_frames = self.motion_end_global - self.map_start_global
        if self.map_motion_frames <= 1:
            raise ValueError("technical camera path has no moving map interval")
        knot_frames = [self.offsets[view.id] for view in views]
        knot_frames.append(self.motion_end_global)
        knot_times = [view.camera_guide_timestamp_window_s[0] for view in views]
        knot_times.append(views[-1].camera_guide_timestamp_window_s[1])
        for left, right in zip(views, views[1:]):
            if not math.isclose(
                left.camera_guide_timestamp_window_s[1],
                right.camera_guide_timestamp_window_s[0],
                abs_tol=1e-9,
            ):
                raise ValueError("technical camera guide timestamp windows must be contiguous")
        self._timestamp_curve = _C3TimestampCurve(knot_frames, knot_times)
        dependency_window = getattr(trajectory, "interpolation_dependency_window_s", None)
        if not callable(dependency_window):
            raise ValueError("technical camera trajectory cannot disclose interpolation dependencies")
        for view in views:
            if view.camera_motion_role != "recorded_camera_optical":
                continue
            actual_dependency = dependency_window(*view.camera_guide_timestamp_window_s)
            if not all(
                math.isclose(actual, declared, abs_tol=1e-9)
                for actual, declared in zip(actual_dependency, view.camera_pose_dependency_window_s)
            ):
                raise ValueError(
                    f"technical view {view.id} camera pose dependency window does not match "
                    f"trajectory interpolation support {actual_dependency}"
                )
        map_dependency_windows = {view.camera_pose_dependency_window_s for view in self.map_views}
        if len(map_dependency_windows) != 1:
            raise ValueError("map camera views must disclose one shared smooth-fit dependency window")
        map_fit_window = (knot_times[0], knot_times[-1])
        actual_map_dependency = dependency_window(*map_fit_window)
        declared_map_dependency = next(iter(map_dependency_windows))
        if not all(
            math.isclose(actual, declared, abs_tol=1e-9)
            for actual, declared in zip(actual_map_dependency, declared_map_dependency)
        ):
            raise ValueError(
                "map camera pose dependency window does not match trajectory interpolation "
                f"support {actual_map_dependency}"
            )
        self._pose_guide = _EstimatedPoseGuide(
            trajectory,
            self.rig_from_optical,
            map_fit_window[0],
            map_fit_window[1],
        )

    def _direct_recorded_camera_pose(self, timestamp_s: float, phase: float) -> CameraPose:
        map_from_rig = self.trajectory.map_from_sensor_rig(timestamp_s)
        rotation = np.asarray(map_from_rig[:3, :3], dtype=np.float64)
        rig_position = np.asarray(map_from_rig[:3, 3], dtype=np.float64)
        optical_offset = np.asarray(self.rig_from_optical[:3, 3], dtype=np.float64)
        optical_axis = np.asarray(self.rig_from_optical[:3, 2], dtype=np.float64)
        eye = rig_position + np.sum(rotation * optical_offset[None, :], axis=1)
        forward = np.sum(rotation * optical_axis[None, :], axis=1)
        target = eye + 5.0 * forward / max(float(np.linalg.norm(forward)), 1e-9)
        return CameraPose(eye.astype(np.float64), target.astype(np.float64), timestamp_s, phase)

    def _desired_map_pose(self, raw_phase: float, source_timestamp_s: float) -> CameraPose:
        motion = _smootherstep_c3(raw_phase)
        rig_position, _optical_eye, _forward = self._pose_guide.evaluate(source_timestamp_s)
        aisle_target = rig_position + np.asarray([0.0, 0.0, 0.85])
        roi_weight = _smooth_window(raw_phase, 0.16, 0.30, 0.52, 0.68)
        target = (1.0 - 0.86 * roi_weight) * aisle_target + (0.86 * roi_weight) * self.roi_focus_m
        angle = math.radians(-96.0 + 24.0 * motion)
        radius = 9.5 - 5.0 * roi_weight + 1.5 * _smootherstep_c3((raw_phase - 0.70) / 0.30)
        elevation = 4.2 - 2.0 * roi_weight
        eye = target + np.asarray(
            [radius * math.cos(angle), radius * math.sin(angle), elevation], dtype=np.float64
        )
        return CameraPose(eye, target, source_timestamp_s, raw_phase)

    def state(self, spec: ViewSpec, frame_index: int) -> CameraPose:
        if frame_index < 0 or frame_index >= spec.frames:
            raise IndexError("technical camera frame index is out of range")
        global_index = self.offsets[spec.id] + frame_index
        timestamp_s = self._timestamp_curve.evaluate(min(global_index, self.motion_end_global))
        phase = global_index / max(self.total_frames - 1, 1)
        if spec.camera_motion_role == "recorded_camera_optical":
            return self._direct_recorded_camera_pose(timestamp_s, phase)
        if spec.camera_motion_role != "continuous_estimated_map_path":
            raise ValueError(f"unsupported technical camera motion role: {spec.camera_motion_role}")

        map_index = global_index - self.map_start_global
        raw_phase = float(np.clip(map_index / self.map_motion_frames, 0.0, 1.0))
        desired = self._desired_map_pose(raw_phase, timestamp_s)
        recorded = self._direct_recorded_camera_pose(timestamp_s, raw_phase)
        bridge = _smootherstep_c3(raw_phase / 0.28)
        eye = (1.0 - bridge) * recorded.eye_m + bridge * desired.eye_m
        target = (1.0 - bridge) * recorded.target_m + bridge * desired.target_m
        return CameraPose(eye, target, desired.source_timestamp_s, phase)

    def trace_rows(self, fps: int) -> list[dict[str, object]]:
        poses: list[tuple[ViewSpec, int, CameraPose]] = []
        for spec in self.views:
            poses.extend((spec, index, self.state(spec, index)) for index in range(spec.frames))
        eyes = np.stack([pose.eye_m for _spec, _index, pose in poses])
        targets = np.stack([pose.target_m for _spec, _index, pose in poses])
        velocities = np.zeros_like(eyes)
        accelerations = np.zeros_like(eyes)
        jerks = np.zeros_like(eyes)
        target_velocities = np.zeros_like(targets)
        target_accelerations = np.zeros_like(targets)
        target_jerks = np.zeros_like(targets)
        velocities[1:] = np.diff(eyes, axis=0) * fps
        accelerations[2:] = np.diff(velocities[1:], axis=0) * fps
        jerks[3:] = np.diff(accelerations[2:], axis=0) * fps
        target_velocities[1:] = np.diff(targets, axis=0) * fps
        target_accelerations[2:] = np.diff(target_velocities[1:], axis=0) * fps
        target_jerks[3:] = np.diff(target_accelerations[2:], axis=0) * fps
        rows: list[dict[str, object]] = []
        for global_index, values in enumerate(
            zip(poses, eyes, targets, velocities, accelerations, jerks,
                target_velocities, target_accelerations, target_jerks)
        ):
            ((spec, local_index, pose), eye, target, velocity, acceleration, jerk,
             target_velocity, target_acceleration, target_jerk) = values
            if spec.temporal_mode == "current_window":
                cutoff_s = spec.display_window_s[0] + (
                    spec.display_window_s[1] - spec.display_window_s[0]
                ) * (local_index / spec.frames)
            elif spec.temporal_mode == "past_only_reveal":
                progress = 0.0 if spec.frames == 1 else local_index / (spec.frames - 1)
                eased = progress * progress * (3.0 - 2.0 * progress)
                cutoff_s = spec.display_window_s[0] + (
                    spec.display_window_s[1] - spec.display_window_s[0]
                ) * eased
            else:
                cutoff_s = spec.display_window_s[1]
            rows.append({
                "global_frame": global_index,
                "view_id": spec.id,
                "view_frame": local_index,
                "boundary_from_previous": local_index == 0 and global_index > 0,
                "camera_motion_role": spec.camera_motion_role,
                "camera_guide_pose_timestamp_s": round(pose.source_timestamp_s, 9),
                "rendered_data_cutoff_s": round(cutoff_s, 9),
                "motion_phase": round(pose.motion_phase, 9),
                "eye_m": [round(float(value), CAMERA_TRACE_POSITION_DECIMALS) for value in eye],
                "target_m": [round(float(value), CAMERA_TRACE_POSITION_DECIMALS) for value in target],
                "eye_velocity_mps": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in velocity],
                "eye_acceleration_mps2": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in acceleration],
                "eye_jerk_mps3": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in jerk],
                "target_velocity_mps": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in target_velocity],
                "target_acceleration_mps2": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in target_acceleration],
                "target_jerk_mps3": [round(float(value), CAMERA_TRACE_DERIVATIVE_DECIMALS) for value in target_jerk],
            })
        return rows


def _camera_trace_view_sha256(rows: Iterable[dict[str, object]]) -> str:
    digest = hashlib.sha256()
    for row in rows:
        bound = {
            "global_frame": row["global_frame"],
            "view_frame": row["view_frame"],
            "camera_guide_pose_timestamp_s": row["camera_guide_pose_timestamp_s"],
            "rendered_data_cutoff_s": row["rendered_data_cutoff_s"],
            "eye_m": row["eye_m"],
            "target_m": row["target_m"],
        }
        digest.update(json.dumps(bound, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        digest.update(b"\n")
    return digest.hexdigest()


def camera_motion_receipt(
    spec: ViewSpec,
    rows: Iterable[dict[str, object]],
    focus_inventory: object,
) -> dict[str, object]:
    values = tuple(rows)
    if len(values) != spec.frames:
        raise ValueError(f"technical camera trace row count is invalid for {spec.id}")
    timestamps = [float(row["camera_guide_pose_timestamp_s"]) for row in values]
    cutoffs = [float(row["rendered_data_cutoff_s"]) for row in values]
    path = (
        "direct recorded camera_optical pose audit; no presentation camera applied"
        if spec.camera_motion_role == "recorded_camera_optical"
        else "continuous presentation orbit fitted to estimated map poses"
    )
    return {
        "role": spec.camera_motion_role,
        "path": path,
        "render_application": (
            "not_applied; trace audits recorded camera pose for the timestamped sensor replay"
            if spec.camera_motion_role == "recorded_camera_optical"
            else "applied to map-view projection"
        ),
        "pose_time_basis": CAMERA_TRACE_BASIS,
        "camera_guide_timestamp_window_s": list(spec.camera_guide_timestamp_window_s),
        "camera_pose_dependency_window_s": list(spec.camera_pose_dependency_window_s),
        "camera_guide_pose_timestamp_range_s": [min(timestamps), max(timestamps)],
        "rendered_data_cutoff_range_s": [min(cutoffs), max(cutoffs)],
        "trace": "technical_camera_trace.jsonl",
        "trace_global_frame_range": [int(values[0]["global_frame"]), int(values[-1]["global_frame"])],
        "trace_eye_target_sha256": _camera_trace_view_sha256(values),
        "final_hold_frames": spec.final_hold_frames,
        "map_focus": (
            None
            if spec.camera_motion_role == "recorded_camera_optical"
            else {
                "track_id": int(getattr(focus_inventory, "track_id")),
                "position_m": [float(value) for value in getattr(focus_inventory, "position")],
                "source": "first deterministic selected row in hash-bound estimated inventory",
            }
        ),
    }


@dataclass(frozen=True)
class InventoryRecord:
    track_id: int
    position: tuple[float, float, float]
    observations: int


@dataclass(frozen=True)
class SourceBundle:
    run_root: Path
    capture_id: str
    source_id: str
    presentation_classification: dict[str, str]
    capture_git_sha: str
    slam_git_sha: str
    perception_git_sha: str
    simulation_time_start_s: float
    simulation_time_end_s: float
    map_version: str
    trajectory_version: str
    object_state_version: str
    trajectory_time_basis: str
    depth_sources: tuple[str, ...]
    map_path: Path
    trajectory_path: Path
    inventory_path: Path
    capture_manifest_path: Path
    slam_manifest_path: Path
    perception_manifest_path: Path
    source_catalog_path: Path
    raw_lidar_bag_path: Path
    bag_metadata_path: Path
    camera_info_path: Path
    sensor_transforms_path: Path
    effective_config_path: Path
    scene_manifest_path: Path
    rgb_video_path: Path
    rgb_frames_path: Path
    annotations_path: Path
    compatibility_status: str
    delivery_eligible: bool
    hashes: dict[str, str]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_text_sha256(path: Path) -> str:
    """Hash repository text independently of the checkout line-ending policy."""

    text = path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def implementation_hashes(repo_root: Path) -> dict[str, str]:
    return {name: canonical_text_sha256(repo_root / name) for name in IMPLEMENTATION_SOURCES}


def _verify_bound_input_snapshot(bundle: SourceBundle) -> None:
    """Rehash every bound input after rendering before a manifest is published."""

    paths = {
        "map": bundle.map_path,
        "trajectory": bundle.trajectory_path,
        "inventory": bundle.inventory_path,
        "capture_manifest": bundle.capture_manifest_path,
        "slam_manifest": bundle.slam_manifest_path,
        "perception_manifest": bundle.perception_manifest_path,
        "raw_lidar_bag": bundle.raw_lidar_bag_path,
        "bag_metadata": bundle.bag_metadata_path,
        "camera_info": bundle.camera_info_path,
        "sensor_transforms": bundle.sensor_transforms_path,
        "effective_config": bundle.effective_config_path,
        "scene_manifest": bundle.scene_manifest_path,
        "rgb_video": bundle.rgb_video_path,
        "rgb_frames": bundle.rgb_frames_path,
        "frame_annotations": bundle.annotations_path,
    }
    changed = [name for name, path in paths.items() if sha256_file(path) != bundle.hashes[name]]
    if canonical_text_sha256(bundle.source_catalog_path) != bundle.hashes["source_catalog"]:
        changed.append("source_catalog")
    if changed:
        raise ValueError(f"technical render inputs changed during rendering: {sorted(changed)}")


def load_plan(path: str | Path) -> tuple[dict[str, RenderProfile], tuple[ViewSpec, ...]]:
    plan_path = Path(path)
    payload = json.loads(plan_path.read_text(encoding="utf-8"))
    fps = int(payload["fps"])
    profiles = {
        name: RenderProfile(
            name=name,
            width=int(values["width"]),
            height=int(values["height"]),
            fps=fps,
            crf=int(values["crf"]),
            preset=str(values["preset"]),
        )
        for name, values in payload["profiles"].items()
    }
    views = tuple(
        ViewSpec(
            **{
                **item,
                "source_window_s": tuple(float(value) for value in item["source_window_s"]),
                "display_window_s": tuple(float(value) for value in item["display_window_s"]),
                "camera_guide_timestamp_window_s": tuple(
                    float(value) for value in item["camera_guide_timestamp_window_s"]
                ),
                "camera_pose_dependency_window_s": tuple(
                    float(value) for value in item["camera_pose_dependency_window_s"]
                ),
            }
        )
        for item in payload["views"]
    )
    if tuple(view.id for view in views) != VIEW_ORDER:
        raise ValueError(f"technical view order must be {VIEW_ORDER}")
    if any(view.frames <= 0 for view in views):
        raise ValueError("every technical view needs a positive frame count")
    if profiles["preview"].width != 1280 or profiles["preview"].height != 720:
        raise ValueError("preview profile must be 1280x720")
    if profiles["delivery"].width != 1920 or profiles["delivery"].height != 1080:
        raise ValueError("delivery profile must be 1920x1080")
    if fps != 30:
        raise ValueError("technical source views must be 30 fps")
    if any(view.selective_current_scan_status != "complete" for view in views):
        raise ValueError("technical views must use the completed selective current-scan contract")
    allowed_modes = {
        "camera_occlusion_surfaces", "navigation_boundaries", "past_structural_history",
        "estimated_roi_front_surfaces",
    }
    allowed_temporal = {"current_window", "past_only_reveal", "snapshot_orbit", "past_only_orbit"}
    for index, view in enumerate(views):
        start_s, end_s = view.source_window_s
        display_start_s, display_end_s = view.display_window_s
        guide_start_s, guide_end_s = view.camera_guide_timestamp_window_s
        dependency_start_s, dependency_end_s = view.camera_pose_dependency_window_s
        if not (math.isfinite(start_s) and math.isfinite(end_s) and 0.0 <= start_s <= end_s):
            raise ValueError(f"technical view {view.id} has an invalid source window")
        if not (
            math.isfinite(display_start_s)
            and math.isfinite(display_end_s)
            and start_s <= display_start_s <= display_end_s <= end_s
        ):
            raise ValueError(f"technical view {view.id} has a display window outside its source dependencies")
        if not (
            math.isfinite(guide_start_s)
            and math.isfinite(guide_end_s)
            and 0.0 <= guide_start_s < guide_end_s
        ):
            raise ValueError(f"technical view {view.id} has an invalid camera guide timestamp window")
        if not (
            math.isfinite(dependency_start_s)
            and math.isfinite(dependency_end_s)
            and 0.0 <= dependency_start_s < dependency_end_s
            and dependency_start_s <= guide_start_s
            and guide_end_s <= dependency_end_s
        ):
            raise ValueError(f"technical view {view.id} has an invalid camera pose dependency window")
        if index and not math.isclose(
            views[index - 1].camera_guide_timestamp_window_s[1], guide_start_s, abs_tol=1e-9
        ):
            raise ValueError("technical camera guide timestamp windows must be contiguous")
        if view.selection_mode not in allowed_modes or view.temporal_mode not in allowed_temporal:
            raise ValueError(f"technical view {view.id} has an unsupported selection or temporal mode")
        if view.maximum_points <= 0 or view.history_stride_scans <= 0:
            raise ValueError(f"technical view {view.id} has invalid point bounds")
        if view.rgb_context and view.temporal_mode != "current_window":
            raise ValueError("RGB context is permitted only for a camera-aligned current window")
        expected_motion_role = "recorded_camera_optical" if index < 2 else "continuous_estimated_map_path"
        if view.camera_motion_role != expected_motion_role:
            raise ValueError(f"technical view {view.id} has an invalid camera motion role")
        expected_hold = 24 if index == len(views) - 1 else 0
        if view.final_hold_frames != expected_hold:
            raise ValueError(f"technical view {view.id} has an invalid final camera hold")
    return profiles, views


def _read_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _known_storyboard_hashes(manifest_path: Path) -> set[str]:
    payload = _read_json(manifest_path)
    hashes: set[str] = set()
    for item in payload.get("storyboards", payload.get("files", [])):
        if isinstance(item, dict):
            for key in ("sha256", "checksum"):
                value = item.get(key)
                if isinstance(value, str) and len(value) == 64:
                    hashes.add(value.lower())
    def walk(value: object) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if key.lower() == "sha256" and isinstance(child, str) and len(child) == 64:
                    hashes.add(child.lower())
                else:
                    walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)
    walk(payload)
    return hashes


def _catalog_source(catalog_path: Path, capture_id: str) -> dict[str, object]:
    catalog = _read_json(catalog_path)
    if catalog.get("schema_version") != 1 or catalog.get("status") != "reviewed_source_catalog":
        raise ValueError("technical source catalog is not a reviewed schema-v1 catalog")
    matches = [
        item for item in catalog.get("sources", [])
        if isinstance(item, dict) and item.get("capture_id") == capture_id
    ]
    if len(matches) != 1:
        raise ValueError(f"capture_id is not uniquely pinned in the reviewed technical source catalog: {capture_id}")
    return matches[0]


def _require_git_commit(repo_root: Path, revision: str, producer: str) -> None:
    if len(revision) != 40:
        raise ValueError(f"{producer} producer revision must be a full Git SHA")
    result = subprocess.run(
        ["git", "-C", str(repo_root), "cat-file", "-e", f"{revision}^{{commit}}"],
        capture_output=True,
    )
    if result.returncode != 0:
        raise ValueError(f"{producer} producer revision is not a repository commit: {revision}")


def _trajectory_time_range(path: Path) -> tuple[float, float]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    try:
        timestamps = [float(row["timestamp_s"]) for row in rows]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("trajectory CSV lacks finite timestamp_s values") from error
    if len(timestamps) < 2 or not all(math.isfinite(value) for value in timestamps) or any(
        later < earlier for earlier, later in zip(timestamps, timestamps[1:])
    ):
        raise ValueError("trajectory timestamps must be finite and ordered")
    return timestamps[0], timestamps[-1]


def _inventory_depth_sources(path: Path, allowed: set[str] | frozenset[str]) -> tuple[str, ...]:
    used: set[str] = set()
    localized_rows = 0
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"estimated_x_m", "estimated_y_m", "estimated_z_m", "3d_observation_count", "depth_source"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("estimated inventory lacks its depth-source contract")
        for row in reader:
            coordinates = [row.get(name) for name in ("estimated_x_m", "estimated_y_m", "estimated_z_m")]
            if not all(value not in (None, "") for value in coordinates):
                continue
            if int(row["3d_observation_count"]) <= 0:
                continue
            source = str(row.get("depth_source", ""))
            if source not in allowed:
                raise ValueError(f"estimated inventory uses an unapproved depth source: {source or '<missing>'}")
            localized_rows += 1
            used.add(source)
    if localized_rows == 0:
        raise ValueError("estimated inventory has no approved LiDAR-localized rows")
    return tuple(sorted(used))


def _inspect_source_bundle_with_catalog(
    run_root: str | Path,
    reference_manifest: str | Path,
    source_catalog: str | Path,
) -> SourceBundle:
    run = Path(run_root).resolve()
    catalog_path = Path(source_catalog).resolve()
    paths = {
        "capture_manifest": run / "capture" / "capture_manifest.json",
        "slam_manifest": run / "slam" / "slam_manifest.json",
        "perception_manifest": run / "perception" / "perception_manifest.json",
    }
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        raise ValueError(f"source run is missing required artifacts: {missing}")
    if any("storyboard" in part.lower() for path in paths.values() for part in path.parts):
        raise ValueError("storyboard paths cannot be technical render inputs")

    capture = _read_json(paths["capture_manifest"])
    slam = _read_json(paths["slam_manifest"])
    perception = _read_json(paths["perception_manifest"])
    if capture.get("status") != "complete" or slam.get("status") != "complete" or perception.get("status") != "complete":
        raise ValueError("capture, SLAM, and perception manifests must all be complete")
    capture_id = str(capture.get("capture_id", ""))
    if not capture_id or str(slam.get("capture_id", "")) != capture_id or run.name != capture_id:
        raise ValueError("run directory, capture manifest, and SLAM manifest capture_id must agree")
    source = _catalog_source(catalog_path, capture_id)
    if source.get("run_directory_name") != run.name:
        raise ValueError("run directory is not the catalog-pinned source identity")
    perception_capture_id = perception.get("capture_id")
    perception_contract = source.get("perception_contract")
    if not isinstance(perception_contract, dict):
        raise ValueError("catalog source lacks a perception contract")
    if perception_capture_id is None:
        if perception_contract.get("legacy_capture_id_omitted") is not True:
            raise ValueError("perception manifest omits capture_id without an exact reviewed legacy pin")
        legacy_perception = True
    elif str(perception_capture_id) != capture_id:
        raise ValueError("capture, SLAM, and perception capture_id must agree")
    else:
        legacy_perception = False
    source_artifacts = source.get("artifacts")
    if not isinstance(source_artifacts, dict):
        raise ValueError("catalog source lacks artifact bindings")
    for artifact_name, default_path in (
        ("map", "slam/slam_map.ply"),
        ("trajectory", "slam/slam_map_poses.csv"),
        ("inventory", "perception/estimated_inventory.csv"),
        ("raw_lidar_bag", "capture/sensors_bag/sensors_bag_0.db3"),
        ("bag_metadata", "capture/bag_metadata.json"),
        ("camera_info", "capture/camera_info.json"),
        ("sensor_transforms", "capture/sensor_transforms.json"),
        ("effective_config", "capture/effective_config.json"),
        ("scene_manifest", "capture/scene_manifest.json"),
        ("rgb_video", "capture/rgb_camera.mp4"),
        ("rgb_frames", "capture/rgb_frames.jsonl"),
        ("frame_annotations", "perception/frame_annotations.jsonl"),
    ):
        binding = source_artifacts.get(artifact_name)
        if not isinstance(binding, dict):
            raise ValueError(f"catalog source lacks {artifact_name} artifact binding")
        declared = str(binding.get("path", default_path))
        if "\\" in declared or Path(declared).is_absolute() or any(part in ("", ".", "..") for part in Path(declared).parts):
            raise ValueError(f"catalog {artifact_name} path is not canonical and relative")
        resolved = (run / declared).resolve()
        try:
            resolved.relative_to(run)
        except ValueError as error:
            raise ValueError(f"catalog {artifact_name} path escapes the reviewed run") from error
        paths[artifact_name] = resolved
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        raise ValueError(f"source run is missing required artifacts: {missing}")
    if any("storyboard" in part.lower() for path in paths.values() for part in path.parts):
        raise ValueError("storyboard paths cannot be technical render inputs")
    hashes = {name: sha256_file(path) for name, path in paths.items()}
    forbidden = _known_storyboard_hashes(Path(reference_manifest).resolve())
    collisions = sorted(name for name, digest in hashes.items() if digest in forbidden)
    if collisions:
        raise ValueError(f"storyboard content hashes cannot be technical render inputs: {collisions}")
    if str(slam.get("capture_sha256", "")) != str(capture.get("capture_sha256", "")):
        raise ValueError("SLAM manifest does not reference the capture manifest checksum")
    if bool(slam.get("ground_truth_subscribed")):
        raise ValueError("technical map input must not subscribe to ground truth")
    if bool(perception.get("ground_truth_consumed")) or not bool(perception.get("lidar_consumed_for_estimation")) or not bool(perception.get("slam_consumed_for_estimation")):
        raise ValueError("estimated inventory must be ground-truth-free and consume LiDAR plus SLAM")
    if perception.get("ground_truth_required") not in (False, None):
        raise ValueError("estimated inventory must not require ground truth")
    if Path(str(perception.get("estimated_inventory", ""))).name != paths["inventory"].name:
        raise ValueError("perception manifest does not name the estimated inventory input")
    capture_git = str(capture.get("git_sha", ""))
    slam_git = str(slam.get("git_sha", ""))
    perception_git = str(perception.get("git_sha", ""))
    revisions = source.get("producer_revisions")
    manifests = source.get("producer_manifests")
    artifacts = source.get("artifacts")
    if not all(isinstance(item, dict) for item in (revisions, manifests, artifacts)):
        raise ValueError("catalog source lacks producer and artifact bindings")
    observed_revisions = {"capture": capture_git, "slam": slam_git, "perception": perception_git}
    for producer, revision in observed_revisions.items():
        if revision != revisions.get(producer):
            raise ValueError(f"{producer} producer revision does not match the reviewed source catalog")
        _require_git_commit(Path(__file__).resolve().parents[1], revision, producer)
    manifest_keys = {"capture": "capture_manifest", "slam": "slam_manifest", "perception": "perception_manifest"}
    for producer, hash_key in manifest_keys.items():
        binding = manifests.get(producer)
        if not isinstance(binding, dict) or binding.get("path") != str(paths[hash_key].relative_to(run)).replace("\\", "/") or binding.get("sha256") != hashes[hash_key]:
            raise ValueError(f"{producer} manifest does not match the reviewed source catalog")
    for artifact_name in (
        "map", "trajectory", "inventory", "raw_lidar_bag", "bag_metadata", "camera_info",
        "sensor_transforms", "effective_config", "scene_manifest", "rgb_video", "rgb_frames",
        "frame_annotations",
    ):
        binding = artifacts.get(artifact_name)
        if not isinstance(binding, dict) or binding.get("path") != str(paths[artifact_name].relative_to(run)).replace("\\", "/") or binding.get("sha256") != hashes[artifact_name]:
            raise ValueError(f"{artifact_name} artifact does not match the reviewed source catalog")
    capture_files = {
        item.get("path"): item for item in capture.get("files", [])
        if isinstance(item, dict) and isinstance(item.get("path"), str)
    }
    for artifact_name in (
        "raw_lidar_bag", "bag_metadata", "camera_info", "sensor_transforms", "effective_config",
        "scene_manifest", "rgb_video", "rgb_frames",
    ):
        relative = paths[artifact_name].relative_to(run / "capture").as_posix()
        record = capture_files.get(relative)
        if (
            not isinstance(record, dict)
            or record.get("sha256") != hashes[artifact_name]
            or int(record.get("size_bytes", -1)) != paths[artifact_name].stat().st_size
        ):
            raise ValueError(f"capture manifest does not bind the exact {artifact_name} bytes")
    trajectory_relative = paths["trajectory"].relative_to(run / "slam").as_posix()
    trajectory_entries = [
        item for item in slam.get("artifacts", [])
        if isinstance(item, dict) and item.get("path") == trajectory_relative
    ]
    if not trajectory_entries:
        legacy_artifacts = slam.get("producer", {}).get("artifacts", {})
        legacy_entry = legacy_artifacts.get(trajectory_relative) if isinstance(legacy_artifacts, dict) else None
        if isinstance(legacy_entry, dict):
            trajectory_entries = [{
                **legacy_entry,
                "size_bytes": paths["trajectory"].stat().st_size,
            }]
    if len(trajectory_entries) != 1:
        raise ValueError("SLAM manifest does not uniquely bind the presentation trajectory")
    trajectory_entry = trajectory_entries[0]
    if trajectory_entry.get("sha256") != hashes["trajectory"] or int(trajectory_entry.get("size_bytes", -1)) != paths["trajectory"].stat().st_size:
        raise ValueError("SLAM trajectory bytes do not match the producer artifact binding")
    if trajectory_relative == "slam_map_poses.csv":
        expected = {
            "role": "dense_corrected_trajectory",
            "frame_id": "map",
            "optimized": True,
            "map_version": slam.get("map_version"),
            "dense_pose_version": slam.get("dense_pose_version"),
        }
        if any(trajectory_entry.get(key) != value for key, value in expected.items()):
            raise ValueError("repaired SLAM presentation trajectory lacks canonical map-frame provenance")
        observer = slam.get("observer")
        observer_binding = slam.get("observer_artifact")
        observer_path = paths["slam_manifest"].with_name("slam_observer.json")
        if not isinstance(observer, dict) or not isinstance(observer_binding, dict) or not observer_path.is_file():
            raise ValueError("repaired SLAM source lacks its bound observer record")
        if observer_binding.get("path") != observer_path.name or observer_binding.get("sha256") != sha256_file(observer_path) or int(observer_binding.get("size_bytes", -1)) != observer_path.stat().st_size:
            raise ValueError("repaired SLAM observer bytes do not match the producer binding")
        if observer != _read_json(observer_path):
            raise ValueError("embedded and file-backed repaired SLAM observer records differ")
    if source.get("capture_sha256") != capture.get("capture_sha256"):
        raise ValueError("capture checksum does not match the reviewed source catalog")
    if perception.get("estimated_inventory") != perception_contract.get("estimated_inventory"):
        raise ValueError("perception manifest inventory association does not match the reviewed source catalog")
    if legacy_perception:
        if perception.get("slam_artifact") != perception_contract.get("slam_artifact"):
            raise ValueError("perception manifest SLAM association does not match the reviewed source catalog")
    else:
        if perception_contract.get("legacy_capture_id_omitted") is True:
            raise ValueError("modern perception provenance cannot use the reviewed legacy omission waiver")
        if perception.get("slam_trajectory") != perception_contract.get("slam_trajectory"):
            raise ValueError("perception manifest trajectory association does not match the reviewed source catalog")
        annotations = run / "perception" / str(perception.get("annotations", ""))
        if not annotations.is_file():
            raise ValueError("perception annotations named by the producer manifest are missing")
        legacy_capture_v0 = perception_contract.get("legacy_capture_manifest_v0") is True
        if legacy_capture_v0:
            if source.get("presentation_compatibility", {}).get("status") != "historical_paired_diagnostic":
                raise ValueError("legacy capture-manifest provenance is restricted to historical diagnostics")
            inputs = perception.get("inputs", {})
            expected_legacy = {
                "capture_id": capture_id,
                "capture_sha256": capture.get("capture_sha256"),
                "capture_manifest_sha256": hashes["capture_manifest"],
                "raw_lidar_sha256": hashes["raw_lidar_bag"],
                "rgb_video_sha256": hashes["rgb_video"],
                "rgb_frames_sha256": hashes["rgb_frames"],
                "sensor_transforms_sha256": hashes["sensor_transforms"],
                "slam_manifest_sha256": hashes["slam_manifest"],
                "trajectory_sha256": hashes["trajectory"],
            }
            bag_files = inputs.get("raw_lidar", {}).get("bag", {}).get("files", [])
            raw_bag = next(
                (item for item in bag_files if isinstance(item, dict) and str(item.get("path", "")).endswith(".db3")),
                {},
            )
            observed_legacy = {
                "capture_id": inputs.get("capture", {}).get("capture_id"),
                "capture_sha256": inputs.get("capture", {}).get("capture_sha256"),
                "capture_manifest_sha256": inputs.get("capture", {}).get("manifest", {}).get("sha256"),
                "raw_lidar_sha256": raw_bag.get("sha256"),
                "rgb_video_sha256": inputs.get("rgb", {}).get("video", {}).get("sha256"),
                "rgb_frames_sha256": inputs.get("rgb", {}).get("frames", {}).get("sha256"),
                "sensor_transforms_sha256": inputs.get("rgb", {}).get("sensor_transforms", {}).get("sha256"),
                "slam_manifest_sha256": inputs.get("slam", {}).get("manifest", {}).get("sha256"),
                "trajectory_sha256": inputs.get("slam", {}).get("trajectory", {}).get("sha256"),
            }
            if observed_legacy != expected_legacy:
                raise ValueError("historical perception input hashes do not match the reviewed paired capture")
            rgb_count = sum(1 for line in paths["rgb_frames"].read_text(encoding="utf-8").splitlines() if line.strip())
            annotation_count = sum(1 for line in annotations.read_text(encoding="utf-8").splitlines() if line.strip())
            if rgb_count != annotation_count or perception.get("frame_count") != rgb_count:
                raise ValueError("historical perception annotations do not cover the paired RGB index")
        else:
            validate_perception_manifest_bindings(perception, run / "capture", run / "slam", run / "perception")
            validate_perception_frame_coverage(perception, run / "capture", annotations)
    allowed_depth_sources = frozenset(perception_contract.get("allowed_depth_sources", []))
    if not allowed_depth_sources or not allowed_depth_sources.issubset(ALLOWED_ESTIMATED_DEPTH_SOURCES):
        raise ValueError("catalog source has no supported estimated depth source")
    depth_sources = _inventory_depth_sources(paths["inventory"], allowed_depth_sources)
    start_s, end_s = _trajectory_time_range(paths["trajectory"])
    time_binding = source.get("simulation_time")
    expected_time_source = f"{paths['trajectory'].relative_to(run).as_posix()}:timestamp_s"
    if not isinstance(time_binding, dict) or time_binding.get("source") != expected_time_source:
        raise ValueError("catalog source lacks a trajectory simulation-time binding")
    if not math.isclose(start_s, float(time_binding.get("start_s", math.nan)), abs_tol=1e-9) or not math.isclose(end_s, float(time_binding.get("end_s", math.nan)), abs_tol=1e-9):
        raise ValueError("trajectory simulation-time range does not match the reviewed source catalog")
    hashes["source_catalog"] = canonical_text_sha256(catalog_path)
    compatibility = source.get("presentation_compatibility")
    if not isinstance(compatibility, dict):
        raise ValueError("catalog source lacks presentation compatibility status")
    compatibility_status = str(compatibility.get("status", ""))
    delivery_eligible = compatibility.get("delivery_eligible")
    if compatibility_status not in {
        "current_goal_paired_capture", "historical_paired_diagnostic", "generated_test_fixture",
    } or not isinstance(delivery_eligible, bool):
        raise ValueError("catalog source has an invalid presentation compatibility status")
    if compatibility_status == "historical_paired_diagnostic" and delivery_eligible:
        raise ValueError("historical diagnostic source cannot be delivery eligible")
    transforms_payload = _read_json(paths["sensor_transforms"])
    effective_payload = _read_json(paths["effective_config"])
    sensor_contract = source.get("sensor_contract")
    observed_sensor_contract = {
        "lidar_topic": transforms_payload.get("topics", {}).get("lidar_points"),
        "camera_info_topic": transforms_payload.get("topics", {}).get("rgb_camera_info"),
        "lidar_frame_id": transforms_payload.get("frames", {}).get("lidar_link"),
        "camera_frame_id": transforms_payload.get("frames", {}).get("camera_optical"),
        "configured_lidar_hz": effective_payload.get("lidar", {}).get("hz"),
        "maximum_current_age_periods": 2.0,
        "maximum_rgb_skew_ns": 17_000_001,
    }
    if sensor_contract != observed_sensor_contract:
        raise ValueError("capture sensor frames, topics, or cadence do not match the reviewed source catalog")
    return SourceBundle(
        run_root=run,
        capture_id=capture_id,
        source_id=str(source["source_id"]),
        presentation_classification=dict(source.get("presentation_classification", {"kind": "reviewed_production"})),
        capture_git_sha=capture_git,
        slam_git_sha=slam_git,
        perception_git_sha=perception_git,
        simulation_time_start_s=start_s,
        simulation_time_end_s=end_s,
        map_version=str(artifacts["map"]["version"]),
        trajectory_version=str(artifacts["trajectory"]["version"]),
        object_state_version=str(artifacts["inventory"]["version"]),
        trajectory_time_basis=expected_time_source,
        depth_sources=depth_sources,
        map_path=paths["map"],
        trajectory_path=paths["trajectory"],
        inventory_path=paths["inventory"],
        capture_manifest_path=paths["capture_manifest"],
        slam_manifest_path=paths["slam_manifest"],
        perception_manifest_path=paths["perception_manifest"],
        source_catalog_path=catalog_path,
        raw_lidar_bag_path=paths["raw_lidar_bag"],
        bag_metadata_path=paths["bag_metadata"],
        camera_info_path=paths["camera_info"],
        sensor_transforms_path=paths["sensor_transforms"],
        effective_config_path=paths["effective_config"],
        scene_manifest_path=paths["scene_manifest"],
        rgb_video_path=paths["rgb_video"],
        rgb_frames_path=paths["rgb_frames"],
        annotations_path=paths["frame_annotations"],
        compatibility_status=compatibility_status,
        delivery_eligible=delivery_eligible,
        hashes=hashes,
    )


def inspect_source_bundle(run_root: str | Path, reference_manifest: str | Path) -> SourceBundle:
    """Inspect a render source against the repository's reviewed immutable catalog."""

    catalog = Path(__file__).resolve().parents[1] / "config" / "technical_source_catalog.json"
    return _inspect_source_bundle_with_catalog(run_root, reference_manifest, catalog)


def load_ascii_ply(path: str | Path) -> np.ndarray:
    ply_path = Path(path)
    with ply_path.open("r", encoding="utf-8") as handle:
        first = handle.readline().strip()
        if first != "ply":
            raise ValueError("not a PLY file")
        vertex_count: int | None = None
        properties: list[str] = []
        in_vertices = False
        for raw in handle:
            line = raw.strip()
            if line == "format ascii 1.0":
                continue
            if line.startswith("format "):
                raise ValueError("only ASCII PLY 1.0 is supported")
            if line.startswith("element vertex "):
                vertex_count = int(line.split()[2])
                in_vertices = True
            elif line.startswith("element "):
                in_vertices = False
            elif line.startswith("property ") and in_vertices:
                properties.append(line.split()[-1])
            elif line == "end_header":
                break
        else:
            raise ValueError("PLY header has no end_header")
        if vertex_count is None or properties[:3] != ["x", "y", "z"]:
            raise ValueError("PLY must declare x/y/z vertex properties first")
        values = np.loadtxt(handle, dtype=np.float32, max_rows=vertex_count)
    values = np.atleast_2d(values)
    if len(values) != vertex_count or values.shape[1] < 3 or not np.isfinite(values[:, :3]).all():
        raise ValueError("PLY vertex data does not match its declared finite XYZ count")
    return values[:, :3]


def load_trajectory(path: str | Path) -> np.ndarray:
    positions: list[tuple[float, float, float]] = []
    timestamps: list[float] = []
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"timestamp_s", "x_m", "y_m", "z_m"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("trajectory CSV lacks timestamp_s/x_m/y_m/z_m")
        for row in reader:
            timestamps.append(float(row["timestamp_s"]))
            positions.append((float(row["x_m"]), float(row["y_m"]), float(row["z_m"])))
    result = np.asarray(positions, dtype=np.float32)
    if len(result) < 2 or not np.isfinite(result).all() or np.any(np.diff(timestamps) < 0):
        raise ValueError("trajectory must contain finite, time-ordered poses")
    return result


def load_inventory(path: str | Path) -> tuple[InventoryRecord, ...]:
    records: list[InventoryRecord] = []
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"track_id", "estimated_x_m", "estimated_y_m", "estimated_z_m", "3d_observation_count", "depth_source"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("estimated inventory CSV lacks the localization contract")
        for row in reader:
            if any(row[name] in (None, "") for name in ("estimated_x_m", "estimated_y_m", "estimated_z_m")):
                continue
            position = tuple(float(row[name]) for name in ("estimated_x_m", "estimated_y_m", "estimated_z_m"))
            observations = int(row["3d_observation_count"])
            if observations > 0 and all(math.isfinite(value) for value in position):
                if row["depth_source"] not in ALLOWED_ESTIMATED_DEPTH_SOURCES:
                    raise ValueError(f"estimated inventory uses an unapproved depth source: {row['depth_source']}")
                records.append(InventoryRecord(int(row["track_id"]), position, observations))
    if not records:
        raise ValueError("estimated inventory has no LiDAR-localized records")
    return tuple(records)


def select_inventory(records: Iterable[InventoryRecord], limit: int = 28, spacing_m: float = 0.22) -> tuple[InventoryRecord, ...]:
    selected: list[InventoryRecord] = []
    for record in sorted(records, key=lambda item: (-item.observations, item.track_id)):
        point = np.asarray(record.position)
        if any(np.linalg.norm(point - np.asarray(other.position)) < spacing_m for other in selected):
            continue
        selected.append(record)
        if len(selected) == limit:
            break
    return tuple(selected)


def _look_at(eye: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray([0.0, 0.0, 1.0], dtype=np.float32))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return np.stack([right, up, forward])


def project_points(points: np.ndarray, eye: np.ndarray, target: np.ndarray, width: int, height: int, focal_scale: float = 0.78) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    camera = (points - eye) @ _look_at(eye, target).T
    depth = camera[:, 2]
    valid = depth > 0.08
    focal = min(width, height) * focal_scale
    safe_depth = np.where(valid, depth, 1.0)
    u = np.rint(width * 0.5 + focal * camera[:, 0] / safe_depth).astype(np.int32)
    v = np.rint(height * 0.5 - focal * camera[:, 1] / safe_depth).astype(np.int32)
    valid &= (u >= 0) & (u < width) & (v >= 0) & (v < height)
    return np.stack([u[valid], v[valid]], axis=1), depth[valid], valid


def fit_font(text: str, font_path: Path, preferred_size: int, minimum_size: int, max_width: int) -> ImageFont.FreeTypeFont:
    """Return the largest font in bounds for a single-line UI label."""

    for size in range(preferred_size, minimum_size - 1, -1):
        font = ImageFont.truetype(str(font_path), size)
        left, _top, right, _bottom = font.getbbox(text)
        if right - left <= max_width:
            return font
    raise ValueError(f"text cannot fit its UI bounds at the minimum font size: {text}")


class TechnicalRenderer:
    def __init__(
        self,
        profile: RenderProfile,
        source: SelectiveLidarSource,
        inventory: tuple[InventoryRecord, ...],
        views: tuple[ViewSpec, ...],
    ):
        self.profile = profile
        self.source = source
        self.views = views
        self.trajectory = source.trajectory.positions_m.astype(np.float32)
        self.inventory = select_inventory(inventory)
        self.camera_focus_inventory = self.inventory[0]
        self._used_scans: dict[str, dict[int, PreparedScan]] = {}
        self._used_rois: dict[str, tuple[RoiObservation, ...]] = {}
        self._rgb_pairs: dict[str, set[tuple[int, int, int]]] = {}
        self._history_views: dict[str, tuple[tuple[PreparedScan, ...], np.ndarray, np.ndarray]] = {}
        roi_specs = tuple(view for view in views if view.selection_mode == "estimated_roi_front_surfaces")
        roi_pools: dict[tuple[float, float], tuple[RoiObservation, ...]] = {}
        self._planned_rois: dict[str, tuple[RoiObservation, ...]] = {}
        for spec in roi_specs:
            key = spec.display_window_s
            if key not in roi_pools:
                roi_pools[key] = source.roi_observations(*key, 64)
            if spec.id == "object_detail":
                matching = tuple(
                    item for item in roi_pools[key]
                    if item.detection.track_id == self.camera_focus_inventory.track_id
                )
                if not matching:
                    raise ValueError(
                        "deterministic camera focus inventory track has no LiDAR-supported ROI observation"
                    )
                self._planned_rois[spec.id] = matching[:1]
            else:
                self._planned_rois[spec.id] = roi_pools[key]
        if not roi_specs or not self._planned_rois.get("object_detail"):
            raise ValueError("technical camera motion requires one fixed LiDAR-supported ROI")
        roi_focus = np.asarray(self.camera_focus_inventory.position, dtype=np.float64)
        rig_from_optical = source.rig_from_lidar @ np.linalg.inv(source.optical_from_lidar)
        self.camera_path = TechnicalCameraPath(views, source.trajectory, rig_from_optical, roi_focus)
        self._camera_trace_rows = tuple(self.camera_path.trace_rows(profile.fps))
        self._camera_trace_by_view = {
            spec.id: tuple(row for row in self._camera_trace_rows if row["view_id"] == spec.id)
            for spec in views
        }
        scale = profile.height / 720.0
        self.fonts = {
            "kicker": ImageFont.truetype(str(FONT_BOLD), max(13, int(15 * scale))),
            "title": ImageFont.truetype(str(FONT_BOLD), max(30, int(42 * scale))),
            "body": ImageFont.truetype(str(FONT_REGULAR), max(14, int(18 * scale))),
            "mono": ImageFont.truetype(str(Path("C:/Windows/Fonts/consola.ttf")), max(12, int(15 * scale))),
        }

    def _background(self) -> np.ndarray:
        height, width = self.profile.height, self.profile.width
        y = np.linspace(0.0, 1.0, height, dtype=np.float32)[:, None, None]
        top = np.asarray([12, 20, 32], dtype=np.float32)
        bottom = np.asarray([2, 6, 12], dtype=np.float32)
        return np.broadcast_to(top * (1.0 - y) + bottom * y, (height, width, 3)).copy().astype(np.uint8)

    def _register_scan(self, view_id: str, scan: PreparedScan) -> None:
        self._used_scans.setdefault(view_id, {})[scan.record.timestamp_ns] = scan

    def _draw_camera_points(self, frame: np.ndarray, scan: PreparedScan, weight: float, color: tuple[int, int, int]) -> None:
        if weight <= 0.0 or not len(scan.raw_point_indices):
            return
        scale_x = self.profile.width / self.source.intrinsics.width_px
        scale_y = self.profile.height / self.source.intrinsics.height_px
        pixels = np.column_stack((np.rint(scan.u_px * scale_x), np.rint(scan.v_px * scale_y))).astype(np.int32)
        valid = (
            (pixels[:, 0] >= 0) & (pixels[:, 0] < self.profile.width)
            & (pixels[:, 1] >= 0) & (pixels[:, 1] < self.profile.height)
        )
        pixels = pixels[valid]
        depths = scan.depth_m[valid]
        layer = np.zeros_like(frame)
        brightness = np.clip(1.15 - depths / 20.0, 0.42, 1.0) * weight
        values = np.clip(np.asarray(color)[None, :] * brightness[:, None], 0, 255).astype(np.uint8)
        order = np.argsort(depths)[::-1]
        xy = pixels[order]
        layer[xy[:, 1], xy[:, 0]] = values[order]
        radius = 1 if self.profile.height <= 720 else 2
        layer = cv2.dilate(layer, np.ones((radius + 1, radius + 1), dtype=np.uint8))
        glow = cv2.GaussianBlur(layer, (0, 0), 2.4)
        np.maximum(frame, (glow * 0.28).astype(np.uint8), out=frame)
        np.maximum(frame, layer, out=frame)

    def _paired_rgb_background(self, spec: ViewSpec, scan: PreparedScan) -> np.ndarray:
        rgb = self.source.rgb_at(scan.record.timestamp_s)
        skew_ns = int(round(abs(rgb.timestamp_s - scan.record.timestamp_s) * 1_000_000_000))
        if skew_ns > 17_000_001:
            raise ValueError("LiDAR scan has no co-timed RGB frame within half an RGB interval")
        self._rgb_pairs.setdefault(spec.id, set()).add((scan.record.timestamp_ns, rgb.frame_index, skew_ns))
        frame = self.source.read_rgb_frame(rgb, self.profile.width, self.profile.height)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cool = cv2.applyColorMap(gray, cv2.COLORMAP_OCEAN)
        return cv2.addWeighted(frame, 0.24, cool, 0.25, -7.0)

    def _draw_sensor_rays(self, frame: np.ndarray, scan: PreparedScan, progress: float, weight: float) -> None:
        if not len(scan.raw_point_indices) or weight <= 0.0:
            return
        scale_x = self.profile.width / self.source.intrinsics.width_px
        scale_y = self.profile.height / self.source.intrinsics.height_px
        pixels = np.column_stack((np.rint(scan.u_px * scale_x), np.rint(scan.v_px * scale_y))).astype(np.int32)
        candidates = np.flatnonzero((scan.depth_m >= 1.5) & (scan.depth_m <= 10.0))
        if not len(candidates):
            return
        count = min(12, len(candidates))
        chosen = candidates[np.linspace(0, len(candidates) - 1, count, dtype=np.int32)]
        opacities = _activation_ray_opacities(progress, count)
        origin = (self.profile.width // 2, int(self.profile.height * 0.90))
        ray_layer = np.zeros_like(frame)
        for point_index, opacity in zip(chosen, opacities):
            if opacity <= 0.0:
                continue
            endpoint = tuple(int(value) for value in pixels[point_index])
            line_color = tuple(int(value * float(opacity)) for value in (238, 194, 56))
            point_color = tuple(int(value * float(opacity)) for value in (250, 229, 101))
            cv2.line(ray_layer, origin, endpoint, line_color, max(1, self.profile.height // 720), cv2.LINE_AA)
            cv2.circle(ray_layer, endpoint, max(2, self.profile.height // 420), point_color, -1, cv2.LINE_AA)
        np.maximum(frame, (ray_layer * (0.18 + 0.30 * weight)).astype(np.uint8), out=frame)
        scale = self.profile.height / 720.0
        housing = np.asarray([
            (origin[0] - int(34 * scale), origin[1] + int(19 * scale)),
            (origin[0] + int(34 * scale), origin[1] + int(19 * scale)),
            (origin[0] + int(21 * scale), origin[1] - int(12 * scale)),
            (origin[0] - int(21 * scale), origin[1] - int(12 * scale)),
        ], dtype=np.int32)
        cv2.fillConvexPoly(frame, housing, (7, 18, 28), cv2.LINE_AA)
        cv2.polylines(frame, [housing], True, (68, 210, 235), max(1, int(2 * scale)), cv2.LINE_AA)

    def _current_frame(self, spec: ViewSpec, index: int, progress: float) -> tuple[np.ndarray, float, int]:
        start_s, end_s = spec.display_window_s
        timestamp_s = start_s + (end_s - start_s) * (index / spec.frames)
        previous_record, latest_record, alpha = self.source.causal_scan_pair(timestamp_s)
        if previous_record.timestamp_s < start_s - 1e-9:
            previous_record = latest_record
            alpha = 1.0
        scans = [
            self.source.prepare_scan(previous_record, spec.selection_mode, spec.maximum_points),
            self.source.prepare_scan(latest_record, spec.selection_mode, spec.maximum_points),
        ]
        for scan in scans:
            self._register_scan(spec.id, scan)
        composites: list[np.ndarray] = []
        for scan in scans:
            base = self._paired_rgb_background(spec, scan) if spec.rgb_context else self._background()
            self._draw_camera_points(base, scan, 1.0, (247, 211, 63))
            composites.append(base)
        frame = cv2.addWeighted(composites[0], 1.0 - alpha, composites[1], alpha, 0.0)
        if spec.id == "sensor_activation":
            self._draw_sensor_rays(frame, scans[0], progress, 1.0 - alpha)
            self._draw_sensor_rays(frame, scans[1], progress, alpha)
        return frame, latest_record.timestamp_s, len(scans[0].raw_point_indices) + len(scans[1].raw_point_indices)

    def _draw_map_points(
        self, frame: np.ndarray, points: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float,
        color: tuple[int, int, int], dim: float = 1.0, point_radius: int | None = None,
    ) -> None:
        if not len(points):
            return
        pixels, depth, _mask = project_points(points, eye, target, self.profile.width, self.profile.height, focal)
        if not len(pixels):
            return
        layer = np.zeros_like(frame)
        fade = np.clip(1.15 - depth / 36.0, 0.30, 1.0) * dim
        colors = np.clip(np.asarray(color)[None, :] * fade[:, None], 0, 255).astype(np.uint8)
        order = np.argsort(depth)[::-1]
        xy = pixels[order]
        layer[xy[:, 1], xy[:, 0]] = colors[order]
        radius = point_radius if point_radius is not None else (1 if self.profile.height <= 720 else 2)
        layer = cv2.dilate(layer, np.ones((radius + 1, radius + 1), dtype=np.uint8))
        glow = cv2.GaussianBlur(layer, (0, 0), 2.5)
        np.maximum(frame, (glow * 0.25).astype(np.uint8), out=frame)
        np.maximum(frame, layer, out=frame)

    def _draw_path(self, frame: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float, cutoff_s: float) -> None:
        keep = self.source.trajectory.timestamps_s <= cutoff_s + 1e-9
        path = self.trajectory[keep] + np.asarray([0, 0, 0.06], dtype=np.float32)
        if len(path) < 2:
            return
        pixels, _depth, mask = project_points(path, eye, target, self.profile.width, self.profile.height, focal)
        if mask.sum() >= 2:
            cv2.polylines(frame, [pixels.reshape(-1, 1, 2)], False, (60, 224, 255), max(2, self.profile.height // 360), cv2.LINE_AA)
            cv2.circle(frame, tuple(pixels[-1]), max(5, self.profile.height // 100), (68, 241, 255), -1, cv2.LINE_AA)

    def _map_frame(self, spec: ViewSpec, index: int, progress: float) -> tuple[np.ndarray, float, int, list[tuple[int, tuple[int, int], int]]]:
        source_start_s, source_end_s = spec.source_window_s
        start_s, end_s = spec.display_window_s
        eased = progress * progress * (3.0 - 2.0 * progress)
        cutoff_s = start_s + (end_s - start_s) * eased if spec.temporal_mode == "past_only_reveal" else end_s
        prepared = self._history_views.get(spec.id)
        if prepared is None:
            per_scan_limit = max(400, min(2600, spec.maximum_points // 20))
            scans = self.source.history_scans(source_start_s, source_end_s, spec.history_stride_scans, per_scan_limit)
            if not scans:
                raise ValueError(f"technical view {spec.id} has no past LiDAR history in its source window")
            all_points = np.concatenate([scan.map_xyz_m for scan in scans], axis=0)
            offsets = np.cumsum([len(scan.map_xyz_m) for scan in scans], dtype=np.int64)
            prepared = (scans, all_points, offsets)
            self._history_views[spec.id] = prepared
        history, all_points, offsets = prepared
        scan_count = int(np.searchsorted([scan.record.timestamp_s for scan in history], cutoff_s, side="right"))
        for scan in history[:scan_count]:
            self._register_scan(spec.id, scan)
        point_end = int(offsets[scan_count - 1]) if scan_count else 0
        history_points = all_points[:point_end]
        rois: tuple[RoiObservation, ...] = ()
        if spec.selection_mode == "estimated_roi_front_surfaces":
            rois = self._planned_rois[spec.id]
            self._used_rois[spec.id] = rois
            focus = np.median(rois[0].map_xyz_m, axis=0)
            radius_m = 3.2 if spec.id == "object_detail" else 5.2
            delta = history_points - focus[None, :]
            local = (np.linalg.norm(delta[:, :2], axis=1) <= radius_m) & (np.abs(delta[:, 2]) <= 2.4)
            history_points = history_points[local]
        else:
            focus = np.asarray([np.median(self.trajectory[:, 0]), 0.0, 0.8], dtype=np.float32)
        stable_stride = max(1, int(math.ceil(len(history_points) / spec.maximum_points)))
        history_points = history_points[::stable_stride][:spec.maximum_points]
        camera = self.camera_path.state(spec, index)
        eye = camera.eye_m.astype(np.float32)
        target = camera.target_m.astype(np.float32)
        focal = 0.76 if spec.selection_mode == "estimated_roi_front_surfaces" else 0.68
        frame = self._background()
        context_dim = 0.16 if spec.id == "object_detail" else (0.30 if rois else 0.68)
        self._draw_map_points(frame, history_points, eye, target, focal, (132, 94, 40), context_dim)
        callouts: list[tuple[int, tuple[int, int], int]] = []
        if rois:
            for roi_index, roi in enumerate(rois):
                roi_color = (48, 238, 255) if roi_index else (70, 255, 180)
                self._draw_map_points(frame, roi.map_xyz_m, eye, target, focal, roi_color, 1.0, 3)
                center = np.mean(roi.map_xyz_m, axis=0, keepdims=True)
                pixels, _depth, mask = project_points(center, eye, target, self.profile.width, self.profile.height, focal)
                if mask.sum():
                    pixel = tuple(int(value) for value in pixels[0])
                    cv2.drawMarker(frame, pixel, (80, 235, 175), cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
                    callouts.append((roi.detection.track_id, pixel, len(roi.raw_point_indices)))
        self._draw_path(frame, eye, target, focal, cutoff_s)
        return frame, cutoff_s, len(history_points) + sum(len(item.map_xyz_m) for item in rois), callouts

    def _typography(self, frame: np.ndarray, spec: ViewSpec, progress: float, timestamp_s: float, point_count: int, callouts: list[tuple[int, tuple[int, int], int]]) -> np.ndarray:
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image, "RGBA")
        width, height = image.size
        scale = height / 720.0
        pad = int(42 * scale)
        draw.rounded_rectangle((pad, pad, min(width - pad, int(650 * scale)), int(162 * scale)), radius=int(12 * scale), fill=(5, 13, 24, 214), outline=(51, 211, 241, 155), width=max(1, int(1 * scale)))
        draw.text((pad + int(22 * scale), pad + int(14 * scale)), "SIMULATED CAPTURE · SELECTIVE REAL LIDAR", font=self.fonts["kicker"], fill=(96, 222, 241, 255))
        draw.text((pad + int(22 * scale), pad + int(42 * scale)), spec.title, font=self.fonts["title"], fill=(238, 247, 250, 255))
        draw.text((pad + int(22 * scale), pad + int(94 * scale)), spec.subtitle, font=self.fonts["body"], fill=(160, 185, 199, 255))
        frame_label = "CAMERA OPTICAL FRAME" if spec.temporal_mode == "current_window" else "ESTIMATED MAP FRAME"
        footer_parts = [frame_label, f"{point_count:,} SELECTED RETURNS", "RIGID HEADER-STAMP SCANS", "NO DESKEW"]
        if spec.temporal_mode != "current_window":
            footer_parts.append(f"{len(self.trajectory)} ESTIMATED POSES")
        footer = " · ".join(footer_parts) + f" · source t={timestamp_s:0.2f}s"
        draw.text((pad, height - pad - int(21 * scale)), footer, font=self.fonts["mono"], fill=(125, 162, 178, 235))
        for index, (track_id, (x, y), support_count) in enumerate(callouts[:3]):
            box_x = width - int(350 * scale)
            box_y = int((84 + index * 86) * scale)
            draw.line((x, y, box_x - int(12 * scale), box_y + int(25 * scale)), fill=(70, 222, 241, 185), width=max(1, int(2 * scale)))
            draw.rounded_rectangle((box_x, box_y, width - pad, box_y + int(67 * scale)), radius=int(8 * scale), fill=(5, 14, 25, 224), outline=(62, 217, 239, 170), width=max(1, int(1 * scale)))
            draw.text((box_x + int(14 * scale), box_y + int(8 * scale)), f"ESTIMATED ROI · TRACK {track_id:05d}", font=self.fonts["kicker"], fill=(102, 233, 246, 255))
            draw.text((box_x + int(14 * scale), box_y + int(32 * scale)), f"{support_count} nearest-surface raw returns", font=self.fonts["mono"], fill=(210, 226, 232, 255))
        if spec.id == "object_detail" and callouts:
            track_id, _pixel, support_count = callouts[0]
            card_x = width - int(520 * scale)
            card_y = height - int(260 * scale)
            draw.rounded_rectangle((card_x, card_y, width - pad, height - int(62 * scale)), radius=int(12 * scale), fill=(4, 12, 22, 232), outline=(75, 230, 248, 210), width=max(1, int(2 * scale)))
            text_x = card_x + int(20 * scale)
            text_right = width - pad - int(20 * scale)
            text_width = text_right - text_x
            title = f"Persistent track {track_id}"
            title_font = fit_font(title, FONT_BOLD, max(30, int(42 * scale)), max(22, int(28 * scale)), text_width)
            observation = f"Front-surface support · {support_count} raw returns"
            observation_font = fit_font(observation, FONT_REGULAR, max(14, int(18 * scale)), max(12, int(14 * scale)), text_width)
            limitation = "Class unknown · Full extent not estimated"
            limitation_font = fit_font(limitation, FONT_REGULAR, max(14, int(18 * scale)), max(12, int(14 * scale)), text_width)
            draw.text((text_x, card_y + int(18 * scale)), "SELECTED ESTIMATED ROI", font=self.fonts["kicker"], fill=(87, 229, 245, 255))
            draw.text((text_x, card_y + int(52 * scale)), title, font=title_font, fill=(241, 247, 249, 255))
            draw.text((text_x, card_y + int(108 * scale)), observation, font=observation_font, fill=(188, 207, 216, 255))
            draw.text((text_x, card_y + int(144 * scale)), limitation, font=limitation_font, fill=(244, 188, 91, 255))
        if spec.id == "final_technical_view":
            panel_x = int(width * 0.64)
            draw.rectangle((panel_x, 0, width, height), fill=(3, 9, 17, 242))
            line_x = panel_x + int(42 * scale)
            draw.text((line_x, int(174 * scale)), "MAPPED FROM", font=self.fonts["kicker"], fill=(91, 225, 244, 255))
            draw.text((line_x, int(214 * scale)), "SIMULATED\nSENSOR DATA.", font=self.fonts["title"], fill=(242, 247, 249, 255), spacing=int(7 * scale))
            draw.rectangle((line_x, int(344 * scale), line_x + int(90 * scale), int(348 * scale)), fill=(72, 224, 242, 255))
            draw.text((line_x, int(378 * scale)), "PAST-ONLY LIDAR HISTORY\nESTIMATED TRAJECTORY\nSELECTIVE RAW RETURNS", font=self.fonts["body"], fill=(174, 197, 207, 255), spacing=int(9 * scale))
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    def frame(self, spec: ViewSpec, index: int) -> np.ndarray:
        progress = 0.0 if spec.frames == 1 else index / (spec.frames - 1)
        if spec.temporal_mode == "current_window":
            frame, timestamp_s, point_count = self._current_frame(spec, index, progress)
            callouts: list[tuple[int, tuple[int, int], int]] = []
        else:
            frame, timestamp_s, point_count, callouts = self._map_frame(spec, index, progress)
        return self._typography(frame, spec, progress, timestamp_s, point_count, callouts)

    def camera_trace_rows(self) -> list[dict[str, object]]:
        return list(self._camera_trace_rows)

    def derivation(self, spec: ViewSpec) -> dict[str, object]:
        scans = tuple(self._used_scans.get(spec.id, {}).values())
        scans = tuple(sorted(scans, key=lambda item: item.record.timestamp_ns))
        rgb_pairs = sorted(self._rgb_pairs.get(spec.id, set()))
        result: dict[str, object] = {
            "point_projection": (
                "recorded lidar_link through recorded calibration into camera_optical_frame"
                if spec.temporal_mode == "current_window"
                else "recorded lidar_link through interpolated estimated sensor-rig pose into map"
            ),
            "selection_mode": spec.selection_mode,
            "temporal_mode": spec.temporal_mode,
            "source_window_s": list(spec.source_window_s),
            "display_window_s": list(spec.display_window_s),
            "future_returns_consumed": False,
            "causal_display_policy": "latest and previous scans only; maximum current age is 2 configured scan periods",
            "storyboard_pixels_consumed": False,
            "simulator_truth_consumed": False,
            "scene_or_asset_metadata_consumed": False,
            "selective_current_scan_status": "complete",
            "rgb_context": spec.rgb_context,
            "rendered_context_point_budget": spec.maximum_points,
            "context_treatment": (
                "local sparse structural silhouette around the estimated ROI"
                if spec.selection_mode == "estimated_roi_front_surfaces"
                else "bounded structural or current-return subset"
            ),
            "scan_time_model": {
                "per_return_timing": "absent_in_bound_PointCloud2_fields",
                "deskew": "not_applied",
                "rigid_pose_time": "PointCloud2 header/database timestamp",
            },
            "camera_calibration": self.source.camera_calibration_receipt,
            "camera_motion": camera_motion_receipt(
                spec, self._camera_trace_by_view[spec.id], self.camera_focus_inventory
            ),
        }
        history_summary = self.source.scan_summary(scans)
        if spec.id in self._used_rois:
            result["scan_selection"] = self.source.roi_scan_summary(self._used_rois[spec.id])
            result["context_scan_selection"] = history_summary
        else:
            result["scan_selection"] = history_summary
        if rgb_pairs:
            result["cotimed_rgb_pairs"] = {
                "pair_count": len(rgb_pairs),
                "maximum_absolute_skew_ns": max(item[2] for item in rgb_pairs),
                "pairs": [
                    {"scan_timestamp_ns": item[0], "rgb_frame_index": item[1], "absolute_skew_ns": item[2]}
                    for item in rgb_pairs
                ],
            }
        if spec.id in self._used_rois:
            result["estimated_roi_selection"] = self.source.roi_summary(self._used_rois[spec.id])
        if spec.id == "sensor_activation":
            result["sensor_ray_overlay"] = (
                "screen-space explanatory origin marker with endpoints from selected raw returns; "
                "the marker does not claim literal visible laser emission"
            )
        return result


class FfmpegWriter:
    def __init__(self, output: Path, profile: RenderProfile, ffmpeg: str):
        output.parent.mkdir(parents=True, exist_ok=True)
        self.output = output
        command = [
            ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{profile.width}x{profile.height}",
            "-r", str(profile.fps), "-i", "-", "-an", "-c:v", "libx264",
            "-preset", profile.preset, "-crf", str(profile.crf), "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(output),
        ]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def write(self, frame: np.ndarray) -> None:
        if self.process.stdin is None:
            raise RuntimeError("ffmpeg stdin is unavailable")
        self.process.stdin.write(frame.tobytes())

    def close(self) -> None:
        if self.process.stdin is not None:
            self.process.stdin.close()
            self.process.stdin = None
        stderr = self.process.stderr.read().decode("utf-8", errors="replace") if self.process.stderr else ""
        code = self.process.wait()
        if code != 0:
            self.output.unlink(missing_ok=True)
            raise RuntimeError(f"ffmpeg failed with exit code {code}: {stderr}")


def _contact_sheet(frames: list[tuple[str, int, np.ndarray]], output: Path) -> None:
    thumb_w, thumb_h = 480, 270
    rows = math.ceil(len(frames) / 3)
    canvas = Image.new("RGB", (thumb_w * 3, thumb_h * rows), (3, 7, 12))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype(str(FONT_BOLD), 15)
    for cell, (view_id, index, frame) in enumerate(frames):
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((thumb_w, thumb_h), Image.Resampling.LANCZOS)
        x, y = (cell % 3) * thumb_w, (cell // 3) * thumb_h
        canvas.paste(image, (x, y))
        draw.rectangle((x + 250, y + 10, x + 470, y + 40), fill=(4, 12, 20, 220))
        draw.text((x + 258, y + 15), f"{view_id} · {index:03d}", font=font, fill=(215, 239, 245))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, optimize=True)


def probe_video(path: str | Path, ffprobe: str) -> dict[str, object]:
    result = subprocess.run(
        [
            ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,r_frame_rate,nb_read_frames", "-of", "json", str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    streams = json.loads(result.stdout).get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"expected one video stream in {path}")
    stream = streams[0]
    numerator, denominator = (int(value) for value in str(stream["r_frame_rate"]).split("/"))
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps": numerator / denominator,
        "frame_count": int(stream["nb_read_frames"]),
    }


def _validate_probe(probe: dict[str, object], profile: RenderProfile, expected_frames: int) -> None:
    expected = (profile.width, profile.height, float(profile.fps), expected_frames)
    actual = (probe["width"], probe["height"], probe["fps"], probe["frame_count"])
    if actual != expected:
        raise ValueError(f"encoded video contract mismatch: expected {expected}, observed {actual}")


def _source_receipt(bundle: SourceBundle) -> dict[str, object]:
    transforms_payload = _read_json(bundle.sensor_transforms_path)
    transform_frames = transforms_payload.get("frames", {})
    transform_topics = transforms_payload.get("topics", {})
    effective_payload = _read_json(bundle.effective_config_path)
    lidar_frame_id = str(transform_frames.get("lidar_link", ""))
    camera_frame_id = str(transform_frames.get("camera_optical", ""))
    if not lidar_frame_id or not camera_frame_id:
        raise ValueError("recorded transform graph lacks LiDAR or camera optical frame identity")
    return {
        "source_id": bundle.source_id,
        "capture_id": bundle.capture_id,
        "presentation_classification": bundle.presentation_classification,
        "producer_revisions": {
            "capture": bundle.capture_git_sha,
            "slam": bundle.slam_git_sha,
            "perception": bundle.perception_git_sha,
        },
        "simulation_time": {
            "basis": bundle.trajectory_time_basis,
            "start_s": bundle.simulation_time_start_s,
            "end_s": bundle.simulation_time_end_s,
        },
        "map_state": {
            "version": bundle.map_version,
            "sha256": bundle.hashes["map"],
            "producer_manifest_sha256": bundle.hashes["slam_manifest"],
            "producer_revision": bundle.slam_git_sha,
        },
        "trajectory_state": {
            "version": bundle.trajectory_version,
            "sha256": bundle.hashes["trajectory"],
            "producer_manifest_sha256": bundle.hashes["slam_manifest"],
            "producer_revision": bundle.slam_git_sha,
        },
        "object_state": {
            "version": bundle.object_state_version,
            "sha256": bundle.hashes["inventory"],
            "producer_manifest_sha256": bundle.hashes["perception_manifest"],
            "producer_revision": bundle.perception_git_sha,
            "depth_sources": list(bundle.depth_sources),
            "ground_truth_consumed": False,
        },
        "paired_capture_state": {
            "compatibility_status": bundle.compatibility_status,
            "delivery_eligible": bundle.delivery_eligible,
            "raw_lidar_bag_sha256": bundle.hashes["raw_lidar_bag"],
            "rgb_video_sha256": bundle.hashes["rgb_video"],
            "rgb_frames_sha256": bundle.hashes["rgb_frames"],
            "camera_info_sha256": bundle.hashes["camera_info"],
            "sensor_transforms_sha256": bundle.hashes["sensor_transforms"],
            "scene_manifest_sha256": bundle.hashes["scene_manifest"],
            "effective_config_sha256": bundle.hashes["effective_config"],
            "trajectory_sha256": bundle.hashes["trajectory"],
            "capture_manifest_sha256": bundle.hashes["capture_manifest"],
            "lidar_frame_id": lidar_frame_id,
            "camera_frame_id": camera_frame_id,
            "lidar_topic": transform_topics.get("lidar_points"),
            "camera_info_topic": transform_topics.get("rgb_camera_info"),
            "configured_lidar_hz": effective_payload.get("lidar", {}).get("hz"),
            "maximum_current_age_periods": 2.0,
            "maximum_rgb_skew_ns": 17_000_001,
        },
        "artifacts": {
            "map": {"sha256": bundle.hashes["map"], "semantics": "finalized offline LiDAR-SLAM point cloud"},
            "trajectory": {"sha256": bundle.hashes["trajectory"], "semantics": "estimated SLAM trajectory in map frame"},
            "inventory": {"sha256": bundle.hashes["inventory"], "semantics": "ground-truth-free LiDAR-localized estimated centers; no estimated extents"},
            "capture_manifest": {"sha256": bundle.hashes["capture_manifest"]},
            "slam_manifest": {"sha256": bundle.hashes["slam_manifest"]},
            "perception_manifest": {"sha256": bundle.hashes["perception_manifest"]},
            "source_catalog": {"sha256": bundle.hashes["source_catalog"]},
            "raw_lidar_bag": {"sha256": bundle.hashes["raw_lidar_bag"], "semantics": "recorded CDR PointCloud2 messages"},
            "bag_metadata": {"sha256": bundle.hashes["bag_metadata"]},
            "camera_info": {"sha256": bundle.hashes["camera_info"]},
            "sensor_transforms": {"sha256": bundle.hashes["sensor_transforms"]},
            "effective_config": {"sha256": bundle.hashes["effective_config"]},
            "scene_manifest": {"sha256": bundle.hashes["scene_manifest"]},
            "rgb_video": {"sha256": bundle.hashes["rgb_video"], "semantics": "recorded RGB from the same capture"},
            "rgb_frames": {"sha256": bundle.hashes["rgb_frames"]},
            "frame_annotations": {"sha256": bundle.hashes["frame_annotations"], "semantics": "ground-truth-free estimated RGB regions"},
        },
    }


def _write_view_receipt(
    output: Path,
    video_path: Path,
    video_hash: str,
    probe: dict[str, object],
    spec: ViewSpec,
    bundle: SourceBundle,
    renderer_hash: str,
    dependency_hashes: dict[str, str],
    plan_hash: str,
    derivation: dict[str, object],
) -> Path:
    receipt = {
        "schema_version": 2,
        "artifact_type": "technical_source_view_receipt",
        "status": "complete",
        "producer": {
            "id": PRODUCER_ID,
            "renderer_sha256": renderer_hash,
            "implementation_sha256": dependency_hashes,
            "plan_sha256": plan_hash,
        },
        "view_id": spec.id,
        "presentation_role": spec.presentation_role,
        "source": _source_receipt(bundle),
        "video": {"path": video_path.name, "sha256": video_hash, **probe},
        "derivation": {
            **derivation,
            "trajectory_overlay": spec.temporal_mode != "current_window",
            "estimated_extents_rendered": False,
            "capability_focus": spec.capability_focus,
            "point_source": spec.point_source,
        },
        "presentation_integration": {
            "adapter_required": True,
            "note": "validated technical source view; presentation consumes the bound video without reinterpreting sensor provenance",
        },
    }
    receipt_path = output / f"{spec.id}_receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return receipt_path


def render(profile: RenderProfile, views: tuple[ViewSpec, ...], bundle: SourceBundle, output_dir: str | Path, ffmpeg: str, ffprobe: str, plan_path: Path) -> dict[str, object]:
    from simulator.technical_lidar import SelectiveLidarSource

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if profile.name == "delivery" and not bundle.delivery_eligible:
        raise ValueError(
            "technical delivery requires a current-goal paired RGB/LiDAR capture; "
            f"source is {bundle.compatibility_status}"
        )
    inventory_all = load_inventory(bundle.inventory_path)
    lidar_source = SelectiveLidarSource(
        database_path=bundle.raw_lidar_bag_path,
        trajectory_path=bundle.trajectory_path,
        camera_info_path=bundle.camera_info_path,
        sensor_transforms_path=bundle.sensor_transforms_path,
        effective_config_path=bundle.effective_config_path,
        rgb_video_path=bundle.rgb_video_path,
        rgb_frames_path=bundle.rgb_frames_path,
        annotations_path=bundle.annotations_path,
        ffmpeg=ffmpeg,
    )
    renderer = TechnicalRenderer(profile, lidar_source, inventory_all, views)
    renderer_hash = canonical_text_sha256(Path(__file__).resolve())
    dependency_hashes = implementation_hashes(Path(__file__).resolve().parents[1])
    plan_hash = canonical_text_sha256(plan_path.resolve())
    started = time.perf_counter()
    outputs: list[dict[str, object]] = []
    samples: list[tuple[str, int, np.ndarray]] = []
    if profile.name == "preview":
        video_path = output / "technical_views_preview_720p.mp4"
        writer = FfmpegWriter(video_path, profile, ffmpeg)
        total_frames = 0
        try:
            for spec in views:
                sample_indices = {0, spec.frames // 2, spec.frames - 1}
                for index in range(spec.frames):
                    frame = renderer.frame(spec, index)
                    writer.write(frame)
                    total_frames += 1
                    if index in sample_indices:
                        samples.append((spec.id, index, frame.copy()))
        finally:
            writer.close()
        probe = probe_video(video_path, ffprobe)
        _validate_probe(probe, profile, total_frames)
        outputs.append({"path": video_path.name, "sha256": sha256_file(video_path), "frames": total_frames, "view_ids": [view.id for view in views], "probe": probe})
        frame_dir = output / "representative_frames"
        frame_dir.mkdir(exist_ok=True)
        representative_frames = []
        for view_id, index, frame in samples:
            frame_path = frame_dir / f"{view_id}_{index:04d}.png"
            cv2.imwrite(str(frame_path), frame)
            representative_frames.append({"path": str(frame_path.relative_to(output)).replace("\\", "/"), "sha256": sha256_file(frame_path)})
        _contact_sheet(samples, output / "technical_views_contact_sheet.png")
    else:
        for spec in views:
            video_path = output / f"{spec.id}_1080p.mp4"
            writer = FfmpegWriter(video_path, profile, ffmpeg)
            try:
                for index in range(spec.frames):
                    writer.write(renderer.frame(spec, index))
            finally:
                writer.close()
            video_hash = sha256_file(video_path)
            probe = probe_video(video_path, ffprobe)
            _validate_probe(probe, profile, spec.frames)
            receipt_path = _write_view_receipt(
                output, video_path, video_hash, probe, spec, bundle, renderer_hash, dependency_hashes,
                plan_hash,
                renderer.derivation(spec),
            )
            outputs.append({
                "path": video_path.name,
                "sha256": video_hash,
                "frames": spec.frames,
                "view_ids": [spec.id],
                "presentation_role": spec.presentation_role,
                "probe": probe,
                "receipt": {"path": receipt_path.name, "sha256": sha256_file(receipt_path)},
            })
    trace_path = output / "technical_camera_trace.jsonl"
    with trace_path.open("w", encoding="utf-8", newline="\n") as handle:
        for row in renderer.camera_trace_rows():
            handle.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    _verify_bound_input_snapshot(bundle)
    if canonical_text_sha256(Path(__file__).resolve()) != renderer_hash:
        raise ValueError("technical renderer changed during rendering")
    if implementation_hashes(Path(__file__).resolve().parents[1]) != dependency_hashes:
        raise ValueError("technical LiDAR implementation changed during rendering")
    if canonical_text_sha256(plan_path.resolve()) != plan_hash:
        raise ValueError("technical render plan changed during rendering")
    elapsed = time.perf_counter() - started
    manifest: dict[str, object] = {
        "schema_version": 2,
        "status": "complete",
        "producer": PRODUCER_ID,
        "profile": profile.name,
        "width": profile.width,
        "height": profile.height,
        "fps": profile.fps,
        "ordered_views": [view.id for view in views],
        "view_frame_counts": {view.id: view.frames for view in views},
        "total_frames": sum(view.frames for view in views),
        "capture_id": bundle.capture_id,
        "capture_git_sha": bundle.capture_git_sha,
        "slam_git_sha": bundle.slam_git_sha,
        "perception_git_sha": bundle.perception_git_sha,
        "simulation_time": {
            "start_s": bundle.simulation_time_start_s,
            "end_s": bundle.simulation_time_end_s,
        },
        "map_version": bundle.map_version,
        "trajectory_version": bundle.trajectory_version,
        "object_state_version": bundle.object_state_version,
        "source": _source_receipt(bundle),
        "renderer": {"path": str(Path(__file__).resolve()), "sha256": renderer_hash},
        "implementation_sha256": dependency_hashes,
        "plan": {"path": str(plan_path.resolve()), "sha256": plan_hash},
        "render_graph_sources": [
            "raw_lidar_bag", "rgb_video", "rgb_frames", "camera_info", "sensor_transforms",
            "trajectory", "inventory", "frame_annotations",
        ],
        "selective_current_scan_goal": {
            "status": "complete",
            "representation": "feature-specific selections from timestamped raw PointCloud2 returns",
            "spatial_registration": "recorded camera/LiDAR calibration and estimated map-frame pose interpolation",
            "temporal_policy": "current scans or bounded past-only scan history; future scans are rejected",
            "ground_truth_consumed": False,
        },
        "storyboard_content_used": False,
        "storyboard_exclusion_basis": "all render inputs are enumerated and hashed; storyboard paths and known reference hashes are rejected before decode",
        "selected_inventory_centers": len(renderer.inventory),
        "localized_inventory_rows": len(inventory_all),
        "raw_lidar_scan_count": len(lidar_source.scan_records),
        "trajectory_pose_count": len(lidar_source.trajectory.timestamps_s),
        "input_snapshot_verification": "post_render_sha256_match",
        "camera_motion_trace": {
            "path": trace_path.name,
            "sha256": sha256_file(trace_path),
            "frame_count": sum(view.frames for view in views),
            "basis": CAMERA_TRACE_BASIS,
        },
        "camera_motion_anchor": {
            "track_id": renderer.camera_focus_inventory.track_id,
            "position_m": list(renderer.camera_focus_inventory.position),
            "source": "first deterministic selected row in hash-bound estimated inventory",
        },
        "view_derivations": {view.id: renderer.derivation(view) for view in views},
        "outputs": outputs,
        "elapsed_wall_s": round(elapsed, 3),
        "frames_per_wall_s": round(sum(item["frames"] for item in outputs) / max(elapsed, 1e-6), 3),
    }
    if profile.name == "preview":
        contact = output / "technical_views_contact_sheet.png"
        manifest["contact_sheet"] = {"path": contact.name, "sha256": sha256_file(contact)}
        manifest["representative_frames"] = representative_frames
    manifest_path = output / f"technical_views_{profile.name}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lidar_source.close()
    return manifest


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", required=True, help="coherent run containing capture/, slam/, and perception/")
    parser.add_argument("--profile", choices=("preview", "delivery"), required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--plan", default=str(root / "config" / "technical_views.json"))
    parser.add_argument("--reference-manifest", default=str(root / "references" / "manifest.json"))
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg") or "ffmpeg")
    parser.add_argument("--ffprobe", default=shutil.which("ffprobe") or "ffprobe")
    args = parser.parse_args()
    profiles, views = load_plan(args.plan)
    bundle = inspect_source_bundle(args.run_root, args.reference_manifest)
    result = render(profiles[args.profile], views, bundle, args.output_dir, args.ffmpeg, args.ffprobe, Path(args.plan))
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
