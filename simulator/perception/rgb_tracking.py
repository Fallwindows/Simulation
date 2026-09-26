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
import hashlib
import heapq
import json
import math
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from evaluation.metrics import PoseSample, interpolate_pose, safe_quaternion
from simulator.perception.provenance import (
    LIDAR_PROJECTED_DEPTH_SOURCE,
    build_perception_input_bindings,
    validate_perception_manifest_bindings,
)
from simulator.sensors.scan_projection import resolve_transform
from simulator.technical_lidar import load_camera_head_transform_artifact

LIDAR_POINT_MAX_DECIMATION_FACTOR = 8


@dataclass(frozen=True)
class Detection:
    bbox_xyxy: tuple[int, int, int, int]
    center_px: tuple[float, float]
    area_px: int
    mean_hue: float
    mean_saturation: float
    confidence: float
    image_size_px: tuple[int, int] | None = None


@dataclass
class Track:
    track_id: int
    first_frame_index: int
    last_frame_index: int
    detection_count: int = 0
    missed_frames: int = 0
    center_px: tuple[float, float] = (0.0, 0.0)
    velocity_px: tuple[float, float] = (0.0, 0.0)
    last_observed_center_px: tuple[float, float] | None = None
    association_velocity_px: tuple[float, float] = (0.0, 0.0)
    lifecycle_state: str = "active"
    area_px: float = 0.0
    mean_hue: float = 0.0
    mean_saturation: float = 0.0
    image_size_px: tuple[int, int] | None = None
    detections: list[dict[str, object]] = field(default_factory=list)

    def update(self, frame_index: int, detection: Detection) -> None:
        previous = self.last_observed_center_px or self.center_px
        alpha = 0.65
        elapsed_frames = max(1, int(frame_index) - self.last_frame_index)
        measured_velocity = (
            (detection.center_px[0] - previous[0]) / elapsed_frames,
            (detection.center_px[1] - previous[1]) / elapsed_frames,
        )
        self.velocity_px = (
            0.7 * self.velocity_px[0] + 0.3 * measured_velocity[0],
            0.7 * self.velocity_px[1] + 0.3 * measured_velocity[1],
        )
        self.association_velocity_px = measured_velocity
        self.last_observed_center_px = detection.center_px
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
        self.lifecycle_state = "active"
        self.image_size_px = detection.image_size_px or self.image_size_px
        self.detections.append(_detection_record(self.track_id, detection))


@dataclass(frozen=True)
class RetiredTrackSummary:
    """Small export record retained after a track leaves association scans."""

    track_id: int
    first_frame_index: int
    last_frame_index: int
    detection_count: int
    center_px: tuple[float, float]


def _detection_record(track_id: int, detection: Detection) -> dict[str, object]:
    x0, y0, x1, y1 = detection.bbox_xyxy
    return {
        "track_id": track_id,
        "raw_track_id": track_id,
        "persistent_track_id": None,
        "canonical_track_id": None,
        "id_namespace": "raw_rgb",
        "bbox_xyxy": [x0, y0, x1, y1],
        "center_px": [round(detection.center_px[0], 3), round(detection.center_px[1], 3)],
        "area_px": detection.area_px,
        "confidence": round(detection.confidence, 4),
        "image_size_px": list(detection.image_size_px) if detection.image_size_px else None,
        "class": "unknown_product",
        "source": "rgb_color_connected_component",
    }


def _resolution_scaled_component_limits(width: int, height: int) -> tuple[float, float, float]:
    area_scale = (float(width) * float(height)) / (1280.0 * 720.0)
    # Keep the minimum evidence threshold tied to the reference pixel count,
    # while allowing large coherent product faces.  Width/height limits below
    # still reject aisle-scale regions; a low fixed max-area cap discarded
    # hero products solely because the input resolution was higher.
    image_area = float(width) * float(height)
    return 70.0 * area_scale, 0.14 * image_area, 5.0 * math.sqrt(area_scale)


def _budget_detections_spatially(
    detections: list[Detection], width: int, height: int, limit: int
) -> list[Detection]:
    """Keep a deterministic, image-wide sample when proposal count is capped."""

    if limit <= 0:
        return []
    columns, rows = 8, 4
    buckets: dict[tuple[int, int], list[Detection]] = {}
    for detection in detections:
        x = min(columns - 1, max(0, int(detection.center_px[0] * columns / max(1, width))))
        y = min(rows - 1, max(0, int(detection.center_px[1] * rows / max(1, height))))
        buckets.setdefault((y, x), []).append(detection)
    for bucket in buckets.values():
        bucket.sort(key=lambda item: (-item.confidence, -item.area_px, item.center_px[1], item.center_px[0]))

    selected: list[Detection] = []
    cells = sorted(buckets)
    depth = 0
    while len(selected) < limit:
        found_at_depth = False
        for cell in cells:
            bucket = buckets[cell]
            if depth < len(bucket):
                selected.append(bucket[depth])
                found_at_depth = True
                if len(selected) == limit:
                    break
        if not found_at_depth:
            break
        depth += 1
    return sorted(selected, key=lambda item: (item.center_px[1], item.center_px[0]))


def detect_product_blobs(
    frame_bgr: np.ndarray,
    max_detections: int = 160,
    diagnostics: dict[str, object] | None = None,
) -> list[Detection]:
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
    # Thresholds are defined at a 1280x720 reference image and scale with
    # pixel area so capture resolution does not change proposal coverage.
    min_area, max_area, min_extent = _resolution_scaled_component_limits(width, height)
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area < min_area or area > max_area or w < min_extent or h < min_extent:
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
        detections.append(Detection(
            (x, y, x + w - 1, y + h - 1), (cx, cy), area, mean_hue, mean_sat, confidence,
            image_size_px=(width, height),
        ))
    detections.sort(key=lambda item: (item.center_px[1], item.center_px[0]))
    if max_detections < 0:
        raise ValueError("max_detections must be nonnegative")
    selected = _budget_detections_spatially(detections, width, height, max_detections)
    if diagnostics is not None:
        diagnostics.update({
            "eligible_proposal_count": len(detections),
            "returned_proposal_count": len(selected),
            "budget_rejected_count": len(detections) - len(selected),
            "budget_limit": max_detections,
            "budget_policy": "round_robin_8x4_image_grid_by_confidence_then_area",
        })
    return selected


class BlobTracker:
    def __init__(
        self,
        max_match_distance_px: float = 85.0,
        max_missed_frames: int = 8,
        max_revisit_frames: int = 90,
    ):
        self.max_match_distance_px = float(max_match_distance_px)
        self.max_missed_frames = int(max_missed_frames)
        self.max_revisit_frames = max(int(max_revisit_frames), self.max_missed_frames)
        self.next_track_id = 1
        self.tracks: dict[int, Track] = {}
        self.retired_tracks: dict[int, RetiredTrackSummary] = {}

    def track_for_export(self, track_id: int) -> Track | RetiredTrackSummary | None:
        return self.tracks.get(track_id) or self.retired_tracks.get(track_id)

    def update(self, frame_index: int, detections: Iterable[Detection]) -> list[dict[str, object]]:
        detections = list(detections)
        candidate_costs: dict[tuple[int, int], float] = {}
        for track_id, track in self.tracks.items():
            elapsed_frames = max(1, frame_index - track.last_frame_index)
            track.missed_frames = max(0, frame_index - track.last_frame_index)
            track.lifecycle_state = "active" if track.missed_frames == 0 else (
                "occluded" if track.missed_frames <= self.max_missed_frames else "archived"
            )
            if track.lifecycle_state == "archived":
                track.association_velocity_px = (0.0, 0.0)
            if elapsed_frames - 1 > self.max_revisit_frames:
                continue
            if track.lifecycle_state == "archived":
                # Motion measured before archival is not evidence that an
                # object continued moving while it was unobserved.
                predicted = track.last_observed_center_px or track.center_px
            else:
                predicted = (
                    (track.last_observed_center_px or track.center_px)[0] + track.association_velocity_px[0] * elapsed_frames,
                    (track.last_observed_center_px or track.center_px)[1] + track.association_velocity_px[1] * elapsed_frames,
                )
            for detection_index, detection in enumerate(detections):
                image_size = detection.image_size_px or track.image_size_px or (1280, 720)
                distance_scale = math.sqrt((float(image_size[0]) * float(image_size[1])) / (1280.0 * 720.0))
                distance = math.hypot(predicted[0] - detection.center_px[0], predicted[1] - detection.center_px[1])
                area_ratio = max(detection.area_px, 1) / max(track.area_px, 1.0)
                hue_distance = abs(detection.mean_hue - track.mean_hue)
                hue_distance = min(hue_distance, 180.0 - hue_distance)
                cost = distance + 10.0 * abs(math.log(area_ratio)) + 0.65 * hue_distance
                association_gate = self.max_match_distance_px * distance_scale
                if track.lifecycle_state == "archived":
                    # Archived image-only tracks have no camera-motion or
                    # instance cue.  Use a deliberately tight spatial gate
                    # for re-identification; larger view shifts start new
                    # raw IDs instead of silently transferring an old one.
                    association_gate = min(association_gate, 20.0 * distance_scale)
                if distance <= association_gate and area_ratio < 6.0 and area_ratio > (1.0 / 6.0):
                    candidate_costs[(track_id, detection_index)] = cost
        assignments = _maximum_cardinality_minimum_cost_assignment(
            {track_id for track_id, _ in candidate_costs}, len(detections), candidate_costs
        )
        assigned_detections: set[int] = set()
        for track_id, detection_index in assignments:
            self.tracks[track_id].update(frame_index, detections[detection_index])
            assigned_detections.add(detection_index)

        # Expire old history so a later unrelated object cannot inherit an
        # identity from an arbitrarily old frame.
        expired_ids = [
            track_id for track_id, track in self.tracks.items()
            if frame_index - track.last_frame_index - 1 > self.max_revisit_frames
        ]
        for track_id in expired_ids:
            track = self.tracks.pop(track_id)
            self.retired_tracks[track_id] = RetiredTrackSummary(
                track_id=track.track_id,
                first_frame_index=track.first_frame_index,
                last_frame_index=track.last_frame_index,
                detection_count=track.detection_count,
                center_px=track.center_px,
            )
        for detection_index, detection in enumerate(detections):
            if detection_index in assigned_detections:
                continue
            track_id = self.next_track_id
            self.next_track_id += 1
            # Initialize from the first measurement.  Calling update here
            # would blend it with the default origin and invent velocity.
            track = Track(
                track_id,
                frame_index,
                frame_index,
                detection_count=1,
                center_px=detection.center_px,
                area_px=float(detection.area_px),
                mean_hue=detection.mean_hue,
                mean_saturation=detection.mean_saturation,
                last_observed_center_px=detection.center_px,
                image_size_px=detection.image_size_px,
                detections=[_detection_record(track_id, detection)],
            )
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


def _map_estimates_to_start_relative(
    map_estimates: dict[int, np.ndarray],
    start_position_world_m: np.ndarray,
    start_orientation_xyzw: tuple[float, float, float, float],
) -> dict[int, np.ndarray]:
    """Transform canonical map-frame estimates through one declared start pose."""

    start_t = np.asarray(start_position_world_m, dtype=np.float64)
    start_r = _quat_to_matrix(start_orientation_xyzw)
    return {
        int(track_id): (np.asarray(estimate_map, dtype=np.float64) - start_t) @ start_r
        for track_id, estimate_map in map_estimates.items()
    }


def _verify_corrected_pose_artifact_contract(path: Path) -> dict[str, object]:
    """Verify both runtime manifests and their corrected/raw pose artifacts."""

    if path.name != "slam_map_poses.csv":
        raise ValueError("Corrected pose loader requires the canonical slam_map_poses.csv artifact")
    observer_path = path.parent / "slam_observer.json"
    slam_manifest_path = path.parent / "slam_manifest.json"
    try:
        manifest_bytes = slam_manifest_path.read_bytes()
        slam_manifest = json.loads(manifest_bytes)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("Corrected pose stream is missing a valid authoritative SLAM manifest") from exc
    observer_artifact = slam_manifest.get("observer_artifact")
    if not isinstance(observer_artifact, dict) or observer_artifact.get("path") != "slam_observer.json":
        raise ValueError("Authoritative SLAM manifest is missing its observer artifact record")
    try:
        observer_bytes = observer_path.read_bytes()
    except OSError as exc:
        raise ValueError("Authoritative SLAM manifest references a missing observer artifact") from exc
    if (
        not isinstance(observer_artifact.get("size_bytes"), int)
        or isinstance(observer_artifact.get("size_bytes"), bool)
        or not isinstance(observer_artifact.get("sha256"), str)
        or len(observer_bytes) != observer_artifact["size_bytes"]
        or hashlib.sha256(observer_bytes).hexdigest() != observer_artifact["sha256"]
    ):
        raise ValueError("slam_observer.json size or SHA-256 does not match the authoritative SLAM manifest")
    try:
        observer = json.loads(observer_bytes)
    except json.JSONDecodeError as exc:
        raise ValueError("Authoritative SLAM manifest references an invalid observer artifact") from exc
    embedded_observer = slam_manifest.get("observer")
    if not isinstance(observer, dict) or not isinstance(embedded_observer, dict):
        raise ValueError("SLAM manifest does not embed the observer integrity record")
    if observer != embedded_observer:
        raise ValueError("On-disk and embedded SLAM observer records disagree")
    if slam_manifest.get("status") != "complete":
        raise ValueError("Top-level SLAM manifest is not complete")
    if observer.get("status") != "complete" or embedded_observer.get("status") != "complete":
        raise ValueError("Corrected pose stream integrity manifests are not complete")
    if (
        slam_manifest.get("ground_truth_subscribed") is not False
        or observer.get("ground_truth_subscribed") is not False
        or embedded_observer.get("ground_truth_subscribed") is not False
    ):
        raise ValueError("Perception cannot consume truth-subscribed SLAM pose provenance")

    version_fields = (
        "pose_source", "graph_pose_version", "pre_publish_graph_version",
        "pre_publish_source_graph_identity", "final_source_graph_identity",
        "source_graph_identity_matches_pre_publish",
        "pre_publish_optimized_pose_version", "final_optimized_pose_version",
        "pre_publish_map_to_odom_version", "final_map_to_odom_version",
        "pre_publish_source_graph_link_count", "final_source_graph_link_count",
        "pre_publish_source_graph_link_type_histogram", "final_source_graph_link_type_histogram",
        "map_graph_matches_final_cloud", "optimized_pose_graph_complete",
        "map_data_graph_fingerprint", "map_graph_fingerprint", "map_data_matches_map_graph",
        "cached_cloud_graph_fingerprint", "final_cloud_graph_fingerprint",
        "map_cloud_identity_state", "final_cloud_origin",
        "final_map_graph_stamp_s", "final_cloud_stamp_s",
        "final_map_graph_frame_id", "final_cloud_frame_id",
        "map_pose_frame_id", "map_pose_sample_count", "dense_pose_version", "map_version",
        "ground_truth_subscribed",
    )
    if any(observer.get(key) != embedded_observer.get(key) for key in version_fields):
        raise ValueError("Observer and SLAM manifest disagree on optimized map pose version")
    if any(slam_manifest.get(key) != observer.get(key) for key in (
        "pre_publish_graph_version", "graph_pose_version", "dense_pose_version", "map_version",
        "pre_publish_source_graph_identity", "final_source_graph_identity",
        "source_graph_identity_matches_pre_publish", "map_cloud_identity_state", "final_cloud_origin",
        "pre_publish_optimized_pose_version", "final_optimized_pose_version",
        "pre_publish_map_to_odom_version", "final_map_to_odom_version",
        "pre_publish_source_graph_link_count", "final_source_graph_link_count",
        "pre_publish_source_graph_link_type_histogram", "final_source_graph_link_type_histogram",
        "map_data_graph_fingerprint", "map_graph_fingerprint", "map_data_matches_map_graph",
        "cached_cloud_graph_fingerprint", "final_cloud_graph_fingerprint", "map_graph_matches_final_cloud",
        "ground_truth_subscribed",
    )):
        raise ValueError("Authoritative SLAM manifest disagrees with the observer pose version")
    sample_count = observer.get("map_pose_sample_count")
    if not isinstance(sample_count, int) or isinstance(sample_count, bool) or sample_count <= 0:
        raise ValueError("map_pose_sample_count must be a positive integer")
    if observer.get("pose_source") != "rtabmap_optimized_graph":
        raise ValueError("Corrected map poses are not sourced from the finalized optimized graph")
    if (
        observer.get("map_graph_matches_final_cloud") is not True
        or observer.get("map_data_matches_map_graph") is not True
        or observer.get("source_graph_identity_matches_pre_publish") is not True
        or observer.get("optimized_pose_graph_complete") is not True
    ):
        raise ValueError("Optimized map pose graph is incomplete or does not match the final map")
    if observer.get("map_pose_frame_id") != "map":
        raise ValueError("Optimized map pose stream does not declare frame_id=map")
    graph_version = observer.get("graph_pose_version")
    if not isinstance(graph_version, str) or len(graph_version) != 64 or any(char not in "0123456789abcdef" for char in graph_version):
        raise ValueError("Optimized map pose stream has an invalid graph_pose_version digest")
    pre_publish_version = observer.get("pre_publish_graph_version")
    if not isinstance(pre_publish_version, str) or len(pre_publish_version) != 64 or any(char not in "0123456789abcdef" for char in pre_publish_version):
        raise ValueError("Optimized map pose stream has an invalid pre_publish_graph_version digest")
    pre_publish_source_identity = observer.get("pre_publish_source_graph_identity")
    final_source_identity = observer.get("final_source_graph_identity")
    for label, digest in (
        ("pre_publish_source_graph_identity", pre_publish_source_identity),
        ("final_source_graph_identity", final_source_identity),
        ("map_data_graph_fingerprint", observer.get("map_data_graph_fingerprint")),
        ("map_graph_fingerprint", observer.get("map_graph_fingerprint")),
        ("final_cloud_graph_fingerprint", observer.get("final_cloud_graph_fingerprint")),
        ("pre_publish_optimized_pose_version", observer.get("pre_publish_optimized_pose_version")),
        ("final_optimized_pose_version", observer.get("final_optimized_pose_version")),
        ("pre_publish_map_to_odom_version", observer.get("pre_publish_map_to_odom_version")),
        ("final_map_to_odom_version", observer.get("final_map_to_odom_version")),
    ):
        if not isinstance(digest, str) or len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise ValueError(f"Optimized map pose stream has an invalid {label} digest")
    if pre_publish_source_identity != final_source_identity:
        raise ValueError("GetMap and PublishMap source graph identities disagree")
    if (
        not isinstance(observer.get("pre_publish_source_graph_link_count"), int)
        or isinstance(observer.get("pre_publish_source_graph_link_count"), bool)
        or observer.get("pre_publish_source_graph_link_count") != observer.get("final_source_graph_link_count")
        or not isinstance(observer.get("pre_publish_source_graph_link_type_histogram"), dict)
        or observer.get("pre_publish_source_graph_link_type_histogram") != observer.get("final_source_graph_link_type_histogram")
    ):
        raise ValueError("GetMap and PublishMap source graph link metadata disagree")
    graph_fingerprint = observer.get("map_graph_fingerprint")
    if observer.get("map_data_graph_fingerprint") != graph_fingerprint or observer.get("final_cloud_graph_fingerprint") != graph_fingerprint:
        raise ValueError("Final /mapData, /mapGraph, and cloud graph identities disagree")
    cloud_identity_state = observer.get("map_cloud_identity_state")
    if cloud_identity_state == "fresh_shared_publication":
        if observer.get("final_cloud_origin") != "fresh_post_publish" or observer.get("final_map_graph_stamp_s") != observer.get("final_cloud_stamp_s"):
            raise ValueError("Fresh final cloud does not share the post-publish graph stamp")
    elif cloud_identity_state == "cached_exact_graph_reuse":
        cached_fingerprint = observer.get("cached_cloud_graph_fingerprint")
        if cached_fingerprint != graph_fingerprint or observer.get("final_cloud_origin") != "cached_pre_publish":
            raise ValueError("Cached final cloud was not paired with the identical graph fingerprint")
    else:
        raise ValueError("Final map cloud has an unsupported graph identity state")
    if observer.get("final_map_graph_frame_id") != "map" or observer.get("final_cloud_frame_id") != "map":
        raise ValueError("Final graph/cloud cohort does not use the map frame")
    dense_pose_version = observer.get("dense_pose_version")
    map_version = observer.get("map_version")
    for label, digest in (("dense_pose_version", dense_pose_version), ("map_version", map_version)):
        if not isinstance(digest, str) or len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
            raise ValueError(f"Optimized map pose stream has an invalid {label} digest")
    if slam_manifest.get("map_frame_id") != "map" or slam_manifest.get("optimized") is not True:
        raise ValueError("Authoritative SLAM manifest does not certify a complete optimized map frame")
    odom_sample_count = observer.get("odom_sample_count")
    if not isinstance(odom_sample_count, int) or isinstance(odom_sample_count, bool) or odom_sample_count <= 0:
        raise ValueError("odom_sample_count must be a positive integer")
    if sample_count != odom_sample_count:
        raise ValueError("map_pose_sample_count and odom_sample_count disagree")
    if "odom_sample_count" in slam_manifest:
        top_odom_sample_count = slam_manifest["odom_sample_count"]
        if (
            not isinstance(top_odom_sample_count, int)
            or isinstance(top_odom_sample_count, bool)
            or top_odom_sample_count != odom_sample_count
        ):
            raise ValueError("Top-level odom_sample_count disagrees with the observer")

    files = observer.get("files")
    artifacts = slam_manifest.get("artifacts")
    if not isinstance(files, list) or not isinstance(artifacts, list):
        raise ValueError("Observer and authoritative SLAM manifest are missing artifact entries")

    correction_policy = (
        "derive optimized_node_pose * inverse(raw_node_odom_pose); linear translation + "
        "quaternion slerp between node timestamps; no extrapolation"
    )
    required_artifacts = {
        "slam_map_poses.csv": (
            "dense_corrected_trajectory", "map", True, map_version, dense_pose_version, 1, correction_policy,
        ),
        "slam_map_keyframes.csv": (
            "optimized_graph_keyframes", "map", True, map_version, None, 2, correction_policy,
        ),
        "slam_poses.csv": ("legacy_map_trajectory", "map", True, map_version, None, 1, None),
        "slam_odom_poses.csv": ("raw_odometry_diagnostic", "odom", False, None, None, 1, None),
        "map_to_odom.csv": ("incremental_tf_diagnostic", "map->odom", False, None, None, 1, None),
        "slam_map.pcd": ("final_optimized_cloud", "map", True, map_version, None, 1, None),
        "slam_map.ply": ("final_optimized_cloud", "map", True, map_version, None, 1, None),
    }
    expected_entry_fields = {
        "path", "role", "size_bytes", "sha256", "frame_id", "optimized",
        "map_version", "dense_pose_version", "schema_version", "correction_policy",
    }

    def unique_entries(entries: list[object], manifest_name: str) -> dict[str, dict[str, object]]:
        result: dict[str, dict[str, object]] = {}
        seen_normalized_paths: set[str] = set()
        for entry in entries:
            if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
                raise ValueError(f"{manifest_name} has a malformed artifact entry")
            name = entry["path"]
            normalized_name = name.replace("\\", "/")
            normalized_name = "/".join(part for part in normalized_name.split("/") if part not in ("", "."))
            normalized_key = normalized_name.casefold()
            if normalized_key in seen_normalized_paths:
                raise ValueError(f"{manifest_name} contains duplicate artifact path after normalization or case-folding: {name!r}")
            seen_normalized_paths.add(normalized_key)
            if name != normalized_name or name not in required_artifacts:
                raise ValueError(f"{manifest_name} has a noncanonical or unknown artifact path {name!r}")
            if name in result:
                raise ValueError(f"{manifest_name} contains duplicate artifact path {name!r}")
            if set(entry) != expected_entry_fields:
                raise ValueError(f"{manifest_name} has malformed artifact metadata for {name}")
            result[name] = entry
        return result

    observer_entries = unique_entries(files, "slam_observer.json")
    manifest_entries = unique_entries(artifacts, "slam_manifest.json")
    embedded_files = embedded_observer.get("files")
    if not isinstance(embedded_files, list):
        raise ValueError("Embedded observer is missing artifact integrity entries")
    if observer_entries != manifest_entries or observer_entries != unique_entries(embedded_files, "embedded observer"):
        raise ValueError("Observer and authoritative SLAM manifest disagree on artifact integrity records")
    if set(observer_entries) != set(required_artifacts):
        raise ValueError("SLAM artifact records do not contain exactly the seven canonical artifacts")
    consumed_artifact_metadata: dict[str, dict[str, object]] = {}
    for artifact_name, (role, frame_id, optimized, expected_map_version, expected_dense_version, schema_version, expected_policy) in required_artifacts.items():
        entry = observer_entries.get(artifact_name)
        artifact_path = path.parent / artifact_name
        if not isinstance(entry, dict) or not artifact_path.is_file():
            raise ValueError(f"SLAM manifests are missing integrity metadata for {artifact_name}")
        expected_size = entry.get("size_bytes")
        expected_digest = entry.get("sha256")
        if not isinstance(expected_size, int) or isinstance(expected_size, bool) or expected_size <= 0 or not isinstance(expected_digest, str):
            raise ValueError(f"SLAM manifest has malformed integrity metadata for {artifact_name}")
        if entry.get("frame_id") != frame_id or entry.get("optimized") is not optimized:
            raise ValueError(f"SLAM artifact {artifact_name} has an unexpected frame or optimization state")
        if entry.get("role") != role:
            raise ValueError(f"SLAM artifact {artifact_name} has an unexpected role")
        if entry.get("map_version") != expected_map_version:
            raise ValueError(f"SLAM artifact {artifact_name} does not match the final map_version")
        if entry.get("dense_pose_version") != expected_dense_version:
            raise ValueError(f"SLAM artifact {artifact_name} does not match the dense_pose_version")
        if (
            not isinstance(entry.get("schema_version"), int)
            or isinstance(entry.get("schema_version"), bool)
            or entry.get("schema_version") != schema_version
        ):
            raise ValueError(f"SLAM artifact {artifact_name} has an unexpected schema_version")
        if entry.get("correction_policy") != expected_policy:
            raise ValueError(f"SLAM artifact {artifact_name} has an unexpected correction_policy")
        content = artifact_path.read_bytes()
        actual_digest = hashlib.sha256(content).hexdigest()
        if len(content) != expected_size or actual_digest != expected_digest:
            raise ValueError(f"{artifact_name} size or SHA-256 does not match the authoritative SLAM manifest")
        consumed_artifact_metadata[artifact_name] = {
            "size_bytes": len(content), "sha256": actual_digest,
        }

    manifest_digest = hashlib.sha256(manifest_bytes).hexdigest()
    return {
        "pre_publish_source_graph_identity": pre_publish_source_identity,
        "final_source_graph_identity": final_source_identity,
        "graph_pose_version": graph_version,
        "dense_pose_version": dense_pose_version,
        "map_version": map_version,
        "slam_manifest_sha256": manifest_digest,
        "slam_manifest_size_bytes": len(manifest_bytes),
        "slam_observer_sha256": hashlib.sha256(observer_bytes).hexdigest(),
        "slam_observer_size_bytes": len(observer_bytes),
        "slam_map_pose_sha256": consumed_artifact_metadata["slam_map_poses.csv"]["sha256"],
        "slam_map_pose_size_bytes": consumed_artifact_metadata["slam_map_poses.csv"]["size_bytes"],
        "slam_map_keyframes_sha256": consumed_artifact_metadata["slam_map_keyframes.csv"]["sha256"],
        "slam_map_keyframes_size_bytes": consumed_artifact_metadata["slam_map_keyframes.csv"]["size_bytes"],
        "slam_map_keyframes_schema_version": required_artifacts["slam_map_keyframes.csv"][5],
        "slam_map_keyframes_map_version": map_version,
        "slam_odom_pose_sha256": consumed_artifact_metadata["slam_odom_poses.csv"]["sha256"],
        "slam_odom_pose_size_bytes": consumed_artifact_metadata["slam_odom_poses.csv"]["size_bytes"],
        "slam_cloud_sha256": consumed_artifact_metadata["slam_map.pcd"]["sha256"],
        "slam_cloud_size_bytes": consumed_artifact_metadata["slam_map.pcd"]["size_bytes"],
        "slam_cloud_ply_sha256": consumed_artifact_metadata["slam_map.ply"]["sha256"],
        "slam_cloud_ply_size_bytes": consumed_artifact_metadata["slam_map.ply"]["size_bytes"],
        "map_pose_sample_count": sample_count,
        "odom_sample_count": odom_sample_count,
    }


def _load_slam_poses(
    path: Path, *, include_provenance: bool = False
) -> list[PoseSample] | tuple[list[PoseSample], dict[str, object]]:
    """Load only the explicitly corrected map-frame pose stream."""

    if not path.is_file():
        return ([], {}) if include_provenance else []
    artifact_contract = _verify_corrected_pose_artifact_contract(path)
    poses: list[PoseSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required_fields = {"timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"}
        if not required_fields.issubset(set(reader.fieldnames or ())):
            raise ValueError(f"Corrected map pose file is missing required fields: {sorted(required_fields)}")
        for row in reader:
            if row["frame_id"] != "map":
                raise ValueError(f"Expected frame_id=map in corrected pose stream, found {row['frame_id']!r}")
            values = {key: float(row[key]) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("Corrected map pose stream contains non-finite numeric data")
            orientation = safe_quaternion(tuple(values[key] for key in ("qx", "qy", "qz", "qw")))
            if orientation is None:
                raise ValueError("Corrected map pose stream contains an invalid quaternion")
            poses.append(PoseSample(
                values["timestamp_s"],
                (values["x_m"], values["y_m"], values["z_m"]),
                orientation,
            ))
    raw_odom_path = path.with_name("slam_odom_poses.csv")
    if not raw_odom_path.is_file():
        raise ValueError("Corrected map pose stream has no raw odom timestamp index")
    with raw_odom_path.open(newline="", encoding="utf-8") as handle:
        odom_reader = csv.DictReader(handle)
        if not required_fields.issubset(set(odom_reader.fieldnames or ())):
            raise ValueError(f"Raw odom pose file is missing required fields: {sorted(required_fields)}")
        odom_timestamps = []
        for row in odom_reader:
            if row["frame_id"] != "odom":
                raise ValueError(f"Expected frame_id=odom in raw pose stream, found {row['frame_id']!r}")
            values = {key: float(row[key]) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("Raw odom pose stream contains non-finite numeric data")
            if safe_quaternion(tuple(values[key] for key in ("qx", "qy", "qz", "qw"))) is None:
                raise ValueError("Raw odom pose stream contains an invalid quaternion")
            odom_timestamps.append(values["timestamp_s"])
    map_timestamps = [pose.timestamp_s for pose in poses]
    for label, timestamps in (("Corrected map", map_timestamps), ("Raw odom", odom_timestamps)):
        if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
            raise ValueError(f"{label} pose timestamps must be strictly increasing with no duplicates")
    if len(map_timestamps) != len(odom_timestamps):
        raise ValueError("Corrected map and raw odom pose streams have different row counts")
    if len(map_timestamps) != artifact_contract.get("map_pose_sample_count"):
        raise ValueError("Corrected map pose row count does not match the optimized graph manifest")
    if len(odom_timestamps) != artifact_contract.get("odom_sample_count"):
        raise ValueError("Raw odom pose row count does not match the optimized graph manifest")
    if map_timestamps != odom_timestamps:
        raise ValueError("Corrected map pose stream does not cover the exact raw odom timestamps")
    return (poses, artifact_contract) if include_provenance else poses


def _load_sensor_geometry(path: Path) -> dict[str, object]:
    data = json.loads(path.read_text(encoding="utf-8"))
    transforms = {item["child"]: item for item in data["transforms"]}
    camera_head_trajectory, camera_head_receipt = load_camera_head_transform_artifact(path, data)
    if camera_head_trajectory is None:
        if "camera_link" in transforms:
            rig_camera_t = np.asarray(transforms["camera_link"]["translation_m"], dtype=np.float64)
            rig_camera_r = _quat_to_matrix(tuple(transforms["camera_link"]["rotation_xyzw"]))
            link_optical_t = np.asarray(
                transforms["camera_optical_frame"]["translation_m"], dtype=np.float64
            )
            link_optical_r = _quat_to_matrix(
                tuple(transforms["camera_optical_frame"]["rotation_xyzw"])
            )
        else:
            frames = data["frames"]
            rig_from_optical = resolve_transform(
                data["transforms"],
                source_frame=str(frames["camera_optical"]),
                target_frame=str(frames["sensor_rig"]),
            )
            rig_camera_t = rig_from_optical[:3, 3]
            rig_camera_r = rig_from_optical[:3, :3]
            link_optical_t = np.zeros(3, dtype=np.float64)
            link_optical_r = np.eye(3, dtype=np.float64)
    else:
        # Dynamic captures intentionally omit the same static graph edge. These
        # placeholders are never used because projection evaluates the bound
        # head trajectory at each image timestamp.
        rig_camera_t = np.zeros(3, dtype=np.float64)
        rig_camera_r = np.eye(3, dtype=np.float64)
        link_optical_t = np.asarray(
            transforms["camera_optical_frame"]["translation_m"], dtype=np.float64
        )
        link_optical_r = _quat_to_matrix(
            tuple(transforms["camera_optical_frame"]["rotation_xyzw"])
        )
    return {
        "rig_lidar_t": np.asarray(transforms["lidar_link"]["translation_m"], dtype=np.float64),
        "rig_lidar_r": _quat_to_matrix(tuple(transforms["lidar_link"]["rotation_xyzw"])),
        "rig_camera_t": rig_camera_t,
        "rig_camera_r": rig_camera_r,
        "link_optical_t": link_optical_t,
        "link_optical_r": link_optical_r,
        "camera_head_trajectory": camera_head_trajectory,
        "camera_head_transform_receipt": camera_head_receipt,
        "fx": float(data["intrinsics"]["fx_px"]),
        "fy": float(data["intrinsics"]["fy_px"]),
        "cx": float(data["intrinsics"]["cx_px"]),
        "cy": float(data["intrinsics"]["cy_px"]),
    }


def _decode_pointcloud2_xyz(message) -> np.ndarray | None:
    """Decode FLOAT32/FLOAT64 XYZ PointCloud2, validating endian and row layout."""

    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 is missing one or more XYZ fields")
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width == 0 or height == 0:
        return None
    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("PointCloud2 has invalid width, height, point_step, or row_step")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step * height")

    values = []
    byte_order = ">" if bool(message.is_bigendian) else "<"
    for name in ("x", "y", "z"):
        field = fields[name]
        datatype = int(field.datatype)
        dtype = {7: np.dtype(byte_order + "f4"), 8: np.dtype(byte_order + "f8")}.get(datatype)
        offset = int(field.offset)
        if dtype is None:
            raise ValueError(f"Unsupported PointCloud2 datatype {datatype} for field {name}")
        if int(field.count) != 1:
            raise ValueError(f"PointCloud2 field {name} has unsupported count {field.count}")
        if offset < 0 or offset + dtype.itemsize > point_step:
            raise ValueError(f"PointCloud2 field {name} is outside point_step")
        try:
            array = np.ndarray(
                shape=(height, width),
                dtype=dtype,
                buffer=raw,
                offset=offset,
                strides=(row_step, point_step),
            )
        except (TypeError, ValueError, BufferError) as exc:
            raise ValueError(f"Could not decode PointCloud2 field {name}") from exc
        values.append(array.reshape(-1))
    return np.column_stack(values).astype(np.float64, copy=False)


def _finite_xyz_points(points: np.ndarray) -> np.ndarray:
    return points[np.isfinite(points).all(axis=1)]


def _select_lidar_point_indices(points: np.ndarray, max_decimation_factor: int = LIDAR_POINT_MAX_DECIMATION_FACTOR) -> np.ndarray:
    """Select deterministic, angle-balanced returns with radial coverage.

    The sample budget is at least one return per ``max_decimation_factor``
    finite points. Equal-azimuth buckets avoid serialized ring ordering; each
    bucket contributes radially stratified samples. Returned indices address
    the original decoded array.
    """

    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be an Nx3 XYZ array")
    count = len(points)
    if count == 0:
        return np.empty(0, dtype=np.int64)
    if max_decimation_factor <= 0:
        raise ValueError("max_decimation_factor must be positive")
    finite_indices = np.flatnonzero(np.isfinite(points).all(axis=1))
    finite_points = points[finite_indices]
    finite_count = len(finite_points)
    if finite_count == 0:
        return np.empty(0, dtype=np.int64)
    budget = max(1, int(math.ceil(finite_count / float(max_decimation_factor))))
    if budget >= finite_count:
        return finite_indices

    azimuth = np.mod(np.arctan2(finite_points[:, 1], finite_points[:, 0]), 2.0 * math.pi)
    angular_bin_count = max(1, budget // 4)
    bin_ids = np.minimum(angular_bin_count - 1, (azimuth * angular_bin_count / (2.0 * math.pi)).astype(int))
    ranges = np.linalg.norm(finite_points, axis=1)
    buckets: dict[int, list[int]] = {}
    for local_index, bin_id in enumerate(bin_ids):
        buckets.setdefault(int(bin_id), []).append(local_index)

    ordered_buckets: dict[int, list[int]] = {}
    for bin_id, members in sorted(buckets.items()):
        members_array = np.asarray(members, dtype=np.int64)
        order = np.lexsort((
            finite_indices[members_array],
            finite_points[members_array, 1],
            finite_points[members_array, 0],
            finite_points[members_array, 2],
            azimuth[members_array],
            ranges[members_array],
        ))
        ordered_buckets[bin_id] = members_array[order].tolist()

    # Give each occupied angle bucket one sample, then distribute remaining
    # budget evenly so dense angles cannot consume the whole scan allowance.
    selected_local: list[int] = []
    allocations = {bin_id: min(1, len(members)) for bin_id, members in ordered_buckets.items()}
    remaining = budget - sum(allocations.values())
    while remaining > 0:
        progressed = False
        for bin_id, members in ordered_buckets.items():
            if allocations[bin_id] >= len(members):
                continue
            allocations[bin_id] += 1
            remaining -= 1
            progressed = True
            if remaining == 0:
                break
        if not progressed:
            break
    for bin_id, members in ordered_buckets.items():
        allocation = allocations[bin_id]
        ranks = np.linspace(0, len(members) - 1, allocation, dtype=np.int64)
        selected_local.extend(members[int(rank)] for rank in ranks)
    selected_original = finite_indices[np.asarray(selected_local, dtype=np.int64)]
    return np.sort(selected_original)


def _read_lidar_scans(bag_dir: Path, diagnostics: dict[str, int] | None = None):
    """Yield sampled XYZ, original decoded indices, and scan header timestamps."""

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
        if diagnostics is not None:
            diagnostics["raw_lidar_message_count"] = diagnostics.get("raw_lidar_message_count", 0) + 1
        message = deserialize_message(serialized, PointCloud2)
        points = _decode_pointcloud2_xyz(message)
        if points is None:
            continue
        if diagnostics is not None:
            diagnostics["valid_decoded_scan_count"] = diagnostics.get("valid_decoded_scan_count", 0) + 1
        source_indices = _select_lidar_point_indices(points)
        if len(source_indices):
            if diagnostics is not None:
                diagnostics["usable_finite_scan_count"] = diagnostics.get("usable_finite_scan_count", 0) + 1
            stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0
            yield stamp, points[source_indices], source_indices


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


def _world_points_to_camera(
    points_world: np.ndarray,
    camera_pose: PoseSample,
    geometry: dict[str, object],
    image_timestamp_s: float | None = None,
) -> np.ndarray:
    """Express world points in the RGB camera optical frame at image time."""

    world_r = _quat_to_matrix(camera_pose.orientation_xyzw)
    pose_t = np.asarray(camera_pose.position_m, dtype=np.float64)
    points_rig = (points_world - pose_t) @ world_r
    rig_camera_t = geometry["rig_camera_t"]
    rig_camera_r = geometry["rig_camera_r"]
    camera_head = geometry.get("camera_head_trajectory")
    if camera_head is not None:
        timestamp_s = camera_pose.timestamp_s if image_timestamp_s is None else image_timestamp_s
        rig_from_camera = camera_head.rig_from_camera_link(timestamp_s)
        rig_camera_t = rig_from_camera[:3, 3]
        rig_camera_r = rig_from_camera[:3, :3]
    points_link = (points_rig - rig_camera_t) @ rig_camera_r
    link_optical_t = geometry.get("link_optical_t", np.zeros(3, dtype=np.float64))
    return (points_link - link_optical_t) @ geometry["link_optical_r"]


def _augment_with_lidar_estimates(
    capture: Path,
    slam: Path,
    frames: list[dict[str, object]],
    frame_annotations: list[dict[str, object]],
    *,
    expected_lidar_message_count: int | None = None,
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray], dict[str, object], dict[int, dict[str, object]]]:
    """Associate projected LiDAR returns with RGB proposals and make start-relative estimates."""

    loaded_poses = _load_slam_poses(slam / "slam_map_poses.csv", include_provenance=True)
    if isinstance(loaded_poses, tuple) and len(loaded_poses) == 2:
        poses, pose_provenance = loaded_poses
    else:  # Preserve injected legacy readers used by focused callers/tests.
        poses, pose_provenance = loaded_poses, {}
    if not poses:
        return {}, {}, {"status": "unavailable", "reason": "no_valid_corrected_map_poses"}, {}
    geometry = _load_sensor_geometry(capture / "sensor_transforms.json")
    start_t = np.asarray(poses[0].position_m, dtype=np.float64)
    frame_timestamps = [float(frame["stamp_s"]) for frame in frames]
    camera_head = geometry.get("camera_head_trajectory")
    if camera_head is not None:
        camera_head.validate_image_timestamps(frame_timestamps)
    annotations_by_frame = {int(record["frame_index"]): record for record in frame_annotations}
    observations_map: dict[int, list[np.ndarray]] = {}
    observation_support: dict[int, dict[str, object]] = {}
    seen_scan_stamps: set[float] = set()
    lidar_read_diagnostics: dict[str, int] = {}
    yielded_scan_count = 0
    pose_covered_scan_count = 0
    projected_scan_count = 0
    projected_point_count = 0

    for scan in _read_lidar_scans(capture / "sensors_bag", diagnostics=lidar_read_diagnostics):
        yielded_scan_count += 1
        if len(scan) == 2:  # Preserve simple injected readers used by callers/tests.
            stamp_s, lidar_points = scan
            source_indices = np.arange(len(lidar_points), dtype=np.int64)
        else:
            stamp_s, lidar_points, source_indices = scan
            source_indices = np.asarray(source_indices, dtype=np.int64)
        if len(source_indices) != len(lidar_points):
            raise ValueError("LiDAR scan source indices must align with sampled points")
        # A repeated bag message at the same sensor timestamp is one scan of
        # evidence and must not produce a duplicate estimate/update count.
        scan_stamp = round(float(stamp_s), 9)
        if scan_stamp in seen_scan_stamps:
            continue
        seen_scan_stamps.add(scan_stamp)
        pose = interpolate_pose(poses, stamp_s, max_gap_s=0.5)
        if pose is None:
            continue
        frame_index = _nearest_frame_index(frames, frame_timestamps, stamp_s)
        if frame_index is None:
            continue
        frame_record = annotations_by_frame.get(int(frames[frame_index]["frame_index"]))
        rgb_stamp_s = frame_timestamps[frame_index]
        rgb_pose = interpolate_pose(poses, rgb_stamp_s, max_gap_s=0.5)
        if rgb_pose is None:
            continue
        pose_covered_scan_count += 1
        if frame_record is None or not frame_record["detections"]:
            continue
        rig_lidar_t = geometry["rig_lidar_t"]
        rig_lidar_r = geometry["rig_lidar_r"]
        points_rig = lidar_points @ rig_lidar_r.T + rig_lidar_t
        world_r = _quat_to_matrix(pose.orientation_xyzw)
        pose_t = np.asarray(pose.position_m, dtype=np.float64)
        points_world = points_rig @ world_r.T + pose_t
        # LiDAR and RGB callbacks have independent timestamps.  Project each
        # return through its measurement-time world pose and then into the
        # camera pose at the selected RGB image timestamp.
        points_optical = _world_points_to_camera(
            points_world, rgb_pose, geometry, image_timestamp_s=rgb_stamp_s
        )
        positive = (points_optical[:, 2] > 0.25) & (points_optical[:, 2] < 45.0)
        points_world = points_world[positive]
        points_optical = points_optical[positive]
        source_indices = source_indices[positive]
        if not len(points_optical):
            continue
        u = geometry["fx"] * points_optical[:, 0] / points_optical[:, 2] + geometry["cx"]
        v = geometry["fy"] * points_optical[:, 1] / points_optical[:, 2] + geometry["cy"]
        in_image = (u >= 0.0) & (u < float(frames[frame_index]["width"])) & (v >= 0.0) & (v < float(frames[frame_index]["height"]))
        points_world = points_world[in_image]
        points_optical = points_optical[in_image]
        source_indices = source_indices[in_image]
        u = u[in_image]
        v = v[in_image]
        if not len(points_optical):
            continue
        projected_scan_count += 1
        projected_point_count += len(points_optical)
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
            # Form one robust estimate in the declared map/SLAM frame, then
            # derive its start-relative representation by the exact inverse
            # transform.  Independent median-vs-mean aggregation made the two
            # exported coordinates disagree even under identity alignment.
            estimate_map = np.median(points_world[selected_indices], axis=0)
            track_id = int(detection["track_id"])
            observations_map.setdefault(track_id, []).append(estimate_map)
            support = observation_support.setdefault(track_id, {
                "scan_stamps_s": set(),
                "unique_retained_depth_return_count": 0,
                "3d_update_event_count": 0,
                "source_indices_by_scan": {},
            })
            support["scan_stamps_s"].add(scan_stamp)
            support["unique_retained_depth_return_count"] += int(len(selected_indices))
            support["3d_update_event_count"] += 1
            source_indices_by_scan = support["source_indices_by_scan"]
            retained_source_indices = sorted(int(value) for value in source_indices[selected_indices])
            source_indices_by_scan.setdefault(scan_stamp, set()).update(retained_source_indices)
            detection["depth_point_count"] = int(len(selected_indices))
            detection["depth_scan_observation_count"] = int(detection.get("depth_scan_observation_count", 0)) + 1
            supporting_stamps = detection.setdefault("supporting_lidar_scan_timestamps_s", [])
            if scan_stamp not in supporting_stamps:
                supporting_stamps.append(scan_stamp)
            detection.setdefault("lidar_depth_support", []).append({
                "scan_timestamp_s": scan_stamp,
                "point_timestamp_s": scan_stamp,
                "point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
                "source_point_indices": retained_source_indices,
            })
    map_estimates = {track_id: np.median(np.stack(values), axis=0) for track_id, values in observations_map.items() if values}
    track_estimates = _map_estimates_to_start_relative(
        map_estimates, start_t, poses[0].orientation_xyzw
    )

    support_timestamps = {
        track_id: sorted(float(value) for value in support["scan_stamps_s"])
        for track_id, support in observation_support.items()
    }
    for frame_record in frame_annotations:
        for detection in frame_record["detections"]:
            track_id = int(detection["track_id"])
            estimate = track_estimates.get(track_id)
            if estimate is None:
                detection["coordinate_source"] = "rgb_only_no_lidar_association"
                continue
            rounded = [round(float(value), 3) for value in estimate]
            detection["estimated_center_start_relative_m"] = rounded
            detection["position_quantity"] = "median_of_associated_front_surface_lidar_returns"
            detection["track_supporting_lidar_scan_timestamps_s"] = support_timestamps.get(track_id, [])
            detection["estimate_is_track_level_backfill"] = not bool(detection.get("depth_scan_observation_count", 0))
            detection["coordinate_source"] = "lidar_projected_with_slam_pose"
    for frame_record in frame_annotations:
        for detection in frame_record["detections"]:
            estimate = map_estimates.get(int(detection["track_id"]))
            if estimate is not None:
                detection["estimated_center_map_m"] = [round(float(value), 3) for value in estimate]
                detection["position_quantity"] = "median_of_associated_front_surface_lidar_returns"
                detection["pose_provenance"] = pose_provenance
                detection["camera_transform_provenance"] = geometry.get(
                    "camera_head_transform_receipt",
                    {
                        "mode": "static_sensor_transform",
                        "artifact_declared": False,
                        "parent_frame": "sensor_rig",
                        "child_frame": "camera_link",
                    },
                )
    support_summary: dict[int, dict[str, object]] = {}
    for track_id, support in observation_support.items():
        stamps = sorted(float(value) for value in support["scan_stamps_s"])
        support_summary[track_id] = {
            "scan_timestamps_s": stamps,
            "unique_supporting_scan_count": len(stamps),
            "unique_retained_depth_return_count": int(support["unique_retained_depth_return_count"]),
            "3d_update_event_count": int(support["3d_update_event_count"]),
            "observation_duration_s": round(stamps[-1] - stamps[0], 6) if len(stamps) > 1 else 0.0,
            "source_point_support": [
                {
                    "scan_timestamp_s": stamp,
                    "source_point_indices": sorted(int(value) for value in source_indices),
                    "point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
                }
                for stamp, source_indices in sorted(support["source_indices_by_scan"].items())
            ],
        }
    raw_lidar_message_count = int(lidar_read_diagnostics.get("raw_lidar_message_count", yielded_scan_count))
    valid_decoded_scan_count = int(lidar_read_diagnostics.get("valid_decoded_scan_count", yielded_scan_count))
    usable_finite_scan_count = int(lidar_read_diagnostics.get("usable_finite_scan_count", yielded_scan_count))
    incomplete_reasons = []
    if raw_lidar_message_count <= 0:
        incomplete_reasons.append("no_lidar_messages_read")
    if valid_decoded_scan_count <= 0:
        incomplete_reasons.append("no_valid_lidar_scans_decoded")
    if usable_finite_scan_count <= 0:
        incomplete_reasons.append("no_finite_usable_lidar_scans")
    if expected_lidar_message_count is not None:
        if raw_lidar_message_count != expected_lidar_message_count:
            incomplete_reasons.append("lidar_message_count_mismatch")
        if valid_decoded_scan_count != expected_lidar_message_count:
            incomplete_reasons.append("valid_lidar_scan_count_mismatch")
        if usable_finite_scan_count != expected_lidar_message_count:
            incomplete_reasons.append("usable_finite_scan_count_mismatch")
    if pose_covered_scan_count <= 0:
        incomplete_reasons.append("no_pose_covered_lidar_scans")
    if projected_scan_count <= 0 or projected_point_count <= 0:
        incomplete_reasons.append("no_lidar_points_projected_into_rgb")
    if not track_estimates:
        incomplete_reasons.append("no_lidar_supported_estimates")

    localization_summary = {
        "status": "incomplete" if incomplete_reasons else "complete",
        "reason": ",".join(incomplete_reasons) if incomplete_reasons else None,
        "start_pose_world_m": [round(float(value), 6) for value in start_t],
        "start_pose_orientation_xyzw": [round(float(value), 8) for value in poses[0].orientation_xyzw],
        "start_pose_world_m_exact": [float(value) for value in start_t],
        "start_pose_orientation_xyzw_exact": [float(value) for value in poses[0].orientation_xyzw],
        "coordinate_frame": "start-relative sensor-rig frame; x forward, y left, z up",
        "source_pose_frame": "map",
        "source_pose_artifact": "slam_map_poses.csv",
        "map_to_odom_correction_applied": True,
        "pose_provenance": pose_provenance,
        "camera_transform_provenance": geometry.get(
            "camera_head_transform_receipt",
            {
                "mode": "static_sensor_transform",
                "artifact_declared": False,
                "parent_frame": "sensor_rig",
                "child_frame": "camera_link",
            },
        ),
        "lidar_input_stream_read": raw_lidar_message_count > 0,
        "position_quantity": "median_of_associated_front_surface_lidar_returns",
        "lidar_pose_time_reference": "PointCloud2 header timestamp; per-return timing and deskew are unavailable",
        "lidar_point_sampling_max_decimation_factor": LIDAR_POINT_MAX_DECIMATION_FACTOR,
        "lidar_point_sampling_policy": "deterministic equal-azimuth buckets with radially stratified samples; original decoded indices retained",
        "lidar_point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
        "expected_lidar_message_count": expected_lidar_message_count,
        "raw_lidar_message_count": raw_lidar_message_count,
        "valid_decoded_scan_count": valid_decoded_scan_count,
        "usable_finite_scan_count": usable_finite_scan_count,
        "unique_lidar_scan_count": len(seen_scan_stamps),
        "pose_covered_scan_count": pose_covered_scan_count,
        "projected_scan_count": projected_scan_count,
        "scan_count_used": projected_scan_count,
        "projected_point_count": projected_point_count,
        "track_count_with_3d_estimate": len(track_estimates),
    }
    return track_estimates, map_estimates, localization_summary, support_summary


def _maximum_cardinality_minimum_cost_assignment(
    track_ids: Iterable[int],
    detection_count: int,
    candidate_costs: dict[tuple[int, int], float],
) -> list[tuple[int, int]]:
    """Return a deterministic max-cardinality, then min-cost bipartite match."""

    ordered_track_ids = sorted(int(track_id) for track_id in track_ids)
    if not ordered_track_ids or detection_count <= 0 or not candidate_costs:
        return []

    source = 0
    track_base = 1
    detection_base = track_base + len(ordered_track_ids)
    sink = detection_base + detection_count
    graph: list[list[list[float | int]]] = [[] for _ in range(sink + 1)]

    def add_edge(start: int, end: int, capacity: int, cost: float) -> int:
        edge_index = len(graph[start])
        reverse_index = len(graph[end])
        graph[start].append([end, reverse_index, capacity, float(cost)])
        graph[end].append([start, edge_index, 0, -float(cost)])
        return edge_index

    track_node = {track_id: track_base + index for index, track_id in enumerate(ordered_track_ids)}
    for track_id in ordered_track_ids:
        add_edge(source, track_node[track_id], 1, 0.0)
    for detection_index in range(detection_count):
        add_edge(detection_base + detection_index, sink, 1, 0.0)

    assignment_edges: dict[tuple[int, int], tuple[int, int]] = {}
    for (track_id, detection_index), cost in sorted(candidate_costs.items()):
        if track_id not in track_node or not 0 <= detection_index < detection_count:
            continue
        edge_index = add_edge(track_node[track_id], detection_base + detection_index, 1, float(cost))
        assignment_edges[(track_id, detection_index)] = (track_node[track_id], edge_index)

    # Augment until no path remains to maximize cardinality. Reduced costs and
    # stable node/edge order minimize total cost deterministically for that size.
    potentials = [0.0] * len(graph)
    while True:
        distances = [math.inf] * len(graph)
        previous_node = [-1] * len(graph)
        previous_edge = [-1] * len(graph)
        distances[source] = 0.0
        queue: list[tuple[float, int]] = [(0.0, source)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance > distances[node] + 1e-12:
                continue
            for edge_index, edge in enumerate(graph[node]):
                target, _, capacity, edge_cost = edge
                if int(capacity) <= 0:
                    continue
                target = int(target)
                reduced_cost = float(edge_cost) + potentials[node] - potentials[target]
                if reduced_cost < 0.0 and reduced_cost > -1e-9:
                    reduced_cost = 0.0
                proposed = distance + reduced_cost
                if proposed + 1e-12 < distances[target]:
                    distances[target] = proposed
                    previous_node[target] = node
                    previous_edge[target] = edge_index
                    heapq.heappush(queue, (proposed, target))
        if previous_node[sink] < 0:
            break
        for node, distance in enumerate(distances):
            if math.isfinite(distance):
                potentials[node] += distance
        node = sink
        while node != source:
            parent = previous_node[node]
            edge_index = previous_edge[node]
            edge = graph[parent][edge_index]
            reverse_index = int(edge[1])
            edge[2] = int(edge[2]) - 1
            graph[node][reverse_index][2] = int(graph[node][reverse_index][2]) + 1
            node = parent

    return sorted(
        (track_id, detection_index)
        for (track_id, detection_index), (node, edge_index) in assignment_edges.items()
        if int(graph[node][edge_index][2]) == 0
    )


def _consolidate_track_estimates(
    track_estimates: dict[int, np.ndarray],
    merge_radius_m: float = 0.21,
    co_visible_pairs: set[tuple[int, int]] | None = None,
) -> tuple[dict[int, int], dict[int, np.ndarray], dict[int, list[int]]]:
    """Merge RGB track fragments by 3D proximity, subject to co-visibility vetoes.

    RGB components can split when a product crosses a shelf edge or changes
    apparent size. The canonical ID is assigned from sensor estimates, while
    raw tracks observed together in one RGB frame are never merged. Simulator
    identity and ground truth are not consumed.
    """

    clusters: list[dict[str, object]] = []
    grid: dict[tuple[int, int, int], list[int]] = {}
    cluster_cells: dict[int, tuple[int, int, int]] = {}
    co_visible_pairs = co_visible_pairs or set()
    for raw_track_id in sorted(track_estimates):
        point = np.asarray(track_estimates[raw_track_id], dtype=np.float64)
        cell = tuple(np.floor(point / merge_radius_m).astype(int))
        candidates = _spatial_grid_candidate_indices(point, merge_radius_m, grid)
        selected = None
        selected_distance = float("inf")
        for cluster_index in candidates:
            members = clusters[cluster_index]["members"]
            if any(
                (min(raw_track_id, int(member)), max(raw_track_id, int(member))) in co_visible_pairs
                for member in members
            ):
                continue
            center = np.asarray(clusters[cluster_index]["center"], dtype=np.float64)
            distance = float(np.linalg.norm(center - point))
            if distance <= merge_radius_m and distance < selected_distance:
                selected = cluster_index
                selected_distance = distance
        if selected is None:
            selected = len(clusters)
            clusters.append({"center": point.copy(), "count": 1, "members": [raw_track_id]})
            grid.setdefault(cell, []).append(selected)
            cluster_cells[selected] = cell
        else:
            cluster = clusters[selected]
            count = int(cluster["count"]) + 1
            cluster["center"] = (np.asarray(cluster["center"]) * (count - 1) + point) / count
            cluster["count"] = count
            cluster["members"].append(raw_track_id)
            # The index is keyed by cluster center.  Move the entry whenever
            # the centroid crosses a voxel so later lookups remain complete.
            center = np.asarray(cluster["center"], dtype=np.float64)
            new_cell = tuple(np.floor(center / merge_radius_m).astype(int))
            old_cell = cluster_cells[selected]
            if new_cell != old_cell:
                grid[old_cell].remove(selected)
                if not grid[old_cell]:
                    del grid[old_cell]
                grid.setdefault(new_cell, []).append(selected)
                cluster_cells[selected] = new_cell

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


def _set_canonical_track_identity(detection: dict[str, object], raw_track_id: int, canonical_id: int) -> None:
    """Retain the RGB track namespace while exposing the localized ID explicitly."""

    detection["raw_track_id"] = int(raw_track_id)
    detection["persistent_track_id"] = int(canonical_id)
    detection["canonical_track_id"] = int(canonical_id)
    detection["id_namespace"] = "canonical_localized"
    # Compatibility alias used by existing overlay and evaluation consumers.
    detection["track_id"] = int(canonical_id)


def _set_unassigned_persistent_identity(detection: dict[str, object], raw_track_id: int) -> None:
    """Export an RGB-only proposal without aliasing its raw ID as persistent."""

    detection["raw_track_id"] = int(raw_track_id)
    detection["persistent_track_id"] = None
    detection["canonical_track_id"] = None
    detection["id_namespace"] = "raw_rgb"
    detection["track_id"] = None


def _spatial_grid_candidate_indices(
    point: np.ndarray, merge_radius_m: float, grid: dict[tuple[int, int, int], list[int]]
) -> list[int]:
    cell = tuple(np.floor(np.asarray(point, dtype=np.float64) / merge_radius_m).astype(int))
    candidates: set[int] = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                candidates.update(grid.get((cell[0] + dx, cell[1] + dy, cell[2] + dz), ()))
    return sorted(candidates)


def _co_visible_raw_track_pairs(frame_annotations: list[dict[str, object]]) -> set[tuple[int, int]]:
    pairs: set[tuple[int, int]] = set()
    for frame in frame_annotations:
        track_ids = sorted({
            int(detection["raw_track_id"] if "raw_track_id" in detection else detection["track_id"])
            for detection in frame["detections"]
        })
        for index, first in enumerate(track_ids):
            pairs.update((first, second) for second in track_ids[index + 1 :])
    return pairs


def run_rgb_tracking(capture_dir: str | Path, slam_dir: str | Path, output_dir: str | Path, repo_root: str | Path | None = None) -> dict[str, object]:
    """Run RGB tracking plus sensor-only 3D localization."""

    capture = Path(capture_dir).resolve()
    slam = Path(slam_dir).resolve()
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    input_bindings = None
    if (capture / "capture_manifest.json").is_file() and (slam / "slam_manifest.json").is_file():
        input_bindings = build_perception_input_bindings(capture, slam, output)
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
            proposal_diagnostics: dict[str, object] = {}
            detections = detect_product_blobs(image, diagnostics=proposal_diagnostics)
            records = tracker.update(int(frames_index[decoded]["frame_index"]), detections)
            frame_annotations.append({
                "frame_index": int(frames_index[decoded]["frame_index"]),
                "stamp_s": float(frames_index[decoded]["stamp_s"]),
                "width": int(frames_index[decoded]["width"]),
                "height": int(frames_index[decoded]["height"]),
                "proposal_budget": proposal_diagnostics,
                "detections": records,
            })
            decoded += 1
    if decoded != len(frames_index):
        raise RuntimeError(f"RGB frame/index mismatch: decoded={decoded}, indexed={len(frames_index)}")

    track_estimates, map_estimates, localization, lidar_scan_observation_counts = _augment_with_lidar_estimates(
        capture,
        slam,
        frames_index,
        frame_annotations,
        expected_lidar_message_count=(
            int(input_bindings["raw_lidar"]["message_count"])
            if input_bindings is not None else None
        ),
    )
    co_visible_pairs = _co_visible_raw_track_pairs(frame_annotations)
    raw_to_canonical, canonical_map_estimates, canonical_members = _consolidate_track_estimates(
        map_estimates, co_visible_pairs=co_visible_pairs
    )
    start_t = np.asarray(localization.get("start_pose_world_m_exact", (0.0, 0.0, 0.0)), dtype=np.float64)
    canonical_estimates = _map_estimates_to_start_relative(
        canonical_map_estimates,
        start_t,
        tuple(localization.get("start_pose_orientation_xyzw_exact", (0.0, 0.0, 0.0, 1.0))),
    )
    localization["canonical_track_count"] = len(canonical_estimates)
    localization["canonicalization_radius_m"] = 0.21
    pose_provenance = localization.get("pose_provenance", {})

    # Replace fragment IDs in the output stream with stable 3D-associated IDs.
    for frame in frame_annotations:
        for detection in frame["detections"]:
            raw_track_id = int(detection["raw_track_id"])
            canonical_id = raw_to_canonical.get(raw_track_id)
            if canonical_id is None:
                _set_unassigned_persistent_identity(detection, raw_track_id)
                continue
            _set_canonical_track_identity(detection, raw_track_id, canonical_id)
            detection["estimated_center_start_relative_m"] = [
                round(float(value), 3) for value in canonical_estimates[canonical_id]
            ]
            detection["position_quantity"] = "median_of_associated_front_surface_lidar_returns"
            if canonical_id in canonical_map_estimates:
                detection["estimated_center_map_m"] = [
                    round(float(value), 3) for value in canonical_map_estimates[canonical_id]
                ]
            detection["coordinate_source"] = "lidar_projected_with_slam_pose_consolidated"
            detection["pose_provenance"] = pose_provenance

    track_rows = []
    for canonical_id in sorted(canonical_estimates):
        raw_members = canonical_members[canonical_id]
        member_tracks = [
            track for raw_id in raw_members
            if (track := tracker.track_for_export(raw_id)) is not None
        ]
        if not member_tracks:
            continue
        member_support = [lidar_scan_observation_counts.get(raw_id, {}) for raw_id in raw_members]
        supporting_stamps = sorted({
            float(stamp)
            for support in member_support
            for stamp in support.get("scan_timestamps_s", [])
        })
        source_point_support_by_stamp: dict[float, set[int]] = {}
        for support in member_support:
            for scan_support in support.get("source_point_support", []):
                stamp = float(scan_support["scan_timestamp_s"])
                source_point_support_by_stamp.setdefault(stamp, set()).update(
                    int(value) for value in scan_support["source_point_indices"]
                )
        source_point_support = [
            {"scan_timestamp_s": stamp, "source_point_indices": sorted(indices)}
            for stamp, indices in sorted(source_point_support_by_stamp.items())
        ]
        retained_return_count = sum(int(support.get("unique_retained_depth_return_count", 0)) for support in member_support)
        update_event_count = sum(int(support.get("3d_update_event_count", 0)) for support in member_support)
        total_detections = sum(track.detection_count for track in member_tracks)
        weighted_u = sum(track.center_px[0] * track.detection_count for track in member_tracks) / max(1, total_detections)
        weighted_v = sum(track.center_px[1] * track.detection_count for track in member_tracks) / max(1, total_detections)
        track_rows.append({
            "track_id": canonical_id,
            "persistent_track_id": canonical_id,
            "canonical_track_id": canonical_id,
            "raw_track_ids": raw_members,
            "id_namespace": "canonical_localized",
            "class": "unknown_product",
            "source": "rgb_lidar_slam_consolidated",
            "first_frame_index": min(track.first_frame_index for track in member_tracks),
            "last_frame_index": max(track.last_frame_index for track in member_tracks),
            "detection_count": total_detections,
            "rgb_detection_count": total_detections,
            "center_u_px": round(weighted_u, 3),
            "center_v_px": round(weighted_v, 3),
            "estimated_x_m": round(float(canonical_estimates[canonical_id][0]), 3),
            "estimated_y_m": round(float(canonical_estimates[canonical_id][1]), 3),
            "estimated_z_m": round(float(canonical_estimates[canonical_id][2]), 3),
            "map_x_m": round(float(canonical_map_estimates[canonical_id][0]), 3) if canonical_id in canonical_map_estimates else None,
            "map_y_m": round(float(canonical_map_estimates[canonical_id][1]), 3) if canonical_id in canonical_map_estimates else None,
            "map_z_m": round(float(canonical_map_estimates[canonical_id][2]), 3) if canonical_id in canonical_map_estimates else None,
            "depth_source": LIDAR_PROJECTED_DEPTH_SOURCE,
            "position_quantity": "median_of_associated_front_surface_lidar_returns",
            "supporting_lidar_scan_timestamps_s": supporting_stamps,
            "supporting_lidar_source_points": source_point_support,
            "3d_observation_count": len(supporting_stamps),
            "unique_supporting_scan_count": len(supporting_stamps),
            "unique_retained_depth_return_count": retained_return_count,
            "3d_update_event_count": update_event_count,
            "observation_duration_s": round(supporting_stamps[-1] - supporting_stamps[0], 6) if len(supporting_stamps) > 1 else 0.0,
            "raw_track_count": len(raw_members),
            "map_version": pose_provenance.get("map_version"),
            "graph_pose_version": pose_provenance.get("graph_pose_version"),
            "dense_pose_version": pose_provenance.get("dense_pose_version"),
            "slam_manifest_sha256": pose_provenance.get("slam_manifest_sha256"),
            "slam_manifest_size_bytes": pose_provenance.get("slam_manifest_size_bytes"),
            "slam_observer_sha256": pose_provenance.get("slam_observer_sha256"),
            "slam_observer_size_bytes": pose_provenance.get("slam_observer_size_bytes"),
            "slam_map_pose_sha256": pose_provenance.get("slam_map_pose_sha256"),
            "slam_map_pose_size_bytes": pose_provenance.get("slam_map_pose_size_bytes"),
            "slam_map_keyframes_sha256": pose_provenance.get("slam_map_keyframes_sha256"),
            "slam_map_keyframes_size_bytes": pose_provenance.get("slam_map_keyframes_size_bytes"),
            "slam_map_keyframes_schema_version": pose_provenance.get("slam_map_keyframes_schema_version"),
            "slam_map_keyframes_map_version": pose_provenance.get("slam_map_keyframes_map_version"),
            "slam_odom_pose_sha256": pose_provenance.get("slam_odom_pose_sha256"),
            "slam_odom_pose_size_bytes": pose_provenance.get("slam_odom_pose_size_bytes"),
            "slam_cloud_sha256": pose_provenance.get("slam_cloud_sha256"),
            "slam_cloud_size_bytes": pose_provenance.get("slam_cloud_size_bytes"),
            "slam_cloud_ply_sha256": pose_provenance.get("slam_cloud_ply_sha256"),
            "slam_cloud_ply_size_bytes": pose_provenance.get("slam_cloud_ply_size_bytes"),
            "map_pose_sample_count": pose_provenance.get("map_pose_sample_count"),
        })

    annotation_path = output / "frame_annotations.jsonl"
    with annotation_path.open("w", encoding="utf-8", newline="\n") as handle:
        for record in frame_annotations:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    for row in track_rows:
        row["3d_observation_count"] = int(row["3d_observation_count"])
    fields = ["track_id", "persistent_track_id", "canonical_track_id", "raw_track_ids", "id_namespace", "class", "source", "first_frame_index", "last_frame_index", "detection_count", "rgb_detection_count", "center_u_px", "center_v_px", "estimated_x_m", "estimated_y_m", "estimated_z_m", "map_x_m", "map_y_m", "map_z_m", "depth_source", "position_quantity", "supporting_lidar_scan_timestamps_s", "supporting_lidar_source_points", "3d_observation_count", "unique_supporting_scan_count", "unique_retained_depth_return_count", "3d_update_event_count", "observation_duration_s", "raw_track_count", "map_version", "graph_pose_version", "dense_pose_version", "slam_manifest_sha256", "slam_manifest_size_bytes", "slam_observer_sha256", "slam_observer_size_bytes", "slam_map_pose_sha256", "slam_map_pose_size_bytes", "slam_map_keyframes_sha256", "slam_map_keyframes_size_bytes", "slam_map_keyframes_schema_version", "slam_map_keyframes_map_version", "slam_odom_pose_sha256", "slam_odom_pose_size_bytes", "slam_cloud_sha256", "slam_cloud_size_bytes", "slam_cloud_ply_sha256", "slam_cloud_ply_size_bytes", "map_pose_sample_count"]
    _write_csv(output / "estimated_inventory.csv", track_rows, fields)
    _write_csv(output / "tracks.csv", track_rows, fields)
    (output / "estimated_inventory.json").write_text(json.dumps(track_rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    localization_complete = localization.get("status") == "complete"
    if localization_complete and input_bindings is None:
        raise ValueError("complete perception requires repaired-v1 capture and SLAM input bindings")
    if localization_complete:
        expected_lidar_messages = int(input_bindings["raw_lidar"]["message_count"])
        exact_counts = (
            int(localization.get("expected_lidar_message_count", -1)) == expected_lidar_messages,
            int(localization.get("raw_lidar_message_count", -1)) == expected_lidar_messages,
            int(localization.get("valid_decoded_scan_count", -1)) == expected_lidar_messages,
            int(localization.get("usable_finite_scan_count", -1)) == expected_lidar_messages,
        )
        positive_support = (
            int(localization.get("pose_covered_scan_count", 0)) > 0,
            int(localization.get("projected_scan_count", 0)) > 0,
            int(localization.get("projected_point_count", 0)) > 0,
            int(localization.get("track_count_with_3d_estimate", 0)) > 0,
        )
        if not all(exact_counts + positive_support):
            raise ValueError("complete perception lacks exact LiDAR message coverage and projected estimate support")
    summary = {
        "status": "complete" if localization_complete else "incomplete",
        "detector_status": "rgb_color_connected_component_baseline",
        "tracker_status": "global_max_cardinality_min_cost_prediction_assignment",
        "capture_only": True,
        "ground_truth_consumed": False,
        "ground_truth_required": False,
        "legacy_capture_id_omitted": False,
        "legacy_capture_manifest_v0": False,
        "slam_consumed_for_estimation": localization_complete,
        "lidar_consumed_for_estimation": bool(localization.get("lidar_input_stream_read", False)),
        "capture_id": input_bindings["capture"]["capture_id"] if input_bindings else None,
        "capture_sha256": input_bindings["capture"]["capture_sha256"] if input_bindings else None,
        "capture_manifest_sha256": input_bindings["capture"]["manifest"]["sha256"] if input_bindings else None,
        "inputs": input_bindings,
        "slam_trajectory": input_bindings["slam"]["trajectory"]["path"] if input_bindings else None,
        "allowed_depth_sources": [LIDAR_PROJECTED_DEPTH_SOURCE],
        "localization": localization,
        "slam_artifact": "../slam/slam_map.pcd" if (slam / "slam_map.pcd").exists() else None,
        "slam_cloud_sha256": pose_provenance.get("slam_cloud_sha256"),
        "slam_cloud_size_bytes": pose_provenance.get("slam_cloud_size_bytes"),
        "slam_artifact_sha256": pose_provenance.get("slam_cloud_sha256"),
        "slam_artifact_size_bytes": pose_provenance.get("slam_cloud_size_bytes"),
        "slam_cloud_ply_sha256": pose_provenance.get("slam_cloud_ply_sha256"),
        "slam_cloud_ply_size_bytes": pose_provenance.get("slam_cloud_ply_size_bytes"),
        "map_version": pose_provenance.get("map_version"),
        "pre_publish_source_graph_identity": pose_provenance.get("pre_publish_source_graph_identity"),
        "final_source_graph_identity": pose_provenance.get("final_source_graph_identity"),
        "graph_pose_version": pose_provenance.get("graph_pose_version"),
        "dense_pose_version": pose_provenance.get("dense_pose_version"),
        "slam_manifest_sha256": input_bindings["slam"]["manifest"]["sha256"] if input_bindings else pose_provenance.get("slam_manifest_sha256"),
        "slam_manifest_size_bytes": pose_provenance.get("slam_manifest_size_bytes"),
        "slam_map_pose_sha256": pose_provenance.get("slam_map_pose_sha256"),
        "slam_map_keyframes_sha256": pose_provenance.get("slam_map_keyframes_sha256"),
        "slam_map_keyframes_size_bytes": pose_provenance.get("slam_map_keyframes_size_bytes"),
        "slam_map_keyframes_schema_version": pose_provenance.get("slam_map_keyframes_schema_version"),
        "slam_map_keyframes_map_version": pose_provenance.get("slam_map_keyframes_map_version"),
        "slam_pose_provenance": pose_provenance or None,
        "frame_count": len(frame_annotations),
        "detection_count": sum(len(item["detections"]) for item in frame_annotations),
        "track_count": len(track_rows),
        "raw_rgb_track_count": len(tracker.tracks) + len(tracker.retired_tracks),
        "video": "../capture/rgb_camera.mp4",
        "frames": "../capture/rgb_frames.jsonl",
        "annotations": "frame_annotations.jsonl",
        "estimated_inventory": "estimated_inventory.csv",
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "notes": "RGB connected components are product-like proposals, not product instances. Localized positions are medians of associated front-surface LiDAR returns projected using scan-time and RGB-time SLAM poses; they are not full-product centers or extents.",
    }
    if localization_complete:
        validate_perception_manifest_bindings(summary, capture, slam, output)
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
