"""A deterministic, capture-only RGB product proposal and tracker.

This is deliberately an estimator, not a ground-truth renderer.  It proposes
visible product-like regions from the captured RGB stream using colour and
connected-component evidence, then maintains IDs with frame-to-frame motion
and appearance association.  Ground-truth inventory is never opened here.

The output is useful for the first visual demonstration while leaving a clean
replacement boundary for a trained detector: the renderer consumes only the
per-frame proposal/tracking JSONL produced by this module.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


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


def run_rgb_tracking(capture_dir: str | Path, slam_dir: str | Path, output_dir: str | Path, repo_root: str | Path | None = None) -> dict[str, object]:
    """Run RGB-only tracking and write the render/evaluation interface."""

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

    annotation_path = output / "frame_annotations.jsonl"
    with annotation_path.open("w", encoding="utf-8", newline="\n") as handle:
        for record in frame_annotations:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    track_rows = []
    for track in sorted(tracker.tracks.values(), key=lambda item: item.track_id):
        if track.detection_count < 2:
            continue
        track_rows.append({
            "track_id": track.track_id,
            "class": "unknown_product",
            "source": "rgb_color_connected_component",
            "first_frame_index": track.first_frame_index,
            "last_frame_index": track.last_frame_index,
            "detection_count": track.detection_count,
            "center_u_px": round(track.center_px[0], 3),
            "center_v_px": round(track.center_px[1], 3),
            "estimated_x_m": "",
            "estimated_y_m": "",
            "estimated_z_m": "",
            "depth_source": "not_available_in_rgb_baseline",
        })
    fields = ["track_id", "class", "source", "first_frame_index", "last_frame_index", "detection_count", "center_u_px", "center_v_px", "estimated_x_m", "estimated_y_m", "estimated_z_m", "depth_source"]
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
        "slam_consumed_for_estimation": False,
        "slam_artifact": str((slam / "slam_map.pcd").relative_to(output.parent)).replace("\\", "/") if (slam / "slam_map.pcd").exists() else None,
        "frame_count": len(frame_annotations),
        "detection_count": sum(len(item["detections"]) for item in frame_annotations),
        "track_count": len(track_rows),
        "video": "../capture/rgb_camera.mp4",
        "frames": "../capture/rgb_frames.jsonl",
        "annotations": "frame_annotations.jsonl",
        "estimated_inventory": "estimated_inventory.csv",
        "git_sha": _git_sha(Path(repo_root).resolve()) if repo_root else None,
        "notes": "Centers are measured RGB component centroids; 3D depth is intentionally blank until a depth-aware detector is added.",
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
