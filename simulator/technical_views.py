"""CPU renderer for honest technical views of recorded SLAM artifacts.

The renderer consumes a coherent capture/SLAM/perception run. It projects the
actual point cloud, estimated trajectory, and ground-truth-free estimated item
centers with NumPy/OpenCV, then uses Pillow for screen typography. It does not
load scene geometry, RGB beauty footage, or storyboard pixels.
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

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


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
    capture_git_sha: str
    slam_git_sha: str
    perception_git_sha: str
    simulation_time_start_s: float
    simulation_time_end_s: float
    map_version: str
    trajectory_version: str
    object_state_version: str
    depth_sources: tuple[str, ...]
    map_path: Path
    trajectory_path: Path
    inventory_path: Path
    capture_manifest_path: Path
    slam_manifest_path: Path
    perception_manifest_path: Path
    source_catalog_path: Path
    hashes: dict[str, str]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
    views = tuple(ViewSpec(**item) for item in payload["views"])
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
        "map": run / "slam" / "slam_map.ply",
        "trajectory": run / "slam" / "slam_poses.csv",
        "inventory": run / "perception" / "estimated_inventory.csv",
        "capture_manifest": run / "capture" / "capture_manifest.json",
        "slam_manifest": run / "slam" / "slam_manifest.json",
        "perception_manifest": run / "perception" / "perception_manifest.json",
    }
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
    elif str(perception_capture_id) != capture_id:
        raise ValueError("capture, SLAM, and perception capture_id must agree")
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
    for artifact_name in ("map", "trajectory", "inventory"):
        binding = artifacts.get(artifact_name)
        if not isinstance(binding, dict) or binding.get("path") != str(paths[artifact_name].relative_to(run)).replace("\\", "/") or binding.get("sha256") != hashes[artifact_name]:
            raise ValueError(f"{artifact_name} artifact does not match the reviewed source catalog")
    if source.get("capture_sha256") != capture.get("capture_sha256"):
        raise ValueError("capture checksum does not match the reviewed source catalog")
    if perception.get("estimated_inventory") != perception_contract.get("estimated_inventory"):
        raise ValueError("perception manifest inventory association does not match the reviewed source catalog")
    if perception.get("slam_artifact") != perception_contract.get("slam_artifact"):
        raise ValueError("perception manifest SLAM association does not match the reviewed source catalog")
    allowed_depth_sources = frozenset(perception_contract.get("allowed_depth_sources", []))
    if not allowed_depth_sources or not allowed_depth_sources.issubset(ALLOWED_ESTIMATED_DEPTH_SOURCES):
        raise ValueError("catalog source has no supported estimated depth source")
    depth_sources = _inventory_depth_sources(paths["inventory"], allowed_depth_sources)
    start_s, end_s = _trajectory_time_range(paths["trajectory"])
    time_binding = source.get("simulation_time")
    if not isinstance(time_binding, dict) or time_binding.get("source") != "slam/slam_poses.csv:timestamp_s":
        raise ValueError("catalog source lacks a trajectory simulation-time binding")
    if not math.isclose(start_s, float(time_binding.get("start_s", math.nan)), abs_tol=1e-9) or not math.isclose(end_s, float(time_binding.get("end_s", math.nan)), abs_tol=1e-9):
        raise ValueError("trajectory simulation-time range does not match the reviewed source catalog")
    hashes["source_catalog"] = sha256_file(catalog_path)
    return SourceBundle(
        run_root=run,
        capture_id=capture_id,
        source_id=str(source["source_id"]),
        capture_git_sha=capture_git,
        slam_git_sha=slam_git,
        perception_git_sha=perception_git,
        simulation_time_start_s=start_s,
        simulation_time_end_s=end_s,
        map_version=str(artifacts["map"]["version"]),
        trajectory_version=str(artifacts["trajectory"]["version"]),
        object_state_version=str(artifacts["inventory"]["version"]),
        depth_sources=depth_sources,
        map_path=paths["map"],
        trajectory_path=paths["trajectory"],
        inventory_path=paths["inventory"],
        capture_manifest_path=paths["capture_manifest"],
        slam_manifest_path=paths["slam_manifest"],
        perception_manifest_path=paths["perception_manifest"],
        source_catalog_path=catalog_path,
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
    def __init__(self, profile: RenderProfile, points: np.ndarray, trajectory: np.ndarray, inventory: tuple[InventoryRecord, ...]):
        self.profile = profile
        self.points = points
        self.trajectory = trajectory
        lower, upper = np.percentile(points, [1, 99], axis=0)
        bounded_inventory = tuple(
            record for record in inventory
            if np.all(np.asarray(record.position) >= lower - 0.5) and np.all(np.asarray(record.position) <= upper + 0.5)
        )
        self.inventory = select_inventory(bounded_inventory)
        self.center = np.median(points, axis=0)
        self.center[2] = np.percentile(points[:, 2], 50)
        self.z_low, self.z_high = np.percentile(points[:, 2], [3, 97])
        scale = profile.height / 720.0
        self.fonts = {
            "kicker": ImageFont.truetype(str(FONT_BOLD), max(13, int(15 * scale))),
            "title": ImageFont.truetype(str(FONT_BOLD), max(30, int(42 * scale))),
            "body": ImageFont.truetype(str(FONT_REGULAR), max(14, int(18 * scale))),
            "mono": ImageFont.truetype(str(Path("C:/Windows/Fonts/consola.ttf")), max(12, int(15 * scale))),
        }

    def _camera(self, view_id: str, progress: float) -> tuple[np.ndarray, np.ndarray, float]:
        if view_id in ("sensor_activation", "lidar_environment"):
            path_index = min(len(self.trajectory) - 1, int((0.28 + 0.25 * progress) * (len(self.trajectory) - 1)))
            base = self.trajectory[path_index]
            eye = base + np.asarray([-0.4, -0.05 + 0.12 * math.sin(progress * math.pi), 1.42], dtype=np.float32)
            target = eye + np.asarray([8.0, 0.15 * math.sin(progress * math.pi * 2), -0.15], dtype=np.float32)
            return eye, target, 0.88
        if view_id in ("object_association", "object_detail") and self.inventory:
            angle = math.radians(-64.0 + 18.0 * progress)
            focus = np.asarray(self.inventory[0].position, dtype=np.float32)
            radius = 4.1 if view_id == "object_association" else 3.2
            elevation = 2.0 if view_id == "object_association" else 1.6
            focal = 0.72
        else:
            angle = math.radians(-96.0 + 12.0 * progress)
            focus = np.asarray([np.median(self.trajectory[:, 0]), 0.0, 0.8], dtype=np.float32)
            radius = 15.5 if view_id == "persistent_map" else 16.5
            elevation = 6.4 if view_id == "persistent_map" else 7.0
            focal = 0.68
        eye = focus + np.asarray([radius * math.cos(angle), radius * math.sin(angle), elevation], dtype=np.float32)
        return eye, focus, focal

    def _background(self) -> np.ndarray:
        height, width = self.profile.height, self.profile.width
        y = np.linspace(0.0, 1.0, height, dtype=np.float32)[:, None, None]
        top = np.asarray([11, 18, 30], dtype=np.float32)
        bottom = np.asarray([2, 6, 12], dtype=np.float32)
        row = top * (1.0 - y) + bottom * y
        return np.broadcast_to(row, (height, width, 3)).copy().astype(np.uint8)

    def _draw_cloud(self, frame: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float, dim: float = 1.0) -> None:
        pixels, depth, mask = project_points(self.points, eye, target, self.profile.width, self.profile.height, focal)
        z = self.points[mask, 2]
        z_norm = np.clip((z - self.z_low) / max(0.1, self.z_high - self.z_low), 0.0, 1.0)
        distance_fade = np.clip(1.2 - depth / 34.0, 0.30, 1.0)
        colors = np.stack([
            178 + 72 * z_norm,
            128 + 92 * z_norm,
            42 + 35 * z_norm,
        ], axis=1) * (distance_fade * dim)[:, None]
        layer = np.zeros_like(frame)
        order = np.argsort(depth)[::-1]
        xy = pixels[order]
        layer[xy[:, 1], xy[:, 0]] = np.clip(colors[order], 0, 255).astype(np.uint8)
        radius = 1 if self.profile.height <= 720 else 2
        layer = cv2.dilate(layer, np.ones((radius + 1, radius + 1), dtype=np.uint8))
        glow = cv2.GaussianBlur(layer, (0, 0), 3.0)
        np.maximum(frame, (glow * 0.34).astype(np.uint8), out=frame)
        np.maximum(frame, layer, out=frame)

    def _draw_sensor_rays(self, frame: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float, progress: float) -> None:
        pixels, depth, _mask = project_points(self.points, eye, target, self.profile.width, self.profile.height, focal)
        candidates = np.flatnonzero((depth >= 2.0) & (depth <= 11.0))
        if len(candidates) == 0:
            return
        count = min(28, len(candidates))
        chosen = candidates[np.linspace(0, len(candidates) - 1, count, dtype=np.int32)]
        origin = (self.profile.width // 2, int(self.profile.height * 0.88))
        overlay = frame.copy()
        activation = float(np.clip((progress - 0.08) / 0.35, 0.0, 1.0))
        for point_index in chosen[: max(1, int(round(count * activation)))]:
            endpoint = tuple(int(value) for value in pixels[point_index])
            cv2.line(overlay, origin, endpoint, (255, 205, 45), max(1, self.profile.height // 720), cv2.LINE_AA)
            cv2.circle(overlay, endpoint, max(2, self.profile.height // 360), (255, 235, 92), -1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.48, frame, 0.52, 0.0, dst=frame)
        scale = self.profile.height / 720.0
        housing = np.asarray([
            (origin[0] - int(35 * scale), origin[1] + int(20 * scale)),
            (origin[0] + int(35 * scale), origin[1] + int(20 * scale)),
            (origin[0] + int(22 * scale), origin[1] - int(13 * scale)),
            (origin[0] - int(22 * scale), origin[1] - int(13 * scale)),
        ], dtype=np.int32)
        cv2.fillConvexPoly(frame, housing, (8, 20, 30), cv2.LINE_AA)
        cv2.polylines(frame, [housing], True, (70, 219, 241), max(1, int(2 * scale)), cv2.LINE_AA)
        cv2.circle(frame, origin, max(4, int(6 * scale)), (255, 223, 71), -1, cv2.LINE_AA)

    def _draw_path(self, frame: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float) -> None:
        pixels, _depth, mask = project_points(self.trajectory + np.asarray([0, 0, 0.06], dtype=np.float32), eye, target, self.profile.width, self.profile.height, focal)
        if mask.sum() >= 2:
            cv2.polylines(frame, [pixels.reshape(-1, 1, 2)], False, (60, 224, 255), max(2, self.profile.height // 360), cv2.LINE_AA)
            cv2.circle(frame, tuple(pixels[-1]), max(5, self.profile.height // 100), (68, 241, 255), -1, cv2.LINE_AA)

    def _draw_inventory(self, frame: np.ndarray, eye: np.ndarray, target: np.ndarray, focal: float, labels: int) -> list[tuple[InventoryRecord, tuple[int, int]]]:
        if not self.inventory:
            return []
        centers = np.asarray([item.position for item in self.inventory], dtype=np.float32)
        pixels, _depth, mask = project_points(centers, eye, target, self.profile.width, self.profile.height, focal)
        visible_records = [item for item, visible in zip(self.inventory, mask) if visible]
        result: list[tuple[InventoryRecord, tuple[int, int]]] = []
        scale = self.profile.height / 720.0
        for index, (record, pixel) in enumerate(zip(visible_records, pixels)):
            x, y = int(pixel[0]), int(pixel[1])
            radius = max(6, int((8 + min(record.observations, 20) * 0.18) * scale))
            color = (52, 216, 255) if index else (92, 236, 164)
            cv2.circle(frame, (x, y), radius + 5, color, 1, cv2.LINE_AA)
            cv2.drawMarker(frame, (x, y), color, cv2.MARKER_CROSS, radius * 2, 2, cv2.LINE_AA)
            if index < labels:
                result.append((record, (x, y)))
        return result

    def _typography(self, frame: np.ndarray, spec: ViewSpec, progress: float, callouts: list[tuple[InventoryRecord, tuple[int, int]]]) -> np.ndarray:
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image, "RGBA")
        width, height = image.size
        scale = height / 720.0
        pad = int(42 * scale)
        draw.rounded_rectangle((pad, pad, min(width - pad, int(650 * scale)), int(162 * scale)), radius=int(12 * scale), fill=(5, 13, 24, 214), outline=(51, 211, 241, 155), width=max(1, int(1 * scale)))
        draw.text((pad + int(22 * scale), pad + int(14 * scale)), "SIMULATED CAPTURE · OFFLINE TECHNICAL VIEW", font=self.fonts["kicker"], fill=(96, 222, 241, 255))
        draw.text((pad + int(22 * scale), pad + int(42 * scale)), spec.title, font=self.fonts["title"], fill=(238, 247, 250, 255))
        draw.text((pad + int(22 * scale), pad + int(94 * scale)), spec.subtitle, font=self.fonts["body"], fill=(160, 185, 199, 255))
        footer_parts = ["MAP FRAME", f"{len(self.points):,} POINTS"]
        if spec.id not in ("sensor_activation", "lidar_environment"):
            footer_parts.append(f"{len(self.trajectory)} ESTIMATED POSES")
        if spec.id in ("object_association", "object_detail", "observed_aisle_overview", "final_technical_view"):
            footer_parts.append(f"{len(self.inventory)} SELECTED CENTERS")
        footer = " · ".join(footer_parts) + f" · t={progress:0.2f}"
        draw.text((pad, height - pad - int(21 * scale)), footer, font=self.fonts["mono"], fill=(125, 162, 178, 235))
        for index, (record, (x, y)) in enumerate(callouts[:3]):
            box_x = width - int(350 * scale)
            box_y = int((84 + index * 86) * scale)
            draw.line((x, y, box_x - int(12 * scale), box_y + int(25 * scale)), fill=(70, 222, 241, 185), width=max(1, int(2 * scale)))
            draw.rounded_rectangle((box_x, box_y, width - pad, box_y + int(67 * scale)), radius=int(8 * scale), fill=(5, 14, 25, 224), outline=(62, 217, 239, 170), width=max(1, int(1 * scale)))
            draw.text((box_x + int(14 * scale), box_y + int(8 * scale)), f"TRACK {record.track_id:05d}", font=self.fonts["kicker"], fill=(102, 233, 246, 255))
            x_m, y_m, z_m = record.position
            draw.text((box_x + int(14 * scale), box_y + int(32 * scale)), f"({x_m:.2f}, {y_m:.2f}, {z_m:.2f}) m  ·  {record.observations} returns", font=self.fonts["mono"], fill=(210, 226, 232, 255))
        if spec.id == "object_detail" and self.inventory:
            record = self.inventory[0]
            card_x = width - int(520 * scale)
            card_y = height - int(260 * scale)
            draw.rounded_rectangle((card_x, card_y, width - pad, height - int(62 * scale)), radius=int(12 * scale), fill=(4, 12, 22, 232), outline=(75, 230, 248, 210), width=max(1, int(2 * scale)))
            text_x = card_x + int(20 * scale)
            text_right = width - pad - int(20 * scale)
            text_width = text_right - text_x
            title = f"Persistent track {record.track_id}"
            title_font = fit_font(title, FONT_BOLD, max(30, int(42 * scale)), max(22, int(28 * scale)), text_width)
            observation = f"Observation support · {record.observations} LiDAR associations"
            observation_font = fit_font(observation, FONT_REGULAR, max(14, int(18 * scale)), max(12, int(14 * scale)), text_width)
            limitation = "Class unknown · Extent not estimated"
            limitation_font = fit_font(limitation, FONT_REGULAR, max(14, int(18 * scale)), max(12, int(14 * scale)), text_width)
            draw.text((text_x, card_y + int(18 * scale)), "SELECTED ESTIMATED CENTER", font=self.fonts["kicker"], fill=(87, 229, 245, 255))
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
            draw.text((line_x, int(378 * scale)), "FINALIZED OFFLINE MAP\nESTIMATED TRAJECTORY\nPERSISTENT ITEM CENTERS", font=self.fonts["body"], fill=(174, 197, 207, 255), spacing=int(9 * scale))
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    def frame(self, spec: ViewSpec, index: int) -> np.ndarray:
        progress = 0.0 if spec.frames == 1 else index / (spec.frames - 1)
        eye, target, focal = self._camera(spec.id, progress)
        frame = self._background()
        self._draw_cloud(frame, eye, target, focal, dim=0.82 if spec.id in ("object_association", "object_detail") else 1.0)
        if spec.id == "sensor_activation":
            self._draw_sensor_rays(frame, eye, target, focal, progress)
        if spec.id not in ("sensor_activation", "lidar_environment"):
            self._draw_path(frame, eye, target, focal)
        labels = 3 if spec.id in ("object_association", "observed_aisle_overview") else 1 if spec.id == "object_detail" else 0
        inventory_views = ("object_association", "object_detail", "observed_aisle_overview", "final_technical_view")
        callouts = self._draw_inventory(frame, eye, target, focal, labels) if spec.id in inventory_views else []
        return self._typography(frame, spec, progress, callouts)


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
    return {
        "source_id": bundle.source_id,
        "capture_id": bundle.capture_id,
        "producer_revisions": {
            "capture": bundle.capture_git_sha,
            "slam": bundle.slam_git_sha,
            "perception": bundle.perception_git_sha,
        },
        "simulation_time": {
            "basis": "slam/slam_poses.csv:timestamp_s",
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
        "artifacts": {
            "map": {"sha256": bundle.hashes["map"], "semantics": "finalized offline LiDAR-SLAM point cloud"},
            "trajectory": {"sha256": bundle.hashes["trajectory"], "semantics": "estimated SLAM trajectory in map frame"},
            "inventory": {"sha256": bundle.hashes["inventory"], "semantics": "ground-truth-free LiDAR-localized estimated centers; no estimated extents"},
            "capture_manifest": {"sha256": bundle.hashes["capture_manifest"]},
            "slam_manifest": {"sha256": bundle.hashes["slam_manifest"]},
            "perception_manifest": {"sha256": bundle.hashes["perception_manifest"]},
            "source_catalog": {"sha256": bundle.hashes["source_catalog"]},
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
    plan_hash: str,
) -> Path:
    receipt = {
        "schema_version": 1,
        "artifact_type": "technical_source_view_receipt",
        "status": "complete",
        "producer": {"id": PRODUCER_ID, "renderer_sha256": renderer_hash, "plan_sha256": plan_hash},
        "view_id": spec.id,
        "presentation_role": spec.presentation_role,
        "source": _source_receipt(bundle),
        "video": {"path": video_path.name, "sha256": video_hash, **probe},
        "derivation": {
            "point_projection": "deterministic perspective projection of finalized map XYZ",
            "trajectory_overlay": spec.id not in ("sensor_activation", "lidar_environment"),
            "inventory_overlay": spec.id in ("object_association", "object_detail", "observed_aisle_overview", "final_technical_view"),
            "sensor_ray_overlay": "selected projected finalized-map points; explanatory rays are not a timestamped current scan" if spec.id == "sensor_activation" else None,
            "estimated_extents_rendered": False,
            "storyboard_pixels_consumed": False,
        },
        "presentation_integration": {
            "adapter_required": True,
            "note": "integration must translate this receipt into the presentation role-bundle schema after the branches merge",
        },
    }
    receipt_path = output / f"{spec.id}_receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return receipt_path


def render(profile: RenderProfile, views: tuple[ViewSpec, ...], bundle: SourceBundle, output_dir: str | Path, ffmpeg: str, ffprobe: str, plan_path: Path) -> dict[str, object]:
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    points = load_ascii_ply(bundle.map_path)
    trajectory = load_trajectory(bundle.trajectory_path)
    inventory_all = load_inventory(bundle.inventory_path)
    renderer = TechnicalRenderer(profile, points, trajectory, inventory_all)
    renderer_hash = sha256_file(Path(__file__).resolve())
    plan_hash = sha256_file(plan_path.resolve())
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
            receipt_path = _write_view_receipt(output, video_path, video_hash, probe, spec, bundle, renderer_hash, plan_hash)
            outputs.append({
                "path": video_path.name,
                "sha256": video_hash,
                "frames": spec.frames,
                "view_ids": [spec.id],
                "presentation_role": spec.presentation_role,
                "probe": probe,
                "receipt": {"path": receipt_path.name, "sha256": sha256_file(receipt_path)},
            })
    elapsed = time.perf_counter() - started
    manifest: dict[str, object] = {
        "schema_version": 1,
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
        "plan": {"path": str(plan_path.resolve()), "sha256": plan_hash},
        "render_graph_sources": ["map", "trajectory", "inventory"],
        "storyboard_content_used": False,
        "storyboard_exclusion_basis": "all render inputs are enumerated and hashed; storyboard paths and known reference hashes are rejected before decode",
        "selected_inventory_centers": len(renderer.inventory),
        "localized_inventory_rows": len(inventory_all),
        "point_count": len(points),
        "trajectory_pose_count": len(trajectory),
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
