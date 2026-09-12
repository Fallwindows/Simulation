"""Typed, early-validating configuration loading.

The repository configs are JSON-compatible YAML. JSON is parsed first so the
core remains usable on a clean Python installation; PyYAML is accepted when
installed for future human-authored YAML files.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _read_mapping(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    raw = path.read_text(encoding="utf-8")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise ValueError(f"{path} is not JSON-compatible YAML and PyYAML is unavailable") from exc
        value = yaml.safe_load(raw)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a mapping")
    return value


def _tuple3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class PoseConfig:
    position_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]


@dataclass(frozen=True)
class AisleConfig:
    name: str
    length_m: float
    width_m: float
    shelf_height_m: float
    shelf_depth_m: float
    bay_width_m: float
    shelf_levels: int
    product_density: float
    seed: int
    floor_z_m: float
    shelf_rows_y_m: tuple[float, ...]
    lighting_lux: float


@dataclass(frozen=True)
class CameraConfig:
    width_px: int
    height_px: int
    fps: float
    horizontal_fov_deg: float
    near_m: float
    far_m: float
    pose_in_rig: PoseConfig


@dataclass(frozen=True)
class LidarConfig:
    preset: str
    hz: float
    min_range_m: float
    max_range_m: float
    vertical_fov_deg: tuple[float, float]
    horizontal_samples: int
    vertical_samples: int
    pose_in_rig: PoseConfig


@dataclass(frozen=True)
class TrajectoryConfig:
    name: str
    duration_s: float
    speed_mps: float
    start_position_m: tuple[float, float, float]
    yaw_deg: float
    sample_hz: float
    bob_amplitude_m: float = 0.0
    bob_hz: float = 0.0
    sway_amplitude_m: float = 0.0
    sway_hz: float = 0.0
    yaw_amplitude_deg: float = 0.0
    pitch_amplitude_deg: float = 0.0
    speed_variation_fraction: float = 0.0


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    environment: AisleConfig
    camera: CameraConfig
    lidar: LidarConfig
    trajectory: TrajectoryConfig
    mapping: dict[str, Any]
    seed: int
    sensor_overrides: dict[str, Any]


def _positive(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def _pose(data: dict[str, Any], name: str) -> PoseConfig:
    return PoseConfig(_tuple3(data.get("position_m", [0.0, 0.0, 0.0]), f"{name}.position_m"), _tuple3(data.get("rpy_deg", [0.0, 0.0, 0.0]), f"{name}.rpy_deg"))


def _aisle(data: dict[str, Any]) -> AisleConfig:
    density = float(data["product_density"])
    if not 0.0 <= density <= 1.0:
        raise ValueError("product_density must be between 0 and 1")
    levels = int(data["shelf_levels"])
    if levels < 1:
        raise ValueError("shelf_levels must be at least 1")
    rows = tuple(float(v) for v in data.get("shelf_rows_y_m", [-1.65, 1.65]))
    if not rows:
        raise ValueError("shelf_rows_y_m must not be empty")
    return AisleConfig(
        name=str(data.get("name", "aisle")),
        length_m=_positive(data["length_m"], "length_m"),
        width_m=_positive(data["width_m"], "width_m"),
        shelf_height_m=_positive(data["shelf_height_m"], "shelf_height_m"),
        shelf_depth_m=_positive(data["shelf_depth_m"], "shelf_depth_m"),
        bay_width_m=_positive(data["bay_width_m"], "bay_width_m"),
        shelf_levels=levels,
        product_density=density,
        seed=int(data.get("seed", 1)),
        floor_z_m=float(data.get("floor_z_m", 0.0)),
        shelf_rows_y_m=rows,
        lighting_lux=_positive(data.get("lighting_lux", 450.0), "lighting_lux"),
    )


def _camera(data: dict[str, Any]) -> CameraConfig:
    width, height = int(data["width_px"]), int(data["height_px"])
    if width < 1 or height < 1:
        raise ValueError("camera dimensions must be positive")
    near_m, far_m = _positive(data["near_m"], "camera.near_m"), _positive(data["far_m"], "camera.far_m")
    if near_m >= far_m:
        raise ValueError("camera.near_m must be less than camera.far_m")
    return CameraConfig(width, height, _positive(data["fps"], "camera.fps"), _positive(data["horizontal_fov_deg"], "camera.horizontal_fov_deg"), near_m, far_m, _pose(data.get("pose_in_rig", {}), "camera.pose_in_rig"))


def _lidar(data: dict[str, Any]) -> LidarConfig:
    fov = tuple(float(v) for v in data.get("vertical_fov_deg", [-25.0, 15.0]))
    if len(fov) != 2 or fov[0] >= fov[1]:
        raise ValueError("lidar.vertical_fov_deg must be an increasing pair")
    horizontal, vertical = int(data["horizontal_samples"]), int(data["vertical_samples"])
    if horizontal < 1 or vertical < 1:
        raise ValueError("lidar sample counts must be positive")
    return LidarConfig(str(data["preset"]), _positive(data["hz"], "lidar.hz"), _positive(data["min_range_m"], "lidar.min_range_m"), _positive(data["max_range_m"], "lidar.max_range_m"), fov, horizontal, vertical, _pose(data.get("pose_in_rig", {}), "lidar.pose_in_rig"))


def _trajectory(data: dict[str, Any]) -> TrajectoryConfig:
    duration = _positive(data["duration_s"], "trajectory.duration_s")
    speed = _positive(data["speed_mps"], "trajectory.speed_mps")
    sample_hz = _positive(data.get("sample_hz", 60.0), "trajectory.sample_hz")
    variation = float(data.get("speed_variation_fraction", 0.0))
    if not 0.0 <= variation < 1.0:
        raise ValueError("trajectory.speed_variation_fraction must be in [0, 1)")
    return TrajectoryConfig(str(data["name"]), duration, speed, _tuple3(data["start_position_m"], "trajectory.start_position_m"), float(data.get("yaw_deg", 0.0)), sample_hz, float(data.get("bob_amplitude_m", 0.0)), float(data.get("bob_hz", 0.0)), float(data.get("sway_amplitude_m", 0.0)), float(data.get("sway_hz", 0.0)), float(data.get("yaw_amplitude_deg", 0.0)), float(data.get("pitch_amplitude_deg", 0.0)), variation)


def load_contracts(path: str | Path) -> dict[str, Any]:
    data = _read_mapping(Path(path))
    for section in ("frames", "topics", "dashboard"):
        if not isinstance(data.get(section), dict):
            raise ValueError(f"contracts.{section} must be a mapping")
    if int(data["dashboard"]["port"]) not in range(1, 65536):
        raise ValueError("dashboard.port is invalid")
    return data


def load_scenario(path: str | Path) -> ScenarioConfig:
    scenario_path = Path(path).resolve()
    scenario = _read_mapping(scenario_path)
    base = scenario_path.parent
    environment = _aisle(_read_mapping((base / scenario["environment"]).resolve()))
    sensors = _read_mapping((base / scenario["sensors"]).resolve())
    trajectory = _trajectory(_read_mapping((base / scenario["trajectory"]).resolve()))
    mapping = _read_mapping((base / scenario["mapping"]).resolve())
    return ScenarioConfig(str(scenario["name"]), environment, _camera(sensors["camera"]), _lidar(sensors["lidar"]), trajectory, mapping, int(scenario.get("seed", environment.seed)), dict(scenario.get("sensor_overrides", {})))
