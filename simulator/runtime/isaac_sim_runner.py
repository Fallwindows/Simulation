"""Run the grocery aisle in the installed Isaac Sim 6.x runtime.

This module is deliberately an Isaac-side entry point.  The dependency-light
configuration and geometry modules remain importable with ordinary Python,
while this file is launched with ``C:\\isaacsim\\python.bat``.  Sensor data is
published by Isaac's ROS 2 bridge; the only Python publisher here is the
ground-truth pose topic, which is intentionally separate from SLAM odometry.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SCENE_PRICE_DISPLAY_ROOT = REPO_ROOT / "assets" / "scene" / "price_displays"
HERO_PRICE_LABELS = {
    "cereal_sunrise": {"display_name": "SUNRISE OATS", "unit_price": "$4.29"},
    "cereal_harvest": {"display_name": "HARVEST LOOP", "unit_price": "$4.79"},
    "cereal_grain": {"display_name": "GRAIN DAY", "unit_price": "$3.99"},
    "cereal_berry": {"display_name": "BERRY CLOUD", "unit_price": "$5.19"},
    "coffee_bag": {"display_name": "HOUSE COFFEE", "unit_price": "$8.49"},
    "tea_box": {"display_name": "GARDEN TEA", "unit_price": "$4.59"},
    "coffee_canister": {"display_name": "ROAST COFFEE", "unit_price": "$7.99"},
    "snack_wafer": {"display_name": "WAFER WAVE", "unit_price": "$3.29"},
}


def scene_price_display_path(sku_key: str) -> Path:
    """Return the committed per-SKU price-display asset path."""

    if sku_key not in HERO_PRICE_LABELS:
        raise KeyError(f"No scene price display is declared for SKU {sku_key!r}")
    return SCENE_PRICE_DISPLAY_ROOT / f"price_{sku_key}.usda"


def _ensure_bundled_ros_environment() -> dict[str, object]:
    """Validate and normalize Isaac's bundled ROS environment before startup.

    The PowerShell launcher enters ``python.bat`` with ``ROS_DISTRO`` unset so
    Isaac selects its bundled Jazzy distribution.  Keeping the signed library
    and ament prefixes explicit here also supports direct runner invocation and
    records the effective values in every runtime receipt.
    """

    isaac_root_value = os.environ.get("ISAAC_PATH")
    isaac_root = (
        Path(isaac_root_value).resolve()
        if isaac_root_value
        else Path(sys.executable).resolve().parents[2]
    )
    bundled_prefix = isaac_root / "exts" / "isaacsim.ros2.core" / "jazzy"
    bundled_lib = bundled_prefix / "lib"
    if not bundled_lib.is_dir():
        raise FileNotFoundError(f"Isaac bundled ROS library path is missing: {bundled_lib}")

    os.environ["ROS_DISTRO"] = "jazzy"
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    path_entries = [entry for entry in os.environ.get("PATH", "").split(os.pathsep) if entry]
    bundled_key = os.path.normcase(str(bundled_lib))
    path_entries = [entry for entry in path_entries if os.path.normcase(os.path.abspath(entry)) != bundled_key]
    os.environ["PATH"] = os.pathsep.join([str(bundled_lib), *path_entries])

    ament_entries = [entry for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if entry]
    prefix_key = os.path.normcase(str(bundled_prefix))
    if all(os.path.normcase(os.path.abspath(entry)) != prefix_key for entry in ament_entries):
        ament_entries.append(str(bundled_prefix))
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(ament_entries)
    snapshot = {
        "ros_distro": os.environ["ROS_DISTRO"],
        "rmw_implementation": os.environ["RMW_IMPLEMENTATION"],
        "ament_prefix_path": os.environ["AMENT_PREFIX_PATH"],
        "path_prefix": os.environ["PATH"].split(os.pathsep)[0],
        "zenoh_session_config": os.environ.get("ZENOH_SESSION_CONFIG_URI"),
        "zenoh_router_config": os.environ.get("ZENOH_ROUTER_CONFIG_URI"),
    }
    for receipt_key in ("zenoh_session_config", "zenoh_router_config"):
        configured_path = snapshot[receipt_key]
        if configured_path:
            path = Path(str(configured_path)).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"Configured Zenoh file is missing: {path}")
            snapshot[f"{receipt_key}_sha256"] = _sha256_path(path)
    print(f"[grocery-runtime] ROS environment {json.dumps(snapshot, sort_keys=True)}", flush=True)
    return snapshot


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the grocery aisle Isaac Sim sensor runtime")
    parser.add_argument("--scenario", default=str(REPO_ROOT / "config/scenarios/walking_baseline.yaml"))
    parser.add_argument("--frames", type=int, default=0, help="Simulation frames; 0 derives the count from the trajectory duration")
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim viewport")
    parser.add_argument("--realtime", action="store_true", help="Pace the simulation at 60 Hz for external ROS/dashboard consumers")
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=1.0,
        help="Wall-clock pacing factor when --realtime is set (0.25 gives consumers 4x wall time)",
    )
    parser.add_argument("--renderer", default="RaytracedLighting")
    parser.add_argument("--status-path", default=str(REPO_ROOT / "runs/isaac_runtime_status.json"))
    parser.add_argument("--capture-dir", default="", help="Empty directory for selected-pose lossless PNG output")
    parser.add_argument("--capture-frames", default="", help="Comma-separated 60 Hz simulation frame indices")
    parser.add_argument("--capture-width", type=int, default=1280)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument("--capture-rt-subframes", type=int, default=4)
    parser.add_argument(
        "--camera-head-transforms-path",
        default="",
        help="Dynamic sensor_rig->camera_link JSON; defaults beside the guarded capture directory or status",
    )
    parser.add_argument(
        "--capture-only",
        action="store_true",
        help="Build and drive the scene for look-development without starting ROS or RTX LiDAR",
    )
    return parser.parse_args()


def _paced_wall_period_s(realtime: bool, realtime_factor: float, simulation_dt_s: float = 1.0 / 60.0) -> float | None:
    """Return the minimum wall period for one simulation tick."""

    if not realtime:
        return None
    if not math.isfinite(realtime_factor) or realtime_factor <= 0.0 or realtime_factor > 1.0:
        raise ValueError("realtime factor must be finite and in (0, 1]")
    return simulation_dt_s / realtime_factor


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _capture_provenance(
    args: argparse.Namespace,
    scenario,
    camera_head_artifact: dict[str, object] | None = None,
) -> dict[str, object]:
    """Bind representative pixels to source, inputs, and renderer settings."""

    def git(*git_args: str) -> str:
        completed = subprocess.run(
            ["git", *git_args],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        return completed.stdout.strip()

    scenario_path = Path(args.scenario).resolve()
    config_inputs = [{"role": "scenario", "path": str(scenario_path), "sha256": _sha256_path(scenario_path)}]
    try:
        scenario_links = json.loads(scenario_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        scenario_links = {}
    for role in ("environment", "sensors", "trajectory", "mapping"):
        relative_path = scenario_links.get(role)
        if isinstance(relative_path, str):
            input_path = (scenario_path.parent / relative_path).resolve()
            config_inputs.append({"role": role, "path": str(input_path), "sha256": _sha256_path(input_path)})
    asset_manifest = Path(scenario.environment.asset_manifest_path).resolve()
    scene_material_manifest = (REPO_ROOT / "assets" / "scene" / "materials" / "manifest.json").resolve()
    scene_price_display_manifest = (SCENE_PRICE_DISPLAY_ROOT / "manifest.json").resolve()
    provenance = {
        "git_commit": git("rev-parse", "HEAD"),
        "git_tree": git("rev-parse", "HEAD^{tree}"),
        "git_worktree_dirty": bool(git("status", "--porcelain")),
        "config_inputs": config_inputs,
        "asset_manifest": {"path": str(asset_manifest), "sha256": _sha256_path(asset_manifest)},
        "scene_material_manifest": {
            "path": str(scene_material_manifest),
            "sha256": _sha256_path(scene_material_manifest),
        },
        "scene_price_display_manifest": {
            "path": str(scene_price_display_manifest),
            "sha256": _sha256_path(scene_price_display_manifest),
        },
        "renderer": args.renderer,
        "headless": bool(args.headless),
        "capture_only": bool(args.capture_only),
        "simulation_hz": 60.0,
    }
    if camera_head_artifact is not None:
        provenance["camera_head_transforms"] = camera_head_artifact
    return provenance


def _sim_time_message(seconds: float):
    from builtin_interfaces.msg import Time

    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


def camera_mount_orientation(sample, camera_config):
    """Return the sampled sensor_rig -> camera_link rotation.

    The configured mount is composed with the trajectory's explicit head
    articulation.  This pure helper is shared by USD, ROS TF, capture metadata,
    and dependency-light tests so those outputs cannot silently diverge.
    """

    from simulator.sensors.transforms import (
        quaternion_from_rpy_deg,
        quaternion_multiply,
        quaternion_normalize,
    )

    configured_mount = quaternion_from_rpy_deg(*camera_config.pose_in_rig.rpy_deg)
    return quaternion_normalize(
        quaternion_multiply(configured_mount, sample.camera_link_orientation_xyzw)
    )


def camera_link_world_pose(sample, camera_config):
    """Compose the rig and sampled head poses for capture provenance."""

    from simulator.motion.trajectory import PoseSample
    from simulator.sensors.transforms import Transform, compose

    rig_world = Transform("sim_world", "sensor_rig", sample.position_m, sample.orientation_xyzw)
    rig_camera = Transform(
        "sensor_rig",
        "camera_link",
        camera_config.pose_in_rig.position_m,
        camera_mount_orientation(sample, camera_config),
    )
    world_camera = compose(rig_world, rig_camera)
    return PoseSample(sample.timestamp_s, world_camera.translation_m, world_camera.rotation_xyzw)


def camera_optical_world_pose(sample, camera_config):
    """Return the ROS camera optical frame pose in the simulation world."""

    from simulator.motion.trajectory import PoseSample
    from simulator.sensors.transforms import Transform, camera_optical_quaternion, compose

    camera_link = camera_link_world_pose(sample, camera_config)
    world_link = Transform(
        "sim_world",
        "camera_link",
        camera_link.position_m,
        camera_link.orientation_xyzw,
    )
    link_optical = Transform(
        "camera_link",
        "camera_optical_frame",
        (0.0, 0.0, 0.0),
        camera_optical_quaternion(),
    )
    world_optical = compose(world_link, link_optical)
    return PoseSample(sample.timestamp_s, world_optical.translation_m, world_optical.rotation_xyzw)


def camera_head_stamp_alignment(
    rgb_timestamps_s: list[float],
    artifact_timestamps_s: list[float],
    tolerance_s: float = 1e-6,
) -> dict[str, object]:
    """Prove observed RGB stamps address exported head samples by time."""

    deltas = [
        min(abs(rgb_stamp - artifact_stamp) for artifact_stamp in artifact_timestamps_s)
        for rgb_stamp in rgb_timestamps_s
    ]
    max_delta = max(deltas, default=None)
    return {
        "observed_rgb_stamps": len(rgb_timestamps_s),
        "matched_head_samples": sum(delta <= tolerance_s for delta in deltas),
        "tolerance_s": tolerance_s,
        "max_abs_delta_s": max_delta,
        "all_rgb_stamps_matched": max_delta is None or max_delta <= tolerance_s,
    }


def write_camera_head_transforms(
    path: str | Path,
    trajectory,
    scenario,
    trajectory_config_path: str | Path,
) -> dict[str, object]:
    """Write the versioned replay contract for the articulated camera mount."""

    output_path = Path(path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path = Path(trajectory_config_path).resolve()
    from simulator.capture.manifest import sha256_json

    trajectory_config_sha256 = _sha256_path(trajectory_path)
    trajectory_effective_sha256 = sha256_json(dataclasses.asdict(scenario.trajectory))
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD", "HEAD^{tree}"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    samples = trajectory.sample_many()
    contract = {
        "schema": "grocery.camera_head_transforms",
        "version": 1,
        "frames": {
            "parent": "sensor_rig",
            "child": "camera_link",
            "optical_child": "camera_optical_frame",
        },
        "direction": "parent_to_child",
        "translation_units": "m",
        "timestamp_units": "s",
        "timestamp_domain": "Isaac simulation time (/clock)",
        "sample_hz": scenario.trajectory.sample_hz,
        "duration_s": scenario.trajectory.duration_s,
        "interpolation": {
            "translation": "linear",
            "rotation": "shortest_arc_quaternion_slerp_xyzw",
            "range": "closed_0_to_duration_no_extrapolation",
        },
        "composition": "q_sensor_rig_camera_link = q_configured_mount * q_head_articulation",
        "source": {
            "trajectory_config": {
                "path": str(trajectory_path),
                "sha256": trajectory_config_sha256,
            },
            "trajectory_effective_sha256": trajectory_effective_sha256,
            "git_commit": revision[0],
            "git_tree": revision[1],
        },
        "static_child_transform": {
            "parent": "camera_link",
            "child": "camera_optical_frame",
            "translation_m": [0.0, 0.0, 0.0],
            "rotation_xyzw": [0.5, -0.5, 0.5, -0.5],
        },
        "samples": [
            {
                "timestamp_s": sample.timestamp_s,
                "translation_m": list(scenario.camera.pose_in_rig.position_m),
                "rotation_xyzw": list(camera_mount_orientation(sample, scenario.camera)),
            }
            for sample in samples
        ],
    }
    output_path.write_text(json.dumps(contract, indent=2) + "\n", encoding="utf-8")
    return {
        "path": str(output_path),
        "sha256": _sha256_path(output_path),
        "schema": contract["schema"],
        "version": contract["version"],
        "source": contract["source"],
    }


def camera_head_output_path(args, has_representative_frames: bool) -> Path:
    """Resolve head output without contaminating the guarded image directory."""

    if args.camera_head_transforms_path:
        output = Path(args.camera_head_transforms_path).resolve()
    elif has_representative_frames:
        capture_dir = Path(args.capture_dir).resolve()
        output = capture_dir.with_name(f"{capture_dir.name}.camera_head_transforms.json")
    else:
        output = Path(args.status_path).resolve().with_name("camera_head_transforms.json")
    if has_representative_frames:
        capture_root = Path(args.capture_dir).resolve()
        try:
            output.relative_to(capture_root)
        except ValueError:
            pass
        else:
            raise ValueError(
                "--camera-head-transforms-path must stay outside the representative capture directory"
            )
    return output


def lidar_runtime_spec(lidar_config) -> dict[str, object]:
    """Map the ROS sensor contract to the Isaac 6.1 LiDAR boundary.

    Scenario poses are expressed relative to ``sensor_rig`` in ROS ``xyzw``
    order.  Isaac's native RTX LiDAR API takes a local mount pose in ``wxyz``
    order.  Keeping both representations in the returned spec makes the
    boundary and its frame semantics inspectable in run artifacts.
    """
    from simulator.sensors.transforms import quaternion_from_rpy_deg, quaternion_xyzw_to_wxyz

    orientation_xyzw = quaternion_from_rpy_deg(*lidar_config.pose_in_rig.rpy_deg)
    vertical_samples = int(lidar_config.vertical_samples)
    vertical_min_deg, vertical_max_deg = (float(value) for value in lidar_config.vertical_fov_deg)
    elevation_step_deg = (
        0.0
        if vertical_samples == 1
        else (vertical_max_deg - vertical_min_deg) / float(vertical_samples - 1)
    )
    elevations_deg = [
        vertical_min_deg + index * elevation_step_deg
        for index in range(vertical_samples)
    ]
    horizontal_samples = int(lidar_config.horizontal_samples)
    pattern_firing_rate_hz = int(round(float(lidar_config.hz) * horizontal_samples))
    return {
        "tick_rate_hz": float(lidar_config.hz),
        "near_range_m": float(lidar_config.min_range_m),
        "far_range_m": float(lidar_config.max_range_m),
        "translation_m": list(lidar_config.pose_in_rig.position_m),
        "orientation_xyzw_ros": list(orientation_xyzw),
        "orientation_wxyz_isaac": list(quaternion_xyzw_to_wxyz(orientation_xyzw)),
        "mount_frame": "sensor_rig",
        "mount_semantics": "rig_relative",
        "world_pose_semantics": "sensor_rig_world_pose_composed_with_local_mount",
        "sensor_asset": "Example_Rotary",
        "sensor_asset_source": "locally authored OmniLidar schema; no USD reference",
        "scan_pattern_source": "scenario-derived local rotary attributes",
        "scan_type": "ROTARY",
        "rotation_direction": "CW",
        "horizontal_samples": horizontal_samples,
        "vertical_samples": vertical_samples,
        "vertical_fov_deg": [vertical_min_deg, vertical_max_deg],
        "elevation_angles_deg": elevations_deg,
        "pattern_firing_rate_hz": pattern_firing_rate_hz,
        "expected_points_per_scan": horizontal_samples * vertical_samples,
    }


def _timestamp_summary(reference: list[float], samples: list[float]) -> dict[str, float | int | None]:
    if not reference or not samples:
        return {"sample_count": len(samples), "mean_offset_s": None, "max_abs_offset_s": None}
    offsets = [min((value - ref for ref in reference), key=abs) for value in samples]
    return {
        "sample_count": len(samples),
        "mean_offset_s": sum(offsets) / len(offsets),
        "max_abs_offset_s": max(abs(value) for value in offsets),
    }


def _point_count_summary(samples: list[int]) -> dict[str, float | int | None]:
    if not samples:
        return {"sample_count": 0, "min_points": None, "max_points": None, "mean_points": None}
    return {
        "sample_count": len(samples),
        "min_points": min(samples),
        "max_points": max(samples),
        "mean_points": sum(samples) / len(samples),
    }


def _set_name_override(prim, name: str) -> None:
    from pxr import Sdf

    attr = prim.GetAttribute("isaac:nameOverride")
    if not attr:
        attr = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    attr.Set(name)


def _define_xform(stage, path: str, frame_name: str):
    from pxr import UsdGeom

    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    _set_name_override(prim, frame_name)
    return prim


def store_shell_spec(environment) -> dict[str, object]:
    """Return deterministic store-shell geometry and motivated fixture data."""

    ceiling_z = max(3.15, float(environment.shelf_height_m) + 0.95)
    shell_start_x = -1.0
    shell_end_x = float(environment.length_m) + 1.5
    shell_length = shell_end_x - shell_start_x
    shell_center_x = (shell_start_x + shell_end_x) / 2.0
    shell_half_width = float(environment.width_m) / 2.0 + 0.95
    boxes: list[dict[str, object]] = [
        {
            "name": "ceiling_slab",
            "center_m": (shell_center_x, 0.0, ceiling_z + 0.05),
            "size_m": (shell_length, shell_half_width * 2.0, 0.10),
            "kind": "ceiling",
        },
        {
            "name": "end_wall",
            "center_m": (shell_end_x, 0.0, ceiling_z / 2.0),
            "size_m": (0.14, shell_half_width * 2.0, ceiling_z),
            "kind": "wall",
        },
    ]
    for label, sign in (("left", -1.0), ("right", 1.0)):
        wall_y = sign * shell_half_width
        boxes.extend(
            [
                {
                    "name": f"side_wall_{label}",
                    "center_m": (shell_center_x, wall_y, ceiling_z / 2.0),
                    "size_m": (shell_length, 0.14, ceiling_z),
                    "kind": "wall",
                },
                {
                    "name": f"baseboard_{label}",
                    "center_m": (shell_center_x, wall_y - sign * 0.09, 0.12),
                    "size_m": (shell_length, 0.055, 0.24),
                    "kind": "baseboard",
                },
            ]
        )

    # Fine terrazzo control joints break up the broad floor plane without
    # becoming dark lane markers.  They sit only 2 mm above the finish plane.
    boxes.append(
        {
            "name": "floor_joint_longitudinal",
            "center_m": (shell_center_x, 0.0, 0.001),
            "size_m": (shell_length - 0.6, 0.012, 0.002),
            "kind": "floor_inlay",
        }
    )
    joint_x = shell_start_x + 1.2
    while joint_x < shell_end_x:
        boxes.append(
            {
                "name": f"floor_joint_cross_{len(boxes):03d}",
                "center_m": (round(joint_x, 4), 0.0, 0.001),
                "size_m": (0.012, 2.64, 0.002),
                "kind": "floor_inlay",
            }
        )
        joint_x += 1.2

    # Subtle alternating terrazzo pours break the broad floor reflection while
    # preserving a level, unobstructed calibrated sensor corridor.
    tile_x = shell_start_x + 0.60
    tile_row = 0
    while tile_x < shell_end_x:
        for tile_column, tile_y in enumerate((-0.88, 0.0, 0.88)):
            boxes.append(
                {
                    "name": f"floor_tile_{tile_row:02d}_{tile_column}",
                    "center_m": (round(tile_x, 4), tile_y, 0.0015),
                    "size_m": (1.176, 0.856, 0.0025),
                    "kind": "floor_tile_warm" if (tile_row + tile_column) % 3 else "floor_tile_cool",
                }
            )
        tile_row += 1
        tile_x += 1.2

    # A 0.6 x 1.2 m suspended-ceiling rhythm supplies believable scale while
    # remaining shallow enough to avoid intruding into the calibrated camera.
    grid_z = ceiling_z - 0.065
    tile_x = shell_start_x + 0.60
    ceiling_row = 0
    while tile_x < shell_end_x:
        tile_y = -2.10
        ceiling_column = 0
        while tile_y <= 2.10:
            boxes.append(
                {
                    "name": f"ceiling_tile_{ceiling_row:02d}_{ceiling_column:02d}",
                    "center_m": (round(tile_x, 4), round(tile_y, 4), ceiling_z - 0.052),
                    "size_m": (1.190, 0.590, 0.012),
                    "kind": "ceiling_tile_warm" if (ceiling_row + 2 * ceiling_column) % 5 else "ceiling_tile_cool",
                }
            )
            tile_y += 0.60
            ceiling_column += 1
        tile_x += 1.20
        ceiling_row += 1
    grid_y = -shell_half_width + 0.30
    while grid_y < shell_half_width:
        boxes.append(
            {
                "name": f"ceiling_rail_long_{len(boxes):03d}",
                "center_m": (shell_center_x, round(grid_y, 4), grid_z),
                "size_m": (shell_length, 0.007, 0.012),
                "kind": "ceiling_grid",
            }
        )
        grid_y += 0.60
    grid_x = shell_start_x + 0.60
    while grid_x < shell_end_x:
        boxes.append(
            {
                "name": f"ceiling_rail_cross_{len(boxes):03d}",
                "center_m": (round(grid_x, 4), 0.0, grid_z),
                "size_m": (0.007, shell_half_width * 2.0, 0.012),
                "kind": "ceiling_grid",
            }
        )
        grid_x += 1.20

    lights: list[dict[str, object]] = []
    fixture_index = 0
    fixture_x = 1.4
    while fixture_x <= float(environment.length_m) + 0.2:
        for fixture_y in (-1.05, 1.05):
            name = f"panel_{fixture_index:02d}"
            boxes.append(
                {
                    "name": name,
                    "center_m": (fixture_x, fixture_y, ceiling_z - 0.085),
                    "size_m": (0.92, 0.34, 0.025),
                    "kind": "light_panel",
                }
            )
            lights.append(
                {
                    "name": name,
                    "position_m": (fixture_x, fixture_y, ceiling_z - 0.11),
                    "width_m": 0.86,
                    "height_m": 0.29,
                    # Normalized USD area lights need more authored energy than
                    # the target illuminance value suggests.  This calibration
                    # keeps the 450 lux scenario readable without multiplying
                    # the number of fixtures or adding unmotivated point lights.
                    "intensity": float(environment.lighting_lux) * 18.0,
                    "color": (1.0, 0.96, 0.88),
                    # Aim each broad diffuser gently toward its adjacent shelf.
                    # The visible panel remains flush with the ceiling grid.
                    "rotation_rpy_deg": (-32.0 if fixture_y < 0.0 else 32.0, 0.0, 0.0),
                }
            )
            fixture_index += 1
        fixture_x += 2.4

    # Recessed modular under-shelf LED runs lift the merchandise plane.  Short
    # segments with bounded output variation avoid one repeated pool pattern
    # while the emitters remain shielded behind the shelf fascia.
    strip_index = 0
    for strip_y, strip_rotation in ((-1.50, -72.0), (1.50, 72.0)):
        # Each emitter sits below the next shelf board rather than crossing
        # the merchandise volume midway through a package.
        for strip_z in (0.72, 1.18, 1.64, 2.10):
            for section_index, section_center_x in enumerate((5.75, 17.75)):
                for segment_index, x_offset in enumerate((-4.05, -1.35, 1.35, 4.05)):
                    strip_x = section_center_x + x_offset
                    name = f"shelf_strip_{strip_index:03d}"
                    output_scale = (0.84, 1.06, 0.93, 1.12)[(segment_index + section_index) % 4]
                    boxes.append(
                        {
                            "name": name,
                            "center_m": (strip_x, strip_y, strip_z),
                            "size_m": (2.54, 0.014, 0.010),
                            "kind": "shelf_light",
                        }
                    )
                    lights.append(
                        {
                            "name": name,
                            "position_m": (strip_x, strip_y, strip_z - 0.012),
                            "width_m": 2.48,
                            "height_m": 0.022,
                            "intensity": float(environment.lighting_lux) * 3.0 * output_scale,
                            "color": (1.0, 0.905 + 0.008 * (segment_index % 2), 0.79 + 0.014 * (section_index % 2)),
                            "rotation_rpy_deg": (strip_rotation, 0.0, 0.0),
                        }
                    )
                    strip_index += 1

    # Small perforation slots on the camera-facing upright edges add real
    # fixture scale and break up the broad modular gray posts.
    for row_y, front_sign in ((-1.65, 1.0), (1.65, -1.0)):
        slot_y = row_y + front_sign * 0.276
        for bay_index in range(20):
            slot_x = 0.60 + bay_index * 1.20
            for slot_index, slot_z in enumerate((0.38, 0.70, 1.02, 1.34, 1.66, 1.98)):
                boxes.append(
                    {
                        "name": f"upright_slot_{'n' if row_y < 0 else 'p'}_{bay_index:02d}_{slot_index}",
                        "center_m": (slot_x, slot_y, slot_z),
                        "size_m": (0.020, 0.006, 0.058),
                        "kind": "upright_slot",
                    }
                )

    # Three deep refrigerated bays use real R5 catalog references behind
    # glass, with physical shelves, handles, and cool internal illumination.
    end_case_x = shell_end_x - 0.22
    asset_references: list[dict[str, object]] = []
    price_displays: list[dict[str, object]] = []
    case_stock = (
        ("milk_gallon", "milk_carton", "yogurt_cup", "sports_drink", "juice_citrus"),
        ("juice_citrus", "juice_berry", "juice_green", "juice_apple", "water_sky", "sports_drink"),
        ("icecream_tub", "soda_orbit", "soda_cherry", "soda_lime", "water_clear", "sports_drink"),
    )
    for case_index, case_y in enumerate((-1.35, 0.0, 1.35)):
        boxes.append(
            {
                "name": f"end_case_interior_{case_index}",
                "center_m": (end_case_x, case_y, 1.24),
                "size_m": (0.42, 1.14, 2.00),
                "kind": "case_interior",
            }
        )
        for edge_y in (-0.60, 0.60):
            boxes.append(
                {
                    "name": f"end_case_frame_{case_index}_{'l' if edge_y < 0 else 'r'}",
                    "center_m": (shell_end_x - 0.60, case_y + edge_y, 1.24),
                    "size_m": (0.075, 0.060, 2.08),
                    "kind": "case_frame",
                }
            )
        boxes.append(
            {
                "name": f"end_case_fill_{case_index}",
                "center_m": (shell_end_x - 0.60, case_y, 2.28),
                "size_m": (0.08, 1.20, 0.16),
                "kind": "light_panel",
            }
        )
        for door_index, door_y_offset in enumerate((-0.285, 0.285)):
            boxes.append(
                {
                    "name": f"end_case_glass_{case_index}_{door_index}",
                    "center_m": (shell_end_x - 0.645, case_y + door_y_offset, 1.24),
                    "size_m": (0.012, 0.525, 1.90),
                    "kind": "case_glass",
                }
            )
            boxes.append(
                {
                    "name": f"end_case_handle_{case_index}_{door_index}",
                    "center_m": (shell_end_x - 0.672, case_y + (-0.055 if door_index == 0 else 0.055), 1.26),
                    "size_m": (0.030, 0.022, 0.78),
                    "kind": "case_handle",
                }
            )
            boxes.append(
                {
                    "name": f"end_case_reflection_{case_index}_{door_index}",
                    "center_m": (shell_end_x - 0.654, case_y + door_y_offset - 0.11, 1.52),
                    "size_m": (0.007, 0.025, 0.82),
                    "kind": "case_glass_reflection",
                }
            )
        boxes.append(
            {
                "name": f"end_case_center_mullion_{case_index}",
                "center_m": (shell_end_x - 0.662, case_y, 1.24),
                "size_m": (0.038, 0.042, 1.98),
                "kind": "case_frame",
            }
        )
        boxes.append(
            {
                "name": f"end_case_toe_kick_{case_index}",
                "center_m": (shell_end_x - 0.49, case_y, 0.14),
                "size_m": (0.24, 1.08, 0.17),
                "kind": "case_frame",
            }
        )
        # Vertical door-edge practicals add a motivated highlight to the
        # glazing and keep late-aisle stock readable without lifting the
        # already bright floor or wall exposure.
        for edge_index, edge_y in enumerate((-0.535, 0.535)):
            boxes.append(
                {
                    "name": f"end_case_edge_light_{case_index}_{edge_index}",
                    "center_m": (shell_end_x - 0.675, case_y + edge_y, 1.24),
                    "size_m": (0.018, 0.016, 1.72),
                    "kind": "light_panel",
                }
            )
            lights.append(
                {
                    "name": f"end_case_edge_light_{case_index}_{edge_index}",
                    "position_m": (shell_end_x - 0.665, case_y + edge_y, 1.24),
                    "width_m": 1.68,
                    "height_m": 0.026,
                    "intensity": float(environment.lighting_lux) * 3.0,
                    "color": (0.88, 0.95, 1.0),
                    "rotation_rpy_deg": (0.0, 90.0, 0.0),
                }
            )
        for shelf_index, shelf_z in enumerate((0.34, 0.80, 1.26, 1.72)):
            boxes.append(
                {
                    "name": f"end_case_shelf_{case_index}_{shelf_index}",
                    "center_m": (shell_end_x - 0.38, case_y, shelf_z),
                    "size_m": (0.38, 1.08, 0.028),
                    "kind": "case_frame",
                }
            )
            for column, offset_y in enumerate((-0.4375, -0.2625, -0.0875, 0.0875, 0.2625, 0.4375)):
                key = case_stock[case_index][(shelf_index + column) % len(case_stock[case_index])]
                pose_offset = ((case_index * 5 + shelf_index * 3 + column) % 5) - 2
                asset_references.append(
                    {
                        "name": f"end_case_stock_{case_index}_{shelf_index}_{column}",
                        "asset_key": key,
                        # Keep stock clearly behind the front glass plane so
                        # silhouettes, frames, and reflections remain legible.
                        "position_xy_m": (shell_end_x - 0.46 - 0.003 * pose_offset, case_y + offset_y),
                        "support_z_m": shelf_z + 0.018 + 0.002 * abs(pose_offset),
                        "rotation_rpy_deg": (0.0, 0.0, 90.0 + 0.8 * pose_offset),
                        "scale_xyz": (-(1.10 + 0.012 * pose_offset), 1.08, 1.10 + 0.01 * pose_offset),
                    }
                )
        lights.append(
            {
                "name": f"end_case_fill_{case_index}",
                "position_m": (shell_end_x - 0.34, case_y, 1.34),
                "width_m": 1.58,
                "height_m": 1.02,
                "intensity": float(environment.lighting_lux) * 9.0,
                "color": (0.88, 0.95, 1.0),
                "rotation_rpy_deg": (0.0, 90.0, 0.0),
            }
        )

    # A compact right-side feature display creates a close, oblique packaging
    # view at the 9-second M1 pose while retaining over a metre of clear aisle.
    boxes.extend(
        [
            {
                "name": "hero_display_back",
                "center_m": (13.60, -1.355, 0.73),
                "size_m": (2.10, 0.045, 1.18),
                "kind": "display_wood",
            },
            {
                "name": "hero_display_base",
                "center_m": (13.60, -1.18, 0.12),
                "size_m": (2.10, 0.34, 0.18),
                "kind": "display_wood",
            },
        ]
    )
    hero_keys = (
        "cereal_sunrise", "juice_citrus", "chips_bag", "coffee_canister",
        "cereal_harvest", "sports_drink", "pasta_box", "can_tomato",
        "juice_berry", "sparkling_wine", "snack_wafer", "jam_jar",
        "tea_box", "rice_bag", "coffee_bag", "bread_loaf", "peanut_jar", "olive_oil",
    )
    hero_asset_index = 0
    for shelf_index, shelf_z in enumerate((0.42, 0.84, 1.26)):
        boxes.append(
            {
                "name": f"hero_display_shelf_{shelf_index}",
                "center_m": (13.60, -1.18, shelf_z),
                "size_m": (2.10, 0.34, 0.045),
                "kind": "shelf",
            }
        )
        for column, product_x in enumerate((12.75, 13.09, 13.43, 13.77, 14.11, 14.45)):
            key = hero_keys[hero_asset_index]
            hero_asset_index += 1
            asset_references.append(
                {
                    "name": f"hero_stock_{shelf_index}_{column}",
                    "asset_key": key,
                    "position_xy_m": (product_x, -1.05),
                    "support_z_m": shelf_z + 0.025,
                    "rotation_rpy_deg": (0.0, 0.0, 42.0 - 2.0 * column),
                    "scale_xyz": (1.08, 1.08, 1.08),
                }
            )
        asset_references.extend(
            [
                {
                    "name": f"hero_price_{shelf_index}",
                    "asset_key": "price_display",
                    "position_xy_m": (13.60, -0.995),
                    "support_z_m": shelf_z + 0.032,
                    "rotation_rpy_deg": (0.0, 0.0, 0.0),
                    "scale_xyz": (1.0, 1.0, 1.0),
                },
                {
                    "name": f"hero_divider_{shelf_index}",
                    "asset_key": "shelf_divider",
                    "position_xy_m": (13.60, -1.17),
                    "support_z_m": shelf_z + 0.024,
                    "rotation_rpy_deg": (0.0, 0.0, 0.0),
                    "scale_xyz": (1.0, 1.0, 1.0),
                },
            ]
        )
    asset_references.extend(
        [
            {
                "name": "hero_market_sign",
                "asset_key": "promo_market_sign",
                "position_xy_m": (13.60, -1.15),
                "support_z_m": 1.42,
                # The R5 panel's authored UV orientation reads horizontally
                # reversed from its +Y front.  A deterministic X reflection
                # preserves the physical pose while making the category text
                # readable from the approaching camera.
                "rotation_rpy_deg": (0.0, 0.0, 42.0),
                "scale_xyz": (-0.78, 0.78, 0.78),
            },
            {
                "name": "end_market_sign",
                "asset_key": "promo_market_sign",
                "position_xy_m": (shell_end_x - 0.72, 0.0),
                "support_z_m": 2.32,
                "rotation_rpy_deg": (0.0, 0.0, 90.0),
                "scale_xyz": (-1.55, 1.0, 1.05),
            },
        ]
    )

    # A small oblique riser presents four package fronts at useful pixel size
    # in the unchanged 9-second camera pose.  Its aisle edge remains beyond
    # y=-0.51 and does not cross the calibrated centre path.
    boxes.extend(
        [
            {
                "name": "hero_focus_plinth",
                "center_m": (12.20, -0.950, 0.73),
                "size_m": (0.55, 0.83, 0.70),
                "kind": "display_wood",
            },
            {
                "name": "hero_focus_top",
                "center_m": (12.20, -0.950, 1.095),
                "size_m": (0.59, 0.87, 0.030),
                "kind": "shelf",
            },
            {
                "name": "hero_focus_price_rail",
                "center_m": (11.932, -0.9175, 1.138),
                "size_m": (0.020, 0.825, 0.055),
                "kind": "price_rail",
            },
        ]
    )
    focus_specs = (
        ("cereal_sunrise", 12.11, -0.610, 74.0, 1.20, 1.115),
        ("cereal_harvest", 12.15, -0.815, 77.0, 1.18, 1.119),
        ("cereal_grain", 12.12, -1.020, 72.0, 1.18, 1.115),
        ("cereal_berry", 12.18, -1.225, 76.0, 1.18, 1.117),
    )
    for focus_index, (key, product_x, product_y, yaw, scale, support_z) in enumerate(focus_specs):
        asset_references.append(
            {
                "name": f"hero_focus_stock_{focus_index}",
                "asset_key": key,
                "position_xy_m": (product_x, product_y),
                "support_z_m": support_z,
                "rotation_rpy_deg": (0.0, 0.0, yaw),
                # These oblique facings expose the authored reverse UV side;
                # reflect X exactly as the validated market sign does so text
                # reads normally from the approaching camera.
                "scale_xyz": (-scale, scale, scale),
            }
        )
    for price_index, (sku_key, _, price_y, _, _, _) in enumerate(focus_specs):
        label = HERO_PRICE_LABELS[sku_key]
        price_displays.append(
            {
                "name": f"hero_focus_price_{price_index}",
                "sku_key": sku_key,
                "display_name": label["display_name"],
                "unit_price": label["unit_price"],
                "asset_path": scene_price_display_path(sku_key),
                "position_xy_m": (11.915, price_y),
                "support_z_m": 1.115,
                "rotation_rpy_deg": (0.0, 0.0, 90.0),
                "scale_xyz": (-0.72, 0.72, 0.72),
            }
        )
    # A second eye-level product group creates the requested close shelf beat
    # near 17 seconds.  It uses the same measured side clearance as the 9 s
    # group, so the M1 rig and LiDAR centerline remain unobstructed.
    boxes.extend(
        [
            {
                "name": "late_focus_plinth",
                "center_m": (21.35, -0.950, 0.73),
                "size_m": (0.55, 0.83, 0.70),
                "kind": "display_wood",
            },
            {
                "name": "late_focus_top",
                "center_m": (21.35, -0.950, 1.095),
                "size_m": (0.59, 0.87, 0.030),
                "kind": "shelf",
            },
            {
                "name": "late_focus_price_rail",
                "center_m": (21.082, -0.9175, 1.138),
                "size_m": (0.020, 0.825, 0.055),
                "kind": "price_rail",
            },
        ]
    )
    late_focus_specs = (
        ("coffee_bag", 21.26, -0.610, 74.0, 1.20, 1.115),
        ("tea_box", 21.30, -0.815, 77.0, 1.24, 1.119),
        ("coffee_canister", 21.27, -1.020, 72.0, 1.20, 1.115),
        ("snack_wafer", 21.33, -1.225, 76.0, 1.16, 1.117),
    )
    for focus_index, (key, product_x, product_y, yaw, scale, support_z) in enumerate(late_focus_specs):
        asset_references.append(
            {
                "name": f"late_focus_stock_{focus_index}",
                "asset_key": key,
                "position_xy_m": (product_x, product_y),
                "support_z_m": support_z,
                "rotation_rpy_deg": (0.0, 0.0, yaw),
                "scale_xyz": (-scale, scale, scale),
            }
        )
    for price_index, (sku_key, _, price_y, _, _, _) in enumerate(late_focus_specs):
        label = HERO_PRICE_LABELS[sku_key]
        price_displays.append(
            {
                "name": f"late_focus_price_{price_index}",
                "sku_key": sku_key,
                "display_name": label["display_name"],
                "unit_price": label["unit_price"],
                "asset_path": scene_price_display_path(sku_key),
                "position_xy_m": (21.065, price_y),
                "support_z_m": 1.115,
                "rotation_rpy_deg": (0.0, 0.0, 90.0),
                "scale_xyz": (-0.72, 0.72, 0.72),
            }
        )
    basket_specs = ((15.12, -1.18, 8.0), (15.68, -1.05, 2.0))
    for basket_index, (basket_x, basket_y, basket_yaw) in enumerate(basket_specs):
        asset_references.append(
            {
                "name": f"store_use_basket_{basket_index}",
                "asset_key": "wicker_basket",
                "position_xy_m": (basket_x, basket_y),
                "support_z_m": 0.02,
                "rotation_rpy_deg": (0.0, 0.0, basket_yaw),
                "scale_xyz": (1.0, 1.0, 1.0),
            }
        )
    # One suspended category blade breaks the tunnel symmetry and provides a
    # familiar retail wayfinding cue while clearing both camera and LiDAR.
    for rod_index, rod_y in enumerate((-0.88, -0.32)):
        boxes.append(
            {
                "name": f"category_sign_rod_{rod_index}",
                "center_m": (7.60, rod_y, 2.63),
                "size_m": (0.016, 0.016, 0.34),
                "kind": "case_frame",
            }
        )
    asset_references.append(
        {
            "name": "suspended_category_sign",
            "asset_key": "promo_market_sign",
            "position_xy_m": (7.60, -0.60),
            "support_z_m": 2.18,
            "rotation_rpy_deg": (0.0, 0.0, 90.0),
            "scale_xyz": (-0.88, 0.88, 0.88),
        }
    )
    return {
        "ceiling_height_m": ceiling_z,
        "boxes": tuple(boxes),
        "lights": tuple(lights),
        "asset_references": tuple(asset_references),
        "price_displays": tuple(price_displays),
        # A restrained cool dome represents secondary bounce from the finished
        # store shell and prevents black shelf cavities under the area lights.
        "ambient_intensity": float(environment.lighting_lux) * 1.2,
    }


def runtime_dense_stock_references(layout, environment) -> tuple[dict[str, object], ...]:
    """Fill only measured edge gaps in existing front-facing SKU blocks."""

    from simulator.environment.retail_catalog import load_retail_catalog

    catalog = load_retail_catalog(environment.asset_manifest_path)
    references: list[dict[str, object]] = []
    for shelf in layout.primitives:
        if shelf.kind != "shelf" or not shelf.name.startswith("shelf_r"):
            continue
        _, row_part, bay_part, level_part = shelf.name.split("_")
        row_index = int(row_part[1:])
        bay_index = int(bay_part[1:])
        level_index = int(level_part[1:])
        identity_marker = f"/r{row_index}/b{bay_index}/l{level_index}/d0/"
        existing = sorted(
            (asset for asset in layout.assets if identity_marker in asset.semantic_id),
            key=lambda asset: asset.position_m[0],
        )
        if not existing:
            continue
        occupied: list[tuple[float, float, object]] = []
        for asset in existing:
            record = catalog.by_key(asset.asset_key)
            half_width = record.dimensions_m[0] * abs(asset.scale_xyz[0]) / 2.0
            occupied.append((asset.position_m[0] - half_width, asset.position_m[0] + half_width, asset))
        shelf_min = shelf.center_m[0] - shelf.size_m[0] / 2.0 + environment.edge_margin_m
        shelf_max = shelf.center_m[0] + shelf.size_m[0] / 2.0 - environment.edge_margin_m
        front_sign = 1.0 if shelf.center_m[1] < 0.0 else -1.0
        for side_name, source_asset in (("left", occupied[0][2]), ("right", occupied[-1][2])):
            source_record = catalog.by_key(source_asset.asset_key)
            compatible_records = tuple(
                candidate
                for candidate in catalog.by_category(source_record.category)
                if candidate.intended_support == "shelf"
                and 0.78 <= candidate.dimensions_m[0] / source_record.dimensions_m[0] <= 1.0
                and 0.78 <= candidate.dimensions_m[1] / source_record.dimensions_m[1] <= 1.0
                and 0.72 <= candidate.dimensions_m[2] / source_record.dimensions_m[2] <= 1.0
            ) or (source_record,)
            cursor = occupied[0][0] if side_name == "left" else occupied[-1][1]
            for fill_index in range(2):
                variation = ((row_index * 17 + bay_index * 7 + level_index * 3 + fill_index) % 5) - 2
                record = compatible_records[
                    (row_index * 13 + bay_index * 7 + level_index * 5 + fill_index * 3) % len(compatible_records)
                ]
                scale = 0.98 + 0.012 * variation
                half_width = record.dimensions_m[0] * scale / 2.0
                if side_name == "left":
                    product_x = cursor - environment.facing_gap_m - half_width
                    if product_x - half_width < shelf_min - 1e-9:
                        break
                    cursor = product_x - half_width
                else:
                    product_x = cursor + environment.facing_gap_m + half_width
                    if product_x + half_width > shelf_max + 1e-9:
                        break
                    cursor = product_x + half_width
                product_y = shelf.center_m[1] + front_sign * (
                    environment.shelf_depth_m / 2.0
                    - record.dimensions_m[1] * scale / 2.0
                    - environment.edge_margin_m
                ) + front_sign * (0.006 * abs(variation))
                references.append(
                    {
                        "name": f"dense_stock_r{row_index}_b{bay_index}_l{level_index}_{side_name}_{fill_index}",
                        "asset_key": record.asset_key,
                        "position_xy_m": (product_x, product_y),
                        "support_z_m": shelf.center_m[2] + shelf.size_m[2] / 2.0,
                        "rotation_rpy_deg": (
                            0.0,
                            0.0,
                            (0.0 if shelf.center_m[1] < 0.0 else 180.0) + 0.65 * variation,
                        ),
                        "scale_xyz": (scale, scale, scale),
                    }
                )
        # Irregularly spaced real price displays replace some blank rail runs.
        shelf_identity_marker = f"/r{row_index}/b{bay_index}/l{level_index}"
        existing_price_display = any(
            asset.asset_key == "price_display" and shelf_identity_marker in asset.semantic_id
            for asset in layout.assets
        )
        if not existing_price_display and (row_index * 11 + bay_index * 5 + level_index * 3) % 7 in (0, 1):
            price = catalog.by_key("price_display")
            price_offset = (-0.22, 0.12, 0.28)[(bay_index + level_index) % 3]
            references.append(
                {
                    "name": f"dense_price_r{row_index}_b{bay_index}_l{level_index}",
                    "asset_key": "price_display",
                    "position_xy_m": (
                        shelf.center_m[0] + price_offset,
                        shelf.center_m[1] + front_sign * (environment.shelf_depth_m / 2.0 + price.dimensions_m[1] / 2.0),
                    ),
                    "support_z_m": shelf.center_m[2] + shelf.size_m[2] / 2.0,
                    "rotation_rpy_deg": (0.0, 0.0, 0.0 if shelf.center_m[1] < 0.0 else 180.0),
                    "scale_xyz": (0.82, 0.82, 0.82),
                }
            )
    # Dense shelf dressing is derived after the authored context geometry, so
    # reject any candidate whose oriented footprint penetrates a context box.
    # This keeps later focus displays and other scene dressing authoritative
    # without relying on one-off bay exclusions.
    context_boxes = store_shell_spec(environment)["boxes"]
    collision_free: list[dict[str, object]] = []
    for reference in references:
        record = catalog.by_key(reference["asset_key"])
        scale = reference["scale_xyz"]
        width = record.dimensions_m[0] * abs(scale[0])
        depth = record.dimensions_m[1] * abs(scale[1])
        yaw = math.radians(reference["rotation_rpy_deg"][2])
        axis_x = (math.cos(yaw), math.sin(yaw))
        axis_y = (-math.sin(yaw), math.cos(yaw))
        center_x, center_y = reference["position_xy_m"]
        corners = tuple(
            (
                center_x + sx * width * axis_x[0] / 2.0 + sy * depth * axis_y[0] / 2.0,
                center_y + sx * width * axis_x[1] / 2.0 + sy * depth * axis_y[1] / 2.0,
            )
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
        )
        z_min = reference["support_z_m"]
        z_max = z_min + record.dimensions_m[2] * abs(scale[2])
        penetrates_context = False
        for box in context_boxes:
            box_x, box_y, box_z = box["center_m"]
            box_width, box_depth, box_height = box["size_m"]
            if min(z_max, box_z + box_height / 2.0) - max(z_min, box_z - box_height / 2.0) <= 1e-4:
                continue
            box_corners = (
                (box_x - box_width / 2.0, box_y - box_depth / 2.0),
                (box_x - box_width / 2.0, box_y + box_depth / 2.0),
                (box_x + box_width / 2.0, box_y - box_depth / 2.0),
                (box_x + box_width / 2.0, box_y + box_depth / 2.0),
            )
            separated = False
            for axis in (axis_x, axis_y, (1.0, 0.0), (0.0, 1.0)):
                reference_interval = [point[0] * axis[0] + point[1] * axis[1] for point in corners]
                box_interval = [point[0] * axis[0] + point[1] * axis[1] for point in box_corners]
                if (
                    min(max(reference_interval), max(box_interval))
                    - max(min(reference_interval), min(box_interval))
                    <= 1e-4
                ):
                    separated = True
                    break
            if not separated:
                penetrates_context = True
                break
        if not penetrates_context:
            collision_free.append(reference)
    return tuple(collision_free)


def _build_scene_price_display(stage, price_spec: dict[str, object]) -> None:
    """Reference one deterministic per-SKU electronic shelf label."""

    from pxr import Gf, Sdf, UsdGeom

    asset_path = Path(price_spec["asset_path"]).resolve()
    if not asset_path.is_file():
        raise FileNotFoundError(
            f"Missing generated scene price display for {price_spec['sku_key']}: {asset_path}"
        )
    prim = stage.DefinePrim(
        f"/World/GroceryAisle/store_context/{price_spec['name']}",
        "Xform",
    )
    if not prim.GetReferences().AddReference(str(asset_path).replace("\\", "/")):
        raise RuntimeError(f"Failed to reference scene price display {asset_path}")
    prim.SetInstanceable(True)
    x, y = price_spec["position_xy_m"]
    scale_x, scale_y, scale_z = price_spec["scale_xyz"]
    # The generated display preserves the catalog fixture's measured envelope.
    z = float(price_spec["support_z_m"]) + 0.065 * abs(float(scale_z)) / 2.0
    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(float(x), float(y), z))
    api.SetRotate(
        Gf.Vec3f(*price_spec["rotation_rpy_deg"]),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    api.SetScale(Gf.Vec3f(float(scale_x), float(scale_y), float(scale_z)))
    prim.CreateAttribute("grocery:asset_key", Sdf.ValueTypeNames.String).Set("price_display")
    prim.CreateAttribute("grocery:sku_key", Sdf.ValueTypeNames.String).Set(price_spec["sku_key"])
    prim.CreateAttribute("grocery:display_name", Sdf.ValueTypeNames.String).Set(price_spec["display_name"])
    prim.CreateAttribute("grocery:unit_price", Sdf.ValueTypeNames.String).Set(price_spec["unit_price"])
    prim.CreateAttribute("grocery:semantic_id", Sdf.ValueTypeNames.String).Set(
        f"scene_context/{price_spec['name']}/price_display/{price_spec['sku_key']}"
    )


def _build_store_shell(stage, environment, builder, layout=None) -> int:
    from pxr import Gf, UsdGeom, UsdLux

    spec = store_shell_spec(environment)
    for box in spec["boxes"]:
        builder.build_static_box(
            f"/World/GroceryAisle/store_shell/{box['kind']}/{box['name']}",
            box["center_m"],
            box["size_m"],
            box["kind"],
        )
    for light_spec in spec["lights"]:
        light = UsdLux.RectLight.Define(
            stage,
            f"/World/GroceryAisle/lighting/{light_spec['name']}",
        )
        light.GetWidthAttr().Set(light_spec["width_m"])
        light.GetHeightAttr().Set(light_spec["height_m"])
        light.GetIntensityAttr().Set(light_spec["intensity"])
        light.GetColorAttr().Set(Gf.Vec3f(*light_spec["color"]))
        light.GetNormalizeAttr().Set(True)
        light_api = UsdGeom.XformCommonAPI(light)
        light_api.SetTranslate(Gf.Vec3d(*light_spec["position_m"]))
        light_api.SetRotate(
            Gf.Vec3f(*light_spec["rotation_rpy_deg"]),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )
    from simulator.environment.retail_catalog import load_retail_catalog

    catalog = load_retail_catalog(environment.asset_manifest_path)
    asset_references = list(spec["asset_references"])
    if layout is not None:
        asset_references.extend(runtime_dense_stock_references(layout, environment))
    for asset_spec in asset_references:
        record = catalog.by_key(asset_spec["asset_key"])
        x, y = asset_spec["position_xy_m"]
        z = asset_spec["support_z_m"] + record.dimensions_m[2] * asset_spec["scale_xyz"][2] / 2.0
        builder.build_catalog_asset(
            f"/World/GroceryAisle/store_context/{asset_spec['name']}",
            record,
            (x, y, z),
            asset_spec["rotation_rpy_deg"],
            asset_spec["scale_xyz"],
            f"scene_context/{asset_spec['name']}/{record.asset_key}",
        )
    for price_spec in spec["price_displays"]:
        _build_scene_price_display(stage, price_spec)
    ambient = UsdLux.DomeLight.Define(stage, "/World/GroceryAisle/lighting/ambient_bounce")
    ambient.GetIntensityAttr().Set(spec["ambient_intensity"])
    ambient.GetColorAttr().Set(Gf.Vec3f(0.92, 0.95, 1.0))
    return (
        len(spec["boxes"])
        + len(spec["lights"])
        + len(asset_references)
        + len(spec["price_displays"])
        + 1
    )


def _build_world(stage, scenario):
    from pxr import UsdGeom

    from simulator.environment.aisle_builder import build_aisle_layout
    from simulator.environment.isaac_builder import IsaacAisleBuilder

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)

    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    _set_name_override(world, "sim_world")
    layout = build_aisle_layout(scenario.environment)
    builder = IsaacAisleBuilder(stage=stage)
    primitive_count = builder.build(layout, "/World/GroceryAisle")
    primitive_count += _build_store_shell(stage, scenario.environment, builder, layout)

    return layout, primitive_count


def _build_sensor_rig(stage, scenario):
    from pxr import Gf, Sdf, UsdGeom
    from simulator.sensors.transforms import camera_usd_quaternion, rpy_deg_from_quaternion

    rig_path = "/World/SensorRig"
    camera_link_path = "/World/SensorRig/camera_link"
    camera_path = "/World/SensorRig/camera_link/camera_optical_frame"
    lidar_path = "/World/SensorRig/lidar_link"

    rig = _define_xform(stage, rig_path, "sensor_rig")
    camera_link = _define_xform(stage, camera_link_path, "camera_link")
    # The LiDAR path is intentionally left undefined here.  Lidar.create()
    # authors the real OmniLidar prim at this path, including the RTX sensor
    # schema, so it must not be shadowed by a plain Xform.

    rig_api = UsdGeom.XformCommonAPI(rig)
    rig_api.SetTranslate(Gf.Vec3d(*scenario.trajectory.start_position_m))
    rig_api.SetRotate(Gf.Vec3f(0.0, 0.0, scenario.trajectory.yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera_api = UsdGeom.XformCommonAPI(camera_link)
    camera_api.SetTranslate(Gf.Vec3d(*scenario.camera.pose_in_rig.position_m))
    camera_api.SetRotate(Gf.Vec3f(*scenario.camera.pose_in_rig.rpy_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # Isaac cameras look along local -Z.  This rotation makes the optical
    # axis point down the aisle (+X), with image up aligned to world +Z.
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_api = UsdGeom.XformCommonAPI(camera.GetPrim())
    camera_api.SetRotate(Gf.Vec3f(*rpy_deg_from_quaternion(camera_usd_quaternion())), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955 * scenario.camera.height_px / scenario.camera.width_px)
    focal_length_mm = 20.955 / (2.0 * math.tan(math.radians(scenario.camera.horizontal_fov_deg) / 2.0))
    camera.GetFocalLengthAttr().Set(focal_length_mm)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(scenario.camera.near_m, scenario.camera.far_m))
    _set_name_override(camera.GetPrim(), "camera_optical_frame")

    # Author explicit frame metadata for consumers that inspect the USD stage.
    stage.GetPrimAtPath(rig_path).CreateAttribute("grocery:frame_id", Sdf.ValueTypeNames.String).Set("sensor_rig")
    stage.GetPrimAtPath(camera_path).CreateAttribute("grocery:frame_id", Sdf.ValueTypeNames.String).Set("camera_optical_frame")
    return rig, camera_path, lidar_path


def _create_clock_graph():
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph_path = "/GroceryROS2Graph"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def _create_camera_graph(camera_path: str, width: int, height: int, fps: float):
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    step = max(1, round(60.0 / float(fps)))
    graph_path = "/GroceryCameraGraph"
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "Rgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "Rgb.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("CreateRenderProduct.inputs:width", int(width)),
                ("CreateRenderProduct.inputs:height", int(height)),
                ("Rgb.inputs:frameId", "camera_optical_frame"),
                ("Rgb.inputs:topicName", "/sim/camera/rgb/image_raw"),
                ("Rgb.inputs:type", "rgb"),
                ("Rgb.inputs:frameSkipCount", step - 1),
                # Native 1080p payloads can briefly outpace Zenoh delivery;
                # retain a bounded publisher history rather than silently
                # throwing away a frame at the helper's default depth of 10.
                ("Rgb.inputs:queueSize", 128),
                ("CameraInfo.inputs:frameId", "camera_optical_frame"),
                ("CameraInfo.inputs:topicName", "/sim/camera/rgb/camera_info"),
                ("CameraInfo.inputs:frameSkipCount", step - 1),
                ("CameraInfo.inputs:queueSize", 128),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    return {
        "requested_fps": float(fps),
        "frame_skip_count": step - 1,
        "effective_fps": 60.0 / step,
        "publisher_queue_size": 128,
        "mode": "ros2_camera_helper_frameSkipCount",
    }


def _create_lidar(lidar_path: str, lidar_config, topic: str):
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
    from pxr import Sdf, Vt

    spec = lidar_runtime_spec(lidar_config)
    # Construct the native OmniLidar schema directly.  Lidar.create(config=...)
    # resolves a downloadable reference even when all scan attributes are
    # supplied; the direct constructor stays local and deterministic.
    lidar = Lidar(
        lidar_path,
        accumulate_outputs=True,
        tick_rate=spec["tick_rate_hz"],
        translations=[spec["translation_m"]],
        orientations=[spec["orientation_wxyz_isaac"]],
        attributes={
            "omni:sensor:Core:scanType": spec["scan_type"],
            "omni:sensor:Core:rotationDirection": spec["rotation_direction"],
            "omni:sensor:Core:scanRateBaseHz": int(round(spec["tick_rate_hz"])),
            "omni:sensor:Core:nearRangeM": spec["near_range_m"],
            "omni:sensor:Core:farRangeM": spec["far_range_m"],
            "omni:sensor:Core:patternFiringRateHz": spec["pattern_firing_rate_hz"],
            "omni:sensor:Core:numberOfChannels": spec["vertical_samples"],
            "omni:sensor:Core:numberOfEmitters": spec["vertical_samples"],
        },
    )
    prim = lidar.prims[0]
    # Replicator's generic attribute helper expands a Python list into the Vt
    # array constructor as positional arguments.  Set typed USD arrays on the
    # schema prim directly so all 32 channels reach the renderer unchanged.
    prim.GetAttribute("omni:sensor:Core:emitterState:s001:azimuthDeg").Set(
        Vt.FloatArray([0.0] * spec["vertical_samples"])
    )
    prim.GetAttribute("omni:sensor:Core:emitterState:s001:channelId").Set(
        Vt.UIntArray(range(1, spec["vertical_samples"] + 1))
    )
    prim.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Set(
        Vt.FloatArray(spec["elevation_angles_deg"])
    )
    prim.GetAttribute("omni:sensor:Core:emitterState:s001:fireTimeNs").Set(
        Vt.UIntArray([0] * spec["vertical_samples"])
    )
    name_override = prim.GetAttribute("isaac:nameOverride")
    if not name_override:
        name_override = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    name_override.Set("lidar_link")
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName=topic,
        frameId="lidar_link",
    )
    return lidar, sensor


def _publish_ground_truth(publisher, sample, timestamp_s: float):
    from geometry_msgs.msg import PoseStamped
    from simulator.ros.topic_contract import GROUND_TRUTH_FRAME

    message = PoseStamped()
    message.header.stamp = _sim_time_message(timestamp_s)
    message.header.frame_id = GROUND_TRUTH_FRAME
    message.pose.position.x, message.pose.position.y, message.pose.position.z = sample.position_m
    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = sample.orientation_xyzw
    publisher.publish(message)


def _publish_tf(publisher, sample, timestamp_s: float, scenario) -> None:
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from simulator.ros.topic_contract import FRAMES

    stamp = _sim_time_message(timestamp_s)
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = FRAMES["sim_world"]
    message.child_frame_id = FRAMES["truth_sensor_rig"]
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = sample.position_m
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = sample.orientation_xyzw

    camera = TransformStamped()
    camera.header.stamp = stamp
    camera.header.frame_id = FRAMES["sensor_rig"]
    camera.child_frame_id = FRAMES["camera_link"]
    camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z = scenario.camera.pose_in_rig.position_m
    camera_rotation = camera_mount_orientation(sample, scenario.camera)
    camera.transform.rotation.x, camera.transform.rotation.y, camera.transform.rotation.z, camera.transform.rotation.w = camera_rotation
    publisher.publish(TFMessage(transforms=[message, camera]))


def _publish_static_tf(broadcaster, scenario) -> None:
    from geometry_msgs.msg import TransformStamped

    from simulator.ros.topic_contract import FRAMES
    from simulator.sensors.transforms import camera_optical_quaternion, quaternion_from_rpy_deg

    transforms = []

    def add(parent: str, child: str, translation, rotation) -> None:
        message = TransformStamped()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = translation
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = rotation
        transforms.append(message)

    add(FRAMES["camera_link"], FRAMES["camera_optical"], (0.0, 0.0, 0.0), camera_optical_quaternion())
    add(FRAMES["sensor_rig"], FRAMES["lidar_link"], scenario.lidar.pose_in_rig.position_m, quaternion_from_rpy_deg(*scenario.lidar.pose_in_rig.rpy_deg))
    broadcaster.sendTransform(transforms)


def run(args: argparse.Namespace) -> dict[str, object]:
    ros_environment = _ensure_bundled_ros_environment()

    from isaacsim import SimulationApp

    from simulator.config.loader import load_scenario
    from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory
    from simulator.ros.topic_contract import FRAMES, TOPICS
    from simulator.sensors.noise import NoiseConfig
    from simulator.capture.stamp_digest import stamp_sequence_sha256

    if args.frames < 0:
        raise ValueError("--frames must be non-negative")
    simulation_dt_s = 1.0 / 60.0
    paced_wall_period_s = _paced_wall_period_s(args.realtime, args.realtime_factor, simulation_dt_s)
    scenario = load_scenario(args.scenario)
    trajectory_cls = WalkingTrajectory if scenario.trajectory.name.lower() == "walking" else StraightTrajectory
    trajectory = trajectory_cls(scenario.trajectory)
    scenario_path = Path(args.scenario).resolve()
    scenario_links = json.loads(scenario_path.read_text(encoding="utf-8"))
    trajectory_config_path = (scenario_path.parent / scenario_links["trajectory"]).resolve()
    frames = args.frames if args.frames > 0 else max(1, math.ceil(scenario.trajectory.duration_s * 60.0))
    from simulator.runtime.representative_capture import parse_capture_frames, validate_capture_dimensions

    selected_capture_frames = parse_capture_frames(args.capture_frames, frames)
    if bool(args.capture_dir) != bool(selected_capture_frames):
        raise ValueError("--capture-dir and a non-empty --capture-frames list must be supplied together")
    validate_capture_dimensions(args.capture_width, args.capture_height, args.capture_rt_subframes)
    camera_head_path = camera_head_output_path(args, bool(selected_capture_frames))
    camera_head_artifact = write_camera_head_transforms(
        camera_head_path,
        trajectory,
        scenario,
        trajectory_config_path,
    )
    noise_config = NoiseConfig.from_mapping(scenario.sensor_overrides.get("noise"))

    print("[grocery-runtime] starting SimulationApp", flush=True)
    simulation_app = SimulationApp({"renderer": args.renderer, "headless": bool(args.headless)})
    node = None
    executor = None
    executor_thread = None
    representative_capture = None
    runtime_ok = False
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager

        if not args.capture_only:
            app_utils.enable_extension("isaacsim.ros2.bridge")
            simulation_app.update()
            if not app_utils.is_extension_enabled("isaacsim.ros2.bridge"):
                raise RuntimeError("isaacsim.ros2.bridge failed during extension startup")
            print("[grocery-runtime] ROS 2 bridge loaded", flush=True)
        stage_utils.set_stage_units(meters_per_unit=1.0)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        print("[grocery-runtime] building aisle USD", flush=True)
        layout, primitive_count = _build_world(stage, scenario)
        print("[grocery-runtime] building sensor rig USD", flush=True)
        _, camera_path, lidar_path = _build_sensor_rig(stage, scenario)
        if selected_capture_frames:
            from simulator.runtime.representative_capture import RepresentativeFrameCapture

            representative_capture = RepresentativeFrameCapture(
                Path(args.capture_dir),
                camera_path,
                (args.capture_width, args.capture_height),
                selected_capture_frames,
                args.capture_rt_subframes,
                seed=scenario.seed,
                provenance=_capture_provenance(args, scenario, camera_head_artifact),
            )
        if args.capture_only:
            print("[grocery-runtime] initializing capture-only simulation", flush=True)
            SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
            simulation_app.update()
            app_utils.play()
            from omni.timeline import get_timeline_interface
            from pxr import Gf, UsdGeom
            from simulator.sensors.transforms import rpy_deg_from_quaternion

            timeline = get_timeline_interface()
            rig_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/SensorRig"))
            camera_link_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/SensorRig/camera_link"))
            last_timestamp_s = None
            render_phase_offsets_s = []
            simulation_wall_started = time.perf_counter()
            for frame in range(frames):
                wall_start = time.perf_counter()
                timeline_before_s = float(timeline.get_current_time())
                commanded_timestamp_s = timeline_before_s + simulation_dt_s
                commanded_sample = trajectory.sample(commanded_timestamp_s)
                rig_api.SetTranslate(Gf.Vec3d(*commanded_sample.position_m))
                rig_api.SetRotate(
                    Gf.Vec3f(*rpy_deg_from_quaternion(commanded_sample.orientation_xyzw)),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                camera_link_api.SetRotate(
                    Gf.Vec3f(*rpy_deg_from_quaternion(camera_mount_orientation(commanded_sample, scenario.camera))),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                simulation_app.update()
                timestamp_s = float(timeline.get_current_time())
                if last_timestamp_s is not None and timestamp_s <= last_timestamp_s:
                    raise RuntimeError("Isaac timeline did not advance during capture-only execution")
                render_phase_offsets_s.append(timestamp_s - commanded_timestamp_s)
                sample = trajectory.sample(timestamp_s)
                if representative_capture is not None and frame in selected_capture_frames:
                    representative_capture.capture(
                        frame,
                        timestamp_s,
                        camera_optical_world_pose(sample, scenario.camera),
                        timeline,
                    )
                last_timestamp_s = timestamp_s
                if paced_wall_period_s is not None:
                    time.sleep(max(0.0, paced_wall_period_s - (time.perf_counter() - wall_start)))

            simulation_wall_elapsed_s = time.perf_counter() - simulation_wall_started

            capture_result = None
            if representative_capture is not None:
                capture_result = representative_capture.finalize()
                representative_capture = None
            result = {
                "runtime": "isaac_sim",
                "isaac_version": "6.1.0",
                "mode": "representative_capture_only",
                "scenario": scenario.name,
                "frames_simulated": frames,
                "primitive_count": primitive_count,
                "product_count": len(layout.products),
                "clock_source": "Isaac timeline current_time",
                "representative_capture": capture_result,
                "timestamp_phase": {
                    "policy": "command_rig_before_update_capture_after_update",
                    "render_phase_offset_mean_s": (
                        None if not render_phase_offsets_s else sum(render_phase_offsets_s) / len(render_phase_offsets_s)
                    ),
                    "render_phase_offset_max_abs_s": (
                        None if not render_phase_offsets_s else max(abs(value) for value in render_phase_offsets_s)
                    ),
                },
                "last_rig_position_m": list(sample.position_m),
                "last_rig_orientation_xyzw": list(sample.orientation_xyzw),
                "last_camera_mount_orientation_xyzw": list(camera_mount_orientation(sample, scenario.camera)),
                "camera_pose_source": "world camera_optical_frame composed from sampled rig, dynamic camera link, and static optical transform",
                "camera_head_transforms": camera_head_artifact,
                "sensor_publishers_started": False,
                "pacing": {
                    "enabled": args.realtime,
                    "requested_realtime_factor": args.realtime_factor if args.realtime else None,
                    "wall_elapsed_s": simulation_wall_elapsed_s,
                    "measured_realtime_factor": (frames * simulation_dt_s) / simulation_wall_elapsed_s,
                },
                "ros_environment": ros_environment,
            }
            Path(args.status_path).parent.mkdir(parents=True, exist_ok=True)
            Path(args.status_path).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            print("[grocery-runtime] capture-only simulation completed", flush=True)
            return result
        print("[grocery-runtime] creating camera ROS graph", flush=True)
        camera_cadence = _create_camera_graph(camera_path, scenario.camera.width_px, scenario.camera.height_px, scenario.camera.fps)

        # Sensor API objects must be created after the bridge extension is
        # loaded and before the simulation starts.
        print("[grocery-runtime] creating RTX LiDAR ROS writer", flush=True)
        lidar_topic = TOPICS["lidar_ideal_points"] if noise_config.enabled else TOPICS["lidar_points"]
        lidar_spec = lidar_runtime_spec(scenario.lidar)
        _, _lidar_sensor = _create_lidar(lidar_path, scenario.lidar, lidar_topic)
        print("[grocery-runtime] creating clock and TF graph", flush=True)
        _create_clock_graph()
        print("[grocery-runtime] initializing simulation", flush=True)
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        simulation_app.update()

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage
        from rclpy.executors import MultiThreadedExecutor
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        rclpy.init(args=None)
        node = rclpy.create_node("grocery_sim_ground_truth")
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        gt_pub = node.create_publisher(PoseStamped, TOPICS["ground_truth_pose"], 10)
        tf_pub = node.create_publisher(TFMessage, "/tf", 10)
        static_tf = StaticTransformBroadcaster(node)
        _publish_static_tf(static_tf, scenario)
        lidar_relay = None
        if noise_config.enabled:
            from simulator.ros.lidar_noise_relay import LidarNoiseRelay

            lidar_relay = LidarNoiseRelay(node, noise_config, scenario.seed)
        rgb_stamps: list[float] = []
        clock_stamps: list[float] = []
        ground_truth_stamps: list[float] = []
        lidar_stamps: list[float] = []
        lidar_point_counts: list[int] = []

        def _on_rgb(message) -> None:
            rgb_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)

        def _on_clock(message) -> None:
            clock_stamps.append(float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0)

        def _on_ground_truth(message) -> None:
            ground_truth_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)

        def _on_lidar(message) -> None:
            lidar_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)
            lidar_point_counts.append(int(getattr(message, "width", 0)) * int(getattr(message, "height", 0)))

        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, PointCloud2

        node.create_subscription(Image, TOPICS["rgb_image"], _on_rgb, 128)
        node.create_subscription(Clock, TOPICS["clock"], _on_clock, 10)
        node.create_subscription(PoseStamped, TOPICS["ground_truth_pose"], _on_ground_truth, 10)
        node.create_subscription(PointCloud2, TOPICS["lidar_points"], _on_lidar, 10)
        executor_thread = threading.Thread(target=executor.spin, name="grocery_sim_ros_executor", daemon=True)
        executor_thread.start()
        app_utils.play()
        print("[grocery-runtime] simulation running", flush=True)

        from pxr import Gf, UsdGeom

        rig_prim = stage.GetPrimAtPath("/World/SensorRig")
        rig_api = UsdGeom.XformCommonAPI(rig_prim)
        camera_link_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/SensorRig/camera_link"))
        from omni.timeline import get_timeline_interface

        timeline = get_timeline_interface()
        last_timestamp_s: float | None = None
        render_phase_offsets_s: list[float] = []
        simulation_wall_started = time.perf_counter()
        for frame in range(frames):
            wall_start = time.perf_counter()
            timeline_before_s = float(timeline.get_current_time())
            # Sensor rendering and the simulation clock advance on update().
            # Command the rig for the measured next simulation tick, then
            # publish truth using the post-update timeline stamp.  This keeps
            # truth, TF, and the rendered sensor pose on one measured phase.
            commanded_timestamp_s = timeline_before_s + simulation_dt_s
            commanded_sample = trajectory.sample(commanded_timestamp_s)
            rig_api.SetTranslate(Gf.Vec3d(*commanded_sample.position_m))
            # Apply the complete sampled walking/trajectory orientation to the
            # actual USD rig.  The TF and ground-truth messages use this same
            # quaternion, so USD, ROS TF, and the pose topic remain aligned.
            from simulator.sensors.transforms import rpy_deg_from_quaternion

            sampled_rpy_deg = rpy_deg_from_quaternion(commanded_sample.orientation_xyzw)
            rig_api.SetRotate(Gf.Vec3f(*sampled_rpy_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_link_api.SetRotate(
                Gf.Vec3f(*rpy_deg_from_quaternion(camera_mount_orientation(commanded_sample, scenario.camera))),
                UsdGeom.XformCommonAPI.RotationOrderXYZ,
            )
            simulation_app.update()
            timestamp_s = float(timeline.get_current_time())
            if last_timestamp_s is not None and timestamp_s <= last_timestamp_s:
                raise RuntimeError("Isaac timeline did not advance after rendering; refusing synthetic timestamps")
            render_phase_offsets_s.append(timestamp_s - commanded_timestamp_s)
            sample = trajectory.sample(timestamp_s)
            _publish_ground_truth(gt_pub, sample, timestamp_s)
            _publish_tf(tf_pub, sample, timestamp_s, scenario)
            if representative_capture is not None and frame in selected_capture_frames:
                representative_capture.capture(
                    frame,
                    timestamp_s,
                    camera_optical_world_pose(sample, scenario.camera),
                    timeline,
                )
            last_timestamp_s = timestamp_s
            if paced_wall_period_s is not None:
                time.sleep(max(0.0, paced_wall_period_s - (time.perf_counter() - wall_start)))

        simulation_wall_elapsed_s = time.perf_counter() - simulation_wall_started
        head_stamp_alignment = camera_head_stamp_alignment(
            rgb_stamps,
            [item.timestamp_s for item in trajectory.sample_many()],
        )
        if not head_stamp_alignment["all_rgb_stamps_matched"]:
            raise RuntimeError(
                "observed RGB timestamps do not match the exported camera head transform samples"
            )

        runtime_ok = True
        print("[grocery-runtime] simulation completed", flush=True)
        capture_result = None
        if representative_capture is not None:
            capture_result = representative_capture.finalize()
            representative_capture = None
        result = {
            "runtime": "isaac_sim",
            "isaac_version": "6.1.0",
            "scenario": scenario.name,
            "frames_simulated": frames,
            "primitive_count": primitive_count,
            "product_count": len(layout.products),
            "topics": {
                "clock": TOPICS["clock"],
                "rgb": TOPICS["rgb_image"],
                "camera_info": TOPICS["rgb_camera_info"],
                "lidar_ideal": TOPICS["lidar_ideal_points"],
                "lidar": TOPICS["lidar_points"],
                "ground_truth_pose": TOPICS["ground_truth_pose"],
            },
            "frames": list(FRAMES.values()),
            "camera_cadence": camera_cadence,
            "observed_rgb_frames": len(rgb_stamps),
            "observed_rgb_stamp_sha256": stamp_sequence_sha256(rgb_stamps),
            "camera_head_stamp_alignment": head_stamp_alignment,
            "observed_rgb_hz": (len(rgb_stamps) - 1) / (rgb_stamps[-1] - rgb_stamps[0]) if len(rgb_stamps) > 1 and rgb_stamps[-1] > rgb_stamps[0] else None,
            "observed_clock_samples": len(clock_stamps),
            "observed_lidar_clouds": len(lidar_stamps),
            "clock_source": "Isaac timeline current_time",
            "ros_callback_service": "MultiThreadedExecutor background thread",
            "pacing": {
                "enabled": args.realtime,
                "requested_realtime_factor": args.realtime_factor if args.realtime else None,
                "wall_elapsed_s": simulation_wall_elapsed_s,
                "measured_realtime_factor": (frames * simulation_dt_s) / simulation_wall_elapsed_s,
            },
            "timestamp_phase": {
                "clock_reference": TOPICS["clock"],
                "truth_and_rig_source": "Isaac timeline current_time",
                "policy": "command_rig_before_update_publish_truth_after_update",
                "evaluation_policy": "interpolate_ground_truth_at_estimator_timestamps",
                "synthetic_frame_time_fallback": False,
                "render_phase_offset_mean_s": None if not render_phase_offsets_s else sum(render_phase_offsets_s) / len(render_phase_offsets_s),
                "render_phase_offset_max_abs_s": None if not render_phase_offsets_s else max(abs(value) for value in render_phase_offsets_s),
            },
            "static_tf_topic": "/tf_static",
            "dynamic_camera_tf_topic": "/tf",
            "dynamic_camera_tf": {
                "parent": FRAMES["sensor_rig"],
                "child": FRAMES["camera_link"],
                "timestamp_source": "Isaac timeline current_time",
                "rotation_source": "configured camera pose_in_rig composed with trajectory sample camera_link_orientation_xyzw",
            },
            "truth_tf_child_frame": FRAMES["truth_sensor_rig"],
            "lidar_config": lidar_spec,
            "timestamp_alignment": {
                "reference": TOPICS["clock"],
                "ground_truth": _timestamp_summary(clock_stamps, ground_truth_stamps),
                "rgb": _timestamp_summary(clock_stamps, rgb_stamps),
                "lidar": _timestamp_summary(clock_stamps, lidar_stamps),
            },
            "lidar_cloud_points": _point_count_summary(lidar_point_counts),
            "lidar_noise": {
                "enabled": noise_config.enabled,
                "relay": "simulator.ros.lidar_noise_relay" if lidar_relay is not None else None,
                "received_clouds": None if lidar_relay is None else lidar_relay.received_clouds,
                "published_clouds": None if lidar_relay is None else lidar_relay.published_clouds,
                "last_point_count": None if lidar_relay is None else lidar_relay.last_point_count,
            },
            "rig_orientation_source": "trajectory.sample.orientation_xyzw",
            "last_rig_orientation_xyzw": list(sample.orientation_xyzw),
            "last_camera_mount_orientation_xyzw": list(camera_mount_orientation(sample, scenario.camera)),
            "camera_pose_source": "world camera_optical_frame composed from sampled rig, dynamic camera link, and static optical transform",
            "camera_head_transforms": camera_head_artifact,
            "ground_truth_odometry_leakage": False,
            "representative_capture": capture_result,
            "ros_environment": ros_environment,
        }
        # Persist before SimulationApp teardown.  Isaac's shutdown sequence
        # can terminate the embedded Python process before the caller regains
        # control, so a successful run must not depend on ``main()`` reaching
        # its final write.
        Path(args.status_path).parent.mkdir(parents=True, exist_ok=True)
        Path(args.status_path).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    except BaseException as exc:
        error = {
            "runtime": "isaac_sim",
            "status": "error",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
            "ros_environment": ros_environment,
        }
        print(json.dumps(error, indent=2), flush=True)
        try:
            Path(args.status_path).parent.mkdir(parents=True, exist_ok=True)
            Path(args.status_path).write_text(json.dumps(error, indent=2) + "\n", encoding="utf-8")
        except Exception:
            pass
        raise
    finally:
        if representative_capture is not None:
            try:
                representative_capture.close()
            except Exception:
                pass
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if executor_thread is not None and executor_thread.is_alive():
            executor_thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            import isaacsim.core.experimental.utils.app as app_utils

            app_utils.stop()
        except Exception:
            pass
        simulation_app.close()


def main() -> None:
    args = _args()
    status_path = Path(args.status_path)
    status_path.parent.mkdir(parents=True, exist_ok=True)
    result = run(args)
    status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
