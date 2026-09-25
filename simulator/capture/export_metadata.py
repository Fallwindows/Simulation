"""Export immutable capture-side scene, sensor, and effective-config metadata."""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

from simulator.capture.inventory import export_inventory
from simulator.capture.manifest import build_experiment_hashes, write_json
from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory
from simulator.runtime.isaac_sim_runner import write_camera_head_transforms
from simulator.sensors.rig import build_sensor_rig_description


def export_metadata(scenario_path: str | Path, capture_dir: str | Path, repo_root: str | Path) -> dict[str, object]:
    scenario_file = Path(scenario_path).resolve()
    target = Path(capture_dir).resolve()
    root = Path(repo_root).resolve()
    target.mkdir(parents=True, exist_ok=True)
    scenario = load_scenario(scenario_file)
    layout = build_aisle_layout(scenario.environment)
    rig = build_sensor_rig_description(scenario.camera, scenario.lidar)
    scenario_links = json.loads(scenario_file.read_text(encoding="utf-8"))
    trajectory_file = (scenario_file.parent / scenario_links["trajectory"]).resolve()
    trajectory_cls = WalkingTrajectory if scenario.trajectory.name.lower() == "walking" else StraightTrajectory
    head_artifact_path = target / "camera_head_transforms.json"
    head_artifact = write_camera_head_transforms(
        head_artifact_path,
        trajectory_cls(scenario.trajectory),
        scenario,
        trajectory_file,
    )
    hashes = build_experiment_hashes(scenario_file, root)
    export_inventory(scenario_file, target)

    scene = {
        "status": "evaluation_only",
        "scenario": scenario.name,
        "primitive_count": len(layout.primitives),
        "asset_count": len(layout.assets),
        "primitives": [dataclasses.asdict(item) for item in layout.primitives],
        "assets": [dataclasses.asdict(item) for item in layout.assets],
    }
    transforms = {
        "units": "m",
        "rotation_order": "xyzw_ros",
        "mount_semantics": "all sensor mounts are local to sensor_rig; Isaac LiDAR input is converted to wxyz",
        "intrinsics": dataclasses.asdict(rig.camera_intrinsics),
        "transforms": [dataclasses.asdict(item) for item in rig.transforms],
        "topics": rig.topics,
        "frames": rig.frames,
        "dynamic_transform_artifacts": [
            {
                "parent_frame": "sensor_rig",
                "child_frame": "camera_link",
                "path": head_artifact_path.name,
                "sha256": head_artifact["sha256"],
                "size_bytes": head_artifact_path.stat().st_size,
                "schema_version": head_artifact["version"],
            }
        ],
    }
    effective = {
        "scenario": scenario.name,
        "environment": dataclasses.asdict(scenario.environment),
        "camera": dataclasses.asdict(scenario.camera),
        "lidar": dataclasses.asdict(scenario.lidar),
        "trajectory": dataclasses.asdict(scenario.trajectory),
        "mapping": scenario.mapping,
        "sensor_overrides": scenario.sensor_overrides,
    }
    write_json(target / "scene_manifest.json", scene)
    write_json(target / "sensor_transforms.json", transforms)
    write_json(target / "effective_config.json", effective)
    write_json(target / "experiment_hashes.json", hashes)
    return {"status": "complete", "scenario": scenario.name, "asset_count": len(layout.assets), "hashes": hashes}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--repo-root", required=True)
    args = parser.parse_args()
    print(json.dumps(export_metadata(args.scenario, args.capture_dir, args.repo_root), indent=2))


if __name__ == "__main__":
    main()
