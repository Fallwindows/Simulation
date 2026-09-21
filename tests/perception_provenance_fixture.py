"""Small canonical capture/SLAM/perception fixture for provenance tests."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

from simulator.capture.manifest import REQUIRED_CAPTURE_TOPIC_TYPES, capture_hash, sha256_file, write_json
from simulator.perception.provenance import build_perception_input_bindings, validate_perception_frame_coverage


TOPICS = list(REQUIRED_CAPTURE_TOPIC_TYPES)


def _jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")


def create_perception_run(parent: Path, frame_count: int = 613, capture_id: str = "perception-fixture") -> dict[str, Path | dict]:
    repo = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    run = parent / capture_id
    capture = run / "capture"
    slam = run / "slam"
    perception = run / "perception"
    for directory in (capture, slam, perception):
        directory.mkdir(parents=True)

    frames = [
        {"frame_index": index, "stamp_s": index / 30.0, "width": 3840, "height": 2160}
        for index in range(frame_count)
    ]
    _jsonl(capture / "rgb_frames.jsonl", frames)
    (capture / "rgb_camera.mp4").write_bytes(b"native-rgb-fixture")
    write_json(capture / "rgb_video.json", {
        "status": "complete", "frame_count": frame_count, "nominal_fps": 30.0,
        "width": 3840, "height": 2160, "first_image_stamp_s": 0.0,
        "last_image_stamp_s": (frame_count - 1) / 30.0,
    })
    write_json(capture / "camera_info.json", {
        "provenance": "configured_intrinsics", "observed_ros_message": False,
    })
    write_json(capture / "sensor_transforms.json", {"frames": {}, "transforms": []})
    (capture / "inventory_ground_truth.csv").write_text("semantic_id\n1\n", encoding="utf-8")
    write_json(capture / "inventory_ground_truth.json", {"evaluation_only": True, "items": [1]})
    bag = capture / "sensors_bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("storage_identifier: sqlite3\n", encoding="utf-8")
    (bag / "capture_0.db3").write_bytes(b"sqlite-fixture")

    files = [
        {"path": path.relative_to(capture).as_posix(), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
        for path in sorted(item for item in capture.rglob("*") if item.is_file())
    ]
    capture_manifest = {
        "manifest_version": 2, "status": "complete", "capture_id": capture_id,
        "scenario": "generated-perception-test", "git_sha": revision,
        "rmw_implementation": "rmw_zenoh_cpp", "ros_domain_id": 42, "duration_s": frame_count / 30.0,
        "bag": {
            "uri": "sensors_bag", "storage_id": "sqlite3", "topics": TOPICS,
            "topic_types": REQUIRED_CAPTURE_TOPIC_TYPES, "counts": {topic: frame_count for topic in TOPICS},
            "first_clock_s": 0.0, "last_clock_s": (frame_count - 1) / 30.0,
        },
        "rgb": {
            "video": "rgb_camera.mp4", "timestamp_index": "rgb_frames.jsonl",
            "camera_info": "camera_info.json", "camera_info_provenance": "configured_intrinsics",
            "metadata": "rgb_video.json", "frame_count": frame_count,
            "first_stamp_s": 0.0, "last_stamp_s": (frame_count - 1) / 30.0,
        },
        "ground_truth": {
            "inventory_csv": "inventory_ground_truth.csv", "inventory_json": "inventory_ground_truth.json",
            "pose_topic": "/sim/ground_truth/pose", "evaluation_only": True,
        },
        "hashes": {
            "geometry_sha256": "1" * 64, "inventory_sha256": "2" * 64,
            "trajectory_sha256": "3" * 64, "sensor_sha256": "4" * 64,
            "appearance_sha256": "5" * 64, "inputs": {}, "git_sha": revision,
        },
        "software_versions": "generated-test-only", "files": files,
    }
    capture_manifest["capture_sha256"] = capture_hash(capture_manifest)
    write_json(capture / "capture_manifest.json", capture_manifest)

    trajectory = slam / "slam_poses.csv"
    trajectory.write_text(
        "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw\n0.0,0,0,0,0,0,0,1\n20.4,2,0,0,0,0,0,1\n",
        encoding="utf-8",
    )
    (slam / "slam_map.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n",
        encoding="utf-8",
    )
    slam_manifest = {
        "status": "complete", "capture_id": capture_id,
        "capture_sha256": capture_manifest["capture_sha256"], "git_sha": revision,
        "bag_replayed": str(bag.resolve()), "ground_truth_subscribed": False,
        "producer": {"artifacts": {"slam_poses.csv": {"path": "slam_poses.csv", "sha256": sha256_file(trajectory)}}},
    }
    write_json(slam / "slam_manifest.json", slam_manifest)

    annotations = [dict(row, detections=[{"track_id": 1}]) for row in frames]
    _jsonl(perception / "frame_annotations.jsonl", annotations)
    (perception / "estimated_inventory.csv").write_text(
        "track_id,estimated_x_m,estimated_y_m,estimated_z_m,depth_source,3d_observation_count\n"
        "1,1.0,0.5,0.8,lidar_projected_with_slam_pose,5\n",
        encoding="utf-8",
    )
    inputs = build_perception_input_bindings(capture, slam, perception)
    perception_manifest = {
        "status": "complete", "capture_only": True, "ground_truth_consumed": False,
        "ground_truth_required": False, "lidar_consumed_for_estimation": True,
        "slam_consumed_for_estimation": True, "capture_id": capture_id,
        "capture_sha256": inputs["capture"]["capture_sha256"],
        "capture_manifest_sha256": inputs["capture"]["manifest"]["sha256"],
        "slam_manifest_sha256": inputs["slam"]["manifest"]["sha256"],
        "inputs": inputs, "slam_trajectory": "../slam/slam_poses.csv",
        "frame_count": frame_count, "track_count": 1, "video": "../capture/rgb_camera.mp4",
        "frames": "../capture/rgb_frames.jsonl", "annotations": "frame_annotations.jsonl",
        "estimated_inventory": "estimated_inventory.csv", "git_sha": revision,
    }
    if frame_count >= 540:
        perception_manifest["source_frame_contract"] = validate_perception_frame_coverage(
            perception_manifest, capture, perception / "frame_annotations.jsonl"
        )
    write_json(perception / "perception_manifest.json", perception_manifest)
    return {
        "run": run, "capture": capture, "slam": slam, "perception": perception,
        "capture_manifest": capture_manifest, "slam_manifest": slam_manifest,
        "perception_manifest": perception_manifest,
    }


def refresh_perception_manifest(fixture: dict[str, Path | dict], payload: dict) -> None:
    write_json(Path(fixture["perception"]) / "perception_manifest.json", payload)
