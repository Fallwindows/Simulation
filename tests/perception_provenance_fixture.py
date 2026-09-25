"""Small canonical capture/SLAM/perception fixture for provenance tests."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess

from simulator.capture.manifest import REQUIRED_CAPTURE_TOPICS, capture_hash, sha256_file, write_json
from simulator.perception.provenance import (
    REQUIRED_CAPTURE_TOPIC_TYPES,
    build_perception_input_bindings,
    validate_perception_frame_coverage,
)


TOPICS = list(REQUIRED_CAPTURE_TOPICS)


def _jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")


def create_perception_run(
    parent: Path,
    frame_count: int = 613,
    capture_id: str = "perception-fixture",
    *,
    decodable_video: bool = False,
) -> dict[str, Path | dict]:
    repo = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    run = parent / capture_id
    capture = run / "capture"
    slam = run / "slam"
    perception = run / "perception"
    for directory in (capture, slam, perception):
        directory.mkdir(parents=True)

    width, height = ((96, 72) if decodable_video else (3840, 2160))
    frames = [
        {"frame_index": index, "stamp_s": index / 30.0, "width": width, "height": height}
        for index in range(frame_count)
    ]
    _jsonl(capture / "rgb_frames.jsonl", frames)
    if decodable_video:
        import cv2
        import numpy as np

        writer = cv2.VideoWriter(
            str(capture / "rgb_camera.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError("test video writer did not open")
        for _ in range(frame_count):
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            frame[28:45, 40:57] = (0, 0, 255)
            writer.write(frame)
        writer.release()
    else:
        (capture / "rgb_camera.mp4").write_bytes(b"native-rgb-fixture")
    write_json(capture / "rgb_video.json", {
        "status": "complete", "frame_count": frame_count, "nominal_fps": 30.0,
        "camera_info_count": frame_count, "width": width, "height": height, "first_image_stamp_s": 0.0,
        "last_image_stamp_s": (frame_count - 1) / 30.0,
    })
    write_json(capture / "camera_info.json", {
        "topic": "/sim/camera/rgb/camera_info", "stamp_s": 0.0, "frame_id": "camera_optical_frame",
        "width": width, "height": height, "distortion_model": "plumb_bob", "d": [],
        "k": [48.0, 0.0, 48.0, 0.0, 48.0, 36.0, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [48.0, 0.0, 48.0, 0.0, 0.0, 48.0, 36.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    })
    write_json(capture / "sensor_transforms.json", {
        "intrinsics": {"fx_px": 48.0, "fy_px": 48.0, "cx_px": 48.0, "cy_px": 36.0},
        "transforms": [
            {"child": name, "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]}
            for name in ("lidar_link", "camera_link", "camera_optical_frame")
        ],
    })
    (capture / "inventory_ground_truth.csv").write_text("semantic_id\n1\n", encoding="utf-8")
    write_json(capture / "inventory_ground_truth.json", {"evaluation_only": True, "items": [1]})
    write_json(capture / "effective_config.json", {"lidar": {"hz": 10.0}})
    write_json(capture / "bag_metadata.json", {
        "status": "complete", "topics": TOPICS, "counts": {topic: frame_count for topic in TOPICS},
        "first_stamp_s": {"/sim/lidar/points": 0.0},
        "last_stamp_s": {"/sim/lidar/points": (frame_count - 1) / 30.0},
    })
    bag = capture / "sensors_bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "storage_identifier: sqlite3\nrelative_file_paths:\n  - capture_0.db3\n", encoding="utf-8"
    )
    (bag / "capture_0.db3").write_bytes(b"sqlite-fixture")

    files = [
        {"path": path.relative_to(capture).as_posix(), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
        for path in sorted(item for item in capture.rglob("*") if item.is_file())
    ]
    capture_manifest = {
        "manifest_version": 1, "status": "complete", "capture_id": capture_id,
        "scenario": "generated-perception-test", "git_sha": revision,
        "rmw_implementation": "rmw_zenoh_cpp", "ros_domain_id": 42, "duration_s": frame_count / 30.0,
        "bag": {
            "uri": "sensors_bag", "storage_id": "sqlite3", "topics": TOPICS,
            "counts": {topic: frame_count for topic in TOPICS},
            "first_clock_s": 0.0, "last_clock_s": (frame_count - 1) / 30.0,
        },
        "rgb": {
            "video": "rgb_camera.mp4", "timestamp_index": "rgb_frames.jsonl",
            "camera_info": "camera_info.json",
            "metadata": "rgb_video.json", "frame_count": frame_count,
            "width_px": width, "height_px": height, "fps": 30.0,
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

    trajectory = slam / "slam_map_poses.csv"
    trajectory.write_text(
        "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw\n0.0,0,0,0,0,0,0,1\n20.4,2,0,0,0,0,0,1\n",
        encoding="utf-8",
    )
    (slam / "slam_map.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n",
        encoding="utf-8",
    )
    map_version = "a" * 64
    dense_pose_version = "b" * 64
    trajectory_binding = {
        "path": trajectory.name, "role": "dense_corrected_trajectory", "frame_id": "map",
        "optimized": True, "map_version": map_version, "dense_pose_version": dense_pose_version,
        "schema_version": 1, "correction_policy": "generated repaired-v1 fixture",
        "sha256": sha256_file(trajectory), "size_bytes": trajectory.stat().st_size,
    }
    observer = {
        "status": "complete", "files": [trajectory_binding], "map_version": map_version,
        "dense_pose_version": dense_pose_version, "graph_pose_version": "c" * 64,
        "pre_publish_graph_version": "c" * 64, "map_graph_matches_final_cloud": True,
        "optimized_pose_graph_complete": True, "map_pose_frame_id": "map", "map_pose_sample_count": 2,
        "odom_sample_count": 2,
    }
    write_json(slam / "slam_observer.json", observer)
    observer_bytes = (slam / "slam_observer.json").read_bytes()
    slam_manifest = {
        "status": "complete", "capture_id": capture_id,
        "capture_sha256": capture_manifest["capture_sha256"], "git_sha": revision,
        "bag_replayed": str(bag.resolve()), "ground_truth_subscribed": False,
        "map_version": map_version, "dense_pose_version": dense_pose_version,
        "map_frame_id": "map", "optimized": True, "artifacts": [trajectory_binding],
        "observer": observer,
        "observer_artifact": {
            "path": "slam_observer.json", "sha256": sha256_file(slam / "slam_observer.json"),
            "size_bytes": len(observer_bytes),
        },
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
        "inputs": inputs, "slam_trajectory": "../slam/slam_map_poses.csv",
        "slam_artifact": "../slam/slam_map.pcd",
        "slam_pose_provenance": {"slam_manifest_sha256": inputs["slam"]["manifest"]["sha256"]},
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
