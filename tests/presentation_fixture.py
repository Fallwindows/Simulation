"""Generated, explicitly test-only complete presentation fixture and preview."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from simulator.capture.manifest import capture_hash, write_json
from simulator.presentation.complete_bundle import emit_complete_bundle
from simulator.presentation.provenance import (
    GENERATED_TEST_FIXTURE_CLASSIFICATION,
    GENERATED_TEST_FIXTURE_MARKER_SHA256,
    PRESENTATION_TRANSFORM_HFLIP,
)
from simulator.presentation.rgb_bundle import emit_rgb_bundle


TOPICS = ["/clock", "/sim/camera/rgb/image_raw", "/sim/camera/rgb/camera_info", "/sim/lidar/points", "/tf", "/tf_static"]
TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
    "/sim/camera/rgb/camera_info": "sensor_msgs/msg/CameraInfo",
    "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
VIEWS = (
    ("sensor_activation", "lidar", 120, "0x12314a"),
    ("lidar_environment", "lidar", 90, "0x17415c"),
    ("persistent_map", "map", 90, "0x1d526b"),
    ("object_association", "reconstruction", 120, "0x245f70"),
    ("object_detail", "reconstruction", 120, "0x2d6b73"),
    ("observed_aisle_overview", "reconstruction", 120, "0x34766f"),
    ("final_technical_view", "reconstruction", 150, "0x3d8067"),
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _run(command: list[str]) -> None:
    subprocess.run(command, check=True, capture_output=True, text=True)


def _video(ffmpeg: str, output: Path, frames: int, color: str, *, mirrored_rgb_fixture: bool = False) -> None:
    filters = []
    if mirrored_rgb_fixture:
        filters = ["-vf", "drawbox=x=0:y=0:w=iw/2:h=ih:color=red:t=fill"]
    _run(
        [
            ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
            f"color=c={color}:s=1920x1080:r=30", *filters, "-frames:v", str(frames),
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "28", "-pix_fmt", "yuv420p", "-y", str(output),
        ]
    )


def build_complete_fixture(root: Path, repo_root: Path, ffmpeg: str, ffprobe: str) -> dict[str, Path]:
    """Build generated media and reviewed test catalogs; none are production evidence."""

    root.mkdir(parents=True, exist_ok=True)
    capture = root / "capture"
    capture.mkdir(exist_ok=True)
    capture_id = "generated-complete-presentation-fixture"
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo_root, text=True).strip()
    video = capture / "rgb_camera.mp4"
    _video(ffmpeg, video, 615, "blue", mirrored_rgb_fixture=True)
    frames = capture / "rgb_frames.jsonl"
    with frames.open("w", encoding="utf-8", newline="\n") as handle:
        for index in range(615):
            handle.write(json.dumps({
                "frame_index": index, "stamp_s": index / 30.0, "frame_id": "camera_optical_frame",
                "width": 1920, "height": 1080, "encoding": "rgb8",
            }, sort_keys=True, separators=(",", ":")) + "\n")
    metadata = capture / "rgb_video.json"
    write_json(metadata, {
        "status": "complete", "frame_count": 615, "first_image_stamp_s": 0.0,
        "last_image_stamp_s": 614 / 30.0, "nominal_fps": 30.0, "width": 1920, "height": 1080,
    })
    transforms = capture / "sensor_transforms.json"
    write_json(transforms, {
        "units": "m", "rotation_order": "xyzw_ros", "transforms": [{"parent": "sensor_rig", "child": "camera_optical_frame"}],
        "topics": {"rgb_camera_info": "/sim/camera/rgb/camera_info", "lidar_points": "/sim/lidar/points"},
        "frames": {"camera_optical": "camera_optical_frame"},
        "intrinsics": {"width_px": 1920, "height_px": 1080, "fx_px": 960.0, "fy_px": 960.0, "cx_px": 959.5, "cy_px": 539.5},
    })
    camera_info = capture / "camera_info.json"
    write_json(camera_info, {
        "topic": "/sim/camera/rgb/camera_info", "stamp_s": 0.0, "frame_id": "camera_optical_frame",
        "width": 1920, "height": 1080, "distortion_model": "plumb_bob", "d": [],
        "k": [960.0, 0.0, 959.5, 0.0, 960.0, 539.5, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [960.0, 0.0, 959.5, 0.0, 0.0, 960.0, 539.5, 0.0, 0.0, 0.0, 1.0, 0.0],
    })
    (capture / "inventory_ground_truth.csv").write_text("semantic_id\n1\n", encoding="utf-8")
    write_json(capture / "inventory_ground_truth.json", {"evaluation_only": True, "items": [1]})
    write_json(capture / "effective_config.json", {"lidar": {"hz": 10.0}})
    write_json(capture / "bag_metadata.json", {
        "status": "complete", "topics": TOPICS, "counts": {topic: 1 for topic in TOPICS},
        "first_stamp_s": {"/sim/lidar/points": 0.2}, "last_stamp_s": {"/sim/lidar/points": 20.4},
    })
    bag = capture / "sensors_bag"
    bag.mkdir(exist_ok=True)
    (bag / "metadata.yaml").write_text(
        "storage_identifier: sqlite3\nrelative_file_paths:\n  - capture_0.db3\n", encoding="utf-8"
    )
    (bag / "capture_0.db3").write_bytes(b"generated test-only sqlite fixture")
    files = [
        {"path": path.relative_to(capture).as_posix(), "sha256": sha256(path), "size_bytes": path.stat().st_size}
        for path in sorted(item for item in capture.rglob("*") if item.is_file())
    ]
    capture_manifest = capture / "capture_manifest.json"
    capture_value = {
        "manifest_version": 1, "status": "complete", "capture_id": capture_id,
        "scenario": "generated-test-only", "git_sha": revision, "rmw_implementation": "rmw_zenoh_cpp",
        "ros_domain_id": 42, "duration_s": 20.5,
        "bag": {
            "uri": "sensors_bag", "storage_id": "sqlite3", "topics": TOPICS,
            "counts": {topic: 1 for topic in TOPICS}, "first_clock_s": 0.0, "last_clock_s": 20.5,
        },
        "rgb": {
            "video": video.name, "timestamp_index": frames.name, "camera_info": camera_info.name,
            "metadata": metadata.name,
            "frame_count": 615, "first_stamp_s": 0.0, "last_stamp_s": 614 / 30.0,
        },
        "ground_truth": {
            "inventory_csv": "inventory_ground_truth.csv", "inventory_json": "inventory_ground_truth.json",
            "pose_topic": "/sim/ground_truth/pose", "evaluation_only": True,
        },
        "hashes": {
            "geometry_sha256": "1" * 64, "inventory_sha256": "2" * 64, "trajectory_sha256": "3" * 64,
            "sensor_sha256": "4" * 64, "appearance_sha256": "5" * 64, "inputs": {}, "git_sha": revision,
        },
        "software_versions": "generated-test-only", "files": files,
    }
    capture_value["capture_sha256"] = capture_hash(capture_value)
    write_json(capture_manifest, capture_value)
    producer_blob = subprocess.check_output(
        ["git", "rev-parse", f"{revision}:scripts/capture_simulation.ps1"], cwd=repo_root, text=True
    ).strip()
    rgb_catalog = root / "rgb_catalog.json"
    required_hashes = {
        "view_video": sha256(video), "frame_index": sha256(frames), "camera_info": sha256(camera_info),
        "rgb_metadata": sha256(metadata), "capture_manifest": sha256(capture_manifest), "sensor_transforms": sha256(transforms),
    }
    write_json(rgb_catalog, {
        "schema_version": 1, "mechanism": "reviewed_exact_capture_allowlist", "captures": [{
            "capture_id": capture_id, "capture_sha256": capture_value["capture_sha256"], "git_sha": revision,
            "producer_source_path": "scripts/capture_simulation.ps1", "producer_blob_sha1": producer_blob,
            "required_artifact_sha256": required_hashes,
            "review": {
                "status": "accepted", "reviewer": "generated-test-fixture", "reviewed_utc": "2026-09-21T00:00:00Z",
                "verdict_path": "review/evidence/generated-presentation-fixture-verdict.txt",
                "verdict_sha256": GENERATED_TEST_FIXTURE_MARKER_SHA256,
            },
            "presentation_classification": GENERATED_TEST_FIXTURE_CLASSIFICATION,
            "presentation_transform": PRESENTATION_TRANSFORM_HFLIP,
        }],
    })
    rgb_bundle = root / "rgb_bundle.json"
    emit_rgb_bundle(capture_manifest, rgb_bundle, repo_root=repo_root, ffprobe=ffprobe, acceptance_catalog_path=rgb_catalog)

    technical_dir = root / "technical"
    technical_dir.mkdir(exist_ok=True)
    digest = lambda text: hashlib.sha256(text.encode("utf-8")).hexdigest()
    technical_catalog = root / "technical_catalog.json"
    technical_catalog_value = {
        "schema_version": 1, "status": "reviewed_source_catalog", "sources": [{
            "source_id": "generated-complete-presentation-fixture-v1", "capture_id": capture_id,
            "presentation_classification": GENERATED_TEST_FIXTURE_CLASSIFICATION,
            "run_directory_name": capture_id, "capture_sha256": capture_value["capture_sha256"],
            "producer_revisions": {"capture": revision, "slam": revision, "perception": revision},
            "producer_manifests": {
                "capture": {"path": "capture/capture_manifest.json", "sha256": sha256(capture_manifest)},
                "slam": {"path": "slam/slam_manifest.json", "sha256": digest("slam manifest")},
                "perception": {"path": "perception/perception_manifest.json", "sha256": digest("perception manifest")},
            },
            "artifacts": {
                "map": {"path": "slam/slam_map.ply", "sha256": digest("map"), "version": "fixture-map-v1"},
                "trajectory": {"path": "slam/slam_map_poses.csv", "sha256": digest("trajectory"), "version": "fixture-trajectory-v1"},
                "inventory": {"path": "perception/estimated_inventory.csv", "sha256": digest("inventory"), "version": "fixture-object-v1"},
            },
            "simulation_time": {"source": "slam/slam_map_poses.csv:timestamp_s", "start_s": 0.2, "end_s": 20.4},
            "perception_contract": {
                "allowed_depth_sources": ["lidar_projected_with_slam_pose"], "estimated_inventory": "estimated_inventory.csv",
                "slam_artifact": "slam/slam_map.pcd", "legacy_capture_id_omitted": True, "ground_truth_consumed": False,
            },
        }],
    }
    write_json(technical_catalog, technical_catalog_value)
    # Exercise the Windows checkout contract: the producer reads CRLF repository
    # text and catalog bytes while the presentation validator may read LF bytes.
    technical_catalog.write_bytes(technical_catalog.read_bytes().replace(b"\n", b"\r\n"))
    from simulator.technical_views import (
        SourceBundle,
        _source_receipt as renderer_source_receipt,
        _write_view_receipt,
        canonical_text_sha256,
        load_plan as load_technical_plan,
    )
    crlf_text = root / "crlf_repository_text"
    crlf_text.mkdir(exist_ok=True)
    crlf_renderer = crlf_text / "technical_views.py"
    crlf_plan = crlf_text / "technical_views.json"
    for source_path, mirror_path in (
        (repo_root / "simulator" / "technical_views.py", crlf_renderer),
        (repo_root / "config" / "technical_views.json", crlf_plan),
    ):
        normalized = source_path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
        mirror_path.write_bytes(normalized.replace("\n", "\r\n").encode("utf-8"))
    renderer_hash = canonical_text_sha256(crlf_renderer)
    plan_hash = canonical_text_sha256(crlf_plan)
    source_entry = technical_catalog_value["sources"][0]
    source_artifacts = source_entry["artifacts"]
    source_manifests = source_entry["producer_manifests"]
    renderer_bundle = SourceBundle(
        run_root=root,
        capture_id=capture_id,
        source_id=source_entry["source_id"],
        presentation_classification=source_entry["presentation_classification"],
        capture_git_sha=revision,
        slam_git_sha=revision,
        perception_git_sha=revision,
        simulation_time_start_s=0.2,
        simulation_time_end_s=20.4,
        map_version=source_artifacts["map"]["version"],
        trajectory_version=source_artifacts["trajectory"]["version"],
        object_state_version=source_artifacts["inventory"]["version"],
        trajectory_time_basis="slam/slam_map_poses.csv:timestamp_s",
        depth_sources=("lidar_projected_with_slam_pose",),
        map_path=root / source_artifacts["map"]["path"],
        trajectory_path=root / source_artifacts["trajectory"]["path"],
        inventory_path=root / source_artifacts["inventory"]["path"],
        capture_manifest_path=capture_manifest,
        slam_manifest_path=root / source_manifests["slam"]["path"],
        perception_manifest_path=root / source_manifests["perception"]["path"],
        source_catalog_path=technical_catalog,
        hashes={
            "map": source_artifacts["map"]["sha256"],
            "trajectory": source_artifacts["trajectory"]["sha256"],
            "inventory": source_artifacts["inventory"]["sha256"],
            "capture_manifest": source_manifests["capture"]["sha256"],
            "slam_manifest": source_manifests["slam"]["sha256"],
            "perception_manifest": source_manifests["perception"]["sha256"],
            "source_catalog": canonical_text_sha256(technical_catalog),
        },
    )
    source = renderer_source_receipt(renderer_bundle)
    _, technical_specs = load_technical_plan(repo_root / "config" / "technical_views.json")
    specs_by_id = {spec.id: spec for spec in technical_specs}
    outputs = []
    for view_id, role, frame_count, color in VIEWS:
        view_video = technical_dir / f"{view_id}_1080p.mp4"
        _video(ffmpeg, view_video, frame_count, color)
        video_hash = sha256(view_video)
        probe = {"width": 1920, "height": 1080, "fps": 30.0, "frame_count": frame_count}
        receipt_path = _write_view_receipt(
            technical_dir, view_video, video_hash, probe, specs_by_id[view_id],
            renderer_bundle, renderer_hash, plan_hash,
        )
        outputs.append({
            "path": view_video.name, "sha256": video_hash, "frames": frame_count, "view_ids": [view_id],
            "presentation_role": role, "probe": probe,
            "receipt": {"path": receipt_path.name, "sha256": sha256(receipt_path)},
        })
    technical_manifest = technical_dir / "technical_views_delivery_manifest.json"
    write_json(technical_manifest, {
        "schema_version": 1, "status": "complete", "producer": "grocery_sim.technical_views.cpu.v1",
        "profile": "delivery", "width": 1920, "height": 1080, "fps": 30,
        "ordered_views": [item[0] for item in VIEWS], "view_frame_counts": {item[0]: item[2] for item in VIEWS},
        "total_frames": sum(item[2] for item in VIEWS), "capture_id": capture_id,
        "capture_git_sha": revision, "slam_git_sha": revision, "perception_git_sha": revision,
        "simulation_time": {"start_s": 0.2, "end_s": 20.4}, "map_version": "fixture-map-v1",
        "trajectory_version": "fixture-trajectory-v1", "object_state_version": "fixture-object-v1", "source": source,
        "renderer": {"path": "simulator/technical_views.py", "sha256": renderer_hash},
        "plan": {"path": "config/technical_views.json", "sha256": plan_hash},
        "storyboard_content_used": False, "storyboard_exclusion_basis": "generated test fixture uses only synthetic colors",
        "selective_current_scan_goal": {
            "status": "unfinished", "current_implementation": "legacy finalized-map projection",
            "required_replacement": "feature-specific point selection from spatially aligned current scans",
        },
        "outputs": outputs,
    })
    complete = root / "complete_inputs.json"
    emit_complete_bundle(
        rgb_bundle, technical_manifest, complete, repo_root, ffprobe,
        rgb_capture_catalog=rgb_catalog, technical_source_catalog=technical_catalog,
    )
    return {
        "capture_manifest": capture_manifest, "rgb_catalog": rgb_catalog, "rgb_bundle": rgb_bundle,
        "technical_catalog": technical_catalog, "technical_manifest": technical_manifest, "complete_inputs": complete,
        "crlf_renderer": crlf_renderer, "crlf_plan": crlf_plan,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--ffprobe", required=True)
    args = parser.parse_args()
    result = build_complete_fixture(Path(args.output), Path(args.repo_root), args.ffmpeg, args.ffprobe)
    print(json.dumps({name: str(path) for name, path in result.items()}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
