"""Emit a validated presentation receipt for one production RGB capture."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from .provenance import (
    CONTENT_VALIDATORS,
    ROLE_SPECS,
    sha256_path,
    source_text_sha256,
    storyboard_hashes,
    validate_rgb_capture_acceptance,
)


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON mapping")
    return value


def _write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.partial")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _relative_descriptor(path: Path, manifest_path: Path) -> dict[str, str]:
    relative = os.path.relpath(path, manifest_path.parent).replace("\\", "/")
    return {"path": relative, "sha256": sha256_path(path)}


def _inventory(capture: dict[str, Any]) -> dict[str, str]:
    return {
        str(item["path"]).replace("\\", "/"): str(item["sha256"]).lower()
        for item in capture.get("files", [])
        if isinstance(item, dict) and "path" in item and "sha256" in item
    }


def _verify_inventory_file(capture_dir: Path, inventory: dict[str, str], path: Path) -> None:
    relative = path.relative_to(capture_dir).as_posix()
    if inventory.get(relative) != sha256_path(path):
        raise ValueError(f"capture manifest does not bind {relative} to its current hash")


def _camera_dimensions(camera_info: dict[str, Any]) -> tuple[int, int]:
    if camera_info.get("provenance") == "configured_intrinsics":
        return int(camera_info["width_px"]), int(camera_info["height_px"])
    return int(camera_info["width"]), int(camera_info["height"])


def _verify_camera_info(camera_info_path: Path, capture_dir: Path, inventory: dict[str, str]) -> None:
    camera_info = _json(camera_info_path)
    if camera_info.get("provenance") != "configured_intrinsics":
        if camera_info.get("topic") != "/sim/camera/rgb/camera_info" or not str(camera_info.get("frame_id", "")):
            raise ValueError("RGB receipt requires an observed camera_info message or configured intrinsics")
        if int(camera_info.get("width", 0)) <= 0 or int(camera_info.get("height", 0)) <= 0:
            raise ValueError("observed camera_info dimensions are invalid")
        for key, length in (("k", 9), ("r", 9), ("p", 12)):
            values = camera_info.get(key)
            if not isinstance(values, list) or len(values) != length:
                raise ValueError(f"observed camera_info {key} is invalid")
        return
    if camera_info.get("observed_ros_message") is not False:
        raise ValueError("configured camera_info provenance flags are invalid")
    transforms_path = (capture_dir / str(camera_info.get("source", ""))).resolve()
    if transforms_path != (capture_dir / "sensor_transforms.json").resolve() or not transforms_path.is_file():
        raise ValueError("configured camera intrinsics must name sensor_transforms.json")
    _verify_inventory_file(capture_dir, inventory, transforms_path)
    transforms = _json(transforms_path)
    configured = transforms.get("intrinsics")
    if not isinstance(configured, dict):
        raise ValueError("sensor_transforms.json has no configured intrinsics")
    for key in ("width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px"):
        if camera_info.get(key) != configured.get(key):
            raise ValueError(f"configured camera intrinsics disagree with sensor_transforms.json: {key}")


def _verify_rgb_timing(
    frame_index_path: Path,
    metadata_path: Path,
    camera_info_path: Path,
    capture: dict[str, Any],
) -> tuple[list[dict[str, Any]], tuple[float, float]]:
    with frame_index_path.open(encoding="utf-8") as handle:
        frames = [json.loads(line) for line in handle if line.strip()]
    metadata = _json(metadata_path)
    camera_info = _json(camera_info_path)
    rgb = capture["rgb"]
    if len(frames) != int(metadata["frame_count"]) or len(frames) != int(rgb["frame_count"]):
        raise ValueError("RGB frame counts disagree across index, metadata, and capture manifest")
    first, last = float(frames[0]["stamp_s"]), float(frames[-1]["stamp_s"])
    expected_dimensions = _camera_dimensions(camera_info)
    if (int(metadata["width"]), int(metadata["height"])) != expected_dimensions:
        raise ValueError("RGB metadata dimensions disagree with configured camera intrinsics")
    if any((int(frame["width"]), int(frame["height"])) != expected_dimensions for frame in frames):
        raise ValueError("RGB frame dimensions disagree with configured camera intrinsics")
    for name, value in (
        ("metadata first stamp", metadata["first_image_stamp_s"]),
        ("capture first stamp", rgb["first_stamp_s"]),
    ):
        if abs(float(value) - first) > 1e-6:
            raise ValueError(f"{name} does not match the RGB frame index")
    for name, value in (
        ("metadata last stamp", metadata["last_image_stamp_s"]),
        ("capture last stamp", rgb["last_stamp_s"]),
    ):
        if abs(float(value) - last) > 1e-6:
            raise ValueError(f"{name} does not match the RGB frame index")
    return frames, (first, last)


def emit_rgb_bundle(
    capture_manifest_path: str | Path,
    output_manifest_path: str | Path,
    view_manifest_path: str | Path | None = None,
    repo_root: str | Path | None = None,
    ffprobe: str = "ffprobe",
    acceptance_catalog_path: str | Path | None = None,
) -> dict[str, Any]:
    """Validate capture outputs, then emit their role bundle and view receipt."""

    root = Path(repo_root).resolve() if repo_root else Path(__file__).resolve().parents[2]
    capture_manifest = Path(capture_manifest_path).resolve()
    output_manifest = Path(output_manifest_path).resolve()
    receipt = (
        Path(view_manifest_path).resolve()
        if view_manifest_path
        else capture_manifest.with_name("rgb_presentation_view_manifest.json")
    )
    capture_dir = capture_manifest.parent
    capture = _json(capture_manifest)
    CONTENT_VALIDATORS["capture_manifest"](capture_manifest)
    rgb = capture["rgb"]
    source_paths = {
        "view_video": (capture_dir / str(rgb["video"])).resolve(),
        "frame_index": (capture_dir / str(rgb["timestamp_index"])).resolve(),
        "camera_info": (capture_dir / str(rgb["camera_info"])).resolve(),
        "rgb_metadata": (capture_dir / str(rgb["metadata"])).resolve(),
        "capture_manifest": capture_manifest,
    }
    source_kinds = {
        "view_video": "video",
        "frame_index": "rgb_frame_index",
        "camera_info": "camera_info",
        "rgb_metadata": "rgb_metadata",
        "capture_manifest": "capture_manifest",
    }
    if output_manifest in source_paths.values() or receipt in source_paths.values() or output_manifest == receipt:
        raise ValueError("RGB receipt outputs must not overwrite capture source artifacts")
    known_storyboard_hashes, _ = storyboard_hashes(root)
    capture_inventory = _inventory(capture)
    for name, path in source_paths.items():
        if not path.is_file():
            raise FileNotFoundError(path)
        digest = sha256_path(path)
        if digest in known_storyboard_hashes:
            raise ValueError(f"known storyboard content is forbidden as an RGB source: {path}")
        CONTENT_VALIDATORS[source_kinds[name]](path)
        if name != "capture_manifest":
            _verify_inventory_file(capture_dir, capture_inventory, path)
    _verify_camera_info(source_paths["camera_info"], capture_dir, capture_inventory)
    frames, source_time_range = _verify_rgb_timing(
        source_paths["frame_index"],
        source_paths["rgb_metadata"],
        source_paths["camera_info"],
        capture,
    )
    from .render_video import probe_video

    video_probe = probe_video(source_paths["view_video"], ffprobe)
    camera_dimensions = _camera_dimensions(_json(source_paths["camera_info"]))
    if (
        video_probe["width"] != camera_dimensions[0]
        or video_probe["height"] != camera_dimensions[1]
        or (video_probe["fps_num"], video_probe["fps_den"]) != (30, 1)
        or (video_probe["real_fps_num"], video_probe["real_fps_den"]) != (30, 1)
        or video_probe["frame_count"] != int(capture["rgb"]["frame_count"])
    ):
        raise ValueError("RGB video stream does not match the capture dimensions, 30 fps, and frame count")

    camera_info = _json(source_paths["camera_info"])
    sensor_transforms = (capture_dir / str(camera_info.get("source", "sensor_transforms.json"))).resolve()
    if sensor_transforms != (capture_dir / "sensor_transforms.json").resolve() or not sensor_transforms.is_file():
        raise ValueError("RGB camera calibration must bind capture/sensor_transforms.json")
    _verify_inventory_file(capture_dir, capture_inventory, sensor_transforms)
    from simulator.technical_lidar import load_camera_head_transform_artifact

    camera_head_trajectory, camera_head_receipt = load_camera_head_transform_artifact(sensor_transforms)
    if camera_head_trajectory is not None:
        camera_head_trajectory.validate_image_timestamps(float(frame["stamp_s"]) for frame in frames)
    acceptance_hashes = {name: sha256_path(path) for name, path in source_paths.items()}
    acceptance_hashes["sensor_transforms"] = sha256_path(sensor_transforms)
    acceptance = validate_rgb_capture_acceptance(
        capture,
        acceptance_hashes,
        root,
        Path(acceptance_catalog_path).resolve() if acceptance_catalog_path else None,
    )

    spec = ROLE_SPECS["rgb"]
    assert spec.view_producer_id is not None and spec.view_producer_source_path is not None
    source_hashes = {
        name: sha256_path(path)
        for name, path in source_paths.items()
        if name != "view_video"
    }
    view_receipt = {
        "schema_id": "simulation.presentation.view_derivation.v1",
        "producer_id": spec.view_producer_id,
        "producer_source_sha256": source_text_sha256(root / spec.view_producer_source_path),
        "role": "rgb",
        "capture_id": str(capture["capture_id"]),
        "output_video_sha256": sha256_path(source_paths["view_video"]),
        "source_artifact_sha256": source_hashes,
        "source_time_range_s": list(source_time_range),
        "source_frame_count": len(frames),
        "source_timestamp_mapping": "rgb_frames.jsonl contiguous frame_index and 30 fps simulation stamps",
        "camera_head_transform": camera_head_receipt,
        "presentation_transform": acceptance["presentation_transform"],
        "map_version": None,
        "object_state_version": None,
        "capture_acceptance": {
            "mechanism": "reviewed_exact_capture_allowlist",
            "catalog_sha256": acceptance["catalog_sha256"],
            "presentation_classification": acceptance["presentation_classification"],
            "presentation_transform": acceptance["presentation_transform"],
            "cryptographic_execution_attestation": False,
        },
    }
    _write_json_atomic(receipt, view_receipt)

    artifacts = {
        name: _relative_descriptor(path, output_manifest)
        for name, path in source_paths.items()
    }
    artifacts["view_manifest"] = _relative_descriptor(receipt, output_manifest)
    role = {
        "contract": "rgb",
        "status": "complete",
        "provenance": "genuine",
        "schema_id": spec.schema_id,
        "producer_id": spec.producer_id,
        "producer_source_sha256": source_text_sha256(root / spec.producer_source_path),
        "capture_id": str(capture["capture_id"]),
        "map_version": None,
        "object_state_version": None,
        "acceptance": {
            "mechanism": "reviewed_exact_capture_allowlist",
            "catalog_sha256": acceptance["catalog_sha256"],
            "presentation_classification": acceptance["presentation_classification"],
            "presentation_transform": acceptance["presentation_transform"],
            "cryptographic_execution_attestation": False,
        },
        "artifacts": artifacts,
    }
    result = {
        "schema_version": 2,
        "label": f"reviewed-allowlist RGB presentation bundle for capture {capture['capture_id']}",
        "ground_truth_consumed": False,
        "roles": {"rgb": role},
    }
    _write_json_atomic(output_manifest, result)

    from .timeline import inspect_inputs, load_plan

    report = inspect_inputs(
        load_plan(root / "config" / "presentation" / "storyboard.yaml"),
        output_manifest,
        rgb_capture_catalog=acceptance_catalog_path,
    )
    if "rgb" not in report.ready_genuine_roles:
        raise RuntimeError(f"emitted RGB bundle did not validate: {report.role_errors.get('rgb', ())}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-manifest", required=True)
    parser.add_argument("--output-manifest", required=True)
    parser.add_argument("--view-manifest", default="")
    parser.add_argument("--repo-root", default="")
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    result = emit_rgb_bundle(
        args.capture_manifest,
        args.output_manifest,
        args.view_manifest or None,
        args.repo_root or None,
        args.ffprobe,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
