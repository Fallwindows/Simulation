"""Canonical experiment/capture manifests and compatibility validation.

The hash inputs intentionally separate physical geometry and sensor/trajectory
configuration from appearance assets.  Offline SLAM consumes the former and
the immutable sensor bag; changing a texture must not invalidate that result.
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import hashlib
import json
import re
import subprocess
from pathlib import Path
from typing import Any, Iterable

CAPTURE_MANIFEST_VERSION = 1
REQUIRED_CAPTURE_TOPICS = (
    "/clock",
    "/sim/camera/rgb/image_raw",
    "/sim/camera/rgb/camera_info",
    "/sim/lidar/points",
    "/tf",
    "/tf_static",
)


def _jsonable(value: Any) -> Any:
    if dataclasses.is_dataclass(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, dict):
        return {str(key): _jsonable(value[key]) for key in sorted(value)}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, float):
        # JSON's representation is stable enough for the finite config values
        # used by this project, while rejecting accidental NaN/Inf inputs.
        if not __import__("math").isfinite(value):
            raise ValueError("non-finite values cannot be hashed")
        return value
    return value


def canonical_json(value: Any) -> bytes:
    return (json.dumps(_jsonable(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_sha(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _relative_file_hashes(root: Path, paths: Iterable[Path]) -> list[dict[str, str]]:
    entries: list[dict[str, str]] = []
    for path in sorted((Path(item).resolve() for item in paths), key=lambda item: item.as_posix()):
        if not path.is_file():
            continue
        try:
            relative = path.relative_to(root.resolve()).as_posix()
        except ValueError:
            relative = path.name
        entries.append({"path": relative, "sha256": sha256_file(path)})
    return entries


class _StripDocstrings(ast.NodeTransformer):
    def _strip(self, node):
        self.generic_visit(node)
        if node.body and isinstance(node.body[0], ast.Expr) and isinstance(node.body[0].value, (ast.Constant,)) and isinstance(node.body[0].value.value, str):
            node.body = node.body[1:]
        return node

    visit_Module = _strip
    visit_ClassDef = _strip
    visit_FunctionDef = _strip
    visit_AsyncFunctionDef = _strip


def _python_semantic_hash(path: Path, function_names: set[str] | None = None, trajectory_only: bool = False) -> str:
    tree = _StripDocstrings().visit(ast.parse(path.read_text(encoding="utf-8")))
    if function_names is not None:
        selected = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.name in function_names]
        tree = ast.Module(body=selected, type_ignores=[])
    if trajectory_only:
        run = next((node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run"), None)
        if run is None:
            return sha256_json([])
        selected_body = [node for node in run.body if any(isinstance(child, ast.Name) and child.id in {"trajectory", "trajectory_cls"} for child in ast.walk(node)) or any(isinstance(child, ast.Attribute) and child.attr == "trajectory" for child in ast.walk(node))]
        tree = ast.Module(body=selected_body, type_ignores=[])
    return sha256_json(ast.dump(tree, include_attributes=False))


def _usda_scope(text: str, scope_name: str) -> str:
    text = re.sub(r"(?m)^\s*#.*$", "", text)
    match = re.search(rf'\bdef\s+\w+\s+"{re.escape(scope_name)}"\s*\{{', text)
    if match is None:
        return ""
    start = text.find("{", match.start())
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    raise ValueError(f"USD scope {scope_name!r} has unbalanced braces")


def _usda_geometry_hash(path: Path) -> str:
    geometry = _usda_scope(path.read_text(encoding="utf-8"), "Geometry")
    # Material and artwork bindings/UVs affect appearance but not mesh identity.
    geometry = re.sub(r'(?m)^\s*prepend\s+apiSchemas\s*=\s*\["MaterialBindingAPI"\]\s*$', "", geometry)
    geometry = re.sub(r"(?m)^\s*rel\s+material:binding\s*=.*$", "", geometry)
    geometry = re.sub(r"(?ms)^\s*texCoord\w*\[\]\s+primvars:[^\n]+(?:\n\s*\([^)]*\))?\s*", "", geometry)
    geometry = re.sub(r"(?m)^\s*normal3f\[\]\s+normals\s*=.*$", "", geometry)
    geometry = re.sub(r"(?m)^\s*uniform\s+token\[\]\s+normals:interpolation\s*=.*$", "", geometry)
    return sha256_json(re.sub(r"\s+", "", geometry))


def _usda_appearance_hash(path: Path) -> str:
    appearance = _usda_scope(path.read_text(encoding="utf-8"), "Looks")
    return sha256_json(re.sub(r"\s+", "", appearance))


def _usda_surface_hash(path: Path) -> str:
    geometry = _usda_scope(path.read_text(encoding="utf-8"), "Geometry")
    authored_surface_data = re.findall(
        r"(?ms)^\s*(?:normal\d+f\[\]\s+normals|texCoord\d+f\[\]\s+primvars:[\w:]+)\s*=\s*\[.*?\](?:\s*\([^)]*\))?",
        geometry,
    )
    return sha256_json(re.sub(r"\s+", "", "\n".join(authored_surface_data)))


def _bag_metadata_files(metadata_path: Path) -> list[str]:
    """Read rosbag2's declared storage shards from its checksummed metadata."""

    text = metadata_path.read_text(encoding="utf-8-sig")
    try:
        import yaml  # type: ignore

        document = yaml.safe_load(text)
    except ImportError:
        document = None
    except Exception as exc:
        raise ValueError(f"invalid rosbag metadata: {metadata_path}") from exc
    if document is not None:
        info = document.get("rosbag2_bagfile_information", document) if isinstance(document, dict) else None
        values = info.get("relative_file_paths") if isinstance(info, dict) else None
    else:
        values = None
        lines = text.splitlines()
        for index, line in enumerate(lines):
            if re.match(r"^\s*relative_file_paths\s*:\s*(?:\[\s*\])?\s*$", line):
                values = []
                for child in lines[index + 1:]:
                    if child.strip() and not child[0].isspace():
                        break
                    match = re.match(r"^\s+-\s*(['\"]?)([^'\"]+?)\1\s*$", child)
                    if match:
                        values.append(match.group(2).strip())
                break
    if not isinstance(values, list) or not values:
        raise ValueError("rosbag metadata does not declare relative_file_paths")
    result: list[str] = []
    for value in values:
        relative = Path(str(value).replace("\\", "/"))
        if relative.is_absolute() or not relative.parts or any(part in {"", ".", ".."} for part in relative.parts):
            raise ValueError(f"rosbag metadata declares an unsafe storage path: {value!r}")
        normalized = relative.as_posix()
        if normalized in result:
            raise ValueError(f"rosbag metadata repeats a storage path: {normalized}")
        result.append(normalized)
    return result


def build_experiment_hashes(scenario_path: str | Path, repo_root: str | Path | None = None) -> dict[str, Any]:
    """Build stable physical/sensor/trajectory/appearance hashes for a scenario."""

    from simulator.config.loader import load_scenario
    from simulator.environment.aisle_builder import build_aisle_layout
    from simulator.environment.retail_catalog import load_retail_catalog

    scenario_file = Path(scenario_path).resolve()
    root = Path(repo_root).resolve() if repo_root else scenario_file.parents[1]
    scenario_data = json.loads(scenario_file.read_text(encoding="utf-8"))
    environment_file = (scenario_file.parent / scenario_data["environment"]).resolve()
    sensor_file = (scenario_file.parent / scenario_data["sensors"]).resolve()
    trajectory_file = (scenario_file.parent / scenario_data["trajectory"]).resolve()
    scenario = load_scenario(scenario_file)
    layout = build_aisle_layout(scenario.environment)
    catalog = load_retail_catalog(scenario.environment.asset_manifest_path)
    records = {asset.asset_key: asset for asset in catalog.assets}
    inventory = []
    for asset in layout.assets:
        record = records[asset.asset_key]
        inventory.append({
            "semantic_id": asset.semantic_id,
            "asset_key": asset.asset_key,
            "category": asset.category,
            "position_m": list(asset.position_m),
            "rotation_rpy_deg": list(asset.rotation_rpy_deg),
            "scale_xyz": list(asset.scale_xyz),
            "dimensions_m": list(record.dimensions_m),
        })
    primitives = [
        {"name": item.name, "center_m": list(item.center_m), "size_m": list(item.size_m), "kind": item.kind}
        for item in layout.primitives
    ]
    environment_source = Path(__file__).resolve().parents[1] / "environment"
    runtime_source = Path(__file__).resolve().parents[1] / "runtime" / "isaac_sim_runner.py"
    motion_source = Path(__file__).resolve().parents[1] / "motion" / "trajectory.py"
    environment_config = dataclasses.asdict(scenario.environment)
    try:
        environment_config["asset_manifest_path"] = catalog.manifest_path.relative_to(root).as_posix()
    except ValueError:
        environment_config["asset_manifest_path"] = catalog.manifest_path.as_posix()
    geometry_dependencies = {
        "environment_config": environment_config,
        "catalog_geometry": [
            {
                "asset_key": asset.asset_key,
                "category": asset.category,
                "dimensions_m": asset.dimensions_m,
                "local_front_axis": asset.local_front_axis,
                "model_type": asset.model_type,
            }
            for asset in catalog.assets
        ],
        "asset_meshes": [
            {"asset_key": asset.asset_key, "path": asset.usd_path.relative_to(root).as_posix(), "mesh_sha256": _usda_geometry_hash(asset.usd_path)}
            for asset in catalog.assets
        ],
        "builder_code": {
            "aisle_builder": _python_semantic_hash(environment_source / "aisle_builder.py"),
            "isaac_builder": _python_semantic_hash(environment_source / "isaac_builder.py"),
            "runtime_scene_builders": _python_semantic_hash(runtime_source, {"_build_world", "_build_sensor_rig"}),
        },
    }
    motion_dependencies = {
        "trajectory_config": dataclasses.asdict(scenario.trajectory),
        "trajectory_code": _python_semantic_hash(motion_source, {"StraightTrajectory", "WalkingTrajectory", "PoseSample"}),
        "runtime_trajectory_use": _python_semantic_hash(runtime_source, trajectory_only=True),
    }
    geometry_payload = {
        "primitives": primitives,
        "assets": inventory,
        "seed": layout.seed,
        "authored_geometry_dependencies": geometry_dependencies,
    }
    sensor_payload = {
        "camera": dataclasses.asdict(scenario.camera),
        "lidar": dataclasses.asdict(scenario.lidar),
        "sensor_overrides": scenario.sensor_overrides,
    }
    trajectory_payload = {
        "config": dataclasses.asdict(scenario.trajectory),
        "authored_motion_dependencies": motion_dependencies,
    }
    appearance_files = {
        "asset_materials": [
            {
                "asset_key": asset.asset_key,
                "product_name": asset.product_name,
                "looks_sha256": _usda_appearance_hash(asset.usd_path),
                "surface_sha256": _usda_surface_hash(asset.usd_path),
            }
            for asset in catalog.assets
        ],
        "artwork": _relative_file_hashes(root, [asset.texture_path for asset in catalog.assets]),
    }
    geometry_sha256 = sha256_json(geometry_payload)
    sensor_sha256 = sha256_json(sensor_payload)
    trajectory_sha256 = sha256_json(trajectory_payload)
    appearance_sha256 = sha256_json(appearance_files)
    slam_map_product = sha256_json({"geometry": geometry_sha256, "sensor": sensor_sha256, "trajectory": trajectory_sha256})
    rgb_perception_product = sha256_json({"slam_map": slam_map_product, "appearance": appearance_sha256})
    return {
        "geometry_sha256": geometry_sha256,
        "inventory_sha256": sha256_json(inventory),
        "trajectory_sha256": trajectory_sha256,
        "sensor_sha256": sensor_sha256,
        "appearance_sha256": appearance_sha256,
        "product_hashes": {"slam_map_sha256": slam_map_product, "rgb_perception_sha256": rgb_perception_product},
        "inputs": {
            "scenario": scenario_file.relative_to(root).as_posix() if scenario_file.is_relative_to(root) else scenario_file.as_posix(),
            "environment_config": environment_file.relative_to(root).as_posix() if environment_file.is_relative_to(root) else environment_file.as_posix(),
            "sensor_config": sensor_file.relative_to(root).as_posix() if sensor_file.is_relative_to(root) else sensor_file.as_posix(),
            "trajectory_config": trajectory_file.relative_to(root).as_posix() if trajectory_file.is_relative_to(root) else trajectory_file.as_posix(),
            "asset_manifest": str(Path(scenario.environment.asset_manifest_path).resolve().relative_to(root)).replace("\\", "/")
            if Path(scenario.environment.asset_manifest_path).resolve().is_relative_to(root)
            else str(Path(scenario.environment.asset_manifest_path).resolve()),
            "geometry_dependencies": geometry_dependencies,
            "motion_dependencies": motion_dependencies,
            "appearance_files": appearance_files,
        },
        "git_sha": _git_sha(root),
    }


def capture_hash(manifest_without_hash: dict[str, Any]) -> str:
    payload = dict(manifest_without_hash)
    payload.pop("capture_sha256", None)
    return sha256_json(payload)


def _load_manifest(capture_dir: Path, manifest: dict[str, Any] | None) -> dict[str, Any]:
    path = capture_dir / "capture_manifest.json"
    return manifest if manifest is not None else json.loads(path.read_text(encoding="utf-8-sig"))


def _verify_manifest_files(root: Path, data: dict[str, Any], required_paths: set[str]) -> None:
    listed = data.get("files")
    if not isinstance(listed, list):
        raise ValueError("capture manifest file checksums are missing")
    entries: dict[str, dict[str, Any]] = {}
    for item in listed:
        if not isinstance(item, dict):
            raise ValueError("capture manifest contains an invalid file checksum entry")
        relative = str(item.get("path", "")).replace("\\", "/")
        if not relative or relative in entries:
            raise ValueError(f"capture manifest contains an empty or duplicate file path: {relative!r}")
        entries[relative] = item
    missing_entries = sorted(required_paths - entries.keys())
    if missing_entries:
        raise ValueError(f"capture sensor inputs are missing checksums: {missing_entries}")
    for relative in sorted(required_paths):
        path = (root / relative).resolve()
        try:
            path.relative_to(root.resolve())
        except ValueError as exc:
            raise ValueError(f"capture sensor input escapes capture directory: {relative}") from exc
        if not path.is_file():
            raise ValueError(f"capture sensor input is missing: {relative}")
        entry = entries[relative]
        try:
            expected_size = int(entry["size_bytes"])
            expected_sha256 = str(entry["sha256"]).lower()
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"capture sensor input checksum entry is invalid: {relative}") from exc
        if path.stat().st_size != expected_size:
            raise ValueError(f"capture sensor input size mismatch: {relative}")
        if sha256_file(path) != expected_sha256:
            raise ValueError(f"capture sensor input checksum mismatch: {relative}")


def validate_capture_for_slam(capture_dir: str | Path, manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    """Validate the sensor-only interface required by offline SLAM.

    Ground-truth inventory and pose files are deliberately not required here;
    this is the isolation test that prevents evaluation data leakage into SLAM.
    """

    root = Path(capture_dir).resolve()
    data = _load_manifest(root, manifest)
    if data.get("status") != "complete":
        raise ValueError("capture manifest is not complete")
    if int(data.get("manifest_version", -1)) != CAPTURE_MANIFEST_VERSION:
        raise ValueError("unsupported capture manifest version")
    missing = [topic for topic in REQUIRED_CAPTURE_TOPICS if topic not in data.get("bag", {}).get("topics", [])]
    if missing:
        raise ValueError(f"capture bag is missing required sensor topics: {missing}")
    bag_uri = (root / str(data.get("bag", {}).get("uri", ""))).resolve()
    try:
        bag_uri.relative_to(root)
    except ValueError as exc:
        raise ValueError("capture bag path escapes capture directory") from exc
    if not bag_uri.is_dir():
        raise ValueError(f"capture bag does not exist: {bag_uri}")
    bag_metadata_relative = (bag_uri / "metadata.yaml").relative_to(root).as_posix()
    required_paths = {
        "rgb_camera.mp4", "rgb_frames.jsonl", "camera_info.json", "sensor_transforms.json",
        "bag_metadata.json", "effective_config.json", bag_metadata_relative,
    }
    _verify_manifest_files(root, data, required_paths)
    bag_metadata = bag_uri / "metadata.yaml"
    bag_base = bag_uri.relative_to(root).as_posix().rstrip("/")
    listed_paths = {
        str(item.get("path", "")).replace("\\", "/")
        for item in data.get("files", []) if isinstance(item, dict)
    }
    declared_from_manifest = {path for path in listed_paths if path.startswith(bag_base + "/")}
    required_paths.update(declared_from_manifest)
    for shard in _bag_metadata_files(bag_metadata):
        required_paths.add(f"{bag_base}/{shard}")
    present_bag_payload = {
        path.relative_to(root).as_posix() for path in bag_uri.rglob("*") if path.is_file()
    }
    required_paths.update(present_bag_payload)
    if not (required_paths - {"rgb_camera.mp4", "rgb_frames.jsonl", "camera_info.json", "sensor_transforms.json", "bag_metadata.json", "effective_config.json", bag_metadata_relative}):
        raise ValueError("capture bag metadata does not declare any data files")
    _verify_manifest_files(root, data, required_paths)
    return {"status": "valid", "capture_dir": str(root), "gt_required": False, "topics": list(data["bag"]["topics"]), "verified_sensor_files": len(required_paths)}


def validate_capture_archive(capture_dir: str | Path, manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    """Validate the complete capture archive, including evaluation-only truth."""

    root = Path(capture_dir).resolve()
    data = _load_manifest(root, manifest)
    validate_capture_for_slam(root, data)
    ground_truth = data.get("ground_truth")
    if not isinstance(ground_truth, dict) or ground_truth.get("evaluation_only") is not True:
        raise ValueError("capture archive must declare evaluation-only ground truth")
    required = {
        "rgb_camera.mp4", "rgb_frames.jsonl", "sensors_bag/metadata.yaml",
        "scene_manifest.json", "sensor_transforms.json", "effective_config.json",
        "experiment_hashes.json", "inventory_ground_truth.csv", "inventory_ground_truth.json",
        "provenance.json", "contracts.yaml", "scenario.yaml", "bag_metadata.json",
        "rgb_video.json", "camera_info.json",
    }
    required.update(str(ground_truth.get(key, "")) for key in ("inventory_csv", "inventory_json"))
    missing = sorted(path for path in required if not path or not (root / path).is_file())
    if missing:
        raise ValueError(f"capture archive is missing evaluation/provenance files: {missing}")
    listed = data.get("files")
    if not isinstance(listed, list) or not listed:
        raise ValueError("capture archive file checksums are missing")
    listed_paths = {str(item.get("path", "")) for item in listed}
    unhashed = sorted(required - listed_paths)
    if unhashed:
        raise ValueError(f"capture archive files are missing checksums: {unhashed}")
    _verify_manifest_files(root, data, listed_paths)
    return {"status": "valid", "capture_dir": str(root), "truth_required": True, "file_count": len(listed)}


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate sensor inputs or the full capture archive")
    parser.add_argument("--validate-for-slam", metavar="CAPTURE_DIR")
    parser.add_argument("--validate-archive", metavar="CAPTURE_DIR")
    args = parser.parse_args()
    if bool(args.validate_for_slam) == bool(args.validate_archive):
        parser.error("choose exactly one validation mode")
    result = validate_capture_for_slam(args.validate_for_slam) if args.validate_for_slam else validate_capture_archive(args.validate_archive)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()


def write_json(path: str | Path, value: Any) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(canonical_json(value))
