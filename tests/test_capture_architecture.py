import json
import shutil
import csv
import importlib.util
import math
import re
import sqlite3
import subprocess
import sys
import threading
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from evaluation.metrics import PoseSample, compute_metrics, interpolate_pose
from simulator.capture.inventory import export_inventory
from simulator.capture.manifest import (
    _python_semantic_hash,
    build_experiment_hashes,
    sha256_file,
    validate_capture_archive,
    validate_capture_for_slam,
)


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "config/scenarios/baseline_straight.yaml"


def _make_sensor_capture(root: Path) -> dict:
    (root / "sensors_bag").mkdir(parents=True, exist_ok=True)
    (root / "sensors_bag/metadata.yaml").write_text("storage_identifier: sqlite3\nrelative_file_paths:\n  - bag_0.db3\n", encoding="utf-8")
    (root / "sensors_bag/bag_0.db3").write_bytes(b"sensor bag fixture")
    (root / "rgb_camera.mp4").write_bytes(b"video")
    (root / "rgb_frames.jsonl").write_text('{"frame_index":0,"stamp_s":1.0,"width":2,"height":2}\n', encoding="utf-8")
    (root / "bag_metadata.json").write_text(json.dumps({"status": "complete", "last_stamp_s": {"/sim/lidar/points": 1.0}}), encoding="utf-8")
    (root / "effective_config.json").write_text(json.dumps({"lidar": {"hz": 10.0}}), encoding="utf-8")
    (root / "camera_info.json").write_text(json.dumps({"status": "complete", "camera_info_count": 1}), encoding="utf-8")
    (root / "sensor_transforms.json").write_text(json.dumps({"transforms": [], "intrinsics": {}}), encoding="utf-8")
    files = [
        {"path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
        for path in sorted(root.rglob("*")) if path.is_file()
    ]
    manifest = {
        "status": "complete", "manifest_version": 1,
        "bag": {"uri": "sensors_bag", "topics": ["/clock", "/sim/camera/rgb/image_raw", "/sim/camera/rgb/camera_info", "/sim/lidar/points", "/tf", "/tf_static"]},
        "files": files,
    }
    (root / "capture_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return manifest


class CaptureArchitectureTests(unittest.TestCase):
    def test_slam_latch_config_is_only_routed_to_slam_and_final_cloud_is_fresh(self):
        root_config = json.loads((ROOT / "config/mapping/rtabmap/params.yaml").read_text(encoding="utf-8"))
        packaged_config = json.loads((ROOT / "ros2_ws/src/grocery_sim_mapping/config/params.yaml").read_text(encoding="utf-8"))
        self.assertIs(root_config["slam_latch"], False)
        self.assertEqual(packaged_config, root_config)

        captured_nodes = []
        ament = types.ModuleType("ament_index_python")
        ament_packages = types.ModuleType("ament_index_python.packages")
        ament_packages.get_package_share_directory = lambda package: str(ROOT / "ros2_ws/src/grocery_sim_mapping")
        launch = types.ModuleType("launch")
        actions = types.ModuleType("launch.actions")
        substitutions = types.ModuleType("launch.substitutions")
        launch_ros = types.ModuleType("launch_ros")
        ros_actions = types.ModuleType("launch_ros.actions")

        class LaunchDescription:
            def __init__(self, entities): self.entities = entities

        class OpaqueFunction:
            def __init__(self, function): self.function = function

        class DeclareLaunchArgument:
            def __init__(self, name, default_value=None): self.name = name; self.default_value = default_value

        class LaunchConfiguration:
            def __init__(self, name): self.name = name
            def perform(self, context): return context[self.name]

        class Node:
            def __init__(self, **kwargs): self.kwargs = kwargs; captured_nodes.append(self)

        launch.LaunchDescription = LaunchDescription
        actions.OpaqueFunction = OpaqueFunction
        actions.DeclareLaunchArgument = DeclareLaunchArgument
        substitutions.LaunchConfiguration = LaunchConfiguration
        ros_actions.Node = Node
        modules = {
            "ament_index_python": ament,
            "ament_index_python.packages": ament_packages,
            "launch": launch,
            "launch.actions": actions,
            "launch.substitutions": substitutions,
            "launch_ros": launch_ros,
            "launch_ros.actions": ros_actions,
        }
        launch_path = ROOT / "ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py"
        spec = importlib.util.spec_from_file_location("grocery_sim_rtabmap_launch_fixture", launch_path)
        self.assertIsNotNone(spec)
        launch_module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(launch_module)
            description = launch_module.generate_launch_description()
            action = next(entity for entity in description.entities if isinstance(entity, OpaqueFunction))
            with tempfile.TemporaryDirectory() as directory:
                context = {
                    "mapping_params_path": str(ROOT / "config/mapping/rtabmap/params.yaml"),
                    "database_path": str(Path(directory) / "rtabmap.db"),
                    "use_sim_time": "true",
                }
                nodes = action.function(context)
        odom = next(node.kwargs for node in nodes if node.kwargs["package"] == "rtabmap_odom")
        slam = next(node.kwargs for node in nodes if node.kwargs["package"] == "rtabmap_slam")
        odom_parameters = odom["parameters"][0]
        slam_parameters = slam["parameters"][0]
        self.assertNotIn("latch", odom_parameters, "slam_latch must not alter ICP odometry parameters")
        self.assertIs(slam_parameters["latch"], False)

        # The observer still requires a new cloud callback after PublishMap and
        # binds the graph/data/cloud by one exact map-frame timestamp.
        observer_source = (ROOT / "simulator/capture/slam_observer.py").read_text(encoding="utf-8")
        self.assertIn("self.map_messages > self.map_messages_before_publish", observer_source)
        self.assertIn("self.map_graph_stamp_s == self.map_data_stamp_s == self.map_stamp_s", observer_source)

    def test_slam_observer_subscribes_to_launch_resolved_map_topics(self):
        source = (ROOT / "simulator/capture/slam_observer.py").read_text(encoding="utf-8")
        self.assertIn('create_subscription(MapGraph, "/mapGraph"', source)
        self.assertIn('create_subscription(MapData, "/mapData"', source)
        self.assertIn('create_subscription(PointCloud2, "/slam/map_cloud"', source)
        self.assertNotIn('create_subscription(MapGraph, "/rtabmap/mapGraph"', source)
        self.assertNotIn('create_subscription(MapData, "/rtabmap/mapData"', source)

    def test_hashes_are_stable_and_physical_inputs_are_separate(self):
        first = build_experiment_hashes(SCENARIO, ROOT)
        second = build_experiment_hashes(SCENARIO, ROOT)
        self.assertEqual(first, second)
        self.assertEqual(len(first["geometry_sha256"]), 64)
        self.assertEqual(len(first["appearance_sha256"]), 64)

    def test_motion_helper_and_constant_changes_invalidate_capture_products(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            shutil.copytree(ROOT / "config", root / "config")
            shutil.copytree(ROOT / "assets/retail", root / "assets/retail")
            for relative_path in (
                "simulator/environment/aisle_builder.py",
                "simulator/environment/isaac_builder.py",
                "simulator/runtime/isaac_sim_runner.py",
                "simulator/motion/trajectory.py",
            ):
                target = root / relative_path
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(ROOT / relative_path, target)
            manifest_stub = root / "simulator/capture/manifest.py"
            motion_source = root / "simulator/motion/trajectory.py"
            original_motion = motion_source.read_text(encoding="utf-8")
            scenario = root / "config/scenarios/walking_baseline.yaml"

            with patch.dict(build_experiment_hashes.__globals__, {"__file__": str(manifest_stub)}):
                before = build_experiment_hashes(scenario, root)
                helper_mutation = original_motion.replace("35.0 + u *", "35.1 + u *", 1)
                self.assertNotEqual(original_motion, helper_mutation)
                motion_source.write_text(helper_mutation, encoding="utf-8")
                after_helper = build_experiment_hashes(scenario, root)
                self.assertNotEqual(before["trajectory_sha256"], after_helper["trajectory_sha256"])
                self.assertNotEqual(
                    before["product_hashes"]["slam_map_sha256"],
                    after_helper["product_hashes"]["slam_map_sha256"],
                )
                self.assertEqual(before["geometry_sha256"], after_helper["geometry_sha256"])
                self.assertEqual(before["appearance_sha256"], after_helper["appearance_sha256"])

                constant_mutation = original_motion.replace(
                    "_DEFAULT_RAMP_DURATION_S = 2.5", "_DEFAULT_RAMP_DURATION_S = 2.6", 1
                )
                self.assertNotEqual(original_motion, constant_mutation)
                motion_source.write_text(constant_mutation, encoding="utf-8")
                after_constant = build_experiment_hashes(scenario, root)
                self.assertNotEqual(before["trajectory_sha256"], after_constant["trajectory_sha256"])
                self.assertNotEqual(
                    before["product_hashes"]["slam_map_sha256"],
                    after_constant["product_hashes"]["slam_map_sha256"],
                )
                self.assertEqual(before["geometry_sha256"], after_constant["geometry_sha256"])
                self.assertEqual(before["appearance_sha256"], after_constant["appearance_sha256"])

    def test_invalidation_buckets_track_mesh_motion_and_artwork_products_selectively(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            shutil.copytree(ROOT / "config", root / "config")
            shutil.copytree(ROOT / "assets/retail", root / "assets/retail")
            scenario = root / "config/scenarios/baseline_straight.yaml"
            manifest_path = root / "assets/retail/manifest.json"
            def write_catalog_artifact(path: Path, content: bytes, hash_field: str) -> None:
                """Keep the copied catalog internally valid for deliberate mutations."""
                path.write_bytes(content)
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                path_field = "usd_path" if hash_field == "usd_sha256" else "texture_path"
                relative_path = path.relative_to(manifest_path.parent).as_posix()
                entry = next(asset for asset in manifest["assets"] if asset[path_field] == relative_path)
                entry[hash_field] = sha256_file(path)
                manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

            dimensions_before = json.loads(manifest_path.read_text(encoding="utf-8"))["assets"][0]["dimensions_m"]
            first = build_experiment_hashes(scenario, root)
            usd = root / "assets/retail/usd/cereal_sunrise.usda"
            authored_geometry = usd.read_bytes()
            write_catalog_artifact(usd, authored_geometry + b"\n# non-semantic audit comment\n", "usd_sha256")
            comment_only = build_experiment_hashes(scenario, root)
            self.assertEqual(first["geometry_sha256"], comment_only["geometry_sha256"])
            self.assertEqual(first["appearance_sha256"], comment_only["appearance_sha256"])
            self.assertIn(b"0.15500", authored_geometry)
            write_catalog_artifact(usd, authored_geometry.replace(b"0.15500", b"0.15600", 1), "usd_sha256")
            dimensions_after = json.loads(manifest_path.read_text(encoding="utf-8"))["assets"][0]["dimensions_m"]
            second = build_experiment_hashes(scenario, root)
            self.assertEqual(dimensions_before, dimensions_after)
            self.assertNotEqual(first["geometry_sha256"], second["geometry_sha256"])
            self.assertEqual(first["appearance_sha256"], second["appearance_sha256"])
            self.assertEqual(first["trajectory_sha256"], second["trajectory_sha256"])
            self.assertNotEqual(first["product_hashes"]["slam_map_sha256"], second["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(first["product_hashes"]["rgb_perception_sha256"], second["product_hashes"]["rgb_perception_sha256"])

            write_catalog_artifact(usd, authored_geometry, "usd_sha256")
            normals_before = build_experiment_hashes(scenario, root)
            normal_bytes = b"normal3f[] normals = [(0, 1, 0)]"
            self.assertIn(normal_bytes, authored_geometry)
            write_catalog_artifact(
                usd, authored_geometry.replace(normal_bytes, b"normal3f[] normals = [(0, 0, 1)]", 1), "usd_sha256"
            )
            normals_after = build_experiment_hashes(scenario, root)
            self.assertEqual(normals_before["geometry_sha256"], normals_after["geometry_sha256"])
            self.assertNotEqual(normals_before["appearance_sha256"], normals_after["appearance_sha256"])
            self.assertEqual(normals_before["product_hashes"]["slam_map_sha256"], normals_after["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(normals_before["product_hashes"]["rgb_perception_sha256"], normals_after["product_hashes"]["rgb_perception_sha256"])

            write_catalog_artifact(usd, authored_geometry, "usd_sha256")
            uv_before = build_experiment_hashes(scenario, root)
            uv_bytes = b"texCoord2f[] primvars:st = [(0, 0), (1, 0)"
            self.assertIn(uv_bytes, authored_geometry)
            write_catalog_artifact(
                usd, authored_geometry.replace(uv_bytes, b"texCoord2f[] primvars:st = [(0, 0), (0.9, 0)", 1), "usd_sha256"
            )
            uv_after = build_experiment_hashes(scenario, root)
            self.assertEqual(uv_before["geometry_sha256"], uv_after["geometry_sha256"])
            self.assertNotEqual(uv_before["appearance_sha256"], uv_after["appearance_sha256"])
            self.assertEqual(uv_before["product_hashes"]["slam_map_sha256"], uv_after["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(uv_before["product_hashes"]["rgb_perception_sha256"], uv_after["product_hashes"]["rgb_perception_sha256"])
            write_catalog_artifact(usd, authored_geometry, "usd_sha256")

            interpolation_before = build_experiment_hashes(scenario, root)
            normal_interpolation = re.compile(
                rb'(normal3f\[\]\s+normals\s*=\s*\[[^\]]*\]\s*\(\s*interpolation\s*=\s*")uniform("\s*\))'
            )
            vertex_interpolation_geometry, replacement_count = normal_interpolation.subn(
                rb'\1vertex\2', authored_geometry, count=1
            )
            self.assertEqual(replacement_count, 1, "expected an authored normals property with uniform interpolation")
            write_catalog_artifact(usd, vertex_interpolation_geometry, "usd_sha256")
            interpolation_after = build_experiment_hashes(scenario, root)
            self.assertEqual(interpolation_before["geometry_sha256"], interpolation_after["geometry_sha256"])
            self.assertNotEqual(interpolation_before["appearance_sha256"], interpolation_after["appearance_sha256"])
            self.assertEqual(interpolation_before["product_hashes"]["slam_map_sha256"], interpolation_after["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(interpolation_before["product_hashes"]["rgb_perception_sha256"], interpolation_after["product_hashes"]["rgb_perception_sha256"])
            write_catalog_artifact(usd, authored_geometry, "usd_sha256")

            color_before = build_experiment_hashes(scenario, root)
            body_color = b"color3f inputs:diffuseColor = (0.94902, 0.65882, 0.18039)"
            self.assertIn(body_color, usd.read_bytes())
            write_catalog_artifact(
                usd,
                usd.read_bytes().replace(body_color, b"color3f inputs:diffuseColor = (0.94902, 0.65882, 0.18038)", 1),
                "usd_sha256",
            )
            color_after = build_experiment_hashes(scenario, root)
            self.assertEqual(color_before["geometry_sha256"], color_after["geometry_sha256"])
            self.assertNotEqual(color_before["appearance_sha256"], color_after["appearance_sha256"])
            self.assertEqual(color_before["product_hashes"]["slam_map_sha256"], color_after["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(color_before["product_hashes"]["rgb_perception_sha256"], color_after["product_hashes"]["rgb_perception_sha256"])

            write_catalog_artifact(usd, authored_geometry, "usd_sha256")
            artwork_before = build_experiment_hashes(scenario, root)
            texture = root / "assets/retail/textures/cereal_sunrise.png"
            write_catalog_artifact(texture, texture.read_bytes() + b"artwork mutation", "texture_sha256")
            artwork_after = build_experiment_hashes(scenario, root)
            self.assertEqual(artwork_before["geometry_sha256"], artwork_after["geometry_sha256"])
            self.assertNotEqual(artwork_before["appearance_sha256"], artwork_after["appearance_sha256"])
            self.assertEqual(artwork_before["product_hashes"]["slam_map_sha256"], artwork_after["product_hashes"]["slam_map_sha256"])
            self.assertNotEqual(artwork_before["product_hashes"]["rgb_perception_sha256"], artwork_after["product_hashes"]["rgb_perception_sha256"])

            walking_scenario = root / "config/scenarios/walking_baseline.yaml"
            motion_before = build_experiment_hashes(walking_scenario, root)
            walking = root / "config/trajectories/walking.yaml"
            walking.write_text(walking.read_text(encoding="utf-8").replace('"sway_amplitude_m": 0.035', '"sway_amplitude_m": 0.045'), encoding="utf-8")
            motion_after = build_experiment_hashes(walking_scenario, root)
            self.assertNotEqual(motion_before["trajectory_sha256"], motion_after["trajectory_sha256"])
            self.assertEqual(motion_before["geometry_sha256"], motion_after["geometry_sha256"])
            self.assertEqual(motion_before["appearance_sha256"], motion_after["appearance_sha256"])
            self.assertNotEqual(motion_before["product_hashes"]["slam_map_sha256"], motion_after["product_hashes"]["slam_map_sha256"])

            with tempfile.TemporaryDirectory(dir=root) as source_dir:
                before_source = Path(source_dir) / "before.py"
                after_source = Path(source_dir) / "after.py"
                before_source.write_text("def sample(t):\n    return t + 1\n", encoding="utf-8")
                after_source.write_text("def sample(t):\n    # comment only\n    return t + 1\n", encoding="utf-8")
                self.assertEqual(_python_semantic_hash(before_source), _python_semantic_hash(after_source))

    def test_inventory_export_is_deterministic_and_has_semantic_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            first_dir = Path(directory) / "first"
            second_dir = Path(directory) / "second"
            result = export_inventory(SCENARIO, first_dir)
            repeated = export_inventory(SCENARIO, second_dir)
            rows = json.loads((first_dir / "inventory_ground_truth.json").read_text(encoding="utf-8"))
            repeated_rows = json.loads((second_dir / "inventory_ground_truth.json").read_text(encoding="utf-8"))
            self.assertEqual(result, repeated)
            self.assertEqual(rows, repeated_rows)
            self.assertEqual(result["asset_count"], len(rows))
            self.assertGreaterEqual(len(rows), 2000)
            self.assertTrue(all(row["semantic_id"].startswith("retail/") for row in rows))
            semantic_ids = [row["semantic_id"] for row in rows]
            self.assertEqual(len(semantic_ids), len(set(semantic_ids)))
            self.assertIn("width_m", rows[0])

    def test_slam_validation_does_not_require_ground_truth(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            manifest = _make_sensor_capture(root)
            self.assertTrue(validate_capture_for_slam(root, manifest)["gt_required"] is False)
            result = subprocess.run(
                [sys.executable, "-m", "simulator.capture.manifest", "--validate-for-slam", str(root)],
                cwd=ROOT,
                capture_output=True,
                text=True,
                check=True,
            )
            self.assertEqual(json.loads(result.stdout)["gt_required"], False)
            sensor_metadata = json.loads((root / "bag_metadata.json").read_text(encoding="utf-8"))
            sensor_metadata["last_stamp_s"]["/sim/lidar/points"] = 2.0
            (root / "bag_metadata.json").write_text(json.dumps(sensor_metadata), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum mismatch: bag_metadata.json"):
                validate_capture_for_slam(root, manifest)
            sensor_metadata["last_stamp_s"]["/sim/lidar/points"] = 1.0
            (root / "bag_metadata.json").write_text(json.dumps(sensor_metadata), encoding="utf-8")
            effective_config = json.loads((root / "effective_config.json").read_text(encoding="utf-8"))
            effective_config["lidar"]["hz"] = 20.0
            (root / "effective_config.json").write_text(json.dumps(effective_config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum mismatch: effective_config.json"):
                validate_capture_for_slam(root, manifest)
            (root / "effective_config.json").write_text(json.dumps({"lidar": {"hz": 10.0}}), encoding="utf-8")
            transforms = (root / "sensor_transforms.json").read_text(encoding="utf-8")
            (root / "sensor_transforms.json").write_text(transforms.replace("[]", "{}"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum mismatch: sensor_transforms.json"):
                validate_capture_for_slam(root, manifest)
            (root / "sensor_transforms.json").write_text(transforms, encoding="utf-8")
            (root / "sensors_bag/bag_0.db3").unlink()
            with self.assertRaisesRegex(ValueError, "sensor input is missing: sensors_bag/bag_0.db3"):
                validate_capture_for_slam(root, manifest)

    def test_archive_validation_is_separate_and_requires_truth_files(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            root = Path(directory)
            manifest = _make_sensor_capture(root)
            names = (
                "scene_manifest.json", "sensor_transforms.json", "effective_config.json", "rgb_camera.mp4", "rgb_frames.jsonl",
                "experiment_hashes.json", "inventory_ground_truth.csv", "inventory_ground_truth.json",
                "provenance.json", "contracts.yaml", "scenario.yaml", "bag_metadata.json", "rgb_video.json", "camera_info.json",
            )
            for name in names:
                (root / name).write_text("fixture\n", encoding="utf-8")
            (root / "sensors_bag/metadata.yaml").write_text("storage_identifier: sqlite3\nrelative_file_paths:\n  - bag_0.db3\n", encoding="utf-8")
            listed = []
            for path in sorted(root.rglob("*")):
                if path.is_file():
                    listed.append({"path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "size_bytes": path.stat().st_size})
            manifest["ground_truth"] = {"evaluation_only": True, "inventory_csv": "inventory_ground_truth.csv", "inventory_json": "inventory_ground_truth.json"}
            manifest["files"] = listed
            self.assertTrue(validate_capture_for_slam(root, manifest)["status"] == "valid")
            self.assertTrue(validate_capture_archive(root, manifest)["truth_required"])
            (root / "inventory_ground_truth.csv").write_text("truth!!\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "checksum mismatch: inventory_ground_truth.csv"):
                validate_capture_archive(root, manifest)
            (root / "inventory_ground_truth.csv").write_text("fixture\n", encoding="utf-8")
            (root / "inventory_ground_truth.json").unlink()
            self.assertTrue(validate_capture_for_slam(root, manifest)["status"] == "valid")
            with self.assertRaisesRegex(ValueError, "missing evaluation/provenance files"):
                validate_capture_archive(root, manifest)

    def test_mapping_and_perception_scripts_have_truth_separated_entrypoints(self):
        mapping = (ROOT / "scripts/run_slam_offline.ps1").read_text(encoding="utf-8")
        attempt_start = mapping.index("$slamAttempt = Start-SlamAttempt -SlamDirectory $slamDir")
        mapping_preflight = mapping.index("--validate-for-slam $captureDir")
        mapping_config_read = mapping.index('"bag_metadata.json"')
        self.assertLess(attempt_start, mapping_preflight)
        self.assertLess(mapping_preflight, mapping_config_read)
        topic_selection = next(line for line in mapping.splitlines() if "$replayTopics =" in line)
        self.assertIn("--clock", mapping)
        self.assertIn('"--topics"', topic_selection)
        self.assertNotIn('"/clock"', topic_selection)
        self.assertIn('"database_path:=$databaseArg"', mapping)
        self.assertIn('"--target-clock-seconds",([string]$targetClockStamp)', mapping)
        self.assertIn('"--expected-first-clock-seconds",([string]$firstClockStamp)', mapping)
        self.assertIn('"--clock-start-tolerance-seconds",([string]$clockStartTolerance)', mapping)
        self.assertIn('"--delay",([string]$replayDiscoveryDelaySeconds)', mapping)
        self.assertIn('status -ne "pending_database_validation"', mapping)
        self.assertIn('$attemptDatabase = [string]$slamAttempt.database_path', mapping)
        self.assertIn('$databaseArg = $attemptDatabase.Replace', mapping)
        mapper_stop = mapping.index("Stop-ProcessTree -RootPid $mappingProcess.Id")
        mapper_exit = mapping.index("$mappingProcess.WaitForExit(30000)")
        database_validation = mapping.index('"scripts/validate_rtabmap_db.py"')
        database_publication = mapping.index("$databasePublication = Publish-ValidatedDatabase")
        observer_completion = mapping.index('$observerMeta.status = "complete"')
        slam_manifest = mapping.index("Write-AtomicJson -Value $slamManifest")
        self.assertLess(mapper_stop, database_validation)
        self.assertLess(mapper_exit, database_validation)
        self.assertLess(database_validation, database_publication)
        self.assertLess(database_publication, observer_completion)
        self.assertLess(database_validation, observer_completion)
        self.assertLess(observer_completion, slam_manifest)
        self.assertNotIn("service call /rtabmap/publish_map", mapping)

        perception = (ROOT / "scripts/run_inventory_offline.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$PerceptionOnly", perception)
        self.assertIn("--validate-for-slam $capture", perception)
        self.assertIn("--validate-archive $capture", perception)
        perception_result = perception.index('"perception_manifest.json"')
        evaluation_skip = perception.index("if ($PerceptionOnly)", perception_result)
        evaluation_launch = perception.index('"simulator.perception.inventory_evaluation"')
        self.assertLess(evaluation_skip, evaluation_launch)
        self.assertLess(perception.index("return", evaluation_skip), evaluation_launch)

    def test_perception_only_powershell_entrypoint_runs_without_truth(self):
        powershell = shutil.which("pwsh")
        if not powershell:
            self.skipTest("PowerShell is unavailable")
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            run_dir = Path(directory)
            capture = run_dir / "capture"
            capture.mkdir()
            _make_sensor_capture(capture)
            self.assertFalse((capture / "inventory_ground_truth.csv").exists())
            slam = run_dir / "slam"
            slam.mkdir()
            (slam / "slam_manifest.json").write_text(json.dumps({"status": "complete"}), encoding="utf-8")
            workspace = run_dir / "ros"
            workspace.mkdir()
            (workspace / "pixi.toml").write_text("[workspace]\n", encoding="utf-8")
            pixi = run_dir / "pixi-test.ps1"
            pixi.write_text(
                """if ($args -contains '--validate-for-slam') {
  & $env:CAPTURE_TEST_PYTHON -m simulator.capture.manifest --validate-for-slam $args[-1]
  $global:LASTEXITCODE = $LASTEXITCODE
  return
}
if ($args -contains '--validate-archive') {
  & $env:CAPTURE_TEST_PYTHON -m simulator.capture.manifest --validate-archive $args[-1]
  $global:LASTEXITCODE = $LASTEXITCODE
  return
}
if ($args -contains 'simulator.perception.inventory_evaluation') {
  Set-Content -LiteralPath $env:CAPTURE_TEST_EVALUATION_MARKER -Value 'called'
  $global:LASTEXITCODE = 0
  return
}
$outIndex = [Array]::IndexOf($args, '--output-dir')
if ($outIndex -ge 0) {
  $output = $args[$outIndex + 1]
  New-Item -ItemType Directory -Force -Path $output | Out-Null
  Set-Content -LiteralPath (Join-Path $output 'perception_manifest.json') -Value '{"status":"complete"}'
  Set-Content -LiteralPath (Join-Path $output 'estimated_inventory.csv') -Value 'semantic_id,count'
}
$global:LASTEXITCODE = 0
""",
                encoding="utf-8",
            )
            marker = run_dir / "evaluation-called.txt"
            environment = __import__("os").environ.copy()
            environment["CAPTURE_TEST_PYTHON"] = sys.executable
            environment["CAPTURE_TEST_EVALUATION_MARKER"] = str(marker)
            command = [
                powershell, "-NoProfile", "-File", str(ROOT / "scripts/run_inventory_offline.ps1"),
                "-RunDir", str(run_dir), "-CaptureDir", str(capture), "-PixiPath", str(pixi),
                "-RosWorkspace", str(workspace), "-PerceptionOnly",
            ]
            result = subprocess.run(command, cwd=ROOT, env=environment, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertFalse(marker.exists(), "PerceptionOnly must skip evaluation")
            self.assertTrue((run_dir / "perception/estimated_inventory.csv").is_file())

            archive_run = run_dir / "archive-mode"
            archive_capture = archive_run / "capture"
            archive_capture.mkdir(parents=True)
            archive_manifest = _make_sensor_capture(archive_capture)
            archive_names = (
                "scene_manifest.json", "sensor_transforms.json", "experiment_hashes.json",
                "inventory_ground_truth.csv", "inventory_ground_truth.json", "provenance.json",
                "contracts.yaml", "scenario.yaml", "rgb_video.json", "camera_info.json",
            )
            for name in archive_names:
                (archive_capture / name).write_text("fixture\n", encoding="utf-8")
            archive_manifest["ground_truth"] = {
                "evaluation_only": True,
                "inventory_csv": "inventory_ground_truth.csv",
                "inventory_json": "inventory_ground_truth.json",
            }
            archive_manifest["files"] = [
                {"path": path.relative_to(archive_capture).as_posix(), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
                for path in sorted(archive_capture.rglob("*")) if path.is_file()
            ]
            (archive_capture / "capture_manifest.json").write_text(json.dumps(archive_manifest), encoding="utf-8")
            (archive_capture / "inventory_ground_truth.csv").write_text("corrupt\n", encoding="utf-8")
            archive_command = [
                powershell, "-NoProfile", "-File", str(ROOT / "scripts/run_inventory_offline.ps1"),
                "-RunDir", str(archive_run), "-CaptureDir", str(archive_capture), "-PixiPath", str(pixi),
                "-RosWorkspace", str(workspace),
            ]
            archive_result = subprocess.run(archive_command, cwd=ROOT, env=environment, capture_output=True, text=True)
            self.assertNotEqual(archive_result.returncode, 0)
            self.assertIn("Full evaluation archive validation failed", archive_result.stderr)
            self.assertFalse((archive_run / "perception").exists(), "archive corruption must stop before perception/evaluation")

    def test_reopened_mapping_database_reports_integrity_and_input_span(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            database = Path(directory) / "rtabmap.db"
            connection = sqlite3.connect(database)
            with connection:
                connection.execute("create table Node (stamp real)")
                connection.executemany("insert into Node values (?)", [(1.0,), (2.0,)])
            connection.close()
            command = [sys.executable, str(ROOT / "scripts/validate_rtabmap_db.py"), str(database), "--minimum-node-stamp", "2.0", "--scan-period-seconds", "0.1"]
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(result.stdout)["integrity_check"], "ok")
            bad_command = [sys.executable, str(ROOT / "scripts/validate_rtabmap_db.py"), str(database), "--minimum-node-stamp", "3.0", "--scan-period-seconds", "0.1"]
            with self.assertRaises(subprocess.CalledProcessError):
                subprocess.run(bad_command, cwd=ROOT, capture_output=True, text=True, check=True)

    def test_raw_writer_exception_still_closes_writer_and_node_idempotently(self):
        from simulator.capture.rosbag_capture import RawCaptureWriter

        class Node:
            def __init__(self): self.destroy_calls = 0
            def destroy_node(self): self.destroy_calls += 1

        class Writer:
            def __init__(self): self.close_calls = 0
            def write(self, *args): raise OSError("writer failed")
            def close(self): self.close_calls += 1

        raw = RawCaptureWriter.__new__(RawCaptureWriter)
        bag_writer = Writer()
        raw.writer = bag_writer; raw.node = Node(); raw.serialize_message = lambda message: b"raw"
        raw.last_clock_s = 0.0; raw.first_clock_s = None; raw.target_clock_s = None
        raw.target_wall_deadline = None; raw.duration_s = 1.0; raw.post_target_wall_s = 2.0
        raw.counts = {"/clock": 0}; raw.first_stamp_s = {"/clock": None}; raw.last_stamp_s = {"/clock": None}
        raw._closed = False; raw._close_result = None; raw._close_error = None; raw._callback_error = None
        raw.close_timeout_s = 1.0
        message = type("ClockMessage", (), {"clock": type("Clock", (), {"sec": 0, "nanosec": 0})()})()
        with self.assertRaisesRegex(OSError, "writer failed"):
            raw._callback("/clock")(message)
        with self.assertRaisesRegex(RuntimeError, "did not finish cleanly"):
            raw.close()
        with self.assertRaisesRegex(RuntimeError, "close failed"):
            raw.close()
        self.assertEqual(raw.node.destroy_calls, 1)
        self.assertEqual(bag_writer.close_calls, 1)
        self.assertEqual(raw.writer, None)

    def test_rgb_close_releases_writer_after_failed_recording(self):
        with patch.dict(sys.modules, {"cv2": types.ModuleType("cv2"), "numpy": types.ModuleType("numpy")}):
            from evaluation.rgb_video_recorder import RgbVideoRecorder

        class Writer:
            def __init__(self): self.release_calls = 0
            def release(self):
                self.release_calls += 1
                raise OSError("release failed")

        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            recorder = RgbVideoRecorder.__new__(RgbVideoRecorder)
            writer_resource = Writer()
            recorder.writer = writer_resource; recorder.output = Path(directory) / "rgb.mp4"
            recorder.metadata_path = Path(directory) / "rgb.json"
            recorder.frames_path = None; recorder.camera_info_path = None
            recorder.first_image_stamp_s = None; recorder.last_image_stamp_s = None
            recorder.frames_written = 0; recorder.done_reason = "writer failed"
            recorder.width = None; recorder.height = None; recorder.codec = "avc1"
            recorder.target_s = None; recorder.last_clock_s = None; recorder.encoding = None
            recorder.invalid_frames = 0; recorder.frame_records = []; recorder.camera_info_record = None
            recorder._closed = False; recorder._close_metadata = None
            recorder.close_timeout_s = 1.0
            with self.assertRaisesRegex(RuntimeError, "RGB video recording failed"):
                recorder.close()
            with self.assertRaisesRegex(RuntimeError, "RGB video recording failed"):
                recorder.close()
            self.assertEqual(writer_resource.release_calls, 1)
            self.assertEqual(json.loads(recorder.metadata_path.read_text(encoding="utf-8"))["status"], "failed")

    def test_stalled_native_writer_closes_are_bounded_and_single_owner(self):
        from simulator.capture.rosbag_capture import RawCaptureWriter
        with patch.dict(sys.modules, {"cv2": types.ModuleType("cv2"), "numpy": types.ModuleType("numpy")}):
            from evaluation.rgb_video_recorder import RgbVideoRecorder

        class Node:
            def destroy_node(self): pass

        class StalledResource:
            def __init__(self, method):
                self.method = method; self.calls = 0
                self.started = threading.Event(); self.unblock = threading.Event(); self.finished = threading.Event()
            def close(self): self._wait()
            def release(self): self._wait()
            def _wait(self):
                self.calls += 1
                self.started.set()
                self.unblock.wait()
                self.finished.set()

        raw_resource = StalledResource("close")
        raw = RawCaptureWriter.__new__(RawCaptureWriter)
        raw.writer = raw_resource; raw.node = Node(); raw.close_timeout_s = 0.02
        raw._closed = False; raw._close_result = None; raw._close_error = None; raw._callback_error = None
        with self.assertRaisesRegex(RuntimeError, "did not finish cleanly"):
            raw.close()
        self.assertTrue(raw_resource.started.wait(0.5))
        with self.assertRaisesRegex(RuntimeError, "close failed"):
            raw.close()
        raw_resource.unblock.set()
        self.assertTrue(raw_resource.finished.wait(0.5))
        self.assertEqual(raw_resource.calls, 1)

        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            rgb_resource = StalledResource("release")
            recorder = RgbVideoRecorder.__new__(RgbVideoRecorder)
            recorder.writer = rgb_resource; recorder.output = Path(directory) / "rgb.mp4"
            recorder.metadata_path = Path(directory) / "rgb.json"
            recorder.frames_path = None; recorder.camera_info_path = None
            recorder.first_image_stamp_s = None; recorder.last_image_stamp_s = None
            recorder.frames_written = 0; recorder.done_reason = "writer failed"
            recorder.width = 2; recorder.height = 2; recorder.codec = "avc1"
            recorder.target_s = None; recorder.last_clock_s = None; recorder.encoding = None
            recorder.invalid_frames = 0; recorder.frame_records = []; recorder.camera_info_record = None
            recorder._closed = False; recorder._close_metadata = None; recorder.close_timeout_s = 0.02
            with self.assertRaisesRegex(RuntimeError, "RGB video recording failed"):
                recorder.close()
            self.assertTrue(rgb_resource.started.wait(0.5))
            metadata = json.loads(recorder.metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["status"], "failed")
            self.assertIn("TimeoutError", metadata["writer_error"])
            with self.assertRaisesRegex(RuntimeError, "RGB video recording failed"):
                recorder.close()
            rgb_resource.unblock.set()
            self.assertTrue(rgb_resource.finished.wait(0.5))
            self.assertEqual(rgb_resource.calls, 1)

    def test_capture_entrypoints_finalize_on_callback_exceptions(self):
        from simulator.capture import rosbag_capture

        class FakeRclpy(types.ModuleType):
            def __init__(self):
                super().__init__("rclpy")
                self.shutdown_calls = 0
            def init(self): pass
            def ok(self): return self.shutdown_calls == 0
            def shutdown(self): self.shutdown_calls += 1

        raw_rclpy = FakeRclpy()

        class FakeRawCapture:
            instance = None
            def __init__(self, *args):
                self.closed = 0
                self._closed = False
                FakeRawCapture.instance = self
            def spin_until_done(self): raise OSError("raw callback failed")
            def close(self): self.closed += 1; self._closed = True

        with patch.dict(sys.modules, {"rclpy": raw_rclpy}), patch.object(rosbag_capture, "RawCaptureWriter", FakeRawCapture), patch.object(sys, "argv", ["rosbag_capture", "--output", "bag", "--duration-seconds", "1", "--metadata", "meta"]):
            with self.assertRaisesRegex(OSError, "raw callback failed"):
                rosbag_capture.main()
        self.assertEqual(FakeRawCapture.instance.closed, 1)
        self.assertEqual(raw_rclpy.shutdown_calls, 1)

        with patch.dict(sys.modules, {"cv2": types.ModuleType("cv2"), "numpy": types.ModuleType("numpy")}):
            from evaluation import rgb_video_recorder
        rgb_rclpy = FakeRclpy()

        class FakeNode:
            def __init__(self): self.destroy_calls = 0
            def destroy_node(self): self.destroy_calls += 1

        class FakeRgbRecorder:
            instance = None
            def __init__(self, *args):
                self.node = FakeNode(); self.close_calls = 0
                FakeRgbRecorder.instance = self
            def spin_until_done(self): raise OSError("RGB callback failed")
            def close(self): self.close_calls += 1

        with patch.dict(sys.modules, {"rclpy": rgb_rclpy}), patch.object(rgb_video_recorder, "RgbVideoRecorder", FakeRgbRecorder), patch.object(sys, "argv", ["rgb_video_recorder", "--output", "video.mp4", "--metadata", "meta.json", "--duration-seconds", "1"]):
            with self.assertRaisesRegex(OSError, "RGB callback failed"):
                rgb_video_recorder.main()
        self.assertEqual(FakeRgbRecorder.instance.node.destroy_calls, 1)
        self.assertEqual(FakeRgbRecorder.instance.close_calls, 1)
        self.assertEqual(rgb_rclpy.shutdown_calls, 1)

    def test_slam_replay_requires_process_signal_clock_target_and_drain(self):
        from simulator.capture.slam_observer import SlamObserver

        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            signal = Path(directory) / "bag_replay.complete"
            signal.write_text("done\n", encoding="utf-8")
            class Future:
                def __init__(self): self.is_done = False
                def done(self): return self.is_done
                def result(self): return type("PublishMapResponse", (), {})()

            class FakeClient:
                def __init__(self): self.future = None; self.request = None
                def wait_for_service(self, timeout_sec): return True
                def call_async(self, request): self.request = request; self.future = Future(); return self.future

            class GraphFuture:
                def __init__(self, response): self.response = response
                def done(self): return True
                def result(self): return self.response

            class FakeGraphClient:
                def __init__(self, responses): self.responses = list(responses); self.requests = []
                def wait_for_service(self, timeout_sec): return True
                def call_async(self, request):
                    self.requests.append(request)
                    return GraphFuture(self.responses.pop(0))

            class GetMapRequest:
                def __init__(self): self.global_map = False; self.optimized = False; self.graph_only = True

            def optimized_graph_response(first_node_x):
                def pose(x):
                    return type("Pose", (), {
                        "position": type("Vector", (), {"x": x, "y": 0.0, "z": 0.0})(),
                        "orientation": type("Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                    })()
                def node(node_id, stamp_s, x):
                    raw_pose = type("Pose", (), {
                        "position": type("Vector", (), {"x": x, "y": 0.0, "z": 0.0})(),
                        "orientation": type("Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                    })()
                    return type("NodeData", (), {"id": node_id, "stamp": stamp_s, "pose": raw_pose})()
                nodes = [node(1, 11.5, 0.0), node(2, 12.0, 1.0)]
                map_to_odom = type("Transform", (), {
                    "translation": type("Vector", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
                    "rotation": type("Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                })()
                graph = type("MapGraph", (), {"poses_id": [1, 2], "poses": [pose(first_node_x), pose(5.0)], "map_to_odom": map_to_odom})()
                stamp = type("Stamp", (), {"sec": 12, "nanosec": 0})()
                header = type("Header", (), {"frame_id": "map", "stamp": stamp})()
                data = type("MapData", (), {"header": header, "graph": graph, "nodes": nodes})()
                graph_message = type("MapGraphMessage", (), {
                    "header": header, "poses_id": graph.poses_id, "poses": graph.poses, "map_to_odom": graph.map_to_odom,
                })()
                return data, graph_message

            class PointCloudReader:
                @staticmethod
                def read_points(message, **kwargs): return [(4.0, 0.0, 0.0)]

            class FakeRos:
                def __init__(self): self.observer = None; self.spin_count = 0
                def ok(self): return True
                def spin_once(self, node, timeout_sec):
                    self.spin_count += 1
                    future = self.observer.publish_map_client.future
                    if future is not None and not future.done():
                        message = type("Message", (), {
                            "fields": [type("Field", (), {"name": name})() for name in ("x", "y", "z")],
                            "header": type("Header", (), {"stamp": type("Stamp", (), {"sec": 12, "nanosec": 0})(), "frame_id": "map"})(),
                        })()
                        self.observer._on_map(message)
                        data, graph_message = optimized_graph_response(4.0)
                        self.observer._on_map_data(data)
                        self.observer._on_map_graph(graph_message)
                        future.is_done = True
                    else:
                        __import__("time").sleep(timeout_sec)

            class Node:
                def destroy_node(self): pass

            observer = SlamObserver.__new__(SlamObserver)
            ros = FakeRos()
            observer.rclpy = ros; observer.node = Node()
            observer.started_wall = __import__("time").monotonic() - 5.0; observer.startup_timeout_s = 30.0
            observer.first_clock_s = 10.0; observer.last_clock_s = 12.0; observer.target_clock_s = 12.0
            observer.expected_first_clock_s = 10.0; observer.clock_start_tolerance_s = 0.1; observer.clock_start_covered = True
            observer.replay_span_s = 2.0
            observer.clock_regressions = 0; observer.clock_target_reached = True
            observer.replay_complete_signal = signal; observer.replay_complete_signal_observed = False
            observer.drain_complete = False; observer.callback_generation = 0; observer.drain_quiet_polls = 0
            observer.replay_drained = False; observer.publish_map_acknowledged = False
            observer.final_map_span = False; observer.mapper_database_span = None
            observer.database_node_count = None; observer.database_last_stamp_s = None
            observer.database_verification_stage = "pending_post_mapper_shutdown"
            observer.expected_sensor_last_stamp_s = 12.0; observer.sensor_scan_period_s = 0.1
            observer.last_odom_stamp_s = 11.95
            # Deliberately append unique samples out of timestamp order, as can
            # happen when callbacks are delivered late during replay.
            observer.odom_rows = [
                {"timestamp_s": 11.95, "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "quaternion_valid": 1.0},
                {"timestamp_s": 11.8, "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "quaternion_valid": 1.0},
            ]
            observer.map_to_odom_rows = []
            def transform_message(seconds, x):
                stamp = type("Stamp", (), {"sec": int(seconds), "nanosec": int((seconds % 1) * 1_000_000_000)})()
                transform = type("Transform", (), {
                    "header": type("Header", (), {"frame_id": "map", "stamp": stamp})(),
                    "child_frame_id": "odom",
                    "transform": type("TransformData", (), {
                        "translation": type("Vector", (), {"x": x, "y": 0.0, "z": 0.0})(),
                        "rotation": type("Quaternion", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})(),
                    })(),
                })()
                return type("TFMessage", (), {"transforms": [transform]})()
            observer._on_tf(transform_message(11.9, 1.0))
            observer._on_tf(transform_message(12.0, 3.0))
            observer.map_pose_rows = []
            observer.graph_pose_version = None; observer.optimized_graph_last_stamp_s = None
            observer.optimized_pose_graph_complete = False; observer.map_graph_matches_final_cloud = False
            observer.pre_publish_graph_version = None
            observer.latest_map = []; observer.map_stamp_s = None; observer.map_messages = 0
            observer.map_graph_messages = 0; observer.map_data_messages = 0
            observer.map_graph_before_publish = 0; observer.map_data_before_publish = 0
            observer.map_graph_message = None; observer.map_data_message = None
            observer.map_graph_stamp_s = None; observer.map_data_stamp_s = None
            observer.map_cloud_frame_id = None; observer.final_map_graph_frame_id = None
            observer.final_map_graph_stamp_s = None; observer.final_cloud_stamp_s = None
            observer.point_cloud2 = PointCloudReader; observer.publish_map_client = FakeClient(); observer.close_timeout_s = 5.0
            observer.publish_map_request_factory = type("PublishMapRequest", (), {"__init__": lambda self: None})
            ros.observer = observer
            pre_data, pre_graph = optimized_graph_response(4.0)
            pre_cloud = type("Message", (), {
                "fields": [type("Field", (), {"name": name})() for name in ("x", "y", "z")],
                "header": type("Header", (), {"stamp": type("Stamp", (), {"sec": 12, "nanosec": 0})(), "frame_id": "map"})(),
            })()
            observer._on_map(pre_cloud); observer._on_map_data(pre_data); observer._on_map_graph(pre_graph)
            observer.spin_until_done()
            self.assertTrue(observer.replay_complete_signal_observed)
            self.assertTrue(observer.drain_complete)
            self.assertTrue(observer.replay_drained)
            self.assertTrue(observer.publish_map_acknowledged)
            self.assertIsNone(observer.mapper_database_span)
            self.assertTrue(observer.final_map_span)
            self.assertEqual(observer.map_messages_before_publish, 1)
            self.assertEqual(observer.map_messages, 2)
            self.assertEqual(observer.final_map_graph_stamp_s, observer.final_cloud_stamp_s)
            self.assertTrue(observer.publish_map_client.request.global_map)
            self.assertTrue(observer.publish_map_client.request.optimized)
            self.assertFalse(observer.publish_map_client.request.graph_only)
            self.assertTrue(observer.map_graph_matches_final_cloud)
            self.assertTrue(observer.optimized_pose_graph_complete)
            dense_version_before_repeat = observer.dense_pose_version
            map_version_before_repeat = observer.map_version
            observer._capture_final_optimized_graph()
            self.assertEqual(observer.dense_pose_version, dense_version_before_repeat)
            self.assertEqual(observer.map_version, map_version_before_repeat)
            observer.output_dir = Path(directory) / "observer_output"
            observer.output_dir.mkdir()
            result = observer.close()
            self.assertEqual(result["status"], "pending_database_validation")
            self.assertTrue(result["processed_sensor_span"])
            self.assertIsNone(result["mapper_database_span"])
            self.assertTrue(result["final_map_span"])
            self.assertIsNone(result["database_last_stamp_s"])
            self.assertEqual(result["database_verification_stage"], "pending_post_mapper_shutdown")
            self.assertTrue(result["map_pose_correction_complete"])
            self.assertEqual(result["pose_source"], "rtabmap_optimized_graph")
            self.assertTrue(result["map_graph_matches_final_cloud"])
            self.assertTrue(result["graph_pose_version"])
            self.assertEqual(result["final_map_graph_stamp_s"], result["final_cloud_stamp_s"])
            self.assertEqual(result["final_map_graph_frame_id"], "map")
            with (observer.output_dir / "slam_map_poses.csv").open(encoding="utf-8") as handle:
                corrected = list(csv.DictReader(handle))
            with (observer.output_dir / "slam_odom_poses.csv").open(encoding="utf-8") as handle:
                raw = list(csv.DictReader(handle))
            with (observer.output_dir / "slam_poses.csv").open(encoding="utf-8") as handle:
                legacy_map = list(csv.DictReader(handle))
            self.assertEqual(corrected[0]["frame_id"], "map")
            self.assertAlmostEqual(float(corrected[0]["x_m"]), 4.0, places=6)
            self.assertAlmostEqual(float(corrected[0]["x_m"]), observer.latest_map[0][0], places=6)
            self.assertNotAlmostEqual(float(corrected[0]["x_m"]), 2.0, places=6, msg="final optimized graph pose must supersede historical incremental TF correction")
            self.assertEqual(raw[0]["frame_id"], "odom")
            expected_timestamps = [11.8, 11.95]
            raw_timestamps = [float(row["timestamp_s"]) for row in raw]
            corrected_timestamps = [float(row["timestamp_s"]) for row in corrected]
            legacy_map_timestamps = [float(row["timestamp_s"]) for row in legacy_map]
            self.assertEqual(raw_timestamps, expected_timestamps)
            self.assertEqual(corrected_timestamps, expected_timestamps)
            self.assertEqual(legacy_map_timestamps, expected_timestamps)
            self.assertEqual([float(row["x_m"]) for row in raw], [0.0, 0.0])
            self.assertEqual(len(raw), len(observer.odom_rows))
            artifact_record = next(item for item in result["files"] if item["path"] == "slam_map_poses.csv")
            self.assertEqual(artifact_record["sha256"], sha256_file(observer.output_dir / "slam_map_poses.csv"))
            self.assertEqual(artifact_record["map_version"], result["map_version"])
            self.assertEqual(artifact_record["frame_id"], "map")
            self.assertTrue(artifact_record["optimized"])
            self.assertEqual(len(corrected), len(raw))
            keyframes = next(item for item in result["files"] if item["path"] == "slam_map_keyframes.csv")
            self.assertEqual(keyframes["map_version"], result["map_version"])
            self.assertEqual(keyframes["schema_version"], 2)
            with (observer.output_dir / "slam_map_keyframes.csv").open(encoding="utf-8") as handle:
                keyframe_rows = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(keyframe_rows[0]["x_m"]), 4.0, places=6)
            self.assertAlmostEqual(float(keyframe_rows[0]["odom_x_m"]), 0.0, places=6)
            self.assertAlmostEqual(float(keyframe_rows[0]["correction_x_m"]), 4.0, places=6)
            self.assertTrue(result["pre_publish_graph_version"])
            self.assertEqual(result["graph_pose_version"], result["pre_publish_graph_version"])
            self.assertTrue(result["dense_pose_version"])
            self.assertTrue(result["map_version"])
            for filename in ("slam_odom_poses.csv", "slam_map_poses.csv", "slam_poses.csv"):
                record = next(item for item in result["files"] if item["path"] == filename)
                output_path = observer.output_dir / filename
                self.assertEqual(record["size_bytes"], output_path.stat().st_size)
                self.assertEqual(record["sha256"], sha256_file(output_path))

            # A later loop closure changes node 1's optimized pose while raw
            # odometry stays fixed; the production exporter follows the final graph.
            loop_closed_data, loop_closed_graph = optimized_graph_response(6.0)
            observer.map_data_message = loop_closed_data
            observer.map_graph_message = loop_closed_graph
            observer._capture_final_optimized_graph()
            loop_closed_latest_sample = next(row for row in observer.map_pose_rows if float(row["timestamp_s"]) == 11.95)
            self.assertAlmostEqual(float(loop_closed_latest_sample["x_m"]), 4.2, places=6)
            self.assertNotEqual(observer.graph_pose_version, result["pre_publish_graph_version"])

            data, graph_message = optimized_graph_response(4.0)
            observer.map_graph_message = graph_message
            observer.map_data_message = data
            observer.map_graph_stamp_s = 12.0
            observer.map_data_stamp_s = 11.9
            with self.assertRaisesRegex(RuntimeError, "stamps do not identify one publication"):
                observer._capture_final_optimized_graph()

            clock_observer = SlamObserver.__new__(SlamObserver)
            clock_observer.callback_generation = 0; clock_observer.last_clock_s = None
            clock_observer.first_clock_s = None; clock_observer.target_clock_s = 5.0
            clock_observer.expected_first_clock_s = 4.0; clock_observer.clock_start_tolerance_s = 0.1
            clock_observer.clock_start_covered = False; clock_observer.clock_regressions = 0
            def clock_message(seconds):
                return type("Message", (), {"clock": type("Clock", (), {"sec": seconds, "nanosec": 0})()})()
            clock_observer._on_clock(clock_message(4))
            clock_observer._on_clock(clock_message(5))
            clock_observer._on_clock(clock_message(4))
            self.assertTrue(clock_observer.clock_target_reached)
            self.assertEqual(clock_observer.clock_regressions, 1)

            missed_start = SlamObserver.__new__(SlamObserver)
            missed_start.callback_generation = 0; missed_start.last_clock_s = None; missed_start.first_clock_s = None
            missed_start.target_clock_s = 20.5; missed_start.expected_first_clock_s = 0.066666666
            missed_start.clock_start_tolerance_s = 0.1; missed_start.clock_start_covered = False
            missed_start.replay_span_s = 20.433333334
            missed_start.clock_regressions = 0; missed_start.clock_target_reached = False
            missed_start._on_clock(clock_message(5.4564382))
            missed_start._on_clock(clock_message(20.5))
            self.assertTrue(missed_start.clock_target_reached, "target is the absolute capture horizon")
            self.assertFalse(missed_start.clock_start_covered, "missing the beginning must remain a strict failure")
            missed_start.rclpy = type("Ros", (), {"ok": lambda self: True, "spin_once": lambda self, node, timeout_sec: None})()
            missed_start.node = object(); missed_start.started_wall = 0.0; missed_start.startup_timeout_s = 30.0
            missed_start.replay_complete_signal = signal; missed_start.replay_complete_signal_observed = False
            missed_start.callback_generation = 0; missed_start.drain_quiet_polls = 0
            missed_start.expected_sensor_last_stamp_s = 20.5; missed_start.last_odom_stamp_s = 20.5
            missed_start.sensor_scan_period_s = 0.1
            with patch("simulator.capture.slam_observer.time.monotonic", side_effect=[0.0, 1.1]):
                with self.assertRaisesRegex(RuntimeError, "missed the beginning"):
                    missed_start.spin_until_done()

            observer = SlamObserver.__new__(SlamObserver)
            class QuietRos:
                def ok(self): return True
                def spin_once(self, node, timeout_sec): pass
            observer.rclpy = QuietRos(); observer.node = object()
            observer.started_wall = __import__("time").monotonic() - 5.0; observer.startup_timeout_s = 30.0
            observer.first_clock_s = 10.0; observer.last_clock_s = 11.5; observer.target_clock_s = 12.0
            observer.expected_first_clock_s = 10.0; observer.clock_start_tolerance_s = 0.1; observer.clock_start_covered = True
            observer.replay_span_s = 2.0
            observer.clock_regressions = 0; observer.clock_target_reached = False
            observer.replay_complete_signal = signal; observer.replay_complete_signal_observed = False
            observer.drain_complete = False; observer.callback_generation = 4; observer.drain_quiet_polls = 0
            observer.expected_sensor_last_stamp_s = 12.0; observer.sensor_scan_period_s = 0.1
            with self.assertRaisesRegex(RuntimeError, "before /clock reached target"):
                observer.spin_until_done()

    def test_map_to_odom_interpolation_composes_nonidentity_time_varying_transform(self):
        from simulator.capture.slam_observer import _correct_odom_pose, _interpolate_map_to_odom

        half = math.sqrt(0.5)
        transforms = [
            {"timestamp_s": 1.0, "x_m": 10.0, "y_m": 0.0, "z_m": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "parent_frame_id": "map", "child_frame_id": "odom"},
            {"timestamp_s": 3.0, "x_m": 12.0, "y_m": 0.0, "z_m": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0, "parent_frame_id": "map", "child_frame_id": "odom"},
        ]
        correction = _interpolate_map_to_odom(transforms, 2.0)
        self.assertIsNotNone(correction)
        odom_pose = {"timestamp_s": 2.0, "x_m": 1.0, "y_m": 0.0, "z_m": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        mapped = _correct_odom_pose(odom_pose, correction)
        self.assertAlmostEqual(float(mapped["x_m"]), 11.0, places=6)
        self.assertAlmostEqual(float(mapped["y_m"]), 1.0, places=6)
        self.assertAlmostEqual(float(mapped["qz"]), half, places=6)
        self.assertAlmostEqual(float(mapped["qw"]), half, places=6)
        self.assertEqual(mapped["frame_id"], "map")
        self.assertIsNone(_interpolate_map_to_odom(transforms, 0.9), "TF extrapolation must fail closed")

    def test_nonfinite_tf_is_excluded_and_close_manifest_fails_incomplete(self):
        from simulator.capture.slam_observer import SlamObserver

        class Node:
            def destroy_node(self): pass

        def odometry_message():
            stamp = types.SimpleNamespace(sec=1, nanosec=0)
            position = types.SimpleNamespace(x=1.0, y=0.0, z=0.0)
            orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            return types.SimpleNamespace(
                header=types.SimpleNamespace(stamp=stamp),
                pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=position, orientation=orientation)),
            )

        for bad_stamp, bad_x in ((float("nan"), 0.0), (1.0, float("inf"))):
            with self.subTest(stamp=bad_stamp, translation=bad_x), tempfile.TemporaryDirectory(dir=ROOT) as directory:
                observer = SlamObserver.__new__(SlamObserver)
                observer.callback_generation = 0
                observer.odom_rows = []
                observer.last_odom_stamp_s = None
                observer.map_to_odom_rows = []
                observer._on_odom(odometry_message())
                bad_tf = types.SimpleNamespace(transforms=[types.SimpleNamespace(
                    header=types.SimpleNamespace(
                        frame_id="map",
                        stamp=types.SimpleNamespace(sec=bad_stamp, nanosec=0),
                    ),
                    child_frame_id="odom",
                    transform=types.SimpleNamespace(
                        translation=types.SimpleNamespace(x=bad_x, y=0.0, z=0.0),
                        rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                )])
                observer._on_tf(bad_tf)
                self.assertEqual(observer.map_to_odom_rows, [])
                observer.output_dir = Path(directory)
                observer.latest_map = []
                observer.map_stamp_s = None
                observer.map_messages = 0
                observer.expected_sensor_last_stamp_s = 1.0
                observer.sensor_scan_period_s = 0.1
                observer.last_odom_stamp_s = 1.0
                observer.first_clock_s = 0.0; observer.last_clock_s = 1.0; observer.target_clock_s = 1.0
                observer.expected_first_clock_s = 0.0; observer.clock_start_tolerance_s = 0.1; observer.clock_start_covered = True
                observer.replay_span_s = 1.0
                observer.clock_target_reached = True; observer.clock_regressions = 0
                observer.replay_complete_signal_observed = False; observer.drain_complete = False
                observer.drain_quiet_polls = 0; observer.replay_drained = False
                observer.publish_map_acknowledged = False; observer.map_messages_before_publish = 0
                observer.final_map_span = False; observer.mapper_database_span = None
                observer.database_node_count = None; observer.database_last_stamp_s = None
                observer.database_verification_stage = "pending_post_mapper_shutdown"
                observer.map_pose_rows = []
                observer.graph_pose_version = None; observer.optimized_graph_last_stamp_s = None
                observer.optimized_pose_graph_complete = False; observer.map_graph_matches_final_cloud = False
                observer.pre_publish_graph_version = None
                observer.node = Node()
                result = observer.close()
                self.assertEqual(result["status"], "incomplete")
                self.assertFalse(result["map_pose_correction_complete"])
                self.assertFalse(result["optimized_pose_graph_complete"])
                with (observer.output_dir / "slam_map_poses.csv").open(encoding="utf-8") as handle:
                    self.assertEqual(list(csv.DictReader(handle)), [])
                for record in result["files"]:
                    path = observer.output_dir / record["path"]
                    self.assertEqual(record["size_bytes"], path.stat().st_size)
                    self.assertEqual(record["sha256"], sha256_file(path))

    def test_interpolation_rejects_invalid_quaternion_and_uses_brackets(self):
        truth = [
            PoseSample(0.0, (0.0, 0.0, 0.0)),
            PoseSample(1.0, (2.0, 0.0, 0.0)),
        ]
        interpolated = interpolate_pose(truth, 0.25, 1.0)
        self.assertAlmostEqual(interpolated.position_m[0], 0.5)
        estimate = [
            PoseSample(0.25, (0.5, 0.0, 0.0)),
            PoseSample(0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        ]
        metrics = compute_metrics(truth, estimate, max_time_gap_s=1.0)
        self.assertEqual(metrics.sample_count, 1)
