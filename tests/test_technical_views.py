import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from simulator.capture.manifest import capture_hash, sha256_file, write_json
from simulator.perception.provenance import build_perception_input_bindings
from tests.perception_provenance_fixture import create_perception_run


ROOT = Path(__file__).resolve().parents[1]


def _run_technical(script: str, *arguments: Path, expect_success: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [sys.executable, "-c", script, *(str(path) for path in arguments)],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if expect_success and result.returncode != 0:
        raise AssertionError(f"technical subprocess failed:\nstdout={result.stdout}\nstderr={result.stderr}")
    return result


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_source_fixture(parent: Path, capture_id: str = "fixture-capture") -> tuple[Path, Path, Path]:
    run = parent / capture_id
    for directory in (run / "capture", run / "slam", run / "perception"):
        directory.mkdir(parents=True)
    (run / "slam" / "slam_map.ply").write_text(
        "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n"
        "end_header\n0 0 0\n1 0 0\n1 1 1\n2 0 1\n",
        encoding="utf-8",
    )
    (run / "slam" / "slam_poses.csv").write_text(
        "timestamp_s,x_m,y_m,z_m\n0.2,0,0,0\n20.4,2,0,0\n",
        encoding="utf-8",
    )
    (run / "perception" / "estimated_inventory.csv").write_text(
        "track_id,estimated_x_m,estimated_y_m,estimated_z_m,depth_source,3d_observation_count\n"
        "1,1.0,0.5,0.8,lidar_projected_with_slam_pose,5\n",
        encoding="utf-8",
    )
    extra_payloads = {
        "capture/sensors_bag/sensors_bag_0.db3": b"fixture raw lidar bag",
        "capture/bag_metadata.json": b"{}\n",
        "capture/camera_info.json": b"{}\n",
        "capture/sensor_transforms.json": json.dumps({
            "topics": {"lidar_points": "/sim/lidar/points", "rgb_camera_info": "/sim/camera/rgb/camera_info"},
            "frames": {"lidar_link": "lidar_link", "camera_optical": "camera_optical_frame"},
        }).encode("utf-8"),
        "capture/effective_config.json": json.dumps({"lidar": {"hz": 10.0}}).encode("utf-8"),
        "capture/scene_manifest.json": b"{}\n",
        "capture/rgb_camera.mp4": b"fixture rgb video",
        "capture/rgb_frames.jsonl": b'{"frame_index":0,"stamp_s":0.2,"width":8,"height":4,"frame_id":"camera_optical_frame"}\n',
        "perception/frame_annotations.jsonl": b'{"frame_index":0,"stamp_s":0.2,"width":8,"height":4,"detections":[]}\n',
    }
    for relative, payload in extra_payloads.items():
        path = run / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    revision = subprocess.run(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"], check=True, capture_output=True, text=True,
    ).stdout.strip()
    capture_sha = "a" * 64
    capture = {
        "status": "complete", "capture_id": capture_id, "capture_sha256": capture_sha, "git_sha": revision,
        "files": [
            {
                "path": path.relative_to(run / "capture").as_posix(),
                "sha256": _sha256(path),
                "size_bytes": path.stat().st_size,
            }
            for path in (
                run / "capture/sensors_bag/sensors_bag_0.db3", run / "capture/bag_metadata.json",
                run / "capture/camera_info.json", run / "capture/sensor_transforms.json",
                run / "capture/effective_config.json", run / "capture/scene_manifest.json",
                run / "capture/rgb_camera.mp4", run / "capture/rgb_frames.jsonl",
            )
        ],
    }
    slam = {
        "status": "complete", "capture_id": capture_id, "capture_sha256": capture_sha,
        "git_sha": revision, "ground_truth_subscribed": False,
        "artifacts": [{
            "path": "slam_poses.csv", "sha256": _sha256(run / "slam/slam_poses.csv"),
            "size_bytes": (run / "slam/slam_poses.csv").stat().st_size,
        }],
    }
    perception = {
        "status": "complete", "git_sha": revision, "ground_truth_consumed": False,
        "ground_truth_required": False, "lidar_consumed_for_estimation": True,
        "slam_consumed_for_estimation": True, "estimated_inventory": "estimated_inventory.csv",
        "slam_artifact": "slam/slam_map.pcd",
    }
    for relative, payload in (
        ("capture/capture_manifest.json", capture),
        ("slam/slam_manifest.json", slam),
        ("perception/perception_manifest.json", perception),
    ):
        (run / relative).write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    paths = {
        "capture": run / "capture/capture_manifest.json",
        "slam": run / "slam/slam_manifest.json",
        "perception": run / "perception/perception_manifest.json",
        "map": run / "slam/slam_map.ply",
        "trajectory": run / "slam/slam_poses.csv",
        "inventory": run / "perception/estimated_inventory.csv",
        "raw_lidar_bag": run / "capture/sensors_bag/sensors_bag_0.db3",
        "bag_metadata": run / "capture/bag_metadata.json",
        "camera_info": run / "capture/camera_info.json",
        "sensor_transforms": run / "capture/sensor_transforms.json",
        "effective_config": run / "capture/effective_config.json",
        "scene_manifest": run / "capture/scene_manifest.json",
        "rgb_video": run / "capture/rgb_camera.mp4",
        "rgb_frames": run / "capture/rgb_frames.jsonl",
        "frame_annotations": run / "perception/frame_annotations.jsonl",
    }
    catalog = {
        "schema_version": 1,
        "status": "reviewed_source_catalog",
        "sources": [{
            "source_id": "fixture-v1", "capture_id": capture_id, "run_directory_name": capture_id,
            "presentation_classification": {"kind": "generated_test_fixture", "marker_id": "simulator.presentation.generated-test-fixture.v1", "marker_sha256": "5ac77eb627c557f5f95ca9d9241960beddaf190084aa375f62d893c88ac6825e"},
            "presentation_compatibility": {"status": "generated_test_fixture", "delivery_eligible": True},
            "sensor_contract": {
                "lidar_topic": "/sim/lidar/points", "camera_info_topic": "/sim/camera/rgb/camera_info",
                "lidar_frame_id": "lidar_link", "camera_frame_id": "camera_optical_frame",
                "configured_lidar_hz": 10.0, "maximum_current_age_periods": 2.0,
                "maximum_rgb_skew_ns": 17000001,
            },
            "capture_sha256": capture_sha,
            "producer_revisions": {"capture": revision, "slam": revision, "perception": revision},
            "producer_manifests": {
                name: {"path": path.relative_to(run).as_posix(), "sha256": _sha256(path)}
                for name, path in (("capture", paths["capture"]), ("slam", paths["slam"]), ("perception", paths["perception"]))
            },
            "artifacts": {
                name: {"path": paths[name].relative_to(run).as_posix(), "sha256": _sha256(paths[name]), "version": f"fixture-{name}-v1"}
                for name in (
                    "map", "trajectory", "inventory", "raw_lidar_bag", "bag_metadata", "camera_info",
                    "sensor_transforms", "effective_config", "scene_manifest", "rgb_video", "rgb_frames",
                    "frame_annotations",
                )
            },
            "simulation_time": {"source": "slam/slam_poses.csv:timestamp_s", "start_s": 0.2, "end_s": 20.4},
            "perception_contract": {
                "allowed_depth_sources": ["lidar_projected_with_slam_pose"],
                "estimated_inventory": "estimated_inventory.csv", "slam_artifact": "slam/slam_map.pcd",
                "legacy_capture_id_omitted": True, "ground_truth_consumed": False,
            },
        }],
    }
    catalog_path = parent / "catalog.json"
    catalog_path.write_text(json.dumps(catalog, indent=2), encoding="utf-8")
    reference_path = parent / "references.json"
    reference_path.write_text('{"storyboards": []}\n', encoding="utf-8")
    return run, catalog_path, reference_path


def _refresh_catalog_hash(catalog_path: Path, kind: str, path: Path) -> None:
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    source = catalog["sources"][0]
    group = "producer_manifests" if kind in ("capture", "slam", "perception") else "artifacts"
    source[group][kind]["sha256"] = _sha256(path)
    catalog_path.write_text(json.dumps(catalog, indent=2), encoding="utf-8")


INSPECT_SCRIPT = """
import json, sys
from simulator.technical_views import _inspect_source_bundle_with_catalog
bundle = _inspect_source_bundle_with_catalog(sys.argv[1], sys.argv[2], sys.argv[3])
print(json.dumps({
    'capture_id': bundle.capture_id,
    'source_id': bundle.source_id,
    'time': [bundle.simulation_time_start_s, bundle.simulation_time_end_s],
    'depth_sources': list(bundle.depth_sources),
}))
"""


class TechnicalViewTests(unittest.TestCase):
    def test_plan_has_exact_profiles_order_and_frame_contract(self):
        payload = json.loads((ROOT / "config/technical_views.json").read_text(encoding="utf-8"))
        self.assertEqual((payload["profiles"]["preview"]["width"], payload["profiles"]["preview"]["height"]), (1280, 720))
        self.assertEqual((payload["profiles"]["delivery"]["width"], payload["profiles"]["delivery"]["height"]), (1920, 1080))
        self.assertEqual([item["id"] for item in payload["views"]], [
            "sensor_activation", "lidar_environment", "persistent_map", "object_association",
            "object_detail", "observed_aisle_overview", "final_technical_view",
        ])
        self.assertEqual([item["frames"] for item in payload["views"]], [120, 90, 90, 120, 120, 120, 150])
        self.assertEqual(sum(item["frames"] for item in payload["views"]), 810)
        self.assertEqual(payload["fps"], 30)
        self.assertEqual({item["selective_current_scan_status"] for item in payload["views"]}, {"complete"})
        self.assertEqual(
            [item["selection_mode"] for item in payload["views"]],
            [
                "camera_occlusion_surfaces", "navigation_boundaries", "past_structural_history",
                "estimated_roi_front_surfaces", "estimated_roi_front_surfaces",
                "past_structural_history", "past_structural_history",
            ],
        )
        self.assertTrue(payload["views"][0]["rgb_context"])
        self.assertTrue(all(not item["rgb_context"] for item in payload["views"][1:]))

    def test_numpy_dependent_selectors_and_projection_run_in_subprocess(self):
        script = """
import json, tempfile
from pathlib import Path
import numpy as np
from simulator.technical_views import load_ascii_ply, load_plan, project_points
from simulator.technical_lidar import EstimatedTrajectory, _transform_points
profiles, views = load_plan('config/technical_views.json')
with tempfile.TemporaryDirectory() as temporary:
    path = Path(temporary) / 'map.ply'
    path.write_text('ply\\nformat ascii 1.0\\nelement vertex 4\\nproperty float x\\nproperty float y\\nproperty float z\\nend_header\\n2 -1 0\\n2 1 1\\n4 0 2\\n7 -1 1\\n', encoding='utf-8')
    points = load_ascii_ply(path)
trajectory = np.asarray([[0,0,0],[1,0,0],[2,0,0]], dtype=np.float32)
pixels, depths, mask = project_points(points, np.asarray([0,0,1], dtype=np.float32), np.asarray([4,0,1], dtype=np.float32), 1280, 720)
poses = EstimatedTrajectory(np.asarray([0.,1.]), np.asarray([[0.,0.,0.],[2.,0.,0.]]), np.asarray([[0.,0.,0.,1.],[0.,0.,0.,1.]]))
mapped = _transform_points(poses.map_from_sensor_rig(0.5), np.asarray([[1.,0.,0.]]))
print(json.dumps({'views': len(views), 'pixel_count': len(pixels), 'depth_count': len(depths), 'mask_bool': str(mask.dtype) == 'bool', 'mapped': mapped.tolist()}))
"""
        result = json.loads(_run_technical(script).stdout)
        self.assertEqual(result["views"], 7)
        self.assertEqual(result["pixel_count"], result["depth_count"])
        self.assertTrue(result["mask_bool"])
        self.assertEqual(result["mapped"], [[2.0, 0.0, 0.0]])

    def test_scan_causality_camera_schema_and_timing_contracts_fail_closed(self):
        script = """
import json
import numpy as np
from simulator.technical_lidar import (
    ScanRecord, SelectiveLidarSource, _camera_intrinsics, _validate_pointcloud_contract,
)
source = SelectiveLidarSource.__new__(SelectiveLidarSource)
source.scan_records = (ScanRecord(1, 200_000_000), ScanRecord(2, 300_000_000), ScanRecord(3, 400_000_000))
source.scan_timestamps_s = np.asarray([0.2, 0.3, 0.4])
source.maximum_current_scan_age_s = 0.15
previous, latest, alpha = source.causal_scan_pair(0.35)
failures = []
for stamp in (0.19, 0.56):
    try: source.causal_scan_pair(stamp)
    except ValueError: failures.append(stamp)
configured, configured_receipt = _camera_intrinsics({
    'topic':'/sim/camera/rgb/camera_info','frame_id':'camera_optical_frame',
    'width_px':3840,'height_px':2160,'fx_px':1920.,'fy_px':1920.,'cx_px':1919.5,'cy_px':1079.5,
    'observed_ros_message':False,'provenance':'configured_intrinsics','model':'ideal_pinhole',
})
observed, observed_receipt = _camera_intrinsics({
    'topic':'/sim/camera/rgb/camera_info','stamp_s':0.0,'frame_id':'camera_optical_frame',
    'width':1920,'height':1080,'distortion_model':'plumb_bob','d':[],
    'k':[960.,0.,959.5,0.,960.,539.5,0.,0.,1.],
})
contract_failures = 0
for frame, fields in [('wrong_frame',['x','y','z']),('lidar_link',['x','y','z','time'])]:
    try: _validate_pointcloud_contract(frame, fields, 'lidar_link')
    except ValueError: contract_failures += 1
fields = _validate_pointcloud_contract('lidar_link', ['x','y','z','intensity'], 'lidar_link')
print(json.dumps({
    'pair':[previous.message_id,latest.message_id,round(alpha,3)], 'causal_failures':failures,
    'configured':[configured.width_px,configured_receipt['representation']],
    'observed':[observed.width_px,observed_receipt['representation']],
    'contract_failures':contract_failures,'fields':list(fields),
}))
"""
        result = json.loads(_run_technical(script).stdout)
        self.assertEqual(result["pair"][:2], [1, 2])
        self.assertEqual(result["causal_failures"], [0.19, 0.56])
        self.assertEqual(result["configured"], [3840, "configured_intrinsics"])
        self.assertEqual(result["observed"], [1920, "observed_ros_camera_info"])
        self.assertEqual(result["contract_failures"], 2)
        self.assertEqual(result["fields"], ["x", "y", "z", "intensity"])

    def test_encoded_probe_and_text_fit_contracts_fail_closed(self):
        script = """
import json
from pathlib import Path
from simulator.technical_views import _validate_probe, fit_font, load_plan
profiles, _ = load_plan('config/technical_views.json')
valid = {'width':1920,'height':1080,'fps':30.0,'frame_count':120}
_validate_probe(valid, profiles['delivery'], 120)
failures = 0
for field, value in [('width',1280),('height',720),('fps',29.97),('frame_count',119)]:
    invalid = dict(valid); invalid[field] = value
    try: _validate_probe(invalid, profiles['delivery'], 120)
    except ValueError: failures += 1
fits = []
for height in (720,1080):
    scale = height / 720
    max_width = int((520-42-40)*scale)
    font = fit_font('Persistent track 2465', Path('C:/Windows/Fonts/segoeuib.ttf'), int(42*scale), int(28*scale), max_width)
    box = font.getbbox('Persistent track 2465')
    fits.append(box[2]-box[0] <= max_width)
print(json.dumps({'failures':failures,'fits':fits}))
"""
        result = json.loads(_run_technical(script).stdout)
        self.assertEqual(result, {"failures": 4, "fits": [True, True]})

    def test_reviewed_legacy_source_is_fully_bound(self):
        with tempfile.TemporaryDirectory() as temporary:
            run, catalog, references = _write_source_fixture(Path(temporary))
            result = json.loads(_run_technical(INSPECT_SCRIPT, run, references, catalog).stdout)
            self.assertEqual(result["capture_id"], "fixture-capture")
            self.assertEqual(result["time"], [0.2, 20.4])
            self.assertEqual(result["depth_sources"], ["lidar_projected_with_slam_pose"])

    def test_repaired_producer_contract_requires_exact_perception_input_bindings(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            fixture = create_perception_run(parent, frame_count=613, capture_id="modern-source")
            run = Path(fixture["run"])
            capture_dir = run / "capture"
            slam_dir = run / "slam"
            perception_dir = run / "perception"
            write_json(capture_dir / "scene_manifest.json", {"status": "generated_test_fixture"})
            transforms_path = capture_dir / "sensor_transforms.json"
            transforms_payload = json.loads(transforms_path.read_text(encoding="utf-8"))
            transforms_payload["topics"] = {
                "lidar_points": "/sim/lidar/points", "rgb_camera_info": "/sim/camera/rgb/camera_info",
            }
            transforms_payload["frames"] = {
                "lidar_link": "lidar_link", "camera_optical": "camera_optical_frame",
            }
            write_json(transforms_path, transforms_payload)
            capture_payload = json.loads((capture_dir / "capture_manifest.json").read_text(encoding="utf-8"))
            transform_record = next(item for item in capture_payload["files"] if item["path"] == "sensor_transforms.json")
            transform_record.update(sha256=sha256_file(transforms_path), size_bytes=transforms_path.stat().st_size)
            scene_path = capture_dir / "scene_manifest.json"
            capture_payload["files"].append({
                "path": "scene_manifest.json", "sha256": sha256_file(scene_path), "size_bytes": scene_path.stat().st_size,
            })
            capture_payload["capture_sha256"] = capture_hash(capture_payload)
            write_json(capture_dir / "capture_manifest.json", capture_payload)
            slam_payload = json.loads((slam_dir / "slam_manifest.json").read_text(encoding="utf-8"))
            slam_payload["capture_sha256"] = capture_payload["capture_sha256"]
            write_json(slam_dir / "slam_manifest.json", slam_payload)
            perception_payload = json.loads((perception_dir / "perception_manifest.json").read_text(encoding="utf-8"))
            bindings = build_perception_input_bindings(capture_dir, slam_dir, perception_dir)
            perception_payload["inputs"] = bindings
            perception_payload["capture_sha256"] = bindings["capture"]["capture_sha256"]
            perception_payload["capture_manifest_sha256"] = bindings["capture"]["manifest"]["sha256"]
            perception_payload["slam_manifest_sha256"] = bindings["slam"]["manifest"]["sha256"]
            write_json(perception_dir / "perception_manifest.json", perception_payload)
            references = parent / "references.json"
            references.write_text('{"storyboards": []}\n', encoding="utf-8")
            revision = str(fixture["perception_manifest"]["git_sha"])
            paths = {
                "capture": run / "capture/capture_manifest.json",
                "slam": run / "slam/slam_manifest.json",
                "perception": run / "perception/perception_manifest.json",
                "map": run / "slam/slam_map.ply",
                "trajectory": run / "slam/slam_map_poses.csv",
                "inventory": run / "perception/estimated_inventory.csv",
                "raw_lidar_bag": run / "capture/sensors_bag/capture_0.db3",
                "bag_metadata": run / "capture/bag_metadata.json",
                "camera_info": run / "capture/camera_info.json",
                "sensor_transforms": run / "capture/sensor_transforms.json",
                "effective_config": run / "capture/effective_config.json",
                "scene_manifest": run / "capture/scene_manifest.json",
                "rgb_video": run / "capture/rgb_camera.mp4",
                "rgb_frames": run / "capture/rgb_frames.jsonl",
                "frame_annotations": run / "perception/frame_annotations.jsonl",
            }
            catalog = parent / "modern-catalog.json"
            write_json(catalog, {
                "schema_version": 1, "status": "reviewed_source_catalog", "sources": [{
                    "source_id": "modern-source-v1", "capture_id": "modern-source",
                    "run_directory_name": "modern-source",
                    "capture_sha256": capture_payload["capture_sha256"],
                    "presentation_classification": {"kind": "generated_test_fixture", "marker_id": "simulator.presentation.generated-test-fixture.v1", "marker_sha256": "5ac77eb627c557f5f95ca9d9241960beddaf190084aa375f62d893c88ac6825e"},
                    "presentation_compatibility": {"status": "generated_test_fixture", "delivery_eligible": True},
                    "sensor_contract": {
                        "lidar_topic": "/sim/lidar/points", "camera_info_topic": "/sim/camera/rgb/camera_info",
                        "lidar_frame_id": "lidar_link", "camera_frame_id": "camera_optical_frame",
                        "configured_lidar_hz": 10.0, "maximum_current_age_periods": 2.0,
                        "maximum_rgb_skew_ns": 17000001,
                    },
                    "producer_revisions": {"capture": revision, "slam": revision, "perception": revision},
                    "producer_manifests": {
                        name: {"path": path.relative_to(run).as_posix(), "sha256": sha256_file(path)}
                        for name, path in (("capture", paths["capture"]), ("slam", paths["slam"]), ("perception", paths["perception"]))
                    },
                    "artifacts": {
                        name: {"path": paths[name].relative_to(run).as_posix(), "sha256": sha256_file(paths[name]), "version": f"modern-{name}-v1"}
                        for name in (
                            "map", "trajectory", "inventory", "raw_lidar_bag", "bag_metadata", "camera_info",
                            "sensor_transforms", "effective_config", "scene_manifest", "rgb_video", "rgb_frames",
                            "frame_annotations",
                        )
                    },
                    "simulation_time": {"source": "slam/slam_map_poses.csv:timestamp_s", "start_s": 0.0, "end_s": 20.4},
                    "perception_contract": {
                        "allowed_depth_sources": ["lidar_projected_with_slam_pose"],
                        "estimated_inventory": "estimated_inventory.csv",
                        "slam_trajectory": "../slam/slam_map_poses.csv",
                        "legacy_capture_id_omitted": False,
                        "ground_truth_consumed": False,
                    },
                }],
            })
            result = json.loads(_run_technical(INSPECT_SCRIPT, run, references, catalog).stdout)
            self.assertEqual(result["capture_id"], "modern-source")

            perception_path = paths["perception"]
            payload = json.loads(perception_path.read_text(encoding="utf-8"))
            payload["inputs"]["slam"]["trajectory"]["sha256"] = "f" * 64
            write_json(perception_path, payload)
            catalog_payload = json.loads(catalog.read_text(encoding="utf-8"))
            catalog_payload["sources"][0]["producer_manifests"]["perception"]["sha256"] = sha256_file(perception_path)
            write_json(catalog, catalog_payload)
            result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
            self.assertIn("input bindings do not match", result.stderr)

    def test_source_binding_rejects_cross_run_gt_unknown_and_capture_mismatch(self):
        attacks = {
            "cross-run map": lambda run: (run / "slam/slam_map.ply").write_text("different map", encoding="utf-8"),
            "cross-run trajectory": lambda run: (run / "slam/slam_poses.csv").write_text(
                "timestamp_s,x_m,y_m,z_m\n0.2,0,0,0\n19.0,1,0,0\n", encoding="utf-8"
            ),
            "cross-run raw lidar": lambda run: (run / "capture/sensors_bag/sensors_bag_0.db3").write_bytes(b"other scan"),
            "cross-run scene": lambda run: (run / "capture/scene_manifest.json").write_text('{"scene":"other"}', encoding="utf-8"),
        }
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            for index, (name, mutate) in enumerate(attacks.items()):
                with self.subTest(name=name):
                    run, catalog, references = _write_source_fixture(parent / f"case-{index}")
                    mutate(run)
                    result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("reviewed source catalog", result.stderr)

            for index, depth_source in enumerate(("ground_truth", "unknown"), start=10):
                with self.subTest(depth_source=depth_source):
                    run, catalog, references = _write_source_fixture(parent / f"case-{index}")
                    inventory = run / "perception/estimated_inventory.csv"
                    inventory.write_text(
                        "track_id,estimated_x_m,estimated_y_m,estimated_z_m,depth_source,3d_observation_count\n"
                        f"1,1.0,0.5,0.8,{depth_source},5\n", encoding="utf-8",
                    )
                    _refresh_catalog_hash(catalog, "inventory", inventory)
                    result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
                    self.assertIn("unapproved depth source", result.stderr)

            run, catalog, references = _write_source_fixture(parent / "capture-mismatch")
            perception = run / "perception/perception_manifest.json"
            payload = json.loads(perception.read_text(encoding="utf-8"))
            payload["capture_id"] = "different-capture"
            perception.write_text(json.dumps(payload), encoding="utf-8")
            _refresh_catalog_hash(catalog, "perception", perception)
            result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
            self.assertIn("capture_id must agree", result.stderr)

    def test_source_binding_rejects_gt_manifest_and_unreviewed_producer_revision(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            run, catalog, references = _write_source_fixture(parent / "gt")
            perception = run / "perception/perception_manifest.json"
            payload = json.loads(perception.read_text(encoding="utf-8"))
            payload["ground_truth_consumed"] = True
            perception.write_text(json.dumps(payload), encoding="utf-8")
            _refresh_catalog_hash(catalog, "perception", perception)
            result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
            self.assertIn("ground-truth-free", result.stderr)

            run, catalog, references = _write_source_fixture(parent / "revision")
            perception = run / "perception/perception_manifest.json"
            payload = json.loads(perception.read_text(encoding="utf-8"))
            payload["git_sha"] = "f" * 40
            perception.write_text(json.dumps(payload), encoding="utf-8")
            catalog_payload = json.loads(catalog.read_text(encoding="utf-8"))
            catalog_payload["sources"][0]["producer_revisions"]["perception"] = "f" * 40
            catalog_payload["sources"][0]["producer_manifests"]["perception"]["sha256"] = _sha256(perception)
            catalog.write_text(json.dumps(catalog_payload), encoding="utf-8")
            result = _run_technical(INSPECT_SCRIPT, run, references, catalog, expect_success=False)
            self.assertIn("not a repository commit", result.stderr)

    def test_test_module_has_no_module_scope_numpy_or_renderer_import(self):
        tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
        imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
        imported_modules = {
            alias.name for node in imports if isinstance(node, ast.Import) for alias in node.names
        } | {
            node.module or "" for node in imports if isinstance(node, ast.ImportFrom)
        }
        self.assertNotIn("numpy", imported_modules)
        self.assertNotIn("simulator.technical_views", imported_modules)


if __name__ == "__main__":
    unittest.main()
