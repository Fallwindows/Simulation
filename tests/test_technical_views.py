import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from simulator.capture.manifest import sha256_file, write_json
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
    revision = subprocess.run(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"], check=True, capture_output=True, text=True,
    ).stdout.strip()
    capture_sha = "a" * 64
    capture = {
        "status": "complete", "capture_id": capture_id, "capture_sha256": capture_sha, "git_sha": revision,
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
    }
    catalog = {
        "schema_version": 1,
        "status": "reviewed_source_catalog",
        "sources": [{
            "source_id": "fixture-v1", "capture_id": capture_id, "run_directory_name": capture_id,
            "capture_sha256": capture_sha,
            "producer_revisions": {"capture": revision, "slam": revision, "perception": revision},
            "producer_manifests": {
                name: {"path": path.relative_to(run).as_posix(), "sha256": _sha256(path)}
                for name, path in (("capture", paths["capture"]), ("slam", paths["slam"]), ("perception", paths["perception"]))
            },
            "artifacts": {
                name: {"path": paths[name].relative_to(run).as_posix(), "sha256": _sha256(paths[name]), "version": f"fixture-{name}-v1"}
                for name in ("map", "trajectory", "inventory")
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
        self.assertEqual({item["selective_current_scan_status"] for item in payload["views"]}, {"unfinished"})

    def test_numpy_dependent_loaders_and_renderer_run_in_subprocess(self):
        script = """
import json, tempfile
from pathlib import Path
import numpy as np
from simulator.technical_views import InventoryRecord, TechnicalRenderer, load_ascii_ply, load_plan, project_points
profiles, views = load_plan('config/technical_views.json')
with tempfile.TemporaryDirectory() as temporary:
    path = Path(temporary) / 'map.ply'
    path.write_text('ply\\nformat ascii 1.0\\nelement vertex 4\\nproperty float x\\nproperty float y\\nproperty float z\\nend_header\\n2 -1 0\\n2 1 1\\n4 0 2\\n7 -1 1\\n', encoding='utf-8')
    points = load_ascii_ply(path)
trajectory = np.asarray([[0,0,0],[1,0,0],[2,0,0]], dtype=np.float32)
records = (InventoryRecord(7, (4.0,0.5,1.0), 8),)
pixels, depths, mask = project_points(points, np.asarray([0,0,1], dtype=np.float32), np.asarray([4,0,1], dtype=np.float32), 1280, 720)
renderer = TechnicalRenderer(profiles['preview'], points, trajectory, records)
shapes = [list(renderer.frame(view, 0).shape) for view in views]
print(json.dumps({'shapes': shapes, 'pixel_count': len(pixels), 'depth_count': len(depths), 'mask_bool': str(mask.dtype) == 'bool'}))
"""
        result = json.loads(_run_technical(script).stdout)
        self.assertEqual(result["shapes"], [[720, 1280, 3]] * 7)
        self.assertEqual(result["pixel_count"], result["depth_count"])
        self.assertTrue(result["mask_bool"])

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
            }
            catalog = parent / "modern-catalog.json"
            write_json(catalog, {
                "schema_version": 1, "status": "reviewed_source_catalog", "sources": [{
                    "source_id": "modern-source-v1", "capture_id": "modern-source",
                    "run_directory_name": "modern-source",
                    "capture_sha256": fixture["capture_manifest"]["capture_sha256"],
                    "producer_revisions": {"capture": revision, "slam": revision, "perception": revision},
                    "producer_manifests": {
                        name: {"path": path.relative_to(run).as_posix(), "sha256": sha256_file(path)}
                        for name, path in (("capture", paths["capture"]), ("slam", paths["slam"]), ("perception", paths["perception"]))
                    },
                    "artifacts": {
                        name: {"path": paths[name].relative_to(run).as_posix(), "sha256": sha256_file(paths[name]), "version": f"modern-{name}-v1"}
                        for name in ("map", "trajectory", "inventory")
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
