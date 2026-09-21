import copy
import json
import tempfile
import unittest
from pathlib import Path

from evaluation.metrics import PoseSample, compute_metrics, interpolate_pose
from simulator.capture.inventory import export_inventory
from simulator.capture.export_metadata import export_metadata
from simulator.capture.manifest import (
    CAPTURE_MANIFEST_VERSION,
    capture_hash,
    build_experiment_hashes,
    finalize_capture_manifest,
    validate_capture_for_slam,
    validate_capture_manifest_hash,
)


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "config/scenarios/baseline_straight.yaml"


def _production_manifest() -> dict[str, object]:
    topics = [
        "/clock", "/sim/camera/rgb/image_raw", "/sim/lidar/points", "/tf", "/tf_static",
    ]
    return {
        "manifest_version": CAPTURE_MANIFEST_VERSION,
        "status": "complete",
        "capture_id": "fixture-capture",
        "scenario": "C:/repo/config/scenarios/walking_baseline.yaml",
        "git_sha": "1" * 40,
        "rmw_implementation": "rmw_zenoh_cpp",
        "ros_domain_id": 0,
        "duration_s": 45.0,
        "bag": {
            "uri": "sensors_bag",
            "storage_id": "sqlite3",
            "topics": topics,
            "topic_types": {
                "/clock": "rosgraph_msgs/msg/Clock",
                "/sim/camera/rgb/image_raw": "sensor_msgs/msg/Image",
                "/sim/lidar/points": "sensor_msgs/msg/PointCloud2",
                "/tf": "tf2_msgs/msg/TFMessage",
                "/tf_static": "tf2_msgs/msg/TFMessage",
            },
            "counts": {topic: index + 1 for index, topic in enumerate(topics)},
            "first_clock_s": 0.066666666,
            "last_clock_s": 45.0,
        },
        "rgb": {
            "video": "rgb_camera.mp4",
            "timestamp_index": "rgb_frames.jsonl",
            "camera_info": "camera_info.json",
            "camera_info_provenance": "configured_intrinsics",
            "metadata": "rgb_video.json",
            "frame_count": 1350,
            "first_stamp_s": 0.1,
            "last_stamp_s": 45.0,
        },
        "ground_truth": {
            "inventory_csv": "inventory_ground_truth.csv",
            "inventory_json": "inventory_ground_truth.json",
            "pose_topic": "/sim/ground_truth/pose",
            "evaluation_only": True,
        },
        "hashes": {
            "geometry_sha256": "2" * 64,
            "inventory_sha256": "3" * 64,
            "trajectory_sha256": "4" * 64,
            "sensor_sha256": "5" * 64,
            "appearance_sha256": "6" * 64,
            "inputs": {"scenario": "config/scenarios/walking_baseline.yaml"},
            "git_sha": "1" * 40,
        },
        "software_versions": "../software_versions.json",
        "files": [{"path": "rgb_camera.mp4", "sha256": "7" * 64, "size_bytes": 12}],
    }


class CaptureArchitectureTests(unittest.TestCase):
    def test_capture_manifest_finalizer_is_bom_free_canonical_and_tamper_evident(self):
        manifest = _production_manifest()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            staging = root / "staging.json"
            output = root / "capture_manifest.json"
            reversed_manifest = dict(reversed(list(manifest.items())))
            staging.write_bytes(b"\xef\xbb\xbf" + json.dumps(reversed_manifest).encode("utf-8"))
            finalized = finalize_capture_manifest(staging, output)

            self.assertFalse(output.read_bytes().startswith(b"\xef\xbb\xbf"))
            strict = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(strict, finalized)
            self.assertEqual(strict["capture_sha256"], capture_hash(manifest))
            self.assertEqual(validate_capture_manifest_hash(strict), strict["capture_sha256"])
            self.assertEqual(capture_hash(reversed_manifest), capture_hash(manifest))

            tampered = dict(strict)
            tampered["capture_id"] = "tampered"
            self.assertNotEqual(capture_hash(tampered), strict["capture_sha256"])
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                validate_capture_manifest_hash(tampered)

    def test_finalizer_rejects_malformed_nested_values_and_preserves_existing_output(self):
        mutations = {
            "empty capture id": lambda value: value.update(capture_id=""),
            "short git sha": lambda value: value.update(git_sha="x"),
            "nonfinite duration": lambda value: value.update(duration_s=float("nan")),
            "bag not mapping": lambda value: value.update(bag=None),
            "wrong topics": lambda value: value["bag"].update(topics=["/clock"]),
            "wrong topic types": lambda value: value["bag"].update(topic_types={"/clock": "wrong"}),
            "negative count": lambda value: value["bag"]["counts"].update({"/clock": -1}),
            "rgb not mapping": lambda value: value.update(rgb=None),
            "wrong camera provenance": lambda value: value["rgb"].update(camera_info_provenance="observed"),
            "ground truth not mapping": lambda value: value.update(ground_truth=None),
            "hashes not mapping": lambda value: value.update(hashes=None),
            "files not list": lambda value: value.update(files=None),
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            staging = root / "staging.json"
            output = root / "capture_manifest.json"
            original = b"existing-output-must-survive"
            for name, mutate in mutations.items():
                with self.subTest(name=name):
                    value = copy.deepcopy(_production_manifest())
                    mutate(value)
                    staging.write_text(json.dumps(value), encoding="utf-8")
                    output.write_bytes(original)
                    with self.assertRaises(ValueError):
                        finalize_capture_manifest(staging, output)
                    self.assertEqual(output.read_bytes(), original)
                    self.assertFalse((root / "capture_manifest.json.tmp").exists())

    def test_finalizer_rejects_unsafe_duplicate_and_self_file_paths(self):
        cases = {
            "traversal": [
                {"path": "../escape", "sha256": "7" * 64, "size_bytes": 1},
            ],
            "absolute": [
                {"path": "C:/escape", "sha256": "7" * 64, "size_bytes": 1},
            ],
            "self": [
                {"path": "capture_manifest.json", "sha256": "7" * 64, "size_bytes": 1},
            ],
            "duplicate": [
                {"path": "rgb_camera.mp4", "sha256": "7" * 64, "size_bytes": 1},
                {"path": "RGB_CAMERA.MP4", "sha256": "8" * 64, "size_bytes": 2},
            ],
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            staging = root / "staging.json"
            output = root / "capture_manifest.json"
            for name, files in cases.items():
                with self.subTest(name=name):
                    value = _production_manifest()
                    value["files"] = files
                    staging.write_text(json.dumps(value), encoding="utf-8")
                    with self.assertRaises(ValueError):
                        finalize_capture_manifest(staging, output)
                    self.assertFalse(output.exists())
                    self.assertFalse((root / "capture_manifest.json.tmp").exists())

    def test_capture_metadata_exports_configured_camera_intrinsics_with_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            export_metadata(SCENARIO, root, ROOT)
            camera_info = json.loads((root / "camera_info.json").read_text(encoding="utf-8"))
            self.assertEqual(camera_info["provenance"], "configured_intrinsics")
            self.assertFalse(camera_info["observed_ros_message"])
            self.assertEqual(camera_info["source"], "sensor_transforms.json")
            self.assertEqual(camera_info["topic"], "/sim/camera/rgb/camera_info")
            self.assertEqual((camera_info["width_px"], camera_info["height_px"]), (1280, 720))

    def test_hashes_are_stable_and_physical_inputs_are_separate(self):
        first = build_experiment_hashes(SCENARIO, ROOT)
        second = build_experiment_hashes(SCENARIO, ROOT)
        self.assertEqual(first, second)
        self.assertEqual(len(first["geometry_sha256"]), 64)
        self.assertEqual(len(first["appearance_sha256"]), 64)

    def test_inventory_export_is_deterministic_and_has_semantic_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            result = export_inventory(SCENARIO, directory)
            self.assertEqual(result["asset_count"], 1995)
            rows = json.loads((Path(directory) / "inventory_ground_truth.json").read_text(encoding="utf-8"))
            self.assertEqual(len(rows), 1995)
            self.assertTrue(all(row["semantic_id"].startswith("retail/") for row in rows))
            self.assertIn("width_m", rows[0])

    def test_slam_validation_does_not_require_ground_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sensors_bag").mkdir()
            (root / "rgb_camera.mp4").write_bytes(b"video")
            (root / "rgb_frames.jsonl").write_text("{\"stamp_s\": 0.0}\n", encoding="utf-8")
            (root / "camera_info.json").write_text(
                json.dumps({"provenance": "configured_intrinsics", "observed_ros_message": False}),
                encoding="utf-8",
            )
            manifest = {
                "status": "complete",
                "manifest_version": CAPTURE_MANIFEST_VERSION,
                "bag": {"uri": "sensors_bag", "topics": [
                    "/clock", "/sim/camera/rgb/image_raw",
                    "/sim/lidar/points", "/tf", "/tf_static",
                ]},
                "rgb": {
                    "camera_info": "camera_info.json",
                    "camera_info_provenance": "configured_intrinsics",
                },
            }
            result = validate_capture_for_slam(root, manifest)
            self.assertTrue(result["gt_required"] is False)
            self.assertNotIn("/sim/camera/rgb/camera_info", result["topics"])

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
