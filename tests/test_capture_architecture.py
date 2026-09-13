import json
import tempfile
import unittest
from pathlib import Path

from evaluation.metrics import PoseSample, compute_metrics, interpolate_pose
from simulator.capture.inventory import export_inventory
from simulator.capture.manifest import build_experiment_hashes, validate_capture_for_slam


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "config/scenarios/baseline_straight.yaml"


class CaptureArchitectureTests(unittest.TestCase):
    def test_hashes_are_stable_and_physical_inputs_are_separate(self):
        first = build_experiment_hashes(SCENARIO, ROOT)
        second = build_experiment_hashes(SCENARIO, ROOT)
        self.assertEqual(first, second)
        self.assertEqual(len(first["geometry_sha256"]), 64)
        self.assertEqual(len(first["appearance_sha256"]), 64)

    def test_inventory_export_is_deterministic_and_has_semantic_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            result = export_inventory(SCENARIO, directory)
            self.assertEqual(result["asset_count"], 2048)
            rows = json.loads((Path(directory) / "inventory_ground_truth.json").read_text(encoding="utf-8"))
            self.assertEqual(len(rows), 2048)
            self.assertTrue(all(row["semantic_id"].startswith("retail/") for row in rows))
            self.assertIn("width_m", rows[0])

    def test_slam_validation_does_not_require_ground_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sensors_bag").mkdir()
            (root / "rgb_camera.mp4").write_bytes(b"video")
            (root / "rgb_frames.jsonl").write_text("{\"stamp_s\": 0.0}\n", encoding="utf-8")
            manifest = {
                "status": "complete",
                "manifest_version": 1,
                "bag": {"uri": "sensors_bag", "topics": [
                    "/clock", "/sim/camera/rgb/image_raw", "/sim/camera/rgb/camera_info",
                    "/sim/lidar/points", "/tf", "/tf_static",
                ]},
            }
            self.assertTrue(validate_capture_for_slam(root, manifest)["gt_required"] is False)

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
