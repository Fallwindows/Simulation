import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simulator.technical_views import (
    InventoryRecord,
    TechnicalRenderer,
    _validate_probe,
    inspect_source_bundle,
    load_ascii_ply,
    load_plan,
    project_points,
    select_inventory,
)


ROOT = Path(__file__).resolve().parents[1]


class TechnicalViewTests(unittest.TestCase):
    def test_plan_has_exact_profiles_order_and_frame_contract(self):
        profiles, views = load_plan(ROOT / "config/technical_views.json")
        self.assertEqual((profiles["preview"].width, profiles["preview"].height), (1280, 720))
        self.assertEqual((profiles["delivery"].width, profiles["delivery"].height), (1920, 1080))
        self.assertEqual([view.id for view in views], [
            "sensor_activation", "lidar_environment", "persistent_map", "object_association",
            "object_detail", "observed_aisle_overview", "final_technical_view",
        ])
        self.assertEqual([view.frames for view in views], [120, 90, 90, 120, 120, 120, 150])
        self.assertEqual([view.presentation_role for view in views], [
            "lidar", "lidar", "map", "reconstruction", "reconstruction",
            "reconstruction", "reconstruction",
        ])
        self.assertEqual(sum(view.frames for view in views), 810)
        self.assertTrue(all(profiles[name].fps == 30 for name in profiles))

    def test_ascii_ply_declared_count_and_xyz_are_enforced(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "map.ply"
            path.write_text(
                "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 1 2\n3 4 5\n",
                encoding="utf-8",
            )
            points = load_ascii_ply(path)
            np.testing.assert_array_equal(points, np.asarray([[0, 1, 2], [3, 4, 5]], dtype=np.float32))

    def test_projection_and_renderer_return_profile_dimensions(self):
        profiles, views = load_plan(ROOT / "config/technical_views.json")
        points = np.asarray([[2, -1, 0], [2, 1, 1], [4, 0, 2], [7, -1, 1]], dtype=np.float32)
        trajectory = np.asarray([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=np.float32)
        records = (InventoryRecord(7, (4.0, 0.5, 1.0), 8),)
        pixels, depths, mask = project_points(points, np.asarray([0, 0, 1], dtype=np.float32), np.asarray([4, 0, 1], dtype=np.float32), 1280, 720)
        self.assertEqual(len(pixels), len(depths))
        self.assertEqual(mask.dtype, np.bool_)
        renderer = TechnicalRenderer(profiles["preview"], points, trajectory, records)
        for view in views:
            with self.subTest(view=view.id):
                frame = renderer.frame(view, 0)
                self.assertEqual(frame.shape, (720, 1280, 3))
                self.assertEqual(frame.dtype, np.uint8)
                self.assertGreater(int(frame.max()), 0)

    def test_encoded_probe_contract_fails_closed(self):
        profiles, _ = load_plan(ROOT / "config/technical_views.json")
        valid = {"width": 1920, "height": 1080, "fps": 30.0, "frame_count": 120}
        _validate_probe(valid, profiles["delivery"], 120)
        for field, bad_value in (
            ("width", 1280), ("height", 720), ("fps", 29.97), ("frame_count", 119),
        ):
            invalid = dict(valid)
            invalid[field] = bad_value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "contract mismatch"):
                _validate_probe(invalid, profiles["delivery"], 120)

    def test_inventory_selection_prefers_supported_spatially_distinct_centers(self):
        records = (
            InventoryRecord(1, (1.0, 0.0, 1.0), 3),
            InventoryRecord(2, (1.01, 0.0, 1.0), 9),
            InventoryRecord(3, (3.0, 0.0, 1.0), 5),
        )
        selected = select_inventory(records, limit=2, spacing_m=0.2)
        self.assertEqual([record.track_id for record in selected], [2, 3])

    def test_bundle_rejects_storyboard_path_before_decode(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary) / "storyboard-run"
            (run / "capture").mkdir(parents=True)
            (run / "slam").mkdir()
            (run / "perception").mkdir()
            for relative in (
                "slam/slam_map.ply", "slam/slam_poses.csv", "perception/estimated_inventory.csv",
                "capture/capture_manifest.json", "slam/slam_manifest.json", "perception/perception_manifest.json",
            ):
                (run / relative).write_text("x", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "storyboard paths"):
                inspect_source_bundle(run, ROOT / "references/manifest.json")


if __name__ == "__main__":
    unittest.main()
