import unittest
from pathlib import Path
from types import SimpleNamespace

from simulator.environment.isaac_builder import STRUCTURAL_MATERIALS
from simulator.runtime.isaac_sim_runner import lidar_runtime_spec, store_shell_spec
from simulator.runtime.representative_capture import (
    MAX_CAPTURE_FRAMES,
    parse_capture_frames,
    png_dimensions,
    validate_capture_dimensions,
)


class SceneLookdevTests(unittest.TestCase):
    def setUp(self):
        self.environment = SimpleNamespace(
            length_m=24.0,
            width_m=6.0,
            shelf_height_m=2.2,
            lighting_lux=450.0,
        )

    def test_store_shell_is_bounded_and_every_light_has_visible_fixture(self):
        spec = store_shell_spec(self.environment)
        boxes = spec["boxes"]
        lights = spec["lights"]
        kinds = [box["kind"] for box in boxes]
        self.assertIn("ceiling", kinds)
        self.assertEqual(kinds.count("wall"), 3)
        self.assertGreaterEqual(kinds.count("ceiling_grid"), 30)
        panel_names = {box["name"] for box in boxes if box["kind"] == "light_panel"}
        self.assertTrue({light["name"] for light in lights} <= panel_names)
        self.assertGreaterEqual(len(lights), 18)
        self.assertTrue(all(light["position_m"][2] > self.environment.shelf_height_m for light in lights))
        self.assertTrue(all(abs(light["rotation_rpy_deg"][0]) == 32.0 for light in lights))
        self.assertGreater(spec["ambient_intensity"], 0.9 * self.environment.lighting_lux)
        self.assertIn("floor_inlay", kinds)
        self.assertIn("sign_green", kinds)

    def test_structural_materials_distinguish_floor_metal_and_paper(self):
        floor = STRUCTURAL_MATERIALS["floor"]
        shelf = STRUCTURAL_MATERIALS["shelf"]
        tag = STRUCTURAL_MATERIALS["price_tag"]
        self.assertNotEqual(floor["roughness"], shelf["roughness"])
        self.assertLess(floor["metallic"], shelf["metallic"])
        self.assertGreater(tag["roughness"], shelf["roughness"])
        self.assertEqual(tag["metallic"], 0.0)

    def test_capture_frame_selection_is_sorted_unique_and_bounded(self):
        self.assertEqual(parse_capture_frames("12, 0, 6", 13), (0, 6, 12))
        with self.assertRaisesRegex(ValueError, "duplicates"):
            parse_capture_frames("1,1", 3)
        with self.assertRaisesRegex(ValueError, "within"):
            parse_capture_frames("3", 3)
        with self.assertRaisesRegex(ValueError, "at most"):
            parse_capture_frames(",".join(str(index) for index in range(MAX_CAPTURE_FRAMES + 1)), 100)

    def test_capture_bounds_and_png_header_validation(self):
        validate_capture_dimensions(1280, 720, 4)
        with self.assertRaisesRegex(ValueError, "width"):
            validate_capture_dimensions(32, 720, 4)
        reference = Path(__file__).resolve().parents[4] / "references/storyboard/01_enter_aisle_rgb.png"
        self.assertEqual(png_dimensions(reference), (1672, 941))

    def test_local_lidar_schema_preserves_scenario_calibration(self):
        lidar = SimpleNamespace(
            hz=10.0,
            min_range_m=0.2,
            max_range_m=60.0,
            vertical_fov_deg=(-25.0, 15.0),
            horizontal_samples=720,
            vertical_samples=32,
            pose_in_rig=SimpleNamespace(position_m=(0.0, 0.0, 1.55), rpy_deg=(0.0, 0.0, 0.0)),
        )
        spec = lidar_runtime_spec(lidar)
        self.assertEqual(spec["scan_type"], "ROTARY")
        self.assertEqual(spec["horizontal_samples"], 720)
        self.assertEqual(spec["vertical_samples"], 32)
        self.assertEqual(spec["vertical_fov_deg"], [-25.0, 15.0])
        self.assertEqual(spec["elevation_angles_deg"][0], -25.0)
        self.assertEqual(spec["elevation_angles_deg"][-1], 15.0)
        self.assertEqual(spec["pattern_firing_rate_hz"], 7200)
        self.assertEqual(spec["expected_points_per_scan"], 23040)


if __name__ == "__main__":
    unittest.main()
