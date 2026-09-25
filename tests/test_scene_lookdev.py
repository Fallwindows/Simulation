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
        fixture_names = {
            box["name"] for box in boxes if box["kind"] in {"light_panel", "shelf_light"}
        }
        self.assertTrue({light["name"] for light in lights} <= fixture_names)
        self.assertGreaterEqual(len(lights), 18)
        panel_lights = [light for light in lights if light["name"].startswith("panel_")]
        shelf_lights = [light for light in lights if light["name"].startswith("shelf_strip_")]
        self.assertTrue(all(light["position_m"][2] > self.environment.shelf_height_m for light in panel_lights))
        self.assertTrue(all(abs(light["rotation_rpy_deg"][0]) == 32.0 for light in panel_lights))
        self.assertEqual(len(shelf_lights), 16)
        self.assertTrue(all(light["position_m"][2] < self.environment.shelf_height_m for light in shelf_lights))
        self.assertGreater(spec["ambient_intensity"], 0.9 * self.environment.lighting_lux)
        self.assertIn("floor_inlay", kinds)

    def test_real_catalog_stock_replaces_procedural_end_wall_and_adds_hero_display(self):
        spec = store_shell_spec(self.environment)
        boxes = spec["boxes"]
        references = spec["asset_references"]
        kinds = {box["kind"] for box in boxes}
        names = {reference["name"] for reference in references}
        keys = {reference["asset_key"] for reference in references}
        self.assertNotIn("end_product_red", kinds)
        self.assertNotIn("end_product_yellow", kinds)
        self.assertNotIn("end_product_blue", kinds)
        self.assertIn("case_glass", kinds)
        self.assertIn("display_wood", kinds)
        self.assertEqual(len([name for name in names if name.startswith("end_case_stock_")]), 48)
        self.assertEqual(len([name for name in names if name.startswith("hero_stock_")]), 12)
        self.assertIn("promo_market_sign", keys)
        self.assertIn("price_display", keys)
        self.assertIn("shelf_divider", keys)
        self.assertTrue({"milk_gallon", "juice_citrus", "frozen_pizza"} <= keys)
        hero_references = [reference for reference in references if reference["name"].startswith("hero_")]
        self.assertTrue(all(reference["position_xy_m"][1] <= -0.995 for reference in hero_references))
        signs = [reference for reference in references if reference["asset_key"] == "promo_market_sign"]
        self.assertEqual(len(signs), 2)
        self.assertTrue(all(reference["scale_xyz"][0] < 0.0 for reference in signs))

    def test_structural_variation_is_bounded_and_shelves_are_not_near_black(self):
        self.assertIn("floor_tile_warm", STRUCTURAL_MATERIALS)
        self.assertIn("floor_tile_cool", STRUCTURAL_MATERIALS)
        self.assertIn("ceiling_tile_warm", STRUCTURAL_MATERIALS)
        self.assertIn("ceiling_tile_cool", STRUCTURAL_MATERIALS)
        self.assertIn("shelf_light", STRUCTURAL_MATERIALS)
        self.assertGreater(min(STRUCTURAL_MATERIALS["shelf"]["color"]), 0.30)
        self.assertLess(STRUCTURAL_MATERIALS["shelf"]["metallic"], 0.15)
        self.assertLess(STRUCTURAL_MATERIALS["case_glass"]["opacity"], 0.25)

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
