import unittest
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace

from simulator.environment.isaac_builder import STRUCTURAL_MATERIALS
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.retail_catalog import load_retail_catalog
from simulator.config.loader import load_scenario
from simulator.runtime.isaac_sim_runner import (
    _paced_wall_period_s,
    lidar_runtime_spec,
    runtime_dense_stock_references,
    store_shell_spec,
)
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
        self.assertEqual(len(shelf_lights), 64)
        self.assertTrue(all(light["position_m"][2] < self.environment.shelf_height_m for light in shelf_lights))
        self.assertTrue(all(abs(light["position_m"][1]) >= 1.42 for light in shelf_lights))
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
        self.assertEqual(len([name for name in names if name.startswith("end_case_stock_")]), 72)
        self.assertEqual(len([name for name in names if name.startswith("hero_stock_")]), 18)
        self.assertEqual(len([name for name in names if name.startswith("hero_focus_stock_")]), 4)
        self.assertEqual(len([name for name in names if name.startswith("store_use_basket_")]), 2)
        self.assertIn("promo_market_sign", keys)
        self.assertIn("price_display", keys)
        self.assertIn("shelf_divider", keys)
        self.assertTrue({"milk_gallon", "juice_citrus", "icecream_tub"} <= keys)
        hero_references = [reference for reference in references if reference["name"].startswith("hero_")]
        self.assertTrue(all(reference["position_xy_m"][1] <= -0.50 for reference in hero_references))
        signs = [reference for reference in references if reference["asset_key"] == "promo_market_sign"]
        self.assertEqual(len(signs), 3)
        self.assertTrue(all(reference["scale_xyz"][0] < 0.0 for reference in signs))
        hero_stock = [reference for reference in references if reference["name"].startswith("hero_stock_")]
        self.assertGreaterEqual(min(reference["position_xy_m"][0] for reference in hero_stock), 12.7)

    def test_hero_focus_and_store_baskets_have_positive_3d_clearance(self):
        scenario = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml")
        catalog = load_retail_catalog(scenario.environment.asset_manifest_path)
        references = store_shell_spec(self.environment)["asset_references"]

        def projections(reference):
            record = catalog.by_key(reference["asset_key"])
            width = record.dimensions_m[0] * abs(reference["scale_xyz"][0])
            depth = record.dimensions_m[1] * abs(reference["scale_xyz"][1])
            yaw = math.radians(reference["rotation_rpy_deg"][2])
            axis_x = (math.cos(yaw), math.sin(yaw))
            axis_y = (-math.sin(yaw), math.cos(yaw))
            center = reference["position_xy_m"]
            corners = tuple(
                (
                    center[0] + sx * width * axis_x[0] / 2.0 + sy * depth * axis_y[0] / 2.0,
                    center[1] + sx * width * axis_x[1] / 2.0 + sy * depth * axis_y[1] / 2.0,
                )
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
            )
            return record, (axis_x, axis_y), corners

        def overlaps(first, second):
            first_record, first_axes, first_corners = projections(first)
            second_record, second_axes, second_corners = projections(second)
            first_z = (
                first["support_z_m"],
                first["support_z_m"] + first_record.dimensions_m[2] * abs(first["scale_xyz"][2]),
            )
            second_z = (
                second["support_z_m"],
                second["support_z_m"] + second_record.dimensions_m[2] * abs(second["scale_xyz"][2]),
            )
            if min(first_z[1], second_z[1]) - max(first_z[0], second_z[0]) <= 1e-4:
                return False
            for axis in (*first_axes, *second_axes):
                first_interval = [corner[0] * axis[0] + corner[1] * axis[1] for corner in first_corners]
                second_interval = [corner[0] * axis[0] + corner[1] * axis[1] for corner in second_corners]
                if min(max(first_interval), max(second_interval)) - max(min(first_interval), min(second_interval)) <= 1e-4:
                    return False
            return True

        for prefix in ("hero_focus_stock_", "store_use_basket_"):
            group = [reference for reference in references if reference["name"].startswith(prefix)]
            for index, first in enumerate(group):
                for second in group[index + 1 :]:
                    self.assertFalse(overlaps(first, second), f"{first['name']} overlaps {second['name']}")

    def test_runtime_stock_fills_measured_edge_gaps_with_real_catalog_assets(self):
        scenario = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml")
        layout = build_aisle_layout(scenario.environment)
        references = runtime_dense_stock_references(layout, scenario.environment)
        names = [reference["name"] for reference in references]
        self.assertEqual(len(names), len(set(names)))
        self.assertGreaterEqual(len(references), 200)
        self.assertGreaterEqual(sum(reference["asset_key"] == "price_display" for reference in references), 30)
        self.assertTrue(all(reference["support_z_m"] > 0.0 for reference in references))
        self.assertTrue(all(reference["asset_key"] != "promo_market_sign" for reference in references))

    def test_context_and_dense_stock_do_not_penetrate_fixtures_or_existing_tags(self):
        scenario = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/walking_baseline.yaml")
        layout = build_aisle_layout(scenario.environment)
        catalog = load_retail_catalog(scenario.environment.asset_manifest_path)
        spec = store_shell_spec(scenario.environment)

        def obb(name, center_xy, width, depth, yaw_deg, z_min, z_max):
            yaw = math.radians(yaw_deg)
            axis_x = (math.cos(yaw), math.sin(yaw))
            axis_y = (-math.sin(yaw), math.cos(yaw))
            corners = tuple(
                (
                    center_xy[0] + sx * width * axis_x[0] / 2.0 + sy * depth * axis_y[0] / 2.0,
                    center_xy[1] + sx * width * axis_x[1] / 2.0 + sy * depth * axis_y[1] / 2.0,
                )
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
            )
            return name, (axis_x, axis_y), corners, (z_min, z_max)

        def intersects(first, second):
            for axis in (*first[1], *second[1]):
                first_projection = [point[0] * axis[0] + point[1] * axis[1] for point in first[2]]
                second_projection = [point[0] * axis[0] + point[1] * axis[1] for point in second[2]]
                if min(max(first_projection), max(second_projection)) - max(min(first_projection), min(second_projection)) <= 1e-4:
                    return False
            return min(first[3][1], second[3][1]) - max(first[3][0], second[3][0]) > 1e-4

        def reference_obb(reference):
            record = catalog.by_key(reference["asset_key"])
            scale = reference["scale_xyz"]
            support_z = reference["support_z_m"]
            return obb(
                reference["name"],
                reference["position_xy_m"],
                record.dimensions_m[0] * abs(scale[0]),
                record.dimensions_m[1] * abs(scale[1]),
                reference["rotation_rpy_deg"][2],
                support_z,
                support_z + record.dimensions_m[2] * abs(scale[2]),
            )

        box_obbs = []
        for box in spec["boxes"]:
            x, y, z = box["center_m"]
            width, depth, height = box["size_m"]
            box_obbs.append(obb(box["name"], (x, y), width, depth, 0.0, z - height / 2.0, z + height / 2.0))
        for reference in spec["asset_references"]:
            if not reference["name"].startswith(("hero_focus_", "store_use_basket_")):
                continue
            candidate = reference_obb(reference)
            collisions = [box[0] for box in box_obbs if intersects(candidate, box)]
            self.assertEqual(collisions, [], f"{reference['name']} penetrates structural boxes {collisions}")

        dense = runtime_dense_stock_references(layout, scenario.environment)
        shelf_lights = [box for box in box_obbs if box[0].startswith("shelf_strip_")]
        for reference in dense:
            candidate = reference_obb(reference)
            self.assertFalse(
                any(intersects(candidate, light) for light in shelf_lights),
                f"{reference['name']} penetrates a shelf light",
            )

        existing_price_obbs = []
        for asset in layout.assets:
            if asset.asset_key != "price_display":
                continue
            record = catalog.by_key(asset.asset_key)
            existing_price_obbs.append(
                obb(
                    asset.name,
                    asset.position_m[:2],
                    record.dimensions_m[0] * abs(asset.scale_xyz[0]),
                    record.dimensions_m[1] * abs(asset.scale_xyz[1]),
                    asset.rotation_rpy_deg[2],
                    asset.position_m[2],
                    asset.position_m[2] + record.dimensions_m[2] * abs(asset.scale_xyz[2]),
                )
            )
        for reference in dense:
            if reference["asset_key"] == "price_display":
                candidate = reference_obb(reference)
                self.assertFalse(
                    any(intersects(candidate, existing) for existing in existing_price_obbs),
                    f"{reference['name']} duplicates an existing price display",
                )

    def test_realtime_factor_expands_wall_period_without_changing_simulation_dt(self):
        self.assertIsNone(_paced_wall_period_s(False, 0.25))
        self.assertAlmostEqual(_paced_wall_period_s(True, 1.0), 1.0 / 60.0)
        self.assertAlmostEqual(_paced_wall_period_s(True, 0.25), 1.0 / 15.0)
        for invalid in (0.0, -1.0, 1.01, float("inf"), float("nan")):
            with self.assertRaisesRegex(ValueError, "realtime factor"):
                _paced_wall_period_s(True, invalid)

    def test_structural_variation_is_bounded_and_shelves_are_not_near_black(self):
        self.assertIn("floor_tile_warm", STRUCTURAL_MATERIALS)
        self.assertIn("floor_tile_cool", STRUCTURAL_MATERIALS)
        self.assertIn("ceiling_tile_warm", STRUCTURAL_MATERIALS)
        self.assertIn("ceiling_tile_cool", STRUCTURAL_MATERIALS)
        self.assertIn("shelf_light", STRUCTURAL_MATERIALS)
        self.assertGreater(min(STRUCTURAL_MATERIALS["shelf"]["color"]), 0.30)
        self.assertLess(STRUCTURAL_MATERIALS["shelf"]["metallic"], 0.15)
        self.assertLess(STRUCTURAL_MATERIALS["case_glass"]["opacity"], 0.25)
        self.assertGreater(STRUCTURAL_MATERIALS["case_glass"]["specular_level"], 1.0)
        material_root = Path(__file__).resolve().parents[1] / "assets/scene/materials"
        import json

        from tools.generate_scene_materials import (
            GENERATOR_VERSION,
            MATERIAL_SETS,
            SEED,
            SIZE,
            _build_outputs,
        )

        committed_manifest = json.loads(
            (material_root / "manifest.json").read_text(encoding="utf-8")
        )
        _, generated_pngs, generated_manifest, generated_manifest_bytes = _build_outputs()
        self.assertEqual(committed_manifest, generated_manifest)
        self.assertEqual(committed_manifest, json.loads(generated_manifest_bytes.decode("utf-8")))
        self.assertEqual(committed_manifest["generator_version"], GENERATOR_VERSION)
        self.assertEqual(committed_manifest["seed"], SEED)
        self.assertEqual(committed_manifest["material_sets"], MATERIAL_SETS)
        self.assertTrue(committed_manifest["tileable"])
        self.assertEqual(
            committed_manifest["normal_convention"],
            "OpenGL tangent space (+X right, +Y up, +Z outward)",
        )
        self.assertEqual(set(committed_manifest["files"]), set(generated_pngs))

        for filename, encoded in generated_pngs.items():
            committed = (material_root / filename).read_bytes()
            metadata = committed_manifest["files"][filename]
            self.assertEqual(committed, encoded, filename)
            self.assertEqual(hashlib.sha256(committed).hexdigest(), metadata["sha256"], filename)
            if filename != "material_preview.png":
                self.assertEqual((metadata["width_px"], metadata["height_px"]), (SIZE, SIZE))
            if filename.endswith("_normal.png"):
                self.assertEqual(metadata["mode"], "RGB")
                self.assertGreaterEqual(metadata["channel_min"][2], 245, filename)
            if filename.endswith("_roughness.png"):
                self.assertEqual(metadata["mode"], "L")
                self.assertGreaterEqual(metadata["channel_min"], 8, filename)
                self.assertLessEqual(metadata["channel_max"], 240, filename)
                self.assertGreaterEqual(
                    metadata["channel_max"] - metadata["channel_min"], 6, filename
                )

        glass_roughness = committed_manifest["files"]["case_glass_roughness.png"]
        self.assertLessEqual(glass_roughness["channel_min"], 24)
        self.assertLessEqual(glass_roughness["channel_max"], 64)
        floor_wear = committed_manifest["files"]["floor_wear_mask.png"]
        self.assertLessEqual(floor_wear["channel_min"], 64)
        self.assertGreaterEqual(floor_wear["channel_max"], 192)
        shelf_albedo = committed_manifest["files"]["powdercoat_albedo.png"]
        self.assertGreater(min(shelf_albedo["channel_min"]), 90)
        self.assertLess(max(shelf_albedo["channel_max"]), 160)

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
