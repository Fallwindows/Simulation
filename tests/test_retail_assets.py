import random
import re
import unittest
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.retail_catalog import load_retail_catalog


class RetailAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.scenario = load_scenario(root / "config/scenarios/baseline_straight.yaml")
        cls.layout = build_aisle_layout(cls.scenario.environment)
        cls.catalog = load_retail_catalog(cls.scenario.environment.asset_manifest_path)
        cls.catalog_by_key = {asset.asset_key: asset for asset in cls.catalog.assets}

    def test_catalog_is_complete_and_portable(self):
        self.assertEqual(len(self.catalog.assets), 34)
        for asset in self.catalog.assets:
            self.assertTrue(asset.usd_path.is_file(), asset.asset_key)
            self.assertTrue(asset.texture_path.is_file(), asset.asset_key)
            self.assertEqual(asset.local_front_axis, "+Y")

    def test_structure_matches_original_open_aisle(self):
        kinds = [primitive.kind for primitive in self.layout.primitives]
        self.assertEqual(len(self.layout.primitives), 241)
        self.assertEqual(kinds.count("floor"), 1)
        self.assertEqual(kinds.count("shelf"), 160)
        self.assertEqual(kinds.count("upright"), 80)
        self.assertEqual(set(kinds), {"floor", "shelf", "upright"})
        forbidden = ("wall", "ceiling", "baseboard", "rear_panel", "shelf_lip", "price_strip", "endcap")
        self.assertFalse(any(kind in forbidden for kind in kinds))
        self.assertEqual(self.layout.primitives[0].center_m, (12.0, 0.0, -0.05))
        self.assertEqual(self.layout.primitives[1].center_m, (0.6, -1.65, 0.55))
        self.assertAlmostEqual(self.layout.primitives[-1].center_m[0], 23.4, places=12)
        self.assertEqual(self.layout.primitives[-1].center_m[1:], (1.881, 1.1))

    def _legacy_product_slots(self):
        config = self.scenario.environment
        rng = random.Random(config.seed)
        bay_count = max(1, int(config.length_m // config.bay_width_m))
        x0 = config.bay_width_m / 2.0
        level_spacing = config.shelf_height_m / config.shelf_levels
        slots = []
        for row_index, row_y in enumerate(config.shelf_rows_y_m):
            for bay in range(bay_count):
                x = x0 + bay * config.bay_width_m
                for level in range(config.shelf_levels):
                    z = config.floor_z_m + (level + 1) * level_spacing
                    for slot in range(4):
                        if rng.random() > config.product_density:
                            continue
                        px = x - config.bay_width_m * 0.36 + (slot + 0.5) * config.bay_width_m * 0.18
                        py = row_y + (rng.random() - 0.5) * max(0.02, config.shelf_depth_m * 0.35)
                        rng.random()
                        slots.append((f"product_r{row_index}_b{bay}_l{level}_s{slot}", px, py, z))
        return slots

    def test_asset_slots_preserve_original_rng_decisions_and_positions(self):
        expected = self._legacy_product_slots()
        actual = [(asset.name, asset.position_m[0], asset.position_m[1]) for asset in self.layout.assets]
        self.assertEqual(len(actual), len(expected))
        for (name, x, y), (expected_name, expected_x, expected_y, _) in zip(actual, expected):
            self.assertEqual(name, expected_name)
            self.assertAlmostEqual(x, expected_x, places=12)
            self.assertAlmostEqual(y, expected_y, places=12)

    def test_same_seed_is_deterministic_and_every_asset_resolves(self):
        self.assertEqual(self.layout, build_aisle_layout(self.scenario.environment))
        self.assertGreater(len(self.layout.assets), 0)
        self.assertTrue(all(asset.asset_key in self.catalog_by_key for asset in self.layout.assets))
        self.assertTrue(all(asset.scale_xyz == (1.0, 1.0, 1.0) for asset in self.layout.assets))

    def test_assets_sit_on_shelves_and_do_not_enter_corridor(self):
        config = self.scenario.environment
        level_spacing = config.shelf_height_m / config.shelf_levels
        for asset in self.layout.assets:
            match = re.match(r"product_r(\d+)_b(\d+)_l(\d+)_s\d+$", asset.name)
            self.assertIsNotNone(match)
            row, bay, level = (int(value) for value in match.groups())
            record = self.catalog_by_key[asset.asset_key]
            shelf_center_z = config.floor_z_m + (level + 1) * level_spacing
            expected_z = shelf_center_z + 0.03 + record.dimensions_m[2] / 2.0
            self.assertAlmostEqual(asset.position_m[2], expected_z, places=12)
            self.assertGreater(abs(asset.position_m[1]), config.shelf_depth_m / 2.0)
            self.assertEqual(row, 0 if asset.position_m[1] < 0.0 else 1)
            self.assertLess(abs(asset.position_m[0] - (config.bay_width_m / 2.0 + bay * config.bay_width_m)), config.bay_width_m / 2.0)

    def test_assets_face_the_aisle_and_fruit_is_in_final_bays(self):
        config = self.scenario.environment
        bay_count = int(config.length_m // config.bay_width_m)
        fruit_categories = {"red_apples", "green_apples", "oranges", "lemons"}
        fruit_assets = []
        for asset in self.layout.assets:
            yaw = asset.rotation_rpy_deg[2] % 360.0
            if asset.position_m[1] < 0.0:
                self.assertLess(min(yaw, 360.0 - yaw), 5.0)
            else:
                self.assertLess(abs(yaw - 180.0), 5.0)
            if asset.category in fruit_categories:
                fruit_assets.append(asset)
                match = re.match(r"product_r\d+_b(\d+)_l(\d+)_s\d+$", asset.name)
                self.assertIsNotNone(match)
                bay, level = (int(value) for value in match.groups())
                self.assertGreaterEqual(bay, bay_count - 2)
                self.assertEqual(level, 0)
        self.assertGreater(len(fruit_assets), 0)

    def test_categories_are_zoned_without_changing_slots(self):
        categories = {asset.category for asset in self.layout.assets}
        self.assertTrue({"cereal", "snacks", "cans", "jars", "juice", "water", "soda"} <= categories)
        self.assertTrue({"red_apples", "green_apples", "oranges", "lemons"} <= categories)
        self.assertFalse(any(asset.name.startswith("endcap_") for asset in self.layout.assets))


if __name__ == "__main__":
    unittest.main()
