import re
import unittest
from collections import Counter
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import (
    SHELF_THICKNESS_M,
    build_aisle_layout,
    shelf_level_counts_by_zone,
)
from simulator.environment.retail_catalog import load_retail_catalog


class RetailAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.scenario = load_scenario(root / "config/scenarios/baseline_straight.yaml")
        cls.config = cls.scenario.environment
        cls.layout = build_aisle_layout(cls.config)
        cls.catalog = load_retail_catalog(cls.config.asset_manifest_path)
        cls.catalog_by_key = {asset.asset_key: asset for asset in cls.catalog.assets}
        cls.bay_count = int(cls.config.length_m // cls.config.bay_width_m)
        cls.shelves = {}
        for primitive in cls.layout.primitives:
            match = re.match(r"shelf_r(\d+)_b(\d+)_l(\d+)$", primitive.name)
            if match:
                row, bay, level = (int(value) for value in match.groups())
                cls.shelves[(row, bay, level)] = primitive

    def test_catalog_is_complete_and_portable(self):
        self.assertEqual(len(self.catalog.assets), 34)
        for asset in self.catalog.assets:
            self.assertTrue(asset.usd_path.is_file(), asset.asset_key)
            self.assertTrue(asset.texture_path.is_file(), asset.asset_key)
            self.assertEqual(asset.local_front_axis, "+Y")

    def test_dimension_aware_shelf_levels_are_dense_but_clear(self):
        counts = shelf_level_counts_by_zone(self.config, self.catalog)
        self.assertEqual(counts, {"cereal": 5, "snacks": 5, "cans_jars": 7, "beverage": 6, "produce": 6})
        self.assertEqual(len(self.shelves), 231)
        for row in range(2):
            for bay in range(self.bay_count):
                levels = [key for key in self.shelves if key[:2] == (row, bay)]
                z_values = [self.shelves[key].center_m[2] for key in sorted(levels)]
                self.assertEqual(z_values, sorted(z_values))
                self.assertEqual(len(set(z_values)), len(z_values))

    def test_dense_count_and_real_depth_facings(self):
        self.assertGreaterEqual(len(self.layout.assets), 1800)
        self.assertLessEqual(len(self.layout.assets), 2300)
        self.assertGreater(len(self.layout.assets), 4 * 433)
        self.assertGreaterEqual(len({asset.asset_key for asset in self.layout.assets}), 30)
        regular = [asset for asset in self.layout.assets if asset.category != "produce_crate"]
        self.assertTrue(any("/d1/" in asset.semantic_id for asset in regular))
        self.assertEqual(Counter(asset.category for asset in self.layout.assets), Counter({
            "cereal": 440,
            "snacks": 341,
            "cans": 320,
            "juice": 267,
            "soda": 221,
            "jars": 215,
            "water": 144,
            "red_apples": 24,
            "green_apples": 24,
            "oranges": 24,
            "lemons": 24,
            "produce_crate": 4,
        }))

    def test_every_physical_object_has_one_unique_identity(self):
        ids = [asset.instance_id for asset in self.layout.assets]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertTrue(all(ids))
        self.assertEqual([asset.instance_id for asset in self.layout.assets], [asset.semantic_id for asset in self.layout.assets])
        self.assertEqual(self.layout, build_aisle_layout(self.config))

    def test_all_assets_resolve_and_products_are_shelf_anchored(self):
        for asset in self.layout.assets:
            self.assertIn(asset.asset_key, self.catalog_by_key)
            self.assertEqual(asset.scale_xyz, (1.0, 1.0, 1.0))
            self.assertGreater(abs(asset.position_m[1]), self.config.shelf_depth_m / 2.0)
        for asset in self.layout.assets:
            if asset.category == "produce_crate":
                continue
            product_match = re.match(r"product_r(\d+)_b(\d+)_l(\d+)_d\d+_f\d+$", asset.name)
            if product_match is None:
                continue
            row, bay, level = (int(value) for value in product_match.groups())
            shelf = self.shelves[(row, bay, level)]
            record = self.catalog_by_key[asset.asset_key]
            expected_z = shelf.center_m[2] + SHELF_THICKNESS_M / 2.0 + record.dimensions_m[2] / 2.0
            self.assertAlmostEqual(asset.position_m[2], expected_z, places=12)
            self.assertLessEqual(abs(asset.position_m[1] - shelf.center_m[1]) + record.dimensions_m[1] / 2.0, self.config.shelf_depth_m / 2.0 + 1e-9)
            next_shelf = self.shelves.get((row, bay, level + 1))
            if next_shelf is not None:
                product_top = asset.position_m[2] + record.dimensions_m[2] / 2.0
                next_shelf_bottom = next_shelf.center_m[2] - SHELF_THICKNESS_M / 2.0
                self.assertGreaterEqual(next_shelf_bottom - product_top, self.config.shelf_clearance_m - 1e-9)

    def test_produce_is_substantial_and_inside_four_bins(self):
        crates = [asset for asset in self.layout.assets if asset.category == "produce_crate"]
        fruits = [asset for asset in self.layout.assets if asset.category in {"red_apples", "green_apples", "oranges", "lemons"}]
        self.assertEqual(len(crates), 4)
        self.assertEqual(len(fruits), 96)
        self.assertEqual(Counter(asset.category for asset in fruits), Counter({
            "red_apples": 24,
            "green_apples": 24,
            "oranges": 24,
            "lemons": 24,
        }))
        crate_by_prefix = {"/".join(asset.semantic_id.split("/")[2:]): asset for asset in crates}
        for fruit in fruits:
            prefix = "/".join(fruit.semantic_id.split("/")[2:-1])
            crate = crate_by_prefix[prefix]
            fruit_record = self.catalog_by_key[fruit.asset_key]
            crate_record = self.catalog_by_key[crate.asset_key]
            self.assertLessEqual(abs(fruit.position_m[0] - crate.position_m[0]) + fruit_record.dimensions_m[0] / 2.0, crate_record.dimensions_m[0] / 2.0 + 0.01)
            self.assertLessEqual(abs(fruit.position_m[1] - crate.position_m[1]) + fruit_record.dimensions_m[1] / 2.0, crate_record.dimensions_m[1] / 2.0 + 0.01)

    def test_products_face_the_aisle_without_corridor_intrusion(self):
        for asset in self.layout.assets:
            if asset.category == "produce_crate":
                continue
            yaw = asset.rotation_rpy_deg[2] % 360.0
            if asset.position_m[1] < 0.0:
                self.assertLess(min(yaw, 360.0 - yaw), 5.0)
            else:
                self.assertLess(abs(yaw - 180.0), 5.0)
            self.assertGreater(abs(asset.position_m[1]), 1.0)


if __name__ == "__main__":
    unittest.main()
