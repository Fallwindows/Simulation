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

    def test_catalog_is_complete_and_portable(self):
        self.assertEqual(len(self.catalog.assets), 34)
        for asset in self.catalog.assets:
            self.assertTrue(asset.usd_path.is_file(), asset.asset_key)
            self.assertTrue(asset.texture_path.is_file(), asset.asset_key)
            self.assertEqual(asset.local_front_axis, "+Y")

    def test_layout_is_dense_deterministic_and_referenced(self):
        other = build_aisle_layout(self.scenario.environment)
        self.assertEqual(self.layout, other)
        self.assertGreaterEqual(len(self.layout.assets), 1000)
        catalog_keys = {asset.asset_key for asset in self.catalog.assets}
        self.assertTrue({asset.asset_key for asset in self.layout.assets} <= catalog_keys)

    def test_each_shelf_bay_and_level_has_configured_facings(self):
        counts = {}
        for asset in self.layout.assets:
            match = re.match(r"product_r(\d+)_b(\d+)_l(\d+)_s\d+$", asset.name)
            if match:
                key = tuple(int(value) for value in match.groups())
                counts[key] = counts.get(key, 0) + 1
                self.assertGreater(abs(asset.position_m[1]), 1.25)
        environment = self.scenario.environment
        expected = len(environment.shelf_rows_y_m) * int(environment.length_m // environment.bay_width_m) * environment.shelf_levels
        self.assertEqual(len(counts), expected)
        self.assertTrue(all(environment.min_facings_per_bay <= count <= environment.max_facings_per_bay for count in counts.values()))

    def test_assets_face_the_aisle_and_stay_above_shelves(self):
        for asset in self.layout.assets:
            if not asset.name.startswith("product_"):
                continue
            yaw = asset.rotation_rpy_deg[2] % 360.0
            if asset.position_m[1] < 0.0:
                self.assertLess(min(yaw, 360.0 - yaw), 5.0)
            else:
                self.assertLess(abs(yaw - 180.0), 5.0)
            self.assertGreater(asset.position_m[2], 0.10)

    def test_zoning_contains_distinct_retail_categories(self):
        categories = {asset.category for asset in self.layout.assets if asset.name.startswith("product_")}
        self.assertTrue({"cereal", "snacks", "cans", "jars", "juice", "water", "soda"} <= categories)
        self.assertTrue(any(asset.name.startswith("endcap_") for asset in self.layout.assets))


if __name__ == "__main__":
    unittest.main()
