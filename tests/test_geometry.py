import unittest
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.scenario = load_scenario(Path(__file__).resolve().parents[1] / "config/scenarios/baseline_straight.yaml")

    def test_fixed_seed_is_deterministic(self):
        first = build_aisle_layout(self.scenario.environment)
        second = build_aisle_layout(self.scenario.environment)
        self.assertEqual(first, second)
        self.assertGreater(len(first.products), 0)

    def test_layout_contains_two_shelf_rows_and_floor(self):
        layout = build_aisle_layout(self.scenario.environment)
        self.assertEqual(sum(p.kind == "floor" for p in layout.primitives), 1)
        shelf_count = sum(p.kind == "shelf" for p in layout.primitives)
        bay_count = int(self.scenario.environment.length_m // self.scenario.environment.bay_width_m)
        self.assertGreater(shelf_count, 2 * bay_count * self.scenario.environment.shelf_levels)
        self.assertEqual(shelf_count, 231)
