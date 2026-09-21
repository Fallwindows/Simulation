import unittest
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import CEILING_HEIGHT_M, OVERHEAD_SIGN_X_M, build_aisle_layout
from simulator.runtime.isaac_sim_runner import shopper_cart_position_at_time


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
        self.assertEqual(shelf_count, 229)

    def test_layout_contains_observable_store_shell_lights_and_sign(self):
        layout = build_aisle_layout(self.scenario.environment)
        kinds = [primitive.kind for primitive in layout.primitives]
        self.assertEqual(kinds.count("ceiling"), 1)
        self.assertEqual(kinds.count("wall"), 3)
        self.assertEqual(kinds.count("light_panel"), 16)
        self.assertGreaterEqual(kinds.count("ceiling_grid"), 20)
        ceiling = next(primitive for primitive in layout.primitives if primitive.kind == "ceiling")
        self.assertEqual(ceiling.center_m[2], CEILING_HEIGHT_M)
        self.assertEqual(len(layout.fixtures), 1)
        sign = layout.fixtures[0]
        self.assertEqual(sign.asset_key, "promo_market_sign")
        self.assertEqual(sign.semantic_id, "fixture/promo_market_sign/overhead_01")
        self.assertEqual(sign.position_m, (OVERHEAD_SIGN_X_M, 0.0, 2.52))
        self.assertEqual(sign.rotation_rpy_deg, (0.0, 0.0, -90.0))
        self.assertNotIn(sign, layout.products)

    def test_shopper_cart_motion_is_deterministic_and_stays_ahead(self):
        samples = [shopper_cart_position_at_time(timestamp) for timestamp in (0.0, 4.0, 10.0, 20.5)]
        for sample, expected_x in zip(samples, (11.5, 14.1, 18.0, 24.825)):
            self.assertAlmostEqual(sample[0], expected_x, places=12)
            self.assertEqual(sample[1:], (0.0, 0.0))
        sensor_x_at_end = self.scenario.trajectory.start_position_m[0] + self.scenario.trajectory.speed_mps * 20.5 + self.scenario.camera.pose_in_rig.position_m[0]
        self.assertGreater(samples[-1][0] - sensor_x_at_end, 2.0)
        with self.assertRaises(ValueError):
            shopper_cart_position_at_time(-0.1)
