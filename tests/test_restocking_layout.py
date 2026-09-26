from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.restocking_layout import (
    CONSERVATIVE_ROBOT_PLANAR_BOUNDS_M,
    PASTA_BOX_DIMENSIONS_M,
    RESTOCKING_ROOT,
    build_restocking_layout,
    route_has_conservative_clearance,
    validate_restocking_layout,
    with_doorway_width,
)


ROOT = Path(__file__).resolve().parents[1]


class RestockingLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scenario = load_scenario(ROOT / "config/scenarios/baseline_straight.yaml")
        cls.aisle = build_aisle_layout(scenario.environment)

    def test_layout_is_deterministic_and_attaches_to_open_end(self):
        first = build_restocking_layout(self.aisle)
        second = build_restocking_layout(self.aisle)
        self.assertEqual(first, second)
        aisle_floor = next(box for box in self.aisle.primitives if box.name == "floor")
        open_end_x = aisle_floor.center_m[0] - aisle_floor.size_m[0] / 2.0
        self.assertAlmostEqual(first.doorway.center_m[0], open_end_x)
        self.assertTrue(all(box.prim_path.startswith(RESTOCKING_ROOT + "/") for box in first.fixtures))
        self.assertEqual(first.product.rigid_body_prim_path, "/World/Restocking/Product")

    def test_doorway_and_routes_clear_conservative_yaw_independent_bounds(self):
        layout = build_restocking_layout(self.aisle)
        required_width = max(CONSERVATIVE_ROBOT_PLANAR_BOUNDS_M)
        self.assertGreater(layout.doorway.clear_width_m, required_width)
        self.assertGreaterEqual(
            (layout.doorway.clear_width_m - required_width) / 2.0,
            0.30,
        )
        self.assertTrue(route_has_conservative_clearance(layout, layout.store_to_pickup_route))
        self.assertTrue(route_has_conservative_clearance(layout, layout.pickup_to_destination_route))

        too_narrow = with_doorway_width(layout, required_width - 0.01)
        with self.assertRaisesRegex(ValueError, "doorway is narrower"):
            validate_restocking_layout(too_narrow)

    def test_product_fits_both_supports_and_clearance_envelopes(self):
        layout = build_restocking_layout(self.aisle)
        self.assertEqual(layout.product.dimensions_m, PASTA_BOX_DIMENSIONS_M)
        self.assertEqual(layout.known_pickup_pose, layout.product.source_reset_pose)
        self.assertEqual(layout.known_shelf_target_pose, layout.product.destination_support_pose)
        for support, pose in (
            (layout.pickup_support, layout.known_pickup_pose),
            (layout.destination_support, layout.known_shelf_target_pose),
        ):
            self.assertAlmostEqual(
                pose.position_m[2] - PASTA_BOX_DIMENSIONS_M[2] / 2.0,
                support.surface_z_m,
            )
            self.assertLessEqual(
                abs(pose.position_m[0] - support.usable_center_xy_m[0]) + PASTA_BOX_DIMENSIONS_M[0] / 2.0,
                support.usable_size_xy_m[0] / 2.0,
            )
            self.assertLessEqual(
                abs(pose.position_m[1] - support.usable_center_xy_m[1]) + PASTA_BOX_DIMENSIONS_M[1] / 2.0,
                support.usable_size_xy_m[1] / 2.0,
            )

    def test_destination_support_is_a_collision_backed_extension_of_existing_shelf(self):
        layout = build_restocking_layout(self.aisle)
        shelf = next(box for box in self.aisle.primitives if box.name == "shelf_r1_b0_l1")
        existing_front_y = shelf.center_m[1] - shelf.size_m[1] / 2.0
        self.assertAlmostEqual(layout.destination_support.fixture.maximum_m[1], existing_front_y)
        self.assertAlmostEqual(layout.destination_support.surface_z_m, shelf.center_m[2] + shelf.size_m[2] / 2.0)
        self.assertTrue(layout.destination_support.fixture.collision_enabled)

    def test_validation_rejects_unsupported_product_pose(self):
        layout = build_restocking_layout(self.aisle)
        bad_pose = replace(
            layout.product.destination_support_pose,
            position_m=(99.0, 99.0, 99.0),
        )
        invalid = replace(
            layout,
            product=replace(layout.product, destination_support_pose=bad_pose),
        )
        with self.assertRaisesRegex(ValueError, "destination support does not fully support"):
            validate_restocking_layout(invalid)


if __name__ == "__main__":
    unittest.main()
