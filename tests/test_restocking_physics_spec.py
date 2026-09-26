from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.isaac_restocking_builder import IsaacRestockingBuilder
from simulator.environment.restocking_layout import (
    ASSUMED_PRODUCT_DYNAMIC_FRICTION,
    ASSUMED_PRODUCT_MASS_KG,
    ASSUMED_PRODUCT_RESTITUTION,
    ASSUMED_PRODUCT_STATIC_FRICTION,
    build_restocking_layout,
    validate_restocking_layout,
)


ROOT = Path(__file__).resolve().parents[1]


class RestockingPhysicsSpecTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scenario = load_scenario(ROOT / "config/scenarios/baseline_straight.yaml")
        cls.layout = build_restocking_layout(build_aisle_layout(scenario.environment))

    def test_product_physics_assumptions_are_explicit_and_valid(self):
        product = self.layout.product
        self.assertEqual(product.mass_kg, ASSUMED_PRODUCT_MASS_KG)
        self.assertEqual(product.static_friction, ASSUMED_PRODUCT_STATIC_FRICTION)
        self.assertEqual(product.dynamic_friction, ASSUMED_PRODUCT_DYNAMIC_FRICTION)
        self.assertEqual(product.restitution, ASSUMED_PRODUCT_RESTITUTION)
        self.assertEqual(product.center_of_mass_m, (0.0, 0.0, 0.0))
        self.assertIn("simulation engineering assumptions", product.assumption_note)
        self.assertIn("not measured", product.assumption_note)

    def test_only_relevant_new_spec_geometry_requests_collision(self):
        self.assertTrue(all(fixture.collision_enabled for fixture in self.layout.fixtures))
        self.assertFalse(self.layout.pickup_manipulation_clearance.collision_enabled)
        self.assertFalse(self.layout.destination_manipulation_clearance.collision_enabled)
        self.assertEqual(self.layout.product.collider_prim_path, "/World/Restocking/Product/Collider")
        self.assertEqual(self.layout.product.visual_prim_path, "/World/Restocking/Product/Visual")

    def test_handle_contract_uses_stable_world_paths(self):
        handles = IsaacRestockingBuilder.handles_for(self.layout)
        self.assertEqual(handles.pickup_support_prim_path, self.layout.pickup_support.fixture.prim_path)
        self.assertEqual(
            handles.destination_support_prim_path,
            self.layout.destination_support.fixture.prim_path,
        )
        self.assertEqual(
            handles.product_rigid_body_prim_path,
            self.layout.product.rigid_body_prim_path,
        )
        self.assertGreaterEqual(len(handles.room_wall_prim_paths), 6)
        for value in (
            handles.root_prim_path,
            handles.room_floor_prim_path,
            *handles.room_wall_prim_paths,
            handles.pickup_support_prim_path,
            handles.destination_support_prim_path,
            handles.product_rigid_body_prim_path,
            handles.product_visual_prim_path,
            handles.product_collider_prim_path,
            handles.product_contact_material_prim_path,
        ):
            self.assertTrue(value.startswith("/World/Restocking"))

    def test_validation_rejects_unphysical_contact_parameters(self):
        invalid = replace(
            self.layout,
            product=replace(
                self.layout.product,
                static_friction=0.2,
                dynamic_friction=0.3,
            ),
        )
        with self.assertRaisesRegex(ValueError, "dynamic <= static"):
            validate_restocking_layout(invalid)

    def test_reset_pose_is_initialization_data_not_a_phase_command(self):
        self.assertEqual(self.layout.product.source_reset_pose, self.layout.known_pickup_pose)
        self.assertNotEqual(
            self.layout.product.source_reset_pose,
            self.layout.product.destination_support_pose,
        )


if __name__ == "__main__":
    unittest.main()
