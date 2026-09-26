from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.isaac_restocking_builder import (
    IsaacRestockingBuilder,
    _author_initial_product_pose,
)
from simulator.environment.restocking_layout import (
    ASSUMED_PRODUCT_DYNAMIC_FRICTION,
    ASSUMED_PRODUCT_MASS_KG,
    ASSUMED_PRODUCT_RESTITUTION,
    ASSUMED_PRODUCT_STATIC_FRICTION,
    build_restocking_layout,
    validate_restocking_layout,
)


ROOT = Path(__file__).resolve().parents[1]


class _Vec3d(tuple):
    def __new__(cls, *values):
        return super().__new__(cls, values)


class _Vec3f(tuple):
    def __new__(cls, *values):
        return super().__new__(cls, values)


class _Quatf:
    def __init__(self, real, imaginary):
        if not isinstance(imaginary, _Vec3f):
            raise TypeError("Quatf requires Vec3f")
        self.real = real
        self.imaginary = imaginary


class _FakeGf:
    Vec3d = _Vec3d
    Vec3f = _Vec3f
    Quatf = _Quatf


class _TypedOp:
    def __init__(self, expected_type):
        self.expected_type = expected_type
        self.value = None

    def Set(self, value):
        if not isinstance(value, self.expected_type):
            raise TypeError(
                f"expected {self.expected_type.__name__}, got {type(value).__name__}"
            )
        self.value = value


class _FakeXformable:
    def __init__(self):
        self.cleared = False
        self.translate = _TypedOp(_Vec3d)
        self.orient = _TypedOp(_Quatf)

    def ClearXformOpOrder(self):
        self.cleared = True

    def AddTranslateOp(self):
        return self.translate

    def AddOrientOp(self):
        return self.orient


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

    def test_product_pose_matches_default_usd_translate_and_orient_types(self):
        xformable = _FakeXformable()
        pose = self.layout.product.source_reset_pose

        _author_initial_product_pose(xformable, _FakeGf, pose)

        self.assertTrue(xformable.cleared)
        self.assertEqual(tuple(xformable.translate.value), pose.position_m)
        self.assertIsInstance(xformable.orient.value, _Quatf)
        self.assertEqual(xformable.orient.value.real, pose.orientation_xyzw[3])
        self.assertEqual(
            tuple(xformable.orient.value.imaginary), pose.orientation_xyzw[:3]
        )


if __name__ == "__main__":
    unittest.main()
