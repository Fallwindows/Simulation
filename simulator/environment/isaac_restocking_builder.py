"""Isaac USD authoring adapter for :mod:`restocking_layout`.

Only the new room fixtures/supports and the one manipulation product receive
collision.  The existing aisle catalog instances remain render-only.
"""

from __future__ import annotations

from dataclasses import dataclass

from simulator.environment.isaac_builder import IsaacAisleBuilder, IsaacRuntimeUnavailable
from simulator.environment.retail_catalog import load_retail_catalog
from simulator.environment.restocking_layout import (
    RESTOCKING_ROOT,
    RestockingLayout,
    validate_restocking_layout,
)


@dataclass(frozen=True)
class IsaacRestockingHandles:
    root_prim_path: str
    room_floor_prim_path: str
    room_wall_prim_paths: tuple[str, ...]
    pickup_support_prim_path: str
    destination_support_prim_path: str
    product_rigid_body_prim_path: str
    product_visual_prim_path: str
    product_collider_prim_path: str
    product_contact_material_prim_path: str


class IsaacRestockingBuilder:
    """Author a validated layout at the frozen ``/World/Restocking`` root."""

    def __init__(self, stage=None):
        try:
            import omni.usd  # type: ignore
            from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade  # type: ignore
        except ImportError as exc:
            raise IsaacRuntimeUnavailable("Isaac Sim USD/PhysX modules are unavailable") from exc
        self._omni_usd = omni.usd
        self._Gf = Gf
        self._PhysxSchema = PhysxSchema
        self._Sdf = Sdf
        self._UsdGeom = UsdGeom
        self._UsdPhysics = UsdPhysics
        self._UsdShade = UsdShade
        self.stage = stage if stage is not None else omni.usd.get_context().get_stage()
        self._visual_builder = IsaacAisleBuilder(self.stage)

    @staticmethod
    def handles_for(layout: RestockingLayout) -> IsaacRestockingHandles:
        """Return the stable integration paths without requiring Isaac imports."""

        validate_restocking_layout(layout)
        wall_paths = tuple(
            fixture.prim_path
            for fixture in layout.fixtures
            if "/Room/Walls/" in fixture.prim_path
        )
        return IsaacRestockingHandles(
            root_prim_path=RESTOCKING_ROOT,
            room_floor_prim_path=f"{RESTOCKING_ROOT}/Room/Floor",
            room_wall_prim_paths=wall_paths,
            pickup_support_prim_path=layout.pickup_support.fixture.prim_path,
            destination_support_prim_path=layout.destination_support.fixture.prim_path,
            product_rigid_body_prim_path=layout.product.rigid_body_prim_path,
            product_visual_prim_path=layout.product.visual_prim_path,
            product_collider_prim_path=layout.product.collider_prim_path,
            product_contact_material_prim_path=(
                f"{RESTOCKING_ROOT}/PhysicsMaterials/ProductContact"
            ),
        )

    def _apply_collision(self, prim, *, contact_offset_m: float, rest_offset_m: float) -> None:
        collision = self._UsdPhysics.CollisionAPI.Apply(prim)
        collision.CreateCollisionEnabledAttr(True)
        physx_collision = self._PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx_collision.CreateContactOffsetAttr(contact_offset_m)
        physx_collision.CreateRestOffsetAttr(rest_offset_m)

    def _author_fixture(self, fixture) -> None:
        self._visual_builder.build_static_box(
            fixture.prim_path,
            fixture.center_m,
            fixture.size_m,
            fixture.kind,
            RESTOCKING_ROOT,
        )
        if fixture.collision_enabled:
            prim = self.stage.GetPrimAtPath(fixture.prim_path)
            self._apply_collision(prim, contact_offset_m=0.004, rest_offset_m=0.0)

    def _author_contact_material(self, layout: RestockingLayout) -> str:
        path = f"{RESTOCKING_ROOT}/PhysicsMaterials/ProductContact"
        material = self._UsdShade.Material.Define(self.stage, path)
        physics = self._UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(layout.product.static_friction)
        physics.CreateDynamicFrictionAttr(layout.product.dynamic_friction)
        physics.CreateRestitutionAttr(layout.product.restitution)
        material.GetPrim().CreateAttribute(
            "restocking:assumption",
            self._Sdf.ValueTypeNames.String,
        ).Set(layout.product.assumption_note)
        return path

    def _author_product(self, layout: RestockingLayout, material_path: str) -> None:
        product = layout.product
        body = self._UsdGeom.Xform.Define(self.stage, product.rigid_body_prim_path).GetPrim()
        xformable = self._UsdGeom.Xformable(body)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(self._Gf.Vec3d(*product.source_reset_pose.position_m))
        qx, qy, qz, qw = product.source_reset_pose.orientation_xyzw
        xformable.AddOrientOp().Set(self._Gf.Quatd(qw, self._Gf.Vec3d(qx, qy, qz)))
        body.CreateAttribute("restocking:asset_key", self._Sdf.ValueTypeNames.String).Set(product.asset_key)
        body.CreateAttribute("restocking:reset_policy", self._Sdf.ValueTypeNames.String).Set(
            "initialization_only_no_phase_snap"
        )

        rigid_body = self._UsdPhysics.RigidBodyAPI.Apply(body)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(False)
        rigid_body.CreateVelocityAttr(self._Gf.Vec3f(0.0, 0.0, 0.0))
        rigid_body.CreateAngularVelocityAttr(self._Gf.Vec3f(0.0, 0.0, 0.0))
        mass = self._UsdPhysics.MassAPI.Apply(body)
        mass.CreateMassAttr(product.mass_kg)
        mass.CreateCenterOfMassAttr(self._Gf.Vec3f(*product.center_of_mass_m))

        catalog = load_retail_catalog(layout.asset_manifest_path)
        record = catalog.by_key(product.asset_key)
        visual = self.stage.DefinePrim(self._Sdf.Path(product.visual_prim_path), "Xform")
        if not visual.GetReferences().AddReference(str(record.usd_path).replace("\\", "/")):
            raise RuntimeError(f"Failed to reference manipulation asset {record.usd_path}")
        visual.SetInstanceable(True)

        collider_prim = self.stage.DefinePrim(self._Sdf.Path(product.collider_prim_path), "Cube")
        collider = self._UsdGeom.Cube(collider_prim)
        collider.GetSizeAttr().Set(1.0)
        collider_api = self._UsdGeom.XformCommonAPI(collider_prim)
        collider_api.SetScale(self._Gf.Vec3f(*product.dimensions_m))
        collider.CreateVisibilityAttr().Set(self._UsdGeom.Tokens.invisible)
        self._apply_collision(collider_prim, contact_offset_m=0.003, rest_offset_m=0.0)
        collider_prim.CreateRelationship("material:binding:physics").SetTargets([
            self._Sdf.Path(material_path)
        ])

    def build(self, layout: RestockingLayout) -> IsaacRestockingHandles:
        validate_restocking_layout(layout)
        if layout.root_prim_path != RESTOCKING_ROOT:
            raise ValueError(f"Restocking root is frozen at {RESTOCKING_ROOT}")
        self._UsdGeom.Xform.Define(self.stage, RESTOCKING_ROOT)
        for fixture in layout.fixtures:
            self._author_fixture(fixture)
        material_path = self._author_contact_material(layout)
        self._author_product(layout, material_path)

        return self.handles_for(layout)
