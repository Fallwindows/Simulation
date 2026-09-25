"""Isaac Sim USD adapter for the deterministic aisle description."""

from __future__ import annotations

import re

from simulator.environment.aisle_builder import AisleLayout
from simulator.environment.retail_catalog import load_retail_catalog


STRUCTURAL_MATERIALS = {
    "floor": {
        "color": (0.34, 0.315, 0.285),
        "roughness": 0.34,
        "metallic": 0.02,
        "round_edges_m": 0.006,
    },
    "shelf": {
        "color": (0.26, 0.275, 0.29),
        "roughness": 0.42,
        "metallic": 0.16,
        "round_edges_m": 0.008,
    },
    "upright": {
        "color": (0.20, 0.215, 0.23),
        "roughness": 0.44,
        "metallic": 0.20,
        "round_edges_m": 0.006,
    },
    "price_rail": {
        "color": (0.22, 0.235, 0.25),
        "roughness": 0.38,
        "metallic": 0.12,
        "round_edges_m": 0.004,
    },
    "price_tag": {
        "color": (0.91, 0.89, 0.82),
        "roughness": 0.68,
        "metallic": 0.0,
        "round_edges_m": 0.002,
    },
    "wall": {
        "color": (0.64, 0.61, 0.55),
        "roughness": 0.78,
        "metallic": 0.0,
        "round_edges_m": 0.004,
    },
    "baseboard": {
        "color": (0.105, 0.115, 0.12),
        "roughness": 0.42,
        "metallic": 0.48,
        "round_edges_m": 0.005,
    },
    "ceiling": {
        "color": (0.69, 0.66, 0.60),
        "roughness": 0.86,
        "metallic": 0.0,
        "round_edges_m": 0.003,
    },
    "ceiling_grid": {
        "color": (0.25, 0.255, 0.25),
        "roughness": 0.46,
        "metallic": 0.35,
        "round_edges_m": 0.002,
    },
    "light_panel": {
        "color": (0.96, 0.91, 0.79),
        "roughness": 0.42,
        "metallic": 0.0,
        "round_edges_m": 0.004,
        "emission": (1.0, 0.91, 0.76),
        "emission_intensity": 1000.0,
    },
    "floor_inlay": {
        "color": (0.285, 0.275, 0.255),
        "roughness": 0.42,
        "metallic": 0.02,
        "round_edges_m": 0.002,
    },
    "sign_green": {
        "color": (0.045, 0.22, 0.15),
        "roughness": 0.50,
        "metallic": 0.0,
        "round_edges_m": 0.008,
    },
    "sign_cream": {
        "color": (0.92, 0.86, 0.68),
        "roughness": 0.58,
        "metallic": 0.0,
        "round_edges_m": 0.004,
    },
    "case_interior": {
        "color": (0.055, 0.075, 0.075),
        "roughness": 0.52,
        "metallic": 0.08,
        "round_edges_m": 0.005,
    },
    "end_product_red": {
        "color": (0.55, 0.10, 0.075),
        "roughness": 0.58,
        "metallic": 0.0,
        "round_edges_m": 0.004,
    },
    "end_product_yellow": {
        "color": (0.78, 0.46, 0.08),
        "roughness": 0.58,
        "metallic": 0.0,
        "round_edges_m": 0.004,
    },
    "end_product_blue": {
        "color": (0.08, 0.30, 0.42),
        "roughness": 0.58,
        "metallic": 0.0,
        "round_edges_m": 0.004,
    },
    "end_panel_green": {
        "color": (0.10, 0.26, 0.19),
        "roughness": 0.62,
        "metallic": 0.0,
        "round_edges_m": 0.006,
    },
    "end_panel_charcoal": {
        "color": (0.10, 0.115, 0.12),
        "roughness": 0.58,
        "metallic": 0.0,
        "round_edges_m": 0.006,
    },
}


class IsaacRuntimeUnavailable(RuntimeError):
    pass


class IsaacAisleBuilder:
    def __init__(self, stage=None):
        try:
            import omni.usd  # type: ignore
            from pxr import Gf, Sdf, UsdGeom, UsdShade  # type: ignore
        except ImportError as exc:
            raise IsaacRuntimeUnavailable("Isaac Sim USD modules are unavailable") from exc
        self._omni_usd = omni.usd
        self._Gf = Gf
        self._Sdf = Sdf
        self._UsdGeom = UsdGeom
        self._UsdShade = UsdShade
        self.stage = stage if stage is not None else omni.usd.get_context().get_stage()
        self._materials = {}

    @staticmethod
    def _safe(value: str) -> str:
        return re.sub(r"[^A-Za-z0-9_]", "_", value)

    def _material(self, root: str, kind: str):
        cache_key = (root, kind)
        if cache_key in self._materials:
            return self._materials[cache_key]

        from isaacsim.core.experimental.materials import OmniPbrMaterial  # type: ignore

        values = STRUCTURAL_MATERIALS.get(
            kind,
            {"color": (0.31, 0.32, 0.33), "roughness": 0.55, "metallic": 0.0, "round_edges_m": 0.003},
        )
        material = OmniPbrMaterial(f"{root}/looks/{self._safe(kind)}")
        material.set_input_values("diffuse_color_constant", values["color"])
        material.set_input_values("reflection_roughness_constant", [values["roughness"]])
        material.set_input_values("metallic_constant", [values["metallic"]])
        material.set_input_values("round_edges_radius", [values["round_edges_m"]])
        material.set_input_values("round_edges_roundness", [0.35])
        if "emission" in values:
            material.set_input_values("enable_emission", [True])
            material.set_input_values("emissive_color", values["emission"])
            material.set_input_values("emissive_intensity", [values["emission_intensity"]])
        usd_material = material.materials[0]
        self._materials[cache_key] = usd_material
        return usd_material

    def build_static_box(self, path: str, center_m, size_m, kind: str, root: str = "/World/GroceryAisle") -> None:
        """Author one material-aware static box without touching referenced assets."""

        prim = self.stage.DefinePrim(self._Sdf.Path(path), "Cube")
        cube = self._UsdGeom.Cube(prim)
        cube.GetSizeAttr().Set(1.0)
        api = self._UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(self._Gf.Vec3d(*center_m))
        api.SetScale(self._Gf.Vec3f(*size_m))
        prim.CreateAttribute("grocery:kind", self._Sdf.ValueTypeNames.String).Set(kind)
        self._UsdShade.MaterialBindingAPI.Apply(prim).Bind(self._material(root, kind))

    def _build_box(self, primitive, path: str, root: str) -> None:
        self.build_static_box(path, primitive.center_m, primitive.size_m, primitive.kind, root)

    def _build_shelf_details(self, primitive, root: str) -> int:
        """Add aisle-facing lips and price cards to a structural shelf board."""

        front_sign = 1.0 if primitive.center_m[1] < 0.0 else -1.0
        x, y, z = primitive.center_m
        width, depth, _ = primitive.size_m
        safe_name = self._safe(primitive.name)
        front_y = y + front_sign * (depth / 2.0 + 0.018)
        self.build_static_box(
            f"{root}/structure/shelf_detail/{safe_name}_rail",
            (x, front_y, z + 0.042),
            (width, 0.036, 0.084),
            "price_rail",
            root,
        )
        # Alternate the card position per bay so the repeating shelf run does
        # not form one artificial vertical stripe in the camera view.
        offset = 0.25 if sum(ord(ch) for ch in primitive.name) % 2 else -0.25
        self.build_static_box(
            f"{root}/structure/shelf_detail/{safe_name}_tag",
            (x + offset * width, front_y + front_sign * 0.021, z + 0.045),
            (0.18, 0.008, 0.072),
            "price_tag",
            root,
        )
        return 2

    def _build_asset(self, asset, catalog, root: str) -> None:
        record = catalog.by_key(asset.asset_key)
        path = f"{root}/retail_assets/{self._safe(asset.category)}/{self._safe(asset.name)}"
        prim = self.stage.DefinePrim(self._Sdf.Path(path), "Xform")
        reference_path = str(record.usd_path).replace("\\", "/")
        if not prim.GetReferences().AddReference(reference_path):
            raise RuntimeError(f"Failed to add USD reference {reference_path} at {path}")
        prim.SetInstanceable(True)
        api = self._UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(self._Gf.Vec3d(*asset.position_m))
        api.SetRotate(self._Gf.Vec3f(*asset.rotation_rpy_deg), self._UsdGeom.XformCommonAPI.RotationOrderXYZ)
        api.SetScale(self._Gf.Vec3f(*asset.scale_xyz))
        prim.CreateAttribute("grocery:asset_key", self._Sdf.ValueTypeNames.String).Set(asset.asset_key)
        prim.CreateAttribute("grocery:category", self._Sdf.ValueTypeNames.String).Set(asset.category)
        prim.CreateAttribute("grocery:semantic_id", self._Sdf.ValueTypeNames.String).Set(asset.semantic_id)

    def build(self, layout: AisleLayout, root: str = "/World/GroceryAisle") -> int:
        """Build original structure and reference assets in occupied slots."""
        catalog = load_retail_catalog(layout.asset_manifest_path)
        authored_count = 0
        for primitive in layout.primitives:
            kind = self._safe(primitive.kind)
            name = self._safe(primitive.name)
            self._build_box(primitive, f"{root}/structure/{kind}/{name}", root)
            authored_count += 1
            if primitive.kind == "shelf":
                authored_count += self._build_shelf_details(primitive, root)
        for asset in layout.assets:
            self._build_asset(asset, catalog, root)
            authored_count += 1
        return authored_count
