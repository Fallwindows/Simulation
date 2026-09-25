"""Isaac Sim USD adapter for the deterministic aisle description."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

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
        "color": (0.42, 0.435, 0.45),
        "roughness": 0.52,
        "metallic": 0.08,
        "round_edges_m": 0.008,
    },
    "shelf_warm": {
        "color": (0.44, 0.43, 0.405),
        "roughness": 0.56,
        "metallic": 0.06,
        "round_edges_m": 0.008,
    },
    "shelf_cool": {
        "color": (0.39, 0.415, 0.435),
        "roughness": 0.48,
        "metallic": 0.09,
        "round_edges_m": 0.008,
    },
    "upright": {
        "color": (0.34, 0.355, 0.37),
        "roughness": 0.50,
        "metallic": 0.10,
        "round_edges_m": 0.006,
    },
    "upright_warm": {
        "color": (0.36, 0.35, 0.33),
        "roughness": 0.54,
        "metallic": 0.08,
        "round_edges_m": 0.006,
    },
    "price_rail": {
        "color": (0.30, 0.315, 0.33),
        "roughness": 0.46,
        "metallic": 0.07,
        "round_edges_m": 0.004,
    },
    "price_tag": {
        "color": (0.91, 0.89, 0.82),
        "roughness": 0.68,
        "metallic": 0.0,
        "round_edges_m": 0.002,
    },
    "wall": {
        "color": (0.72, 0.70, 0.65),
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
        "color": (0.78, 0.76, 0.71),
        "roughness": 0.86,
        "metallic": 0.0,
        "round_edges_m": 0.003,
    },
    "ceiling_grid": {
        "color": (0.70, 0.705, 0.685),
        "roughness": 0.72,
        "metallic": 0.07,
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
    "floor_tile_warm": {
        "color": (0.365, 0.345, 0.315),
        "roughness": 0.39,
        "metallic": 0.015,
        "round_edges_m": 0.001,
    },
    "floor_tile_cool": {
        "color": (0.335, 0.33, 0.315),
        "roughness": 0.43,
        "metallic": 0.015,
        "round_edges_m": 0.001,
    },
    "ceiling_tile_warm": {
        "color": (0.80, 0.775, 0.72),
        "roughness": 0.90,
        "metallic": 0.0,
        "round_edges_m": 0.002,
    },
    "ceiling_tile_cool": {
        "color": (0.73, 0.745, 0.73),
        "roughness": 0.88,
        "metallic": 0.0,
        "round_edges_m": 0.002,
    },
    "shelf_light": {
        "color": (0.28, 0.29, 0.29),
        "roughness": 0.44,
        "metallic": 0.18,
        "round_edges_m": 0.002,
        "emission": (1.0, 0.90, 0.74),
        "emission_intensity": 55.0,
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
        "color": (0.235, 0.265, 0.28),
        "roughness": 0.48,
        "metallic": 0.08,
        "round_edges_m": 0.005,
    },
    "case_frame": {
        "color": (0.24, 0.265, 0.28),
        "roughness": 0.34,
        "metallic": 0.30,
        "round_edges_m": 0.004,
    },
    "case_glass": {
        "color": (0.68, 0.75, 0.76),
        "roughness": 0.035,
        "metallic": 0.0,
        "specular_level": 1.35,
        "round_edges_m": 0.001,
        "opacity": 0.11,
    },
    "case_glass_reflection": {
        "color": (0.82, 0.90, 0.92),
        "roughness": 0.025,
        "metallic": 0.0,
        "specular_level": 1.6,
        "round_edges_m": 0.001,
        "opacity": 0.16,
    },
    "case_handle": {
        "color": (0.58, 0.60, 0.60),
        "roughness": 0.25,
        "metallic": 0.62,
        "round_edges_m": 0.006,
    },
    "display_wood": {
        "color": (0.34, 0.19, 0.085),
        "roughness": 0.64,
        "metallic": 0.0,
        "round_edges_m": 0.008,
    },
    "upright_slot": {
        "color": (0.075, 0.082, 0.086),
        "roughness": 0.52,
        "metallic": 0.18,
        "round_edges_m": 0.001,
    },
    "shelf_wear": {
        "color": (0.54, 0.55, 0.55),
        "roughness": 0.28,
        "metallic": 0.12,
        "round_edges_m": 0.001,
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


_STRUCTURAL_VARIANTS = {
    "shelf": ("shelf", "shelf_warm", "shelf_cool"),
    "upright": ("upright", "upright_warm"),
}

_SCENE_MATERIAL_ROOT = Path(__file__).resolve().parents[2] / "assets" / "scene" / "materials"
_TEXTURE_SETS = {
    "floor": ("terrazzo_albedo.png", "terrazzo_normal.png", "terrazzo_roughness.png", 0.18, 0.62, 3.0),
    "floor_tile_warm": ("terrazzo_albedo.png", "terrazzo_normal.png", "terrazzo_roughness.png", 0.18, 0.62, 3.0),
    "floor_tile_cool": ("terrazzo_albedo.png", "terrazzo_normal.png", "terrazzo_roughness.png", 0.18, 0.62, 3.0),
    "shelf": ("powdercoat_albedo.png", "micro_normal.png", "micro_roughness.png", 0.28, 0.55, 7.0),
    "shelf_warm": ("powdercoat_warm_albedo.png", "micro_normal.png", "micro_roughness.png", 0.28, 0.55, 7.0),
    "shelf_cool": ("powdercoat_cool_albedo.png", "micro_normal.png", "micro_roughness.png", 0.28, 0.55, 7.0),
    "upright": ("powdercoat_albedo.png", "micro_normal.png", "micro_roughness.png", 0.30, 0.58, 6.0),
    "upright_warm": ("powdercoat_warm_albedo.png", "micro_normal.png", "micro_roughness.png", 0.30, 0.58, 6.0),
    "ceiling": (None, "micro_normal.png", "micro_roughness.png", 0.10, 0.24, 8.0),
    "ceiling_tile_warm": (None, "micro_normal.png", "micro_roughness.png", 0.10, 0.24, 8.0),
    "ceiling_tile_cool": (None, "micro_normal.png", "micro_roughness.png", 0.10, 0.24, 8.0),
    "price_rail": ("price_rail_albedo.png", "price_rail_normal.png", "price_rail_roughness.png", 0.18, 0.72, 8.0),
    "case_frame": ("case_frame_albedo.png", "case_frame_normal.png", "case_frame_roughness.png", 0.22, 0.82, 5.0),
    "case_glass": (None, "case_glass_normal.png", "case_glass_roughness.png", 0.10, 0.92, 2.0),
    "case_glass_reflection": (None, "case_glass_normal.png", "case_glass_roughness.png", 0.10, 0.92, 2.0),
    "display_wood": ("laminate_albedo.png", "laminate_normal.png", "laminate_roughness.png", 0.20, 0.72, 2.4),
    "sign_green": ("category_detail_albedo.png", "category_detail_normal.png", "category_detail_roughness.png", 0.14, 0.64, 3.5),
    "end_panel_green": ("category_detail_albedo.png", "category_detail_normal.png", "category_detail_roughness.png", 0.14, 0.64, 3.5),
    "end_panel_charcoal": ("category_detail_albedo.png", "category_detail_normal.png", "category_detail_roughness.png", 0.12, 0.58, 3.5),
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
        if "specular_level" in values:
            material.set_input_values("specular_level", [values["specular_level"]])
        material.set_input_values("round_edges_radius", [values["round_edges_m"]])
        material.set_input_values("round_edges_roundness", [0.35])
        if kind in _TEXTURE_SETS:
            albedo_name, normal_name, roughness_name, bump, roughness_influence, texture_scale = _TEXTURE_SETS[kind]
            if albedo_name:
                albedo_path = str((_SCENE_MATERIAL_ROOT / albedo_name).resolve()).replace("\\", "/")
                material.set_input_values("diffuse_texture", [albedo_path])
            normal_path = str((_SCENE_MATERIAL_ROOT / normal_name).resolve()).replace("\\", "/")
            roughness_path = str((_SCENE_MATERIAL_ROOT / roughness_name).resolve()).replace("\\", "/")
            material.set_input_values("normalmap_texture", [normal_path])
            material.set_input_values("bump_factor", [bump])
            material.set_input_values("reflectionroughness_texture", [roughness_path])
            material.set_input_values("reflection_roughness_texture_influence", [roughness_influence])
            material.set_input_values("project_uvw", [True])
            material.set_input_values("texture_scale", [(texture_scale, texture_scale)])
        if "emission" in values:
            material.set_input_values("enable_emission", [True])
            material.set_input_values("emissive_color", values["emission"])
            material.set_input_values("emissive_intensity", [values["emission_intensity"]])
        if "opacity" in values:
            material.set_input_values("enable_opacity", [True])
            material.set_input_values("opacity_constant", [values["opacity"]])
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
        variants = _STRUCTURAL_VARIANTS.get(kind, (kind,))
        variant_index = hashlib.sha256(path.encode("utf-8")).digest()[0] % len(variants)
        self._UsdShade.MaterialBindingAPI.Apply(prim).Bind(self._material(root, variants[variant_index]))

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
            (width, 0.016, 0.030),
            "price_rail",
            root,
        )
        # Alternate the card position per bay so the repeating shelf run does
        # not form one artificial vertical stripe in the camera view.
        offset = 0.25 if sum(ord(ch) for ch in primitive.name) % 2 else -0.25
        self.build_static_box(
            f"{root}/structure/shelf_detail/{safe_name}_tag",
            (x + offset * width, front_y + front_sign * 0.021, z + 0.045),
            (0.16, 0.006, 0.044),
            "price_tag",
            root,
        )
        return 2

    def build_catalog_asset(
        self,
        path: str,
        record,
        position_m,
        rotation_rpy_deg=(0.0, 0.0, 0.0),
        scale_xyz=(1.0, 1.0, 1.0),
        semantic_id: str = "",
    ) -> None:
        """Reference one validated catalog asset for scene dressing."""

        prim = self.stage.DefinePrim(self._Sdf.Path(path), "Xform")
        reference_path = str(record.usd_path).replace("\\", "/")
        if not prim.GetReferences().AddReference(reference_path):
            raise RuntimeError(f"Failed to add USD reference {reference_path} at {path}")
        prim.SetInstanceable(True)
        api = self._UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(self._Gf.Vec3d(*position_m))
        api.SetRotate(self._Gf.Vec3f(*rotation_rpy_deg), self._UsdGeom.XformCommonAPI.RotationOrderXYZ)
        api.SetScale(self._Gf.Vec3f(*scale_xyz))
        prim.CreateAttribute("grocery:asset_key", self._Sdf.ValueTypeNames.String).Set(record.asset_key)
        prim.CreateAttribute("grocery:category", self._Sdf.ValueTypeNames.String).Set(record.category)
        prim.CreateAttribute("grocery:semantic_id", self._Sdf.ValueTypeNames.String).Set(
            semantic_id or f"scene/{record.asset_key}/{self._safe(path)}"
        )

    def _build_asset(self, asset, catalog, root: str) -> None:
        record = catalog.by_key(asset.asset_key)
        path = f"{root}/retail_assets/{self._safe(asset.category)}/{self._safe(asset.name)}"
        self.build_catalog_asset(
            path,
            record,
            asset.position_m,
            asset.rotation_rpy_deg,
            asset.scale_xyz,
            asset.semantic_id,
        )

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
