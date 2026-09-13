"""Isaac Sim USD adapter for the deterministic aisle description."""

from __future__ import annotations

import re

from simulator.environment.aisle_builder import AisleLayout
from simulator.environment.retail_catalog import load_retail_catalog


class IsaacRuntimeUnavailable(RuntimeError):
    pass


class IsaacAisleBuilder:
    def __init__(self, stage=None):
        try:
            import omni.usd  # type: ignore
            from pxr import Gf, Sdf, UsdGeom  # type: ignore
        except ImportError as exc:
            raise IsaacRuntimeUnavailable("Isaac Sim USD modules are unavailable") from exc
        self._omni_usd = omni.usd
        self._Gf = Gf
        self._Sdf = Sdf
        self._UsdGeom = UsdGeom
        self.stage = stage if stage is not None else omni.usd.get_context().get_stage()

    @staticmethod
    def _safe(value: str) -> str:
        return re.sub(r"[^A-Za-z0-9_]", "_", value)

    def _build_box(self, primitive, path: str) -> None:
        prim = self.stage.DefinePrim(self._Sdf.Path(path), "Cube")
        cube = self._UsdGeom.Cube(prim)
        cube.GetSizeAttr().Set(1.0)
        api = self._UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(self._Gf.Vec3d(*primitive.center_m))
        api.SetScale(self._Gf.Vec3f(*primitive.size_m))
        prim.CreateAttribute("grocery:kind", self._Sdf.ValueTypeNames.String).Set(primitive.kind)

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
        for primitive in layout.primitives:
            kind = self._safe(primitive.kind)
            name = self._safe(primitive.name)
            self._build_box(primitive, f"{root}/structure/{kind}/{name}")
        for asset in layout.assets:
            self._build_asset(asset, catalog, root)
        return len(layout.primitives) + len(layout.assets)
