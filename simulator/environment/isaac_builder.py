"""Optional Isaac Sim USD adapter for the deterministic aisle description.

Imports are intentionally delayed: the core and its tests do not require the
Isaac runtime. The USD primitive API is stable enough for this small adapter,
while sensor graph/API choices remain isolated in the runtime integration.
"""

from __future__ import annotations

from simulator.environment.aisle_builder import AisleLayout


class IsaacRuntimeUnavailable(RuntimeError):
    pass


class IsaacAisleBuilder:
    def __init__(self, stage=None):
        try:
            import omni.usd  # type: ignore
            from pxr import Gf, UsdGeom  # type: ignore
        except ImportError as exc:
            raise IsaacRuntimeUnavailable("Isaac Sim USD modules are unavailable") from exc
        self._omni_usd = omni.usd
        self._Gf = Gf
        self._UsdGeom = UsdGeom
        self.stage = stage or omni.usd.get_context().get_stage()

    def build(self, layout: AisleLayout, root: str = "/World/GroceryAisle") -> int:
        """Materialize layout boxes and return the number of created primitives."""
        for primitive in layout.primitives:
            path = f"{root}/{primitive.kind}/{primitive.name}"
            cube = self._UsdGeom.Cube.Define(self.stage, path)
            cube.AddTranslateOp().Set(self._Gf.Vec3d(*primitive.center_m))
            cube.AddScaleOp().Set(self._Gf.Vec3d(*(size / 2.0 for size in primitive.size_m)))
        return len(layout.primitives)
