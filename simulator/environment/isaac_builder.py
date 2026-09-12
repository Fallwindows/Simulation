"""Optional Isaac Sim USD adapter for the deterministic aisle description.

Imports are intentionally delayed: the core and its tests do not require the
Isaac runtime. The USD primitive API is stable enough for this small adapter,
while sensor graph/API choices remain isolated in the runtime integration.
"""

from __future__ import annotations

import re

from simulator.environment.aisle_builder import AisleLayout


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
        # USD Stage objects can evaluate false when they have no default prim;
        # use an explicit None check so a freshly created Isaac stage is kept.
        self.stage = stage if stage is not None else omni.usd.get_context().get_stage()

    def build(self, layout: AisleLayout, root: str = "/World/GroceryAisle") -> int:
        """Materialize layout boxes and return the number of created primitives."""
        for primitive in layout.primitives:
            # USD path components must be valid identifiers.  The deterministic
            # geometry uses ``-1``/``1`` side suffixes for uprights, so convert
            # those (and any future punctuation) without changing object order.
            kind = re.sub(r"[^A-Za-z0-9_]", "_", primitive.kind)
            name = re.sub(r"[^A-Za-z0-9_]", "_", primitive.name)
            path = f"{root}/{kind}/{name}"
            sdf_path = self._Sdf.Path(path)
            if not sdf_path.IsAbsolutePath:
                raise ValueError(f"USD primitive path must be absolute, got {path!r}")
            prim = self.stage.DefinePrim(sdf_path, "Cube")
            cube = self._UsdGeom.Cube(prim)
            cube.AddTranslateOp().Set(self._Gf.Vec3d(*primitive.center_m))
            cube.AddScaleOp().Set(self._Gf.Vec3d(*(size / 2.0 for size in primitive.size_m)))
        return len(layout.primitives)
