"""Shared USD preview materials for the retail environment.

The module intentionally imports ``pxr`` only when Isaac code constructs the
library.  Layout tests and catalog validation remain usable on a clean Python
installation.
"""

from __future__ import annotations

from pathlib import Path


class MaterialLibrary:
    def __init__(self, stage, scope: str = "/World/Looks"):
        from pxr import UsdShade

        self.stage = stage
        self.scope = scope.rstrip("/")
        self._UsdShade = UsdShade
        self._materials = {}
        stage.DefinePrim(self.scope, "Scope")

    def get_or_create(
        self,
        name: str,
        color: tuple[float, float, float],
        roughness: float = 0.58,
        metallic: float = 0.0,
        texture_path: str | Path | None = None,
    ):
        from pxr import Gf, Sdf

        safe_name = "".join(character if character.isalnum() or character == "_" else "_" for character in name)
        if safe_name in self._materials:
            return self._materials[safe_name]
        material = self._UsdShade.Material.Define(self.stage, f"{self.scope}/{safe_name}")
        shader = self._UsdShade.Shader.Define(self.stage, f"{self.scope}/{safe_name}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
        surface_output = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(surface_output)

        if texture_path is not None:
            reader = self._UsdShade.Shader.Define(self.stage, f"{self.scope}/{safe_name}/StReader")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            texture = self._UsdShade.Shader.Define(self.stage, f"{self.scope}/{safe_name}/Texture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(Path(texture_path).resolve()).replace("\\", "/"))
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            shader.GetInput("diffuseColor").ConnectToSource(texture.ConnectableAPI(), "rgb")

        self._materials[safe_name] = material
        return material

    def bind(self, prim, material) -> None:
        self._UsdShade.MaterialBindingAPI(prim).Bind(material)

    def create_defaults(self) -> dict[str, object]:
        """Create the named shared materials used by the aisle builder."""
        return {
            "shelving_metal": self.get_or_create("ShelvingMetal", (0.20, 0.24, 0.27), 0.32, 0.55),
            "floor_tile": self.get_or_create("FloorTile", (0.42, 0.45, 0.46), 0.72, 0.0),
            "drywall": self.get_or_create("Drywall", (0.72, 0.69, 0.63), 0.86, 0.0),
            "ceiling": self.get_or_create("Ceiling", (0.78, 0.80, 0.79), 0.90, 0.0),
            "plastic": self.get_or_create("Plastic", (0.16, 0.20, 0.22), 0.48, 0.0),
            "glass": self.get_or_create("Glass", (0.62, 0.78, 0.82), 0.18, 0.05),
            "cans_metal": self.get_or_create("CansMetal", (0.38, 0.41, 0.43), 0.26, 0.72),
            "cardboard": self.get_or_create("Cardboard", (0.55, 0.39, 0.22), 0.78, 0.0),
            "price_strip": self.get_or_create("PriceStrip", (0.92, 0.87, 0.62), 0.48, 0.0),
            "endcap": self.get_or_create("Endcap", (0.32, 0.21, 0.12), 0.60, 0.0),
        }
