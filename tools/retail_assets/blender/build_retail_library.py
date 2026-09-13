"""Optional Blender authoring path for the reusable retail library.

Run from the repository root with:

    blender --background --python tools/retail_assets/blender/build_retail_library.py

The checked-in USDA files remain the runtime source of truth.  This script is
kept intentionally small: it creates representative beveled source meshes and
exports them when the installed Blender build provides USD export support.  It
then regenerates the deterministic runtime catalog/textures so the two paths
share the same asset keys.
"""

from __future__ import annotations

import sys
from pathlib import Path


try:
    import bpy  # type: ignore
except ImportError as exc:  # pragma: no cover - only executed outside Blender
    raise SystemExit("This script must be run by Blender: blender --background --python tools/retail_assets/blender/build_retail_library.py") from exc


REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def _material(name: str, color: tuple[float, float, float]):
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color, 1.0)
    return material


def _beveled_box(name: str, dimensions: tuple[float, float, float], material):
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    bevel = obj.modifiers.new("small packaging bevel", "BEVEL")
    bevel.width = min(dimensions) * 0.06
    bevel.segments = 3
    obj.data.materials.append(material)
    return obj


def build_source_meshes() -> None:
    _clear_scene()
    cardboard = _material("FictionalCardboard", (0.8, 0.35, 0.1))
    plastic = _material("FictionalPlastic", (0.1, 0.4, 0.8))
    _beveled_box("CartonSource", (0.18, 0.10, 0.31), cardboard)
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.05, depth=0.26)
    bottle = bpy.context.object
    bottle.name = "BottleSource"
    bottle.data.materials.append(plastic)
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=0.055, depth=0.13)
    can = bpy.context.object
    can.name = "CanSource"
    can.data.materials.append(cardboard)


def export_source_blend() -> None:
    source_dir = REPO / "tools" / "retail_assets" / "source"
    source_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(source_dir / "retail_library_source.blend"))


def export_usd_if_available() -> None:
    export = getattr(getattr(bpy.ops, "wm", None), "usd_export", None)
    if export is None:
        print("Blender USD exporter unavailable; keeping deterministic checked-in USDA runtime assets.")
        return
    output = REPO / "assets" / "retail" / "usd" / "blender_source_library.usda"
    output.parent.mkdir(parents=True, exist_ok=True)
    export(filepath=str(output), selected_objects_only=False)


if __name__ == "__main__":
    build_source_meshes()
    export_source_blend()
    export_usd_if_available()
    from tools.retail_assets.generate_packaging import generate_library

    print(f"Generated catalog: {generate_library(REPO)}")
