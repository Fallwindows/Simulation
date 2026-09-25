"""Generate the deterministic, self-contained grocery shopping-cart asset.

The asset is authored as USDA text so generation needs only the Python standard
library.  Geometry uses metres, Z-up coordinates, and a single placeable root.
The basket is a real open wire lattice; no opaque basket shell is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = ROOT / "assets" / "scene" / "store_context"
ASSET_BOUNDS_MIN = (-0.518, -0.300, 0.000)
ASSET_BOUNDS_MAX = (0.512, 0.300, 1.030)
ASSET_DIMENSIONS = (1.030, 0.600, 1.030)
OBB_CENTER = (-0.003, 0.000, 0.515)
OBB_HALF_EXTENTS = (0.515, 0.300, 0.515)


@dataclass(frozen=True)
class Tube:
    name: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    radius: float
    material: str
    role: str


@dataclass(frozen=True)
class CollisionBox:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    role: str


def _f(value: float) -> str:
    value = 0.0 if abs(value) < 0.0000005 else value
    return f"{value:.6f}"


def _v3(value: Iterable[float]) -> str:
    return "(" + ", ".join(_f(component) for component in value) + ")"


def _lerp(a: float, b: float, amount: float) -> float:
    return a + (b - a) * amount


def _tube(
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: str,
    role: str,
) -> Tube:
    if start == end:
        raise ValueError(f"zero-length tube: {name}")
    return Tube(name, start, end, radius, material, role)


def build_tubes() -> tuple[Tube, ...]:
    tubes: list[Tube] = []

    def add(name: str, start: tuple[float, float, float], end: tuple[float, float, float],
            radius: float, material: str, role: str) -> None:
        tubes.append(_tube(name, start, end, radius, material, role))

    # Load-bearing chassis and rear handle structure.
    for side, y in (("L", -0.252), ("R", 0.252)):
        add(f"Frame_Base_{side}", (-0.365, y, 0.190), (0.405, y, 0.165), 0.016, "Steel", "frame")
        add(f"Frame_RearPost_{side}", (-0.410, y, 0.185), (-0.392, y, 0.905), 0.016, "Steel", "frame")
        add(f"Frame_Diagonal_{side}", (-0.402, y, 0.455), (0.405, y, 0.185), 0.014, "Steel", "frame")
        add(f"Handle_Riser_{side}", (-0.392, y, 0.905), (-0.500, y, 1.010), 0.016, "Steel", "handle_support")
    add("Frame_FrontCross", (0.405, -0.252, 0.165), (0.405, 0.252, 0.165), 0.016, "Steel", "frame")
    add("Frame_RearCross", (-0.365, -0.252, 0.190), (-0.365, 0.252, 0.190), 0.016, "Steel", "frame")
    add("Handle_Grip", (-0.500, -0.300, 1.010), (-0.500, 0.300, 1.010), 0.018, "HandlePolymer", "handle")

    # Basket perimeter.  Bottom rails rise slightly toward the handle, as on a
    # nested commercial cart, while the top opening remains level.
    side_y = 0.282
    for side, y in (("L", -side_y), ("R", side_y)):
        add(f"Basket_Top_{side}", (-0.355, y, 0.875), (0.500, y, 0.875), 0.010, "ZincWire", "basket_perimeter")
        add(f"Basket_Bottom_{side}", (-0.325, y, 0.505), (0.455, y, 0.455), 0.010, "ZincWire", "basket_perimeter")
    add("Basket_Top_Front", (0.500, -side_y, 0.875), (0.500, side_y, 0.875), 0.010, "ZincWire", "basket_perimeter")
    add("Basket_Bottom_Front", (0.455, -side_y, 0.455), (0.455, side_y, 0.455), 0.010, "ZincWire", "basket_perimeter")
    add("Basket_Top_Rear", (-0.355, -side_y, 0.875), (-0.355, side_y, 0.875), 0.010, "ZincWire", "basket_perimeter")
    add("Basket_Bottom_Rear", (-0.325, -side_y, 0.505), (-0.325, side_y, 0.505), 0.010, "ZincWire", "basket_perimeter")

    # Basket side lattice: four horizontal courses and eleven vertical wires on
    # each side.  The opening above remains unobstructed.
    for side, y in (("L", -side_y), ("R", side_y)):
        for index, fraction in enumerate((0.20, 0.40, 0.60, 0.80), start=1):
            start = (
                _lerp(-0.325, -0.355, fraction), y,
                _lerp(0.505, 0.875, fraction),
            )
            end = (
                _lerp(0.455, 0.500, fraction), y,
                _lerp(0.455, 0.875, fraction),
            )
            add(f"Basket_Side_{side}_Horizontal_{index:02d}", start, end, 0.0042, "ZincWire", "basket_wire")
        for index in range(1, 12):
            fraction = index / 12.0
            x_bottom = _lerp(-0.325, 0.455, fraction)
            x_top = _lerp(-0.355, 0.500, fraction)
            z_bottom = _lerp(0.505, 0.455, fraction)
            add(
                f"Basket_Side_{side}_Vertical_{index:02d}",
                (x_bottom, y, z_bottom), (x_top, y, 0.875),
                0.0042, "ZincWire", "basket_wire",
            )

    # Front and rear panels close the basket with wire only.
    for panel, x_bottom, x_top, z_bottom in (
        ("Front", 0.455, 0.500, 0.455),
        ("Rear", -0.325, -0.355, 0.505),
    ):
        for index, fraction in enumerate((0.20, 0.40, 0.60, 0.80), start=1):
            x = _lerp(x_bottom, x_top, fraction)
            z = _lerp(z_bottom, 0.875, fraction)
            add(f"Basket_{panel}_Horizontal_{index:02d}", (x, -side_y, z), (x, side_y, z), 0.0042, "ZincWire", "basket_wire")
        for index in range(1, 8):
            y = _lerp(-side_y, side_y, index / 8.0)
            add(f"Basket_{panel}_Vertical_{index:02d}", (x_bottom, y, z_bottom), (x_top, y, 0.875), 0.0042, "ZincWire", "basket_wire")

    # Open lattice basket floor and lower parcel rack.
    for index in range(1, 8):
        fraction = index / 8.0
        y = _lerp(-0.245, 0.245, fraction)
        add(f"Basket_Floor_Long_{index:02d}", (-0.325, y, 0.505), (0.455, y, 0.455), 0.0042, "ZincWire", "basket_floor_wire")
    for index in range(1, 12):
        fraction = index / 12.0
        x = _lerp(-0.325, 0.455, fraction)
        z = _lerp(0.505, 0.455, fraction)
        add(f"Basket_Floor_Cross_{index:02d}", (x, -0.245, z), (x, 0.245, z), 0.0042, "ZincWire", "basket_floor_wire")
    for side, y in (("L", -0.205), ("R", 0.205)):
        add(f"Rack_Long_{side}", (-0.285, y, 0.285), (0.315, y, 0.255), 0.007, "ZincWire", "lower_rack")
    for index in range(7):
        fraction = index / 6.0
        x = _lerp(-0.285, 0.315, fraction)
        z = _lerp(0.285, 0.255, fraction)
        add(f"Rack_Cross_{index:02d}", (x, -0.205, z), (x, 0.205, z), 0.005, "ZincWire", "lower_rack")

    # Four independent swivel caster assemblies.  Wheels are cylinders along Y,
    # with visible hubs and two fork stays per caster.
    for longitudinal, x in (("Rear", -0.335), ("Front", 0.375)):
        for lateral, y in (("L", -0.258), ("R", 0.258)):
            prefix = f"Caster_{longitudinal}_{lateral}"
            add(f"{prefix}_Stem", (x, y, 0.132), (x, y, 0.205), 0.011, "Steel", "caster_stem")
            add(f"{prefix}_ForkA", (x, y - 0.023, 0.145), (x, y - 0.023, 0.068), 0.008, "Steel", "caster_fork")
            add(f"{prefix}_ForkB", (x, y + 0.023, 0.145), (x, y + 0.023, 0.068), 0.008, "Steel", "caster_fork")
            add(f"{prefix}_Wheel", (x, y - 0.019, 0.068), (x, y + 0.019, 0.068), 0.068, "Rubber", "wheel")
            add(f"{prefix}_Hub", (x, y - 0.023, 0.068), (x, y + 0.023, 0.068), 0.020, "Steel", "wheel_hub")

    names = [tube.name for tube in tubes]
    if len(names) != len(set(names)):
        raise AssertionError("tube names must be unique")
    return tuple(tubes)


COLLISION_BOXES: tuple[CollisionBox, ...] = (
    CollisionBox("FrameBaseLeft", (0.020, -0.252, 0.180), (0.802, 0.040, 0.060), "frame"),
    CollisionBox("FrameBaseRight", (0.020, 0.252, 0.180), (0.802, 0.040, 0.060), "frame"),
    CollisionBox("RearPostLeft", (-0.401, -0.252, 0.550), (0.060, 0.060, 0.750), "frame"),
    CollisionBox("RearPostRight", (-0.401, 0.252, 0.550), (0.060, 0.060, 0.750), "frame"),
    CollisionBox("BasketLeft", (0.073, -0.282, 0.668), (0.875, 0.026, 0.440), "basket_side"),
    CollisionBox("BasketRight", (0.073, 0.282, 0.668), (0.875, 0.026, 0.440), "basket_side"),
    CollisionBox("BasketFront", (0.478, 0.000, 0.665), (0.056, 0.580, 0.440), "basket_front"),
    CollisionBox("BasketRear", (-0.340, 0.000, 0.690), (0.050, 0.580, 0.400), "basket_rear"),
    CollisionBox("Handle", (-0.500, 0.000, 1.010), (0.036, 0.600, 0.036), "handle"),
    CollisionBox("CasterRearLeft", (-0.335, -0.258, 0.068), (0.140, 0.050, 0.136), "caster"),
    CollisionBox("CasterRearRight", (-0.335, 0.258, 0.068), (0.140, 0.050, 0.136), "caster"),
    CollisionBox("CasterFrontLeft", (0.375, -0.258, 0.068), (0.140, 0.050, 0.136), "caster"),
    CollisionBox("CasterFrontRight", (0.375, 0.258, 0.068), (0.140, 0.050, 0.136), "caster"),
)


MATERIALS = (
    ("Steel", (0.31, 0.33, 0.35), 0.88, 0.24),
    ("ZincWire", (0.48, 0.51, 0.54), 0.82, 0.29),
    ("HandlePolymer", (0.025, 0.105, 0.210), 0.00, 0.33),
    ("Rubber", (0.012, 0.014, 0.017), 0.00, 0.78),
)


def _orientation_from_z(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float, float]:
    vector = tuple(end[index] - start[index] for index in range(3))
    length = math.sqrt(sum(component * component for component in vector))
    direction = tuple(component / length for component in vector)
    dot = direction[2]
    if dot < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)
    scale = math.sqrt((1.0 + dot) * 2.0)
    return (scale * 0.5, -direction[1] / scale, direction[0] / scale, 0.0)


def _usd_material(name: str, color: tuple[float, float, float], metallic: float, roughness: float) -> str:
    return f'''        def Material "{name}"
        {{
            token outputs:surface.connect = </ShoppingCart/Looks/{name}/PreviewSurface.outputs:surface>
            def Shader "PreviewSurface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {_v3(color)}
                float inputs:metallic = {_f(metallic)}
                float inputs:roughness = {_f(roughness)}
                float inputs:clearcoat = 0.080000
                float inputs:clearcoatRoughness = 0.180000
                token outputs:surface
            }}
        }}'''


def _usd_tube(tube: Tube) -> str:
    midpoint = tuple((tube.start[index] + tube.end[index]) * 0.5 for index in range(3))
    length = math.sqrt(sum((tube.end[index] - tube.start[index]) ** 2 for index in range(3)))
    quaternion = _orientation_from_z(tube.start, tube.end)
    quat_text = f"({_f(quaternion[0])}, ({_f(quaternion[1])}, {_f(quaternion[2])}, {_f(quaternion[3])}))"
    return f'''            def Cylinder "{tube.name}"
            {{
                uniform token axis = "Z"
                double height = {_f(length)}
                double radius = {_f(tube.radius)}
                rel material:binding = </ShoppingCart/Looks/{tube.material}>
                custom string cart:role = "{tube.role}"
                quatf xformOp:orient = {quat_text}
                double3 xformOp:translate = {_v3(midpoint)}
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
            }}'''


def _usd_collision_box(box: CollisionBox) -> str:
    return f'''            def Cube "{box.name}"
            {{
                uniform token purpose = "guide"
                token visibility = "invisible"
                double size = 1.000000
                bool physics:collisionEnabled = 1
                token physics:approximation = "box"
                custom string cart:role = "{box.role}"
                custom double3 collision:size_m = {_v3(box.size)}
                double3 xformOp:scale = {_v3(box.size)}
                double3 xformOp:translate = {_v3(box.center)}
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            }}'''


def render_usda(tubes: tuple[Tube, ...]) -> str:
    material_text = "\n\n".join(_usd_material(*material) for material in MATERIALS)
    tube_text = "\n\n".join(_usd_tube(tube) for tube in tubes)
    collision_text = "\n\n".join(_usd_collision_box(box) for box in COLLISION_BOXES)
    return f'''#usda 1.0
(
    defaultPrim = "ShoppingCart"
    metersPerUnit = 1
    upAxis = "Z"
    customLayerData = {{
        string asset_identifier = "shopping_cart"
        string generator = "tools/generate_shopping_cart.py"
        string generator_version = "1"
    }}
)

def Xform "ShoppingCart" (
    kind = "component"
    customData = {{
        string asset_identifier = "shopping_cart"
        double3 dimensions_m = {_v3(ASSET_DIMENSIONS)}
        double3 obb_center_m = {_v3(OBB_CENTER)}
        double3 obb_half_extents_m = {_v3(OBB_HALF_EXTENTS)}
        string collision_metadata = "shopping_cart.metadata.json"
        bool independently_placeable = 1
        bool requires_human = 0
    }}
)
{{
    def Scope "Looks"
    {{
{material_text}
    }}

    def Xform "Visual"
    {{
{tube_text}
    }}

    def Xform "Collision"
    {{
        custom string collision:representation = "authored_box_proxies"
{collision_text}
    }}
}}
'''


def _project(point: tuple[float, float, float]) -> tuple[float, float]:
    x, y, z = point
    return (440.0 + 610.0 * x - 225.0 * y, 650.0 - 505.0 * z + 105.0 * y)


def render_svg(tubes: tuple[Tube, ...]) -> str:
    colors = {"Steel": "#8d969d", "ZincWire": "#b8c1c7", "HandlePolymer": "#17629f", "Rubber": "#20252a"}
    ordered = sorted(tubes, key=lambda tube: ((tube.start[1] + tube.end[1]) * 0.5, tube.radius), reverse=True)
    lines: list[str] = []
    for tube in ordered:
        x1, y1 = _project(tube.start)
        x2, y2 = _project(tube.end)
        width = max(1.2, tube.radius * 720.0)
        lines.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{colors[tube.material]}" stroke-width="{width:.2f}" stroke-linecap="round"/>'
        )
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="960" height="720" viewBox="0 0 960 720">
  <rect width="960" height="720" fill="#eef2f4"/>
  <path d="M80 650 L780 650 L915 585 L215 585 Z" fill="#dfe5e8" stroke="#c3ccd1"/>
  <g stroke="#cad2d6" stroke-width="1">
    <line x1="160" y1="650" x2="295" y2="585"/><line x1="300" y1="650" x2="435" y2="585"/>
    <line x1="440" y1="650" x2="575" y2="585"/><line x1="580" y1="650" x2="715" y2="585"/>
  </g>
  <g>{''.join(lines)}</g>
  <text x="42" y="52" font-family="Segoe UI, sans-serif" font-size="28" font-weight="600" fill="#17242d">Grocery shopping cart</text>
  <text x="42" y="82" font-family="Segoe UI, sans-serif" font-size="16" fill="#445761">CPU vector preview • 1.03 × 0.60 × 1.03 m • open wire basket</text>
  <g fill="none" stroke="#607783" stroke-width="1.5">
    <line x1="124" y1="678" x2="752" y2="678"/><line x1="124" y1="671" x2="124" y2="685"/><line x1="752" y1="671" x2="752" y2="685"/>
  </g>
  <text x="380" y="704" font-family="Segoe UI, sans-serif" font-size="14" fill="#607783">1.03 m overall length</text>
</svg>
'''


def build_metadata(tubes: tuple[Tube, ...]) -> dict[str, object]:
    role_counts: dict[str, int] = {}
    for tube in tubes:
        role_counts[tube.role] = role_counts.get(tube.role, 0) + 1
    return {
        "schema_version": 1,
        "asset_id": "shopping_cart",
        "asset_type": "store_context",
        "units": "meters",
        "coordinate_system": {"up_axis": "+Z", "forward_axis": "+X", "handle_side": "-X"},
        "dimensions_m": {"length_x": ASSET_DIMENSIONS[0], "width_y": ASSET_DIMENSIONS[1], "height_z": ASSET_DIMENSIONS[2]},
        "oriented_bounding_box": {
            "center_m": list(OBB_CENTER),
            "half_extents_m": list(OBB_HALF_EXTENTS),
            "axes": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "min_m": list(ASSET_BOUNDS_MIN),
            "max_m": list(ASSET_BOUNDS_MAX),
        },
        "footprint": {
            "aabb_xy_m": {"min": [ASSET_BOUNDS_MIN[0], ASSET_BOUNDS_MIN[1]], "max": [ASSET_BOUNDS_MAX[0], ASSET_BOUNDS_MAX[1]]},
            "corners_xy_m": [[-0.518, -0.300], [0.512, -0.300], [0.512, 0.300], [-0.518, 0.300]],
        },
        "placement": {
            "root_prim": "/ShoppingCart",
            "pivot_m": [0.0, 0.0, 0.0],
            "ground_plane_z_m": 0.0,
            "independently_placeable": True,
            "scene_placement": "unassigned",
            "requires_human": False,
            "corridor_membership": "unassigned; scene owner must place outside M1 if used as context",
        },
        "casters": {
            "count": 4,
            "wheel_radius_m": 0.068,
            "wheel_width_m": 0.038,
            "wheel_contact_z_m": 0.0,
            "centers_m": [[-0.335, -0.258, 0.068], [-0.335, 0.258, 0.068], [0.375, -0.258, 0.068], [0.375, 0.258, 0.068]],
        },
        "basket": {
            "construction": "open_wire_lattice",
            "open_top": True,
            "opaque_shell": False,
            "usable_interior_aabb_m": {"min": [-0.300, -0.235, 0.525], "max": [0.420, 0.235, 0.845]},
            "top_opening_corners_m": [[-0.355, -0.282, 0.875], [0.500, -0.282, 0.875], [0.500, 0.282, 0.875], [-0.355, 0.282, 0.875]],
        },
        "geometry": {"visual_primitive_types": ["capped_cylinder"], "tube_count": len(tubes), "role_counts": role_counts},
        "materials": [
            {"name": name, "shader": "UsdPreviewSurface", "base_color": list(color), "metallic": metallic, "roughness": roughness}
            for name, color, metallic, roughness in MATERIALS
        ],
        "collision": {
            "representation": "authored_box_proxies",
            "purpose": "guide",
            "primitives": [
                {"name": box.name, "shape": "box", "center_m": list(box.center), "size_m": list(box.size), "role": box.role}
                for box in COLLISION_BOXES
            ],
        },
        "visual_targets": [
            {"path": "references/storyboard/02_walk_forward_rgb.png", "sha256": "1dd93c320ce31c79eaa1a40c7e3b96507656ae59ab28eadec7d94e089cfc77d7"},
            {"path": "references/storyboard/03_explore_shelves_rgb.png", "sha256": "09154a4f078b9d625c4cb80bd3a9e46397c49159338cdfba98929eab2dd39297"},
        ],
        "provenance": {"method": "procedural original geometry", "license": "project-authored", "generator": "tools/generate_shopping_cart.py"},
    }


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n").encode("utf-8")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def generate(output_dir: Path) -> dict[str, object]:
    tubes = build_tubes()
    files = {
        ".gitattributes": b"* -text\n",
        "shopping_cart.usda": render_usda(tubes).encode("utf-8"),
        "shopping_cart.metadata.json": _json_bytes(build_metadata(tubes)),
        "shopping_cart_preview.svg": render_svg(tubes).encode("utf-8"),
    }
    # Git may expose Python sources with CRLF on Windows.  Hash the canonical
    # source text so the same generator commit has one identity on every host.
    generator_bytes = Path(__file__).read_bytes().replace(b"\r\n", b"\n")
    manifest = {
        "schema_version": 1,
        "asset_id": "shopping_cart",
        "generator": {"path": "tools/generate_shopping_cart.py", "sha256": _sha256(generator_bytes), "hash_basis": "UTF-8 with LF line endings", "runtime": "Python >= 3.10, standard library only"},
        "files": [
            {"path": name, "sha256": _sha256(data), "bytes": len(data), "role": {".gitattributes": "preserve_generated_bytes", "shopping_cart.usda": "render_asset", "shopping_cart.metadata.json": "geometry_and_collision_contract", "shopping_cart_preview.svg": "cpu_visual_preview"}[name]}
            for name, data in sorted(files.items())
        ],
        "source_references": [
            {"path": "references/storyboard/02_walk_forward_rgb.png", "sha256": "1dd93c320ce31c79eaa1a40c7e3b96507656ae59ab28eadec7d94e089cfc77d7"},
            {"path": "references/storyboard/03_explore_shelves_rgb.png", "sha256": "09154a4f078b9d625c4cb80bd3a9e46397c49159338cdfba98929eab2dd39297"},
        ],
    }
    files["manifest.json"] = _json_bytes(manifest)
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, data in sorted(files.items()):
        (output_dir / name).write_bytes(data)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    manifest = generate(args.output_dir.resolve())
    print(json.dumps({"asset_id": manifest["asset_id"], "output_dir": str(args.output_dir.resolve()), "files": len(manifest["files"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
