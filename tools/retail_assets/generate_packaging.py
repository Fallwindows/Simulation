"""Generate the fictional retail asset library used by the Isaac runtime.

The runtime library is intentionally procedural and self-contained, but the
generated assets are authored as small product assemblies rather than colored
placeholder primitives: packages have chamfered shells and label panels,
bottles have shoulders/necks/caps, cans and jars have rims/lids, and produce is
shaped and shaded as a natural display object.  The same catalog is consumed by
the deterministic layout builder, so visual refinements never change semantic
placement decisions.
"""

from __future__ import annotations

import json
import math
import struct
import zlib
from dataclasses import dataclass, asdict
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover - the Isaac/Pixi runtime supplies Pillow
    Image = ImageDraw = ImageFont = None


@dataclass(frozen=True)
class AssetSpec:
    asset_key: str
    category: str
    dimensions_m: tuple[float, float, float]
    model_type: str
    color: tuple[float, float, float]
    accent: tuple[float, float, float]
    product_name: str


def _spec(key: str, category: str, dims: tuple[float, float, float], model: str, color: tuple[int, int, int], accent: tuple[int, int, int], name: str) -> AssetSpec:
    return AssetSpec(key, category, dims, model, tuple(v / 255.0 for v in color), tuple(v / 255.0 for v in accent), name)


ASSET_SPECS: tuple[AssetSpec, ...] = (
    _spec("cereal_sunrise", "cereal", (0.13, 0.10, 0.31), "box", (242, 168, 46), (39, 94, 155), "SUNRISE OATS"),
    _spec("cereal_harvest", "cereal", (0.13, 0.10, 0.31), "box", (205, 76, 53), (244, 208, 75), "HARVEST LOOP"),
    _spec("cereal_grain", "cereal", (0.13, 0.10, 0.30), "box", (116, 164, 77), (236, 224, 159), "GRAIN DAY"),
    _spec("cereal_berry", "cereal", (0.13, 0.10, 0.31), "box", (92, 87, 160), (245, 117, 131), "BERRY CLOUD"),
    _spec("cereal_honey", "cereal", (0.13, 0.10, 0.31), "box", (228, 153, 38), (108, 68, 31), "HONEY CRUNCH"),
    _spec("cereal_morning", "cereal", (0.13, 0.10, 0.30), "box", (57, 142, 158), (239, 225, 169), "MORNING MIX"),
    _spec("snack_cracker", "snacks", (0.13, 0.09, 0.24), "box", (224, 133, 48), (121, 49, 35), "CRISP STACK"),
    _spec("snack_pretzel", "snacks", (0.13, 0.09, 0.24), "box", (180, 83, 43), (245, 201, 80), "PRETZEL POP"),
    _spec("snack_chips", "snacks", (0.13, 0.09, 0.25), "box", (65, 132, 87), (246, 208, 74), "GARDEN CRUNCH"),
    _spec("snack_wafer", "snacks", (0.13, 0.09, 0.23), "box", (128, 76, 145), (246, 189, 219), "WAFER WAVE"),
    _spec("snack_popcorn", "snacks", (0.13, 0.09, 0.24), "box", (237, 209, 74), (49, 91, 158), "POPCORN FIELD"),
    _spec("juice_citrus", "juice", (0.13, 0.11, 0.28), "carton", (230, 122, 38), (246, 222, 112), "CITRUS PRESS"),
    _spec("juice_berry", "juice", (0.13, 0.11, 0.28), "carton", (114, 68, 142), (236, 158, 195), "BERRY POUR"),
    _spec("juice_green", "juice", (0.13, 0.11, 0.28), "carton", (64, 139, 94), (207, 232, 133), "GREEN GROVE"),
    _spec("juice_apple", "juice", (0.13, 0.11, 0.28), "carton", (193, 69, 54), (247, 219, 103), "APPLE PRESS"),
    _spec("water_sky", "water", (0.10, 0.10, 0.28), "bottle", (72, 147, 206), (210, 239, 252), "SKY WATER"),
    _spec("water_clear", "water", (0.10, 0.10, 0.28), "bottle", (71, 174, 157), (223, 247, 235), "CLEAR SPRING"),
    _spec("soda_orbit", "soda", (0.10, 0.10, 0.27), "bottle", (54, 91, 177), (238, 220, 85), "ORBIT FIZZ"),
    _spec("soda_cherry", "soda", (0.10, 0.10, 0.27), "bottle", (186, 47, 57), (246, 185, 83), "CHERRY SPARK"),
    _spec("soda_lime", "soda", (0.10, 0.10, 0.27), "bottle", (93, 155, 62), (225, 240, 131), "LIME LIFT"),
    _spec("can_soup_red", "cans", (0.11, 0.11, 0.13), "can", (187, 55, 45), (246, 205, 91), "HEARTH SOUP"),
    _spec("can_soup_green", "cans", (0.11, 0.11, 0.13), "can", (67, 125, 77), (232, 218, 132), "HERB SOUP"),
    _spec("can_beans", "cans", (0.11, 0.11, 0.13), "can", (69, 94, 159), (239, 187, 81), "BLUE BEANS"),
    _spec("can_tomato", "cans", (0.11, 0.11, 0.13), "can", (208, 77, 46), (239, 236, 184), "TOMATO TABLE"),
    _spec("jar_honey", "jars", (0.12, 0.12, 0.18), "jar", (218, 157, 47), (244, 222, 119), "GOLDEN HIVE"),
    _spec("jar_pickle", "jars", (0.12, 0.12, 0.18), "jar", (74, 139, 86), (210, 235, 151), "DILL GARDEN"),
    _spec("jar_sauce", "jars", (0.12, 0.12, 0.18), "jar", (177, 64, 47), (244, 198, 90), "SAUCE HOUSE"),
    _spec("produce_crate_green", "produce_crate", (0.44, 0.32, 0.22), "crate", (110, 137, 68), (223, 175, 66), "FARM CRATE"),
    _spec("produce_crate_red", "produce_crate", (0.44, 0.32, 0.22), "crate", (159, 77, 55), (236, 194, 73), "MARKET CRATE"),
    # Grocery produce needs to read as produce at first-person distance, not
    # as sub-pixel decoration inside the bins.
    _spec("red_apple", "red_apples", (0.20, 0.20, 0.20), "fruit", (190, 46, 47), (242, 168, 70), "RED APPLE"),
    _spec("green_apple", "green_apples", (0.20, 0.20, 0.20), "fruit", (92, 159, 67), (236, 202, 75), "GREEN APPLE"),
    _spec("orange", "oranges", (0.21, 0.21, 0.21), "fruit", (230, 128, 35), (247, 213, 90), "ORANGE"),
    _spec("lemon", "lemons", (0.20, 0.20, 0.18), "fruit", (238, 205, 58), (110, 154, 67), "LEMON"),
    _spec("promo_market_sign", "promotional_sign", (0.92, 0.05, 0.48), "sign", (49, 105, 153), (243, 187, 62), "FRESH MARKET"),
)


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def _write_png(path: Path, width: int, height: int, pixels: list[list[tuple[int, int, int]]]) -> None:
    raw = b"".join(b"\x00" + bytes(channel for pixel in row for channel in pixel) for row in pixels)
    payload = b"\x89PNG\r\n\x1a\n"
    payload += _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    payload += _png_chunk(b"IDAT", zlib.compress(raw, 9))
    payload += _png_chunk(b"IEND", b"")
    path.write_bytes(payload)


def _blend(a: tuple[int, int, int], b: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return tuple(int(round(x * (1.0 - amount) + y * amount)) for x, y in zip(a, b))


def _font(size: int, bold: bool = False):
    if ImageFont is None:
        return None
    candidates = (
        r"C:\Windows\Fonts\segoeuib.ttf" if bold else r"C:\Windows\Fonts\segoeui.ttf",
        r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf",
    )
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _write_rich_texture(path: Path, spec: AssetSpec, size: int = 768) -> None:
    """Write a dense, readable fictional package face with no trademarked art."""
    if Image is None:
        _write_png(path, 256, 256, _fallback_texture(spec))
        return
    base = tuple(int(round(v * 255)) for v in spec.color)
    accent = tuple(int(round(v * 255)) for v in spec.accent)
    dark = _blend(base, (12, 18, 24), 0.62)
    light = _blend(base, (255, 255, 255), 0.28)
    image = Image.new("RGB", (size, size), base)
    draw = ImageDraw.Draw(image)
    # Directional print-like gradient and restrained paper grain.
    for y in range(size):
        t = y / max(1, size - 1)
        row = _blend(light, base, t)
        draw.line((0, y, size, y), fill=row)
    for x in range(0, size, 24):
        draw.line((x, 0, x + size // 4, size), fill=_blend(base, accent, 0.16), width=3)
    # Brand band and a contrasting product ribbon.
    band_h = int(size * 0.16)
    draw.rounded_rectangle((int(size * 0.055), int(size * 0.045), int(size * 0.945), band_h), radius=22, fill=dark)
    draw.text((int(size * 0.09), int(size * 0.068)), "NORTHSTAR MARKET", font=_font(int(size * 0.055), True), fill=(245, 244, 232))
    ribbon_y = int(size * 0.77)
    draw.rectangle((0, ribbon_y, size, int(size * 0.88)), fill=accent)
    draw.text((int(size * 0.08), ribbon_y + int(size * 0.025)), "SMALL-BATCH • EVERYDAY GOOD", font=_font(int(size * 0.026), True), fill=dark)
    # Product badge and category-specific graphic accents.
    cx, cy = int(size * 0.72), int(size * 0.42)
    radius = int(size * 0.155)
    draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill=accent, outline=(255, 255, 255), width=7)
    draw.ellipse((cx - int(radius * .55), cy - int(radius * .55), cx + int(radius * .55), cy + int(radius * .55)), outline=light, width=5)
    if spec.category in {"cereal", "snacks"}:
        for dx, dy, rr in ((-70, 40, 25), (-18, 58, 31), (42, 42, 22), (84, 62, 28)):
            draw.ellipse((cx + dx - rr, cy + dy - rr, cx + dx + rr, cy + dy + rr), fill=_blend(accent, (255, 255, 255), .25), outline=dark, width=3)
    elif spec.category in {"juice", "water", "soda"}:
        draw.arc((cx - 80, cy - 82, cx + 80, cy + 82), 200, 340, fill=(255, 255, 255), width=12)
        draw.line((cx - 55, cy + 40, cx + 65, cy - 46), fill=(255, 255, 255), width=10)
    elif spec.category in {"cans", "jars"}:
        draw.rectangle((cx - 66, cy - 42, cx + 66, cy + 44), outline=(255, 255, 255), width=7)
        draw.line((cx - 42, cy, cx + 42, cy), fill=(255, 255, 255), width=8)
    else:
        draw.arc((cx - 76, cy - 76, cx + 76, cy + 76), 25, 315, fill=(255, 255, 255), width=10)
    # Product title hierarchy.
    title_font = _font(int(size * (0.095 if len(spec.product_name) < 13 else 0.073)), True)
    subtitle = {
        "cereal": "WHOLE GRAIN • BREAKFAST",
        "snacks": "CRISP • SHAREABLE",
        "juice": "FRUIT BLEND • NO FUSS",
        "water": "SPRING WATER • 500 ml",
        "soda": "SPARKLING BOTANICAL",
        "cans": "PANTRY CLASSIC",
        "jars": "SMALL BATCH • PANTRY",
        "red_apples": "ORCHARD SELECT",
        "green_apples": "ORCHARD SELECT",
        "oranges": "CITRUS SELECT",
        "lemons": "CITRUS SELECT",
        "produce_crate": "FARM STAND",
        "promotional_sign": "FRESH MARKET",
    }.get(spec.category, "QUALITY GROCERIES")
    draw.text((int(size * 0.08), int(size * 0.31)), spec.product_name, font=title_font, fill=(255, 255, 255), stroke_width=3, stroke_fill=dark)
    draw.text((int(size * 0.09), int(size * 0.56)), subtitle, font=_font(int(size * 0.033), True), fill=dark)
    # Side nutrition panel and real-looking machine-readable mark.
    panel_x = int(size * 0.08)
    panel_y = int(size * 0.62)
    draw.rounded_rectangle((panel_x, panel_y, int(size * 0.47), panel_y + int(size * 0.105)), radius=10, fill=_blend(base, (255, 255, 255), .72))
    for row in range(4):
        yy = panel_y + int(size * 0.018) + row * int(size * 0.021)
        draw.line((panel_x + int(size * .025), yy, panel_x + int(size * .34), yy), fill=dark, width=3 if row == 0 else 2)
    barcode_x = int(size * 0.73)
    barcode_y = int(size * 0.895)
    for index in range(22):
        width = 3 if index % 5 == 0 else 1 + (index % 3)
        draw.rectangle((barcode_x + index * 7, barcode_y, barcode_x + index * 7 + width, barcode_y + int(size * .075)), fill=dark)
    draw.text((int(size * 0.08), int(size * 0.91)), "FICTIONAL PRODUCT • 100% DEMO", font=_font(int(size * .022)), fill=dark)
    image.save(path, format="PNG", optimize=True)


def _fallback_texture(spec: AssetSpec, size: int = 256) -> list[list[tuple[int, int, int]]]:
    base = tuple(int(round(v * 255)) for v in spec.color)
    accent = tuple(int(round(v * 255)) for v in spec.accent)
    pixels = [[base for _ in range(size)] for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if y < 28 or y > size - 30:
                pixels[y][x] = accent
            elif (x // 24 + y // 24) % 2 == 0 and y > 55:
                pixels[y][x] = _blend(base, (255, 255, 255), .08)
    return pixels


def _material(name: str, color: tuple[float, float, float], texture_path: str | None = None, roughness: float = 0.58, metallic: float = 0.0) -> str:
    safe = name.replace("-", "_")
    lines = [
        f'        def Material "{safe}" {{',
        f'            token outputs:surface.connect = </Asset/Looks/{safe}/Preview.outputs:surface>',
    ]
    if texture_path:
        lines += [
            '            def Shader "StReader" {',
            '                uniform token info:id = "UsdPrimvarReader_float2"',
            '                token inputs:varname = "st"',
            '                float2 outputs:result',
            '            }',
            '            def Shader "Texture" {',
            '                uniform token info:id = "UsdUVTexture"',
            f'                asset inputs:file = @{texture_path}@',
            '                float2 inputs:st.connect = </Asset/Looks/' + safe + '/StReader.outputs:result>',
            '                token outputs:rgb',
            '            }',
        ]
    lines += [
        '            def Shader "Preview" {',
        '                uniform token info:id = "UsdPreviewSurface"',
        f'                color3f inputs:diffuseColor = ({color[0]:.5f}, {color[1]:.5f}, {color[2]:.5f})',
        f'                float inputs:roughness = {roughness:.4f}',
        f'                float inputs:metallic = {metallic:.4f}',
    ]
    if texture_path:
        lines.append(f'                color3f inputs:diffuseColor.connect = </Asset/Looks/{safe}/Texture.outputs:rgb>')
    lines += ['                token outputs:surface', '            }', '        }']
    return "\n".join(lines)


def _front_panel(width: float, depth: float, height: float) -> str:
    y = depth / 2.0 + 0.002
    w = width * 0.88 / 2.0
    h = height * 0.84 / 2.0
    return f'''        def Mesh "FrontPanel" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [(-{w:.5f}, {y:.5f}, -{h:.5f}), ({w:.5f}, {y:.5f}, -{h:.5f}), ({w:.5f}, {y:.5f}, {h:.5f}), (-{w:.5f}, {y:.5f}, {h:.5f})]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            normal3f[] normals = [(0, 1, 0)]
            uniform token[] normals:interpolation = ["uniform"]
            texCoord2f[] primvars:st = [(0, 0), (1, 0), (1, 1), (0, 1)] (
                interpolation = "vertex"
            )
            rel material:binding = </Asset/Looks/Front>
        }}'''


def _cube(name: str, size: tuple[float, float, float], translate: tuple[float, float, float], material: str) -> str:
    # Isaac's USD bounds/render path does not apply nonuniform xform ops
    # authored directly on a Gprim. Put the scale on an Xform parent so the
    # physical dimensions remain correct in both USD consumers.
    return f'''        def Xform "{name}" {{
            double3 xformOp:translate = ({translate[0]:.5f}, {translate[1]:.5f}, {translate[2]:.5f})
            double3 xformOp:scale = ({size[0] / 2.0:.5f}, {size[1] / 2.0:.5f}, {size[2] / 2.0:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            def Cube "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double size = 2.0
                rel material:binding = </Asset/Looks/{material}>
            }}
        }}'''


def _beveled_box(name: str, size: tuple[float, float, float], bevel: float, material: str) -> str:
    """Create a chamfered rectangular shell with stable, explicit dimensions."""
    width, depth, height = size
    half_w, half_d, half_h = width / 2.0, depth / 2.0, height / 2.0
    b = min(bevel, width * 0.18, depth * 0.18)
    ring = [
        (-half_w + b, -half_d), (half_w - b, -half_d),
        (half_w, -half_d + b), (half_w, half_d - b),
        (half_w - b, half_d), (-half_w + b, half_d),
        (-half_w, half_d - b), (-half_w, -half_d + b),
    ]
    points = [(x, y, -half_h) for x, y in ring] + [(x, y, half_h) for x, y in ring]
    faces = [[index for index in reversed(range(8))], list(range(8, 16))]
    for index in range(8):
        next_index = (index + 1) % 8
        faces.append([index, next_index, next_index + 8, index + 8])
    counts = ", ".join("8" if index < 2 else "4" for index in range(len(faces)))
    indices = ", ".join(str(vertex) for face in faces for vertex in face)
    point_text = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in points)
    return f'''        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            point3f[] points = [{point_text}]
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _cylinder(name: str, radius: float, height: float, material: str, translate_z: float = 0.0, vertices: int = 32) -> str:
    translate = f'''\n            double3 xformOp:translate = (0, 0, {translate_z:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate"]''' if abs(translate_z) > 1e-9 else ""
    return f'''        def Cylinder "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            int vertices = {vertices}
            double radius = {radius:.5f}
            double height = {height:.5f}{translate}
            rel material:binding = </Asset/Looks/{material}>
        }}'''


def _asset_usda(spec: AssetSpec, texture_name: str) -> str:
    width, depth, height = spec.dimensions_m
    body_material = "Body"
    front_material = "Front"
    texture_path = f"../textures/{texture_name}"
    geometry: list[str] = []
    if spec.model_type in {"box", "carton", "sign"}:
        geometry.append(_beveled_box("BodyShell", spec.dimensions_m, min(width, depth) * 0.075, body_material))
        geometry.append(_front_panel(width, depth, height))
        geometry.append(_cube("TopBand", (width * 0.86, 0.012, height * 0.10), (0.0, depth / 2.0 + 0.008, height * 0.34), front_material))
        geometry.append(_cube("BottomTrim", (width * 0.82, 0.010, height * 0.025), (0.0, depth / 2.0 + 0.009, -height * 0.40), "Metal"))
        if spec.model_type == "carton":
            geometry.append(_beveled_box("CartonTop", (width * 0.84, depth * 0.88, height * 0.075), min(width, depth) * 0.05, front_material))
            geometry.append(_cylinder("PourCap", width * 0.15, height * 0.055, "Metal", height * 0.505, vertices=24))
    elif spec.model_type in {"can", "jar"}:
        radius = min(width, depth) / 2.0
        geometry.append(_cylinder("Body", radius, height, body_material, vertices=40))
        if spec.model_type == "jar":
            geometry.append(_cylinder("Shoulder", radius * 0.92, height * 0.08, body_material, height * 0.39, vertices=40))
            geometry.append(_cylinder("Lid", radius * 0.90, height * 0.07, "Metal", height * 0.51, vertices=40))
            geometry.append(_cylinder("LidBand", radius * 0.94, height * 0.025, front_material, height * 0.475, vertices=40))
        else:
            geometry.append(_cylinder("TopRim", radius * 0.97, height * 0.035, "Metal", height * 0.515, vertices=40))
            geometry.append(_cylinder("BottomRim", radius * 0.97, height * 0.025, "Metal", -height * 0.515, vertices=40))
        geometry.append(_front_panel(width, depth, height * 0.65))
    elif spec.model_type == "bottle":
        radius = min(width, depth) / 2.0
        geometry.append(_cylinder("Body", radius * 0.91, height * 0.64, body_material, -height * 0.12, vertices=40))
        geometry.append(f'''        def Cone "Shoulder" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        ) {{
            int vertices = 40
            double radius = {radius * 0.91:.5f}
            double radius2 = {radius * 0.57:.5f}
            double height = {height * 0.095:.5f}
            double3 xformOp:translate = (0, 0, {height * 0.245:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
            rel material:binding = </Asset/Looks/{body_material}>
        }}''')
        geometry.append(_cylinder("Neck", radius * 0.57, height * 0.17, body_material, height * 0.36, vertices=32))
        geometry.append(_front_panel(width, depth, height * 0.42))
        geometry.append(_cylinder("Cap", radius * 0.60, height * 0.055, "Metal", height * 0.49, vertices=32))
        geometry.append(_cylinder("CapRidge", radius * 0.63, height * 0.014, "Metal", height * 0.46, vertices=32))
    elif spec.model_type == "fruit":
        geometry.append(f'''        def Xform "Fruit" {{
            double3 xformOp:scale = ({width:.5f}, {depth:.5f}, {height:.5f})
            uniform token[] xformOpOrder = ["xformOp:scale"]
            def Sphere "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double radius = 0.5
                rel material:binding = </Asset/Looks/{body_material}>
            }}
        }}''')
        geometry.append(_cylinder("Stem", width * 0.055, height * 0.20, "Stem", height * 0.56, vertices=12))
    elif spec.model_type == "crate":
        geometry.append(_cube("Base", spec.dimensions_m, (0.0, 0.0, 0.0), body_material))
        for index, x in enumerate((-width * 0.38, width * 0.38)):
            geometry.append(_cube(f"SlatX{index}", (width * 0.08, depth * 1.03, height * 1.25), (x, 0.0, 0.0), front_material))
        for index, z in enumerate((-height * 0.28, height * 0.28)):
            geometry.append(_cube(f"SlatZ{index}", (width * 1.03, depth * 0.08, height * 0.10), (0.0, 0.0, z), front_material))
        geometry.append(f'''        def Xform "Fruit" {{
            double3 xformOp:translate = (0, 0, {height * 0.66:.5f})
            double3 xformOp:scale = ({width * 0.23:.5f}, {depth * 0.23:.5f}, {height * 0.45:.5f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            def Sphere "Shape" (
                prepend apiSchemas = ["MaterialBindingAPI"]
            ) {{
                double radius = 0.5
                rel material:binding = </Asset/Looks/Front>
            }}
        }}''')
    else:
        raise ValueError(spec.model_type)
    return f'''#usda 1.0
(
    defaultPrim = "Asset"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Asset" (
    kind = "component"
) {{
    def Xform "Geometry" {{
{chr(10).join(geometry)}
    }}
    def Scope "Looks" {{
{_material(body_material, spec.color, None, 0.62, 0.0)}
{_material(front_material, spec.accent, texture_path, 0.54, 0.0)}
{_material("Metal", (0.48, 0.52, 0.55), None, 0.28, 0.7)}
{_material("Stem", (0.12, 0.23, 0.08), None, 0.72, 0.0)}
    }}
}}
'''


def generate_library(root: Path | None = None) -> Path:
    repo = root or Path(__file__).resolve().parents[2]
    asset_root = repo / "assets" / "retail"
    usd_root = asset_root / "usd"
    texture_root = asset_root / "textures"
    usd_root.mkdir(parents=True, exist_ok=True)
    texture_root.mkdir(parents=True, exist_ok=True)
    entries = []
    for spec in ASSET_SPECS:
        texture_name = f"{spec.asset_key}.png"
        _write_rich_texture(texture_root / texture_name, spec)
        (usd_root / f"{spec.asset_key}.usda").write_text(_asset_usda(spec, texture_name), encoding="utf-8")
        entries.append({
            "asset_key": spec.asset_key,
            "category": spec.category,
            # Paths are relative to this manifest.  Keeping them portable is
            # important because the same catalog is opened by Python, Isaac,
            # and an optional Blender source-authoring pass.
            "usd_path": f"usd/{spec.asset_key}.usda",
            "texture_path": f"textures/{texture_name}",
            "dimensions_m": list(spec.dimensions_m),
            "local_front_axis": "+Y",
            "model_type": spec.model_type,
            "product_name": spec.product_name,
        })
    manifest = {
        "schema_version": 1,
        "standard_front_axis": "+Y",
        "units": "meters",
        "assets": entries,
    }
    manifest_path = asset_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


if __name__ == "__main__":
    print(generate_library())
