"""Generate deterministic, SKU-bound electronic shelf labels for hero displays.

The retail catalog keeps one generic ``price_display`` fixture so its 79-key
contract remains stable.  These scene assets reuse that fixture's measured USD
envelope while replacing only its front texture and attaching explicit SKU and
price metadata.  ``--check`` rebuilds every output in memory and compares exact
bytes with the committed scene assets.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator.runtime.isaac_sim_runner import HERO_PRICE_LABELS
from tools.retail_assets.generate_packaging import (
    REQUIRED_FONTS,
    REQUIRED_PILLOW_VERSION,
    REQUIRED_ZLIB_VERSION,
    _font,
    _validate_texture_toolchain,
)


OUTPUT_ROOT = ROOT / "assets" / "scene" / "price_displays"
BASE_USD = ROOT / "assets" / "retail" / "usd" / "price_display.usda"
GENERATOR_VERSION = 1
TEXTURE_SIZE = (768, 256)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _png_bytes(sku_key: str, display_name: str, unit_price: str) -> bytes:
    width, height = TEXTURE_SIZE
    seed = hashlib.sha256(sku_key.encode("utf-8")).digest()
    accent = (70 + seed[0] % 65, 92 + seed[1] % 55, 88 + seed[2] % 70)
    paper = (232, 235, 222)
    ink = (20, 25, 28)
    image = Image.new("RGB", TEXTURE_SIZE, paper)
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle((5, 5, width - 6, height - 6), radius=18, outline=ink, width=8)
    draw.rounded_rectangle((18, 18, 438, height - 19), radius=11, fill=(242, 244, 235))
    draw.rectangle((18, 18, 438, 42), fill=accent)
    draw.line((454, 20, 454, height - 20), fill=(82, 89, 88), width=4)

    words = display_name.split()
    if len(words) > 1:
        first_line = " ".join(words[:-1])
        second_line = words[-1]
    else:
        first_line, second_line = display_name, ""
    draw.text((35, 60), first_line, font=_font(48, True), fill=ink)
    if second_line:
        draw.text((35, 116), second_line, font=_font(55, True), fill=ink)
    draw.text((35, 208), sku_key.upper(), font=_font(18), fill=(64, 70, 70))

    draw.text((610, 84), unit_price, font=_font(78, True), fill=ink, anchor="mm")
    draw.text((610, 168), "EACH", font=_font(28, True), fill=(48, 54, 55), anchor="mm")
    draw.rectangle((502, 205, 718, 222), fill=accent)

    encoded = io.BytesIO()
    image.save(encoded, format="PNG", optimize=False, compress_level=9)
    return encoded.getvalue()


def _usd_bytes(template: str, sku_key: str, display_name: str, unit_price: str) -> bytes:
    template = template.replace("\r\n", "\n")
    texture_name = f"price_{sku_key}.png"
    source = template.replace("@../textures/price_display.png@", f"@{texture_name}@")
    source = source.replace(
        "@../textures/price_display_normal.png@",
        "@../../retail/textures/price_display_normal.png@",
    )
    source = source.replace(
        "@../textures/price_display_roughness.png@",
        "@../../retail/textures/price_display_roughness.png@",
    )
    marker = 'def Xform "Asset" (\n    kind = "component"\n) {\n'
    metadata = (
        marker
        + f'    string grocery:sku_key = "{sku_key}"\n'
        + f'    string grocery:display_name = "{display_name}"\n'
        + f'    string grocery:unit_price = "{unit_price}"\n'
    )
    if marker not in source:
        raise RuntimeError(f"Price-display template root marker changed: {BASE_USD}")
    return source.replace(marker, metadata, 1).encode("utf-8")


def _build_outputs() -> tuple[dict[str, bytes], dict[str, object], bytes]:
    template_bytes = BASE_USD.read_bytes()
    template = template_bytes.decode("utf-8")
    outputs: dict[str, bytes] = {}
    labels: dict[str, object] = {}
    for sku_key, label in HERO_PRICE_LABELS.items():
        png_name = f"price_{sku_key}.png"
        usd_name = f"price_{sku_key}.usda"
        png = _png_bytes(sku_key, label["display_name"], label["unit_price"])
        usd = _usd_bytes(template, sku_key, label["display_name"], label["unit_price"])
        outputs[png_name] = png
        outputs[usd_name] = usd
        labels[sku_key] = {
            "display_name": label["display_name"],
            "unit_price": label["unit_price"],
            "texture": png_name,
            "texture_sha256": _sha256(png),
            "usd": usd_name,
            "usd_sha256": _sha256(usd),
        }
    manifest: dict[str, object] = {
        "schema": "grocery.scene_price_displays",
        "version": 1,
        "generator_version": GENERATOR_VERSION,
        "texture_size_px": list(TEXTURE_SIZE),
        "fixture_dimensions_m": [0.26, 0.018, 0.065],
        "source_template": str(BASE_USD.relative_to(ROOT)).replace("\\", "/"),
        "source_template_sha256": _sha256(template_bytes),
        "toolchain": {
            "pillow": REQUIRED_PILLOW_VERSION,
            "zlib": REQUIRED_ZLIB_VERSION,
            "fonts": {
                str(path): digest for path, digest in REQUIRED_FONTS.values()
            },
        },
        "labels": labels,
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    return outputs, manifest, manifest_bytes


def _check(outputs: dict[str, bytes], manifest_bytes: bytes) -> None:
    expected = {**outputs, "manifest.json": manifest_bytes}
    actual = {
        path.name: path.read_bytes()
        for path in OUTPUT_ROOT.iterdir()
        if path.is_file()
    } if OUTPUT_ROOT.is_dir() else {}
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    changed = sorted(name for name in set(expected) & set(actual) if expected[name] != actual[name])
    if missing or extra or changed:
        raise RuntimeError(
            f"Scene price displays are stale; missing={missing}, extra={extra}, changed={changed}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    _validate_texture_toolchain()
    outputs, _, manifest_bytes = _build_outputs()
    if args.check:
        _check(outputs, manifest_bytes)
        print(f"validated {len(outputs)} deterministic scene price-display files")
        return 0
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    for path in OUTPUT_ROOT.iterdir():
        if path.is_file():
            path.unlink()
    for name, data in outputs.items():
        (OUTPUT_ROOT / name).write_bytes(data)
    (OUTPUT_ROOT / "manifest.json").write_bytes(manifest_bytes)
    print(f"wrote {len(outputs)} deterministic scene price-display files to {OUTPUT_ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
