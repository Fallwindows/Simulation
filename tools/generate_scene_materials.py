"""Generate deterministic, tileable PBR maps for the grocery store shell.

The generator uses only local NumPy/Pillow operations and a fixed seed.
``--check`` rebuilds every declared output in memory and compares the encoded
PNG bytes and canonical manifest content with committed assets.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


SEED = 20260925
SIZE = 512
GENERATOR_VERSION = 2
MATERIAL_ROOT = Path(__file__).resolve().parents[1] / "assets" / "scene" / "materials"

MATERIAL_SETS = {
    "shelf_powdercoat": {
        "albedo": "powdercoat_albedo.png",
        "albedo_warm": "powdercoat_warm_albedo.png",
        "albedo_cool": "powdercoat_cool_albedo.png",
        "normal": "micro_normal.png",
        "roughness": "micro_roughness.png",
        "intent": "powder-coated shelf decks, lips, and uprights",
    },
    "floor_terrazzo": {
        "albedo": "terrazzo_albedo.png",
        "normal": "terrazzo_normal.png",
        "roughness": "terrazzo_roughness.png",
        "wear_mask": "floor_wear_mask.png",
        "intent": "polished terrazzo or resilient aggregate aisle flooring",
    },
    "display_laminate": {
        "albedo": "laminate_albedo.png",
        "normal": "laminate_normal.png",
        "roughness": "laminate_roughness.png",
        "intent": "wood-look feature display laminate",
    },
    "case_glass": {
        "normal": "case_glass_normal.png",
        "roughness": "case_glass_roughness.png",
        "intent": "refrigerated case glazing micro-surface; color and opacity remain shader parameters",
    },
    "case_frame": {
        "albedo": "case_frame_albedo.png",
        "normal": "case_frame_normal.png",
        "roughness": "case_frame_roughness.png",
        "intent": "dark anodized or powder-coated refrigerated-case frames",
    },
    "price_rail": {
        "albedo": "price_rail_albedo.png",
        "normal": "price_rail_normal.png",
        "roughness": "price_rail_roughness.png",
        "intent": "satin extruded shelf-edge ticket rail",
    },
    "category_detail": {
        "albedo": "category_detail_albedo.png",
        "normal": "category_detail_normal.png",
        "roughness": "category_detail_roughness.png",
        "intent": "printed category-panel stock with restrained fiber and ink variation",
    },
}


def _normalize(field: np.ndarray) -> np.ndarray:
    field = field.astype(np.float32)
    span = float(field.max() - field.min())
    if span <= 1e-8:
        return np.zeros_like(field)
    return (field - field.min()) / span


def _periodic_blur(field: np.ndarray, sigma_px: float) -> np.ndarray:
    """Gaussian-filter a 2D field with wrap boundaries using its Fourier basis."""

    frequencies = np.fft.fftfreq(field.shape[0])
    fy, fx = np.meshgrid(frequencies, frequencies, indexing="ij")
    transfer = np.exp(-2.0 * np.pi**2 * sigma_px**2 * (fx * fx + fy * fy))
    return np.fft.ifft2(np.fft.fft2(field) * transfer).real.astype(np.float32)


def _noise(rng: np.random.Generator, sigma_px: float) -> np.ndarray:
    return _normalize(_periodic_blur(rng.normal(size=(SIZE, SIZE)), sigma_px))


def _normal_from_height(height: np.ndarray, strength: float) -> Image.Image:
    """Encode an OpenGL-style tangent normal from a periodic unit height field."""

    height = height.astype(np.float32)
    gradient_x = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
    gradient_y = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
    normal_x = -gradient_x * strength
    normal_y = -gradient_y * strength
    normal_z = np.ones_like(normal_x)
    magnitude = np.sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z)
    normal = np.stack(
        (
            (normal_x / magnitude * 0.5 + 0.5) * 255.0,
            (normal_y / magnitude * 0.5 + 0.5) * 255.0,
            (normal_z / magnitude * 0.5 + 0.5) * 255.0,
        ),
        axis=-1,
    ).clip(0.0, 255.0).astype(np.uint8)
    return Image.fromarray(normal, "RGB")


def _rgb(values: np.ndarray) -> Image.Image:
    return Image.fromarray(np.clip(np.rint(values), 0, 255).astype(np.uint8), "RGB")


def _gray(values: np.ndarray) -> Image.Image:
    return Image.fromarray(np.clip(np.rint(values), 0, 255).astype(np.uint8), "L")


def _png_bytes(image: Image.Image) -> bytes:
    encoded = io.BytesIO()
    image.save(encoded, format="PNG", optimize=False, compress_level=9)
    return encoded.getvalue()


def _powdercoat(rng: np.random.Generator) -> dict[str, Image.Image]:
    broad = _noise(rng, 32.0) - 0.5
    orange_peel = _noise(rng, 1.15) - 0.5
    fine = _noise(rng, 0.35) - 0.5
    abrasion = _noise(rng, 13.0)
    rubbed = np.clip((abrasion - 0.64) / 0.24, 0.0, 1.0)
    scratch_seed = rng.random((SIZE, SIZE))
    scratches = _periodic_blur((scratch_seed > 0.9972).astype(np.float32), 0.55)
    scratches = _normalize(scratches) ** 1.8

    height = 0.68 * orange_peel + 0.20 * fine - 0.10 * scratches
    base = np.array((112.0, 116.0, 119.0), dtype=np.float32)
    albedo = base + broad[..., None] * 7.0 + orange_peel[..., None] * 2.2
    albedo += rubbed[..., None] * 4.0 + scratches[..., None] * 8.0
    roughness = 151.0 + orange_peel * 13.0 + broad * 8.0 - rubbed * 25.0 - scratches * 13.0

    return {
        "powdercoat_albedo.png": _rgb(albedo),
        "powdercoat_warm_albedo.png": _rgb(albedo + np.array((6.0, 1.0, -5.0))),
        "powdercoat_cool_albedo.png": _rgb(albedo + np.array((-6.0, 0.0, 7.0))),
        "micro_normal.png": _normal_from_height(height, 1.75),
        "micro_roughness.png": _gray(roughness),
    }


def _floor(rng: np.random.Generator) -> dict[str, Image.Image]:
    broad = _noise(rng, 46.0) - 0.5
    medium = _noise(rng, 12.0) - 0.5
    aggregate_seed = rng.random((SIZE, SIZE))
    aggregate = _normalize(_periodic_blur((aggregate_seed > 0.982).astype(np.float32), 0.8))
    light_stones = aggregate > 0.54
    dark_stones = (_noise(rng, 0.75) > 0.69) & ~light_stones
    warm_stones = (_noise(rng, 1.05) > 0.74) & ~light_stones & ~dark_stones

    broad_wear = _noise(rng, 52.0)
    local_wear = _noise(rng, 17.0)
    wear = np.clip(0.13 + broad_wear * 0.55 + local_wear * 0.22, 0.0, 1.0)
    scuffs = _periodic_blur((rng.random((SIZE, SIZE)) > 0.9985).astype(np.float32), 0.45)
    scuffs = _normalize(scuffs) ** 2.4

    floor = np.empty((SIZE, SIZE, 3), dtype=np.float32)
    floor[:] = (164.0, 157.0, 146.0)
    floor += broad[..., None] * np.array((12.0, 10.0, 8.0))
    floor += medium[..., None] * 3.5
    floor[light_stones] = (198.0, 193.0, 181.0)
    floor[dark_stones] = (91.0, 88.0, 82.0)
    floor[warm_stones] = (142.0, 122.0, 103.0)
    floor += wear[..., None] * 3.0
    floor -= scuffs[..., None] * 8.0

    stone_height = aggregate * 0.42 + light_stones.astype(np.float32) * 0.08
    height = 0.22 * medium + stone_height - scuffs * 0.16
    roughness = 170.0 + medium * 12.0 - wear * 33.0 + scuffs * 19.0
    wear_mask = wear * 224.0 + scuffs * 31.0
    return {
        "terrazzo_albedo.png": _rgb(floor),
        "terrazzo_normal.png": _normal_from_height(height, 0.95),
        "terrazzo_roughness.png": _gray(roughness),
        "floor_wear_mask.png": _gray(wear_mask),
    }


def _laminate(rng: np.random.Generator) -> dict[str, Image.Image]:
    axis = np.linspace(0.0, 2.0 * np.pi, SIZE, endpoint=False, dtype=np.float32)
    xx, yy = np.meshgrid(axis, axis, indexing="xy")
    warp = 0.11 * np.sin(yy * 2.0) + 0.04 * np.sin(yy * 7.0)
    grain = (
        0.55 * np.sin(xx * 10.0 + warp)
        + 0.24 * np.sin(xx * 23.0 + 0.7 * np.sin(yy * 3.0))
        + 0.12 * np.sin(xx * 47.0)
    )
    pores = _noise(rng, 0.45) - 0.5
    broad = _noise(rng, 20.0) - 0.5
    height = grain * 0.20 + pores * 0.10
    wood = np.empty((SIZE, SIZE, 3), dtype=np.float32)
    wood[..., 0] = 104.0 + grain * 20.0 + broad * 9.0
    wood[..., 1] = 61.0 + grain * 11.0 + broad * 5.0
    wood[..., 2] = 34.0 + grain * 6.0 + broad * 3.0
    roughness = 116.0 + grain * 8.0 + pores * 7.0
    return {
        "laminate_albedo.png": _rgb(wood),
        "laminate_normal.png": _normal_from_height(height, 1.15),
        "laminate_roughness.png": _gray(roughness),
    }


def _case_glass(rng: np.random.Generator) -> dict[str, Image.Image]:
    axis = np.linspace(0.0, 2.0 * np.pi, SIZE, endpoint=False, dtype=np.float32)
    xx, yy = np.meshgrid(axis, axis, indexing="xy")
    wipe = 0.34 * np.sin(xx * 3.0 + 0.28 * np.sin(yy * 2.0))
    wipe += 0.15 * np.sin(xx * 11.0 + yy * 0.5)
    haze = _noise(rng, 28.0) - 0.5
    height = wipe * 0.035 + haze * 0.018
    roughness = 18.0 + (_noise(rng, 18.0) ** 2.2) * 21.0 + np.maximum(wipe, 0.0) * 5.0
    return {
        "case_glass_normal.png": _normal_from_height(height, 0.46),
        "case_glass_roughness.png": _gray(roughness),
    }


def _case_frame(rng: np.random.Generator) -> dict[str, Image.Image]:
    axis = np.linspace(0.0, 2.0 * np.pi, SIZE, endpoint=False, dtype=np.float32)
    vertical = np.broadcast_to(np.sin(axis[:, None] * 48.0), (SIZE, SIZE))
    fine = _noise(rng, 0.5) - 0.5
    broad = _noise(rng, 18.0) - 0.5
    height = vertical * 0.035 + fine * 0.12
    albedo = np.empty((SIZE, SIZE, 3), dtype=np.float32)
    albedo[:] = (42.0, 45.0, 47.0)
    albedo += (broad * 5.0 + vertical * 1.5)[..., None]
    roughness = 91.0 + fine * 12.0 + broad * 8.0
    return {
        "case_frame_albedo.png": _rgb(albedo),
        "case_frame_normal.png": _normal_from_height(height, 1.10),
        "case_frame_roughness.png": _gray(roughness),
    }


def _price_rail(rng: np.random.Generator) -> dict[str, Image.Image]:
    axis = np.linspace(0.0, 2.0 * np.pi, SIZE, endpoint=False, dtype=np.float32)
    extrusion = np.broadcast_to(
        0.45 * np.sin(axis[:, None] * 3.0) + 0.16 * np.sin(axis[:, None] * 7.0),
        (SIZE, SIZE),
    )
    fine = _noise(rng, 0.55) - 0.5
    handling = _noise(rng, 15.0)
    rubbed = np.clip((handling - 0.64) / 0.23, 0.0, 1.0)
    height = extrusion * 0.055 + fine * 0.08
    value = 31.0 + extrusion * 2.0 + fine * 2.5 + rubbed * 4.5
    albedo = np.stack((value * 0.93, value, value * 1.05), axis=-1)
    roughness = 103.0 + fine * 10.0 - rubbed * 20.0
    return {
        "price_rail_albedo.png": _rgb(albedo),
        "price_rail_normal.png": _normal_from_height(height, 1.25),
        "price_rail_roughness.png": _gray(roughness),
    }


def _category_detail(rng: np.random.Generator) -> dict[str, Image.Image]:
    fiber = _noise(rng, 0.6) - 0.5
    paper = _noise(rng, 8.0) - 0.5
    ink = _noise(rng, 2.0) - 0.5
    albedo = np.empty((SIZE, SIZE, 3), dtype=np.float32)
    albedo[:] = (22.0, 57.0, 70.0)
    albedo += paper[..., None] * np.array((5.0, 7.0, 8.0))
    albedo += ink[..., None] * 2.2
    height = fiber * 0.16 + paper * 0.06
    roughness = 139.0 + fiber * 12.0 + paper * 8.0
    return {
        "category_detail_albedo.png": _rgb(albedo),
        "category_detail_normal.png": _normal_from_height(height, 0.85),
        "category_detail_roughness.png": _gray(roughness),
    }


def _validate_images(images: dict[str, Image.Image]) -> None:
    texture_roles = {"albedo", "albedo_warm", "albedo_cool", "normal", "roughness", "wear_mask"}
    declared = {
        filename
        for material_set in MATERIAL_SETS.values()
        for key, filename in material_set.items()
        if key in texture_roles
    }
    if set(images) != declared:
        raise ValueError(
            "generated material set differs from declared bindings: "
            f"missing={sorted(declared - set(images))}, extra={sorted(set(images) - declared)}"
        )
    for filename, image in images.items():
        if image.size != (SIZE, SIZE):
            raise ValueError(f"{filename}: expected {SIZE}x{SIZE}, got {image.size}")
        role = filename.rsplit("_", 1)[-1].removesuffix(".png")
        if role in {"albedo", "normal"} and image.mode != "RGB":
            raise ValueError(f"{filename}: {role} maps must be RGB")
        if role in {"roughness", "mask"} and image.mode != "L":
            raise ValueError(f"{filename}: {role} maps must be single-channel raw data")
        pixels = np.asarray(image)
        if role == "normal":
            mean_z = float(pixels[..., 2].mean())
            if mean_z < 245.0:
                raise ValueError(f"{filename}: implausibly strong tangent normal (mean Z={mean_z:.2f})")
        if role == "roughness":
            low, high = int(pixels.min()), int(pixels.max())
            if low < 8 or high > 240 or high - low < 6:
                raise ValueError(f"{filename}: invalid or flat roughness range {low}..{high}")


def _make_preview(images: dict[str, Image.Image]) -> Image.Image:
    glass_swatch = Image.new("RGB", (SIZE, SIZE), (42, 58, 65))
    glass_draw = ImageDraw.Draw(glass_swatch)
    checker = 64
    for y in range(0, SIZE, checker):
        for x in range(0, SIZE, checker):
            if (x // checker + y // checker) % 2:
                glass_draw.rectangle((x, y, x + checker - 1, y + checker - 1), fill=(61, 75, 80))
    groups = [
        ("SHELF METAL", "powdercoat_albedo.png", "micro_normal.png", "micro_roughness.png"),
        ("FLOOR + WEAR", "terrazzo_albedo.png", "floor_wear_mask.png", "terrazzo_roughness.png"),
        ("CASE GLASS", glass_swatch, "case_glass_normal.png", "case_glass_roughness.png"),
        ("CASE FRAME", "case_frame_albedo.png", "case_frame_normal.png", "case_frame_roughness.png"),
        ("PRICE RAIL", "price_rail_albedo.png", "price_rail_normal.png", "price_rail_roughness.png"),
        ("CATEGORY", "category_detail_albedo.png", "category_detail_normal.png", "category_detail_roughness.png"),
        ("LAMINATE", "laminate_albedo.png", "laminate_normal.png", "laminate_roughness.png"),
    ]
    thumb = 192
    label_h = 26
    gutter = 8
    canvas = Image.new(
        "RGB",
        (gutter * 4 + thumb * 3, gutter * (len(groups) + 1) + (thumb + label_h) * len(groups)),
        (15, 18, 20),
    )
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    for row, (label, albedo, normal, roughness) in enumerate(groups):
        y = gutter + row * (thumb + label_h + gutter)
        draw.text((gutter, y + 6), label, fill=(225, 230, 232), font=font)
        for column, source in enumerate((albedo, normal, roughness)):
            image = source if isinstance(source, Image.Image) else images[source]
            tile = image.convert("RGB").resize((thumb, thumb), Image.Resampling.NEAREST)
            x = gutter + column * (thumb + gutter)
            canvas.paste(tile, (x, y + label_h))
    return canvas


def _build_outputs() -> tuple[dict[str, Image.Image], dict[str, bytes], dict[str, object], bytes]:
    rng = np.random.default_rng(SEED)
    images: dict[str, Image.Image] = {}
    builders = (
        _powdercoat,
        _floor,
        _laminate,
        _case_glass,
        _case_frame,
        _price_rail,
        _category_detail,
    )
    for builder in builders:
        result = builder(rng)
        overlap = images.keys() & result.keys()
        if overlap:
            raise RuntimeError(f"duplicate generated material paths: {sorted(overlap)}")
        images.update(result)
    _validate_images(images)
    images["material_preview.png"] = _make_preview(images)
    encoded = {filename: _png_bytes(image) for filename, image in sorted(images.items())}

    files: dict[str, object] = {}
    for filename, image in sorted(images.items()):
        pixels = np.asarray(image)
        if pixels.ndim == 3:
            extrema_min: list[int] | int = [
                int(v) for v in pixels.reshape(-1, pixels.shape[-1]).min(axis=0)
            ]
            extrema_max: list[int] | int = [
                int(v) for v in pixels.reshape(-1, pixels.shape[-1]).max(axis=0)
            ]
        else:
            extrema_min = int(pixels.min())
            extrema_max = int(pixels.max())
        files[filename] = {
            "sha256": hashlib.sha256(encoded[filename]).hexdigest(),
            "mode": image.mode,
            "width_px": image.width,
            "height_px": image.height,
            "channel_min": extrema_min,
            "channel_max": extrema_max,
        }
    manifest: dict[str, object] = {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "seed": SEED,
        "tileable": True,
        "normal_convention": "OpenGL tangent space (+X right, +Y up, +Z outward)",
        "color_space": {"albedo": "sRGB", "normal": "raw", "roughness": "raw", "mask": "raw"},
        "material_sets": MATERIAL_SETS,
        "files": files,
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    return images, encoded, manifest, manifest_bytes


def generate(target: Path) -> dict[str, object]:
    target.mkdir(parents=True, exist_ok=True)
    _, encoded, manifest, manifest_bytes = _build_outputs()
    for filename, content in encoded.items():
        (target / filename).write_bytes(content)
    (target / "manifest.json").write_bytes(manifest_bytes)
    return manifest


def check(target: Path) -> None:
    _, encoded, _, manifest_bytes = _build_outputs()
    missing = [name for name in [*sorted(encoded), "manifest.json"] if not (target / name).is_file()]
    if missing:
        raise SystemExit(f"missing committed material outputs: {missing}")
    mismatched = [
        name for name, content in sorted(encoded.items()) if (target / name).read_bytes() != content
    ]
    try:
        committed_manifest = json.loads((target / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(f"invalid committed material manifest: {error}") from error
    generated_manifest = json.loads(manifest_bytes.decode("utf-8"))
    if committed_manifest != generated_manifest:
        mismatched.append("manifest.json")
    if mismatched:
        raise SystemExit(f"material outputs differ from deterministic generator: {mismatched}")
    print(f"verified {len(encoded) + 1} deterministic material files in {target}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=Path, default=MATERIAL_ROOT)
    parser.add_argument("--check", action="store_true", help="rebuild in memory and compare committed bytes")
    args = parser.parse_args()
    if args.check:
        check(args.target.resolve())
    else:
        manifest = generate(args.target.resolve())
        hashes = {name: data["sha256"] for name, data in manifest["files"].items()}
        print(json.dumps(hashes, indent=2))


if __name__ == "__main__":
    main()
