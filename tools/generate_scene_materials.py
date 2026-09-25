"""Generate deterministic local surface maps for the grocery shell."""

from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


def _normal_from_height(height: np.ndarray, strength: float) -> Image.Image:
    gradient_y, gradient_x = np.gradient(height.astype(np.float32) / 255.0)
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


def main() -> None:
    target = Path(__file__).resolve().parents[1] / "assets" / "scene" / "materials"
    target.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20260925)

    height_source = rng.integers(0, 256, (256, 256), dtype=np.uint8)
    height = np.asarray(
        Image.fromarray(height_source, mode="L").filter(ImageFilter.GaussianBlur(2.2)),
        dtype=np.float32,
    ) / 255.0
    _normal_from_height((height * 255.0).astype(np.uint8), 0.22).save(target / "micro_normal.png", optimize=True)

    roughness = rng.integers(118, 190, (256, 256), dtype=np.uint8)
    roughness = np.asarray(Image.fromarray(roughness, "L").filter(ImageFilter.GaussianBlur(3.0)))
    Image.fromarray(roughness, "L").save(target / "micro_roughness.png", optimize=True)

    # Fine powder coat: visible orange-peel roughness plus a few restrained
    # rubbed edges.  The map is neutral enough to work with all shelf tints.
    coat_noise = rng.normal(0.0, 5.5, (256, 256, 1))
    coat = np.full((256, 256, 3), (150, 154, 157), dtype=np.float32) + coat_noise
    for row in (38, 151, 218):
        coat[row : row + 1, 12:244] += 16.0
    for _ in range(34):
        y = int(rng.integers(0, 256))
        x = int(rng.integers(0, 248))
        length = int(rng.integers(3, 18))
        coat[y : y + 1, x : x + length] += rng.uniform(9.0, 24.0)
    coat = coat.clip(0, 255)
    Image.fromarray(coat.astype(np.uint8), "RGB").save(target / "powdercoat_albedo.png", optimize=True)
    warm_coat = coat + np.array((8.0, -1.0, -10.0), dtype=np.float32)
    cool_coat = coat + np.array((-10.0, 0.0, 9.0), dtype=np.float32)
    Image.fromarray(warm_coat.clip(0, 255).astype(np.uint8), "RGB").save(
        target / "powdercoat_warm_albedo.png", optimize=True
    )
    Image.fromarray(cool_coat.clip(0, 255).astype(np.uint8), "RGB").save(
        target / "powdercoat_cool_albedo.png", optimize=True
    )

    # Terrazzo-like vinyl floor with embedded aggregate and broad mottling.
    floor = np.full((256, 256, 3), (158, 151, 140), dtype=np.float32)
    low = rng.integers(0, 256, (32, 32), dtype=np.uint8)
    low = np.asarray(Image.fromarray(low, "L").resize((256, 256), Image.Resampling.BICUBIC), dtype=np.float32)
    floor += ((low[..., None] - 127.5) / 22.0)
    for _ in range(1450):
        y = int(rng.integers(0, 256))
        x = int(rng.integers(0, 256))
        tone = rng.choice(((78, 72, 65), (205, 198, 184), (135, 124, 108)))
        floor[y : y + 1 + int(rng.random() > 0.78), x : x + 1 + int(rng.random() > 0.78)] = tone
    floor_img = Image.fromarray(floor.clip(0, 255).astype(np.uint8), "RGB")
    floor_img.save(target / "terrazzo_albedo.png", optimize=True)
    floor_height = np.asarray(floor_img.convert("L").filter(ImageFilter.GaussianBlur(0.45)))
    _normal_from_height(floor_height, 0.42).save(target / "terrazzo_normal.png", optimize=True)
    floor_roughness = np.asarray(floor_img.convert("L"), dtype=np.float32)
    floor_roughness = np.clip(168.0 - (floor_roughness - floor_roughness.mean()) * 0.38, 120, 205).astype(np.uint8)
    Image.fromarray(floor_roughness, "L").save(target / "terrazzo_roughness.png", optimize=True)

    # Printed laminate grain gives the feature display a material identity
    # distinct from the powder-coated steel around it.
    wood = np.zeros((256, 256, 3), dtype=np.float32)
    x = np.arange(256, dtype=np.float32)[None, :]
    grain = np.broadcast_to(10.0 * np.sin(x / 7.8) + 4.0 * np.sin(x / 2.7), (256, 256)).copy()
    grain += rng.normal(0.0, 2.2, (256, 256))
    wood[..., 0] = 98.0 + grain
    wood[..., 1] = 57.0 + grain * 0.55
    wood[..., 2] = 28.0 + grain * 0.25
    Image.fromarray(wood.clip(0, 255).astype(np.uint8), "RGB").save(target / "laminate_albedo.png", optimize=True)


if __name__ == "__main__":
    main()
