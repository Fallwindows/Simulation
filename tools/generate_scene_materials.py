"""Generate deterministic local micro-surface maps for the grocery shell."""

from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


def main() -> None:
    target = Path(__file__).resolve().parents[1] / "assets" / "scene" / "materials"
    target.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20260925)

    height_source = rng.integers(0, 256, (128, 128), dtype=np.uint8)
    height = np.asarray(
        Image.fromarray(height_source, mode="L").filter(ImageFilter.GaussianBlur(2.2)),
        dtype=np.float32,
    ) / 255.0
    gradient_y, gradient_x = np.gradient(height)
    normal_x = -gradient_x * 0.12
    normal_y = -gradient_y * 0.12
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
    Image.fromarray(normal, "RGB").save(target / "micro_normal.png", optimize=True)

    roughness = rng.integers(118, 190, (128, 128), dtype=np.uint8)
    roughness = np.asarray(Image.fromarray(roughness, "L").filter(ImageFilter.GaussianBlur(3.0)))
    Image.fromarray(roughness, "L").save(target / "micro_roughness.png", optimize=True)


if __name__ == "__main__":
    main()
