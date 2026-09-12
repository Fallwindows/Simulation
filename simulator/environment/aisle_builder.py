"""Deterministic, Isaac-independent grocery aisle geometry."""

from __future__ import annotations

import random
from dataclasses import dataclass

from simulator.config.loader import AisleConfig


@dataclass(frozen=True)
class Box:
    name: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    kind: str


@dataclass(frozen=True)
class AisleLayout:
    primitives: tuple[Box, ...]
    seed: int

    @property
    def products(self) -> tuple[Box, ...]:
        return tuple(p for p in self.primitives if p.kind == "product")


def build_aisle_layout(config: AisleConfig) -> AisleLayout:
    """Create a repeatable floor, shelf, and product-proxy layout.

    Geometry is expressed as boxes so it can be unit-tested without Isaac Sim
    and translated to USD primitives by the optional runtime adapter.
    """
    rng = random.Random(config.seed)
    primitives: list[Box] = []
    floor_z = config.floor_z_m - 0.05
    primitives.append(Box("floor", (config.length_m / 2.0, 0.0, floor_z), (config.length_m + 2.0, config.width_m + 2.0, 0.1), "floor"))
    bay_count = max(1, int(config.length_m // config.bay_width_m))
    x0 = config.bay_width_m / 2.0
    level_spacing = config.shelf_height_m / config.shelf_levels
    for row_index, row_y in enumerate(config.shelf_rows_y_m):
        for bay in range(bay_count):
            x = x0 + bay * config.bay_width_m
            for level in range(config.shelf_levels):
                z = config.floor_z_m + (level + 1) * level_spacing
                primitives.append(Box(f"shelf_r{row_index}_b{bay}_l{level}", (x, row_y, z), (config.bay_width_m * 0.98, config.shelf_depth_m, 0.06), "shelf"))
                slot_count = 4
                for slot in range(slot_count):
                    if rng.random() > config.product_density:
                        continue
                    px = x - config.bay_width_m * 0.36 + (slot + 0.5) * config.bay_width_m * 0.18
                    py = row_y + (rng.random() - 0.5) * max(0.02, config.shelf_depth_m * 0.35)
                    pz = z + 0.17
                    scale = 0.16 + rng.random() * 0.08
                    primitives.append(Box(f"product_r{row_index}_b{bay}_l{level}_s{slot}", (px, py, pz), (scale, scale * 0.75, scale * 1.5), "product"))
            for side in (-1.0, 1.0):
                y = row_y + side * config.shelf_depth_m * 0.42
                primitives.append(Box(f"upright_r{row_index}_b{bay}_{int(side)}", (x, y, config.floor_z_m + config.shelf_height_m / 2.0), (0.08, 0.08, config.shelf_height_m), "upright"))
    return AisleLayout(tuple(primitives), config.seed)
