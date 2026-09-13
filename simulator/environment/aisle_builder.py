"""Deterministic grocery aisle geometry with asset-backed product slots."""

from __future__ import annotations

import hashlib
import random
from dataclasses import dataclass

from simulator.config.loader import AisleConfig
from simulator.environment.retail_catalog import RetailAsset, RetailAssetCatalog, load_retail_catalog


@dataclass(frozen=True)
class Box:
    name: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    kind: str


@dataclass(frozen=True)
class AssetInstance:
    """One reusable retail asset occupying an original product slot."""

    name: str
    asset_key: str
    category: str
    position_m: tuple[float, float, float]
    rotation_rpy_deg: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    semantic_id: str


@dataclass(frozen=True)
class AisleLayout:
    primitives: tuple[Box, ...]
    assets: tuple[AssetInstance, ...]
    seed: int
    asset_manifest_path: str = ""

    @property
    def products(self) -> tuple[AssetInstance, ...]:
        """Compatibility name for callers that count occupied product slots."""
        return self.assets


def _zone_categories(bay: int, bay_count: int, row_index: int, level: int) -> tuple[str, ...]:
    """Choose an asset category zone without changing the old slot layout."""
    if bay >= max(0, bay_count - 2) and level == 0:
        return ("red_apples", "green_apples", "oranges", "lemons")
    zones = (
        ("cereal",),
        ("snacks",),
        ("cans", "jars"),
        ("juice", "water", "soda"),
    )
    zone_index = min(len(zones) - 1, int(bay * len(zones) / max(1, bay_count)))
    if row_index % 2:
        zone_index = len(zones) - 1 - zone_index
    return zones[zone_index]


def _candidate_assets(catalog: RetailAssetCatalog, categories: tuple[str, ...]) -> tuple[RetailAsset, ...]:
    candidates = tuple(asset for category in categories for asset in catalog.by_category(category))
    if not candidates:
        raise ValueError(f"No retail assets are available for categories {categories!r}")
    return candidates


def _slot_asset_rng(seed: int, row_index: int, bay: int, level: int, slot: int) -> random.Random:
    """Return a process-stable RNG independent of the original layout RNG."""
    key = f"{seed}:{row_index}:{bay}:{level}:{slot}".encode("utf-8")
    value = int.from_bytes(hashlib.sha256(key).digest()[:8], "big", signed=False)
    return random.Random(value)


def _asset_for_slot(
    catalog: RetailAssetCatalog,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    bay: int,
    level: int,
    slot: int,
    position_xy: tuple[float, float],
    shelf_center_z: float,
    bay_count: int,
) -> AssetInstance:
    rng = _slot_asset_rng(config.seed, row_index, bay, level, slot)
    categories = _zone_categories(bay, bay_count, row_index, level)
    if categories == ("red_apples", "green_apples", "oranges", "lemons") and row_index == 0 and bay == bay_count - 2 and level == 0 and slot == 0:
        record = catalog.by_category("red_apples")[0]
    else:
        record = rng.choice(_candidate_assets(catalog, categories))
    shelf_top_z = shelf_center_z + 0.06 / 2.0
    center_z = shelf_top_z + record.dimensions_m[2] / 2.0
    yaw = 0.0 if row_y < 0.0 else 180.0
    yaw += rng.uniform(-2.0, 2.0)
    return AssetInstance(
        name=f"product_r{row_index}_b{bay}_l{level}_s{slot}",
        asset_key=record.asset_key,
        category=record.category,
        position_m=(position_xy[0], position_xy[1], center_z),
        rotation_rpy_deg=(0.0, 0.0, yaw),
        scale_xyz=(1.0, 1.0, 1.0),
        semantic_id=f"retail/{record.category}/{record.asset_key}/r{row_index}/b{bay}/l{level}/s{slot}",
    )


def build_aisle_layout(config: AisleConfig) -> AisleLayout:
    """Reproduce the approved aisle geometry and replace occupied cubes with assets."""
    # This RNG loop intentionally mirrors the original builder. Do not use
    # the asset RNG for occupancy, slot jitter, or later structural decisions.
    rng = random.Random(config.seed)
    catalog = load_retail_catalog(config.asset_manifest_path)
    primitives: list[Box] = []
    assets: list[AssetInstance] = []
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
                    # Consume the original placeholder scale draw so all
                    # subsequent occupancy and slot decisions stay identical.
                    rng.random()
                    assets.append(_asset_for_slot(catalog, config, row_index, row_y, bay, level, slot, (px, py), z, bay_count))
            for side in (-1.0, 1.0):
                y = row_y + side * config.shelf_depth_m * 0.42
                primitives.append(Box(f"upright_r{row_index}_b{bay}_{int(side)}", (x, y, config.floor_z_m + config.shelf_height_m / 2.0), (0.08, 0.08, config.shelf_height_m), "upright"))
    return AisleLayout(tuple(primitives), tuple(assets), config.seed, config.asset_manifest_path)
