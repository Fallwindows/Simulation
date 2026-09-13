"""Deterministic, Isaac-independent grocery aisle geometry and asset layout."""

from __future__ import annotations

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
    """One deterministic reference to a reusable retail asset."""

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
    asset_manifest_path: str

    @property
    def products(self) -> tuple[AssetInstance, ...]:
        """Compatibility name for callers that previously counted product boxes."""
        return self.assets


def _zone_categories(bay: int, bay_count: int, row_index: int) -> tuple[str, ...]:
    """Return a stable category zone, alternating direction across the aisle."""
    zones = (
        ("cereal",),
        ("snacks",),
        ("cans", "jars"),
        ("juice", "water", "soda"),
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


def _place_shelf_assets(
    assets: list[AssetInstance],
    catalog: RetailAssetCatalog,
    rng: random.Random,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    bay: int,
    level: int,
    shelf_center_z: float,
    bay_count: int,
) -> None:
    categories = _zone_categories(bay, bay_count, row_index)
    pool = _candidate_assets(catalog, categories)
    target = rng.randint(config.min_facings_per_bay, config.max_facings_per_bay)
    aisle_sign = 1.0 if row_y < 0.0 else -1.0
    shelf_top_z = shelf_center_z + 0.03
    usable_width = config.bay_width_m * 0.93
    gap = 0.006

    chosen: list[RetailAsset] = []
    total_width = 0.0
    for _ in range(target):
        shuffled = list(pool)
        rng.shuffle(shuffled)
        selected = next(
            (candidate for candidate in shuffled if total_width + candidate.dimensions_m[0] + (gap if chosen else 0.0) <= usable_width),
            None,
        )
        if selected is None:
            break
        chosen.append(selected)
        total_width += selected.dimensions_m[0] + (gap if len(chosen) > 1 else 0.0)

    if len(chosen) < config.min_facings_per_bay:
        raise ValueError(
            f"Could not fit the configured minimum {config.min_facings_per_bay} facings in bay {bay}, "
            f"level {level}; usable width is {usable_width:.3f} m"
        )

    cursor_x = config.bay_width_m / 2.0 + bay * config.bay_width_m - total_width / 2.0
    for slot, record in enumerate(chosen):
        width, depth, height = record.dimensions_m
        scale = 0.99 + rng.random() * 0.02
        scaled_width, scaled_depth, scaled_height = width * scale, depth * scale, height * scale
        center_x = cursor_x + scaled_width / 2.0 + rng.uniform(-0.004, 0.004)
        front_y = row_y + aisle_sign * (config.shelf_depth_m / 2.0 - scaled_depth / 2.0 - 0.012)
        center_y = front_y + rng.uniform(-0.003, 0.003)
        center_z = shelf_top_z + scaled_height / 2.0
        yaw = 0.0 if aisle_sign > 0.0 else 180.0
        yaw += rng.uniform(-2.0, 2.0)
        assets.append(AssetInstance(
            name=f"product_r{row_index}_b{bay}_l{level}_s{slot}",
            asset_key=record.asset_key,
            category=record.category,
            position_m=(center_x, center_y, center_z),
            rotation_rpy_deg=(0.0, 0.0, yaw),
            scale_xyz=(scale, scale, scale),
            semantic_id=f"retail/{record.category}/{record.asset_key}/r{row_index}/b{bay}/l{level}/s{slot}",
        ))
        cursor_x += scaled_width + gap


def _place_endcap_assets(assets: list[AssetInstance], catalog: RetailAssetCatalog, rng: random.Random, config: AisleConfig, bay_count: int) -> None:
    """Create a produce/promotional feature at the far end without filling the aisle."""
    candidates = tuple(
        asset for category in ("produce_crate", "red_apples", "green_apples", "oranges", "lemons", "promotional_sign")
        for asset in catalog.by_category(category)
    )
    by_key = {asset.asset_key: asset for asset in candidates}
    end_x = bay_count * config.bay_width_m - 0.52
    display_items = (
        ("produce_crate_green", -0.72, 0.20, 0.0),
        ("produce_crate_red", 0.72, 0.20, 180.0),
        ("red_apple", -0.72, 0.39, 0.0),
        ("green_apple", -0.56, 0.39, 0.0),
        ("orange", 0.56, 0.39, 180.0),
        ("lemon", 0.72, 0.39, 180.0),
    )
    for slot, (key, y, z, yaw) in enumerate(display_items):
        record = by_key[key]
        scale = 0.98 + rng.random() * 0.04
        assets.append(AssetInstance(
            name=f"endcap_{slot}_{key}",
            asset_key=key,
            category=record.category,
            position_m=(end_x, y, z + record.dimensions_m[2] * scale / 2.0),
            rotation_rpy_deg=(0.0, 0.0, yaw),
            scale_xyz=(scale, scale, scale),
            semantic_id=f"retail/endcap/{key}/{slot}",
        ))
    sign = by_key["promo_market_sign"]
    assets.append(AssetInstance(
        name="endcap_promo_sign",
        asset_key=sign.asset_key,
        category=sign.category,
        position_m=(end_x - 0.10, 0.0, config.shelf_height_m + 0.45),
        rotation_rpy_deg=(0.0, 0.0, -90.0),
        scale_xyz=(1.0, 1.0, 1.0),
        semantic_id="retail/endcap/promotional_sign/main",
    ))


def build_aisle_layout(config: AisleConfig) -> AisleLayout:
    """Create a repeatable structural aisle and dense reusable asset placement."""
    rng = random.Random(config.seed)
    catalog = load_retail_catalog(config.asset_manifest_path)
    primitives: list[Box] = []
    assets: list[AssetInstance] = []
    floor_z = config.floor_z_m - 0.05
    bay_count = max(1, int(config.length_m // config.bay_width_m))
    level_spacing = config.shelf_height_m / (config.shelf_levels + 1)

    primitives.append(Box("floor", (config.length_m / 2.0, 0.0, floor_z), (config.length_m + 2.0, config.width_m + 2.0, 0.1), "floor"))
    primitives.extend((
        Box("left_wall", (config.length_m / 2.0, -config.width_m / 2.0, config.floor_z_m + config.ceiling_height_m / 2.0), (config.length_m + 1.0, 0.12, config.ceiling_height_m), "wall"),
        Box("right_wall", (config.length_m / 2.0, config.width_m / 2.0, config.floor_z_m + config.ceiling_height_m / 2.0), (config.length_m + 1.0, 0.12, config.ceiling_height_m), "wall"),
        Box("far_wall", (config.length_m + 0.20, 0.0, config.floor_z_m + config.ceiling_height_m / 2.0), (0.16, config.width_m, config.ceiling_height_m), "wall"),
        Box("ceiling", (config.length_m / 2.0, 0.0, config.floor_z_m + config.ceiling_height_m), (config.length_m + 1.0, config.width_m + 1.0, 0.10), "ceiling"),
        Box("baseboard_left", (config.length_m / 2.0, -config.width_m / 2.0 + 0.08, config.floor_z_m + 0.10), (config.length_m + 1.0, 0.08, 0.20), "baseboard"),
        Box("baseboard_right", (config.length_m / 2.0, config.width_m / 2.0 - 0.08, config.floor_z_m + 0.10), (config.length_m + 1.0, 0.08, 0.20), "baseboard"),
    ))

    for row_index, row_y in enumerate(config.shelf_rows_y_m):
        aisle_sign = 1.0 if row_y < 0.0 else -1.0
        rear_y = row_y - aisle_sign * config.shelf_depth_m / 2.0
        front_y = row_y + aisle_sign * config.shelf_depth_m / 2.0
        primitives.append(Box(f"rear_panel_r{row_index}", (config.length_m / 2.0, rear_y, config.floor_z_m + config.shelf_height_m / 2.0), (config.length_m, 0.045, config.shelf_height_m), "rear_panel"))
        for boundary in range(bay_count + 1):
            x = boundary * config.bay_width_m
            primitives.append(Box(f"upright_r{row_index}_b{boundary}", (x, row_y, config.floor_z_m + config.shelf_height_m / 2.0), (0.075, 0.075, config.shelf_height_m), "upright"))
        for bay in range(bay_count):
            x = config.bay_width_m / 2.0 + bay * config.bay_width_m
            for level in range(config.shelf_levels):
                z = config.floor_z_m + (level + 1) * level_spacing
                primitives.append(Box(f"shelf_r{row_index}_b{bay}_l{level}", (x, row_y, z), (config.bay_width_m * 0.98, config.shelf_depth_m, 0.06), "shelf"))
                primitives.append(Box(f"front_lip_r{row_index}_b{bay}_l{level}", (x, front_y, z + 0.025), (config.bay_width_m * 0.98, 0.035, 0.085), "shelf_lip"))
                primitives.append(Box(f"price_strip_r{row_index}_b{bay}_l{level}", (x, front_y + aisle_sign * 0.021, z - 0.015), (config.bay_width_m * 0.96, 0.018, 0.045), "price_strip"))
                _place_shelf_assets(assets, catalog, rng, config, row_index, row_y, bay, level, z, bay_count)

    primitives.extend((
        Box("endcap_header", (config.length_m - 0.55, 0.0, config.shelf_height_m + 0.08), (0.10, 1.9, 0.12), "endcap"),
        Box("endcap_counter", (config.length_m - 0.55, 0.0, 0.12), (0.8, 1.9, 0.24), "endcap"),
    ))
    _place_endcap_assets(assets, catalog, rng, config, bay_count)
    return AisleLayout(tuple(primitives), tuple(assets), config.seed, config.asset_manifest_path)
