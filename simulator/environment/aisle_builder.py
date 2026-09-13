"""Deterministic grocery aisle geometry with dense, asset-backed stocking."""

from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass

from simulator.config.loader import AisleConfig
from simulator.environment.retail_catalog import RetailAsset, RetailAssetCatalog, load_retail_catalog


SHELF_THICKNESS_M = 0.06
SHELF_BOTTOM_CENTER_OFFSET_M = 0.30
SHELF_TOP_CENTER_MARGIN_M = 0.05


@dataclass(frozen=True)
class Box:
    name: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    kind: str


@dataclass(frozen=True)
class AssetInstance:
    """One physical retail object with a stable semantic identity."""

    name: str
    asset_key: str
    category: str
    position_m: tuple[float, float, float]
    rotation_rpy_deg: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    semantic_id: str

    @property
    def instance_id(self) -> str:
        """Stable export-facing identity; semantic_id is the authoritative ID."""
        return self.semantic_id


@dataclass(frozen=True)
class AisleLayout:
    primitives: tuple[Box, ...]
    assets: tuple[AssetInstance, ...]
    seed: int
    asset_manifest_path: str = ""

    @property
    def products(self) -> tuple[AssetInstance, ...]:
        """Compatibility name for callers that count physical retail objects."""
        return self.assets


def _stable_rng(seed: int, *parts: object) -> random.Random:
    """Return a process-stable RNG independent of Python hash randomization."""
    key = ":".join(str(part) for part in (seed, *parts)).encode("utf-8")
    value = int.from_bytes(hashlib.sha256(key).digest()[:8], "big", signed=False)
    return random.Random(value)


def _zone_name(bay: int, bay_count: int, row_index: int) -> str:
    """Return the merchandising zone for a bay while mirroring both rows."""
    if bay >= max(0, bay_count - 2):
        return "produce"
    zones = ("cereal", "snacks", "cans_jars", "beverage")
    zone_index = min(len(zones) - 1, int(bay * len(zones) / max(1, bay_count - 2)))
    if row_index % 2:
        zone_index = len(zones) - 1 - zone_index
    return zones[zone_index]


def _zone_categories(zone: str, level: int) -> tuple[str, ...]:
    if zone == "cereal":
        return ("cereal",)
    if zone == "snacks":
        return ("snacks",)
    if zone == "cans_jars":
        return ("cans", "jars")
    if zone == "beverage":
        return ("juice", "water", "soda")
    if zone == "produce":
        # The lowest shelf is occupied by explicit produce crates and fruit.
        # Upper shelves in the final bays remain useful beverage shelving.
        return () if level == 0 else ("juice", "water", "soda")
    raise ValueError(f"Unknown aisle merchandising zone: {zone}")


def _shelf_height_categories(zone: str, catalog: RetailAssetCatalog) -> tuple[str, ...]:
    if zone == "produce":
        return ("juice", "water", "soda", "produce_crate")
    return _zone_categories(zone, 1)


def _candidate_assets(catalog: RetailAssetCatalog, categories: tuple[str, ...]) -> tuple[RetailAsset, ...]:
    candidates = tuple(asset for category in categories for asset in catalog.by_category(category))
    if not candidates:
        raise ValueError(f"No retail assets are available for categories {categories!r}")
    return candidates


def shelf_level_count_for_zone(config: AisleConfig, catalog: RetailAssetCatalog, zone: str) -> int:
    """Derive a zone's shelf count from asset height and usable vertical space."""
    bottom = config.floor_z_m + SHELF_BOTTOM_CENTER_OFFSET_M
    top = config.floor_z_m + config.shelf_height_m - SHELF_TOP_CENTER_MARGIN_M
    available = top - bottom
    if available <= 0.0:
        raise ValueError("shelf_height_m is too small for the shelf base/top margins")
    max_height = max(asset.dimensions_m[2] for asset in _candidate_assets(catalog, _shelf_height_categories(zone, catalog)))
    required_spacing = max_height + SHELF_THICKNESS_M + config.shelf_clearance_m
    return max(2, int(math.floor(available / required_spacing)) + 1)


def shelf_positions_for_zone(config: AisleConfig, catalog: RetailAssetCatalog, zone: str) -> tuple[float, ...]:
    """Return monotonically increasing shelf centers for one merchandising zone."""
    count = shelf_level_count_for_zone(config, catalog, zone)
    bottom = config.floor_z_m + SHELF_BOTTOM_CENTER_OFFSET_M
    top = config.floor_z_m + config.shelf_height_m - SHELF_TOP_CENTER_MARGIN_M
    step = (top - bottom) / (count - 1)
    return tuple(bottom + index * step for index in range(count))


def shelf_level_counts_by_zone(config: AisleConfig, catalog: RetailAssetCatalog | None = None) -> dict[str, int]:
    catalog = catalog or load_retail_catalog(config.asset_manifest_path)
    return {zone: shelf_level_count_for_zone(config, catalog, zone) for zone in ("cereal", "snacks", "cans_jars", "beverage", "produce")}


def _make_asset(
    record: RetailAsset,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    position: tuple[float, float, float],
    identity_parts: tuple[object, ...],
    name: str,
) -> AssetInstance:
    yaw_rng = _stable_rng(config.seed, "yaw", *identity_parts)
    yaw = 0.0 if row_y < 0.0 else 180.0
    yaw += yaw_rng.uniform(-2.0, 2.0)
    suffix = "/".join(str(part) for part in identity_parts)
    return AssetInstance(
        name=name,
        asset_key=record.asset_key,
        category=record.category,
        position_m=position,
        rotation_rpy_deg=(0.0, 0.0, yaw),
        scale_xyz=(1.0, 1.0, 1.0),
        semantic_id=f"retail/{record.asset_key}/{suffix}",
    )


def _facing_count(config: AisleConfig, candidates: tuple[RetailAsset, ...], row: int, bay: int, level: int) -> int:
    max_width = max(asset.dimensions_m[0] for asset in candidates)
    usable_width = config.bay_width_m - 2.0 * config.edge_margin_m
    capacity = max(1, int((usable_width + config.facing_gap_m) // (max_width + config.facing_gap_m)))
    count_rng = _stable_rng(config.seed, "facing_count", row, bay, level)
    if capacity > 1 and count_rng.random() < 0.22:
        capacity -= 1
    # Keep the aisle richly stocked without turning each bay into a tiled wall.
    return min(capacity, 4)


def _depth_offsets(config: AisleConfig, max_depth: float) -> tuple[float, ...]:
    usable_depth = config.shelf_depth_m - 2.0 * config.edge_margin_m
    required_depth = config.depth_facings * max_depth + max(0, config.depth_facings - 1) * config.depth_gap_m
    if required_depth > usable_depth + 1e-9:
        raise ValueError("depth_facings and depth_gap_m do not fit within shelf_depth_m")
    first = config.shelf_depth_m / 2.0 - config.edge_margin_m - max_depth / 2.0
    return tuple(first - index * (max_depth + config.depth_gap_m) for index in range(config.depth_facings))


def _populate_shelf_products(
    assets: list[AssetInstance],
    catalog: RetailAssetCatalog,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    bay: int,
    level: int,
    shelf_z: float,
    bay_count: int,
) -> None:
    zone = _zone_name(bay, bay_count, row_index)
    categories = _zone_categories(zone, level)
    if not categories:
        return
    candidates = _candidate_assets(catalog, categories)
    facing_count = _facing_count(config, candidates, row_index, bay, level)
    selected = [
        _stable_rng(config.seed, "asset", row_index, bay, level, facing).choice(candidates)
        for facing in range(facing_count)
    ]
    total_width = sum(asset.dimensions_m[0] for asset in selected) + config.facing_gap_m * (facing_count - 1)
    cursor = -total_width / 2.0
    max_depth = max(asset.dimensions_m[1] for asset in selected)
    depth_offsets = _depth_offsets(config, max_depth)
    front_sign = 1.0 if row_y < 0.0 else -1.0
    for facing, record in enumerate(selected):
        width = record.dimensions_m[0]
        px = (config.bay_width_m / 2.0) + bay * config.bay_width_m + cursor + width / 2.0
        cursor += width + config.facing_gap_m
        for depth, depth_offset in enumerate(depth_offsets):
            occupied_rng = _stable_rng(config.seed, "occupied", row_index, bay, level, depth, facing)
            if occupied_rng.random() > config.product_density:
                continue
            py = row_y + front_sign * depth_offset
            identity = ("r" + str(row_index), "b" + str(bay), "l" + str(level), "d" + str(depth), "f" + str(facing))
            assets.append(_make_asset(
                record,
                config,
                row_index,
                row_y,
                (px, py, shelf_z + SHELF_THICKNESS_M / 2.0 + record.dimensions_m[2] / 2.0),
                identity,
                "product_" + "_".join(identity),
            ))


def _populate_produce(
    assets: list[AssetInstance],
    catalog: RetailAssetCatalog,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    bay: int,
    shelf_z: float,
    fruit_category: str,
    bin_index: int,
) -> None:
    crates = catalog.by_category("produce_crate")
    crate = crates[bin_index % len(crates)]
    fruit = catalog.by_category(fruit_category)[0]
    x = config.bay_width_m / 2.0 + bay * config.bay_width_m
    crate_identity = ("bin", "r" + str(row_index), "b" + str(bay), "i" + str(bin_index))
    crate_position = (x, row_y, shelf_z + SHELF_THICKNESS_M / 2.0 + crate.dimensions_m[2] / 2.0)
    assets.append(_make_asset(crate, config, row_index, row_y, crate_position, crate_identity, "produce_bin_" + "_".join(crate_identity[1:])))

    x_offsets = (-0.14, -0.047, 0.047, 0.14)
    y_offsets = (-0.085, 0.0, 0.085)
    for fruit_index in range(config.produce_items_per_crate):
        layer, remainder = divmod(fruit_index, len(x_offsets) * len(y_offsets))
        x_index, y_index = divmod(remainder, len(y_offsets))
        jitter = _stable_rng(config.seed, "fruit_jitter", row_index, bay, bin_index, fruit_index)
        px = x + x_offsets[x_index] + jitter.uniform(-0.006, 0.006)
        py = row_y + y_offsets[y_index] + jitter.uniform(-0.006, 0.006)
        pz = crate_position[2] + crate.dimensions_m[2] / 2.0 + fruit.dimensions_m[2] / 2.0 + layer * 0.07
        identity = ("bin", "r" + str(row_index), "b" + str(bay), "i" + str(bin_index), "fruit" + str(fruit_index))
        assets.append(_make_asset(fruit, config, row_index, row_y, (px, py, pz), identity, "fruit_" + "_".join(identity[1:])))


def build_aisle_layout(config: AisleConfig) -> AisleLayout:
    """Build the original open aisle with dimension-aware, dense stocking."""
    catalog = load_retail_catalog(config.asset_manifest_path)
    primitives: list[Box] = []
    assets: list[AssetInstance] = []
    floor_z = config.floor_z_m - 0.05
    primitives.append(Box("floor", (config.length_m / 2.0, 0.0, floor_z), (config.length_m + 2.0, config.width_m + 2.0, 0.1), "floor"))
    bay_count = max(1, int(config.length_m // config.bay_width_m))
    shelf_positions: dict[tuple[int, int], tuple[float, ...]] = {}
    for row_index, row_y in enumerate(config.shelf_rows_y_m):
        for bay in range(bay_count):
            zone = _zone_name(bay, bay_count, row_index)
            positions = shelf_positions_for_zone(config, catalog, zone)
            shelf_positions[(row_index, bay)] = positions
            x = config.bay_width_m / 2.0 + bay * config.bay_width_m
            for level, z in enumerate(positions):
                primitives.append(Box(f"shelf_r{row_index}_b{bay}_l{level}", (x, row_y, z), (config.bay_width_m * 0.98, config.shelf_depth_m, SHELF_THICKNESS_M), "shelf"))
                _populate_shelf_products(assets, catalog, config, row_index, row_y, bay, level, z, bay_count)
            for side in (-1.0, 1.0):
                y = row_y + side * config.shelf_depth_m * 0.42
                primitives.append(Box(f"upright_r{row_index}_b{bay}_{int(side)}", (x, y, config.floor_z_m + config.shelf_height_m / 2.0), (0.08, 0.08, config.shelf_height_m), "upright"))

    produce_assignments = (
        (0, bay_count - 2, "red_apples", 0),
        (0, bay_count - 1, "green_apples", 1),
        (1, bay_count - 2, "oranges", 2),
        (1, bay_count - 1, "lemons", 3),
    )
    for row_index, bay, fruit_category, bin_index in produce_assignments:
        _populate_produce(
            assets,
            catalog,
            config,
            row_index,
            config.shelf_rows_y_m[row_index],
            bay,
            shelf_positions[(row_index, bay)][0],
            fruit_category,
            bin_index,
        )
    return AisleLayout(tuple(primitives), tuple(assets), config.seed, config.asset_manifest_path)
