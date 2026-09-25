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
    if row_index == 0 and bay <= 1:
        return "hero"
    # Put a compact produce section within the walking view rather than only
    # at the exit, so the RGB proof video visibly contains fruit.
    if 7 <= bay <= 10:
        return "produce"
    zones = ("cereal", "snacks", "cans_jars", "beverage")
    zone_index = min(len(zones) - 1, int(bay * len(zones) / max(1, bay_count - 2)))
    if row_index % 2:
        zone_index = len(zones) - 1 - zone_index
    return zones[zone_index]


def _zone_categories(zone: str, level: int) -> tuple[str, ...]:
    if zone == "hero":
        plan = (("cereal",), ("pantry_box",), ("snack_bag", "bagged_goods"), ("milk", "refrigerated"), ("beverage", "juice"))
        return plan[level % len(plan)]
    if zone == "cereal":
        plan = (("cereal",), ("pantry_box",), ("cereal",), ("pantry_box",), ("cereal",))
        return plan[level % len(plan)]
    if zone == "snacks":
        plan = (("snack_bag", "snacks"), ("bagged_goods",), ("bakery", "snacks"), ("bagged_goods",), ("snack_bag", "snacks"))
        return plan[level % len(plan)]
    if zone == "cans_jars":
        plan = (("cans",), ("jars",), ("condiments",), ("household",), ("cleaning",), ("cans",), ("jars",))
        return plan[level % len(plan)]
    if zone == "beverage":
        plan = (("milk",), ("refrigerated",), ("frozen",), ("beverage",), ("juice",), ("water", "soda"))
        return plan[level % len(plan)]
    if zone == "produce":
        # Level zero is reserved for the fixture bins and the top level is
        # reserved for the four camera-visible crate/fruit displays.  Keeping
        # both footprints out of the regular product pass prevents two
        # independently populated families from sharing the same shelf volume.
        plan = ((), ("fresh_produce",), ("fresh_produce",), ("beverage",), (), ("juice",))
        return plan[level % len(plan)]
    raise ValueError(f"Unknown aisle merchandising zone: {zone}")


def _shelf_height_categories(zone: str, catalog: RetailAssetCatalog) -> tuple[str, ...]:
    zone_categories = {
        "hero": ("cereal", "pantry_box", "snack_bag", "bagged_goods", "milk", "refrigerated", "beverage", "juice"),
        "cereal": ("cereal", "pantry_box"),
        "snacks": ("snack_bag", "snacks", "bagged_goods", "bakery"),
        "cans_jars": ("cans", "jars", "condiments", "household", "cleaning"),
        "beverage": ("milk", "refrigerated", "frozen", "beverage", "juice", "water", "soda"),
        "produce": ("fresh_produce", "beverage", "refrigerated", "juice", "produce_crate", "produce_fixture"),
    }
    return zone_categories[zone]


def _candidate_assets(catalog: RetailAssetCatalog, categories: tuple[str, ...]) -> tuple[RetailAsset, ...]:
    candidates = tuple(sorted(
        (asset for category in categories for asset in catalog.by_category(category)),
        key=lambda asset: asset.asset_key,
    ))
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
    return {zone: shelf_level_count_for_zone(config, catalog, zone) for zone in ("hero", "cereal", "snacks", "cans_jars", "beverage", "produce")}


def _make_asset(
    record: RetailAsset,
    config: AisleConfig,
    row_index: int,
    row_y: float,
    position: tuple[float, float, float],
    identity_parts: tuple[object, ...],
    name: str,
    scale_xyz: tuple[float, float, float] | None = None,
) -> AssetInstance:
    pose_rng = _stable_rng(config.seed, "pose", *identity_parts)
    yaw = 0.0 if row_y < 0.0 else 180.0
    yaw += pose_rng.uniform(-3.6, 3.6) if record.intended_support == "shelf" else pose_rng.uniform(-2.0, 2.0)
    # Small per-object offsets make a row read as hand-stocked while remaining
    # safely inside the shelf footprint.  The RNG key is the semantic identity,
    # so inserting a facing cannot shift every later object's pose.
    exact_support_placement = record.model_type in {
        "crate", "angled_bin", "wicker_basket", "shelf_divider", "bottle_rack", "price_display",
    } or (record.model_type == "fruit" and scale_xyz is not None)
    x_offset = 0.0 if exact_support_placement else pose_rng.uniform(-0.0025, 0.0025)
    depth_offset = 0.0 if exact_support_placement else pose_rng.uniform(-0.003, 0.003)
    px, py, pz = position
    scale = scale_xyz or (1.0, 1.0, 1.0)
    if record.model_type == "fruit" and scale_xyz is None:
        # Fruit is intentionally less uniform than packaged goods.  Keep the
        # center fixed so the sampled shelf support remains valid.
        sx = pose_rng.uniform(0.93, 1.07)
        sy = pose_rng.uniform(0.93, 1.07)
        sz = pose_rng.uniform(0.94, 1.09)
        scale = (sx, sy, sz)
    suffix = "/".join(str(part) for part in identity_parts)
    return AssetInstance(
        name=name,
        asset_key=record.asset_key,
        category=record.category,
        position_m=(px + x_offset, py + depth_offset, pz),
        rotation_rpy_deg=(0.0, 0.0, yaw),
        scale_xyz=scale,
        semantic_id=f"retail/{record.asset_key}/{suffix}",
    )


def _facing_count(config: AisleConfig, candidates: tuple[RetailAsset, ...], row: int, bay: int, level: int) -> int:
    min_width = min(asset.dimensions_m[0] for asset in candidates)
    usable_width = config.bay_width_m - 2.0 * config.edge_margin_m
    capacity = max(1, int((usable_width + config.facing_gap_m) // (min_width + config.facing_gap_m)))
    count_rng = _stable_rng(config.seed, "facing_count", row, bay, level)
    if capacity > 1 and count_rng.random() < 0.22:
        capacity -= 1
    # Keep the aisle richly stocked without turning each bay into a tiled wall.
    return min(capacity, 6)


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
    # Two neighboring facings share a SKU, as real merchandising blocks do.
    # Key sorting above makes manifest ordering irrelevant to the selection.
    start = (config.seed + row_index * bay_count + bay * 3 + level) % len(candidates)
    selected = [candidates[(start + facing // 2) % len(candidates)] for facing in range(facing_count)]
    usable_width = config.bay_width_m - 2.0 * config.edge_margin_m
    while len(selected) > 1 and sum(asset.dimensions_m[0] for asset in selected) + config.facing_gap_m * (len(selected) - 1) > usable_width:
        selected.pop()
    facing_count = len(selected)
    slot_widths = [asset.dimensions_m[0] for asset in selected]
    total_width = sum(slot_widths) + config.facing_gap_m * (facing_count - 1)
    cursor = -total_width / 2.0
    max_depth = max(asset.dimensions_m[1] for asset in selected)
    depth_offsets = _depth_offsets(config, max_depth)
    front_sign = 1.0 if row_y < 0.0 else -1.0
    for facing, record in enumerate(selected):
        width = slot_widths[facing]
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
    crate_instance = _make_asset(crate, config, row_index, row_y, crate_position, crate_identity, "produce_bin_" + "_".join(crate_identity[1:]))
    assets.append(crate_instance)

    # Two 4x3 layers fit inside the 44x32x22 cm bin.  Fruit is scaled to a
    # deterministic near-spherical 9.2-9.4 cm diameter so grid spacing leaves
    # real clearance instead of the severe pairwise intersections in the old
    # full-size pile.
    x_offsets = (-0.15, -0.05, 0.05, 0.15)
    y_offsets = (-0.10, 0.0, 0.10)
    layer_capacity = len(x_offsets) * len(y_offsets)
    if config.produce_items_per_crate > 2 * layer_capacity:
        raise ValueError("produce_items_per_crate exceeds the non-overlapping two-layer crate capacity of 24")
    crate_base_thickness = 0.025
    lower_centers: dict[int, tuple[float, float, float]] = {}
    crate_yaw = math.radians(crate_instance.rotation_rpy_deg[2])
    crate_cos, crate_sin = math.cos(crate_yaw), math.sin(crate_yaw)
    for fruit_index in range(config.produce_items_per_crate):
        layer, remainder = divmod(fruit_index, layer_capacity)
        x_index, y_index = divmod(remainder, len(y_offsets))
        identity = ("bin", "r" + str(row_index), "b" + str(bay), "i" + str(bin_index), "fruit" + str(fruit_index))
        size_rng = _stable_rng(config.seed, "fruit_size", row_index, bay, bin_index, remainder)
        diameter = 0.093 + size_rng.uniform(-0.001, 0.001)
        # Correct for catalog axis differences (notably lemons) so the USD
        # sphere remains spherical after instance scaling.
        scale = tuple(diameter / dimension for dimension in fruit.dimensions_m)
        local_x = x_offsets[x_index]
        local_y = y_offsets[y_index]
        px = x + crate_cos * local_x - crate_sin * local_y
        py = row_y + crate_sin * local_x + crate_cos * local_y
        half_height = diameter / 2.0
        crate_bottom = crate_position[2] - crate.dimensions_m[2] / 2.0
        if layer == 0:
            pz = crate_bottom + crate_base_thickness + half_height
            lower_centers[remainder] = (px, py, pz)
        else:
            support = lower_centers[remainder]
            pz = support[2] + half_height + fruit.dimensions_m[2] * scale[2] / 2.0
        assets.append(_make_asset(
            fruit, config, row_index, row_y, (px, py, pz), identity,
            "fruit_" + "_".join(identity[1:]), scale_xyz=scale,
        ))


def _populate_store_fixtures(
    assets: list[AssetInstance],
    catalog: RetailAssetCatalog,
    config: AisleConfig,
    shelf_positions: dict[tuple[int, int], tuple[float, ...]],
) -> None:
    """Place bounded shelf hardware and empty produce bins without product overlap."""
    price = catalog.by_key("price_display")
    divider = catalog.by_key("shelf_divider")
    rack = catalog.by_key("bottle_rack")
    bin_keys = ("angled_produce_bin", "wicker_basket")

    # Hero price labels attach to the front lip; dividers occupy the unused
    # outer 3 cm rather than cutting through the centered product block.
    for bay in (0, 1):
        row_index = 0
        row_y = config.shelf_rows_y_m[row_index]
        front_sign = 1.0
        bay_center = config.bay_width_m / 2.0 + bay * config.bay_width_m
        for level, shelf_z in enumerate(shelf_positions[(row_index, bay)]):
            identity = ("fixture", "price", "r0", f"b{bay}", f"l{level}")
            price_position = (
                bay_center,
                row_y + front_sign * (config.shelf_depth_m / 2.0 + price.dimensions_m[1] / 2.0),
                shelf_z,
            )
            assets.append(_make_asset(price, config, row_index, row_y, price_position, identity, "_".join(identity)))
            if level in (1, 3):
                divider_identity = ("fixture", "divider", "r0", f"b{bay}", f"l{level}")
                divider_position = (
                    bay_center + config.bay_width_m / 2.0 - config.edge_margin_m - divider.dimensions_m[0] / 2.0,
                    row_y,
                    shelf_z + SHELF_THICKNESS_M / 2.0 + divider.dimensions_m[2] / 2.0,
                )
                assets.append(_make_asset(divider, config, row_index, row_y, divider_position, divider_identity, "_".join(divider_identity)))

    # Produce level zero is intentionally product-free, so racks/bins have
    # dedicated shelf footprints and cannot clip regular stock.
    for offset, bay in enumerate(range(7, 11)):
        shelf_z = shelf_positions[(1, bay)][0]
        row_y = config.shelf_rows_y_m[1]
        record = catalog.by_key(bin_keys[offset % len(bin_keys)])
        identity = ("fixture", "produce", "r1", f"b{bay}")
        position = (
            config.bay_width_m / 2.0 + bay * config.bay_width_m,
            row_y,
            shelf_z + SHELF_THICKNESS_M / 2.0 + record.dimensions_m[2] / 2.0,
        )
        assets.append(_make_asset(record, config, 1, row_y, position, identity, "_".join(identity)))

    for bay in (7, 8):
        shelf_z = shelf_positions[(0, bay)][0]
        row_y = config.shelf_rows_y_m[0]
        identity = ("fixture", "rack", "r0", f"b{bay}")
        position = (
            config.bay_width_m / 2.0 + bay * config.bay_width_m,
            row_y,
            shelf_z + SHELF_THICKNESS_M / 2.0 + rack.dimensions_m[2] / 2.0,
        )
        assets.append(_make_asset(rack, config, 0, row_y, position, identity, "_".join(identity)))


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

    _populate_store_fixtures(assets, catalog, config, shelf_positions)

    # Keep one clearly visible produce run on the near row so the walkthrough
    # actually presents fruit before the aisle exit.  It remains grouped at
    # the endcap and uses four deterministic bins/categories.
    produce_assignments = (
        (0, 7, "red_apples", 0),
        (0, 8, "green_apples", 1),
        (0, 9, "oranges", 2),
        (0, 10, "lemons", 3),
    )
    for row_index, bay, fruit_category, bin_index in produce_assignments:
        _populate_produce(
            assets,
            catalog,
            config,
            row_index,
            config.shelf_rows_y_m[row_index],
            bay,
            # Top-shelf produce displays clear the next shelf board, so the
            # fruit remains visible from the calibrated first-person camera
            # while the crate stays shelf-supported.
            shelf_positions[(row_index, bay)][-1],
            fruit_category,
            bin_index,
        )
    return AisleLayout(tuple(primitives), tuple(assets), config.seed, config.asset_manifest_path)
