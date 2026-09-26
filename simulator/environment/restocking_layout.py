"""Deterministic geometry and physics specification for one restocking cycle.

The stockroom attaches to the negative-x open end of the existing aisle.  All
values are SI and use the repository's x-forward, y-width, z-up world frame.
This module is dependency-light and does not import Isaac Sim.

The product mass and contact coefficients below are simulation assumptions for
the first engineering proof.  They are not measurements of the catalog item's
paperboard, contents, or packaging materials.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from simulator.environment.aisle_builder import AisleLayout, Box
from simulator.environment.retail_catalog import load_retail_catalog


RESTOCKING_ROOT = "/World/Restocking"
PASTA_BOX_ASSET_KEY = "pasta_box"
PASTA_BOX_DIMENSIONS_M = (0.085, 0.060, 0.275)

# The supplied biped footprint is approximately 0.51 x 0.76 m.  Route checks
# deliberately use the larger dimension on both axes so clearance does not
# depend on the robot's yaw at a sampled point.
CONSERVATIVE_ROBOT_PLANAR_BOUNDS_M = (0.76, 0.76)
CONSERVATIVE_ROBOT_HEIGHT_M = 1.80

# Engineering assumptions for a partly filled retail pasta carton.  These are
# chosen to be plausible and numerically well behaved, not presented as
# measured hardware/material properties.
ASSUMED_PRODUCT_MASS_KG = 0.35
ASSUMED_PRODUCT_STATIC_FRICTION = 0.55
ASSUMED_PRODUCT_DYNAMIC_FRICTION = 0.45
ASSUMED_PRODUCT_RESTITUTION = 0.05


Vec3 = tuple[float, float, float]
QuaternionXyzw = tuple[float, float, float, float]


@dataclass(frozen=True)
class WorldPose:
    position_m: Vec3
    orientation_xyzw: QuaternionXyzw


@dataclass(frozen=True)
class AxisAlignedBox:
    """World-frame box used for fixtures, supports, and clearance envelopes."""

    name: str
    prim_path: str
    center_m: Vec3
    size_m: Vec3
    kind: str
    collision_enabled: bool = True

    @property
    def minimum_m(self) -> Vec3:
        return tuple(
            center - size / 2.0 for center, size in zip(self.center_m, self.size_m)
        )  # type: ignore[return-value]

    @property
    def maximum_m(self) -> Vec3:
        return tuple(
            center + size / 2.0 for center, size in zip(self.center_m, self.size_m)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class DoorwaySpec:
    center_m: Vec3
    clear_width_m: float
    clear_height_m: float
    wall_thickness_m: float


@dataclass(frozen=True)
class SupportSpec:
    name: str
    fixture: AxisAlignedBox
    usable_center_xy_m: tuple[float, float]
    usable_size_xy_m: tuple[float, float]
    surface_z_m: float


@dataclass(frozen=True)
class NavigationRoute:
    name: str
    waypoints_world_m: tuple[Vec3, ...]


@dataclass(frozen=True)
class ProductPhysicsSpec:
    asset_key: str
    dimensions_m: Vec3
    mass_kg: float
    center_of_mass_m: Vec3
    static_friction: float
    dynamic_friction: float
    restitution: float
    source_reset_pose: WorldPose
    destination_support_pose: WorldPose
    rigid_body_prim_path: str = f"{RESTOCKING_ROOT}/Product"
    visual_prim_path: str = f"{RESTOCKING_ROOT}/Product/Visual"
    collider_prim_path: str = f"{RESTOCKING_ROOT}/Product/Collider"
    assumption_note: str = (
        "Mass and friction are simulation engineering assumptions; they are not "
        "measured pasta-box or packaging-material values."
    )


@dataclass(frozen=True)
class RestockingLayout:
    """Complete pure specification consumed by later runner/controller work."""

    root_prim_path: str
    fixtures: tuple[AxisAlignedBox, ...]
    doorway: DoorwaySpec
    pickup_support: SupportSpec
    destination_support: SupportSpec
    pickup_manipulation_clearance: AxisAlignedBox
    destination_manipulation_clearance: AxisAlignedBox
    product: ProductPhysicsSpec
    store_to_pickup_route: NavigationRoute
    pickup_to_destination_route: NavigationRoute
    asset_manifest_path: str
    seed: int

    @property
    def known_pickup_pose(self) -> WorldPose:
        return self.product.source_reset_pose

    @property
    def known_shelf_target_pose(self) -> WorldPose:
        return self.product.destination_support_pose


def _find_unique_box(layout: AisleLayout, name: str) -> Box:
    matches = tuple(box for box in layout.primitives if box.name == name)
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one aisle primitive named {name!r}, got {len(matches)}")
    return matches[0]


def _fixture(
    name: str,
    path: str,
    center_m: Vec3,
    size_m: Vec3,
    kind: str,
) -> AxisAlignedBox:
    return AxisAlignedBox(name, path, center_m, size_m, kind, True)


def _robot_volume_intersects_box(
    point: Vec3,
    box: AxisAlignedBox,
    half_extent_xy: tuple[float, float],
) -> bool:
    minimum = box.minimum_m
    maximum = box.maximum_m
    return (
        minimum[0] - half_extent_xy[0] <= point[0] <= maximum[0] + half_extent_xy[0]
        and minimum[1] - half_extent_xy[1] <= point[1] <= maximum[1] + half_extent_xy[1]
        and minimum[2] < point[2] + CONSERVATIVE_ROBOT_HEIGHT_M
        and maximum[2] > point[2]
    )


def _sample_route(route: NavigationRoute, spacing_m: float = 0.025) -> tuple[Vec3, ...]:
    samples: list[Vec3] = []
    for start, end in zip(route.waypoints_world_m, route.waypoints_world_m[1:]):
        distance = math.dist(start[:2], end[:2])
        steps = max(1, int(math.ceil(distance / spacing_m)))
        for index in range(steps):
            alpha = index / steps
            samples.append(tuple(
                start[axis] + alpha * (end[axis] - start[axis]) for axis in range(3)
            ))
    samples.append(route.waypoints_world_m[-1])
    return tuple(samples)


def route_has_conservative_clearance(layout: RestockingLayout, route: NavigationRoute) -> bool:
    """Check a yaw-independent 0.76 m square footprint along a route."""

    half_extent = tuple(value / 2.0 for value in CONSERVATIVE_ROBOT_PLANAR_BOUNDS_M)
    obstacles = tuple(
        box
        for box in layout.fixtures
        if box.collision_enabled and box.kind != "floor"
    )
    return not any(
        _robot_volume_intersects_box(sample, obstacle, half_extent)
        for sample in _sample_route(route)
        for obstacle in obstacles
    )


def _support_contains_product(support: SupportSpec, pose: WorldPose, dimensions_m: Vec3) -> bool:
    x, y, z = pose.position_m
    width_x, width_y = support.usable_size_xy_m
    center_x, center_y = support.usable_center_xy_m
    return (
        abs(x - center_x) + dimensions_m[0] / 2.0 <= width_x / 2.0 + 1e-9
        and abs(y - center_y) + dimensions_m[1] / 2.0 <= width_y / 2.0 + 1e-9
        and math.isclose(z - dimensions_m[2] / 2.0, support.surface_z_m, abs_tol=1e-9)
    )


def _clearance_contains_product(clearance: AxisAlignedBox, pose: WorldPose, dimensions_m: Vec3) -> bool:
    minimum = clearance.minimum_m
    maximum = clearance.maximum_m
    return all(
        minimum[axis] <= pose.position_m[axis] - dimensions_m[axis] / 2.0 + 1e-9
        and pose.position_m[axis] + dimensions_m[axis] / 2.0 <= maximum[axis] + 1e-9
        for axis in range(3)
    )


def validate_restocking_layout(layout: RestockingLayout) -> None:
    """Reject inconsistent geometry before any Isaac stage is authored."""

    errors: list[str] = []
    if layout.root_prim_path != RESTOCKING_ROOT:
        errors.append(f"root_prim_path must be {RESTOCKING_ROOT}")
    if len({box.prim_path for box in layout.fixtures}) != len(layout.fixtures):
        errors.append("fixture prim paths must be unique")
    if any(not box.prim_path.startswith(f"{RESTOCKING_ROOT}/") for box in layout.fixtures):
        errors.append("all fixture prim paths must be below /World/Restocking")
    if any(not all(math.isfinite(v) and v > 0.0 for v in box.size_m) for box in layout.fixtures):
        errors.append("fixture dimensions must be finite and positive")

    required_width = max(CONSERVATIVE_ROBOT_PLANAR_BOUNDS_M)
    if layout.doorway.clear_width_m < required_width:
        errors.append("doorway is narrower than the conservative robot footprint")
    if layout.doorway.clear_height_m < CONSERVATIVE_ROBOT_HEIGHT_M:
        errors.append("doorway is lower than the conservative robot height")

    product = layout.product
    if product.asset_key != PASTA_BOX_ASSET_KEY:
        errors.append("first manipulation product must be the pasta_box catalog item")
    if product.dimensions_m != PASTA_BOX_DIMENSIONS_M:
        errors.append("pasta_box dimensions do not match the catalog manifest")
    product_paths = (
        product.rigid_body_prim_path,
        product.visual_prim_path,
        product.collider_prim_path,
    )
    if any(not path.startswith(f"{RESTOCKING_ROOT}/") for path in product_paths):
        errors.append("all product prim paths must be below /World/Restocking")
    if len(set(product_paths)) != len(product_paths):
        errors.append("product rigid-body, visual, and collider prim paths must be distinct")
    if not math.isfinite(product.mass_kg) or product.mass_kg <= 0.0:
        errors.append("product mass must be finite and positive")
    if not (0.0 <= product.dynamic_friction <= product.static_friction):
        errors.append("product friction must satisfy 0 <= dynamic <= static")
    if not (0.0 <= product.restitution <= 1.0):
        errors.append("product restitution must be in [0, 1]")
    for label, pose in (
        ("source reset", product.source_reset_pose),
        ("destination support", product.destination_support_pose),
    ):
        if not all(math.isfinite(value) for value in (*pose.position_m, *pose.orientation_xyzw)):
            errors.append(f"{label} pose must contain only finite values")
            continue
        quaternion_norm = math.sqrt(sum(value * value for value in pose.orientation_xyzw))
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-9):
            errors.append(f"{label} orientation must be a unit quaternion")

    support_checks = (
        ("pickup", layout.pickup_support, product.source_reset_pose, layout.pickup_manipulation_clearance),
        (
            "destination",
            layout.destination_support,
            product.destination_support_pose,
            layout.destination_manipulation_clearance,
        ),
    )
    for label, support, pose, clearance in support_checks:
        if not _support_contains_product(support, pose, product.dimensions_m):
            errors.append(f"{label} support does not fully support the product pose")
        if not _clearance_contains_product(clearance, pose, product.dimensions_m):
            errors.append(f"{label} manipulation envelope does not contain the product")
        if clearance.minimum_m[2] < support.surface_z_m - 1e-9:
            errors.append(f"{label} manipulation envelope extends below the support surface")

    if product.source_reset_pose != layout.known_pickup_pose:
        errors.append("known pickup pose must equal the deterministic source reset pose")
    if len(layout.store_to_pickup_route.waypoints_world_m) < 2:
        errors.append("store-to-pickup route requires at least two waypoints")
    if len(layout.pickup_to_destination_route.waypoints_world_m) < 2:
        errors.append("pickup-to-destination route requires at least two waypoints")
    if not route_has_conservative_clearance(layout, layout.store_to_pickup_route):
        errors.append("store-to-pickup route lacks conservative robot clearance")
    if not route_has_conservative_clearance(layout, layout.pickup_to_destination_route):
        errors.append("pickup-to-destination route lacks conservative robot clearance")

    if errors:
        raise ValueError("Invalid restocking layout: " + "; ".join(errors))


def build_restocking_layout(aisle_layout: AisleLayout) -> RestockingLayout:
    """Attach one deterministic stockroom to the aisle's negative-x open end."""

    aisle_floor = _find_unique_box(aisle_layout, "floor")
    destination_shelf = _find_unique_box(aisle_layout, "shelf_r1_b0_l1")
    catalog = load_retail_catalog(aisle_layout.asset_manifest_path)
    product_record = catalog.by_key(PASTA_BOX_ASSET_KEY)
    if product_record.dimensions_m != PASTA_BOX_DIMENSIONS_M:
        raise ValueError(
            f"pasta_box catalog dimensions changed: {product_record.dimensions_m!r}"
        )

    floor_surface_z = aisle_floor.center_m[2] + aisle_floor.size_m[2] / 2.0
    open_end_x = aisle_floor.center_m[0] - aisle_floor.size_m[0] / 2.0
    room_length = 4.8
    room_half_width = 2.2
    room_height = 2.8
    wall_thickness = 0.12
    doorway_width = 1.50
    doorway_height = 2.20
    room_center_x = open_end_x - room_length / 2.0
    back_wall_x = open_end_x - room_length

    fixtures: list[AxisAlignedBox] = [
        _fixture(
            "stockroom_floor",
            f"{RESTOCKING_ROOT}/Room/Floor",
            (room_center_x, 0.0, floor_surface_z - 0.05),
            (room_length, room_half_width * 2.0, 0.10),
            "floor",
        ),
        _fixture(
            "stockroom_back_wall",
            f"{RESTOCKING_ROOT}/Room/Walls/Back",
            (back_wall_x, 0.0, floor_surface_z + room_height / 2.0),
            (wall_thickness, room_half_width * 2.0, room_height),
            "wall",
        ),
        _fixture(
            "stockroom_left_wall",
            f"{RESTOCKING_ROOT}/Room/Walls/Left",
            (room_center_x, -room_half_width, floor_surface_z + room_height / 2.0),
            (room_length, wall_thickness, room_height),
            "wall",
        ),
        _fixture(
            "stockroom_right_wall",
            f"{RESTOCKING_ROOT}/Room/Walls/Right",
            (room_center_x, room_half_width, floor_surface_z + room_height / 2.0),
            (room_length, wall_thickness, room_height),
            "wall",
        ),
    ]

    entry_segment_width = room_half_width - doorway_width / 2.0
    for label, sign in (("Left", -1.0), ("Right", 1.0)):
        fixtures.append(_fixture(
            f"stockroom_entry_{label.lower()}",
            f"{RESTOCKING_ROOT}/Room/Walls/Entry{label}",
            (
                open_end_x,
                sign * (doorway_width / 2.0 + entry_segment_width / 2.0),
                floor_surface_z + room_height / 2.0,
            ),
            (wall_thickness, entry_segment_width, room_height),
            "wall",
        ))
    fixtures.append(_fixture(
        "stockroom_entry_header",
        f"{RESTOCKING_ROOT}/Room/Walls/EntryHeader",
        (
            open_end_x,
            0.0,
            floor_surface_z + doorway_height + (room_height - doorway_height) / 2.0,
        ),
        (wall_thickness, doorway_width, room_height - doorway_height),
        "wall",
    ))

    pickup_center_x = open_end_x - 3.35
    pickup_support_height = 0.78
    pickup_board = _fixture(
        "pickup_support",
        f"{RESTOCKING_ROOT}/Pickup/Support",
        (pickup_center_x, 1.15, pickup_support_height - 0.04),
        (0.90, 0.70, 0.08),
        "shelf",
    )
    fixtures.append(pickup_board)
    for label, x_offset in (("Rear", -0.32), ("Front", 0.32)):
        fixtures.append(_fixture(
            f"pickup_leg_{label.lower()}",
            f"{RESTOCKING_ROOT}/Pickup/Legs/{label}",
            (pickup_center_x + x_offset, 1.29, pickup_support_height / 2.0 - 0.04),
            (0.09, 0.09, pickup_support_height - 0.08),
            "upright",
        ))
    pickup_support = SupportSpec(
        "pickup",
        pickup_board,
        (pickup_center_x, 1.15),
        (0.74, 0.54),
        pickup_support_height,
    )

    shelf_front_y = destination_shelf.center_m[1] - destination_shelf.size_m[1] / 2.0
    destination_depth = 0.40
    destination_board = _fixture(
        "destination_support",
        f"{RESTOCKING_ROOT}/Destination/Support",
        (
            destination_shelf.center_m[0],
            shelf_front_y - destination_depth / 2.0,
            destination_shelf.center_m[2],
        ),
        (0.76, destination_depth, destination_shelf.size_m[2]),
        "shelf",
    )
    fixtures.append(destination_board)
    destination_surface_z = destination_board.maximum_m[2]
    destination_support = SupportSpec(
        "destination",
        destination_board,
        (destination_board.center_m[0], destination_board.center_m[1]),
        (0.60, 0.30),
        destination_surface_z,
    )

    source_pose = WorldPose(
        (
            pickup_center_x,
            0.96,
            pickup_support.surface_z_m + PASTA_BOX_DIMENSIONS_M[2] / 2.0,
        ),
        (0.0, 0.0, 1.0, 0.0),
    )
    destination_pose = WorldPose(
        (
            destination_board.center_m[0],
            destination_board.center_m[1] - 0.08,
            destination_surface_z + PASTA_BOX_DIMENSIONS_M[2] / 2.0,
        ),
        (0.0, 0.0, 1.0, 0.0),
    )
    product = ProductPhysicsSpec(
        asset_key=PASTA_BOX_ASSET_KEY,
        dimensions_m=PASTA_BOX_DIMENSIONS_M,
        mass_kg=ASSUMED_PRODUCT_MASS_KG,
        center_of_mass_m=(0.0, 0.0, 0.0),
        static_friction=ASSUMED_PRODUCT_STATIC_FRICTION,
        dynamic_friction=ASSUMED_PRODUCT_DYNAMIC_FRICTION,
        restitution=ASSUMED_PRODUCT_RESTITUTION,
        source_reset_pose=source_pose,
        destination_support_pose=destination_pose,
    )

    pickup_clearance = AxisAlignedBox(
        "pickup_manipulation_clearance",
        f"{RESTOCKING_ROOT}/Pickup/ManipulationClearance",
        (pickup_center_x, 0.94, pickup_support.surface_z_m + 0.36),
        (0.72, 0.64, 0.72),
        "clearance",
        False,
    )
    destination_clearance = AxisAlignedBox(
        "destination_manipulation_clearance",
        f"{RESTOCKING_ROOT}/Destination/ManipulationClearance",
        (
            destination_pose.position_m[0],
            destination_pose.position_m[1],
            destination_surface_z + 0.36,
        ),
        (0.62, 0.52, 0.72),
        "clearance",
        False,
    )

    store_to_pickup = NavigationRoute(
        "store_to_pickup",
        (
            (open_end_x + 1.60, 0.0, floor_surface_z),
            (open_end_x + 0.55, 0.0, floor_surface_z),
            (open_end_x - 0.55, 0.0, floor_surface_z),
            (open_end_x - 2.10, 0.0, floor_surface_z),
            (pickup_center_x, 0.20, floor_surface_z),
        ),
    )
    pickup_to_destination = NavigationRoute(
        "pickup_to_destination",
        (
            (pickup_center_x, 0.20, floor_surface_z),
            (open_end_x - 2.10, 0.0, floor_surface_z),
            (open_end_x - 0.55, 0.0, floor_surface_z),
            (open_end_x + 0.55, 0.0, floor_surface_z),
            (destination_pose.position_m[0], 0.35, floor_surface_z),
        ),
    )

    layout = RestockingLayout(
        root_prim_path=RESTOCKING_ROOT,
        fixtures=tuple(fixtures),
        doorway=DoorwaySpec(
            (open_end_x, 0.0, floor_surface_z + doorway_height / 2.0),
            doorway_width,
            doorway_height,
            wall_thickness,
        ),
        pickup_support=pickup_support,
        destination_support=destination_support,
        pickup_manipulation_clearance=pickup_clearance,
        destination_manipulation_clearance=destination_clearance,
        product=product,
        store_to_pickup_route=store_to_pickup,
        pickup_to_destination_route=pickup_to_destination,
        asset_manifest_path=aisle_layout.asset_manifest_path,
        seed=aisle_layout.seed,
    )
    validate_restocking_layout(layout)
    return layout


def with_doorway_width(layout: RestockingLayout, clear_width_m: float) -> RestockingLayout:
    """Test/support helper that returns a candidate layout for validation."""

    return replace(layout, doorway=replace(layout.doorway, clear_width_m=clear_width_m))
