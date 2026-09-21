"""Run the grocery aisle in the installed Isaac Sim 6.x runtime.

This module is deliberately an Isaac-side entry point.  The dependency-light
configuration and geometry modules remain importable with ordinary Python,
while this file is launched with ``C:\\isaacsim\\python.bat``.  Sensor data is
published by Isaac's ROS 2 bridge; the only Python publisher here is the
ground-truth pose topic, which is intentionally separate from SLAM odometry.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import traceback
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SHOPPER_CART_START_X_M = 11.5
SHOPPER_CART_CRUISE_SPEED_MPS = 0.65
SHOPPER_CART_DECEL_START_S = 17.75
SHOPPER_CART_STOP_S = 20.5
# Furthest authored local-X extent: basket-end center 1.13 m + 0.0125 m half-width.
SHOPPER_CART_FRONT_OFFSET_M = 1.1425


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the grocery aisle Isaac Sim sensor runtime")
    parser.add_argument("--scenario", default=str(REPO_ROOT / "config/scenarios/walking_baseline.yaml"))
    parser.add_argument("--frames", type=int, default=0, help="Simulation frames; 0 derives the count from the trajectory duration")
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim viewport")
    parser.add_argument("--realtime", action="store_true", help="Pace the simulation at 60 Hz for external ROS/dashboard consumers")
    parser.add_argument("--realtime-factor", type=float, default=1.0, help="Maximum simulation/wall-time ratio when --realtime is enabled")
    parser.add_argument("--renderer", default="RaytracedLighting")
    parser.add_argument("--status-path", default=str(REPO_ROOT / "runs/isaac_runtime_status.json"))
    return parser.parse_args()


def _sim_time_message(seconds: float):
    from builtin_interfaces.msg import Time

    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


def lidar_runtime_spec(lidar_config) -> dict[str, object]:
    """Map the ROS sensor contract to the Isaac 6.1 LiDAR boundary.

    Scenario poses are expressed relative to ``sensor_rig`` in ROS ``xyzw``
    order.  Isaac's native RTX LiDAR API takes a local mount pose in ``wxyz``
    order.  Keeping both representations in the returned spec makes the
    boundary and its frame semantics inspectable in run artifacts.
    """
    from simulator.sensors.transforms import quaternion_from_rpy_deg, quaternion_xyzw_to_wxyz

    orientation_xyzw = quaternion_from_rpy_deg(*lidar_config.pose_in_rig.rpy_deg)
    return {
        "tick_rate_hz": float(lidar_config.hz),
        "near_range_m": float(lidar_config.min_range_m),
        "far_range_m": float(lidar_config.max_range_m),
        "translation_m": list(lidar_config.pose_in_rig.position_m),
        "orientation_xyzw_ros": list(orientation_xyzw),
        "orientation_wxyz_isaac": list(quaternion_xyzw_to_wxyz(orientation_xyzw)),
        "mount_frame": "sensor_rig",
        "mount_semantics": "rig_relative",
        "world_pose_semantics": "sensor_rig_world_pose_composed_with_local_mount",
        "sensor_asset": "Example_Rotary",
        "scan_pattern_source": "Isaac Sim 6.1 Example_Rotary asset",
    }


def _timestamp_summary(reference: list[float], samples: list[float]) -> dict[str, float | int | None]:
    if not reference or not samples:
        return {"sample_count": len(samples), "mean_offset_s": None, "max_abs_offset_s": None}
    offsets = [min((value - ref for ref in reference), key=abs) for value in samples]
    return {
        "sample_count": len(samples),
        "mean_offset_s": sum(offsets) / len(offsets),
        "max_abs_offset_s": max(abs(value) for value in offsets),
    }


def _point_count_summary(samples: list[int]) -> dict[str, float | int | None]:
    if not samples:
        return {"sample_count": 0, "min_points": None, "max_points": None, "mean_points": None}
    return {
        "sample_count": len(samples),
        "min_points": min(samples),
        "max_points": max(samples),
        "mean_points": sum(samples) / len(samples),
    }


def _set_name_override(prim, name: str) -> None:
    from pxr import Sdf

    attr = prim.GetAttribute("isaac:nameOverride")
    if not attr:
        attr = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    attr.Set(name)


def _define_xform(stage, path: str, frame_name: str):
    from pxr import UsdGeom

    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    _set_name_override(prim, frame_name)
    return prim


def shopper_cart_position_at_time(timestamp_s: float) -> tuple[float, float, float]:
    """Deterministic aisle-space pose for the contextual shopper and cart."""

    if not math.isfinite(timestamp_s) or timestamp_s < 0.0:
        raise ValueError("timestamp_s must be finite and non-negative")
    if timestamp_s <= SHOPPER_CART_DECEL_START_S:
        x_m = SHOPPER_CART_START_X_M + SHOPPER_CART_CRUISE_SPEED_MPS * timestamp_s
    else:
        # Integrate a smoothstep velocity ramp from cruise speed to rest.  The
        # clamped phase keeps the complete assembly stationary after the
        # production horizon without a position, velocity, or acceleration
        # discontinuity at either end of the stop.
        stop_duration_s = SHOPPER_CART_STOP_S - SHOPPER_CART_DECEL_START_S
        phase = min((timestamp_s - SHOPPER_CART_DECEL_START_S) / stop_duration_s, 1.0)
        integrated_velocity = phase - phase**3 + 0.5 * phase**4
        x_m = (
            SHOPPER_CART_START_X_M
            + SHOPPER_CART_CRUISE_SPEED_MPS * SHOPPER_CART_DECEL_START_S
            + SHOPPER_CART_CRUISE_SPEED_MPS * stop_duration_s * integrated_velocity
        )
    return (x_m, 0.0, 0.0)


def _preview_material(stage, path: str, color, roughness: float, metallic: float = 0.0):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _build_shopper_cart(stage):
    """Author one restrained code-native shopper/cart depth cue."""

    from pxr import Gf, Sdf, UsdGeom, UsdShade

    root_path = "/World/GroceryAisle/dynamic/shopper_cart"
    root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    root.CreateAttribute("grocery:dynamic", Sdf.ValueTypeNames.Bool).Set(True)
    root.CreateAttribute("grocery:semantic_id", Sdf.ValueTypeNames.String).Set("context/shopper_cart/primary")
    root_api = UsdGeom.XformCommonAPI(root)
    root_api.SetTranslate(Gf.Vec3d(*shopper_cart_position_at_time(0.0)))

    clothing = _preview_material(stage, "/World/GroceryAisle/looks/shopper_clothing", (0.035, 0.085, 0.13), 0.62)
    denim = _preview_material(stage, "/World/GroceryAisle/looks/shopper_denim", (0.055, 0.11, 0.20), 0.68)
    skin = _preview_material(stage, "/World/GroceryAisle/looks/shopper_skin", (0.46, 0.27, 0.18), 0.72)
    cart_metal = _preview_material(stage, "/World/GroceryAisle/looks/cart_metal", (0.12, 0.17, 0.18), 0.24, 0.72)
    wheel_material = _preview_material(stage, "/World/GroceryAisle/looks/cart_wheel", (0.018, 0.021, 0.024), 0.70)

    def bind(prim, material) -> None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

    def cube(name: str, center, size, material) -> None:
        prim = UsdGeom.Cube.Define(stage, f"{root_path}/{name}").GetPrim()
        UsdGeom.Cube(prim).GetSizeAttr().Set(1.0)
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(*center))
        api.SetScale(Gf.Vec3f(*size))
        bind(prim, material)

    def capsule(name: str, center, radius: float, height: float, material) -> None:
        geometry = UsdGeom.Capsule.Define(stage, f"{root_path}/{name}")
        geometry.GetRadiusAttr().Set(radius)
        geometry.GetHeightAttr().Set(height)
        geometry.GetAxisAttr().Set(UsdGeom.Tokens.z)
        UsdGeom.XformCommonAPI(geometry.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        bind(geometry.GetPrim(), material)

    def sphere(name: str, center, radius: float, material) -> None:
        geometry = UsdGeom.Sphere.Define(stage, f"{root_path}/{name}")
        geometry.GetRadiusAttr().Set(radius)
        UsdGeom.XformCommonAPI(geometry.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        bind(geometry.GetPrim(), material)

    def wheel(name: str, center) -> None:
        geometry = UsdGeom.Cylinder.Define(stage, f"{root_path}/{name}")
        geometry.GetRadiusAttr().Set(0.09)
        geometry.GetHeightAttr().Set(0.055)
        geometry.GetAxisAttr().Set(UsdGeom.Tokens.y)
        UsdGeom.XformCommonAPI(geometry.GetPrim()).SetTranslate(Gf.Vec3d(*center))
        bind(geometry.GetPrim(), wheel_material)

    # Human silhouette, facing +X with hands resting at the cart handle.
    capsule("torso", (0.0, 0.0, 1.28), 0.20, 0.56, clothing)
    sphere("head", (0.0, 0.0, 1.78), 0.16, skin)
    capsule("leg_left", (-0.02, -0.115, 0.55), 0.075, 0.67, denim)
    capsule("leg_right", (0.02, 0.115, 0.55), 0.075, 0.67, denim)
    capsule("arm_left", (0.20, -0.24, 1.20), 0.055, 0.48, clothing)
    capsule("arm_right", (0.20, 0.24, 1.20), 0.055, 0.48, clothing)

    # Open cart rails preserve sightlines and read better than a solid box.
    for rail_index, z in enumerate((0.68, 1.02)):
        cube(f"basket_side_l_{rail_index}", (0.72, -0.32, z), (0.82, 0.025, 0.035), cart_metal)
        cube(f"basket_side_r_{rail_index}", (0.72, 0.32, z), (0.82, 0.025, 0.035), cart_metal)
        cube(f"basket_end_f_{rail_index}", (1.13, 0.0, z), (0.025, 0.64, 0.035), cart_metal)
        cube(f"basket_end_b_{rail_index}", (0.31, 0.0, z), (0.025, 0.64, 0.035), cart_metal)
    for corner_index, (x, y) in enumerate(((0.31, -0.32), (0.31, 0.32), (1.13, -0.32), (1.13, 0.32))):
        cube(f"basket_post_{corner_index}", (x, y, 0.85), (0.025, 0.025, 0.36), cart_metal)
    cube("cart_base", (0.78, 0.0, 0.37), (0.76, 0.50, 0.035), cart_metal)
    cube("cart_handle", (0.20, 0.0, 1.10), (0.035, 0.76, 0.045), cart_metal)
    for wheel_index, (x, y) in enumerate(((0.46, -0.28), (0.46, 0.28), (1.04, -0.28), (1.04, 0.28))):
        wheel(f"wheel_{wheel_index}", (x, y, 0.19))
    return root_api


def _build_world(stage, scenario):
    from pxr import Gf, UsdGeom, UsdLux

    from simulator.environment.aisle_builder import build_aisle_layout
    from simulator.environment.isaac_builder import IsaacAisleBuilder

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)

    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    _set_name_override(world, "sim_world")
    layout = build_aisle_layout(scenario.environment)
    primitive_count = IsaacAisleBuilder(stage=stage).build(layout, "/World/GroceryAisle")

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(float(scenario.environment.lighting_lux) * 0.35)
    dome.GetColorAttr().Set(Gf.Vec3f(0.78, 0.82, 0.86))
    for panel_index, x in enumerate((2.2, 5.2, 8.2, 11.2, 14.2, 17.2, 20.2, 23.2)):
        for side_index, y in enumerate((-0.95, 0.95)):
            light = UsdLux.RectLight.Define(stage, f"/World/GroceryAisle/lighting/panel_{panel_index:02d}_{side_index}")
            light.GetWidthAttr().Set(0.84)
            light.GetHeightAttr().Set(0.30)
            light.GetIntensityAttr().Set(float(scenario.environment.lighting_lux) * 8.0)
            light.GetColorAttr().Set(Gf.Vec3f(1.0, 0.91, 0.76))
            light.GetNormalizeAttr().Set(True)
            UsdGeom.XformCommonAPI(light).SetTranslate(Gf.Vec3d(x, y, 2.94))
        # RaytracedLighting has limited indirect bounce in this capture path.
        # A small invisible all-direction fixture at each ceiling bay supplies
        # the neutral shelf-face fill that the physical ceiling would reflect.
        fill = UsdLux.SphereLight.Define(stage, f"/World/GroceryAisle/lighting/fill_{panel_index:02d}")
        fill.GetRadiusAttr().Set(0.16)
        fill.GetIntensityAttr().Set(float(scenario.environment.lighting_lux) * 5.5)
        fill.GetColorAttr().Set(Gf.Vec3f(0.82, 0.88, 1.0))
        fill.GetNormalizeAttr().Set(True)
        UsdGeom.XformCommonAPI(fill).SetTranslate(Gf.Vec3d(x, 0.0, 2.72))

    shopper_cart = _build_shopper_cart(stage)

    return layout, primitive_count, shopper_cart


def _build_sensor_rig(stage, scenario):
    from pxr import Gf, Sdf, UsdGeom
    from simulator.sensors.transforms import camera_usd_quaternion, rpy_deg_from_quaternion

    rig_path = "/World/SensorRig"
    camera_link_path = "/World/SensorRig/camera_link"
    camera_path = "/World/SensorRig/camera_link/camera_optical_frame"
    lidar_path = "/World/SensorRig/lidar_link"

    rig = _define_xform(stage, rig_path, "sensor_rig")
    camera_link = _define_xform(stage, camera_link_path, "camera_link")
    # The LiDAR path is intentionally left undefined here.  Lidar.create()
    # authors the real OmniLidar prim at this path, including the RTX sensor
    # schema, so it must not be shadowed by a plain Xform.

    rig_api = UsdGeom.XformCommonAPI(rig)
    rig_api.SetTranslate(Gf.Vec3d(*scenario.trajectory.start_position_m))
    rig_api.SetRotate(Gf.Vec3f(0.0, 0.0, scenario.trajectory.yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera_api = UsdGeom.XformCommonAPI(camera_link)
    camera_api.SetTranslate(Gf.Vec3d(*scenario.camera.pose_in_rig.position_m))
    camera_api.SetRotate(Gf.Vec3f(*scenario.camera.pose_in_rig.rpy_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # Isaac cameras look along local -Z.  This rotation makes the optical
    # axis point down the aisle (+X), with image up aligned to world +Z.
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_api = UsdGeom.XformCommonAPI(camera.GetPrim())
    camera_api.SetRotate(Gf.Vec3f(*rpy_deg_from_quaternion(camera_usd_quaternion())), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(20.955 * scenario.camera.height_px / scenario.camera.width_px)
    focal_length_mm = 20.955 / (2.0 * math.tan(math.radians(scenario.camera.horizontal_fov_deg) / 2.0))
    camera.GetFocalLengthAttr().Set(focal_length_mm)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(scenario.camera.near_m, scenario.camera.far_m))
    _set_name_override(camera.GetPrim(), "camera_optical_frame")

    # Author explicit frame metadata for consumers that inspect the USD stage.
    stage.GetPrimAtPath(rig_path).CreateAttribute("grocery:frame_id", Sdf.ValueTypeNames.String).Set("sensor_rig")
    stage.GetPrimAtPath(camera_path).CreateAttribute("grocery:frame_id", Sdf.ValueTypeNames.String).Set("camera_optical_frame")
    return rig, camera_path, lidar_path


def _create_clock_graph():
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph_path = "/GroceryROS2Graph"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def _create_camera_graph(camera_path: str, width: int, height: int, fps: float):
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    step = max(1, round(60.0 / float(fps)))
    publisher_queue_size = 120
    rgb_qos_profile = json.dumps({
        "history": "keepAll",
        "depth": 0,
        "reliability": "reliable",
        "durability": "volatile",
        "deadline": 0.0,
        "lifespan": 0.0,
        "liveliness": "systemDefault",
        "leaseDuration": 0.0,
    }, separators=(",", ":"))
    graph_path = "/GroceryCameraGraph"
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "Rgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "Rgb.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("CreateRenderProduct.inputs:width", int(width)),
                ("CreateRenderProduct.inputs:height", int(height)),
                ("Rgb.inputs:frameId", "camera_optical_frame"),
                ("Rgb.inputs:topicName", "/sim/camera/rgb/image_raw"),
                ("Rgb.inputs:type", "rgb"),
                ("Rgb.inputs:frameSkipCount", step - 1),
                ("Rgb.inputs:queueSize", publisher_queue_size),
                ("Rgb.inputs:qosProfile", rgb_qos_profile),
                ("CameraInfo.inputs:frameId", "camera_optical_frame"),
                ("CameraInfo.inputs:topicName", "/sim/camera/rgb/camera_info"),
                ("CameraInfo.inputs:frameSkipCount", step - 1),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    return {
        "requested_fps": float(fps),
        "frame_skip_count": step - 1,
        "effective_fps": 60.0 / step,
        "publisher_queue_size": publisher_queue_size,
        "publisher_qos": "reliable_keep_all",
        "mode": "ros2_camera_helper_frameSkipCount",
    }


def _create_lidar(lidar_path: str, lidar_config, topic: str):
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
    from pxr import Sdf

    spec = lidar_runtime_spec(lidar_config)
    # Create the current native OmniLidar prim locally.  The downloadable
    # Example_Rotary asset is optional and may not be present on a workstation;
    # the schema-created sensor is still a real RTX LiDAR with scenario-derived
    # scan bounds, cadence, and pose.
    lidar = Lidar.create(
        path=lidar_path,
        config="Example_Rotary",
        accumulate_outputs=True,
        tick_rate=spec["tick_rate_hz"],
        translations=[spec["translation_m"]],
        orientations=[spec["orientation_wxyz_isaac"]],
        attributes={
            "omni:sensor:Core:scanRateBaseHz": spec["tick_rate_hz"],
            "omni:sensor:Core:nearRangeM": spec["near_range_m"],
            "omni:sensor:Core:farRangeM": spec["far_range_m"],
            "omni:sensor:Core:patternFiringRateHz": 20000,
        },
    )
    prim = lidar.prims[0]
    name_override = prim.GetAttribute("isaac:nameOverride")
    if not name_override:
        name_override = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    name_override.Set("lidar_link")
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName=topic,
        frameId="lidar_link",
    )
    return lidar, sensor


def _publish_ground_truth(publisher, sample, timestamp_s: float):
    from geometry_msgs.msg import PoseStamped
    from simulator.ros.topic_contract import GROUND_TRUTH_FRAME

    message = PoseStamped()
    message.header.stamp = _sim_time_message(timestamp_s)
    message.header.frame_id = GROUND_TRUTH_FRAME
    message.pose.position.x, message.pose.position.y, message.pose.position.z = sample.position_m
    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = sample.orientation_xyzw
    publisher.publish(message)


def _publish_tf(publisher, sample, timestamp_s: float) -> None:
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from simulator.ros.topic_contract import FRAMES

    stamp = _sim_time_message(timestamp_s)
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = FRAMES["sim_world"]
    message.child_frame_id = FRAMES["truth_sensor_rig"]
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = sample.position_m
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = sample.orientation_xyzw
    publisher.publish(TFMessage(transforms=[message]))


def _publish_static_tf(broadcaster, scenario) -> None:
    from geometry_msgs.msg import TransformStamped

    from simulator.ros.topic_contract import FRAMES
    from simulator.sensors.transforms import camera_optical_quaternion, quaternion_from_rpy_deg

    transforms = []

    def add(parent: str, child: str, translation, rotation) -> None:
        message = TransformStamped()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = translation
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = rotation
        transforms.append(message)

    add(FRAMES["sensor_rig"], FRAMES["camera_link"], scenario.camera.pose_in_rig.position_m, quaternion_from_rpy_deg(*scenario.camera.pose_in_rig.rpy_deg))
    add(FRAMES["camera_link"], FRAMES["camera_optical"], (0.0, 0.0, 0.0), camera_optical_quaternion())
    add(FRAMES["sensor_rig"], FRAMES["lidar_link"], scenario.lidar.pose_in_rig.position_m, quaternion_from_rpy_deg(*scenario.lidar.pose_in_rig.rpy_deg))
    broadcaster.sendTransform(transforms)


def run(args: argparse.Namespace) -> dict[str, object]:
    from isaacsim import SimulationApp

    from simulator.config.loader import load_scenario
    from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory
    from simulator.ros.topic_contract import FRAMES, TOPICS
    from simulator.sensors.noise import NoiseConfig

    if args.frames < 0:
        raise ValueError("--frames must be non-negative")
    scenario = load_scenario(args.scenario)
    trajectory_cls = WalkingTrajectory if scenario.trajectory.name.lower() == "walking" else StraightTrajectory
    trajectory = trajectory_cls(scenario.trajectory)
    frames = args.frames if args.frames > 0 else max(1, math.ceil(scenario.trajectory.duration_s * 60.0))
    noise_config = NoiseConfig.from_mapping(scenario.sensor_overrides.get("noise"))

    print("[grocery-runtime] starting SimulationApp", flush=True)
    simulation_app = SimulationApp({"renderer": args.renderer, "headless": bool(args.headless)})
    node = None
    executor = None
    executor_thread = None
    runtime_ok = False
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager

        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        print("[grocery-runtime] ROS 2 bridge loaded", flush=True)
        stage_utils.set_stage_units(meters_per_unit=1.0)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        print("[grocery-runtime] building aisle USD", flush=True)
        layout, primitive_count, shopper_cart_api = _build_world(stage, scenario)
        print("[grocery-runtime] building sensor rig USD", flush=True)
        _, camera_path, lidar_path = _build_sensor_rig(stage, scenario)
        print("[grocery-runtime] creating camera ROS graph", flush=True)
        camera_cadence = _create_camera_graph(camera_path, scenario.camera.width_px, scenario.camera.height_px, scenario.camera.fps)

        # Sensor API objects must be created after the bridge extension is
        # loaded and before the simulation starts.
        print("[grocery-runtime] creating RTX LiDAR ROS writer", flush=True)
        lidar_topic = TOPICS["lidar_ideal_points"] if noise_config.enabled else TOPICS["lidar_points"]
        lidar_spec = lidar_runtime_spec(scenario.lidar)
        _, _lidar_sensor = _create_lidar(lidar_path, scenario.lidar, lidar_topic)
        print("[grocery-runtime] creating clock and TF graph", flush=True)
        _create_clock_graph()
        print("[grocery-runtime] initializing simulation", flush=True)
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        simulation_app.update()

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage
        from rclpy.executors import MultiThreadedExecutor
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        rclpy.init(args=None)
        node = rclpy.create_node("grocery_sim_ground_truth")
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        gt_pub = node.create_publisher(PoseStamped, TOPICS["ground_truth_pose"], 10)
        tf_pub = node.create_publisher(TFMessage, "/tf", 10)
        static_tf = StaticTransformBroadcaster(node)
        _publish_static_tf(static_tf, scenario)
        lidar_relay = None
        if noise_config.enabled:
            from simulator.ros.lidar_noise_relay import LidarNoiseRelay

            lidar_relay = LidarNoiseRelay(node, noise_config, scenario.seed)
        rgb_stamps: list[float] = []
        clock_stamps: list[float] = []
        ground_truth_stamps: list[float] = []
        lidar_stamps: list[float] = []
        lidar_point_counts: list[int] = []

        def _on_rgb(message) -> None:
            rgb_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)

        def _on_clock(message) -> None:
            clock_stamps.append(float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0)

        def _on_ground_truth(message) -> None:
            ground_truth_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)

        def _on_lidar(message) -> None:
            lidar_stamps.append(float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0)
            lidar_point_counts.append(int(getattr(message, "width", 0)) * int(getattr(message, "height", 0)))

        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, PointCloud2

        node.create_subscription(Image, TOPICS["rgb_image"], _on_rgb, 10)
        node.create_subscription(Clock, TOPICS["clock"], _on_clock, 10)
        node.create_subscription(PoseStamped, TOPICS["ground_truth_pose"], _on_ground_truth, 10)
        node.create_subscription(PointCloud2, TOPICS["lidar_points"], _on_lidar, 10)
        executor_thread = threading.Thread(target=executor.spin, name="grocery_sim_ros_executor", daemon=True)
        executor_thread.start()
        app_utils.play()
        print("[grocery-runtime] simulation running", flush=True)

        from pxr import Gf, UsdGeom

        rig_prim = stage.GetPrimAtPath("/World/SensorRig")
        rig_api = UsdGeom.XformCommonAPI(rig_prim)
        from omni.timeline import get_timeline_interface

        timeline = get_timeline_interface()
        last_timestamp_s: float | None = None
        render_phase_offsets_s: list[float] = []
        simulation_dt_s = 1.0 / 60.0
        for frame in range(frames):
            wall_start = time.perf_counter()
            timeline_before_s = float(timeline.get_current_time())
            # Sensor rendering and the simulation clock advance on update().
            # Command the rig for the measured next simulation tick, then
            # publish truth using the post-update timeline stamp.  This keeps
            # truth, TF, and the rendered sensor pose on one measured phase.
            commanded_timestamp_s = timeline_before_s + simulation_dt_s
            commanded_sample = trajectory.sample(commanded_timestamp_s)
            rig_api.SetTranslate(Gf.Vec3d(*commanded_sample.position_m))
            # Apply the complete sampled walking/trajectory orientation to the
            # actual USD rig.  The TF and ground-truth messages use this same
            # quaternion, so USD, ROS TF, and the pose topic remain aligned.
            from simulator.sensors.transforms import rpy_deg_from_quaternion

            sampled_rpy_deg = rpy_deg_from_quaternion(commanded_sample.orientation_xyzw)
            rig_api.SetRotate(Gf.Vec3f(*sampled_rpy_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            shopper_cart_api.SetTranslate(Gf.Vec3d(*shopper_cart_position_at_time(commanded_timestamp_s)))
            simulation_app.update()
            timestamp_s = float(timeline.get_current_time())
            if last_timestamp_s is not None and timestamp_s <= last_timestamp_s:
                raise RuntimeError("Isaac timeline did not advance after rendering; refusing synthetic timestamps")
            render_phase_offsets_s.append(timestamp_s - commanded_timestamp_s)
            sample = trajectory.sample(timestamp_s)
            _publish_ground_truth(gt_pub, sample, timestamp_s)
            _publish_tf(tf_pub, sample, timestamp_s)
            last_timestamp_s = timestamp_s
            if args.realtime:
                time.sleep(max(0.0, (simulation_dt_s / args.realtime_factor) - (time.perf_counter() - wall_start)))

        runtime_ok = True
        print("[grocery-runtime] simulation completed", flush=True)
        result = {
            "runtime": "isaac_sim",
            "isaac_version": "6.1.0",
            "scenario": scenario.name,
            "frames_simulated": frames,
            "primitive_count": primitive_count,
            "product_count": len(layout.products),
            "topics": {
                "clock": TOPICS["clock"],
                "rgb": TOPICS["rgb_image"],
                "camera_info": TOPICS["rgb_camera_info"],
                "lidar_ideal": TOPICS["lidar_ideal_points"],
                "lidar": TOPICS["lidar_points"],
                "ground_truth_pose": TOPICS["ground_truth_pose"],
            },
            "frames": list(FRAMES.values()),
            "camera_cadence": camera_cadence,
            "observed_rgb_frames": len(rgb_stamps),
            "observed_rgb_hz": (len(rgb_stamps) - 1) / (rgb_stamps[-1] - rgb_stamps[0]) if len(rgb_stamps) > 1 and rgb_stamps[-1] > rgb_stamps[0] else None,
            "clock_source": "Isaac timeline current_time",
            "realtime_factor_limit": args.realtime_factor if args.realtime else None,
            "ros_callback_service": "MultiThreadedExecutor background thread",
            "timestamp_phase": {
                "clock_reference": TOPICS["clock"],
                "truth_and_rig_source": "Isaac timeline current_time",
                "policy": "command_rig_before_update_publish_truth_after_update",
                "evaluation_policy": "interpolate_ground_truth_at_estimator_timestamps",
                "synthetic_frame_time_fallback": False,
                "render_phase_offset_mean_s": None if not render_phase_offsets_s else sum(render_phase_offsets_s) / len(render_phase_offsets_s),
                "render_phase_offset_max_abs_s": None if not render_phase_offsets_s else max(abs(value) for value in render_phase_offsets_s),
            },
            "static_tf_topic": "/tf_static",
            "truth_tf_child_frame": FRAMES["truth_sensor_rig"],
            "lidar_config": lidar_spec,
            "timestamp_alignment": {
                "reference": TOPICS["clock"],
                "ground_truth": _timestamp_summary(clock_stamps, ground_truth_stamps),
                "rgb": _timestamp_summary(clock_stamps, rgb_stamps),
                "lidar": _timestamp_summary(clock_stamps, lidar_stamps),
            },
            "lidar_cloud_points": _point_count_summary(lidar_point_counts),
            "lidar_noise": {
                "enabled": noise_config.enabled,
                "relay": "simulator.ros.lidar_noise_relay" if lidar_relay is not None else None,
                "received_clouds": None if lidar_relay is None else lidar_relay.received_clouds,
                "published_clouds": None if lidar_relay is None else lidar_relay.published_clouds,
                "last_point_count": None if lidar_relay is None else lidar_relay.last_point_count,
            },
            "rig_orientation_source": "trajectory.sample.orientation_xyzw",
            "last_rig_orientation_xyzw": list(sample.orientation_xyzw),
            "ground_truth_odometry_leakage": False,
            "dynamic_context": {
                "shopper_cart": True,
                "motion_source": "deterministic_presentation_scene_time",
                "semantic_id": "context/shopper_cart/primary",
                "last_position_m": list(shopper_cart_position_at_time(timestamp_s)),
            },
        }
        # Persist before SimulationApp teardown.  Isaac's shutdown sequence
        # can terminate the embedded Python process before the caller regains
        # control, so a successful run must not depend on ``main()`` reaching
        # its final write.
        Path(args.status_path).parent.mkdir(parents=True, exist_ok=True)
        Path(args.status_path).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    except BaseException as exc:
        error = {
            "runtime": "isaac_sim",
            "status": "error",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        print(json.dumps(error, indent=2), flush=True)
        try:
            Path(args.status_path).parent.mkdir(parents=True, exist_ok=True)
            Path(args.status_path).write_text(json.dumps(error, indent=2) + "\n", encoding="utf-8")
        except Exception:
            pass
        raise
    finally:
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if executor_thread is not None and executor_thread.is_alive():
            executor_thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            import isaacsim.core.experimental.utils.app as app_utils

            app_utils.stop()
        except Exception:
            pass
        simulation_app.close()


def main() -> None:
    args = _args()
    if not math.isfinite(args.realtime_factor) or not 0.0 < args.realtime_factor <= 1.0:
        raise ValueError("--realtime-factor must be greater than zero and no more than one")
    status_path = Path(args.status_path)
    status_path.parent.mkdir(parents=True, exist_ok=True)
    result = run(args)
    status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
