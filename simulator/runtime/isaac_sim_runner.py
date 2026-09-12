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
import time
import traceback
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the grocery aisle Isaac Sim sensor runtime")
    parser.add_argument("--scenario", default=str(REPO_ROOT / "config/scenarios/baseline_straight.yaml"))
    parser.add_argument("--frames", type=int, default=600, help="Simulation frames; 60 frames is one simulated second")
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim viewport")
    parser.add_argument("--realtime", action="store_true", help="Pace the simulation at 60 Hz for external ROS/dashboard consumers")
    parser.add_argument("--renderer", default="RaytracedLighting")
    parser.add_argument("--status-path", default=str(REPO_ROOT / "runs/isaac_runtime_status.json"))
    return parser.parse_args()


def _sim_time_message(seconds: float):
    from builtin_interfaces.msg import Time

    whole = int(seconds)
    return Time(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


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
    dome.GetIntensityAttr().Set(float(scenario.environment.lighting_lux))
    dome.GetColorAttr().Set(Gf.Vec3f(1.0, 0.96, 0.9))
    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.GetIntensityAttr().Set(2500.0)
    distant.GetAngleAttr().Set(0.5)
    UsdGeom.XformCommonAPI(distant).SetRotate(Gf.Vec3f(-35.0, -25.0, 25.0))

    return layout, primitive_count


def _build_sensor_rig(stage, scenario):
    from pxr import Gf, Sdf, UsdGeom

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
    camera_api.SetRotate(Gf.Vec3f(-90.0, -90.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
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
                ("CameraInfo.inputs:frameId", "camera_optical_frame"),
                ("CameraInfo.inputs:topicName", "/sim/camera/rgb/camera_info"),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)

    # The camera helper is driven by the render pipeline.  A simulation gate
    # is still present in the generated graph; set it to the configured rate
    # when possible, while allowing non-integer FPS values to use every frame.
    try:
        import omni.syntheticdata._syntheticdata as sd

        render_product = og.Controller.attribute(f"{graph_path}/CreateRenderProduct.outputs:renderProductPath").get()
        rv_rgb = sd.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        gate_path = sd.SyntheticData._get_node_path(rv_rgb + "IsaacSimulationGate", render_product)
        step = max(1, round(60.0 / float(fps)))
        og.Controller.attribute(gate_path + ".inputs:step").set(step)
    except Exception:
        # Sensor publication is valid without a gate override; the bridge's
        # default is once per simulation frame.
        pass


def _create_lidar(lidar_path: str, translation: tuple[float, float, float]):
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
    from pxr import Sdf

    # Create the current native OmniLidar prim locally.  The downloadable
    # Example_Rotary asset is optional and may not be present on a workstation;
    # the schema-created sensor is still a real RTX LiDAR with explicit scan
    # bounds and a 10 Hz tick rate.
    lidar = Lidar.create(
        path=lidar_path,
        tick_rate=10.0,
        translations=[list(translation)],
        attributes={
            "omni:sensor:Core:scanRateBaseHz": 10.0,
            "omni:sensor:Core:nearRangeM": 0.2,
            "omni:sensor:Core:farRangeM": 60.0,
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
        topicName="/sim/lidar/points",
        frameId="lidar_link",
    )
    return lidar, sensor


def _publish_ground_truth(node, publisher, trajectory, timestamp_s: float):
    from geometry_msgs.msg import PoseStamped

    sample = trajectory.sample(timestamp_s)
    message = PoseStamped()
    message.header.stamp = _sim_time_message(timestamp_s)
    message.header.frame_id = "sim_world"
    message.pose.position.x, message.pose.position.y, message.pose.position.z = sample.position_m
    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = sample.orientation_xyzw
    publisher.publish(message)


def _publish_tf(publisher, scenario, sample, timestamp_s: float) -> None:
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    from simulator.sensors.transforms import camera_optical_quaternion, quaternion_from_rpy_deg

    stamp = _sim_time_message(timestamp_s)
    transforms = []

    def add(parent: str, child: str, translation, rotation) -> None:
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = translation
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = rotation
        transforms.append(message)

    add("sim_world", "sensor_rig", sample.position_m, sample.orientation_xyzw)
    add(
        "sensor_rig",
        "camera_link",
        scenario.camera.pose_in_rig.position_m,
        quaternion_from_rpy_deg(*scenario.camera.pose_in_rig.rpy_deg),
    )
    add("camera_link", "camera_optical_frame", (0.0, 0.0, 0.0), camera_optical_quaternion())
    add(
        "sensor_rig",
        "lidar_link",
        scenario.lidar.pose_in_rig.position_m,
        quaternion_from_rpy_deg(*scenario.lidar.pose_in_rig.rpy_deg),
    )
    publisher.publish(TFMessage(transforms=transforms))


def run(args: argparse.Namespace) -> dict[str, object]:
    from isaacsim import SimulationApp

    from simulator.config.loader import load_scenario
    from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory

    if args.frames < 1:
        raise ValueError("--frames must be positive")
    scenario = load_scenario(args.scenario)
    trajectory_cls = WalkingTrajectory if scenario.trajectory.name.lower() == "walking" else StraightTrajectory
    trajectory = trajectory_cls(scenario.trajectory)

    print("[grocery-runtime] starting SimulationApp", flush=True)
    simulation_app = SimulationApp({"renderer": args.renderer, "headless": bool(args.headless)})
    node = None
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
        layout, primitive_count = _build_world(stage, scenario)
        print("[grocery-runtime] building sensor rig USD", flush=True)
        _, camera_path, lidar_path = _build_sensor_rig(stage, scenario)
        print("[grocery-runtime] creating camera ROS graph", flush=True)
        _create_camera_graph(camera_path, scenario.camera.width_px, scenario.camera.height_px, scenario.camera.fps)

        # Sensor API objects must be created after the bridge extension is
        # loaded and before the simulation starts.
        print("[grocery-runtime] creating RTX LiDAR ROS writer", flush=True)
        _, _lidar_sensor = _create_lidar(lidar_path, scenario.lidar.pose_in_rig.position_m)
        print("[grocery-runtime] creating clock and TF graph", flush=True)
        _create_clock_graph()
        print("[grocery-runtime] initializing simulation", flush=True)
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        simulation_app.update()

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage

        rclpy.init(args=None)
        node = rclpy.create_node("grocery_sim_ground_truth")
        gt_pub = node.create_publisher(PoseStamped, "/sim/ground_truth/pose", 10)
        tf_pub = node.create_publisher(TFMessage, "/tf", 10)
        app_utils.play()
        print("[grocery-runtime] simulation running", flush=True)

        from pxr import Gf, UsdGeom

        rig_prim = stage.GetPrimAtPath("/World/SensorRig")
        rig_api = UsdGeom.XformCommonAPI(rig_prim)
        for frame in range(args.frames):
            wall_start = time.perf_counter()
            timestamp_s = frame / 60.0
            sample = trajectory.sample(timestamp_s)
            rig_api.SetTranslate(Gf.Vec3d(*sample.position_m))
            # The trajectory's quaternion is generated from RPY; the runtime
            # contract uses yaw for the rig's authored USD transform.
            rig_api.SetRotate(Gf.Vec3f(0.0, 0.0, scenario.trajectory.yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            _publish_ground_truth(node, gt_pub, trajectory, timestamp_s)
            _publish_tf(tf_pub, scenario, sample, timestamp_s)
            simulation_app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            if args.realtime:
                time.sleep(max(0.0, (1.0 / 60.0) - (time.perf_counter() - wall_start)))

        runtime_ok = True
        print("[grocery-runtime] simulation completed", flush=True)
        result = {
            "runtime": "isaac_sim",
            "isaac_version": "6.1.0",
            "scenario": scenario.name,
            "frames_simulated": args.frames,
            "primitive_count": primitive_count,
            "product_count": len(layout.products),
            "topics": {
                "clock": "/clock",
                "rgb": "/sim/camera/rgb/image_raw",
                "camera_info": "/sim/camera/rgb/camera_info",
                "lidar": "/sim/lidar/points",
                "ground_truth_pose": "/sim/ground_truth/pose",
            },
            "frames": ["sim_world", "sensor_rig", "camera_link", "camera_optical_frame", "lidar_link"],
            "ground_truth_odometry_leakage": False,
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
    status_path = Path(args.status_path)
    status_path.parent.mkdir(parents=True, exist_ok=True)
    result = run(args)
    status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
