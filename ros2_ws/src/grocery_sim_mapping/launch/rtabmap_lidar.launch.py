"""RTAB-Map 3D LiDAR pipeline with explicit TF ownership.

The simulator publishes ``sim_world -> sensor_rig`` and the static sensor
extrinsics.  ICP owns ``odom -> sensor_rig`` and RTAB-Map owns ``map -> odom``;
ground truth is never remapped to an estimator topic.
"""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_json(name: str) -> dict:
    package_share = Path(get_package_share_directory("grocery_sim_mapping"))
    return json.loads((package_share / "config" / name).read_text(encoding="utf-8"))


def _rtabmap_value(key: str, value):
    # RTAB-Map slash-qualified parameters are strings on the native Windows
    # build.  ROS-native booleans/integers remain typed parameters.
    if "/" in key:
        return str(value).lower() if isinstance(value, bool) else str(value)
    return value


def generate_launch_description():
    contract = _load_json("contracts.yaml")
    mapping = _load_json("params.yaml")
    topics = contract["topics"]
    frames = contract["frames"]
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")

    common = {
        "use_sim_time": use_sim_time,
        "frame_id": frames["sensor_rig"],
        "odom_frame_id": frames["odom"],
    }
    odom_params = dict(common)
    slam_params = dict(common)
    slam_params["map_frame_id"] = frames["map"]
    for key, value in mapping.items():
        if key not in {"frame_id", "odom_frame_id", "map_frame_id", "use_sim_time"}:
            target = odom_params if key in {"subscribe_scan_cloud", "subscribe_odom_info", "approx_sync", "queue_size", "Reg/Strategy"} else slam_params
            target[key] = _rtabmap_value(key, value)
    odom_params.update({"publish_tf": True, "scan_cloud_max_points": 50000})
    for key in ("subscribe_scan_cloud", "subscribe_odom_info", "approx_sync", "queue_size"):
        if key in mapping:
            slam_params[key] = mapping[key]
    slam_params["database_path"] = database_path

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("database_path", default_value=""),
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=[odom_params],
            remappings=[("scan_cloud", topics["lidar_points"]), ("odom", topics["estimated_odom"])],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[slam_params],
            remappings=[
                ("scan_cloud", topics["lidar_points"]),
                ("odom", topics["estimated_odom"]),
                ("rgb/image", topics["rgb_image"]),
                ("rgb/camera_info", topics["rgb_camera_info"]),
                ("cloud_map", topics["map_points"]),
            ],
        ),
    ])
