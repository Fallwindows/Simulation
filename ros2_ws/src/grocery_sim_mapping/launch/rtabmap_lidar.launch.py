"""RTAB-Map 3D LiDAR pipeline with explicit TF ownership.

The simulator publishes only the optional visualization branch
``sim_world -> truth_sensor_rig``.  ICP owns ``odom -> sensor_rig`` and
RTAB-Map owns ``map -> odom``; ground truth is never part of the estimator
tree or remapped to an estimator topic.
"""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _rtabmap_value(key: str, value):
    # RTAB-Map slash-qualified parameters are strings on the native Windows
    # build.  ROS-native booleans/integers remain typed parameters.
    if "/" in key:
        return str(value).lower() if isinstance(value, bool) else str(value)
    return value


def generate_launch_description():
    def build_nodes(context):
        package_share = Path(get_package_share_directory("grocery_sim_mapping"))
        contract = _load_json(package_share / "config" / "contracts.yaml")
        mapping_path = LaunchConfiguration("mapping_params_path").perform(context)
        mapping = _load_json(mapping_path if mapping_path else package_share / "config" / "params.yaml")
        slam_latch = mapping.pop("slam_latch", None)
        topics = contract["topics"]
        frames = contract["frames"]
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
        database_path = LaunchConfiguration("database_path").perform(context)

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
                formatted = _rtabmap_value(key, value)
                odom_params[key] = formatted
                slam_params[key] = formatted
        if slam_latch is not None:
            if not isinstance(slam_latch, bool):
                raise ValueError("slam_latch must be a JSON boolean")
            slam_params["latch"] = _rtabmap_value("latch", slam_latch)
        odom_params.update({"publish_tf": True, "scan_cloud_max_points": 50000})
        slam_params.update({"publish_tf": True, "database_path": database_path})

        return [
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
        ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("mapping_params_path", default_value=""),
        OpaqueFunction(function=build_nodes),
    ])
