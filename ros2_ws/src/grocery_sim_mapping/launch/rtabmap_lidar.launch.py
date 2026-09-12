"""RTAB-Map 3D LiDAR boundary; simulator truth is never wired as odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": "lidar_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "subscribe_scan_cloud": True,
                "scan_cloud_max_points": 50000,
            }],
            remappings=[("scan_cloud", "/sim/lidar/points"), ("odom", "/slam/odom")],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": "lidar_link",
                "odom_frame_id": "odom",
                "map_frame_id": "map",
                "subscribe_scan_cloud": True,
                "subscribe_rgb": True,
                "subscribe_depth": False,
                "approx_sync": True,
                "Grid/3D": True,
                "Mem/IncrementalMemory": True,
            }],
            remappings=[("scan_cloud", "/sim/lidar/points"), ("odom", "/slam/odom"), ("rgb/image", "/sim/camera/rgb/image_raw"), ("rgb/camera_info", "/sim/camera/rgb/camera_info"), ("mapData", "/slam/map_cloud")],
        ),
    ])
