"""RTAB-Map 3D LiDAR pipeline; simulator truth is never wired as odometry.

The ICP odometry node consumes only ``/sim/lidar/points``.  RTAB-Map SLAM
consumes that estimated odometry plus the ROS camera and publishes its own
``cloud_map`` output on the dashboard contract ``/slam/map_cloud``.
"""

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
                # The camera remains available on the ROS contract, but a
                # depth image is not part of the baseline RGB sensor.  Use
                # RTAB-Map's native LiDAR-only 3D pipeline here; RGB can be
                # enabled later with an RGB-D boundary.
                "subscribe_rgb": False,
                "subscribe_depth": False,
                # RTAB-Map internal parameters must be strings.  Passing
                # Python bools for slash-qualified keys makes the native
                # Windows node abort while applying parameter overrides.
                "Grid/3D": "true",
                "Grid/Sensor": "0",
                "Mem/IncrementalMemory": "true",
            }],
            remappings=[
                ("scan_cloud", "/sim/lidar/points"),
                ("odom", "/slam/odom"),
                ("rgb/image", "/sim/camera/rgb/image_raw"),
                ("rgb/camera_info", "/sim/camera/rgb/camera_info"),
                ("cloud_map", "/slam/map_cloud"),
            ],
        ),
    ])
