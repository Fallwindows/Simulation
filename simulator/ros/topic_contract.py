"""Public simulator topic/frame contract kept independent of consumers."""

FRAMES = {
    "sim_world": "sim_world",
    "sensor_rig": "sensor_rig",
    "camera_link": "camera_link",
    "camera_optical": "camera_optical_frame",
    "lidar_link": "lidar_link",
    "map": "map",
    "odom": "odom",
}

TOPICS = {
    "clock": "/clock",
    "rgb_image": "/sim/camera/rgb/image_raw",
    "rgb_camera_info": "/sim/camera/rgb/camera_info",
    "lidar_points": "/sim/lidar/points",
    "ground_truth_pose": "/sim/ground_truth/pose",
    "estimated_odom": "/slam/odom",
    "map_points": "/slam/map_cloud",
}

GROUND_TRUTH_MESSAGE = "geometry_msgs/msg/PoseStamped"
GROUND_TRUTH_FRAME = FRAMES["sim_world"]
