"""Alternate non-Isaac ROS helper for tests and small adapters.

All ROS imports are delayed to construction so config/math tests run on a
machine without ROS. The live Isaac path is
``simulator.runtime.isaac_sim_runner``; it uses Isaac's native RGB and RTX
LiDAR writers rather than this helper.
"""

from __future__ import annotations

from typing import Iterable

from simulator.config.loader import CameraConfig
from simulator.motion.trajectory import PoseSample
from simulator.ros.topic_contract import FRAMES, TOPICS
from simulator.sensors.rig import make_camera_intrinsics


class RosSensorPublisher:
    def __init__(self, node, camera: CameraConfig):
        try:
            from builtin_interfaces.msg import Time  # noqa: F401
            from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: F401
            from rosgraph_msgs.msg import Clock  # noqa: F401
            from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # noqa: F401
            from sensor_msgs_py import point_cloud2  # noqa: F401
        except ImportError as exc:
            raise RuntimeError("ROS 2 message packages are unavailable") from exc
        self.node = node
        self.camera = camera
        self._Image = Image
        self._CameraInfo = CameraInfo
        self._PointCloud2 = PointCloud2
        self._PoseStamped = PoseStamped
        self._Clock = Clock
        self._point_cloud2 = point_cloud2
        self.image_pub = node.create_publisher(Image, TOPICS["rgb_image"], 10)
        self.camera_info_pub = node.create_publisher(CameraInfo, TOPICS["rgb_camera_info"], 10)
        self.lidar_pub = node.create_publisher(PointCloud2, TOPICS["lidar_points"], 10)
        self.truth_pub = node.create_publisher(PoseStamped, TOPICS["ground_truth_pose"], 10)
        self.clock_pub = node.create_publisher(Clock, TOPICS["clock"], 10)

    @staticmethod
    def _stamp(node, timestamp_s: float):
        seconds = int(timestamp_s)
        nanoseconds = int(round((timestamp_s - seconds) * 1_000_000_000))
        stamp = node.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = seconds, nanoseconds
        return stamp

    def publish_rgb(self, raw_rgb: bytes, timestamp_s: float) -> None:
        expected = self.camera.width_px * self.camera.height_px * 3
        if len(raw_rgb) != expected:
            raise ValueError(f"raw_rgb must contain {expected} bytes")
        message = self._Image()
        message.header.stamp = self._stamp(self.node, timestamp_s)
        message.header.frame_id = FRAMES["camera_optical"]
        message.height, message.width = self.camera.height_px, self.camera.width_px
        message.encoding, message.is_bigendian, message.step = "rgb8", 0, self.camera.width_px * 3
        message.data = raw_rgb
        self.image_pub.publish(message)

    def publish_camera_info(self, timestamp_s: float) -> None:
        intrinsics = make_camera_intrinsics(self.camera)
        message = self._CameraInfo()
        message.header.stamp = self._stamp(self.node, timestamp_s)
        message.header.frame_id = FRAMES["camera_optical"]
        message.width, message.height = intrinsics.width_px, intrinsics.height_px
        message.k = [intrinsics.fx_px, 0.0, intrinsics.cx_px, 0.0, intrinsics.fy_px, intrinsics.cy_px, 0.0, 0.0, 1.0]
        message.p = [intrinsics.fx_px, 0.0, intrinsics.cx_px, 0.0, 0.0, intrinsics.fy_px, intrinsics.cy_px, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(message)

    def publish_lidar(self, points: Iterable[tuple[float, float, float]], timestamp_s: float) -> None:
        header = __import__("std_msgs.msg", fromlist=["Header"]).Header()
        header.stamp = self._stamp(self.node, timestamp_s)
        header.frame_id = FRAMES["lidar_link"]
        self.lidar_pub.publish(self._point_cloud2.create_cloud_xyz32(header, list(points)))

    def publish_ground_truth(self, sample: PoseSample) -> None:
        message = self._PoseStamped()
        message.header.stamp = self._stamp(self.node, sample.timestamp_s)
        message.header.frame_id = FRAMES["sim_world"]
        message.pose.position.x, message.pose.position.y, message.pose.position.z = sample.position_m
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = sample.orientation_xyzw
        self.truth_pub.publish(message)

    def publish_clock(self, timestamp_s: float) -> None:
        message = self._Clock()
        message.clock = self._stamp(self.node, timestamp_s)
        self.clock_pub.publish(message)
