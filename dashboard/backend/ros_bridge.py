"""Optional rclpy subscription boundary for the dashboard caches."""

from __future__ import annotations

from dashboard.backend.state import DashboardState
from simulator.ros.topic_contract import TOPICS


class RosDashboardBridge:
    """rclpy subscriptions; no Isaac data bypasses ROS."""

    def __init__(self, state: DashboardState):
        try:
            import rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from cv_bridge import CvBridge  # type: ignore
            from sensor_msgs.msg import Image, PointCloud2  # type: ignore
            from sensor_msgs_py import point_cloud2  # type: ignore
        except ImportError as exc:
            raise RuntimeError("ROS 2 rclpy is unavailable") from exc
        self.rclpy = rclpy
        self.Node = Node
        self.state = state
        self.topics = dict(TOPICS)
        self._CvBridge = CvBridge
        self._Image = Image
        self._PointCloud2 = PointCloud2
        self._point_cloud2 = point_cloud2
        self.node = None

    def topic_contract(self) -> dict[str, str]:
        return dict(self.topics)

    def start(self) -> None:
        if not self.rclpy.ok():
            self.rclpy.init()
        self.node = self.Node("grocery_sim_dashboard_bridge")
        self.bridge = self._CvBridge()
        self.node.create_subscription(self._Image, self.topics["rgb_image"], self._on_image, 10)
        self.node.create_subscription(self._PointCloud2, self.topics["lidar_points"], self._on_lidar, 10)
        self.node.create_subscription(self._PointCloud2, self.topics["map_points"], self._on_map, 10)
        self.rclpy.spin(self.node)

    def _on_image(self, message) -> None:
        import cv2  # type: ignore
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        ok, encoded = cv2.imencode(".jpg", image)
        if ok:
            self.state.update_rgb(encoded.tobytes())

    def _points(self, message):
        return [(float(x), float(y), float(z)) for x, y, z in self._point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)]

    def _on_lidar(self, message) -> None:
        self.state.update_points("lidar", self._points(message))

    def _on_map(self, message) -> None:
        self.state.update_points("map", self._points(message))

    def stop(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
