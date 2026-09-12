"""Optional rclpy subscription boundary for the dashboard caches."""

from __future__ import annotations

from evaluation.metrics import compute_metrics
from dashboard.backend.state import DashboardState
from simulator.ros.topic_contract import TOPICS


class RosDashboardBridge:
    """rclpy subscriptions; no Isaac data bypasses ROS."""

    def __init__(self, state: DashboardState):
        try:
            import rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from geometry_msgs.msg import PoseStamped  # type: ignore
            from nav_msgs.msg import Odometry  # type: ignore
            from sensor_msgs.msg import Image, PointCloud2  # type: ignore
            from sensor_msgs_py import point_cloud2  # type: ignore
        except ImportError as exc:
            raise RuntimeError("ROS 2 rclpy is unavailable") from exc
        self.rclpy = rclpy
        self.Node = Node
        self.state = state
        self.topics = dict(TOPICS)
        self._Image = Image
        self._PointCloud2 = PointCloud2
        self._PoseStamped = PoseStamped
        self._Odometry = Odometry
        self._point_cloud2 = point_cloud2
        self._ground_truth: list[tuple[float, tuple[float, float, float]]] = []
        self._estimate: list[tuple[float, tuple[float, float, float]]] = []
        self.node = None

    def topic_contract(self) -> dict[str, str]:
        return dict(self.topics)

    def start(self) -> None:
        if not self.rclpy.ok():
            self.rclpy.init()
        self.node = self.Node("grocery_sim_dashboard_bridge")
        self.node.create_subscription(self._Image, self.topics["rgb_image"], self._on_image, 10)
        self.node.create_subscription(self._PointCloud2, self.topics["lidar_points"], self._on_lidar, 10)
        self.node.create_subscription(self._PointCloud2, self.topics["map_points"], self._on_map, 10)
        self.node.create_subscription(self._PoseStamped, self.topics["ground_truth_pose"], self._on_ground_truth, 10)
        self.node.create_subscription(self._Odometry, self.topics["estimated_odom"], self._on_odom, 10)
        self.rclpy.spin(self.node)

    def _on_image(self, message) -> None:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        encoding = str(message.encoding).lower()
        channels = 1 if encoding in {"mono8", "8uc1"} else 4 if encoding in {"rgba8", "bgra8"} else 3
        image = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape((message.height, message.width, channels))
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif channels == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        ok, encoded = cv2.imencode(".jpg", image)
        if ok:
            self.state.update_rgb(encoded.tobytes())

    def _points(self, message):
        return [(float(x), float(y), float(z)) for x, y, z in self._point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)]

    def _on_lidar(self, message) -> None:
        self.state.update_points("lidar", self._points(message))

    def _on_map(self, message) -> None:
        self.state.update_points("map", self._points(message))

    @staticmethod
    def _stamp(message) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0

    def _refresh_metrics(self) -> None:
        if not self._ground_truth or not self._estimate:
            return
        try:
            metrics = compute_metrics(self._ground_truth, self._estimate, max_time_gap_s=0.2).as_dict()
        except ValueError:
            return
        metrics["tracking_state"] = "tracking" if metrics["current_position_error_m"] is not None else "lost"
        self.state.update_metrics(metrics)

    def _on_ground_truth(self, message) -> None:
        self._ground_truth.append((self._stamp(message), (float(message.pose.position.x), float(message.pose.position.y), float(message.pose.position.z))))
        self._ground_truth = self._ground_truth[-10000:]
        self._refresh_metrics()

    def _on_odom(self, message) -> None:
        self._estimate.append((self._stamp(message), (float(message.pose.pose.position.x), float(message.pose.pose.position.y), float(message.pose.pose.position.z))))
        self._estimate = self._estimate[-10000:]
        self._refresh_metrics()

    def stop(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
