"""rclpy subscription boundary for dashboard caches and bounded live metrics."""

from __future__ import annotations

import time

from dashboard.backend.state import DashboardState
from evaluation.metrics import PoseAlignment, PoseSample, compute_metrics, initial_se3_alignment, nearest_pose, safe_quaternion
from simulator.ros.topic_contract import TOPICS


TRACKING_FRESHNESS_THRESHOLD_S = 0.5
TRACKING_LOST_THRESHOLD_S = 2.0
METRICS_REFRESH_PERIOD_S = 1.0
METRIC_MATCH_GAP_S = 0.2


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
        self._ground_truth: list[PoseSample] = []
        self._estimate: list[PoseSample] = []
        self._alignment: PoseAlignment | None = None
        self._last_metrics_refresh = 0.0
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

    def _tracking_payload(self) -> dict[str, object]:
        if not self._ground_truth:
            return {"tracking_state": "waiting", "tracking_freshness_age_s": None, "current_position_error_m": None}
        latest_gt = self._ground_truth[-1]
        if not self._estimate:
            return {"tracking_state": "waiting", "tracking_freshness_age_s": None, "current_position_error_m": None}
        latest_est = nearest_pose(latest_gt, self._estimate, max_gap_s=10_000.0)
        if latest_est is None:
            return {"tracking_state": "lost", "tracking_freshness_age_s": None, "current_position_error_m": None}
        freshness = abs(latest_gt.timestamp_s - latest_est.timestamp_s)
        if freshness <= TRACKING_FRESHNESS_THRESHOLD_S:
            tracking_state = "tracking"
        elif freshness <= TRACKING_LOST_THRESHOLD_S:
            tracking_state = "stale"
        else:
            tracking_state = "lost"
        current_error = None
        if self._alignment is not None:
            current_error = math_distance(self._alignment.apply(latest_est).position_m, latest_gt.position_m)
        return {
            "tracking_state": tracking_state,
            "tracking_freshness_age_s": freshness,
            "latest_ground_truth_timestamp_s": latest_gt.timestamp_s,
            "latest_odom_timestamp_s": latest_est.timestamp_s,
            "current_position_error_m": current_error,
        }

    def _refresh_live_metrics(self) -> None:
        tracking = self._tracking_payload()
        self.state.merge_metrics(tracking)
        if not self._ground_truth or not self._estimate:
            return
        now = time.monotonic()
        if now - self._last_metrics_refresh < METRICS_REFRESH_PERIOD_S:
            return
        try:
            if self._alignment is None:
                self._alignment = initial_se3_alignment(self._ground_truth, self._estimate, METRIC_MATCH_GAP_S)
            metrics = compute_metrics(self._ground_truth, self._estimate, max_time_gap_s=METRIC_MATCH_GAP_S, alignment="initial_se3").as_dict()
        except ValueError:
            return
        self._last_metrics_refresh = now
        metrics.update(self._tracking_payload())
        self.state.update_metrics(metrics)

    def _on_ground_truth(self, message) -> None:
        self._ground_truth.append(PoseSample(
            self._stamp(message),
            (float(message.pose.position.x), float(message.pose.position.y), float(message.pose.position.z)),
            safe_quaternion((message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w)),
        ))
        self._ground_truth = self._ground_truth[-10000:]
        self._refresh_live_metrics()

    def _on_odom(self, message) -> None:
        self._estimate.append(PoseSample(
            self._stamp(message),
            (float(message.pose.pose.position.x), float(message.pose.pose.position.y), float(message.pose.pose.position.z)),
            safe_quaternion((message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w)),
        ))
        self._estimate = self._estimate[-10000:]
        self._refresh_live_metrics()

    def stop(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def math_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5
