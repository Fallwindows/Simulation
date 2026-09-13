"""Optional seeded LiDAR realism relay for the live ROS graph.

Isaac publishes the ideal RTX cloud on a private contract topic.  This relay
is enabled only for the sensor-realism scenario and publishes the public topic
consumed by RTAB-Map and the dashboard.
"""

from __future__ import annotations

from simulator.ros.topic_contract import FRAMES, TOPICS
from simulator.sensors.noise import NoiseConfig, apply_lidar_noise, jitter_timestamp


class LidarNoiseRelay:
    def __init__(self, node, config: NoiseConfig, seed: int, max_points: int = 20000):
        try:
            from sensor_msgs_py import point_cloud2
        except ImportError as exc:  # pragma: no cover - ROS-only path
            raise RuntimeError("sensor_msgs_py is required for the LiDAR noise relay") from exc
        self.node = node
        self.config = config
        self.seed = int(seed)
        self.max_points = max(1, int(max_points))
        self._counter = 0
        self.received_clouds = 0
        self.published_clouds = 0
        self.last_point_count = 0
        self._point_cloud2 = point_cloud2
        from sensor_msgs.msg import PointCloud2

        self._PointCloud2 = PointCloud2
        self.publisher = node.create_publisher(PointCloud2, TOPICS["lidar_points"], 10)
        self.subscription = node.create_subscription(PointCloud2, TOPICS["lidar_ideal_points"], self._on_cloud, 10)

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    @staticmethod
    def _seconds_to_stamp(stamp, seconds: float) -> None:
        whole = int(seconds)
        stamp.sec = whole
        stamp.nanosec = max(0, min(999_999_999, int((seconds - whole) * 1_000_000_000)))

    def _on_cloud(self, message) -> None:
        frame_seed = self.seed + self._counter
        self._counter += 1
        self.received_clouds += 1
        points = [
            (float(x), float(y), float(z))
            for x, y, z in self._point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        ]
        points = apply_lidar_noise(points, self.config, frame_seed)
        if len(points) > self.max_points:
            stride = max(1, (len(points) + self.max_points - 1) // self.max_points)
            points = points[::stride][: self.max_points]
        self.last_point_count = len(points)
        header = message.header
        header.frame_id = FRAMES["lidar_link"]
        stamp_s = jitter_timestamp(self._stamp_to_seconds(header.stamp), self.config, frame_seed)
        self._seconds_to_stamp(header.stamp, stamp_s)
        output = self._point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(output)
        self.published_clouds += 1
