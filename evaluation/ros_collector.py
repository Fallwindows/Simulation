"""ROS 2 collector that writes one reproducible evaluation artifact per run."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from evaluation.metrics import PoseSample, compute_metrics, safe_quaternion
from evaluation.run_io import write_run
from simulator.ros.topic_contract import TOPICS


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0


class CollectorNode:
    SIM_CLOCK_START_TOLERANCE_S = 1.0

    def __init__(self, run_dir: Path, scenario: str, duration_s: float, startup_timeout_s: float):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, PointCloud2
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.node = Node("grocery_sim_evaluation_collector")
        self.run_dir = run_dir
        self.scenario = scenario
        self.duration_s = duration_s
        self.startup_timeout_s = startup_timeout_s
        self.started = time.monotonic()
        self.capture_started: float | None = None
        self.capture_started_sim_time_s: float | None = None
        self.latest_sim_time_s: float | None = None
        self.sim_time_target_s: float | None = None
        self.sim_time_complete = False
        self.completion_reason = "not_started"
        self.post_target_grace_s = 15.0
        self.post_target_grace_deadline: float | None = None
        self.wall_guard_timeout_s = max(30.0, self.duration_s * 2.0)
        self.callback_group = ReentrantCallbackGroup()
        self.ground_truth: list[PoseSample] = []
        self.estimate: list[PoseSample] = []
        self.invalid_quaternion_counts = {"ground_truth": 0, "estimate": 0}
        self.topic_observations = {
            "ground_truth": {"topic": TOPICS["ground_truth_pose"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "estimate": {"topic": TOPICS["estimated_odom"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "clock": {"topic": TOPICS["clock"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "rgb": {"topic": TOPICS["rgb_image"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "lidar": {"topic": TOPICS["lidar_points"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "map": {"topic": TOPICS["map_points"], "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "tf": {"topic": "/tf", "count": 0, "first_stamp_s": None, "last_stamp_s": None},
            "tf_static": {"topic": "/tf_static", "count": 0, "first_stamp_s": None, "last_stamp_s": None},
        }
        self.node.create_subscription(PoseStamped, TOPICS["ground_truth_pose"], self._on_ground_truth, 50, callback_group=self.callback_group)
        self.node.create_subscription(Odometry, TOPICS["estimated_odom"], self._on_odom, 50, callback_group=self.callback_group)
        self.node.create_subscription(Clock, TOPICS["clock"], self._on_clock, 50, callback_group=self.callback_group)
        self.node.create_subscription(Image, TOPICS["rgb_image"], lambda message: self._observe("rgb", message), 2, callback_group=self.callback_group)
        self.node.create_subscription(PointCloud2, TOPICS["lidar_points"], lambda message: self._observe("lidar", message), 1, callback_group=self.callback_group)
        self.node.create_subscription(PointCloud2, TOPICS["map_points"], lambda message: self._observe("map", message), 1, callback_group=self.callback_group)
        self.node.create_subscription(TFMessage, "/tf", lambda message: self._observe("tf", message), 50, callback_group=self.callback_group)
        static_qos = QoSProfile(depth=1)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        self.node.create_subscription(TFMessage, "/tf_static", lambda message: self._observe("tf_static", message), static_qos, callback_group=self.callback_group)

    @staticmethod
    def _message_stamp(message) -> float | None:
        if hasattr(message, "clock"):
            return float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0
        if hasattr(message, "header"):
            return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0
        if hasattr(message, "transforms") and message.transforms:
            stamp = message.transforms[0].header.stamp
            return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        return None

    def _observe(self, name: str, message) -> None:
        observation = self.topic_observations[name]
        observation["count"] += 1
        stamp = self._message_stamp(message)
        if stamp is not None:
            if observation["first_stamp_s"] is None:
                observation["first_stamp_s"] = stamp
            observation["last_stamp_s"] = stamp

    def _on_clock(self, message) -> None:
        self._observe("clock", message)
        self.latest_sim_time_s = self._message_stamp(message)

    def _on_ground_truth(self, message) -> None:
        self._observe("ground_truth", message)
        orientation = safe_quaternion((message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w))
        if orientation is None:
            self.invalid_quaternion_counts["ground_truth"] += 1
            return
        self.ground_truth.append(PoseSample(
            _stamp(message),
            (float(message.pose.position.x), float(message.pose.position.y), float(message.pose.position.z)),
            orientation,
        ))
        if self.capture_started_sim_time_s is None:
            self.capture_started_sim_time_s = _stamp(message)

    def _on_odom(self, message) -> None:
        self._observe("estimate", message)
        orientation = safe_quaternion((message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w))
        if orientation is None:
            self.invalid_quaternion_counts["estimate"] += 1
            return
        self.estimate.append(PoseSample(
            _stamp(message),
            (float(message.pose.pose.position.x), float(message.pose.pose.position.y), float(message.pose.pose.position.z)),
            orientation,
        ))
        if self.capture_started_sim_time_s is None:
            self.capture_started_sim_time_s = _stamp(message)

    def spin_until_done(self, executor) -> None:
        startup_deadline = self.started + self.startup_timeout_s
        while self.rclpy.ok():
            now = time.monotonic()
            if self.capture_started is None:
                if self.capture_started_sim_time_s is not None:
                    self.capture_started = now
                    # Isaac resets the simulation clock to zero for each run.
                    # The first received sample can be one render tick late,
                    # so target the scenario duration itself while the clock
                    # is still near zero instead of extending the run by that
                    # transport delay.
                    if self.capture_started_sim_time_s <= self.SIM_CLOCK_START_TOLERANCE_S:
                        self.sim_time_target_s = self.duration_s
                    else:
                        self.sim_time_target_s = self.capture_started_sim_time_s + self.duration_s
                    self.completion_reason = "capturing_sim_time"
                elif now >= startup_deadline:
                    self.completion_reason = "startup_timeout"
                    break
            elif self.sim_time_complete and self.post_target_grace_deadline is not None and now >= self.post_target_grace_deadline:
                break
            elif not self.sim_time_complete and self.latest_sim_time_s is not None and self.sim_time_target_s is not None and self.latest_sim_time_s >= self.sim_time_target_s - 1e-3:
                self.sim_time_complete = True
                self.completion_reason = "simulation_time_reached"
                self.post_target_grace_deadline = now + self.post_target_grace_s
            elif self.capture_started is not None and now - self.capture_started >= self.wall_guard_timeout_s:
                # A queued clock callback can deliver the target timestamp at
                # the same boundary as the wall guard. Preserve the semantic
                # completion reason when the target is already observable.
                if self.latest_sim_time_s is not None and self.sim_time_target_s is not None and self.latest_sim_time_s >= self.sim_time_target_s - 1e-3:
                    self.sim_time_complete = True
                    self.completion_reason = "simulation_time_reached"
                    self.post_target_grace_deadline = now + self.post_target_grace_s
                else:
                    self.completion_reason = "simulation_time_stalled_wall_guard"
                    break
            executor.spin_once(timeout_sec=0.0)
            time.sleep(0.01)

    def write(self) -> None:
        metadata = {
            "scenario": self.scenario,
            "topics": {"ground_truth": TOPICS["ground_truth_pose"], "estimate": TOPICS["estimated_odom"]},
            "sample_counts": {"ground_truth": len(self.ground_truth), "estimate": len(self.estimate)},
            "invalid_quaternion_counts": dict(self.invalid_quaternion_counts),
            "topic_observations": self.topic_observations,
            "required_live_topics": [name for name in self.topic_observations],
            "collector_duration_s": self.duration_s,
            "collector_startup_timeout_s": self.startup_timeout_s,
            "capture_started": self.capture_started is not None,
            "simulation_time_complete": self.sim_time_complete,
            "simulation_time_start_s": self.capture_started_sim_time_s,
            "simulation_time_end_s": self.latest_sim_time_s,
            "simulation_time_target_s": self.sim_time_target_s,
            "completion_reason": self.completion_reason,
            "wall_guard_timeout_s": self.wall_guard_timeout_s,
            "post_target_grace_s": self.post_target_grace_s,
            "alignment_policy": "initial_se3",
            "sampling_policy": "interpolate_ground_truth_at_estimator_timestamps",
            "csv_schema": ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"],
        }
        self.run_dir.mkdir(parents=True, exist_ok=True)
        if self.ground_truth and self.estimate:
            metrics = compute_metrics(self.ground_truth, self.estimate, max_time_gap_s=0.2, alignment="initial_se3")
            write_run(self.run_dir, metadata, metrics.as_dict(), self.ground_truth, self.estimate)
        else:
            (self.run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            (self.run_dir / "metrics.json").write_text(json.dumps({"status": "insufficient_samples", **metadata["sample_counts"]}, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--scenario", default="baseline_straight")
    parser.add_argument("--duration-seconds", type=float, default=10.0)
    parser.add_argument("--startup-timeout-seconds", type=float, default=90.0)
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    collector = CollectorNode(
        Path(args.run_dir),
        args.scenario,
        args.duration_seconds,
        args.startup_timeout_seconds,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(collector.node)
    try:
        collector.spin_until_done(executor)
        collector.write()
    finally:
        executor.remove_node(collector.node)
        executor.shutdown(timeout_sec=2.0)
        collector.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
