"""ROS 2 collector that writes one reproducible evaluation artifact per run."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from evaluation.metrics import compute_metrics
from evaluation.run_io import write_run
from simulator.ros.topic_contract import TOPICS


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0


class CollectorNode:
    def __init__(self, run_dir: Path, scenario: str, duration_s: float):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node

        self.rclpy = rclpy
        self.node = Node("grocery_sim_evaluation_collector")
        self.run_dir = run_dir
        self.scenario = scenario
        self.duration_s = duration_s
        self.started = time.monotonic()
        self.ground_truth: list[tuple[float, tuple[float, float, float]]] = []
        self.estimate: list[tuple[float, tuple[float, float, float]]] = []
        self.node.create_subscription(PoseStamped, TOPICS["ground_truth_pose"], self._on_ground_truth, 50)
        self.node.create_subscription(Odometry, TOPICS["estimated_odom"], self._on_odom, 50)

    def _on_ground_truth(self, message) -> None:
        self.ground_truth.append((_stamp(message), (float(message.pose.position.x), float(message.pose.position.y), float(message.pose.position.z))))

    def _on_odom(self, message) -> None:
        self.estimate.append((_stamp(message), (float(message.pose.pose.position.x), float(message.pose.pose.position.y), float(message.pose.pose.position.z))))

    def spin_until_done(self) -> None:
        while self.rclpy.ok() and time.monotonic() - self.started < self.duration_s:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def write(self) -> None:
        metadata = {
            "scenario": self.scenario,
            "topics": {"ground_truth": TOPICS["ground_truth_pose"], "estimate": TOPICS["estimated_odom"]},
            "sample_counts": {"ground_truth": len(self.ground_truth), "estimate": len(self.estimate)},
            "collector_duration_s": self.duration_s,
        }
        self.run_dir.mkdir(parents=True, exist_ok=True)
        if self.ground_truth and self.estimate:
            metrics = compute_metrics(self.ground_truth, self.estimate, max_time_gap_s=0.2)
            write_run(self.run_dir, metadata, metrics.as_dict(), self.ground_truth, self.estimate)
        else:
            (self.run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            (self.run_dir / "metrics.json").write_text(json.dumps({"status": "insufficient_samples", **metadata["sample_counts"]}, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--scenario", default="baseline_straight")
    parser.add_argument("--duration-seconds", type=float, default=10.0)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    collector = CollectorNode(Path(args.run_dir), args.scenario, args.duration_seconds)
    try:
        collector.spin_until_done()
        collector.write()
    finally:
        collector.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
