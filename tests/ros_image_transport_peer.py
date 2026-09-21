"""Cross-process ROS Image publisher used by recorder integration tests."""

from __future__ import annotations

import argparse
import sys
import time

from evaluation.rgb_video_recorder import _create_recorder_node, _load_ros_image_type


def publish(topic: str) -> None:
    import rclpy

    Image = _load_ros_image_type()
    rclpy.init()
    node = _create_recorder_node("grocery_sim_rgb_video_test_publisher")
    publisher = node.create_publisher(Image, topic, 10)
    try:
        discovery_deadline = time.monotonic() + 15.0
        while publisher.get_subscription_count() < 1:
            if time.monotonic() >= discovery_deadline:
                raise RuntimeError("publisher did not discover the recorder subscription")
            rclpy.spin_once(node, timeout_sec=0.1)

        for index, stamp_s in enumerate((2.0, 2.05, 2.1)):
            message = Image()
            message.header.stamp.sec = int(stamp_s)
            message.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1_000_000_000))
            message.header.frame_id = "camera_optical_frame"
            message.width = 1280
            message.height = 720
            message.encoding = "rgb8"
            message.is_bigendian = 0
            message.step = 3840
            color = (255, 0, 0) if index % 2 == 0 else (0, 255, 0)
            message.data = bytes(color) * (message.width * message.height)
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        time.sleep(0.5)
        print(f"published=3 numpy_loaded={'numpy' in sys.modules}", flush=True)
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/sim/camera/rgb/image_raw")
    args = parser.parse_args()
    publish(args.topic)


if __name__ == "__main__":
    main()
