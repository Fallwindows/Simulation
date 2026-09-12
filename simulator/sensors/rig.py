"""Sensor-rig contract and geometry shared by Isaac and ROS adapters."""

from __future__ import annotations

import math
from dataclasses import dataclass

from simulator.config.loader import CameraConfig, LidarConfig
from simulator.ros.topic_contract import FRAMES, TOPICS
from simulator.sensors.transforms import Transform, camera_optical_quaternion, quaternion_from_rpy_deg


@dataclass(frozen=True)
class CameraIntrinsics:
    width_px: int
    height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float


@dataclass(frozen=True)
class SensorRigDescription:
    camera: CameraConfig
    lidar: LidarConfig
    camera_intrinsics: CameraIntrinsics
    transforms: tuple[Transform, ...]
    topics: dict[str, str]
    frames: dict[str, str]


def make_camera_intrinsics(camera: CameraConfig) -> CameraIntrinsics:
    fx = (camera.width_px / 2.0) / math.tan(math.radians(camera.horizontal_fov_deg) / 2.0)
    return CameraIntrinsics(camera.width_px, camera.height_px, fx, fx, (camera.width_px - 1) / 2.0, (camera.height_px - 1) / 2.0)


def build_sensor_rig_description(camera: CameraConfig, lidar: LidarConfig) -> SensorRigDescription:
    camera_pose = camera.pose_in_rig
    lidar_pose = lidar.pose_in_rig
    camera_link = Transform(FRAMES["sensor_rig"], FRAMES["camera_link"], camera_pose.position_m, quaternion_from_rpy_deg(*camera_pose.rpy_deg))
    optical = Transform(FRAMES["camera_link"], FRAMES["camera_optical"], (0.0, 0.0, 0.0), camera_optical_quaternion())
    lidar_link = Transform(FRAMES["sensor_rig"], FRAMES["lidar_link"], lidar_pose.position_m, quaternion_from_rpy_deg(*lidar_pose.rpy_deg))
    return SensorRigDescription(camera, lidar, make_camera_intrinsics(camera), (camera_link, optical, lidar_link), dict(TOPICS), dict(FRAMES))


def assert_no_ground_truth_odometry_leakage(description: SensorRigDescription) -> None:
    if description.topics["ground_truth_pose"] == description.topics["estimated_odom"]:
        raise ValueError("ground truth and estimated odometry must remain separate")
    if description.frames["map"] == description.frames["sim_world"]:
        raise ValueError("map and sim_world must remain distinct frames")
