"""Deterministic rig motion independent of camera or LiDAR implementation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from simulator.config.loader import TrajectoryConfig
from simulator.sensors.transforms import quaternion_from_rpy_deg


@dataclass(frozen=True)
class PoseSample:
    timestamp_s: float
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


class StraightTrajectory:
    def __init__(self, config: TrajectoryConfig):
        self.config = config

    def sample(self, timestamp_s: float) -> PoseSample:
        t = min(max(float(timestamp_s), 0.0), self.config.duration_s)
        x0, y0, z0 = self.config.start_position_m
        x = x0 + self.config.speed_mps * t
        return PoseSample(t, (x, y0, z0), quaternion_from_rpy_deg(0.0, 0.0, self.config.yaw_deg))

    def sample_many(self) -> tuple[PoseSample, ...]:
        count = int(round(self.config.duration_s * self.config.sample_hz))
        return tuple(self.sample(i / self.config.sample_hz) for i in range(count + 1))


class WalkingTrajectory(StraightTrajectory):
    def sample(self, timestamp_s: float) -> PoseSample:
        t = min(max(float(timestamp_s), 0.0), self.config.duration_s)
        x0, y0, z0 = self.config.start_position_m
        variation = self.config.speed_variation_fraction
        x = x0 + self.config.speed_mps * (t + variation * (1.0 - math.cos(0.7 * t)) / 0.7)
        y = y0 + self.config.sway_amplitude_m * math.sin(2.0 * math.pi * self.config.sway_hz * t)
        z = z0 + self.config.bob_amplitude_m * (0.5 - 0.5 * math.cos(2.0 * math.pi * self.config.bob_hz * t))
        yaw = self.config.yaw_deg + self.config.yaw_amplitude_deg * math.sin(2.0 * math.pi * self.config.sway_hz * t)
        pitch = self.config.pitch_amplitude_deg * math.sin(2.0 * math.pi * self.config.bob_hz * t + math.pi / 2.0)
        return PoseSample(t, (x, y, z), quaternion_from_rpy_deg(0.0, pitch, yaw))
