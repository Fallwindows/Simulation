"""Opt-in, seeded sensor-realism hooks.

These helpers deliberately keep ideal mode a no-op. They operate on sensor
outputs before ROS publication, so browser decimation never changes SLAM input.
"""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass(frozen=True)
class NoiseConfig:
    enabled: bool = False
    range_std_m: float = 0.0
    dropout_probability: float = 0.0
    timestamp_jitter_ms: float = 0.0

    @classmethod
    def from_mapping(cls, data: dict[str, object] | None) -> "NoiseConfig":
        data = data or {}
        probability = float(data.get("dropout_probability", 0.0))
        if not 0.0 <= probability < 1.0:
            raise ValueError("dropout_probability must be in [0, 1)")
        std = float(data.get("range_std_m", 0.0))
        jitter = float(data.get("timestamp_jitter_ms", 0.0))
        if std < 0.0 or jitter < 0.0:
            raise ValueError("noise magnitudes must be non-negative")
        return cls(bool(data.get("enabled", False)), std, probability, jitter)


def apply_lidar_noise(points: list[tuple[float, float, float]], config: NoiseConfig, seed: int) -> list[tuple[float, float, float]]:
    if not config.enabled:
        return list(points)
    rng = random.Random(seed)
    result: list[tuple[float, float, float]] = []
    for x, y, z in points:
        if rng.random() < config.dropout_probability:
            continue
        radius = (x * x + y * y + z * z) ** 0.5
        if radius == 0.0:
            result.append((x, y, z))
            continue
        noisy_radius = max(0.0, radius + rng.gauss(0.0, config.range_std_m))
        scale = noisy_radius / radius
        result.append((x * scale, y * scale, z * scale))
    return result


def jitter_timestamp(timestamp_s: float, config: NoiseConfig, seed: int) -> float:
    if not config.enabled or config.timestamp_jitter_ms == 0.0:
        return timestamp_s
    return timestamp_s + random.Random(seed).gauss(0.0, config.timestamp_jitter_ms / 1000.0)
