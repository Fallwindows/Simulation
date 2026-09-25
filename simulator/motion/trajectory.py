"""Deterministic, continuously eased rig motion.

``speed_mps`` remains the mean forward speed over ``duration_s`` so existing
scenario distances do not change.  A 2.5 second launch and stop (or half the
duration for short moves) use a seventh-order velocity ramp with zero velocity,
acceleration, and jerk at the clamped endpoints.  The resulting cruise speed is
slightly above the configured mean to preserve total distance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from simulator.config.loader import TrajectoryConfig
from simulator.sensors.transforms import (
    interpolate_position,
    quaternion_from_rpy_deg,
    quaternion_slerp,
)


_DEFAULT_RAMP_DURATION_S = 2.5
_SPEED_VARIATION_RAD_PER_S = 0.7


@dataclass(frozen=True)
class PoseSample:
    timestamp_s: float
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class _MotionState:
    distance_m: float
    speed_mps: float
    cruise_speed_mps: float


def _clamped_time(timestamp_s: float, duration_s: float) -> float:
    timestamp_s = float(timestamp_s)
    if not math.isfinite(timestamp_s):
        raise ValueError("trajectory timestamp must be finite")
    return min(max(timestamp_s, 0.0), duration_s)


def _velocity_ramp(unit_time: float) -> float:
    """Seventh-order 0..1 velocity ramp with zero first three end derivatives."""

    u = min(max(unit_time, 0.0), 1.0)
    return u**4 * (35.0 + u * (-84.0 + u * (70.0 - 20.0 * u)))


def _velocity_ramp_integral(unit_time: float) -> float:
    """Integral of :func:`_velocity_ramp`; its value at one is exactly 0.5."""

    u = min(max(unit_time, 0.0), 1.0)
    return u**5 * (7.0 + u * (-14.0 + u * (10.0 - 2.5 * u)))


def _look_weight(timestamp_s: float, center_s: float, rise_s: float, fall_s: float) -> float:
    """Return a C3-continuous 0..1..0 envelope for a composed shelf look."""

    start_s, end_s = center_s - rise_s, center_s + fall_s
    if timestamp_s <= start_s or timestamp_s >= end_s:
        return 0.0
    if timestamp_s < center_s:
        return _velocity_ramp((timestamp_s - start_s) / rise_s)
    if timestamp_s > center_s:
        return 1.0 - _velocity_ramp((timestamp_s - center_s) / fall_s)
    return 1.0


def _motion_state(config: TrajectoryConfig, timestamp_s: float) -> _MotionState:
    """Return eased forward distance and speed while preserving endpoint distance."""

    duration = config.duration_s
    t = _clamped_time(timestamp_s, duration)
    total_distance = config.speed_mps * duration
    ramp_duration = min(_DEFAULT_RAMP_DURATION_S, duration / 2.0)
    cruise_speed = total_distance / (duration - ramp_duration)

    if t <= 0.0:
        return _MotionState(0.0, 0.0, cruise_speed)
    if t >= duration:
        return _MotionState(total_distance, 0.0, cruise_speed)
    if t < ramp_duration:
        unit_time = t / ramp_duration
        distance = cruise_speed * ramp_duration * _velocity_ramp_integral(unit_time)
        speed = cruise_speed * _velocity_ramp(unit_time)
    elif t <= duration - ramp_duration:
        distance = cruise_speed * (t - 0.5 * ramp_duration)
        speed = cruise_speed
    else:
        unit_time = (duration - t) / ramp_duration
        remaining = cruise_speed * ramp_duration * _velocity_ramp_integral(unit_time)
        distance = total_distance - remaining
        speed = cruise_speed * _velocity_ramp(unit_time)

    variation = config.speed_variation_fraction
    if variation:
        # Warp normalized distance monotonically.  An integer number of cycles
        # keeps both endpoints fixed, while dq_warp/dq stays positive because
        # configuration validation requires variation < 1.
        cycles = max(1, round(duration * _SPEED_VARIATION_RAD_PER_S / (2.0 * math.pi)))
        phase = 2.0 * math.pi * cycles * distance / total_distance
        distance += total_distance * variation * math.sin(phase) / (2.0 * math.pi * cycles)
        speed *= 1.0 + variation * math.cos(phase)
    return _MotionState(distance, speed, cruise_speed)


def interpolate_pose(start: PoseSample, end: PoseSample, timestamp_s: float) -> PoseSample:
    """Interpolate a bracketed pose at an arbitrary timestamp.

    Position uses linear interpolation and orientation uses shortest-arc slerp,
    which lets renderers sample a pose continuously instead of snapping to a
    source-frame integer.  Extrapolation is rejected to keep timestamp meaning
    explicit.
    """

    timestamp_s = float(timestamp_s)
    if not math.isfinite(timestamp_s):
        raise ValueError("pose timestamp must be finite")
    if not math.isfinite(start.timestamp_s) or not math.isfinite(end.timestamp_s):
        raise ValueError("pose bracket timestamps must be finite")
    if end.timestamp_s <= start.timestamp_s:
        raise ValueError("pose interpolation requires increasing timestamps")
    if not start.timestamp_s <= timestamp_s <= end.timestamp_s:
        raise ValueError("pose interpolation does not extrapolate")
    fraction = (timestamp_s - start.timestamp_s) / (end.timestamp_s - start.timestamp_s)
    return PoseSample(
        timestamp_s,
        interpolate_position(start.position_m, end.position_m, fraction),
        quaternion_slerp(start.orientation_xyzw, end.orientation_xyzw, fraction),
    )


class StraightTrajectory:
    def __init__(self, config: TrajectoryConfig):
        self.config = config

    def sample(self, timestamp_s: float) -> PoseSample:
        t = _clamped_time(timestamp_s, self.config.duration_s)
        motion = _motion_state(self.config, t)
        x0, y0, z0 = self.config.start_position_m
        x = x0 + motion.distance_m
        return PoseSample(t, (x, y0, z0), quaternion_from_rpy_deg(0.0, 0.0, self.config.yaw_deg))

    def sample_many(self) -> tuple[PoseSample, ...]:
        count = int(math.floor(self.config.duration_s * self.config.sample_hz))
        timestamps = [i / self.config.sample_hz for i in range(count + 1)]
        if timestamps[-1] < self.config.duration_s:
            timestamps.append(self.config.duration_s)
        return tuple(self.sample(timestamp_s) for timestamp_s in timestamps)


class WalkingTrajectory(StraightTrajectory):
    def sample(self, timestamp_s: float) -> PoseSample:
        t = _clamped_time(timestamp_s, self.config.duration_s)
        motion = _motion_state(self.config, t)
        x0, y0, z0 = self.config.start_position_m
        x = x0 + motion.distance_m

        # Phase advances with traveled distance, so gait cadence slows naturally
        # during launch/stop.  Scaling by speed removes gait offsets while the
        # rig is stationary.  Harmonic shaping avoids an obvious pure sine while
        # remaining deterministic and bounded by each configured amplitude.
        gait_scale = motion.speed_mps / (
            motion.cruise_speed_mps * (1.0 + self.config.speed_variation_fraction)
        )
        sway_phase = 2.0 * math.pi * self.config.sway_hz * motion.distance_m / motion.cruise_speed_mps
        bob_phase = 2.0 * math.pi * self.config.bob_hz * motion.distance_m / motion.cruise_speed_mps
        sway_wave = 0.88 * math.sin(sway_phase) + 0.12 * math.sin(2.0 * sway_phase)
        bob_wave = 0.5 - 0.5 * math.cos(bob_phase + 0.08 * math.sin(2.0 * bob_phase))
        pitch_wave = 0.9 * math.sin(bob_phase) + 0.1 * math.sin(2.0 * bob_phase)

        look_y = 0.0
        look_yaw = 0.0
        look_pitch = 0.0
        for beat in self.config.look_beats:
            weight = _look_weight(t, beat.center_s, beat.rise_s, beat.fall_s)
            look_y += weight * beat.lateral_offset_m
            look_yaw += weight * beat.yaw_offset_deg
            look_pitch += weight * beat.pitch_offset_deg

        y = y0 + self.config.sway_amplitude_m * gait_scale * sway_wave + look_y
        z = z0 + self.config.bob_amplitude_m * gait_scale * bob_wave
        yaw = self.config.yaw_deg + self.config.yaw_amplitude_deg * gait_scale * sway_wave + look_yaw
        pitch = self.config.pitch_amplitude_deg * gait_scale * pitch_wave + look_pitch
        return PoseSample(t, (x, y, z), quaternion_from_rpy_deg(0.0, pitch, yaw))
