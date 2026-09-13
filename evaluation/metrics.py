"""Deterministic trajectory metrics with explicit initial-pose alignment.

Live baseline runs use alignment="initial_se3": the first matched estimate pose
is transformed into the first matched ground-truth pose using rotation and
translation only. Scale and full-trajectory fitting are never performed.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass
from typing import Sequence


Point = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
LegacySample = tuple[float, Point]
PoseTuple = tuple[float, Point, Quaternion]
Sample = "PoseSample | LegacySample | PoseTuple"


@dataclass(frozen=True)
class PoseSample:
    timestamp_s: float
    position_m: Point
    orientation_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class PoseAlignment:
    rotation_xyzw: Quaternion
    translation_m: Point
    policy: str

    def apply(self, sample: PoseSample) -> PoseSample:
        rotated = _rotate(self.rotation_xyzw, sample.position_m)
        position = tuple(rotated[i] + self.translation_m[i] for i in range(3))
        orientation = _normalize(_multiply(self.rotation_xyzw, sample.orientation_xyzw))
        return PoseSample(sample.timestamp_s, position, orientation)


@dataclass(frozen=True)
class TrajectoryMetrics:
    ate_rmse_m: float
    ate_median_m: float
    max_position_error_m: float
    rpe_rmse_m: float | None
    rpe_interval_s: float | None
    sample_count: int
    distance_traveled_m: float
    duration_s: float
    tracking_loss_count: int
    alignment_policy: str = "none"
    max_time_gap_s: float = 0.1
    current_position_error_m: float | None = None
    orientation_rmse_deg: float | None = None

    def as_dict(self) -> dict[str, float | int | str | None]:
        return {
            "ate_rmse_m": self.ate_rmse_m,
            "ate_median_m": self.ate_median_m,
            "max_position_error_m": self.max_position_error_m,
            "rpe_rmse_m": self.rpe_rmse_m,
            "rpe_interval_s": self.rpe_interval_s,
            "sample_count": self.sample_count,
            "distance_traveled_m": self.distance_traveled_m,
            "duration_s": self.duration_s,
            "tracking_loss_count": self.tracking_loss_count,
            "alignment_policy": self.alignment_policy,
            "max_time_gap_s": self.max_time_gap_s,
            "current_position_error_m": self.current_position_error_m,
            "orientation_rmse_deg": self.orientation_rmse_deg,
        }


def as_pose_sample(sample: PoseSample | LegacySample | PoseTuple) -> PoseSample | None:
    if isinstance(sample, PoseSample):
        orientation = safe_quaternion(sample.orientation_xyzw)
        return None if orientation is None else PoseSample(sample.timestamp_s, sample.position_m, orientation)
    timestamp, position, *rest = sample
    orientation = safe_quaternion(rest[0]) if rest else (0.0, 0.0, 0.0, 1.0)
    if orientation is None:
        return None
    return PoseSample(float(timestamp), tuple(float(v) for v in position), orientation)  # type: ignore[arg-type]


def safe_quaternion(values) -> Quaternion | None:
    """Normalize a ROS quaternion, returning ``None`` for invalid samples."""
    orientation = tuple(float(value) for value in values)
    if len(orientation) != 4:
        raise ValueError("orientation must contain four quaternion values")
    norm = math.sqrt(sum(value * value for value in orientation))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return tuple(value / norm for value in orientation)  # type: ignore[return-value]


def _normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        raise ValueError("zero quaternion")
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def _conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def _multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q: Quaternion, point: Point) -> Point:
    vector = _multiply(_multiply(_normalize(q), (point[0], point[1], point[2], 0.0)), _conjugate(_normalize(q)))
    return (vector[0], vector[1], vector[2])


def _subtract(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _distance(a: Point, b: Point) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def nearest_pose(sample: PoseSample, candidates: Sequence[PoseSample], max_gap_s: float) -> PoseSample | None:
    match = min(candidates, key=lambda item: abs(item.timestamp_s - sample.timestamp_s), default=None)
    if match is None or abs(match.timestamp_s - sample.timestamp_s) > max_gap_s:
        return None
    return match


def _valid_pose_samples(samples: Sequence[PoseSample | LegacySample | PoseTuple]) -> list[PoseSample]:
    return sorted(
        (sample for item in samples if (sample := as_pose_sample(item)) is not None),
        key=lambda item: item.timestamp_s,
    )


def _slerp(a: Quaternion, b: Quaternion, fraction: float) -> Quaternion:
    """Spherical interpolation for normalized ``xyzw`` quaternions."""
    first = _normalize(a)
    second = _normalize(b)
    dot = sum(x * y for x, y in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)  # type: ignore[assignment]
        dot = -dot
    if dot > 0.9995:
        return _normalize(tuple(x + fraction * (y - x) for x, y in zip(first, second)))  # type: ignore[arg-type]
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sine = math.sin(theta)
    first_weight = math.sin((1.0 - fraction) * theta) / sine
    second_weight = math.sin(fraction * theta) / sine
    return _normalize(tuple(first_weight * x + second_weight * y for x, y in zip(first, second)))  # type: ignore[arg-type]


def interpolate_pose(samples: Sequence[PoseSample], timestamp_s: float, max_gap_s: float) -> PoseSample | None:
    """Interpolate high-rate truth at an estimator timestamp.

    The gap bound applies to the bracketing truth interval and to endpoint
    extrapolation.  No nearest-sample substitution is performed.
    """
    if not samples:
        return None
    ordered = samples if all(samples[i - 1].timestamp_s <= samples[i].timestamp_s for i in range(1, len(samples))) else sorted(samples, key=lambda item: item.timestamp_s)
    timestamps = [sample.timestamp_s for sample in ordered]
    index = bisect_left(timestamps, timestamp_s)
    if index < len(ordered) and timestamps[index] == timestamp_s:
        return ordered[index]
    if index == 0:
        return ordered[0] if ordered[0].timestamp_s - timestamp_s <= max_gap_s else None
    if index == len(ordered):
        return ordered[-1] if timestamp_s - ordered[-1].timestamp_s <= max_gap_s else None

    before, after = ordered[index - 1], ordered[index]
    span = after.timestamp_s - before.timestamp_s
    if span <= 0.0 or span > max_gap_s:
        return None
    fraction = (timestamp_s - before.timestamp_s) / span
    position = tuple(before.position_m[i] + fraction * (after.position_m[i] - before.position_m[i]) for i in range(3))
    orientation = _slerp(before.orientation_xyzw, after.orientation_xyzw, fraction)
    return PoseSample(timestamp_s, position, orientation)  # type: ignore[arg-type]


def initial_se3_alignment(ground_truth: Sequence[PoseSample], estimate: Sequence[PoseSample], max_gap_s: float) -> PoseAlignment:
    ordered_gt = sorted(ground_truth, key=lambda item: item.timestamp_s)
    for est in sorted(estimate, key=lambda item: item.timestamp_s):
        gt = interpolate_pose(ordered_gt, est.timestamp_s, max_gap_s)
        if gt is not None:
            rotation = _normalize(_multiply(gt.orientation_xyzw, _conjugate(est.orientation_xyzw)))
            translation = _subtract(gt.position_m, _rotate(rotation, est.position_m))
            return PoseAlignment(rotation, translation, "initial_se3")
    raise ValueError("no initial pose pair within max_time_gap_s")


def initial_translation_alignment(ground_truth: Sequence[PoseSample], estimate: Sequence[PoseSample], max_gap_s: float) -> PoseAlignment:
    ordered_gt = sorted(ground_truth, key=lambda item: item.timestamp_s)
    for est in sorted(estimate, key=lambda item: item.timestamp_s):
        gt = interpolate_pose(ordered_gt, est.timestamp_s, max_gap_s)
        if gt is not None:
            return PoseAlignment((0.0, 0.0, 0.0, 1.0), _subtract(gt.position_m, est.position_m), "initial_translation")
    raise ValueError("no initial pose pair within max_time_gap_s")


def _alignment(ground_truth: Sequence[PoseSample], estimate: Sequence[PoseSample], policy: str, max_gap_s: float) -> PoseAlignment | None:
    if policy == "none":
        return None
    if policy == "initial_translation":
        return initial_translation_alignment(ground_truth, estimate, max_gap_s)
    if policy == "initial_se3":
        return initial_se3_alignment(ground_truth, estimate, max_gap_s)
    raise ValueError("alignment must be none, initial_translation, or initial_se3")


def _orientation_error_deg(ground_truth: PoseSample, estimate: PoseSample) -> float:
    relative = _normalize(_multiply(_conjugate(ground_truth.orientation_xyzw), estimate.orientation_xyzw))
    scalar = min(1.0, max(-1.0, abs(relative[3])))
    return math.degrees(2.0 * math.acos(scalar))


def compute_metrics(
    ground_truth: Sequence[PoseSample | LegacySample | PoseTuple],
    estimate: Sequence[PoseSample | LegacySample | PoseTuple],
    rpe_interval_s: float | None = 1.0,
    max_time_gap_s: float = 0.1,
    alignment: str = "none",
) -> TrajectoryMetrics:
    """Evaluate each odometry timestamp against interpolated high-rate truth."""
    if not ground_truth or not estimate:
        raise ValueError("ground_truth and estimate must be non-empty")
    if max_time_gap_s < 0:
        raise ValueError("max_time_gap_s must be non-negative")

    ordered_gt = _valid_pose_samples(ground_truth)
    ordered_est = _valid_pose_samples(estimate)
    if not ordered_gt or not ordered_est:
        raise ValueError("no valid ground-truth and estimate samples remain after quaternion validation")
    alignment_transform = _alignment(ordered_gt, ordered_est, alignment, max_time_gap_s)
    matches = [interpolate_pose(ordered_gt, sample.timestamp_s, max_time_gap_s) for sample in ordered_est]
    valid = [(gt, est) for est, gt in zip(ordered_est, matches) if gt is not None]
    if not valid:
        raise ValueError("no interpolated ground-truth samples are available at estimator timestamps")

    aligned_estimate = [alignment_transform.apply(est) if alignment_transform else est for _, est in valid]
    errors = [_distance(gt.position_m, est.position_m) for (gt, _), est in zip(valid, aligned_estimate)]
    sorted_errors = sorted(errors)
    middle = len(sorted_errors) // 2
    median = sorted_errors[middle] if len(sorted_errors) % 2 else (sorted_errors[middle - 1] + sorted_errors[middle]) / 2.0

    rpe_errors: list[float] = []
    if rpe_interval_s is not None and rpe_interval_s > 0:
        for index, (gt_now, est_now) in enumerate(valid):
            target_timestamp = est_now.timestamp_s + rpe_interval_s
            future = next(((future_index, pair) for future_index, pair in enumerate(valid[index + 1:], start=index + 1) if pair[1].timestamp_s >= target_timestamp), None)
            if future is None:
                continue
            _, (gt_future, est_future) = future
            est_now_aligned = alignment_transform.apply(est_now) if alignment_transform else est_now
            est_future_aligned = alignment_transform.apply(est_future) if alignment_transform else est_future
            rpe_errors.append(_distance(_subtract(gt_future.position_m, gt_now.position_m), _subtract(est_future_aligned.position_m, est_now_aligned.position_m)))

    orientation_errors = [_orientation_error_deg(gt, est) for (gt, _), est in zip(valid, aligned_estimate)]
    distance = sum(_distance(valid[i - 1][0].position_m, valid[i][0].position_m) for i in range(1, len(valid)))
    duration = max(0.0, valid[-1][0].timestamp_s - valid[0][0].timestamp_s)
    return TrajectoryMetrics(
        ate_rmse_m=math.sqrt(sum(error * error for error in errors) / len(errors)),
        ate_median_m=median,
        max_position_error_m=max(errors),
        rpe_rmse_m=None if not rpe_errors else math.sqrt(sum(error * error for error in rpe_errors) / len(rpe_errors)),
        rpe_interval_s=rpe_interval_s if rpe_errors else None,
        sample_count=len(valid),
        distance_traveled_m=distance,
        duration_s=duration,
        tracking_loss_count=_tracking_loss_count(matches),
        alignment_policy=alignment,
        max_time_gap_s=max_time_gap_s,
        current_position_error_m=errors[-1],
        orientation_rmse_deg=math.sqrt(sum(error * error for error in orientation_errors) / len(orientation_errors)) if orientation_errors else None,
    )


def _tracking_loss_count(matches: Sequence[PoseSample | None]) -> int:
    """Count contiguous unmatched GT runs, rather than counting every frame."""
    losses = 0
    in_loss = False
    for match in matches:
        if match is None and not in_loss:
            losses += 1
            in_loss = True
        elif match is not None:
            in_loss = False
    return losses
