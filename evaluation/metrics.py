"""Trajectory metrics for synchronized ground truth and estimate samples."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


Point = tuple[float, float, float]


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

    def as_dict(self) -> dict[str, float | int | None]:
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
        }


def _distance(a: Point, b: Point) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def compute_metrics(ground_truth: Sequence[tuple[float, Point]], estimate: Sequence[tuple[float, Point]], rpe_interval_s: float | None = 1.0) -> TrajectoryMetrics:
    if not ground_truth or not estimate:
        raise ValueError("ground_truth and estimate must be non-empty")
    estimated = list(estimate)
    errors = [_distance(gt_point, min(estimated, key=lambda item: abs(item[0] - t))[1]) for t, gt_point in ground_truth]
    squared = sum(e * e for e in errors)
    ordered_gt = list(ground_truth)
    distance = sum(_distance(ordered_gt[i - 1][1], ordered_gt[i][1]) for i in range(1, len(ordered_gt)))
    rpe_errors: list[float] = []
    if rpe_interval_s is not None and rpe_interval_s > 0:
        for t, gt_point in ordered_gt:
            future = next(((t2, p2) for t2, p2 in ordered_gt if t2 >= t + rpe_interval_s), None)
            if future is None:
                continue
            t2, gt_future = future
            est_now = min(estimated, key=lambda item: abs(item[0] - t))[1]
            est_future = min(estimated, key=lambda item: abs(item[0] - t2))[1]
            gt_delta = _distance(gt_future, gt_point)
            est_delta = _distance(est_future, est_now)
            rpe_errors.append(abs(est_delta - gt_delta))
    sorted_errors = sorted(errors)
    median = sorted_errors[len(sorted_errors) // 2] if len(sorted_errors) % 2 else (sorted_errors[len(sorted_errors) // 2 - 1] + sorted_errors[len(sorted_errors) // 2]) / 2.0
    return TrajectoryMetrics(math.sqrt(squared / len(errors)), median, max(errors), None if not rpe_errors else math.sqrt(sum(e * e for e in rpe_errors) / len(rpe_errors)), rpe_interval_s if rpe_errors else None, len(errors), distance, ordered_gt[-1][0] - ordered_gt[0][0], max(0, len(ground_truth) - len(estimate)))
