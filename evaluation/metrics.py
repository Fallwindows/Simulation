"""Deterministic trajectory metrics for synchronized ground truth and estimates.

The default report stays in the world frame; an explicit initial-translation
policy is available when origin alignment is desired. Sample matching is time
based and bounded so a stale estimate cannot masquerade as a valid measurement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Point = tuple[float, float, float]
Sample = tuple[float, Point]


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
        }


def _distance(a: Point, b: Point) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _subtract(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _nearest(sample: Sample, candidates: Sequence[Sample], max_gap_s: float) -> Sample | None:
    match = min(candidates, key=lambda item: abs(item[0] - sample[0]), default=None)
    if match is None or abs(match[0] - sample[0]) > max_gap_s:
        return None
    return match


def _tracking_loss_count(ground_truth: Sequence[Sample], matches: Sequence[Sample | None]) -> int:
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


def compute_metrics(
    ground_truth: Sequence[Sample],
    estimate: Sequence[Sample],
    rpe_interval_s: float | None = 1.0,
    max_time_gap_s: float = 0.1,
    alignment: str = "none",
) -> TrajectoryMetrics:
    """Compute ATE/RPE with bounded nearest-time matching.

    ``alignment='none'`` preserves the world-frame error (the default for
    backwards-compatible reports). ``alignment='initial_translation'`` removes
    only the first pose's translation offset; it never fits rotation, scale, or
    a full trajectory transform. RPE is the Euclidean error of relative
    translation vectors, not the difference of travel distances, so
    wrong-direction motion is detected.
    """
    if not ground_truth or not estimate:
        raise ValueError("ground_truth and estimate must be non-empty")
    if max_time_gap_s < 0:
        raise ValueError("max_time_gap_s must be non-negative")
    if alignment not in {"none", "initial_translation"}:
        raise ValueError("alignment must be none or initial_translation")

    ordered_gt = sorted(ground_truth, key=lambda item: item[0])
    ordered_est = sorted(estimate, key=lambda item: item[0])
    matches = [_nearest(sample, ordered_est, max_time_gap_s) for sample in ordered_gt]
    valid = [(gt, est) for gt, est in zip(ordered_gt, matches) if est is not None]
    if not valid:
        raise ValueError("no ground-truth samples have an estimate within max_time_gap_s")

    if alignment == "initial_translation":
        gt_first, est_first = valid[0][0][1], valid[0][1][1]
        translation = _subtract(gt_first, est_first)
    else:
        translation = (0.0, 0.0, 0.0)
    errors = [_distance(gt[1], tuple(est[1][i] + translation[i] for i in range(3))) for gt, est in valid]
    sorted_errors = sorted(errors)
    middle = len(sorted_errors) // 2
    median = sorted_errors[middle] if len(sorted_errors) % 2 else (sorted_errors[middle - 1] + sorted_errors[middle]) / 2.0

    rpe_errors: list[float] = []
    if rpe_interval_s is not None and rpe_interval_s > 0:
        for gt_now, est_now in valid:
            gt_future = next((item for item in ordered_gt if item[0] >= gt_now[0] + rpe_interval_s), None)
            if gt_future is None:
                continue
            est_future = _nearest(gt_future, ordered_est, max_time_gap_s)
            if est_future is None:
                continue
            gt_delta = _subtract(gt_future[1], gt_now[1])
            est_now_aligned = tuple(est_now[1][i] + translation[i] for i in range(3))
            est_future_aligned = tuple(est_future[1][i] + translation[i] for i in range(3))
            est_delta = _subtract(est_future_aligned, est_now_aligned)
            rpe_errors.append(_distance(gt_delta, est_delta))

    distance = sum(_distance(ordered_gt[i - 1][1], ordered_gt[i][1]) for i in range(1, len(ordered_gt)))
    duration = max(0.0, ordered_gt[-1][0] - ordered_gt[0][0])
    return TrajectoryMetrics(
        ate_rmse_m=math.sqrt(sum(error * error for error in errors) / len(errors)),
        ate_median_m=median,
        max_position_error_m=max(errors),
        rpe_rmse_m=None if not rpe_errors else math.sqrt(sum(error * error for error in rpe_errors) / len(rpe_errors)),
        rpe_interval_s=rpe_interval_s if rpe_errors else None,
        sample_count=len(valid),
        distance_traveled_m=distance,
        duration_s=duration,
        tracking_loss_count=_tracking_loss_count(ordered_gt, matches),
        alignment_policy=alignment,
        max_time_gap_s=max_time_gap_s,
        current_position_error_m=errors[-1],
    )
