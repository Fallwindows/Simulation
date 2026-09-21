"""Validate the native RGB timestamp cadence without hiding subscriber loss."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable


def analyze_rgb_cadence(
    stamps_s: Iterable[float],
    expected_fps: float,
    *,
    target_stamp_s: float | None = None,
) -> dict[str, object]:
    stamps = [float(value) for value in stamps_s]
    if not math.isfinite(expected_fps) or expected_fps <= 0.0:
        raise ValueError("expected_fps must be positive and finite")
    if any(not math.isfinite(value) for value in stamps):
        raise ValueError("RGB timestamps must be finite")

    interval_s = 1.0 / expected_fps
    tolerance_s = min(0.001, interval_s * 0.1)
    missing_intervals = 0
    irregular_intervals = 0
    duplicate_intervals = 0
    out_of_order_intervals = 0
    max_gap_s = 0.0
    interval_errors_s: list[float] = []
    for previous, current in zip(stamps, stamps[1:]):
        delta_s = current - previous
        max_gap_s = max(max_gap_s, delta_s)
        if delta_s == 0.0:
            duplicate_intervals += 1
            continue
        if delta_s < 0.0:
            out_of_order_intervals += 1
            continue
        steps = max(1, int(round(delta_s / interval_s)))
        error_s = abs(delta_s - steps * interval_s)
        interval_errors_s.append(error_s)
        if error_s > tolerance_s:
            irregular_intervals += 1
        if steps > 1:
            missing_intervals += steps - 1

    trailing_missing_intervals = 0
    if target_stamp_s is not None and stamps:
        target = float(target_stamp_s)
        if not math.isfinite(target):
            raise ValueError("target_stamp_s must be finite")
        delta_to_target_s = target - stamps[-1]
        if delta_to_target_s > interval_s + tolerance_s:
            trailing_missing_intervals = max(0, int(round(delta_to_target_s / interval_s)) - 1)

    contiguous = (
        bool(stamps)
        and missing_intervals == 0
        and trailing_missing_intervals == 0
        and irregular_intervals == 0
        and duplicate_intervals == 0
        and out_of_order_intervals == 0
    )
    return {
        "status": "contiguous" if contiguous else "incomplete",
        "window_mode": "first_to_last_observed_rgb_with_one_interval_target_tail_allowance",
        "expected_fps": expected_fps,
        "nominal_interval_s": interval_s,
        "interval_tolerance_s": tolerance_s,
        "observed_frame_count": len(stamps),
        "observed_interval_count": max(0, len(stamps) - 1),
        "first_stamp_s": stamps[0] if stamps else None,
        "last_stamp_s": stamps[-1] if stamps else None,
        "target_stamp_s": target_stamp_s,
        "missing_intervals": missing_intervals,
        "trailing_missing_intervals": trailing_missing_intervals,
        "irregular_intervals": irregular_intervals,
        "duplicate_intervals": duplicate_intervals,
        "out_of_order_intervals": out_of_order_intervals,
        "max_gap_s": max_gap_s if len(stamps) > 1 else None,
        "max_interval_error_s": max(interval_errors_s, default=None),
        "contiguous": contiguous,
    }


def verify_common_rgb_window(
    recorder_stamps_s: Iterable[float],
    bag_stamps_s: Iterable[float],
    expected_fps: float,
    target_stamp_s: float,
) -> dict[str, object]:
    recorder_stamps = [float(value) for value in recorder_stamps_s]
    bag_stamps = [float(value) for value in bag_stamps_s]
    recorder = analyze_rgb_cadence(recorder_stamps, expected_fps, target_stamp_s=target_stamp_s)
    bag = analyze_rgb_cadence(bag_stamps, expected_fps, target_stamp_s=target_stamp_s)
    interval_s = 1.0 / expected_fps
    tolerance_s = float(recorder["interval_tolerance_s"])

    common_start_s = max(recorder_stamps[0], bag_stamps[0]) if recorder_stamps and bag_stamps else None
    common_end_s = min(recorder_stamps[-1], bag_stamps[-1]) if recorder_stamps and bag_stamps else None
    recorder_common: list[float] = []
    bag_common: list[float] = []
    if common_start_s is not None and common_end_s is not None and common_end_s >= common_start_s:
        recorder_common = [
            value for value in recorder_stamps
            if common_start_s - tolerance_s <= value <= common_end_s + tolerance_s
        ]
        bag_common = [
            value for value in bag_stamps
            if common_start_s - tolerance_s <= value <= common_end_s + tolerance_s
        ]
    paired = len(recorder_common) == len(bag_common) and all(
        abs(recorder_stamp - bag_stamp) <= tolerance_s
        for recorder_stamp, bag_stamp in zip(recorder_common, bag_common)
    )
    common = analyze_rgb_cadence(recorder_common, expected_fps) if paired else {
        "status": "incomplete",
        "contiguous": False,
        "observed_frame_count": len(recorder_common),
    }
    target_tail_ok = bool(recorder_stamps and bag_stamps) and all(
        target_stamp_s - stamps[-1] <= interval_s + tolerance_s
        for stamps in (recorder_stamps, bag_stamps)
    )
    complete = bool(recorder["contiguous"] and bag["contiguous"] and paired and common["contiguous"] and target_tail_ok)
    return {
        "status": "complete" if complete else "incomplete",
        "expected_fps": expected_fps,
        "nominal_interval_s": interval_s,
        "interval_tolerance_s": tolerance_s,
        "startup_window_mode": "starts_at_later_first_observed_rgb_stamp",
        "common_start_stamp_s": common_start_s,
        "common_end_stamp_s": common_end_s,
        "pre_common_start_s_not_evaluated": common_start_s,
        "target_stamp_s": target_stamp_s,
        "target_tail_allowance_s": interval_s + tolerance_s,
        "target_tail_ok": target_tail_ok,
        "recorder": recorder,
        "bag": bag,
        "common_frame_count": len(recorder_common),
        "common_stamps_match": paired,
        "common_cadence": common,
    }


def _read_frame_stamps(path: Path) -> list[float]:
    return [json.loads(line)["stamp_s"] for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--recorder-frames", required=True)
    parser.add_argument("--bag-metadata", required=True)
    parser.add_argument("--expected-fps", type=float, required=True)
    parser.add_argument("--target-stamp-seconds", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    bag_metadata = json.loads(Path(args.bag_metadata).read_text(encoding="utf-8"))
    result = verify_common_rgb_window(
        _read_frame_stamps(Path(args.recorder_frames)),
        bag_metadata["rgb_cadence"]["stamps_s"],
        args.expected_fps,
        args.target_stamp_seconds,
    )
    output = Path(args.output)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if result["status"] != "complete":
        raise RuntimeError(f"RGB recorder/bag cadence validation failed: {result}")


if __name__ == "__main__":
    main()
