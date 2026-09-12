"""Portable run artifact serialization."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Sequence

from evaluation.metrics import PoseSample, as_pose_sample


def write_run(run_dir: str | Path, metadata: dict[str, Any], metrics: dict[str, Any], ground_truth: Sequence[PoseSample | tuple], estimate: Sequence[PoseSample | tuple], notes: str = "") -> Path:
    target = Path(run_dir)
    target.mkdir(parents=True, exist_ok=True)
    (target / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (target / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    for name, samples in (("ground_truth.csv", ground_truth), ("estimate.csv", estimate)):
        with (target / name).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"))
            writer.writerows((sample.timestamp_s, *sample.position_m, *sample.orientation_xyzw) for sample in (as_pose_sample(item) for item in samples))
    (target / "notes.txt").write_text(notes, encoding="utf-8")
    return target
