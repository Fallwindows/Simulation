from __future__ import annotations

import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from simulator.presentation.provenance import (
    Artifact,
    _role_associations,
    _validate_slam_pose_coverage,
    _validate_slam_poses,
)


FIELDS = ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "quaternion_valid")


def _write_poses(path: Path, stamps: list[float]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(FIELDS)
        for index, stamp in enumerate(stamps):
            writer.writerow((stamp, index, 0, 0, 0, 0, 0, 1, 1))


class SlamProvenanceTests(unittest.TestCase):
    def test_pose_and_map_role_consumers_enforce_capture_clock_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "capture-id"
            capture_dir = run / "capture"
            slam_dir = run / "slam"
            bag = capture_dir / "sensors_bag"
            bag.mkdir(parents=True)
            slam_dir.mkdir()
            capture_path = capture_dir / "capture_manifest.json"
            slam_path = slam_dir / "slam_manifest.json"
            poses = slam_dir / "slam_poses.csv"
            pcd = slam_dir / "slam_map.pcd"
            snapshots = slam_dir / "map_snapshot_index.json"
            pcd.write_text("map", encoding="utf-8")
            snapshots.write_text(
                json.dumps({"status": "complete", "snapshots": [
                    {"measurement_cutoff_s": 0.2}, {"measurement_cutoff_s": 9.4}, {"measurement_cutoff_s": 18.6},
                ]}),
                encoding="utf-8",
            )
            capture = {
                "capture_id": "capture-id",
                "capture_sha256": "a" * 64,
                "bag": {"uri": "sensors_bag", "first_clock_s": 0.066666666, "last_clock_s": 20.533333333},
            }
            slam = {
                "capture_id": "capture-id",
                "capture_sha256": "a" * 64,
                "bag_replayed": str(bag),
                "producer_mode": "rtabmap_database_export",
                "producer": {"first_pose_timestamp_s": 0.2, "last_pose_timestamp_s": 18.6},
            }
            capture_path.write_text(json.dumps(capture), encoding="utf-8")
            slam_path.write_text(json.dumps(slam), encoding="utf-8")
            _write_poses(poses, [0.2, 9.4, 18.6])
            digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
            common = {
                "capture_manifest": Artifact("capture_manifest", capture_path, digest(capture_path), "capture_manifest"),
                "slam_manifest": Artifact("slam_manifest", slam_path, digest(slam_path), "slam_manifest"),
            }
            item = {"capture_id": "capture-id", "map_version": digest(slam_path)}
            _role_associations(
                "pose",
                item,
                {**common, "trajectory": Artifact("trajectory", poses, digest(poses), "slam_poses")},
            )
            _role_associations(
                "map",
                item,
                {
                    **common,
                    "pose_association": Artifact("pose_association", poses, digest(poses), "slam_poses"),
                    "map_states": Artifact("map_states", pcd, digest(pcd), "pcd"),
                    "snapshot_index": Artifact("snapshot_index", snapshots, digest(snapshots), "map_snapshot_index"),
                },
            )

            _write_poses(poses, [0.2, 5.0, 10.0])
            with self.assertRaisesRegex(ValueError, "does not cover the capture"):
                _role_associations(
                    "pose",
                    item,
                    {**common, "trajectory": Artifact("trajectory", poses, digest(poses), "slam_poses")},
                )
            snapshots.write_text(
                json.dumps({"status": "complete", "snapshots": [
                    {"measurement_cutoff_s": 0.2}, {"measurement_cutoff_s": 10.0},
                ]}),
                encoding="utf-8",
            )
            _write_poses(poses, [0.2, 9.4, 18.6])
            with self.assertRaisesRegex(ValueError, "map snapshots does not cover the capture"):
                _role_associations(
                    "map",
                    item,
                    {
                        **common,
                        "pose_association": Artifact("pose_association", poses, digest(poses), "slam_poses"),
                        "map_states": Artifact("map_states", pcd, digest(pcd), "pcd"),
                        "snapshot_index": Artifact("snapshot_index", snapshots, digest(snapshots), "map_snapshot_index"),
                    },
                )

    def test_actual_consumer_accepts_production_and_reviewed_legacy_time_ranges(self):
        capture = {"bag": {"first_clock_s": 0.066666666, "last_clock_s": 20.533333333}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slam_poses.csv"
            _write_poses(path, [0.2, 9.4, 18.6])
            slam = {
                "producer_mode": "rtabmap_database_export",
                "producer": {"first_pose_timestamp_s": 0.2, "last_pose_timestamp_s": 18.6},
            }
            _validate_slam_poses(path)
            _validate_slam_pose_coverage(path, capture, slam)

            _write_poses(path, [0.2, 10.0, 20.4])
            _validate_slam_poses(path)
            _validate_slam_pose_coverage(path, capture, {})

    def test_actual_consumer_rejects_truncation_and_incoherent_declared_range(self):
        capture = {"bag": {"first_clock_s": 0.0, "last_clock_s": 20.5}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slam_poses.csv"
            _write_poses(path, [0.2, 5.0, 10.0])
            with self.assertRaisesRegex(ValueError, "does not cover the capture"):
                _validate_slam_pose_coverage(path, capture, {})

            _write_poses(path, [0.2, 9.0, 18.6])
            inconsistent = {
                "producer_mode": "rtabmap_database_export",
                "producer": {"first_pose_timestamp_s": 0.3, "last_pose_timestamp_s": 18.6},
            }
            with self.assertRaisesRegex(ValueError, "does not match database-export provenance"):
                _validate_slam_pose_coverage(path, capture, inconsistent)

    def test_structural_pose_validation_still_rejects_non_increasing_and_invalid_quaternions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slam_poses.csv"
            _write_poses(path, [0.2, 0.2])
            with self.assertRaisesRegex(ValueError, "timestamps/quaternions"):
                _validate_slam_poses(path)
            _write_poses(path, [0.2, 1.0])
            rows = path.read_text(encoding="utf-8").replace(",0,0,0,1,1\n", ",0,0,0,0,1\n")
            path.write_text(rows, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "quaternion"):
                _validate_slam_poses(path)


if __name__ == "__main__":
    unittest.main()
