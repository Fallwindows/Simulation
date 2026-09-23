from __future__ import annotations

import hashlib
import json
import unittest
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

import numpy as np

from evaluation.metrics import PoseSample
from simulator.perception.inventory_evaluation import (
    _interpolate_truth_start_pose,
    evaluate_inventory,
    _quat_to_matrix,
    _match_rows,
    _slam_start_timestamp,
    _start_relative_truth_rows,
    _write_inventory_map,
    _validate_eligibility_manifest,
    _load_eligibility_manifest,
)


def truth(semantic_id: str, position: tuple[float, float, float], category: str = "cereal"):
    return {
        "semantic_id": semantic_id,
        "asset_key": "asset",
        "category": category,
        "relative": np.asarray(position, dtype=np.float64),
        "world": np.asarray(position, dtype=np.float64),
    }


class InventoryEvaluationTests(unittest.TestCase):
    def test_partial_coordinates_do_not_default_to_origin(self):
        estimates = [{
            "track_id": "7",
            "estimated_x_m": "0",
            "estimated_y_m": "",
            "estimated_z_m": "0",
        }]
        rows, metrics = _match_rows(estimates, [truth("origin", (0.0, 0.0, 0.0))])
        self.assertEqual(metrics["invalid_estimate_coordinate_count"], 1)
        self.assertEqual(metrics["spatially_associated_estimate_count"], 0)
        self.assertIsNone(rows[0]["relative_y_m"])
        self.assertEqual(rows[0]["spatially_associated_gt_id"], "")

    def test_assignment_maximizes_cardinality_before_minimizing_distance(self):
        estimates = [
            {"track_id": "1", "estimated_x_m": "0.16", "estimated_y_m": "0", "estimated_z_m": "0"},
            {"track_id": "2", "estimated_x_m": "-0.17", "estimated_y_m": "0", "estimated_z_m": "0"},
        ]
        gt = [truth("left", (0.0, 0.0, 0.0)), truth("right", (0.34, 0.0, 0.0))]
        rows, metrics = _match_rows(estimates, gt, threshold_m=0.35)
        self.assertEqual(metrics["spatially_associated_estimate_count"], 2)
        self.assertEqual({row["spatially_associated_gt_id"] for row in rows}, {"left", "right"})

    def test_visible_eligibility_is_supplied_independently_of_match_success(self):
        estimates = [
            {"track_id": "1", "estimated_x_m": "0.02", "estimated_y_m": "0", "estimated_z_m": "0"},
            {"track_id": "2", "estimated_x_m": "5.01", "estimated_y_m": "0", "estimated_z_m": "0"},
        ]
        gt = [truth("eligible", (0.0, 0.0, 0.0)), truth("occluded", (5.0, 0.0, 0.0))]
        rows, metrics = _match_rows(estimates, gt, eligible_truth_ids={"eligible"})
        self.assertEqual({row["spatially_associated_gt_id"] for row in rows}, {"eligible", "occluded"})
        self.assertEqual(metrics["eligible_ground_truth_count"], 1)
        self.assertEqual(metrics["eligible_ground_truth_association_count"], 1)
        self.assertEqual(metrics["eligible_recall"], 1.0)
        self.assertEqual(metrics["category_agreement_evaluated_count"], 0)

    def test_assignment_minimizes_distance_after_cardinality(self):
        estimates = [
            {"track_id": "1", "estimated_x_m": "1", "estimated_y_m": "0", "estimated_z_m": "0"},
            {"track_id": "2", "estimated_x_m": "-1.1", "estimated_y_m": "0", "estimated_z_m": "0"},
        ]
        gt = [truth("left", (0.0, 0.0, 0.0)), truth("right", (3.0, 0.0, 0.0))]
        rows, metrics = _match_rows(estimates, gt, threshold_m=4.2)
        self.assertEqual(metrics["spatially_associated_estimate_count"], 2)
        self.assertEqual({row["spatially_associated_gt_id"] for row in rows}, {"left", "right"})
        self.assertAlmostEqual(sum(row["spatial_association_error_m"] for row in rows), 3.1)

    def test_evaluate_inventory_persists_validated_eligibility_evidence(self):
        estimates = [{"track_id": "1", "estimated_x_m": "0", "estimated_y_m": "0", "estimated_z_m": "0"}]
        slam_poses = [{"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"}]
        aligned_truth = [truth("eligible", (0.0, 0.0, 0.0)), truth("occluded", (5.0, 0.0, 0.0))]
        evidence = {"schema_version": 1, "capture_id": "run", "method": "manual review", "source_sha256": "source-hash", "eligible_truth_ids": ["eligible"], "ineligible_truth_ids": ["occluded"]}
        with (
            patch("simulator.perception.inventory_evaluation._load_csv", side_effect=[estimates, slam_poses]),
            patch("simulator.perception.inventory_evaluation._gt_start_relative", return_value=aligned_truth),
            patch("simulator.perception.inventory_evaluation._load_eligibility_manifest", return_value=({"eligible"}, evidence)),
            patch("simulator.perception.inventory_evaluation._write_csv"),
            patch("simulator.perception.inventory_evaluation._write_xlsx"),
            patch("simulator.perception.inventory_evaluation._write_inventory_map", return_value=0),
            patch.object(Path, "write_text"),
        ):
            result = evaluate_inventory("run/capture", "run/slam", "run/perception", eligibility_manifest_path="labels.json")
        self.assertEqual(result["eligibility_evidence"], evidence)
        self.assertEqual(result["metrics"]["eligible_ground_truth_count"], 1)

    def test_eligibility_manifest_binds_capture_rule_hash_and_exhaustive_ids(self):
        source = {
            "schema_version": 1,
            "capture_id": "capture-42",
            "labels": {"visible": "eligible", "occluded": "ineligible"},
        }
        source_bytes = json.dumps(source, sort_keys=True).encode("utf-8")
        source_hash = hashlib.sha256(source_bytes).hexdigest()
        manifest = {
            "schema_version": 1,
            "capture_id": "capture-42",
            "method": "manual visibility review v1",
            "rule": "eligible when visible area and dwell criteria are met",
            "frozen_at_utc": "2026-09-20T12:00:00Z",
            "source": {"path": "visibility-labels.json", "sha256": source_hash},
            "eligible_truth_ids": ["visible"],
            "ineligible_truth_ids": ["occluded"],
        }
        manifest_bytes = json.dumps(manifest, sort_keys=True).encode("utf-8")
        with patch.object(Path, "read_bytes", side_effect=[manifest_bytes, source_bytes]):
            eligible, evidence = _load_eligibility_manifest(
                Path("eligibility.json"),
                "capture-42",
                {"visible", "occluded"},
                datetime(2026, 9, 22, tzinfo=timezone.utc),
            )
        self.assertEqual(eligible, {"visible"})
        self.assertEqual(evidence["ineligible_truth_ids"], ["occluded"])
        self.assertEqual(evidence["source_sha256"], source_hash)
        self.assertEqual(evidence["manifest_sha256"], hashlib.sha256(manifest_bytes).hexdigest())
        self.assertEqual(evidence["method"], manifest["method"])

    def test_eligibility_manifest_must_precede_evaluation(self):
        source = {
            "schema_version": 1,
            "capture_id": "capture-42",
            "labels": {"visible": "eligible"},
        }
        source_hash = hashlib.sha256(json.dumps(source, sort_keys=True).encode("utf-8")).hexdigest()
        manifest = {
            "schema_version": 1,
            "capture_id": "capture-42",
            "method": "manual review",
            "rule": "declared visibility rule",
            "frozen_at_utc": "2026-09-23T12:00:00Z",
            "source": {"path": "visibility-labels.json", "sha256": source_hash},
            "eligible_truth_ids": ["visible"],
            "ineligible_truth_ids": [],
        }
        with self.assertRaisesRegex(ValueError, "frozen before evaluation"):
            _validate_eligibility_manifest(
                manifest, source, "manifest-hash", source_hash, Path("eligibility.json"),
                "capture-42", {"visible"}, datetime(2026, 9, 22, tzinfo=timezone.utc),
            )

    def test_unknown_visibility_keeps_eligible_metrics_unavailable(self):
        _, metrics = _match_rows(
            [{"track_id": "1", "estimated_x_m": "0", "estimated_y_m": "0", "estimated_z_m": "0"}],
            [truth("authored", (0.0, 0.0, 0.0))],
        )
        self.assertIsNone(metrics["eligible_ground_truth_count"])
        self.assertIsNone(metrics["eligible_recall"])
        self.assertEqual(metrics["eligibility_status"], "unavailable_no_independent_visibility_labels")

    def test_even_sized_spatial_error_median_uses_average_of_middle_pair(self):
        estimates = [
            {"track_id": "1", "estimated_x_m": "0.1", "estimated_y_m": "0", "estimated_z_m": "0"},
            {"track_id": "2", "estimated_x_m": "1.3", "estimated_y_m": "0", "estimated_z_m": "0"},
        ]
        _, metrics = _match_rows(estimates, [truth("a", (0, 0, 0)), truth("b", (1, 0, 0))], threshold_m=0.4)
        self.assertAlmostEqual(metrics["median_spatial_association_error_m"], 0.2)

    def test_start_pose_is_interpolated_at_shifted_slam_timestamp(self):
        pose = _interpolate_truth_start_pose([
            PoseSample(101.0, (0.0, 0.0, 0.0)),
            PoseSample(101.1, (2.0, 0.0, 0.0)),
        ], 101.05)
        self.assertEqual(pose.timestamp_s, 101.05)
        self.assertAlmostEqual(pose.position_m[0], 1.0)

    def test_slam_start_timestamp_selects_actual_first_estimate_stamp(self):
        with patch("simulator.perception.inventory_evaluation._load_csv", return_value=[
            {"timestamp_s": "100.0", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "0"},
            {"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
        ]):
            self.assertEqual(_slam_start_timestamp(Path("unused")), 100.2)

    def test_nonfinite_position_on_first_valid_quaternion_fails_closed(self):
        for bad_position in ("nan", "inf"):
            with self.subTest(position=bad_position), patch(
                "simulator.perception.inventory_evaluation._load_csv",
                return_value=[
                    {"timestamp_s": "100.0", "x_m": bad_position, "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
                    {"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
                ],
            ):
                with self.assertRaisesRegex(RuntimeError, "first accepted pose"):
                    _slam_start_timestamp(Path("unused"))

    def test_evaluation_aligns_truth_to_slam_start_timestamp(self):
        estimates = [{"track_id": "1", "estimated_x_m": "0", "estimated_y_m": "0", "estimated_z_m": "0"}]
        slam_poses = [
            {"timestamp_s": "100.4", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
            {"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
        ]
        aligned_truth = [truth("item", (0.0, 0.0, 0.0))]
        with (
            patch("simulator.perception.inventory_evaluation._load_csv", side_effect=[estimates, slam_poses]),
            patch("simulator.perception.inventory_evaluation._gt_start_relative", return_value=aligned_truth) as make_truth,
            patch("simulator.perception.inventory_evaluation._write_csv"),
            patch("simulator.perception.inventory_evaluation._write_xlsx"),
            patch("simulator.perception.inventory_evaluation._write_inventory_map", return_value=0),
            patch.object(Path, "write_text"),
        ):
            result = evaluate_inventory("capture", "slam", "perception")
        make_truth.assert_called_once()
        self.assertEqual(make_truth.call_args.args[1], 100.2)
        self.assertEqual(result["evaluation_start_timestamp_s"], 100.2)

    def test_truth_transform_uses_nonidentity_start_translation_and_rotation(self):
        start_t = np.asarray([10.0, -2.0, 1.0])
        start_r = _quat_to_matrix((0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)))
        rows = _start_relative_truth_rows([
            {"semantic_id": "item", "x_m": "11", "y_m": "-2", "z_m": "1", "category": "cereal"}
        ], start_t, start_r)
        np.testing.assert_allclose(rows[0]["relative"], [0.0, -1.0, 0.0], atol=1e-8)

    def test_inventory_map_uses_corrected_map_coordinates_not_start_relative(self):
        source = Path("slam_map.ply")
        output = Path("map_with_inventory.ply")
        source_text = "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
        rows = [{
            "relative_x_m": 1.0, "relative_y_m": 2.0, "relative_z_m": 3.0,
            "map_x_m": 8.0, "map_y_m": 9.0, "map_z_m": 10.0,
        }]
        with patch.object(Path, "read_text", return_value=source_text), patch.object(Path, "write_text") as write_text:
            self.assertEqual(_write_inventory_map(output, source, rows), 1)
        rendered_map = write_text.call_args.args[0]
        self.assertIn("8.0 9.0 10.0 235 45 45", rendered_map)
        self.assertNotIn("1.0 2.0 3.0 235 45 45", rendered_map)


if __name__ == "__main__":
    unittest.main()
