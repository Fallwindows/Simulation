from __future__ import annotations

import hashlib
import json
import csv
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch

import numpy as np

import simulator.perception.inventory_evaluation as evaluation
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
    INVENTORY_FIELDS,
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
        estimates = [{"track_id": "1", "estimated_x_m": "0", "estimated_y_m": "0", "estimated_z_m": "0", "map_version": "a" * 64}]
        slam_poses = [{"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"}]
        aligned_truth = [truth("eligible", (0.0, 0.0, 0.0)), truth("occluded", (5.0, 0.0, 0.0))]
        evidence = {"schema_version": 1, "capture_id": "run", "method": "manual review", "source_sha256": "source-hash", "eligible_truth_ids": ["eligible"], "ineligible_truth_ids": ["occluded"]}
        with (
            patch("simulator.perception.inventory_evaluation._validate_map_provenance", return_value=({"map_version": "a" * 64}, b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n", b"timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw\n100.2,0,0,0,0,0,0,1\n")),
            patch("simulator.perception.inventory_evaluation._load_csv_with_fields", return_value=(["track_id", "estimated_x_m", "estimated_y_m", "estimated_z_m", "map_version"], estimates)),
            patch("simulator.perception.inventory_evaluation._load_csv", return_value=slam_poses),
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
        estimates = [{"track_id": "1", "estimated_x_m": "0", "estimated_y_m": "0", "estimated_z_m": "0", "map_version": "a" * 64}]
        slam_poses = [
            {"timestamp_s": "100.4", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
            {"timestamp_s": "100.2", "x_m": "0", "y_m": "0", "z_m": "0", "qx": "0", "qy": "0", "qz": "0", "qw": "1"},
        ]
        aligned_truth = [truth("item", (0.0, 0.0, 0.0))]
        with (
            patch("simulator.perception.inventory_evaluation._validate_map_provenance", return_value=({"map_version": "a" * 64}, b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n", b"timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw\n100.2,0,0,0,0,0,0,1\n")),
            patch("simulator.perception.inventory_evaluation._load_csv_with_fields", return_value=(["track_id", "estimated_x_m", "estimated_y_m", "estimated_z_m", "map_version"], estimates)),
            patch("simulator.perception.inventory_evaluation._load_csv", return_value=slam_poses),
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
        with patch.object(Path, "write_text") as write_text:
            self.assertEqual(_write_inventory_map(output, source_text.encode("utf-8"), rows), 1)
        rendered_map = write_text.call_args.args[0]
        self.assertIn("8.0 9.0 10.0 235 45 45", rendered_map)
        self.assertNotIn("1.0 2.0 3.0 235 45 45", rendered_map)


class InventoryEvaluationProvenanceTests(unittest.TestCase):
    ARTIFACTS = {
        "slam_map_poses.csv", "slam_map_keyframes.csv", "slam_odom_poses.csv", "map_to_odom.csv",
        "slam_poses.csv", "slam_map.pcd", "slam_map.ply",
    }

    @staticmethod
    def _sha(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def _fixture(self, root: Path) -> tuple[Path, Path, Path]:
        capture = root / "run" / "capture"
        slam = root / "run" / "slam"
        perception = root / "run" / "perception"
        for directory in (capture, slam, perception):
            directory.mkdir(parents=True)
        contents = {
            "slam_map_poses.csv": "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n1,0,0,0,0,0,0,1,map\n",
            "slam_map_keyframes.csv": "node_id,timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n1,1,0,0,0,0,0,1,map\n",
            "slam_odom_poses.csv": "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n1,0,0,0,0,0,0,1,odom\n",
            "map_to_odom.csv": "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,parent_frame_id,child_frame_id\n1,0,0,0,0,0,0,1,map,odom\n",
            "slam_poses.csv": "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n1,0,0,0,0,0,0,1,map\n",
            "slam_map.pcd": "# PCD fixture\n",
            "slam_map.ply": "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n1 2 3\n",
        }
        for name, content in contents.items():
            (slam / name).write_text(content, encoding="ascii")
        observer = {
            "status": "complete", "map_version": "a" * 64, "graph_pose_version": "b" * 64,
            "pre_publish_graph_version": "c" * 64,
            "pre_publish_source_graph_identity": "d" * 64,
            "final_source_graph_identity": "d" * 64,
            "source_graph_identity_matches_pre_publish": True,
            "pre_publish_optimized_pose_version": "e" * 64,
            "final_optimized_pose_version": "f" * 64,
            "pre_publish_map_to_odom_version": "0" * 64,
            "final_map_to_odom_version": "1" * 64,
            "pre_publish_source_graph_link_count": 1,
            "final_source_graph_link_count": 1,
            "pre_publish_source_graph_link_type_histogram": {"0": 1},
            "final_source_graph_link_type_histogram": {"0": 1},
            "map_pose_frame_id": "map",
            "map_graph_matches_final_cloud": True,
            "map_data_graph_fingerprint": "2" * 64,
            "map_graph_fingerprint": "2" * 64,
            "map_data_matches_map_graph": True,
            "cached_cloud_graph_fingerprint": "2" * 64,
            "final_cloud_graph_fingerprint": "2" * 64,
            "map_cloud_identity_state": "fresh_shared_publication",
            "final_cloud_origin": "fresh_post_publish",
            "final_map_graph_stamp_s": 1.0,
            "final_cloud_stamp_s": 1.0,
            "final_map_graph_frame_id": "map",
            "final_cloud_frame_id": "map",
            "optimized_pose_graph_complete": True,
        }
        observer_path = slam / "slam_observer.json"
        observer_path.write_text(json.dumps(observer, sort_keys=True), encoding="utf-8")
        metadata = {
            "slam_map_poses.csv": ("dense_corrected_trajectory", "map", True),
            "slam_map_keyframes.csv": ("optimized_graph_keyframes", "map", True),
            "slam_odom_poses.csv": ("raw_odometry_diagnostic", "odom", False),
            "map_to_odom.csv": ("incremental_tf_diagnostic", "map->odom", False),
            "slam_poses.csv": ("legacy_map_trajectory", "map", True),
            "slam_map.pcd": ("final_optimized_cloud", "map", True),
            "slam_map.ply": ("final_optimized_cloud", "map", True),
        }
        records = []
        for name in sorted(self.ARTIFACTS):
            path = slam / name
            role, frame, optimized = metadata[name]
            records.append({
                "path": name, "role": role, "frame_id": frame, "optimized": optimized,
                "map_version": "a" * 64 if optimized else None,
                "size_bytes": path.stat().st_size, "sha256": self._sha(path),
            })
        slam_record = {
            "status": "complete", "map_version": "a" * 64, "graph_pose_version": "b" * 64,
            "pre_publish_graph_version": "c" * 64, "map_frame_id": "map", "optimized": True,
            "observer": observer,
            "observer_artifact": {"path": observer_path.name, "size_bytes": observer_path.stat().st_size, "sha256": self._sha(observer_path)},
            "artifacts": records,
        }
        for key in (
            "pre_publish_source_graph_identity", "final_source_graph_identity",
            "source_graph_identity_matches_pre_publish",
            "pre_publish_optimized_pose_version", "final_optimized_pose_version",
            "pre_publish_map_to_odom_version", "final_map_to_odom_version",
            "pre_publish_source_graph_link_count", "final_source_graph_link_count",
            "pre_publish_source_graph_link_type_histogram", "final_source_graph_link_type_histogram",
            "map_cloud_identity_state", "final_cloud_origin",
            "map_data_graph_fingerprint", "map_graph_fingerprint", "map_data_matches_map_graph",
            "cached_cloud_graph_fingerprint", "final_cloud_graph_fingerprint", "map_graph_matches_final_cloud",
        ):
            slam_record[key] = observer[key]
        slam_manifest = slam / "slam_manifest.json"
        slam_manifest.write_text(json.dumps(slam_record, sort_keys=True), encoding="utf-8")
        perception_record = {
            "status": "complete", "map_version": "a" * 64,
            "pre_publish_source_graph_identity": observer["pre_publish_source_graph_identity"],
            "final_source_graph_identity": observer["final_source_graph_identity"],
            "slam_manifest_sha256": self._sha(slam_manifest), "slam_manifest_size_bytes": slam_manifest.stat().st_size,
            "slam_cloud_ply_sha256": self._sha(slam / "slam_map.ply"),
            "slam_cloud_ply_size_bytes": (slam / "slam_map.ply").stat().st_size,
        }
        (perception / "perception_manifest.json").write_text(json.dumps(perception_record, sort_keys=True), encoding="utf-8")
        with (perception / "estimated_inventory.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                "track_id", "class", "estimated_x_m", "estimated_y_m", "estimated_z_m", "map_x_m", "map_y_m", "map_z_m",
                "3d_observation_count", "map_version",
            ])
            writer.writeheader()
            writer.writerow({
                "track_id": "4", "class": "cereal", "estimated_x_m": "1", "estimated_y_m": "2", "estimated_z_m": "3",
                "map_x_m": "4", "map_y_m": "5", "map_z_m": "6", "3d_observation_count": "5", "map_version": "a" * 64,
            })
        return capture, slam, perception

    def _rebind_perception_manifest(self, slam: Path, perception: Path) -> None:
        path = slam / "slam_manifest.json"
        output = perception / "perception_manifest.json"
        record = json.loads(output.read_text(encoding="utf-8"))
        record["slam_manifest_sha256"] = self._sha(path)
        record["slam_manifest_size_bytes"] = path.stat().st_size
        output.write_text(json.dumps(record, sort_keys=True), encoding="utf-8")

    def _evaluate(self, capture: Path, slam: Path, perception: Path):
        aligned_truth = [truth("truth-1", (1.0, 2.0, 3.0), "cereal")]
        with patch.object(evaluation, "_gt_start_relative", return_value=aligned_truth):
            return evaluation.evaluate_inventory(capture, slam, perception)

    def test_valid_integrity_is_recorded_without_changing_p13_inventory_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            capture, slam, perception = self._fixture(Path(temporary))
            manifest = json.loads((slam / "slam_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual({item["path"] for item in manifest["artifacts"]}, self.ARTIFACTS)
            result = self._evaluate(capture, slam, perception)
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["map_version"], "a" * 64)
            self.assertEqual(result["input_provenance"]["slam_map_cloud"]["sha256"], self._sha(slam / "slam_map.ply"))
            with (perception / "inventory.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(list(rows[0]), INVENTORY_FIELDS)
            self.assertNotIn("map_version", rows[0])
            saved = json.loads((perception / "inventory_evaluation.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["input_provenance"], result["input_provenance"])

    def test_each_of_the_seven_runtime_artifacts_is_required_before_truth(self):
        for missing in sorted(self.ARTIFACTS):
            with self.subTest(missing=missing), tempfile.TemporaryDirectory() as temporary:
                capture, slam, perception = self._fixture(Path(temporary))
                path = slam / "slam_manifest.json"
                manifest = json.loads(path.read_text(encoding="utf-8"))
                manifest["artifacts"] = [record for record in manifest["artifacts"] if record["path"] != missing]
                path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
                self._rebind_perception_manifest(slam, perception)
                with patch.object(evaluation, "_gt_start_relative", side_effect=AssertionError("truth opened before provenance validation")):
                    with self.assertRaisesRegex(ValueError, "artifact set is incomplete"):
                        evaluation.evaluate_inventory(capture, slam, perception)

    def test_validated_cloud_bytes_are_used_after_later_source_replacement(self):
        with tempfile.TemporaryDirectory() as temporary:
            capture, slam, perception = self._fixture(Path(temporary))
            replacement = (slam / "slam_map.ply").read_bytes().replace(b"1 2 3", b"91 92 93")
            aligned_truth = [truth("truth-1", (1.0, 2.0, 3.0), "cereal")]

            def replace_cloud(_capture: Path, _timestamp: float):
                (slam / "slam_map.ply").write_bytes(replacement)
                return aligned_truth

            with patch.object(evaluation, "_gt_start_relative", side_effect=replace_cloud):
                evaluation.evaluate_inventory(capture, slam, perception)
            output = (slam / "slam_map_with_inventory.ply").read_bytes()
            self.assertIn(b"1 2 3 150 150 150", output)
            self.assertNotIn(b"91 92 93 150 150 150", output)

    def test_pose_csv_replacement_after_manifest_fails_before_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            capture, slam, perception = self._fixture(Path(temporary))
            (slam / "slam_poses.csv").write_text(
                "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n999,0,0,0,0,0,0,1,map\n",
                encoding="ascii",
            )
            with patch.object(evaluation, "_gt_start_relative", side_effect=AssertionError("truth opened with an unverified pose stream")):
                with self.assertRaisesRegex(ValueError, "slam_poses.csv size or SHA-256"):
                    evaluation.evaluate_inventory(capture, slam, perception)

    def test_timestamp_uses_pose_bytes_validated_before_later_source_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            capture, slam, perception = self._fixture(Path(temporary))
            original_validator = evaluation._validate_map_provenance
            aligned_truth = [truth("truth-1", (1.0, 2.0, 3.0), "cereal")]

            def validate_then_replace(slam_dir: Path, perception_dir: Path):
                result = original_validator(slam_dir, perception_dir)
                (slam / "slam_poses.csv").write_text(
                    "timestamp_s,x_m,y_m,z_m,qx,qy,qz,qw,frame_id\n999,0,0,0,0,0,0,1,map\n",
                    encoding="ascii",
                )
                return result

            with (
                patch.object(evaluation, "_validate_map_provenance", side_effect=validate_then_replace),
                patch.object(evaluation, "_gt_start_relative", return_value=aligned_truth) as make_truth,
            ):
                result = evaluation.evaluate_inventory(capture, slam, perception)
            self.assertEqual(result["evaluation_start_timestamp_s"], 1.0)
            self.assertEqual(make_truth.call_args.args[1], 1.0)

    def test_duplicate_estimate_header_fails_before_truth_and_map_version_must_match(self):
        cases = (("duplicate", "duplicate CSV column"), ("mismatch", "inconsistent map_version"), ("missing", "missing the required map_version column"))
        for case, error in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temporary:
                capture, slam, perception = self._fixture(Path(temporary))
                estimate_path = perception / "estimated_inventory.csv"
                with estimate_path.open(newline="", encoding="utf-8") as handle:
                    reader = csv.DictReader(handle)
                    fields, rows = list(reader.fieldnames or []), list(reader)
                if case == "duplicate":
                    fields.append("map_version")
                elif case == "mismatch":
                    rows[0]["map_version"] = "d" * 64
                else:
                    fields = [field for field in fields if field != "map_version"]
                with estimate_path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(fields)
                    for row in rows:
                        writer.writerow([row.get(field, "") for field in fields])
                with patch.object(evaluation, "_gt_start_relative", side_effect=AssertionError("truth opened before input validation")):
                    with self.assertRaisesRegex(ValueError, error):
                        evaluation.evaluate_inventory(capture, slam, perception)

    def test_observer_cloud_and_path_alias_integrity_fail_closed(self):
        cases = (
            "observer", "ply_hash", "ply_size", "cloud_tamper", "slam_manifest_hash", "perception_map_version",
            "perception_cloud_hash", "perception_cloud_size", "casefold_path", "unsafe_path", "extra_path",
            "duplicate_path", "malformed_record",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temporary:
                capture, slam, perception = self._fixture(Path(temporary))
                manifest_path = slam / "slam_manifest.json"
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                perception_path = perception / "perception_manifest.json"
                perception_manifest = json.loads(perception_path.read_text(encoding="utf-8"))
                if case == "observer":
                    manifest["observer"]["extra_but_unbound"] = "value"
                elif case in {"ply_hash", "ply_size"}:
                    ply = next(item for item in manifest["artifacts"] if item["path"] == "slam_map.ply")
                    ply["sha256" if case == "ply_hash" else "size_bytes"] = "0" * 64 if case == "ply_hash" else ply["size_bytes"] + 1
                elif case == "cloud_tamper":
                    path = slam / "slam_map.ply"
                    path.write_bytes(path.read_bytes() + b"\n")
                elif case == "slam_manifest_hash":
                    manifest["optimized"] = False
                elif case == "perception_map_version":
                    perception_manifest["map_version"] = "c" * 64
                elif case == "perception_cloud_hash":
                    perception_manifest["slam_cloud_ply_sha256"] = "0" * 64
                elif case == "perception_cloud_size":
                    perception_manifest["slam_cloud_ply_size_bytes"] += 1
                elif case == "casefold_path":
                    manifest["artifacts"][0]["path"] = manifest["artifacts"][0]["path"].upper()
                elif case == "unsafe_path":
                    manifest["artifacts"][0]["path"] = "../" + manifest["artifacts"][0]["path"]
                elif case == "extra_path":
                    extra = dict(manifest["artifacts"][0])
                    extra["path"] = "extra.bin"
                    manifest["artifacts"].append(extra)
                elif case == "duplicate_path":
                    manifest["artifacts"].append(dict(manifest["artifacts"][0]))
                elif case == "malformed_record":
                    manifest["artifacts"][0]["path"] = None
                if case not in {"perception_map_version", "perception_cloud_hash", "perception_cloud_size", "cloud_tamper"}:
                    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
                    self._rebind_perception_manifest(slam, perception)
                else:
                    perception_path.write_text(json.dumps(perception_manifest, sort_keys=True), encoding="utf-8")
                with patch.object(evaluation, "_gt_start_relative", side_effect=AssertionError("truth opened before provenance validation")):
                    with self.assertRaises(ValueError):
                        evaluation.evaluate_inventory(capture, slam, perception)


if __name__ == "__main__":
    unittest.main()
