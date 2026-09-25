import unittest
import struct
import csv
import hashlib
import json
import tempfile
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from evaluation.metrics import PoseSample
from simulator.technical_lidar import CameraHeadTransformTrajectory
from simulator.perception.rgb_tracking import (
    BlobTracker,
    Detection,
    _augment_with_lidar_estimates,
    _budget_detections_spatially,
    _co_visible_raw_track_pairs,
    _consolidate_track_estimates,
    _decode_pointcloud2_xyz,
    _finite_xyz_points,
    _map_estimates_to_start_relative,
    _load_sensor_geometry,
    _load_slam_poses,
    _maximum_cardinality_minimum_cost_assignment,
    _set_canonical_track_identity,
    _set_unassigned_persistent_identity,
    _resolution_scaled_component_limits,
    _select_lidar_point_indices,
    _spatial_grid_candidate_indices,
    _world_points_to_camera,
    detect_product_blobs,
    run_rgb_tracking,
)


def _write_pose_integrity_manifests(slam_directory: Path, map_pose_count: int) -> None:
    (slam_directory / "slam_map_keyframes.csv").write_text("optimized keyframe fixture\n", encoding="utf-8")
    (slam_directory / "slam_poses.csv").write_text("legacy map trajectory fixture\n", encoding="utf-8")
    (slam_directory / "map_to_odom.csv").write_text("incremental correction fixture\n", encoding="utf-8")
    (slam_directory / "slam_map.pcd").write_bytes(b"fixture final map cloud")
    (slam_directory / "slam_map.ply").write_bytes(b"fixture final map ply")
    map_version = "e" * 64
    dense_pose_version = "d" * 64
    files = []
    for name, metadata in (
        ("slam_map_poses.csv", {"role": "dense_corrected_trajectory", "frame_id": "map", "optimized": True, "map_version": map_version, "dense_pose_version": dense_pose_version}),
        ("slam_map_keyframes.csv", {"role": "optimized_graph_keyframes", "frame_id": "map", "optimized": True, "map_version": map_version, "dense_pose_version": None}),
        ("slam_poses.csv", {"role": "legacy_map_trajectory", "frame_id": "map", "optimized": True, "map_version": map_version, "dense_pose_version": None}),
        ("slam_odom_poses.csv", {"role": "raw_odometry_diagnostic", "frame_id": "odom", "optimized": False, "map_version": None, "dense_pose_version": None}),
        ("map_to_odom.csv", {"role": "incremental_tf_diagnostic", "frame_id": "map->odom", "optimized": False, "map_version": None, "dense_pose_version": None}),
        ("slam_map.pcd", {"role": "final_optimized_cloud", "frame_id": "map", "optimized": True, "map_version": map_version, "dense_pose_version": None}),
        ("slam_map.ply", {"role": "final_optimized_cloud", "frame_id": "map", "optimized": True, "map_version": map_version, "dense_pose_version": None}),
    ):
        artifact = slam_directory / name
        files.append({
            "path": name,
            "size_bytes": artifact.stat().st_size,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            **metadata,
            "schema_version": 2 if name == "slam_map_keyframes.csv" else 1,
            "correction_policy": (
                "derive optimized_node_pose * inverse(raw_node_odom_pose); linear translation + "
                "quaternion slerp between node timestamps; no extrapolation"
            ) if name in ("slam_map_poses.csv", "slam_map_keyframes.csv") else None,
        })
    observer = {
        "status": "complete",
        "files": files,
        "pose_source": "rtabmap_optimized_graph",
        "graph_pose_version": "a" * 64,
        "pre_publish_graph_version": "a" * 64,
        "dense_pose_version": dense_pose_version,
        "map_version": map_version,
        "map_graph_matches_final_cloud": True,
        "optimized_pose_graph_complete": True,
        "map_pose_frame_id": "map",
        "map_pose_sample_count": map_pose_count,
        "odom_sample_count": map_pose_count,
    }
    observer_bytes = json.dumps(observer).encode("utf-8")
    (slam_directory / "slam_observer.json").write_bytes(observer_bytes)
    manifest = {
        "status": "complete",
        "pre_publish_graph_version": observer["pre_publish_graph_version"],
        "graph_pose_version": observer["graph_pose_version"],
        "dense_pose_version": dense_pose_version,
        "map_version": map_version,
        "map_frame_id": "map",
        "optimized": True,
        "artifacts": files,
        "observer_artifact": {
            "path": "slam_observer.json",
            "size_bytes": len(observer_bytes),
            "sha256": hashlib.sha256(observer_bytes).hexdigest(),
        },
        "observer": observer,
    }
    (slam_directory / "slam_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def _rebind_observer_artifact(slam_directory: Path, observer: dict[str, object], *, top_level: dict[str, object] | None = None) -> None:
    observer_bytes = json.dumps(observer).encode("utf-8")
    (slam_directory / "slam_observer.json").write_bytes(observer_bytes)
    manifest_path = slam_directory / "slam_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["observer"] = observer
    manifest["artifacts"] = observer["files"]
    manifest["observer_artifact"] = {
        "path": "slam_observer.json",
        "size_bytes": len(observer_bytes),
        "sha256": hashlib.sha256(observer_bytes).hexdigest(),
    }
    for key, value in (top_level or {}).items():
        manifest[key] = value
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")


def _write_pose_contract_case(
    root: Path,
    name: str,
    map_rows: list[dict[str, object]],
    odom_rows: list[dict[str, object]],
    *,
    map_pose_count: int = 2,
    odom_sample_count: int = 2,
) -> Path:
    directory = root / name
    directory.mkdir()
    fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"]
    for filename, rows in (("slam_map_poses.csv", map_rows), ("slam_odom_poses.csv", odom_rows)):
        with (directory / filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    _write_pose_integrity_manifests(directory, map_pose_count)
    observer_path = directory / "slam_observer.json"
    observer = json.loads(observer_path.read_text(encoding="utf-8"))
    observer["odom_sample_count"] = odom_sample_count
    _rebind_observer_artifact(directory, observer)
    return directory


class PerceptionTests(unittest.TestCase):
    def test_rgb_blob_detector_returns_measured_centers(self):
        image = np.zeros((120, 180, 3), dtype=np.uint8)
        cv2.rectangle(image, (20, 30), (54, 68), (0, 0, 220), -1)
        cv2.rectangle(image, (100, 20), (144, 60), (0, 210, 0), -1)
        detections = detect_product_blobs(image)
        self.assertEqual(len(detections), 2)
        centers = {tuple(round(value, 1) for value in item.center_px) for item in detections}
        self.assertIn((37.0, 49.0), centers)
        self.assertIn((122.0, 40.0), centers)

    def test_tracker_reuses_ids_for_small_motion(self):
        image_a = np.zeros((120, 180, 3), dtype=np.uint8)
        image_b = np.zeros((120, 180, 3), dtype=np.uint8)
        cv2.rectangle(image_a, (20, 30), (54, 68), (0, 0, 220), -1)
        cv2.rectangle(image_b, (24, 31), (58, 69), (0, 0, 220), -1)
        tracker = BlobTracker()
        first = tracker.update(0, detect_product_blobs(image_a))
        second = tracker.update(1, detect_product_blobs(image_b))
        self.assertEqual(len(first), 1)
        self.assertEqual(len(second), 1)
        self.assertEqual(first[0]["track_id"], second[0]["track_id"])

    def test_3d_track_consolidation_merges_rgb_fragments_without_ground_truth(self):
        mapping, centers, members = _consolidate_track_estimates({
            12: np.asarray([2.00, -1.00, 0.80]),
            31: np.asarray([2.08, -1.02, 0.81]),
            44: np.asarray([4.00, 1.00, 1.20]),
        }, merge_radius_m=0.15)
        self.assertEqual(len(centers), 2)
        self.assertEqual(mapping[12], mapping[31])
        self.assertNotEqual(mapping[12], mapping[44])
        self.assertEqual(sorted(members[mapping[12]]), [12, 31])

    def test_stationary_tracks_at_distant_positions_and_resolutions_keep_one_id(self):
        for width, height in ((640, 360), (1280, 720), (1920, 1080)):
            for u, v in ((width * 0.04, height * 0.05), (width * 0.96, height * 0.94), (width * 0.5, height * 0.5)):
                detection = Detection((0, 0, 20, 20), (u, v), 441, 0.0, 240.0, 0.9)
                tracker = BlobTracker()
                ids = []
                for frame_index in range(180):
                    ids.extend(row["track_id"] for row in tracker.update(frame_index, [detection]))
                self.assertEqual(len(set(ids)), 1, (width, height, u, v))
                track = tracker.tracks[ids[0]]
                self.assertEqual(track.center_px, (u, v))
                self.assertAlmostEqual(track.velocity_px[0], 0.0)
                self.assertAlmostEqual(track.velocity_px[1], 0.0)

    def test_elapsed_frame_prediction_and_missed_lifecycle_use_frame_indices(self):
        tracker = BlobTracker(max_missed_frames=3)
        first = Detection((0, 0, 20, 20), (100.0, 100.0), 441, 0.0, 240.0, 0.9)
        moved = Detection((0, 0, 20, 20), (103.0, 100.0), 441, 0.0, 240.0, 0.9)
        returned = Detection((0, 0, 20, 20), (106.0, 100.0), 441, 0.0, 240.0, 0.9)
        first_row = tracker.update(0, [first])[0]
        self.assertIsNone(first_row["persistent_track_id"])
        self.assertEqual(first_row["id_namespace"], "raw_rgb")
        second_row = tracker.update(3, [moved])[0]
        self.assertEqual(first_row["raw_track_id"], second_row["raw_track_id"])
        self.assertAlmostEqual(tracker.tracks[1].velocity_px[0], 0.3)
        tracker.update(4, [])
        tracker.update(5, [])
        self.assertEqual(tracker.tracks[1].missed_frames, 2)
        third_row = tracker.update(6, [returned])[0]
        self.assertEqual(third_row["raw_track_id"], first_row["raw_track_id"])
        self.assertEqual(tracker.tracks[1].missed_frames, 0)

    def test_association_gate_scales_for_equivalent_motion_at_multiple_resolutions(self):
        for width, height, displacement in ((1280, 720, 60.0), (1920, 1080, 90.0)):
            image_size = (width, height)
            first = Detection((0, 0, 30, 30), (300.0, 300.0), 900, 0.0, 240.0, 0.9, image_size)
            moved = Detection((0, 0, 30, 30), (300.0 + displacement, 300.0), 900, 0.0, 240.0, 0.9, image_size)
            tracker = BlobTracker()
            first_row = tracker.update(0, [first])[0]
            moved_row = tracker.update(1, [moved])[0]
            self.assertEqual(first_row["raw_track_id"], moved_row["raw_track_id"], (width, height))

    def test_global_assignment_preserves_maximum_cardinality_when_greedy_fails(self):
        tracker = BlobTracker(max_match_distance_px=85.0)
        make_detection = lambda x: Detection((0, 0, 20, 20), (float(x), 100.0), 400, 0.0, 240.0, 0.9, (1280, 720))
        initial = tracker.update(0, [make_detection(0), make_detection(80)])
        self.assertEqual([row["raw_track_id"] for row in initial], [1, 2])

        # Track 1 can reach both detections, while track 2 can only reach
        # detection 0. Greedy (track 1, detection 0) loses one match.
        crossing = tracker.update(1, [make_detection(40), make_detection(-80)])
        by_id = {row["raw_track_id"]: row["center_px"] for row in crossing}
        self.assertEqual(set(by_id), {1, 2})
        self.assertEqual(by_id[1], [-80.0, 100.0])
        self.assertEqual(by_id[2], [40.0, 100.0])

    def test_global_assignment_minimizes_cost_among_maximum_cardinality_solutions(self):
        costs = {(1, 0): 1.0, (1, 1): 2.0, (2, 0): 1.5, (2, 1): 100.0}
        expected = [(1, 1), (2, 0)]
        self.assertEqual(_maximum_cardinality_minimum_cost_assignment([1, 2], 2, costs), expected)
        self.assertEqual(_maximum_cardinality_minimum_cost_assignment([1, 2], 2, costs), expected)

    def test_two_object_approach_cross_occlusion_and_revisit_keep_unique_ids(self):
        tracker = BlobTracker(max_missed_frames=2, max_revisit_frames=50)

        def object_detection(x, hue):
            return Detection((0, 0, 20, 20), (float(x), 100.0), 400, float(hue), 240.0, 0.9, (1280, 720))

        same_appearance = 75.0
        sequence = [
            [object_detection(100, same_appearance), object_detection(300, same_appearance)],
            [object_detection(140, same_appearance), object_detection(260, same_appearance)],
            [object_detection(180, same_appearance), object_detection(220, same_appearance)],
            [object_detection(220, same_appearance), object_detection(180, same_appearance)], # physical crossing
            [object_detection(260, same_appearance), object_detection(140, same_appearance)],
            [object_detection(300, same_appearance), object_detection(100, same_appearance)],
            [object_detection(340, same_appearance)],                                         # B briefly occluded
            [object_detection(380, same_appearance)],
            [], [], [],                                                                      # archive with nonzero exit velocity
            [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [],  # longer hidden interval
            [object_detection(380, same_appearance), object_detection(100, same_appearance)], # revisit at last observed locations
        ]
        rows_by_frame = []
        lifecycle_by_frame = []
        velocity_by_frame = []
        for frame_index, detections in enumerate(sequence):
            rows = tracker.update(frame_index, detections)
            ids = [row["raw_track_id"] for row in rows]
            self.assertEqual(len(ids), len(set(ids)), frame_index)
            rows_by_frame.append(rows)
            lifecycle_by_frame.append({track_id: track.lifecycle_state for track_id, track in tracker.tracks.items()})
            velocity_by_frame.append({track_id: track.association_velocity_px for track_id, track in tracker.tracks.items()})
        a_id = rows_by_frame[0][0]["raw_track_id"]
        b_id = rows_by_frame[0][1]["raw_track_id"]
        self.assertNotEqual(a_id, b_id)
        expected_centers = {
            1: {140.0: a_id, 260.0: b_id},
            2: {180.0: a_id, 220.0: b_id},
            3: {220.0: a_id, 180.0: b_id},
            4: {260.0: a_id, 140.0: b_id},
            5: {300.0: a_id, 100.0: b_id},
            len(sequence) - 1: {380.0: a_id, 100.0: b_id},
        }
        for frame_index, expected in expected_centers.items():
            observed = {row["center_px"][0]: row["raw_track_id"] for row in rows_by_frame[frame_index]}
            self.assertEqual(observed, expected, frame_index)
        self.assertEqual(rows_by_frame[6][0]["raw_track_id"], a_id)
        self.assertEqual(rows_by_frame[7][0]["raw_track_id"], a_id)
        self.assertEqual(lifecycle_by_frame[8][a_id], "occluded")
        self.assertNotEqual(velocity_by_frame[8][a_id], (0.0, 0.0))
        self.assertEqual(lifecycle_by_frame[10][a_id], "archived")
        self.assertEqual(lifecycle_by_frame[10][b_id], "archived")
        self.assertEqual(tracker.tracks[a_id].lifecycle_state, "active")
        self.assertEqual(tracker.tracks[b_id].lifecycle_state, "active")
        self.assertEqual(set(tracker.tracks), {a_id, b_id})

    def test_archived_tracks_do_not_claim_ids_after_camera_view_shift_or_expiry(self):
        tracker = BlobTracker(max_missed_frames=1, max_revisit_frames=10)
        make = lambda x: Detection((0, 0, 20, 20), (float(x), 100.0), 400, 75.0, 240.0, 0.9, (1280, 720))
        first = tracker.update(0, [make(100), make(300)])
        original_ids = {row["center_px"][0]: row["raw_track_id"] for row in first}
        tracker.update(1, [])
        tracker.update(2, [])
        tracker.update(3, [])
        self.assertTrue(all(track.lifecycle_state == "archived" for track in tracker.tracks.values()))
        # A 110px camera-relative shift has no image-only instance evidence here.
        # The tracker must start new raw tracks instead of silently assigning
        # the old IDs to a different physical location based on hue.
        shifted = tracker.update(4, [make(210), make(410)])
        self.assertEqual(len({row["raw_track_id"] for row in shifted}), 2)
        self.assertTrue(all(row["raw_track_id"] not in set(original_ids.values()) for row in shifted))
        tracker.update(20, [])
        self.assertFalse(set(original_ids.values()) & set(tracker.tracks))

    def test_first_frame_beyond_occlusion_uses_archived_prediction(self):
        tracker = BlobTracker(max_missed_frames=2, max_revisit_frames=12)
        make = lambda x: Detection((0, 0, 20, 20), (float(x), 100.0), 400, 75.0, 240.0, 0.9, (1280, 720))
        first = tracker.update(0, [make(100)])[0]
        tracker.update(1, [make(140)])
        tracker.update(2, [make(180)])
        tracker.update(3, [])
        tracker.update(4, [])
        self.assertEqual(tracker.tracks[1].lifecycle_state, "occluded")
        reacquired = tracker.update(5, [make(180)])
        self.assertEqual(reacquired[0]["raw_track_id"], first["raw_track_id"])
        self.assertEqual(set(tracker.tracks), {1})

    def test_map_corrected_estimate_derives_start_coordinates_through_rotated_pose(self):
        start_t = np.asarray([10.0, -2.0, 1.0])
        half_sqrt = 2.0 ** -0.5
        start_q = (0.0, 0.0, half_sqrt, half_sqrt)
        start_r = np.asarray([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        observed_start = np.asarray([2.0, 0.5, 3.0])
        observed_map = start_t + observed_start @ start_r.T
        map_correction = np.asarray([0.25, -0.10, 0.05])
        corrected_map = observed_map + map_correction
        raw_to_canonical, canonical_map, members = _consolidate_track_estimates({7: corrected_map})
        self.assertEqual(raw_to_canonical[7], 1)
        self.assertEqual(members[1], [7])
        start_relative = _map_estimates_to_start_relative(canonical_map, start_t, start_q)
        expected_start = (corrected_map - start_t) @ start_r
        np.testing.assert_allclose(canonical_map[1], corrected_map, atol=1e-12)
        np.testing.assert_allclose(start_relative[1], expected_start, atol=1e-12)
        np.testing.assert_allclose(start_t + start_relative[1] @ start_r.T, canonical_map[1], atol=1e-12)

    def test_corrected_map_pose_loader_drives_time_varying_lidar_map_estimate(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            slam_directory = Path(temporary_directory)
            map_pose_path = slam_directory / "slam_map_poses.csv"
            half_sqrt = 2.0 ** -0.5
            fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"]
            def write_pose_file(path, rows):
                with path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)

            corrected_rows = [
                {"timestamp_s": 0.0, "x_m": 10.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": half_sqrt, "qw": half_sqrt, "frame_id": "map"},
                {"timestamp_s": 0.1, "x_m": 12.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": half_sqrt, "qw": half_sqrt, "frame_id": "map"},
            ]
            with map_pose_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(corrected_rows)
            # A raw odom file alongside this artifact must never be treated as
            # a corrected map stream or silently used as its fallback.
            raw_odom_path = slam_directory / "slam_odom_poses.csv"
            with raw_odom_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows([
                    {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                    for row in corrected_rows
                ])
            _write_pose_integrity_manifests(slam_directory, 2)

            incomplete_directory = slam_directory / "incomplete_manifest"
            incomplete_directory.mkdir()
            write_pose_file(incomplete_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(incomplete_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(incomplete_directory, 2)
            top_manifest_path = incomplete_directory / "slam_manifest.json"
            top_manifest = json.loads(top_manifest_path.read_text(encoding="utf-8"))
            top_manifest["status"] = "incomplete"
            top_manifest_path.write_text(json.dumps(top_manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Top-level SLAM manifest is not complete"):
                _load_slam_poses(incomplete_directory / "slam_map_poses.csv")

            for case_name, invalid_count in (
                ("true", True), ("false", False), ("string", "2"),
                ("float", 2.0), ("zero", 0), ("negative", -1), ("null", None), ("missing", "__missing__"),
            ):
                count_directory = slam_directory / f"bad_count_{case_name}"
                count_directory.mkdir()
                write_pose_file(count_directory / "slam_map_poses.csv", corrected_rows)
                write_pose_file(count_directory / "slam_odom_poses.csv", [
                    {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                    for row in corrected_rows
                ])
                _write_pose_integrity_manifests(count_directory, 2)
                count_observer_path = count_directory / "slam_observer.json"
                count_observer = json.loads(count_observer_path.read_text(encoding="utf-8"))
                if invalid_count == "__missing__":
                    del count_observer["map_pose_sample_count"]
                else:
                    count_observer["map_pose_sample_count"] = invalid_count
                count_observer_bytes = json.dumps(count_observer).encode("utf-8")
                count_observer_path.write_bytes(count_observer_bytes)
                count_manifest_path = count_directory / "slam_manifest.json"
                count_manifest = json.loads(count_manifest_path.read_text(encoding="utf-8"))
                count_manifest["observer"] = count_observer
                count_manifest["observer_artifact"] = {
                    "path": "slam_observer.json",
                    "size_bytes": len(count_observer_bytes),
                    "sha256": hashlib.sha256(count_observer_bytes).hexdigest(),
                }
                count_manifest_path.write_text(json.dumps(count_manifest), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "map_pose_sample_count must be a positive integer"):
                    _load_slam_poses(count_directory / "slam_map_poses.csv")

            version_mismatch_directory = slam_directory / "version_mismatch"
            version_mismatch_directory.mkdir()
            write_pose_file(version_mismatch_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(version_mismatch_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(version_mismatch_directory, 2)
            observer_path = version_mismatch_directory / "slam_observer.json"
            observer_record = json.loads(observer_path.read_text(encoding="utf-8"))
            observer_record["pre_publish_graph_version"] = "c" * 64
            observer_bytes = json.dumps(observer_record).encode("utf-8")
            observer_path.write_bytes(observer_bytes)
            top_manifest_path = version_mismatch_directory / "slam_manifest.json"
            top_manifest = json.loads(top_manifest_path.read_text(encoding="utf-8"))
            top_manifest["observer"]["pre_publish_graph_version"] = "c" * 64
            top_manifest["pre_publish_graph_version"] = "c" * 64
            top_manifest["observer_artifact"] = {
                "path": "slam_observer.json",
                "size_bytes": len(observer_bytes),
                "sha256": hashlib.sha256(observer_bytes).hexdigest(),
            }
            top_manifest_path.write_text(json.dumps(top_manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "does not match the pre-publish graph version"):
                _load_slam_poses(version_mismatch_directory / "slam_map_poses.csv")

            observer_tamper_directory = slam_directory / "observer_tamper"
            observer_tamper_directory.mkdir()
            write_pose_file(observer_tamper_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(observer_tamper_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(observer_tamper_directory, 2)
            observer_path = observer_tamper_directory / "slam_observer.json"
            tampered_observer = json.loads(observer_path.read_text(encoding="utf-8"))
            tampered_observer["map_version"] = "f" * 64
            observer_path.write_text(json.dumps(tampered_observer), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "slam_observer.json size or SHA-256"):
                _load_slam_poses(observer_tamper_directory / "slam_map_poses.csv")

            observer_mismatch_directory = slam_directory / "observer_mismatch"
            observer_mismatch_directory.mkdir()
            write_pose_file(observer_mismatch_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(observer_mismatch_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(observer_mismatch_directory, 2)
            observer_path = observer_mismatch_directory / "slam_observer.json"
            changed_observer = json.loads(observer_path.read_text(encoding="utf-8"))
            changed_observer["unembedded_diagnostic"] = "different but validly rebound observer"
            observer_bytes = json.dumps(changed_observer).encode("utf-8")
            observer_path.write_bytes(observer_bytes)
            mismatch_manifest_path = observer_mismatch_directory / "slam_manifest.json"
            mismatch_manifest = json.loads(mismatch_manifest_path.read_text(encoding="utf-8"))
            mismatch_manifest["observer_artifact"] = {
                "path": "slam_observer.json",
                "size_bytes": len(observer_bytes),
                "sha256": hashlib.sha256(observer_bytes).hexdigest(),
            }
            mismatch_manifest_path.write_text(json.dumps(mismatch_manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "On-disk and embedded SLAM observer records disagree"):
                _load_slam_poses(observer_mismatch_directory / "slam_map_poses.csv")

            duplicate_directory = slam_directory / "duplicate_artifact"
            duplicate_directory.mkdir()
            write_pose_file(duplicate_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(duplicate_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(duplicate_directory, 2)
            duplicate_manifest_path = duplicate_directory / "slam_manifest.json"
            duplicate_manifest = json.loads(duplicate_manifest_path.read_text(encoding="utf-8"))
            duplicate_manifest["artifacts"].append(dict(duplicate_manifest["artifacts"][0]))
            duplicate_manifest_path.write_text(json.dumps(duplicate_manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate artifact path"):
                _load_slam_poses(duplicate_directory / "slam_map_poses.csv")

            cloud_tamper_directory = slam_directory / "cloud_tamper"
            cloud_tamper_directory.mkdir()
            write_pose_file(cloud_tamper_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(cloud_tamper_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(cloud_tamper_directory, 2)
            cloud_path = cloud_tamper_directory / "slam_map.pcd"
            cloud_path.write_bytes(b"tampered final map cloud")
            with self.assertRaisesRegex(ValueError, "slam_map.pcd size or SHA-256"):
                _load_slam_poses(cloud_tamper_directory / "slam_map_poses.csv")

            for artifact_name in (
                "slam_map.ply", "slam_map_poses.csv", "slam_map_keyframes.csv",
                "slam_poses.csv", "slam_odom_poses.csv", "map_to_odom.csv",
            ):
                with self.subTest(tampered_artifact=artifact_name):
                    tampered_directory = slam_directory / f"tampered_{artifact_name.replace('.', '_')}"
                    tampered_directory.mkdir()
                    write_pose_file(tampered_directory / "slam_map_poses.csv", corrected_rows)
                    write_pose_file(tampered_directory / "slam_odom_poses.csv", [
                        {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                        for row in corrected_rows
                    ])
                    _write_pose_integrity_manifests(tampered_directory, 2)
                    artifact_path = tampered_directory / artifact_name
                    artifact_path.write_bytes(artifact_path.read_bytes() + b"tampered")
                    with self.assertRaisesRegex(ValueError, f"{artifact_name} size or SHA-256"):
                        _load_slam_poses(tampered_directory / "slam_map_poses.csv")

            bad_cloud_directory = slam_directory / "bad_cloud_version"
            bad_cloud_directory.mkdir()
            write_pose_file(bad_cloud_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(bad_cloud_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(bad_cloud_directory, 2)
            bad_cloud_observer_path = bad_cloud_directory / "slam_observer.json"
            bad_cloud_observer = json.loads(bad_cloud_observer_path.read_text(encoding="utf-8"))
            for entry in bad_cloud_observer["files"]:
                if entry["path"] == "slam_map.pcd":
                    entry["map_version"] = "f" * 64
            bad_cloud_observer_bytes = json.dumps(bad_cloud_observer).encode("utf-8")
            bad_cloud_observer_path.write_bytes(bad_cloud_observer_bytes)
            bad_cloud_manifest_path = bad_cloud_directory / "slam_manifest.json"
            bad_cloud_manifest = json.loads(bad_cloud_manifest_path.read_text(encoding="utf-8"))
            bad_cloud_manifest["observer"] = bad_cloud_observer
            for entry in bad_cloud_manifest["artifacts"]:
                if entry["path"] == "slam_map.pcd":
                    entry["map_version"] = "f" * 64
            bad_cloud_manifest["observer_artifact"] = {
                "path": "slam_observer.json",
                "size_bytes": len(bad_cloud_observer_bytes),
                "sha256": hashlib.sha256(bad_cloud_observer_bytes).hexdigest(),
            }
            bad_cloud_manifest_path.write_text(json.dumps(bad_cloud_manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "does not match the final map_version"):
                _load_slam_poses(bad_cloud_directory / "slam_map_poses.csv")

            for artifact_name in ("slam_odom_poses.csv", "slam_map.pcd", "slam_map.ply"):
                with self.subTest(unexpected_dense_pose_version=artifact_name):
                    version_directory = slam_directory / f"bad_dense_version_{artifact_name.replace('.', '_')}"
                    version_directory.mkdir()
                    write_pose_file(version_directory / "slam_map_poses.csv", corrected_rows)
                    write_pose_file(version_directory / "slam_odom_poses.csv", [
                        {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                        for row in corrected_rows
                    ])
                    _write_pose_integrity_manifests(version_directory, 2)
                    observer_path = version_directory / "slam_observer.json"
                    observer_record = json.loads(observer_path.read_text(encoding="utf-8"))
                    for entry in observer_record["files"]:
                        if entry["path"] == artifact_name:
                            entry["dense_pose_version"] = "d" * 64
                    observer_bytes = json.dumps(observer_record).encode("utf-8")
                    observer_path.write_bytes(observer_bytes)
                    manifest_path = version_directory / "slam_manifest.json"
                    manifest_record = json.loads(manifest_path.read_text(encoding="utf-8"))
                    manifest_record["observer"] = observer_record
                    for entry in manifest_record["artifacts"]:
                        if entry["path"] == artifact_name:
                            entry["dense_pose_version"] = "d" * 64
                    manifest_record["observer_artifact"] = {
                        "path": "slam_observer.json",
                        "size_bytes": len(observer_bytes),
                        "sha256": hashlib.sha256(observer_bytes).hexdigest(),
                    }
                    manifest_path.write_text(json.dumps(manifest_record), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "does not match the dense_pose_version"):
                        _load_slam_poses(version_directory / "slam_map_poses.csv")

            bad_odom_role_directory = slam_directory / "bad_odom_role"
            bad_odom_role_directory.mkdir()
            write_pose_file(bad_odom_role_directory / "slam_map_poses.csv", corrected_rows)
            write_pose_file(bad_odom_role_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(bad_odom_role_directory, 2)
            observer_path = bad_odom_role_directory / "slam_observer.json"
            observer_record = json.loads(observer_path.read_text(encoding="utf-8"))
            for entry in observer_record["files"]:
                if entry["path"] == "slam_odom_poses.csv":
                    entry["role"] = "raw_odom_diagnostic"
            observer_bytes = json.dumps(observer_record).encode("utf-8")
            observer_path.write_bytes(observer_bytes)
            manifest_path = bad_odom_role_directory / "slam_manifest.json"
            manifest_record = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest_record["observer"] = observer_record
            for entry in manifest_record["artifacts"]:
                if entry["path"] == "slam_odom_poses.csv":
                    entry["role"] = "raw_odom_diagnostic"
            manifest_record["observer_artifact"] = {
                "path": "slam_observer.json",
                "size_bytes": len(observer_bytes),
                "sha256": hashlib.sha256(observer_bytes).hexdigest(),
            }
            manifest_path.write_text(json.dumps(manifest_record), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "slam_odom_poses.csv has an unexpected role"):
                _load_slam_poses(bad_odom_role_directory / "slam_map_poses.csv")

            poses = _load_slam_poses(map_pose_path)
            self.assertEqual(len(poses), 2)
            with self.assertRaisesRegex(ValueError, "requires the canonical slam_map_poses.csv"):
                _load_slam_poses(slam_directory / "slam_poses.csv")

            tampered_directory = slam_directory / "tampered"
            tampered_directory.mkdir()
            tampered_map = tampered_directory / "slam_map_poses.csv"
            write_pose_file(tampered_map, corrected_rows)
            write_pose_file(tampered_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(tampered_directory, 2)
            # Keep the recorded hash unchanged while making a finite edit.
            tampered_text = tampered_map.read_text(encoding="utf-8").replace(",10.0,", ",11.0,", 1)
            tampered_map.write_text(tampered_text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA-256 does not match"):
                _load_slam_poses(tampered_map)

            tampered_raw_directory = slam_directory / "tampered_raw"
            tampered_raw_directory.mkdir()
            tampered_raw_map = tampered_raw_directory / "slam_map_poses.csv"
            tampered_raw_odom = tampered_raw_directory / "slam_odom_poses.csv"
            write_pose_file(tampered_raw_map, corrected_rows)
            write_pose_file(tampered_raw_odom, [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(tampered_raw_directory, 2)
            raw_text = tampered_raw_odom.read_text(encoding="utf-8").replace(",0.0,0.0,", ",0.25,0.0,", 1)
            tampered_raw_odom.write_text(raw_text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "slam_odom_poses.csv size or SHA-256"):
                _load_slam_poses(tampered_raw_map)

            partial_directory = slam_directory / "partial"
            partial_directory.mkdir()
            partial_path = partial_directory / "slam_map_poses.csv"
            write_pose_file(partial_path, [corrected_rows[0]])
            write_pose_file(partial_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows
            ])
            _write_pose_integrity_manifests(partial_directory, 1)
            with self.assertRaisesRegex(ValueError, "different row counts"):
                _load_slam_poses(partial_path)

            invalid_directory = slam_directory / "invalid"
            invalid_directory.mkdir()
            invalid_path = invalid_directory / "slam_map_poses.csv"
            write_pose_file(invalid_path, [{**corrected_rows[0], "frame_id": "odom"}])
            write_pose_file(invalid_directory / "slam_odom_poses.csv", [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in corrected_rows[:1]
            ])
            _write_pose_integrity_manifests(invalid_directory, 1)
            with self.assertRaisesRegex(ValueError, "Expected frame_id=map"):
                _load_slam_poses(invalid_path)

            finite_cases = (
                ("map_timestamp_nan", "map", 0, "timestamp_s", "nan"),
                ("map_position_inf", "map", 0, "x_m", "inf"),
                ("map_orientation_nan", "map", 1, "qw", "nan"),
                ("odom_timestamp_inf", "odom", 0, "timestamp_s", "inf"),
                ("odom_position_nan", "odom", 0, "y_m", "nan"),
                ("odom_orientation_inf", "odom", 1, "qx", "inf"),
            )
            for name, namespace, row_index, key, value in finite_cases:
                bad_map_rows = [dict(row) for row in corrected_rows]
                bad_odom_rows = [
                    {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                    for row in corrected_rows
                ]
                if namespace == "map":
                    bad_map_rows[row_index][key] = value
                else:
                    bad_odom_rows[row_index][key] = value
                bad_directory = slam_directory / name
                bad_directory.mkdir()
                bad_map_path = bad_directory / "slam_map_poses.csv"
                write_pose_file(bad_map_path, bad_map_rows)
                write_pose_file(bad_directory / "slam_odom_poses.csv", bad_odom_rows)
                _write_pose_integrity_manifests(bad_directory, 2)
                with self.assertRaisesRegex(ValueError, "non-finite numeric data"):
                    _load_slam_poses(bad_map_path)

            geometry = {
                "rig_lidar_t": np.zeros(3), "rig_lidar_r": np.eye(3),
                "rig_camera_t": np.zeros(3), "rig_camera_r": np.eye(3),
                "link_optical_r": np.eye(3), "fx": 1.0, "fy": 1.0, "cx": 50.0, "cy": 50.0,
            }
            frames = [
                {"frame_index": 0, "stamp_s": 0.0, "width": 100, "height": 100},
                {"frame_index": 1, "stamp_s": 0.03, "width": 100, "height": 100},
            ]
            annotations = [{
                "frame_index": 0,
                "detections": [{"track_id": 2, "raw_track_id": 2, "bbox_xyxy": [0, 0, 99, 99]}],
            }]
            points = np.asarray([[-0.01, 0.0, 2.0], [0.0, 0.0, 2.0], [0.01, 0.0, 2.0]])
            with patch("simulator.perception.rgb_tracking._load_sensor_geometry", return_value=geometry), \
                 patch("simulator.perception.rgb_tracking._read_lidar_scans", return_value=[(0.01, points, np.asarray([20, 21, 22]))]):
                start_estimates, map_estimates, localization, _ = _augment_with_lidar_estimates(
                    Path("capture"), slam_directory, frames, annotations
                )
            self.assertTrue(localization["map_to_odom_correction_applied"])
            self.assertEqual(localization["source_pose_artifact"], "slam_map_poses.csv")
            self.assertEqual(localization["pose_provenance"]["map_version"], "e" * 64)
            self.assertEqual(localization["pose_provenance"]["dense_pose_version"], "d" * 64)
            self.assertEqual(localization["pose_provenance"]["slam_map_pose_sha256"], hashlib.sha256(map_pose_path.read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_map_keyframes_sha256"], hashlib.sha256((slam_directory / "slam_map_keyframes.csv").read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_map_keyframes_schema_version"], 2)
            self.assertEqual(localization["pose_provenance"]["slam_map_keyframes_map_version"], "e" * 64)
            self.assertEqual(localization["pose_provenance"]["slam_manifest_sha256"], hashlib.sha256((slam_directory / "slam_manifest.json").read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_observer_sha256"], hashlib.sha256((slam_directory / "slam_observer.json").read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_odom_pose_sha256"], hashlib.sha256((slam_directory / "slam_odom_poses.csv").read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_cloud_sha256"], hashlib.sha256((slam_directory / "slam_map.pcd").read_bytes()).hexdigest())
            self.assertEqual(localization["pose_provenance"]["slam_cloud_ply_sha256"], hashlib.sha256((slam_directory / "slam_map.ply").read_bytes()).hexdigest())
            np.testing.assert_allclose(map_estimates[2], [10.2, 2.0, 2.0], atol=1e-9)
            np.testing.assert_allclose(start_estimates[2], [0.0, -0.2, 2.0], atol=1e-9)
            detection = annotations[0]["detections"][0]
            np.testing.assert_allclose(detection["estimated_center_map_m"], [10.2, 2.0, 2.0], atol=1e-3)
            np.testing.assert_allclose(detection["estimated_center_start_relative_m"], [0.0, -0.2, 2.0], atol=1e-3)

    def test_corrected_pose_manifest_enforces_full_artifact_and_timestamp_contract(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            base_map_rows = [
                {"timestamp_s": 1.0, "x_m": 10.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "frame_id": "map"},
                {"timestamp_s": 2.0, "x_m": 11.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "frame_id": "map"},
            ]
            base_odom_rows = [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in base_map_rows
            ]

            valid = _write_pose_contract_case(root, "valid", base_map_rows, base_odom_rows)
            poses, provenance = _load_slam_poses(valid / "slam_map_poses.csv", include_provenance=True)
            self.assertEqual(len(poses), 2)
            keyframe_path = valid / "slam_map_keyframes.csv"
            self.assertEqual(provenance["slam_map_keyframes_sha256"], hashlib.sha256(keyframe_path.read_bytes()).hexdigest())
            self.assertEqual(provenance["slam_map_keyframes_size_bytes"], keyframe_path.stat().st_size)
            self.assertEqual(provenance["slam_map_keyframes_schema_version"], 2)
            self.assertEqual(provenance["slam_map_keyframes_map_version"], "e" * 64)

            for index, alias in enumerate(("SLAM_MAP_POSES.CSV", "./slam_map_poses.csv", "nested/../slam_map_poses.csv")):
                with self.subTest(noncanonical_artifact_alias=alias):
                    directory = _write_pose_contract_case(root, f"alias_{index}", base_map_rows, base_odom_rows)
                    observer = json.loads((directory / "slam_observer.json").read_text(encoding="utf-8"))
                    duplicate = dict(next(entry for entry in observer["files"] if entry["path"] == "slam_map_poses.csv"))
                    duplicate["path"] = alias
                    observer["files"].append(duplicate)
                    _rebind_observer_artifact(directory, observer)
                    with self.assertRaisesRegex(ValueError, "artifact path"):
                        _load_slam_poses(directory / "slam_map_poses.csv")

            for index, extra_entry in enumerate(({}, {"path": "unlisted.txt"})):
                with self.subTest(malformed_extra_artifact=extra_entry):
                    directory = _write_pose_contract_case(root, f"extra_{index}", base_map_rows, base_odom_rows)
                    observer = json.loads((directory / "slam_observer.json").read_text(encoding="utf-8"))
                    observer["files"].append(extra_entry)
                    _rebind_observer_artifact(directory, observer)
                    with self.assertRaisesRegex(ValueError, "malformed artifact entry|noncanonical or unknown artifact path"):
                        _load_slam_poses(directory / "slam_map_poses.csv")

            required_roles = {
                "slam_map_poses.csv": "dense_corrected_trajectory",
                "slam_map_keyframes.csv": "optimized_graph_keyframes",
                "slam_poses.csv": "legacy_map_trajectory",
                "slam_odom_poses.csv": "raw_odometry_diagnostic",
                "map_to_odom.csv": "incremental_tf_diagnostic",
                "slam_map.pcd": "final_optimized_cloud",
                "slam_map.ply": "final_optimized_cloud",
            }
            for artifact_name in required_roles:
                with self.subTest(wrong_artifact_role=artifact_name):
                    directory = _write_pose_contract_case(root, f"wrong_role_{artifact_name.replace('.', '_')}", base_map_rows, base_odom_rows)
                    observer = json.loads((directory / "slam_observer.json").read_text(encoding="utf-8"))
                    for entry in observer["files"]:
                        if entry["path"] == artifact_name:
                            entry["role"] = "wrong_role"
                    _rebind_observer_artifact(directory, observer)
                    with self.assertRaisesRegex(ValueError, f"{artifact_name} has an unexpected role"):
                        _load_slam_poses(directory / "slam_map_poses.csv")

            for metadata_field, bad_value, message in (
                ("schema_version", 1, "unexpected schema_version"),
                ("correction_policy", "incorrect interpolation", "unexpected correction_policy"),
            ):
                with self.subTest(invalid_artifact_metadata=metadata_field):
                    directory = _write_pose_contract_case(root, f"bad_{metadata_field}", base_map_rows, base_odom_rows)
                    observer = json.loads((directory / "slam_observer.json").read_text(encoding="utf-8"))
                    keyframe = next(entry for entry in observer["files"] if entry["path"] == "slam_map_keyframes.csv")
                    keyframe[metadata_field] = bad_value
                    _rebind_observer_artifact(directory, observer)
                    with self.assertRaisesRegex(ValueError, message):
                        _load_slam_poses(directory / "slam_map_poses.csv")

            keyframe_tamper = _write_pose_contract_case(root, "keyframe_tamper", base_map_rows, base_odom_rows)
            keyframe_path = keyframe_tamper / "slam_map_keyframes.csv"
            keyframe_path.write_bytes(keyframe_path.read_bytes() + b"post-manifest change")
            with self.assertRaisesRegex(ValueError, "slam_map_keyframes.csv size or SHA-256"):
                _load_slam_poses(keyframe_tamper / "slam_map_poses.csv")

            for index, invalid_count in enumerate((True, False, "2", 2.0, 0, -1, None, "__missing__")):
                with self.subTest(invalid_odom_sample_count=invalid_count):
                    directory = _write_pose_contract_case(root, f"bad_odom_count_{index}", base_map_rows, base_odom_rows)
                    observer = json.loads((directory / "slam_observer.json").read_text(encoding="utf-8"))
                    if invalid_count == "__missing__":
                        del observer["odom_sample_count"]
                    else:
                        observer["odom_sample_count"] = invalid_count
                    _rebind_observer_artifact(directory, observer)
                    with self.assertRaisesRegex(ValueError, "odom_sample_count must be a positive integer"):
                        _load_slam_poses(directory / "slam_map_poses.csv")

            for index, invalid_top_count in enumerate((3, None)):
                top_count = _write_pose_contract_case(root, f"top_count_mismatch_{index}", base_map_rows, base_odom_rows)
                observer = json.loads((top_count / "slam_observer.json").read_text(encoding="utf-8"))
                _rebind_observer_artifact(top_count, observer, top_level={"odom_sample_count": invalid_top_count})
                with self.assertRaisesRegex(ValueError, "Top-level odom_sample_count disagrees"):
                    _load_slam_poses(top_count / "slam_map_poses.csv")

            scalar_count_mismatch = _write_pose_contract_case(
                root, "scalar_count_mismatch", base_map_rows, base_odom_rows,
                map_pose_count=2, odom_sample_count=1,
            )
            with self.assertRaisesRegex(ValueError, "map_pose_sample_count and odom_sample_count disagree"):
                _load_slam_poses(scalar_count_mismatch / "slam_map_poses.csv")

            row_count_mismatch = _write_pose_contract_case(
                root, "row_count_mismatch", base_map_rows, base_odom_rows[:1],
                map_pose_count=2, odom_sample_count=2,
            )
            with self.assertRaisesRegex(ValueError, "different row counts"):
                _load_slam_poses(row_count_mismatch / "slam_map_poses.csv")

            false_row_count = _write_pose_contract_case(
                root, "false_row_count", base_map_rows[:1], base_odom_rows[:1],
                map_pose_count=2, odom_sample_count=2,
            )
            with self.assertRaisesRegex(ValueError, "row count does not match"):
                _load_slam_poses(false_row_count / "slam_map_poses.csv")

            for stream_name in ("map", "odom"):
                for timestamp_pair in ((1.0, 1.0), (2.0, 1.0)):
                    with self.subTest(nonincreasing_timestamp_stream=stream_name, timestamps=timestamp_pair):
                        map_rows = [dict(row) for row in base_map_rows]
                        odom_rows = [dict(row) for row in base_odom_rows]
                        target_rows = map_rows if stream_name == "map" else odom_rows
                        target_rows[0]["timestamp_s"], target_rows[1]["timestamp_s"] = timestamp_pair
                        directory = _write_pose_contract_case(
                            root, f"bad_time_{stream_name}_{timestamp_pair[0]}_{timestamp_pair[1]}",
                            map_rows, odom_rows,
                        )
                        with self.assertRaisesRegex(ValueError, "timestamps must be strictly increasing"):
                            _load_slam_poses(directory / "slam_map_poses.csv")

            shifted_raw_rows = [dict(row) for row in base_odom_rows]
            shifted_raw_rows[1]["timestamp_s"] = 2.001
            shifted = _write_pose_contract_case(root, "shifted_raw_timestamps", base_map_rows, shifted_raw_rows)
            with self.assertRaisesRegex(ValueError, "exact raw odom timestamps"):
                _load_slam_poses(shifted / "slam_map_poses.csv")

    def test_pose_loader_accepts_runtime_sorted_streams_after_out_of_order_callbacks(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            # Runtime rev6 sorts raw odometry and corrected map rows by their
            # timestamps before emitting the CSVs; callback arrival order is
            # allowed to differ from emitted order.
            map_callbacks = [
                {"timestamp_s": 2.0, "x_m": 11.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "frame_id": "map"},
                {"timestamp_s": 1.0, "x_m": 10.0, "y_m": 2.0, "z_m": 0.0,
                 "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "frame_id": "map"},
            ]
            odom_callbacks = [
                {**row, "x_m": 0.0, "y_m": 0.0, "frame_id": "odom"}
                for row in map_callbacks
            ]
            map_csv_rows = sorted(map_callbacks, key=lambda row: float(row["timestamp_s"]))
            odom_csv_rows = sorted(odom_callbacks, key=lambda row: float(row["timestamp_s"]))
            directory = _write_pose_contract_case(root, "runtime_sorted", map_csv_rows, odom_csv_rows)
            poses = _load_slam_poses(directory / "slam_map_poses.csv")
            self.assertEqual([pose.timestamp_s for pose in poses], [1.0, 2.0])

    def test_run_rgb_tracking_marks_missing_corrected_map_poses_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            capture = root / "capture"
            slam = root / "slam"
            output = root / "perception"
            capture.mkdir()
            slam.mkdir()

            class FakeVideo:
                def __enter__(self):
                    return self

                def __exit__(self, *_args):
                    return False

                def decode(self, video=0):
                    return iter([SimpleNamespace(to_ndarray=lambda **_kwargs: np.zeros((8, 8, 3), dtype=np.uint8))])

            fake_av = SimpleNamespace(open=lambda _path: FakeVideo())
            frame_index = [{"frame_index": 0, "stamp_s": 0.0, "width": 8, "height": 8}]
            with patch.dict(sys.modules, {"av": fake_av}), \
                 patch("simulator.perception.rgb_tracking._read_timestamp_index", return_value=frame_index), \
                 patch("simulator.perception.rgb_tracking.detect_product_blobs", return_value=[]):
                summary = run_rgb_tracking(capture, slam, output)
            self.assertEqual(summary["status"], "incomplete")
            self.assertEqual(summary["localization"]["status"], "unavailable")
            self.assertFalse(summary["slam_consumed_for_estimation"])
            self.assertFalse(summary["lidar_consumed_for_estimation"])
            written = json.loads((output / "perception_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(written["status"], "incomplete")
            self.assertFalse(written["slam_consumed_for_estimation"])
            self.assertFalse(written["lidar_consumed_for_estimation"])

            corrupt_slam = root / "corrupt_slam"
            corrupt_slam.mkdir()
            pose_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"]
            map_row = {"timestamp_s": 0.0, "x_m": "nan", "y_m": 0.0, "z_m": 0.0,
                       "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0, "frame_id": "map"}
            odom_row = {**map_row, "x_m": 0.0, "frame_id": "odom"}
            for filename, row in (("slam_map_poses.csv", map_row), ("slam_odom_poses.csv", odom_row)):
                with (corrupt_slam / filename).open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=pose_fields)
                    writer.writeheader()
                    writer.writerow(row)
            _write_pose_integrity_manifests(corrupt_slam, 1)
            corrupt_output = root / "corrupt_perception"
            with patch.dict(sys.modules, {"av": fake_av}), \
                 patch("simulator.perception.rgb_tracking._read_timestamp_index", return_value=frame_index), \
                 patch("simulator.perception.rgb_tracking.detect_product_blobs", return_value=[]):
                with self.assertRaisesRegex(ValueError, "non-finite numeric data"):
                    run_rgb_tracking(capture, corrupt_slam, corrupt_output)
            self.assertFalse((corrupt_output / "perception_manifest.json").exists())

    def test_expired_track_remains_in_localized_inventory_and_annotations(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            capture, slam, output = root / "capture", root / "slam", root / "perception"
            capture.mkdir()
            slam.mkdir()
            (slam / "slam_manifest.json").write_text("fixture manifest", encoding="utf-8")
            pcd_bytes, ply_bytes = b"fixture pcd", b"fixture ply"
            (slam / "slam_map.pcd").write_bytes(pcd_bytes)
            (slam / "slam_map.ply").write_bytes(ply_bytes)
            manifest_bytes = (slam / "slam_manifest.json").read_bytes()

            class FakeVideo:
                def __enter__(self):
                    return self

                def __exit__(self, *_args):
                    return False

                def decode(self, video=0):
                    image = np.zeros((8, 8, 3), dtype=np.uint8)
                    return iter([
                        SimpleNamespace(to_ndarray=lambda **_kwargs: image),
                        SimpleNamespace(to_ndarray=lambda **_kwargs: image),
                    ])

            detection = Detection((1, 1, 5, 5), (3.0, 3.0), 16, 20.0, 200.0, 0.9, (8, 8))
            frame_index = [
                {"frame_index": 0, "stamp_s": 0.0, "width": 8, "height": 8},
                {"frame_index": 100, "stamp_s": 100.0, "width": 8, "height": 8},
            ]
            localization = {
                "status": "complete",
                "start_pose_world_m_exact": [10.0, 0.0, 0.0],
                "start_pose_orientation_xyzw_exact": [0.0, 0.0, 0.0, 1.0],
                "lidar_input_stream_read": True,
                "pose_provenance": {
                    "map_version": "e" * 64,
                    "graph_pose_version": "a" * 64,
                    "dense_pose_version": "d" * 64,
                    "slam_manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
                    "slam_manifest_size_bytes": len(manifest_bytes),
                    "slam_observer_sha256": "2" * 64,
                    "slam_observer_size_bytes": 128,
                    "slam_map_pose_sha256": "1" * 64,
                    "slam_map_pose_size_bytes": 256,
                    "slam_map_keyframes_sha256": "4" * 64,
                    "slam_map_keyframes_size_bytes": 512,
                    "slam_map_keyframes_schema_version": 2,
                    "slam_map_keyframes_map_version": "e" * 64,
                    "slam_odom_pose_sha256": "3" * 64,
                    "slam_odom_pose_size_bytes": 256,
                    "slam_cloud_sha256": hashlib.sha256(pcd_bytes).hexdigest(),
                    "slam_cloud_size_bytes": len(pcd_bytes),
                    "slam_cloud_ply_sha256": hashlib.sha256(ply_bytes).hexdigest(),
                    "slam_cloud_ply_size_bytes": len(ply_bytes),
                    "map_pose_sample_count": 2,
                },
            }
            lidar_support = {
                1: {
                    "scan_timestamps_s": [0.0],
                    "unique_supporting_scan_count": 1,
                    "unique_retained_depth_return_count": 3,
                    "3d_update_event_count": 1,
                    "observation_duration_s": 0.0,
                    "source_point_support": [{"scan_timestamp_s": 0.0, "source_point_indices": [5, 6, 7]}],
                }
            }
            with patch.dict(sys.modules, {"av": SimpleNamespace(open=lambda _path: FakeVideo())}), \
                 patch("simulator.perception.rgb_tracking._read_timestamp_index", return_value=frame_index), \
                 patch("simulator.perception.rgb_tracking.detect_product_blobs", side_effect=[[detection], []]), \
                 patch("simulator.perception.rgb_tracking._augment_with_lidar_estimates", return_value=(
                     {1: np.asarray([0.0, 0.0, 2.0])},
                     {1: np.asarray([10.0, 0.0, 2.0])},
                     localization,
                     lidar_support,
                 )):
                summary = run_rgb_tracking(capture, slam, output)

            self.assertEqual(summary["status"], "complete")
            self.assertEqual(summary["detection_count"], 1)
            self.assertEqual(summary["raw_rgb_track_count"], 1)
            self.assertEqual(summary["track_count"], 1)
            self.assertEqual(summary["slam_pose_provenance"]["map_version"], "e" * 64)
            self.assertEqual(summary["slam_cloud_sha256"], hashlib.sha256(pcd_bytes).hexdigest())
            self.assertEqual(summary["slam_cloud_size_bytes"], len(pcd_bytes))
            self.assertEqual(summary["slam_cloud_ply_sha256"], hashlib.sha256(ply_bytes).hexdigest())
            self.assertEqual(summary["slam_cloud_ply_size_bytes"], len(ply_bytes))
            self.assertEqual(summary["map_version"], "e" * 64)
            self.assertEqual(summary["slam_manifest_sha256"], hashlib.sha256(manifest_bytes).hexdigest())
            self.assertEqual(summary["slam_map_keyframes_sha256"], "4" * 64)
            self.assertEqual(summary["slam_map_keyframes_size_bytes"], 512)
            self.assertEqual(summary["slam_map_keyframes_schema_version"], 2)
            self.assertEqual(summary["slam_map_keyframes_map_version"], "e" * 64)
            inventory = json.loads((output / "estimated_inventory.json").read_text(encoding="utf-8"))
            self.assertEqual(len(inventory), 1)
            self.assertEqual(inventory[0]["detection_count"], 1)
            self.assertEqual(inventory[0]["raw_track_ids"], [1])
            self.assertEqual(inventory[0]["unique_supporting_scan_count"], 1)
            self.assertEqual(inventory[0]["unique_retained_depth_return_count"], 3)
            self.assertEqual(inventory[0]["map_version"], "e" * 64)
            self.assertEqual(inventory[0]["dense_pose_version"], "d" * 64)
            self.assertEqual(inventory[0]["slam_manifest_sha256"], hashlib.sha256(manifest_bytes).hexdigest())
            self.assertEqual(inventory[0]["slam_observer_sha256"], "2" * 64)
            self.assertEqual(inventory[0]["slam_map_pose_sha256"], "1" * 64)
            self.assertEqual(inventory[0]["slam_map_keyframes_sha256"], "4" * 64)
            self.assertEqual(inventory[0]["slam_map_keyframes_schema_version"], 2)
            self.assertEqual(inventory[0]["slam_map_keyframes_map_version"], "e" * 64)
            self.assertEqual(inventory[0]["slam_odom_pose_sha256"], "3" * 64)
            self.assertEqual(inventory[0]["slam_cloud_ply_sha256"], hashlib.sha256(ply_bytes).hexdigest())
            self.assertEqual(inventory[0]["map_pose_sample_count"], 2)
            with (output / "estimated_inventory.csv").open(newline="", encoding="utf-8") as handle:
                inventory_csv = next(csv.DictReader(handle))
            self.assertEqual(inventory_csv["map_version"], "e" * 64)
            self.assertEqual(inventory_csv["slam_manifest_sha256"], hashlib.sha256(manifest_bytes).hexdigest())
            self.assertEqual(inventory_csv["slam_observer_sha256"], "2" * 64)
            self.assertEqual(inventory_csv["slam_map_pose_sha256"], "1" * 64)
            self.assertEqual(inventory_csv["slam_map_keyframes_sha256"], "4" * 64)
            self.assertEqual(inventory_csv["slam_map_keyframes_schema_version"], "2")
            self.assertEqual(inventory_csv["slam_map_keyframes_map_version"], "e" * 64)
            self.assertEqual(inventory_csv["slam_odom_pose_sha256"], "3" * 64)
            self.assertEqual(inventory_csv["slam_cloud_ply_sha256"], hashlib.sha256(ply_bytes).hexdigest())
            self.assertEqual(inventory[0]["supporting_lidar_source_points"], [
                {"scan_timestamp_s": 0.0, "source_point_indices": [5, 6, 7]},
            ])
            annotations = [json.loads(line) for line in (output / "frame_annotations.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(annotations[0]["detections"][0]["raw_track_id"], 1)
            self.assertEqual(annotations[0]["detections"][0]["persistent_track_id"], 1)
            self.assertEqual(annotations[0]["detections"][0]["pose_provenance"]["map_version"], "e" * 64)
            self.assertEqual(annotations[0]["detections"][0]["pose_provenance"]["slam_map_keyframes_sha256"], "4" * 64)
            self.assertEqual(annotations[1]["detections"], [])

    def test_lidar_sampling_covers_ordered_rings_and_is_deterministic(self):
        angles = np.linspace(-np.pi, np.pi, 256, endpoint=False)
        rings = []
        for radius in (1.0, 2.0, 3.0, 4.0):
            rings.append(np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles))))
        ordered_ring_points = np.vstack(rings)
        selected = _select_lidar_point_indices(ordered_ring_points)
        selected_again = _select_lidar_point_indices(ordered_ring_points)
        self.assertTrue(np.array_equal(selected, selected_again))
        self.assertEqual(len(selected), 128)
        selected_points = ordered_ring_points[selected]
        self.assertEqual(set(np.round(np.linalg.norm(selected_points[:, :2], axis=1), 6)), {1.0, 2.0, 3.0, 4.0})
        selected_angles = np.mod(np.arctan2(selected_points[:, 1], selected_points[:, 0]), 2.0 * np.pi)
        self.assertGreaterEqual(len(np.unique(np.floor(selected_angles * 32 / (2.0 * np.pi)).astype(int))), 30)

        permutation = np.random.default_rng(17).permutation(len(ordered_ring_points))
        permuted = ordered_ring_points[permutation]
        permuted_selection = _select_lidar_point_indices(permuted)
        canonical_selected = sorted(tuple(np.round(point, 8)) for point in selected_points)
        canonical_permuted = sorted(tuple(np.round(point, 8)) for point in permuted[permuted_selection])
        self.assertEqual(canonical_selected, canonical_permuted)

    def test_detector_area_limits_scale_with_image_resolution(self):
        for width, height in ((640, 360), (1280, 720), (1920, 1080)):
            image = np.zeros((height, width, 3), dtype=np.uint8)
            x0, y0 = int(width * 0.70), int(height * 0.68)
            x1, y1 = int(width * 0.74), int(height * 0.75)
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 220), -1)
            diagnostics = {}
            detections = detect_product_blobs(image, diagnostics=diagnostics)
            self.assertEqual(len(detections), 1, (width, height))
            self.assertEqual(diagnostics["budget_rejected_count"], 0)
            self.assertEqual(diagnostics["returned_proposal_count"], 1)

    def test_large_hero_blob_survives_area_filter_and_bounded_budget(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.rectangle(image, (200, 200), (549, 399), (0, 0, 220), -1)
        diagnostics = {}
        detections = detect_product_blobs(image, diagnostics=diagnostics)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].area_px, 350 * 200)
        self.assertEqual(diagnostics["eligible_proposal_count"], 1)
        self.assertLessEqual(diagnostics["returned_proposal_count"], diagnostics["budget_limit"])

    def test_component_area_and_extent_limits_follow_resolution_scale(self):
        small = _resolution_scaled_component_limits(640, 360)
        reference = _resolution_scaled_component_limits(1280, 720)
        large = _resolution_scaled_component_limits(1920, 1080)
        for actual, expected in (
            (small, (17.5, 32256.0, 2.5)),
            (reference, (70.0, 129024.0, 5.0)),
            (large, (157.5, 290304.0, 7.5)),
        ):
            np.testing.assert_allclose(actual, expected)

    def test_co_visible_neighboring_instances_are_not_consolidated(self):
        annotations = [{"detections": [{"raw_track_id": 4}, {"raw_track_id": 9}]}]
        visible = _co_visible_raw_track_pairs(annotations)
        mapping, centers, members = _consolidate_track_estimates(
            {4: np.asarray([1.0, 0.0, 0.0]), 9: np.asarray([1.1, 0.0, 0.0])},
            merge_radius_m=0.21,
            co_visible_pairs=visible,
        )
        self.assertEqual(len(centers), 2)
        self.assertNotEqual(mapping[4], mapping[9])
        self.assertEqual(sorted(members[mapping[4]]), [4])

    def test_map_support_reconnects_shifted_raw_id_but_keeps_neighbors_distinct(self):
        co_visible = _co_visible_raw_track_pairs([
            {"detections": [{"raw_track_id": 1}, {"raw_track_id": 2}]},
            {"detections": [{"raw_track_id": 2}, {"raw_track_id": 3}]},
        ])
        mapping, centers, members = _consolidate_track_estimates(
            {
                1: np.asarray([0.0, 0.0, 2.0]),
                2: np.asarray([0.10, 0.0, 2.0]),
                3: np.asarray([0.03, 0.0, 2.0]),
            },
            merge_radius_m=0.21,
            co_visible_pairs=co_visible,
        )
        # Raw ID 3 is a fresh image-space ID after a view shift, but its
        # independent map support agrees with the earlier track 1.  Nearby
        # track 2 was visible alongside each, so it remains a distinct object.
        self.assertEqual(mapping[1], mapping[3])
        self.assertNotEqual(mapping[2], mapping[1])
        self.assertEqual(members[mapping[1]], [1, 3])
        self.assertEqual(members[mapping[2]], [2])
        self.assertEqual(len(centers), 2)

        ambiguous = {"raw_track_id": 4}
        _set_unassigned_persistent_identity(ambiguous, 4)
        self.assertIsNone(ambiguous["persistent_track_id"])

    def test_canonicalized_output_keeps_raw_and_canonical_namespaces(self):
        detection = {"track_id": 17, "raw_track_id": 17, "persistent_track_id": None, "canonical_track_id": None, "id_namespace": "raw_rgb"}
        self.assertIsNone(detection["persistent_track_id"])
        _set_canonical_track_identity(detection, 17, 3)
        self.assertEqual(detection["raw_track_id"], 17)
        self.assertEqual(detection["canonical_track_id"], 3)
        self.assertEqual(detection["persistent_track_id"], 3)
        self.assertEqual(detection["track_id"], 3)
        self.assertEqual(detection["id_namespace"], "canonical_localized")
        raw_only = {"track_id": 3, "raw_track_id": 3, "persistent_track_id": None, "canonical_track_id": None, "id_namespace": "raw_rgb"}
        _set_unassigned_persistent_identity(raw_only, 3)
        self.assertIsNone(raw_only["persistent_track_id"])
        self.assertIsNone(raw_only["canonical_track_id"])
        self.assertIsNone(raw_only["track_id"])
        self.assertEqual(raw_only["raw_track_id"], detection["persistent_track_id"])

    def test_detection_budget_preserves_spatial_coverage_and_is_deterministic(self):
        detections = [
            Detection((i, 0, i + 1, 1), (float(i), 5.0), 100, 0.0, 200.0, 0.5)
            for i in range(200)
        ]
        detections += [
            Detection((800, 700, 801, 701), (800.0, 700.0), 100, 0.0, 200.0, 0.8)
        ]
        selected = _budget_detections_spatially(detections, 1000, 800, 160)
        selected_again = _budget_detections_spatially(detections, 1000, 800, 160)
        self.assertEqual(selected, selected_again)
        self.assertIn(detections[-1], selected)
        self.assertEqual(len(selected), 160)

    def test_consolidation_spatial_index_tracks_a_drifting_centroid(self):
        estimates = {1: np.asarray([0.0, 0.0, 0.0])}
        center_x = 0.0
        count = 1
        for raw_id in range(2, 400):
            point_x = center_x + 0.20
            estimates[raw_id] = np.asarray([point_x, 0.0, 0.0])
            center_x = (center_x * count + point_x) / (count + 1)
            count += 1
        mapping, centers, members = _consolidate_track_estimates(estimates, merge_radius_m=0.21)
        self.assertEqual(len(centers), 1)
        self.assertEqual(len(members[1]), len(estimates))
        self.assertEqual(set(mapping.values()), {1})

    def test_spatial_grid_query_matches_brute_force_neighbors(self):
        radius = 0.21
        centers = [np.asarray([x, y, z]) for x, y, z in (
            (-0.2, 0.0, 0.0), (0.0, 0.0, 0.0), (0.18, 0.03, 0.0),
            (0.21, 0.21, 0.0), (0.7, -0.1, 0.3), (1.2, 1.2, 1.2),
        )]
        grid = {}
        for cluster_index, point in enumerate(centers):
            cell = tuple(np.floor(point / radius).astype(int))
            grid.setdefault(cell, []).append(cluster_index)
        queries = [np.asarray([0.0, 0.0, 0.0]), np.asarray([0.21, 0.21, 0.0]), np.asarray([1.0, 1.0, 1.0])]
        for query in queries:
            indexed = _spatial_grid_candidate_indices(query, radius, grid)
            indexed_neighbors = {i for i in indexed if np.linalg.norm(centers[i] - query) <= radius}
            brute_force_neighbors = {i for i, center in enumerate(centers) if np.linalg.norm(center - query) <= radius}
            self.assertEqual(indexed_neighbors, brute_force_neighbors)

    def test_one_lidar_scan_is_not_counted_as_many_rgb_backfills(self):
        poses = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(0.1, (0.0, 0.0, 0.0))]
        geometry = {
            "rig_lidar_t": np.zeros(3), "rig_lidar_r": np.eye(3),
            "rig_camera_t": np.zeros(3), "rig_camera_r": np.eye(3),
            "link_optical_r": np.eye(3), "fx": 1.0, "fy": 1.0, "cx": 50.0, "cy": 50.0,
        }
        frames = [{"frame_index": i, "stamp_s": i / 30.0, "width": 100, "height": 100} for i in range(20)]
        annotations = [{"frame_index": i, "detections": [{"track_id": 2, "raw_track_id": 2, "bbox_xyxy": [0, 0, 99, 99]}]} for i in range(20)]
        points = np.asarray([[0.0, 0.0, 2.0], [0.01, 0.0, 2.0], [0.02, 0.0, 2.0], [0.03, 0.0, 2.0], [0.50, 0.0, 2.0]])
        with patch("simulator.perception.rgb_tracking._load_slam_poses", return_value=poses), \
             patch("simulator.perception.rgb_tracking._load_sensor_geometry", return_value=geometry), \
             patch("simulator.perception.rgb_tracking._read_lidar_scans", return_value=[(0.01, points, np.arange(100, 105)), (0.01, points, np.arange(100, 105))]):
            start_estimates, map_estimates, _, scan_counts = _augment_with_lidar_estimates(
                __import__("pathlib").Path("capture"), __import__("pathlib").Path("slam"), frames, annotations
            )
        backfilled = sum(
            row["detections"][0].get("coordinate_source") == "lidar_projected_with_slam_pose"
            for row in annotations
        )
        self.assertEqual(backfilled, 20)
        self.assertEqual(scan_counts[2]["unique_supporting_scan_count"], 1)
        self.assertEqual(scan_counts[2]["unique_retained_depth_return_count"], 5)
        self.assertEqual(scan_counts[2]["3d_update_event_count"], 1)
        self.assertEqual(scan_counts[2]["observation_duration_s"], 0.0)
        np.testing.assert_allclose(start_estimates[2], map_estimates[2])
        np.testing.assert_allclose(start_estimates[2], [0.02, 0.0, 2.0])
        self.assertEqual(annotations[0]["detections"][0]["supporting_lidar_scan_timestamps_s"], [0.01])
        self.assertEqual(annotations[0]["detections"][0]["lidar_depth_support"], [{
            "scan_timestamp_s": 0.01,
            "point_timestamp_s": 0.01,
            "point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
            "source_point_indices": [100, 101, 102, 103, 104],
        }])
        self.assertEqual(scan_counts[2]["source_point_support"], [{
            "scan_timestamp_s": 0.01,
            "source_point_indices": [100, 101, 102, 103, 104],
            "point_timestamp_reference": "PointCloud2 header timestamp; per-return timestamps unavailable",
        }])
        self.assertNotIn("supporting_lidar_scan_timestamps_s", annotations[1]["detections"][0])
        self.assertFalse(annotations[0]["detections"][0]["estimate_is_track_level_backfill"])
        self.assertTrue(annotations[1]["detections"][0]["estimate_is_track_level_backfill"])
        self.assertEqual(annotations[1]["detections"][0]["track_supporting_lidar_scan_timestamps_s"], [0.01])

    def test_world_point_projection_uses_rgb_measurement_time_pose(self):
        geometry = {
            "rig_camera_t": np.asarray([0.2, 0.0, 0.0]),
            "rig_camera_r": np.eye(3), "link_optical_r": np.eye(3),
        }
        point = np.asarray([[2.0, 0.0, 4.0]])
        at_origin = _world_points_to_camera(point, PoseSample(0.0, (0.0, 0.0, 0.0)), geometry)
        after_translation = _world_points_to_camera(point, PoseSample(1.0, (1.0, 0.0, 0.0)), geometry)
        self.assertAlmostEqual(at_origin[0, 0], 1.8)
        self.assertAlmostEqual(after_translation[0, 0], 0.8)
        quarter_turn = PoseSample(2.0, (0.0, 0.0, 0.0), (0.0, 0.0, 2 ** -0.5, 2 ** -0.5))
        rotated = _world_points_to_camera(np.asarray([[1.0, 0.0, 4.0]]), quarter_turn, geometry)
        self.assertAlmostEqual(rotated[0, 0], -0.2)
        self.assertAlmostEqual(rotated[0, 1], -1.0)

    def test_world_point_projection_uses_articulated_head_at_image_time(self):
        half_turn = 2 ** -0.5
        trajectory = CameraHeadTransformTrajectory(
            np.asarray([0.0, 1.0]),
            np.zeros((2, 3)),
            np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, half_turn, half_turn]]),
            {"mode": "dynamic_bound_artifact"},
        )
        geometry = {
            "rig_camera_t": np.zeros(3),
            "rig_camera_r": np.eye(3),
            "link_optical_t": np.zeros(3),
            "link_optical_r": np.eye(3),
            "camera_head_trajectory": trajectory,
        }
        point = np.asarray([[1.0, 0.0, 4.0]])
        base_pose = PoseSample(0.0, (0.0, 0.0, 0.0))
        at_start = _world_points_to_camera(point, base_pose, geometry, image_timestamp_s=0.0)
        after_head_turn = _world_points_to_camera(point, base_pose, geometry, image_timestamp_s=1.0)
        np.testing.assert_allclose(at_start, [[1.0, 0.0, 4.0]], atol=1e-12)
        np.testing.assert_allclose(after_head_turn, [[0.0, -1.0, 4.0]], atol=1e-12)

    def test_sensor_geometry_preserves_legacy_direct_static_optical_edge(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sensor_transforms.json"
            path.write_text(json.dumps({
                "frames": {
                    "sensor_rig": "sensor_rig", "camera_optical": "camera_optical_frame",
                    "lidar_link": "lidar_link",
                },
                "transforms": [
                    {
                        "parent": "sensor_rig", "child": "camera_optical_frame",
                        "translation_m": [0.2, 0.0, 1.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                    {
                        "parent": "sensor_rig", "child": "lidar_link",
                        "translation_m": [0.0, 0.0, 0.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                ],
                "intrinsics": {"fx_px": 100.0, "fy_px": 100.0, "cx_px": 50.0, "cy_px": 50.0},
            }), encoding="utf-8")
            geometry = _load_sensor_geometry(path)
        np.testing.assert_allclose(geometry["rig_camera_t"], [0.2, 0.0, 1.0])
        np.testing.assert_allclose(geometry["rig_camera_r"], np.eye(3))
        np.testing.assert_allclose(geometry["link_optical_r"], np.eye(3))
        self.assertIsNone(geometry["camera_head_trajectory"])

    def test_lidar_association_skips_scan_when_rgb_pose_interpolation_is_invalid(self):
        poses = [PoseSample(0.0, (0.0, 0.0, 0.0)), PoseSample(1.0, (1.0, 0.0, 0.0))]
        geometry = {
            "rig_lidar_t": np.zeros(3), "rig_lidar_r": np.eye(3),
            "rig_camera_t": np.zeros(3), "rig_camera_r": np.eye(3),
            "link_optical_r": np.eye(3), "fx": 1.0, "fy": 1.0, "cx": 50.0, "cy": 50.0,
        }
        frames = [{"frame_index": 0, "stamp_s": 0.5, "width": 100, "height": 100}]
        annotations = [{"frame_index": 0, "detections": [{"track_id": 2, "raw_track_id": 2, "bbox_xyxy": [0, 0, 99, 99]}]}]
        points = np.asarray([[0.0, 0.0, 2.0], [0.01, 0.0, 2.0], [-0.01, 0.0, 2.0]])
        with patch("simulator.perception.rgb_tracking._load_slam_poses", return_value=poses), \
             patch("simulator.perception.rgb_tracking._load_sensor_geometry", return_value=geometry), \
             patch("simulator.perception.rgb_tracking._read_lidar_scans", return_value=[(0.5, points)]):
            estimates, _, _, support = _augment_with_lidar_estimates(
                __import__("pathlib").Path("capture"), __import__("pathlib").Path("slam"), frames, annotations
            )
        self.assertEqual(estimates, {})
        self.assertEqual(support, {})
        self.assertEqual(annotations[0]["detections"][0]["coordinate_source"], "rgb_only_no_lidar_association")

    def test_pointcloud2_decoder_supported_and_unsupported_layouts(self):
        fields32 = [SimpleNamespace(name=name, offset=offset, datatype=7, count=1) for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        packed = SimpleNamespace(width=2, height=1, point_step=12, row_step=24, is_bigendian=False,
                                 fields=fields32, data=struct.pack("<ffffff", 1, 2, 3, 4, 5, 6))
        np.testing.assert_allclose(_decode_pointcloud2_xyz(packed), [[1, 2, 3], [4, 5, 6]])
        nan_cloud = SimpleNamespace(**{**packed.__dict__, "data": struct.pack("<ffffff", float("nan"), 2, 3, 4, 5, 6)})
        decoded_nan = _decode_pointcloud2_xyz(nan_cloud)
        np.testing.assert_allclose(_finite_xyz_points(decoded_nan), [[4, 5, 6]])

        fields64 = [SimpleNamespace(name=name, offset=offset, datatype=8, count=1) for name, offset in (("x", 0), ("y", 8), ("z", 16))]
        row_a, row_b = struct.pack(">ddd", 1, 2, 3), struct.pack(">ddd", 4, 5, 6)
        organized = SimpleNamespace(width=1, height=2, point_step=24, row_step=28, is_bigendian=True,
                                    fields=fields64, data=row_a + b"pad!" + row_b + b"tail")
        np.testing.assert_allclose(_decode_pointcloud2_xyz(organized), [[1, 2, 3], [4, 5, 6]])

        unsupported = SimpleNamespace(**{**packed.__dict__, "fields": [SimpleNamespace(name=f.name, offset=f.offset, datatype=4, count=1) for f in fields32]})
        with self.assertRaisesRegex(ValueError, "Unsupported PointCloud2 datatype"):
            _decode_pointcloud2_xyz(unsupported)
        unsupported_count = SimpleNamespace(**{**packed.__dict__, "fields": [SimpleNamespace(name=f.name, offset=f.offset, datatype=7, count=2) for f in fields32]})
        with self.assertRaisesRegex(ValueError, "unsupported count"):
            _decode_pointcloud2_xyz(unsupported_count)
        invalid_offset = SimpleNamespace(**{**packed.__dict__, "fields": [SimpleNamespace(name=f.name, offset=11 if f.name == "z" else f.offset, datatype=7, count=1) for f in fields32]})
        with self.assertRaisesRegex(ValueError, "outside point_step"):
            _decode_pointcloud2_xyz(invalid_offset)
        truncated = SimpleNamespace(**{**packed.__dict__, "data": packed.data[:10]})
        with self.assertRaisesRegex(ValueError, "shorter than row_step"):
            _decode_pointcloud2_xyz(truncated)
        empty = SimpleNamespace(**{**packed.__dict__, "width": 0, "height": 1, "data": b""})
        self.assertIsNone(_decode_pointcloud2_xyz(empty))
