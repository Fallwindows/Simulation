import unittest
import struct
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from evaluation.metrics import PoseSample
from simulator.perception.rgb_tracking import (
    BlobTracker,
    Detection,
    _augment_with_lidar_estimates,
    _budget_detections_spatially,
    _co_visible_raw_track_pairs,
    _consolidate_track_estimates,
    _decode_pointcloud2_xyz,
    _finite_xyz_points,
    _set_canonical_track_identity,
    _set_unassigned_persistent_identity,
    _resolution_scaled_component_limits,
    _spatial_grid_candidate_indices,
    _world_points_to_camera,
    detect_product_blobs,
)


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
             patch("simulator.perception.rgb_tracking._read_lidar_scans", return_value=[(0.01, points), (0.01, points)]):
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
