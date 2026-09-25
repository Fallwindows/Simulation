"""CPU-only checks for the continuous technical-view camera and overlays."""

from __future__ import annotations

import hashlib
import json
import tempfile
import time
import unittest
from collections import OrderedDict
from dataclasses import replace
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import patch

import numpy as np

from simulator.technical_views import (
    TechnicalCameraPath,
    _activation_ray_opacities,
    camera_motion_receipt,
    load_plan,
)
from simulator.technical_lidar import (
    EstimatedTrajectory,
    RgbFrameRecord,
    ScanRecord,
    SelectiveLidarSource,
    _transform_points,
    load_camera_head_transform_artifact,
)
from simulator.sensors.scan_projection import CameraIntrinsics
from simulator.presentation.technical_bundle import (
    CAMERA_TRACE_BASIS,
    _validate_camera_head_transform_receipt,
    _validate_camera_motion_trace,
)


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config" / "technical_views.json"


def _write_camera_head_fixture(directory: Path) -> tuple[Path, Path]:
    artifact = directory / "camera_head_transforms.json"
    optical = {
        "parent": "camera_link", "child": "camera_optical_frame",
        "translation_m": [0.0, 0.0, 0.0],
        "rotation_xyzw": [0.5, -0.5, 0.5, -0.5],
    }
    artifact_payload = {
        "schema": "grocery.camera_head_transforms", "version": 1,
        "frames": {"parent": "sensor_rig", "child": "camera_link", "optical_child": "camera_optical_frame"},
        "direction": "parent_to_child", "translation_units": "m", "timestamp_units": "s",
        "timestamp_domain": "Isaac simulation time (/clock)", "sample_hz": 2.0, "duration_s": 1.0,
        "interpolation": {
            "translation": "linear", "rotation": "shortest_arc_quaternion_slerp_xyzw",
            "range": "closed_0_to_duration_no_extrapolation",
        },
        "composition": "q_sensor_rig_camera_link = q_configured_mount * q_head_articulation",
        "source": {
            "trajectory_config": {"path": "config/scenario.yaml", "sha256": "a" * 64},
            "git_commit": "b" * 40, "git_tree": "c" * 40,
        },
        "static_child_transform": optical,
        "samples": [
            {"timestamp_s": 0.0, "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]},
            {"timestamp_s": 0.5, "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.3826834323650898, 0.9238795325112867]},
            {"timestamp_s": 1.0, "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 2 ** -0.5, 2 ** -0.5]},
        ],
    }
    artifact.write_text(json.dumps(artifact_payload, sort_keys=True) + "\n", encoding="utf-8")
    sensor_transforms = directory / "sensor_transforms.json"
    sensor_payload = {
        "frames": {"sensor_rig": "sensor_rig", "camera_link": "camera_link", "camera_optical": "camera_optical_frame", "lidar_link": "lidar_link"},
        "transforms": [
            optical,
            {"parent": "sensor_rig", "child": "lidar_link", "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        ],
        "dynamic_transform_artifacts": [{
            "parent_frame": "sensor_rig", "child_frame": "camera_link", "path": artifact.name,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "size_bytes": artifact.stat().st_size, "schema_version": 1,
        }],
    }
    sensor_transforms.write_text(json.dumps(sensor_payload, sort_keys=True) + "\n", encoding="utf-8")
    (directory / "capture_manifest.json").write_text(json.dumps({
        "files": [{
            "path": artifact.name,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "size_bytes": artifact.stat().st_size,
        }],
    }, sort_keys=True) + "\n", encoding="utf-8")
    return sensor_transforms, artifact


class _SyntheticTrajectory:
    timestamps_s = np.asarray([
        0.2, 1.2, 2.3, 3.5, 4.6, 5.6, 6.8, 8.0, 9.2,
        10.2, 11.3, 12.5, 13.7, 14.9, 16.1, 17.3, 18.6,
    ], dtype=np.float64)

    def interpolation_dependency_window_s(self, start_s: float, end_s: float) -> tuple[float, float]:
        lower = max(0, int(np.searchsorted(self.timestamps_s, start_s, side="right")) - 1)
        upper = min(len(self.timestamps_s) - 1, int(np.searchsorted(self.timestamps_s, end_s, side="right")))
        return float(self.timestamps_s[lower]), float(self.timestamps_s[upper])

    def map_from_sensor_rig(self, timestamp_s: float) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        # Optical +Z points down the synthetic aisle (+X).  The position has
        # gentle lateral/elevation motion so the trace exercises all axes.
        matrix[:3, :3] = np.asarray([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ])
        matrix[:3, 3] = (
            0.42 * timestamp_s,
            0.18 * np.sin(timestamp_s / 4.0),
            1.15 + 0.03 * np.cos(timestamp_s / 5.0),
        )
        return matrix


class TechnicalMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        profiles, cls.views = load_plan(PLAN)
        cls.fps = profiles["delivery"].fps
        cls.path = TechnicalCameraPath(
            cls.views,
            _SyntheticTrajectory(),
            np.eye(4, dtype=np.float64),
            np.asarray([5.2, 1.4, 1.25], dtype=np.float64),
        )
        cls.trace = cls.path.trace_rows(cls.fps)

    def test_plan_assigns_one_continuous_path_and_a_24_frame_final_hold(self):
        self.assertEqual(
            [view.camera_motion_role for view in self.views],
            ["recorded_camera_optical"] * 2 + ["continuous_estimated_map_path"] * 5,
        )
        self.assertEqual([view.final_hold_frames for view in self.views], [0, 0, 0, 0, 0, 0, 24])
        self.assertEqual(len(self.trace), 810)
        self.assertEqual(self.views[0].title, "EARLIER RECORDED SENSOR REPLAY")
        self.assertEqual(self.views[0].subtitle, "RECORDED t=2.0–5.9s · CO-TIMED RGB + LIDAR")
        self.assertEqual(
            [view.camera_pose_sampling_window_s for view in self.views],
            [(2.0, 5.9), (5.9, 8.8)] + [(2.0, 18.5)] * 5,
        )
        self.assertEqual(
            self.path.camera_pose_dependency_windows_receipt(),
            {
                "sensor_activation": [1.2, 6.8],
                "lidar_environment": [5.6, 9.2],
                **{view.id: [1.2, 18.6] for view in self.views[2:]},
            },
        )

    def test_camera_pose_time_and_rendered_data_cutoff_are_distinct_and_in_declared_windows(self):
        by_id = {view.id: view for view in self.views}
        for row in self.trace:
            spec = by_id[row["view_id"]]
            pose_time = row["camera_guide_pose_timestamp_s"]
            cutoff = row["rendered_data_cutoff_s"]
            self.assertGreaterEqual(pose_time, spec.camera_guide_timestamp_window_s[0] - 1e-9)
            self.assertLessEqual(pose_time, spec.camera_guide_timestamp_window_s[1] + 1e-9)
            dependency = self.path.camera_pose_dependency_window_s(spec)
            self.assertGreaterEqual(pose_time, dependency[0] - 1e-9)
            self.assertLessEqual(pose_time, dependency[1] + 1e-9)
            self.assertGreaterEqual(cutoff, spec.display_window_s[0] - 1e-9)
            self.assertLessEqual(cutoff, spec.display_window_s[1] + 1e-9)
        detail = [row for row in self.trace if row["view_id"] == "object_detail"]
        self.assertGreater(detail[-1]["camera_guide_pose_timestamp_s"], 12.8)
        self.assertTrue(all(row["rendered_data_cutoff_s"] == 12.8 for row in detail))
        receipt = camera_motion_receipt(self.views[0], self.trace[:120], type("Focus", (), {
            "track_id": 7, "position": (5.2, 1.4, 1.25),
        })(), self.path)
        self.assertEqual(receipt["render_application"], "not_applied; trace audits recorded camera pose for the timestamped sensor replay")
        self.assertIn("no presentation camera applied", receipt["path"])
        self.assertEqual(receipt["camera_pose_sampling_window_s"], [2.0, 5.9])
        self.assertEqual(receipt["camera_pose_dependency_window_s"], [1.2, 6.8])

    def test_sparse_nonaligned_knot_dependencies_are_exact_and_mutation_bound(self):
        timestamps = np.asarray([
            0.2, 1.2, 2.3, 3.5, 4.6, 5.6, 6.8, 8.0, 9.2,
            10.2, 11.3, 12.5, 13.7, 14.9, 16.1, 17.3, 18.6, 19.7,
        ])
        base_positions = np.column_stack((timestamps, 0.03 * timestamps, np.ones_like(timestamps)))
        outside_positions = base_positions.copy()
        outside_positions[(timestamps < 1.2) | (timestamps > 18.6), 1] += 40.0
        quaternions = np.tile(np.asarray([0.0, 0.0, 0.0, 1.0]), (len(timestamps), 1))
        base_trajectory = EstimatedTrajectory(timestamps, base_positions, quaternions)
        outside_trajectory = EstimatedTrajectory(timestamps, outside_positions, quaternions)
        self.assertEqual(base_trajectory.interpolation_dependency_window_s(2.0, 5.9), (1.2, 6.8))
        self.assertEqual(base_trajectory.interpolation_dependency_window_s(5.9, 8.8), (5.6, 9.2))
        self.assertEqual(base_trajectory.interpolation_dependency_window_s(2.0, 18.5), (1.2, 18.6))
        base_path = TechnicalCameraPath(
            self.views, base_trajectory,
            np.eye(4), np.asarray([5.2, 1.4, 1.25]),
        )
        outside_path = TechnicalCameraPath(
            self.views, outside_trajectory,
            np.eye(4), np.asarray([5.2, 1.4, 1.25]),
        )
        base = base_path.trace_rows(self.fps)
        outside_changed = outside_path.trace_rows(self.fps)
        self.assertEqual(len(base), 810)
        self.assertEqual(base, outside_changed)
        self.assertEqual(base_path.camera_pose_dependency_windows_receipt(), {
            "sensor_activation": [1.2, 6.8],
            "lidar_environment": [5.6, 9.2],
            **{view.id: [1.2, 18.6] for view in self.views[2:]},
        })

        support_positions = base_positions.copy()
        support_positions[timestamps == 6.8, 1] += 40.0
        support_changed = TechnicalCameraPath(
            self.views, EstimatedTrajectory(timestamps, support_positions, quaternions),
            np.eye(4), np.asarray([5.2, 1.4, 1.25]),
        ).trace_rows(self.fps)
        changed_shot6 = [
            index for index, (left, right) in enumerate(zip(base[:120], support_changed[:120]))
            if left["eye_m"] != right["eye_m"]
        ]
        self.assertEqual(changed_shot6, list(range(111, 120)))

        for support_timestamp in (1.2, 18.6):
            map_support_positions = base_positions.copy()
            map_support_positions[timestamps == support_timestamp, 1] += 40.0
            map_support_changed = TechnicalCameraPath(
                self.views, EstimatedTrajectory(timestamps, map_support_positions, quaternions),
                np.eye(4), np.asarray([5.2, 1.4, 1.25]),
            ).trace_rows(self.fps)
            self.assertTrue(any(
                left["eye_m"] != right["eye_m"]
                for left, right in zip(base[210:], map_support_changed[210:])
            ))

    def test_dense_canonical_producer_derives_its_own_exact_dependency_windows(self):
        timestamps = np.round(np.arange(0.2, 20.4 + 0.025, 0.05), 9)
        positions = np.column_stack((timestamps, np.zeros_like(timestamps), np.ones_like(timestamps)))
        quaternions = np.tile(np.asarray([0.0, 0.0, 0.0, 1.0]), (len(timestamps), 1))
        path = TechnicalCameraPath(
            self.views,
            EstimatedTrajectory(timestamps, positions, quaternions),
            np.eye(4),
            np.asarray([5.2, 1.4, 1.25]),
        )
        self.assertEqual(len(path.trace_rows(self.fps)), 810)
        self.assertEqual(path.camera_pose_dependency_windows_receipt(), {
            "sensor_activation": [2.0, 5.95],
            "lidar_environment": [5.9, 8.85],
            **{view.id: [2.0, 18.55] for view in self.views[2:]},
        })

    def test_camera_steps_and_velocity_stay_continuous_at_every_boundary(self):
        eyes = np.asarray([row["eye_m"] for row in self.trace], dtype=np.float64)
        velocities = np.asarray([row["eye_velocity_mps"] for row in self.trace], dtype=np.float64)
        boundary_indices = [index for index, row in enumerate(self.trace) if row["boundary_from_previous"]]
        self.assertEqual(boundary_indices, [120, 210, 300, 420, 540, 660])
        steps = np.linalg.norm(np.diff(eyes, axis=0), axis=1)
        for boundary in boundary_indices:
            local_steps = np.concatenate((steps[max(0, boundary - 10):boundary - 1], steps[boundary:boundary + 9]))
            self.assertLessEqual(
                steps[boundary - 1],
                max(0.025, float(np.max(local_steps)) * 1.35),
                f"camera position reset at technical boundary frame {boundary}",
            )
            outer_changes = np.concatenate((
                np.linalg.norm(np.diff(velocities[boundary - 10:boundary - 1], axis=0), axis=1),
                np.linalg.norm(np.diff(velocities[boundary + 2:boundary + 11], axis=0), axis=1),
            ))
            limit = max(0.20, float(np.max(outer_changes)) * 1.75)
            for first, second in ((boundary - 1, boundary), (boundary, boundary + 1)):
                self.assertLessEqual(
                    float(np.linalg.norm(velocities[second] - velocities[first])),
                    limit,
                    f"camera velocity pulse at technical boundary frame {boundary}",
                )

    def test_final_view_eases_into_an_exact_24_frame_hold(self):
        final_rows = [row for row in self.trace if row["view_id"] == "final_technical_view"]
        held = final_rows[-24:]
        self.assertTrue(all(row["eye_m"] == held[0]["eye_m"] for row in held))
        self.assertTrue(all(row["target_m"] == held[0]["target_m"] for row in held))
        self.assertTrue(all(row["eye_velocity_mps"] == [0.0, 0.0, 0.0] for row in held[1:]))
        self.assertTrue(all(row["eye_acceleration_mps2"] == [0.0, 0.0, 0.0] for row in held[2:]))
        pre_hold_speed = np.linalg.norm(np.asarray(final_rows[-25]["eye_velocity_mps"], dtype=np.float64))
        self.assertLess(pre_hold_speed, 0.02)
        self.assertNotEqual(final_rows[-25]["eye_m"], held[0]["eye_m"])

    def test_validator_rejects_a_self_consistent_25_frame_hold(self):
        long_hold_views = (*self.views[:-1], replace(self.views[-1], final_hold_frames=25))
        long_hold_path = TechnicalCameraPath(
            long_hold_views,
            _SyntheticTrajectory(),
            np.eye(4, dtype=np.float64),
            np.asarray([5.2, 1.4, 1.25], dtype=np.float64),
        )
        rows = long_hold_path.trace_rows(self.fps)
        plan = json.loads(PLAN.read_text(encoding="utf-8"))
        plan_views = {item["id"]: item for item in plan["views"]}
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            trace = directory / "technical_camera_trace.jsonl"
            trace.write_text(
                "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows),
                encoding="utf-8",
            )
            info = {
                "path": trace.name,
                "sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
                "frame_count": 810,
                "basis": CAMERA_TRACE_BASIS,
                "camera_pose_dependency_windows_s": long_hold_path.camera_pose_dependency_windows_receipt(),
            }
            with self.assertRaisesRegex(ValueError, "exceeds the exact declared frame count"):
                _validate_camera_motion_trace(
                    info,
                    directory=directory,
                    plan_views=plan_views,
                    expected_order=[view.id for view in self.views],
                    expected_counts={view.id: view.frames for view in self.views},
                    simulation_window_s=(0.2, 20.4),
                    fps=self.fps,
                    expected_rows=rows,
                    expected_dependency_windows_s=long_hold_path.camera_pose_dependency_windows_receipt(),
                )

    def test_activation_rays_fade_continuously_instead_of_integer_popping(self):
        samples = np.stack([_activation_ray_opacities(index / 120.0) for index in range(121)])
        self.assertTrue(np.array_equal(samples[0], np.zeros(12, dtype=np.float32)))
        self.assertTrue(np.allclose(samples[-1], np.ones(12, dtype=np.float32)))
        self.assertTrue(np.all(np.diff(samples, axis=0) >= -1e-7))
        self.assertLess(float(np.max(np.diff(samples, axis=0))), 0.13)
        self.assertGreater(np.count_nonzero((samples > 0.0) & (samples < 1.0)), 100)

    def test_piecewise_estimated_trajectory_has_no_boundary_velocity_acceleration_or_jerk_spike(self):
        timestamps = np.asarray([
            0.2, 1.2, 2.3, 3.5, 4.6, 5.6, 6.8, 8.0, 9.2,
            10.2, 11.3, 12.5, 13.7, 14.9, 16.1, 17.3, 18.6,
        ])
        segment = np.arange(len(timestamps) - 1, dtype=np.float64)
        steps = np.column_stack((
            0.28 + 0.12 * np.sin(segment * 1.7),
            0.11 * np.sign(np.sin(segment * 1.3)),
            0.025 * np.cos(segment * 2.1),
        ))
        positions = np.vstack((
            np.asarray([0.0, 0.0, 1.1]),
            np.asarray([0.0, 0.0, 1.1]) + np.cumsum(steps, axis=0),
        ))
        trajectory = EstimatedTrajectory(
            timestamps,
            positions,
            np.tile(np.asarray([0.0, 0.0, 0.0, 1.0]), (len(timestamps), 1)),
        )
        trace = TechnicalCameraPath(
            self.views,
            trajectory,
            np.eye(4, dtype=np.float64),
            np.asarray([4.8, 1.1, 1.2]),
        ).trace_rows(self.fps)
        boundaries = [120, 210, 300, 420, 540, 660]
        tolerances = {
            "eye_velocity_mps": 0.05,
            "eye_acceleration_mps2": 0.20,
            "eye_jerk_mps3": 1.0,
            "target_velocity_mps": 0.05,
            "target_acceleration_mps2": 0.20,
            "target_jerk_mps3": 1.0,
        }
        for field, absolute_tolerance in tolerances.items():
            values = np.asarray([row[field] for row in trace], dtype=np.float64)
            magnitudes = np.linalg.norm(values, axis=1)
            changes = np.linalg.norm(np.diff(values, axis=0), axis=1)
            for boundary in boundaries:
                neighbor_magnitude = max(magnitudes[boundary - 2], magnitudes[boundary + 2])
                self.assertLessEqual(
                    magnitudes[boundary],
                    neighbor_magnitude * 1.35 + absolute_tolerance,
                    f"{field} magnitude pulse at frame {boundary}",
                )
                outside = np.concatenate((
                    changes[boundary - 8:boundary - 2],
                    changes[boundary + 2:boundary + 8],
                ))
                local_limit = max(absolute_tolerance, float(np.max(outside)) * 1.8)
                self.assertLessEqual(changes[boundary - 1], local_limit, f"{field} entry jump at {boundary}")
                self.assertLessEqual(changes[boundary], local_limit, f"{field} exit jump at {boundary}")

    def test_four_frame_rgb_lru_prevents_pair_backtracking_restarts(self):
        source = object.__new__(SelectiveLidarSource)
        source._decoder_size = None
        source._decoded_frame_index = -1
        source._decoded_frame = None
        source._rgb_frame_cache = OrderedDict()
        source.rgb_decoder_restart_count = 0
        source.rgb_decoded_frame_count = 0

        def restart(instance, width, height):
            instance._decoder_size = (width, height)
            instance._decoded_frame_index = -1
            instance._decoded_frame = None
            instance.rgb_decoder_restart_count += 1

        def read_bytes(_instance, size):
            return bytes(size)

        source._restart_decoder = MethodType(restart, source)
        source._read_decoder_bytes = MethodType(read_bytes, source)
        records = {
            index: RgbFrameRecord(index, index / 30.0, 2, 2, "camera_optical_frame")
            for index in range(175)
        }
        access_order: list[int] = []
        for previous in range(0, 172, 3):
            latest = previous + 3
            access_order.extend((previous, latest, previous, latest))
        for index in access_order:
            source.read_rgb_frame(records[index], 2, 2)
        self.assertLessEqual(source.rgb_decoder_restart_count, 1)
        self.assertLessEqual(source.rgb_decoded_frame_count, 175)
        self.assertLessEqual(len(source._rgb_frame_cache), 4)

    def test_bound_camera_head_transform_drives_timestamped_lidar_alignment(self):
        with tempfile.TemporaryDirectory() as temporary:
            transforms_path, _artifact_path = _write_camera_head_fixture(Path(temporary))
            trajectory, receipt = load_camera_head_transform_artifact(transforms_path)
        self.assertIsNotNone(trajectory)
        self.assertEqual(receipt["mode"], "dynamic_bound_artifact")
        self.assertEqual(receipt["sample_count"], 3)
        trajectory.validate_image_timestamps([0.0, 0.5, 1.0])
        with self.assertRaisesRegex(ValueError, "no matching camera head transform sample"):
            trajectory.validate_image_timestamps([0.0, 0.4, 1.0])
        _validate_camera_head_transform_receipt(receipt, receipt, "sensor_activation")
        forged_receipt = dict(receipt, sha256="0" * 64)
        with self.assertRaisesRegex(ValueError, "camera head transform receipt"):
            _validate_camera_head_transform_receipt(forged_receipt, receipt, "sensor_activation")
        with self.assertRaisesRegex(ValueError, "camera head transform receipt"):
            _validate_camera_head_transform_receipt(None, receipt, "sensor_activation")
        source = object.__new__(SelectiveLidarSource)
        source.camera_head_trajectory = trajectory
        source.optical_from_lidar = np.eye(4, dtype=np.float64)
        source.optical_from_camera_link = np.eye(4, dtype=np.float64)
        source.rig_from_lidar = np.eye(4, dtype=np.float64)
        source.trajectory = EstimatedTrajectory(
            np.asarray([0.0, 1.0]),
            np.zeros((2, 3)),
            np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
        )
        point = np.asarray([[1.0, 0.0, 4.0]])
        at_start = _transform_points(source._optical_from_lidar_at(0.0, 0.0), point)
        after_turn = _transform_points(source._optical_from_lidar_at(0.0, 1.0), point)
        np.testing.assert_allclose(at_start, [[1.0, 0.0, 4.0]], atol=1e-12)
        np.testing.assert_allclose(after_turn, [[0.0, -1.0, 4.0]], atol=1e-12)
        with self.assertRaises(ValueError):
            source._optical_from_lidar_at(0.0, 1.01)

        source.trajectory = EstimatedTrajectory(
            np.asarray([0.0, 1.0]),
            np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
        )
        moved_base = _transform_points(
            source._optical_from_lidar_at(0.0, 1.0), np.asarray([[2.0, 0.0, 4.0]])
        )
        np.testing.assert_allclose(moved_base, [[0.0, -1.0, 4.0]], atol=1e-12)

        started = time.perf_counter()
        for timestamp in np.linspace(0.0, 1.0, 5000):
            trajectory.rig_from_camera_link(float(timestamp))
        self.assertLess(time.perf_counter() - started, 2.0)

    def test_camera_head_artifact_rejects_missing_stale_duplicate_and_nonmonotonic_data(self):
        def refresh_binding(transforms_path: Path, artifact_path: Path) -> None:
            payload = json.loads(transforms_path.read_text(encoding="utf-8"))
            declaration = payload["dynamic_transform_artifacts"][0]
            declaration["sha256"] = hashlib.sha256(artifact_path.read_bytes()).hexdigest()
            declaration["size_bytes"] = artifact_path.stat().st_size
            transforms_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
            manifest_path = transforms_path.with_name("capture_manifest.json")
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["files"][0].update(
                sha256=declaration["sha256"], size_bytes=declaration["size_bytes"]
            )
            manifest_path.write_text(json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            # Each attack gets an independent, valid starting artifact.
            for attack in (
                "missing", "stale", "missing_declaration", "duplicate_declaration",
                "duplicate_static_edge", "unbound_manifest", "nonmonotonic",
            ):
                case = root / attack
                case.mkdir()
                transforms_path, artifact_path = _write_camera_head_fixture(case)
                if attack == "missing":
                    artifact_path.unlink()
                    expected = "missing"
                elif attack == "stale":
                    artifact_path.write_bytes(artifact_path.read_bytes() + b"stale")
                    expected = "size or SHA-256"
                elif attack == "missing_declaration":
                    payload = json.loads(transforms_path.read_text(encoding="utf-8"))
                    payload.pop("dynamic_transform_artifacts")
                    transforms_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
                    expected = "neither a complete static path nor a dynamic artifact"
                elif attack == "duplicate_declaration":
                    payload = json.loads(transforms_path.read_text(encoding="utf-8"))
                    payload["dynamic_transform_artifacts"].append(dict(payload["dynamic_transform_artifacts"][0]))
                    transforms_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
                    expected = "duplicates a frame pair"
                elif attack == "duplicate_static_edge":
                    payload = json.loads(transforms_path.read_text(encoding="utf-8"))
                    payload["transforms"].append({
                        "parent": "sensor_rig", "child": "camera_link",
                        "translation_m": [0.0, 0.0, 0.0],
                        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    })
                    transforms_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
                    expected = "disjoint dynamic and static graph edges"
                elif attack == "unbound_manifest":
                    manifest_path = transforms_path.with_name("capture_manifest.json")
                    manifest_path.write_text('{"files":[]}\n', encoding="utf-8")
                    expected = "capture manifest does not bind"
                else:
                    payload = json.loads(artifact_path.read_text(encoding="utf-8"))
                    payload["samples"][1]["timestamp_s"] = 0.0
                    artifact_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
                    refresh_binding(transforms_path, artifact_path)
                    expected = "strictly increasing"
                with self.subTest(attack=attack), self.assertRaisesRegex(ValueError, expected):
                    load_camera_head_transform_artifact(transforms_path)

        with tempfile.TemporaryDirectory() as temporary:
            transforms_path, _artifact_path = _write_camera_head_fixture(Path(temporary))
            payload = json.loads(transforms_path.read_text(encoding="utf-8"))
            payload.pop("dynamic_transform_artifacts")
            payload["transforms"].append({
                "parent": "sensor_rig", "child": "camera_link",
                "translation_m": [0.1, 0.0, 1.2],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            })
            transforms_path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
            trajectory, receipt = load_camera_head_transform_artifact(transforms_path)
            self.assertIsNone(trajectory)
            self.assertEqual(receipt["mode"], "static_sensor_transform")

    def test_dynamic_projection_cache_is_keyed_by_rgb_frame_and_remains_bounded(self):
        with tempfile.TemporaryDirectory() as temporary:
            transforms_path, _artifact_path = _write_camera_head_fixture(Path(temporary))
            trajectory, _receipt = load_camera_head_transform_artifact(transforms_path)
        source = object.__new__(SelectiveLidarSource)
        source.database_path = Path("unused.db3")
        source.lidar_frame_id = "lidar_link"
        source.minimum_depth_m = 0.05
        source.maximum_depth_m = 20.0
        source.intrinsics = CameraIntrinsics(100, 100, 10.0, 10.0, 50.0, 50.0)
        source.camera_head_trajectory = trajectory
        source.optical_from_lidar = np.eye(4, dtype=np.float64)
        source.optical_from_camera_link = np.eye(4, dtype=np.float64)
        source.rig_from_lidar = np.eye(4, dtype=np.float64)
        source.trajectory = EstimatedTrajectory(
            np.asarray([0.0, 1.0]), np.zeros((2, 3)),
            np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
        )
        source._projected_cache = OrderedDict()
        record = ScanRecord(7, 0)
        cloud = SimpleNamespace(
            frame_id="lidar_link",
            fields=tuple(SimpleNamespace(name=name) for name in ("x", "y", "z", "intensity")),
            xyz_m=np.asarray([[1.0, 0.0, 4.0], [0.5, 0.0, 3.0]]),
            raw_point_indices=np.asarray([11, 29]),
        )
        bag_record = SimpleNamespace(cloud=cloud)
        projected = []
        with patch("simulator.technical_lidar.read_pointcloud2_sqlite", return_value=bag_record):
            for frame_index, timestamp in enumerate(np.linspace(0.0, 1.0, 6)):
                rgb = RgbFrameRecord(frame_index, float(timestamp), 100, 100, "camera_optical_frame")
                projected.append(source._projected(record, rgb)[0])
        self.assertEqual(len(source._projected_cache), 4)
        self.assertEqual(list(source._projected_cache), [
            (0, 400_000_000), (0, 600_000_000), (0, 800_000_000), (0, 1_000_000_000),
        ])
        self.assertFalse(np.allclose(projected[0].u_px, projected[-1].u_px))
        np.testing.assert_array_equal(projected[0].raw_point_indices, [11, 29])
        np.testing.assert_array_equal(projected[-1].raw_point_indices, [11, 29])


if __name__ == "__main__":
    unittest.main()
