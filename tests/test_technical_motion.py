"""CPU-only checks for the continuous technical-view camera and overlays."""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from simulator.technical_views import (
    TechnicalCameraPath,
    _activation_ray_opacities,
    load_plan,
)


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config" / "technical_views.json"


class _SyntheticTrajectory:
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

    def test_activation_rays_fade_continuously_instead_of_integer_popping(self):
        samples = np.stack([_activation_ray_opacities(index / 120.0) for index in range(121)])
        self.assertTrue(np.array_equal(samples[0], np.zeros(12, dtype=np.float32)))
        self.assertTrue(np.allclose(samples[-1], np.ones(12, dtype=np.float32)))
        self.assertTrue(np.all(np.diff(samples, axis=0) >= -1e-7))
        self.assertLess(float(np.max(np.diff(samples, axis=0))), 0.13)
        self.assertGreater(np.count_nonzero((samples > 0.0) & (samples < 1.0)), 100)


if __name__ == "__main__":
    unittest.main()
