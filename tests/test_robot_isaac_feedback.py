from __future__ import annotations

import ast
from argparse import Namespace
import copy
import contextlib
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import unittest

from robot_spike.production.isaac_feedback import (
    FeedbackUnavailableError,
    Isaac61LocomotionFeedbackAdapter,
    convex_hull_xy,
    support_polygon_center_and_margin,
)
from robot_spike.production.locomotion import LEG_JOINTS
from robot_spike.production.run_locomotion_smoke import (
    EVIDENCE_FILES,
    EvidenceIdentityGuard,
    EvidenceIdentityMismatchError,
    _execute_smoke,
    _record_identity_boundary,
    _validate_arguments,
    acceptance_spec_identity,
)


ROOT = Path(__file__).resolve().parents[1]


class Array:
    def __init__(self, values):
        self.values = values

    def numpy(self):
        return self

    def tolist(self):
        return self.values


class FakeArticulation:
    dof_names = list(LEG_JOINTS)
    link_names = ["pelvis_link", "left_ankle_roll_link", "right_ankle_roll_link"]

    def get_world_poses(self):
        half = math.sqrt(0.5)
        return Array([[1.0, 2.0, 0.635]]), Array([[half, 0.0, 0.0, half]])

    def get_velocities(self):
        return Array([[0.0, 0.2, 0.0]]), Array([[0.0, 0.0, 0.1]])

    def get_dof_positions(self):
        return Array([[0.01 * index for index in range(len(self.dof_names))]])

    def get_dof_velocities(self):
        return Array([[0.001 * index for index in range(len(self.dof_names))]])

    def get_link_masses(self):
        return Array([[8.0, 1.0, 1.0]])

    def get_link_coms(self):
        return Array([[[0.0, 0.0, 0.0]] * 3]), Array([[[1.0, 0.0, 0.0, 0.0]] * 3])


class FakeLinks:
    def get_world_poses(self):
        return (
            Array([[1.0, 2.0, 0.60], [1.0, 2.10, 0.034], [1.0, 1.90, 0.034]]),
            Array([[1.0, 0.0, 0.0, 0.0]] * 3),
        )


@dataclass
class Reading:
    is_valid: bool = True
    in_contact: bool = True
    value: float = 150.0
    time: float = 1.0


class FakeSensor:
    def __init__(self, reading: Reading, points):
        self.reading = reading
        self.points = points

    def get_sensor_reading(self):
        return self.reading

    def get_raw_data(self):
        return [
            {"position": {"x": point[0], "y": point[1], "z": point[2]}}
            for point in self.points
        ]


class IsaacFeedbackTests(unittest.TestCase):
    def adapter(self, left=Reading(), right=Reading()):
        left_points = [
            (0.955, 2.080, 0.0),
            (0.955, 2.120, 0.0),
            (1.122, 2.072, 0.0),
            (1.122, 2.128, 0.0),
        ] if left.in_contact else []
        right_points = [
            (0.955, 1.880, 0.0),
            (0.955, 1.920, 0.0),
            (1.122, 1.872, 0.0),
            (1.122, 1.928, 0.0),
        ] if right.in_contact else []
        return Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            FakeLinks(),
            FakeSensor(left, left_points),
            FakeSensor(right, right_points),
            lambda: 1.0,
            maximum_contact_age_s=0.01,
        )

    def test_adapter_normalizes_measured_frames_and_com(self):
        adapter = self.adapter()
        feedback = adapter.read_feedback()

        self.assertAlmostEqual(feedback.root_pose.x_m, 1.0)
        self.assertAlmostEqual(feedback.root_pose.y_m, 2.0)
        self.assertAlmostEqual(feedback.root_pose.yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(feedback.root_linear_velocity_body_mps[0], 0.2)
        self.assertAlmostEqual(feedback.root_linear_velocity_body_mps[1], 0.0, places=7)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[0], 1.0385)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[1], 2.10)
        self.assertAlmostEqual(feedback.left_foot.pose.position_m[2], 0.0)
        self.assertEqual(feedback.joint_position_rad[LEG_JOINTS[-1]], 0.11)
        self.assertEqual(feedback.joint_velocity_rad_s[LEG_JOINTS[-1]], 0.011)
        self.assertAlmostEqual(feedback.com_position_world_m[0], 1.0)
        self.assertAlmostEqual(feedback.com_position_world_m[1], 2.0)
        self.assertAlmostEqual(feedback.com_position_world_m[2], 0.4868)
        self.assertGreater(feedback.support_margin_m, 0.0)
        self.assertEqual(adapter.last_contact_forces_n, {"left": 150.0, "right": 150.0})

    def test_single_contact_uses_only_that_sole_polygon(self):
        feedback = self.adapter(right=Reading(in_contact=False, value=0.0)).read_feedback()
        self.assertTrue(feedback.left_foot.in_contact)
        self.assertFalse(feedback.right_foot.in_contact)
        self.assertAlmostEqual(feedback.support_center_world_m[1], 2.10, places=7)
        self.assertLess(feedback.support_margin_m, 0.0)

    def test_missing_stale_or_absent_contact_fails_closed(self):
        with self.assertRaisesRegex(FeedbackUnavailableError, "left contact sensor reading is invalid"):
            self.adapter(left=Reading(is_valid=False)).read_feedback()
        with self.assertRaisesRegex(FeedbackUnavailableError, "stale"):
            self.adapter(left=Reading(time=0.5)).read_feedback()
        with self.assertRaisesRegex(FeedbackUnavailableError, "neither foot"):
            self.adapter(
                left=Reading(in_contact=False, value=0.0),
                right=Reading(in_contact=False, value=0.0),
            ).read_feedback()

    def test_shape_and_mass_failures_do_not_create_plausible_feedback(self):
        articulation = FakeArticulation()
        articulation.get_link_masses = lambda: Array([[8.0, -1.0, 1.0]])
        adapter = Isaac61LocomotionFeedbackAdapter(
            articulation,
            FakeLinks(),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            lambda: 1.0,
        )
        with self.assertRaisesRegex(FeedbackUnavailableError, "finite and nonnegative"):
            adapter.read_feedback()

        links = FakeLinks()
        links.get_world_poses = lambda: (Array([[0.0, 0.0, 0.0]]), Array([[1.0, 0.0, 0.0, 0.0]]))
        adapter = Isaac61LocomotionFeedbackAdapter(
            FakeArticulation(),
            links,
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            FakeSensor(Reading(), [(0.0, 0.0, 0.0)] * 3),
            lambda: 1.0,
        )
        with self.assertRaisesRegex(FeedbackUnavailableError, "link positions"):
            adapter.read_feedback()

    def test_support_hull_margin_and_centroid_are_deterministic(self):
        points = [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]
        self.assertEqual(convex_hull_xy(points), ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)))
        center, inside = support_polygon_center_and_margin(points, (0.0, 0.0))
        self.assertEqual(center, (0.0, 0.0))
        self.assertAlmostEqual(inside, 1.0)
        _center, outside = support_polygon_center_and_margin(points, (1.25, 0.0))
        self.assertAlmostEqual(outside, -0.25)

    def test_future_smoke_harness_has_no_direct_state_setters(self):
        path = ROOT / "robot_spike" / "production" / "run_locomotion_smoke.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        called_attributes = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        top_level_imports = [
            node
            for node in tree.body
            if isinstance(node, (ast.Import, ast.ImportFrom))
        ]
        self.assertFalse(
            any(
                (isinstance(node, ast.ImportFrom) and (node.module or "").startswith("isaacsim"))
                or (
                    isinstance(node, ast.Import)
                    and any(alias.name.startswith("isaacsim") for alias in node.names)
                )
                for node in top_level_imports
            )
        )
        self.assertTrue({"reset", "command_joint_positions"} <= called_attributes)
        self.assertFalse(
            {
                "set_world_poses",
                "set_velocities",
                "set_dof_positions",
                "set_dof_velocities",
            }
            & called_attributes
        )

    def test_smoke_arguments_fail_before_isaac_for_invalid_values(self):
        valid = {
            "physics_hz": 120,
            "repeats": 2,
            "forward_m": 0.06,
            "settle_s": 2.0,
            "timeout_s": 12.0,
        }
        _validate_arguments(Namespace(**valid))
        for name, value in (
            ("physics_hz", 0),
            ("repeats", 0),
            ("forward_m", 0.0),
            ("settle_s", math.inf),
            ("timeout_s", math.nan),
        ):
            invalid = dict(valid)
            invalid[name] = value
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    _validate_arguments(Namespace(**invalid))

    def test_acceptance_spec_bytes_and_consumed_metadata_are_in_identity(self):
        self.assertIn(
            "robot_spike/production/production_manifest.json", EVIDENCE_FILES
        )
        self.assertIn("robot_spike/production/robot_config.json", EVIDENCE_FILES)
        self.assertIn(
            "robot_spike/production/asimov_orcahand_restocking.urdf",
            EVIDENCE_FILES,
        )
        self.assertIn("robot_spike/asimov_orcahand_right.urdf", EVIDENCE_FILES)

        class MemoryPath:
            def __init__(self, data: bytes):
                self.data = data
                self.exists = True

            def resolve(self):
                return self

            def is_file(self):
                return self.exists

            def open(self, mode):
                self.assert_binary_mode = mode
                return io.BytesIO(self.data)

            def __str__(self):
                return "memory://acceptance.md"

        original = b"owner acceptance bytes\r\n"
        expected = hashlib.sha256(original).hexdigest()
        path = MemoryPath(original)
        identity = acceptance_spec_identity(path, expected)  # type: ignore[arg-type]
        self.assertEqual(identity["sha256"], expected)
        self.assertEqual(identity["path"], str(path))
        self.assertEqual(path.assert_binary_mode, "rb")

        path.data = b"changed bytes\n"
        with self.assertRaisesRegex(RuntimeError, "SHA-256 mismatch"):
            acceptance_spec_identity(path, expected)  # type: ignore[arg-type]
        path.exists = False
        with self.assertRaisesRegex(RuntimeError, "is missing"):
            acceptance_spec_identity(path, expected)  # type: ignore[arg-type]

    def test_repeat_and_final_identity_checks_fail_closed_and_persist(self):
        initial = {
            "acceptance_spec": {"sha256": "spec"},
            "source": {
                "source_files_sha256": {
                    "robot_spike/production/production_manifest.json": "manifest",
                    "robot_spike/production/robot_config.json": "config",
                }
            },
            "installed_isaac": {
                "api_source_files_sha256": {"articulation.py": "api"}
            },
        }
        state = {"identity": copy.deepcopy(initial)}
        guard = EvidenceIdentityGuard(
            initial, lambda: copy.deepcopy(state["identity"])
        )
        result = {"status": "running", "identity_checks": guard.checks, "repeats": []}
        repeat = {"repeat_index": 0, "identity_checks": []}
        result["repeats"].append(repeat)

        class CapturingStatusPath:
            text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

        status_path = CapturingStatusPath()
        _record_identity_boundary(
            guard, "repeat_0_start", result, status_path, repeat  # type: ignore[arg-type]
        )
        del state["identity"]["source"]["source_files_sha256"][
            "robot_spike/production/production_manifest.json"
        ]
        with self.assertRaises(EvidenceIdentityMismatchError):
            _record_identity_boundary(
                guard,
                "repeat_0_end",
                result,
                status_path,  # type: ignore[arg-type]
                repeat,
            )
        persisted = json.loads(status_path.text)
        self.assertEqual(
            [check["status"] for check in persisted["repeats"][0]["identity_checks"]],
            ["match", "mismatch"],
        )
        self.assertIn(
            "production_manifest.json",
            persisted["repeats"][0]["identity_checks"][1]["mismatches"][0],
        )

        final_state = copy.deepcopy(initial)
        final_guard = EvidenceIdentityGuard(
            initial, lambda: copy.deepcopy(final_state)
        )
        final_result = {
            "status": "running",
            "identity_checks": final_guard.checks,
            "repeats": [],
        }
        final_state["installed_isaac"]["api_source_files_sha256"][
            "articulation.py"
        ] = "changed-api"
        with self.assertRaises(EvidenceIdentityMismatchError):
            _record_identity_boundary(
                final_guard,
                "final_success",
                final_result,
                status_path,  # type: ignore[arg-type]
            )
        persisted = json.loads(status_path.text)
        self.assertEqual(persisted["identity_checks"][0]["phase"], "final_success")
        self.assertEqual(persisted["identity_checks"][0]["status"], "mismatch")

    def test_preflight_identity_failures_are_durable_and_do_not_launch_runtime(self):
        class MemoryStatusFile:
            def __init__(self):
                self.text = ""

            def write_text(self, text, **_kwargs):
                self.text = text

            def is_file(self):
                return bool(self.text)

            def read_text(self, **_kwargs):
                return self.text

        class MemoryOutput:
            def __init__(self):
                self.created = False
                self.status = MemoryStatusFile()

            def resolve(self):
                return self

            def exists(self):
                return False

            def mkdir(self, **_kwargs):
                self.created = True

            def __truediv__(self, name):
                self.assert_status_name = name
                return self.status

            def __str__(self):
                return "memory://smoke-output"

        for error in (
            "acceptance specification is missing: owner.md",
            "source identity changed: production_manifest.json",
        ):
            with self.subTest(error=error):
                output = MemoryOutput()
                launched = {"value": False}

                def identity_reader(*_args, **_kwargs):
                    raise RuntimeError(error)

                def runtime_runner(*_args, **_kwargs):
                    launched["value"] = True
                    raise AssertionError("runtime must not launch after preflight failure")

                args = Namespace(
                    output=output,
                    expected_candidate_sha="candidate",
                    expected_candidate_tree_sha="tree",
                    acceptance_spec=Path("owner.md"),
                )
                with contextlib.redirect_stderr(io.StringIO()):
                    exit_code = _execute_smoke(
                        args,
                        ROOT,
                        identity_reader=identity_reader,
                        runtime_runner=runtime_runner,
                    )
                self.assertEqual(exit_code, 2)
                self.assertTrue(output.created)
                self.assertEqual(
                    output.assert_status_name, "locomotion_smoke_status.json"
                )
                report = json.loads(output.status.text)
                self.assertEqual(report["phase"], "preflight_identity")
                self.assertFalse(report["runtime_invoked"])
                self.assertEqual(report["error"], error)
                self.assertFalse(launched["value"])


if __name__ == "__main__":
    unittest.main()
