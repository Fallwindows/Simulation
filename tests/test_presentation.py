import json
import tempfile
import unittest
from pathlib import Path

from simulator.presentation.render_video import plan_segments, render_presentation
from simulator.presentation.timeline import EXPECTED_SHOT_BOUNDARIES, inspect_inputs, load_plan


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config" / "presentation" / "storyboard.yaml"
BASELINE_INPUTS = ROOT / "config" / "presentation" / "diagnostic_baseline_inputs.json"


class PresentationTimelineTests(unittest.TestCase):
    def test_all_twelve_shots_have_exact_order_and_boundaries(self):
        plan = load_plan(PLAN)
        self.assertEqual([shot.number for shot in plan.shots], list(range(1, 13)))
        self.assertEqual(
            [(shot.start_frame, shot.end_frame_exclusive) for shot in plan.shots],
            list(EXPECTED_SHOT_BOUNDARIES),
        )
        self.assertEqual(sum(shot.frame_count for shot in plan.shots), 1350)
        self.assertEqual(plan.shot_for_frame(0).number, 1)
        self.assertEqual(plan.shot_for_frame(89).number, 1)
        self.assertEqual(plan.shot_for_frame(90).number, 2)
        self.assertEqual(plan.shot_for_frame(1349).number, 12)
        with self.assertRaises(IndexError):
            plan.shot_for_frame(1350)

    def test_profiles_are_native_delivery_and_preview(self):
        plan = load_plan(PLAN)
        self.assertEqual((plan.fps, plan.duration_seconds, plan.frame_count), (30, 45, 1350))
        self.assertEqual((plan.profiles["delivery"].width, plan.profiles["delivery"].height), (1920, 1080))
        self.assertEqual((plan.profiles["preview"].width, plan.profiles["preview"].height), (1280, 720))
        self.assertEqual(plan.profiles["delivery"].frame_count, 1350)
        self.assertEqual(plan.profiles["preview"].fps, 30)

    def test_invalid_profile_and_boundary_are_rejected(self):
        source = json.loads(PLAN.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            source["profiles"]["delivery"]["width"] = 1280
            path.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "delivery profile must be 1920x1080"):
                load_plan(path)
            source["profiles"]["delivery"]["width"] = 1920
            source["shots"][5]["start_frame"] = 541
            path.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "shot 06 must use boundary"):
                load_plan(path)

    def test_technical_shots_require_genuine_sensor_and_estimator_roles(self):
        plan = load_plan(PLAN)
        expected = {
            1: {"rgb"},
            5: {"rgb"},
            6: {"rgb", "lidar", "pose"},
            7: {"lidar", "pose"},
            8: {"map", "pose"},
            9: {"map", "reconstruction"},
            10: {"rgb", "map", "reconstruction"},
            11: {"map", "reconstruction", "pose"},
            12: {"rgb", "map", "reconstruction", "pose"},
        }
        for number, roles in expected.items():
            self.assertEqual(set(plan.shots[number - 1].required_roles), roles)
        self.assertEqual(
            set(plan.role_contracts),
            {"rgb", "lidar", "pose", "map", "reconstruction"},
        )
        for role in ("rgb", "lidar", "map", "reconstruction"):
            self.assertIn("view_video", plan.role_contracts[role].required_artifacts)

    def test_existing_rgb_baseline_is_diagnostic_and_never_complete(self):
        plan = load_plan(PLAN)
        report = inspect_inputs(plan, BASELINE_INPUTS)
        self.assertFalse(report.complete)
        self.assertEqual(report.ready_genuine_roles, frozenset())
        self.assertIn("rgb", report.missing_by_shot[1])
        segments = plan_segments(plan, report, "diagnostic")
        self.assertEqual([segment.kind for segment in segments[:5]], ["diagnostic_baseline"] * 5)
        self.assertEqual([segment.kind for segment in segments[5:]], ["missing_input_slate"] * 7)
        self.assertTrue(all("references/storyboard" not in str(segment.video_path) for segment in segments))
        with self.assertRaises(ValueError):
            plan_segments(plan, report, "complete")

    def test_complete_contract_fixture_requires_every_declared_artifact(self):
        plan = load_plan(PLAN)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            roles = {}
            for name, contract in plan.role_contracts.items():
                artifacts = {}
                for artifact_name in contract.required_artifacts:
                    artifact = root / f"{name}_{artifact_name}.dat"
                    artifact.write_text(f"{name}:{artifact_name}\n", encoding="utf-8")
                    artifacts[artifact_name] = artifact.name
                roles[name] = {
                    "contract": name,
                    "status": "complete",
                    "provenance": "genuine",
                    "capture_id": "fixture-capture",
                    "artifacts": artifacts,
                }
            manifest = root / "inputs.json"
            manifest.write_text(
                json.dumps({"schema_version": 1, "label": "fixture", "ground_truth_consumed": False, "roles": roles}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            self.assertTrue(report.complete)
            self.assertEqual(report.ready_genuine_roles, set(plan.role_contracts))
            self.assertTrue(all(not missing for missing in report.missing_by_shot.values()))
            missing_artifact = root / "lidar_scan_index.dat"
            missing_artifact.unlink()
            incomplete = inspect_inputs(plan, manifest)
            self.assertFalse(incomplete.complete)
            self.assertIn("lidar", incomplete.missing_by_shot[6])

    def test_storyboard_pixels_are_rejected_as_inputs(self):
        plan = load_plan(PLAN)
        reference = ROOT / "references" / "storyboard" / "01_enter_aisle_rgb.png"
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            manifest = Path(directory) / "inputs.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "label": "invalid",
                        "ground_truth_consumed": False,
                        "roles": {
                            "rgb": {
                                "contract": "rgb",
                                "status": "complete",
                                "provenance": "genuine",
                                "capture_id": "invalid",
                                "artifacts": {
                                    "view_video": str(reference),
                                    "frame_index": str(reference),
                                    "camera_info": str(reference),
                                },
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "storyboard references cannot be presentation inputs"):
                inspect_inputs(plan, manifest)

    def test_diagnostic_mode_cannot_write_delivery_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "restricted to the 1280x720 preview"):
                render_presentation(PLAN, BASELINE_INPUTS, directory, "delivery", "diagnostic", "ffmpeg", "ffprobe")


if __name__ == "__main__":
    unittest.main()
