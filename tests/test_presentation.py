import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simulator.presentation.complete_bundle import emit_complete_bundle
from simulator.presentation.render_video import PlannedSegment, _build_filter, plan_segments, render_presentation
from simulator.presentation.provenance import (
    DIAGNOSTIC_BASELINE_IDENTITY,
    PRESENTATION_TRANSFORM_HFLIP,
    PRESENTATION_TRANSFORM_NONE,
    ROLE_SPECS,
    schema_catalog,
    sha256_path,
    source_text_sha256,
    validate_rgb_capture_acceptance,
    validate_presentation_transform,
)
from simulator.presentation.technical_bundle import validate_technical_delivery
from simulator.presentation.timeline import (
    EXPECTED_SHOT_BOUNDARIES,
    SMOOTH_TRANSITION_BOUNDARIES,
    inspect_inputs,
    load_plan,
)
from tests.presentation_fixture import build_complete_fixture


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config" / "presentation" / "storyboard.yaml"
LEGACY_DIAGNOSTIC_PLAN = ROOT / "config" / "presentation" / "diagnostic_storyboard_legacy.yaml"
BASELINE_INPUTS = ROOT / "config" / "presentation" / "diagnostic_baseline_inputs.json"
BASELINE_VIDEO = ROOT / "demo" / "walking_aisle_final_hifi.mp4"
FFMPEG = Path(r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe")
FFPROBE = FFMPEG.with_name("ffprobe.exe")
REFERENCE_SOURCE_COMMIT = "ab8ebbd51989342c3b1acb6b8b96cf947288cd55"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _descriptor(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": _sha256(path)}


def _materialize_reference(target: Path, relative: str = "references/storyboard/01_enter_aisle_rgb.png") -> Path:
    """Read an original storyboard blob from Git without adding the 12 PNGs to this bounded port."""

    result = subprocess.run(
        ["git", "show", f"{REFERENCE_SOURCE_COMMIT}:{relative}"],
        cwd=ROOT,
        capture_output=True,
        check=True,
    )
    target.write_bytes(result.stdout)
    return target


def _placeholder_role(root: Path, role_name: str, capture_id: str, view_video: Path) -> dict[str, object]:
    spec = ROLE_SPECS[role_name]
    artifacts = {}
    for artifact_name, kind in spec.required_artifacts:
        if artifact_name == "view_video":
            artifact = view_video
        elif kind == "rosbag2":
            artifact = root / f"{role_name}_{artifact_name}"
            artifact.mkdir(exist_ok=True)
            (artifact / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
            (artifact / "capture.db3").write_bytes(b"not a sqlite database")
            from simulator.presentation.provenance import sha256_path
            artifacts[artifact_name] = {"path": str(artifact), "sha256": sha256_path(artifact)}
            continue
        else:
            artifact = root / f"{role_name}_{artifact_name}.dat"
            artifact.write_text("arbitrary placeholder\n", encoding="utf-8")
        artifacts[artifact_name] = _descriptor(artifact)
    return {
        "contract": role_name,
        "status": "complete",
        "provenance": "genuine",
        "schema_id": spec.schema_id,
        "producer_id": spec.producer_id,
        "producer_source_sha256": source_text_sha256(ROOT / spec.producer_source_path),
        "capture_id": capture_id,
        "map_version": "0" * 64 if role_name in {"pose", "map", "reconstruction"} else None,
        "object_state_version": "1" * 64 if role_name == "reconstruction" else None,
        "artifacts": artifacts,
    }


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

    def test_transition_windows_cover_every_technical_boundary_without_changing_frame_budget(self):
        plan = load_plan(PLAN)
        self.assertEqual([item.boundary_frame for item in plan.transitions], list(SMOOTH_TRANSITION_BOUNDARIES))
        self.assertEqual([(item.from_shot, item.to_shot) for item in plan.transitions], [
            (5, 6), (6, 7), (7, 8), (8, 9), (9, 10), (10, 11), (11, 12),
        ])
        self.assertEqual([item.duration_frames for item in plan.transitions], [18, 18, 18, 12, 12, 12, 12])
        self.assertTrue(all(item.style == "smooth_crossfade" and item.easing == "smoothstep" for item in plan.transitions))
        self.assertTrue(all(item.start_frame < item.boundary_frame < item.end_frame_exclusive for item in plan.transitions))
        self.assertEqual(sum(shot.frame_count for shot in plan.shots), plan.frame_count)

        segments = tuple(
            PlannedSegment(shot, "missing_input_slate", None, None, 0, ("test-only",))
            for shot in plan.shots
        )
        inputs, graph = _build_filter(plan, "preview", segments)
        self.assertEqual(inputs, [])
        self.assertEqual(graph.count("xfade=transition=custom"), 7)
        self.assertEqual(graph.count("P*P*(3-2*P)"), 14)
        self.assertIn("offset=17.7", graph)
        self.assertIn("offset=21.7", graph)
        self.assertIn("offset=24.7", graph)
        self.assertIn("trim=start_frame=0:end_frame=1350", graph)

    def test_invalid_transition_contracts_fail_closed(self):
        source = json.loads(PLAN.read_text(encoding="utf-8"))
        attacks = {
            "all seven": lambda value: value["transition_policy"]["boundaries"].pop(),
            "short positive even duration": lambda value: value["transition_policy"]["boundaries"][0].update(duration_frames=17),
            "does not match its shot boundary": lambda value: value["transition_policy"]["boundaries"][0].update(from_shot=4, to_shot=5),
            "approved style": lambda value: value["transition_policy"]["boundaries"][0].update(style="hard_cut"),
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            for expected, mutate in attacks.items():
                with self.subTest(expected=expected):
                    candidate = json.loads(json.dumps(source))
                    mutate(candidate)
                    path.write_text(json.dumps(candidate), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, expected):
                        load_plan(path)

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

    def test_checked_in_role_schema_catalog_matches_validator(self):
        catalog = json.loads((ROOT / "config" / "presentation" / "input_schemas.json").read_text(encoding="utf-8"))
        self.assertEqual(catalog, schema_catalog())
        accepted = json.loads((ROOT / "config" / "presentation" / "accepted_rgb_captures.json").read_text(encoding="utf-8"))
        self.assertEqual(accepted["mechanism"], "reviewed_exact_capture_allowlist")
        self.assertEqual(len(accepted["captures"]), 1)
        production = accepted["captures"][0]
        self.assertEqual(production["capture_id"], "20260921-053814804")
        self.assertEqual(production["capture_sha256"], "ab84ccc1f71790f896bab1375062ed1db9f40bf9a1531d582f7cd0e589121eba")
        self.assertEqual(production["presentation_transform"], PRESENTATION_TRANSFORM_NONE)
        self.assertEqual(
            production["review"]["verdict_sha256"],
            "bbff6fbf052b70b14d9c1f7b0014fd2788bb92077db3e5f1ad8ba26e37e85c20",
        )
        with self.assertRaisesRegex(ValueError, "differs from the current repaired-base producer source"):
            validate_rgb_capture_acceptance(
                {
                    "capture_id": production["capture_id"],
                    "capture_sha256": production["capture_sha256"],
                    "git_sha": production["git_sha"],
                },
                production["required_artifact_sha256"],
                ROOT,
            )
        self.assertTrue(catalog["roles"]["rgb"]["view_producer_available"])
        for role in ("lidar", "map", "reconstruction"):
            self.assertFalse(catalog["roles"][role]["view_producer_available"])
        self.assertEqual(
            catalog["repaired_source_contract"],
            {
                "capture_manifest_version": 1,
                "capture_topics": [
                    "/clock",
                    "/sim/camera/rgb/image_raw",
                    "/sim/camera/rgb/camera_info",
                    "/sim/lidar/points",
                    "/tf",
                    "/tf_static",
                ],
                "camera_info_provenance": "observed_ros_message",
                "slam_trajectory": "slam/slam_map_poses.csv",
                "slam_trajectory_role": "dense_corrected_trajectory",
                "slam_frame_id": "map",
                "slam_optimized": True,
                "perception_slam_artifact": "slam/slam_map.pcd",
                "perception_ground_truth_consumed": False,
            },
        )

    def test_rgb_capture_review_evidence_is_checked_in_and_fail_closed(self):
        catalog_path = ROOT / "config" / "presentation" / "accepted_rgb_captures.json"
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
        production = catalog["captures"][0]
        review = production["review"]
        evidence = ROOT / review["verdict_path"]
        self.assertTrue(evidence.is_file())
        self.assertEqual(_sha256(evidence), review["verdict_sha256"])
        capture = {
            "capture_id": production["capture_id"],
            "capture_sha256": production["capture_sha256"],
            "git_sha": production["git_sha"],
        }

        def rejected(mutator, message: str) -> None:
            forged = json.loads(json.dumps(catalog))
            mutator(forged["captures"][0]["review"])
            with tempfile.TemporaryDirectory() as directory:
                forged_path = Path(directory) / "accepted_rgb_captures.json"
                forged_path.write_text(json.dumps(forged), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    validate_rgb_capture_acceptance(
                        capture,
                        production["required_artifact_sha256"],
                        ROOT,
                        catalog_path=forged_path,
                    )

        rejected(lambda record: record.update(verdict_sha256="f" * 64), "verdict hash")
        rejected(
            lambda record: record.update(verdict_path="review/evidence/missing-verdict.md"),
            "existing file inside the repository",
        )
        rejected(lambda record: record.update(verdict_path="../escaped-verdict.md"), "invalid verdict path")

    def test_existing_rgb_baseline_is_diagnostic_and_never_complete(self):
        plan = load_plan(LEGACY_DIAGNOSTIC_PLAN)
        report = inspect_inputs(plan, BASELINE_INPUTS)
        self.assertFalse(report.complete)
        self.assertEqual(report.ready_genuine_roles, frozenset())
        self.assertIn("rgb", report.missing_by_shot[1])
        segments = plan_segments(plan, report, "diagnostic")
        self.assertEqual([segment.kind for segment in segments[:5]], ["diagnostic_baseline"] * 5)
        self.assertEqual([segment.kind for segment in segments[5:]], ["missing_input_slate"] * 7)
        self.assertTrue(all("references/storyboard" not in str(segment.video_path) for segment in segments))
        self.assertEqual(_sha256(BASELINE_VIDEO), DIAGNOSTIC_BASELINE_IDENTITY["video_sha256"])
        with self.assertRaises(ValueError):
            plan_segments(plan, report, "complete")

        with self.assertRaisesRegex(ValueError, "canonical immutable presentation plan"):
            inspect_inputs(load_plan(PLAN), BASELINE_INPUTS)

    def test_pinned_diagnostic_blob_predates_the_storyboard_manifest(self):
        subprocess.run(
            [
                "git", "merge-base", "--is-ancestor",
                DIAGNOSTIC_BASELINE_IDENTITY["introduced_commit"],
                DIAGNOSTIC_BASELINE_IDENTITY["storyboard_manifest_commit"],
            ],
            cwd=ROOT,
            check=True,
        )
        blob = subprocess.check_output(
            [
                "git", "rev-parse",
                f"{DIAGNOSTIC_BASELINE_IDENTITY['introduced_commit']}:{DIAGNOSTIC_BASELINE_IDENTITY['video_path']}",
            ],
            cwd=ROOT,
            text=True,
        ).strip()
        self.assertEqual(blob, DIAGNOSTIC_BASELINE_IDENTITY["video_git_blob"])
        old_manifest = subprocess.run(
            [
                "git", "cat-file", "-e",
                f"{DIAGNOSTIC_BASELINE_IDENTITY['introduced_commit']}:references/manifest.json",
            ],
            cwd=ROOT,
            capture_output=True,
        )
        self.assertNotEqual(old_manifest.returncode, 0)
        subprocess.run(
            [
                "git", "cat-file", "-e",
                f"{DIAGNOSTIC_BASELINE_IDENTITY['storyboard_manifest_commit']}:references/manifest.json",
            ],
            cwd=ROOT,
            check=True,
        )

    def test_alternate_diagnostic_manifest_with_png_transcoded_to_h264_is_rejected_before_render(self):
        self.assertTrue(FFMPEG.is_file())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = _materialize_reference(root / "01_enter_aisle_rgb.png")
            transcoded = root / "apparently-new-baseline.mp4"
            subprocess.run(
                [
                    str(FFMPEG), "-hide_banner", "-loglevel", "error", "-loop", "1", "-i", str(reference),
                    "-frames:v", "3", "-r", "30", "-vf", "scale=1280:720", "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-y", str(transcoded),
                ],
                check=True,
            )
            self.assertNotEqual(_sha256(transcoded), _sha256(reference))
            self.assertIn(b"ftyp", transcoded.read_bytes()[:32])
            manifest = root / "alternate_diagnostic.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "label": "transcoded storyboard attack",
                        "ground_truth_consumed": False,
                        "roles": {
                            "rgb_baseline": {
                                "contract": "rgb",
                                "status": "complete",
                                "provenance": "diagnostic_baseline",
                                "schema_id": "simulation.presentation.diagnostic_baseline.v1",
                                "producer_id": "repository.demo.baseline.v1",
                                "capture_id": "forged",
                                "artifacts": {"view_video": _descriptor(transcoded)},
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "canonical immutable input manifest"):
                inspect_inputs(load_plan(LEGACY_DIAGNOSTIC_PLAN), manifest)
            with self.assertRaisesRegex(ValueError, "canonical immutable input manifest"):
                render_presentation(LEGACY_DIAGNOSTIC_PLAN, manifest, root / "output", "preview", "diagnostic", "unreachable", "unreachable")

    def test_arbitrary_placeholders_and_missing_hashes_cannot_be_complete(self):
        plan = load_plan(PLAN)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            roles = {}
            for name in plan.role_contracts:
                view = root / f"{name}.mp4"
                shutil.copyfile(BASELINE_VIDEO, view)
                with view.open("ab") as handle:
                    handle.write(name.encode("ascii"))
                roles[name] = _placeholder_role(root, name, "fixture-capture", view)
            manifest = root / "inputs.json"
            manifest.write_text(
                json.dumps({"schema_version": 2, "label": "adversarial", "ground_truth_consumed": False, "roles": roles}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            self.assertFalse(report.complete)
            self.assertEqual(report.ready_genuine_roles, frozenset())
            self.assertTrue(any("placeholder" in error or "must contain" in error or "not a" in error for errors in report.role_errors.values() for error in errors))
            with self.assertRaisesRegex(ValueError, "complete render inputs are unavailable"):
                render_presentation(PLAN, manifest, root / "output", "delivery", "complete", "unreachable", "unreachable")

            del roles["rgb"]["artifacts"]["frame_index"]["sha256"]
            manifest.write_text(
                json.dumps({"schema_version": 2, "label": "missing-hash", "ground_truth_consumed": False, "roles": roles}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            self.assertTrue(any("must declare a SHA-256" in error for error in report.role_errors["rgb"]))

    def test_cross_capture_and_reused_cross_role_video_are_rejected(self):
        plan = load_plan(PLAN)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            view = root / "reused.mp4"
            shutil.copyfile(BASELINE_VIDEO, view)
            roles = {
                "rgb": _placeholder_role(root, "rgb", "capture-a", view),
                "lidar": _placeholder_role(root, "lidar", "capture-b", view),
            }
            manifest = root / "inputs.json"
            manifest.write_text(
                json.dumps({"schema_version": 2, "label": "cross-role", "ground_truth_consumed": False, "roles": roles}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            for role in ("rgb", "lidar"):
                self.assertTrue(any("do not share one capture_id" in error for error in report.role_errors[role]))
                self.assertTrue(any("view video hash is reused across roles" in error for error in report.role_errors[role]))

    def test_view_manifest_must_bind_video_and_all_source_hashes(self):
        plan = load_plan(PLAN)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            view = root / "view.mp4"
            shutil.copyfile(BASELINE_VIDEO, view)
            role = _placeholder_role(root, "rgb", "capture-a", view)
            view_manifest_path = Path(role["artifacts"]["view_manifest"]["path"])
            view_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_id": "simulation.presentation.view_derivation.v1",
                        "producer_id": ROLE_SPECS["rgb"].view_producer_id,
                        "producer_source_sha256": source_text_sha256(ROOT / ROLE_SPECS["rgb"].view_producer_source_path),
                        "role": "rgb",
                        "capture_id": "capture-a",
                        "output_video_sha256": _sha256(view),
                        "source_artifact_sha256": {},
                        "source_time_range_s": [0, 45],
                        "map_version": None,
                        "object_state_version": None,
                    }
                ),
                encoding="utf-8",
            )
            role["artifacts"]["view_manifest"] = _descriptor(view_manifest_path)
            manifest = root / "inputs.json"
            manifest.write_text(
                json.dumps({"schema_version": 2, "label": "bad-link", "ground_truth_consumed": False, "roles": {"rgb": role}}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            self.assertTrue(any("source hashes do not match" in error for error in report.role_errors["rgb"]))

    def test_declared_role_and_view_producer_source_hashes_are_verified(self):
        plan = load_plan(PLAN)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            view = root / "view.mp4"
            shutil.copyfile(BASELINE_VIDEO, view)
            role = _placeholder_role(root, "rgb", "capture-a", view)
            role["producer_source_sha256"] = "0" * 64
            view_manifest_path = Path(role["artifacts"]["view_manifest"]["path"])
            view_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_id": "simulation.presentation.view_derivation.v1",
                        "producer_id": ROLE_SPECS["rgb"].view_producer_id,
                        "producer_source_sha256": "0" * 64,
                        "role": "rgb",
                        "capture_id": "capture-a",
                        "output_video_sha256": _sha256(view),
                        "source_artifact_sha256": {
                            name: value["sha256"]
                            for name, value in role["artifacts"].items()
                            if name not in {"view_video", "view_manifest"}
                        },
                        "source_time_range_s": [0, 45],
                        "map_version": None,
                        "object_state_version": None,
                    }
                ),
                encoding="utf-8",
            )
            role["artifacts"]["view_manifest"] = _descriptor(view_manifest_path)
            manifest = root / "inputs.json"
            manifest.write_text(
                json.dumps({"schema_version": 2, "label": "bad-producer", "ground_truth_consumed": False, "roles": {"rgb": role}}),
                encoding="utf-8",
            )
            report = inspect_inputs(plan, manifest)
            errors = report.role_errors["rgb"]
            self.assertTrue(any("producer source hash does not match" in error for error in errors))
            self.assertTrue(any("view manifest producer source hash mismatch" in error for error in errors))

    def test_alternate_diagnostic_manifest_is_rejected_even_for_byte_identical_storyboard_content(self):
        plan = load_plan(LEGACY_DIAGNOSTIC_PLAN)
        with tempfile.TemporaryDirectory() as directory:
            source_reference = _materialize_reference(Path(directory) / "01_enter_aisle_rgb.png")
            copied_reference = Path(directory) / "unrelated-looking-video.mp4"
            shutil.copyfile(source_reference, copied_reference)
            manifest = Path(directory) / "inputs.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "label": "invalid",
                        "ground_truth_consumed": False,
                        "roles": {
                            "rgb_baseline": {
                                "contract": "rgb",
                                "status": "complete",
                                "provenance": "diagnostic_baseline",
                                "schema_id": "simulation.presentation.diagnostic_baseline.v1",
                                "producer_id": "repository.demo.baseline.v1",
                                "capture_id": "invalid",
                                "artifacts": {
                                    "view_video": _descriptor(copied_reference),
                                },
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "canonical immutable input manifest"):
                inspect_inputs(plan, manifest)

    def test_diagnostic_mode_cannot_write_delivery_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "restricted to the 1280x720 preview"):
                render_presentation(LEGACY_DIAGNOSTIC_PLAN, BASELINE_INPUTS, directory, "delivery", "diagnostic", "ffmpeg", "ffprobe")

    def test_diagnostic_output_claim_is_limited_to_the_pinned_source_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = render_presentation(
                LEGACY_DIAGNOSTIC_PLAN,
                BASELINE_INPUTS,
                directory,
                "preview",
                "diagnostic",
                str(FFMPEG),
                str(FFPROBE),
            )
            self.assertNotIn("storyboard_pixels_consumed", manifest)
            self.assertEqual(manifest["diagnostic_source_identity"]["video_git_blob"], DIAGNOSTIC_BASELINE_IDENTITY["video_git_blob"])
            self.assertTrue(manifest["provenance_validation"]["exact_storyboard_file_hashes_rejected"])


class CompletePresentationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not FFMPEG.is_file() or not FFPROBE.is_file():
            raise unittest.SkipTest("trusted FFmpeg tools are unavailable")
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        cls.fixture = build_complete_fixture(cls.root / "fixture", ROOT, str(FFMPEG), str(FFPROBE))

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def _report(self, complete_inputs: Path | None = None, technical_catalog: Path | None = None):
        return inspect_inputs(
            load_plan(PLAN),
            complete_inputs or self.fixture["complete_inputs"],
            rgb_capture_catalog=self.fixture["rgb_catalog"],
            technical_source_catalog=technical_catalog or self.fixture["technical_catalog"],
            ffprobe=str(FFPROBE),
        )

    def test_repaired_producers_feed_the_1350_frame_presentation_contract(self):
        capture = json.loads(self.fixture["capture_manifest"].read_text(encoding="utf-8"))
        self.assertEqual(capture["manifest_version"], 1)
        self.assertEqual(capture["bag"]["topics"], [
            "/clock", "/sim/camera/rgb/image_raw", "/sim/camera/rgb/camera_info", "/sim/lidar/points", "/tf", "/tf_static",
        ])
        self.assertEqual(capture["rgb"]["frame_count"], 615)
        report = self._report()
        self.assertTrue(report.complete)
        segments = plan_segments(load_plan(PLAN), report, "complete")
        self.assertTrue(all(item.presentation_transform == PRESENTATION_TRANSFORM_HFLIP for item in segments[:5]))
        self.assertTrue(all(item.presentation_transform is None for item in segments[5:]))
        self.assertEqual([item.source_start_frame for item in segments[:5]], [0, 90, 180, 300, 420])
        self.assertEqual([item.source_start_frame for item in segments[5:]], [0] * 7)
        self.assertEqual([item.shot.frame_count for item in segments[5:]], [120, 90, 90, 120, 120, 120, 150])
        self.assertAlmostEqual(segments[4].source_time_range_s[1], 539 / 30.0)
        self.assertEqual(len({item.video_sha256 for item in segments[5:]}), 7)

    def test_real_renderer_receipts_round_trip_across_windows_crlf_checkout(self):
        from simulator.technical_views import canonical_text_sha256

        for path in (
            self.fixture["crlf_renderer"],
            self.fixture["crlf_plan"],
            self.fixture["technical_catalog"],
        ):
            self.assertIn(b"\r\n", path.read_bytes())
            self.assertNotEqual(sha256_path(path), canonical_text_sha256(path))
        delivery = validate_technical_delivery(
            self.fixture["technical_manifest"], ROOT, str(FFPROBE), self.fixture["technical_catalog"]
        )
        self.assertEqual(len(delivery.shots), 7)
        manifest = json.loads(self.fixture["technical_manifest"].read_text(encoding="utf-8"))
        self.assertEqual(manifest["renderer"]["sha256"], canonical_text_sha256(self.fixture["crlf_renderer"]))
        self.assertEqual(manifest["plan"]["sha256"], canonical_text_sha256(self.fixture["crlf_plan"]))
        self.assertEqual(
            manifest["source"]["artifacts"]["source_catalog"]["sha256"],
            canonical_text_sha256(self.fixture["technical_catalog"]),
        )

    def test_production_catalog_rejects_unlisted_fixture_bundle_by_default(self):
        catalog = json.loads(
            (ROOT / "config" / "presentation" / "accepted_rgb_captures.json").read_text(encoding="utf-8")
        )
        self.assertEqual([item["capture_id"] for item in catalog["captures"]], ["20260921-053814804"])
        with self.assertRaisesRegex(ValueError, "not present exactly once in the reviewed RGB capture allowlist"):
            inspect_inputs(load_plan(PLAN), self.fixture["complete_inputs"], ffprobe=str(FFPROBE))

    def test_rgb_timestamp_gap_is_rejected_before_render(self):
        from simulator.presentation.provenance import CONTENT_VALIDATORS

        source = self.fixture["capture_manifest"].parent / "rgb_frames.jsonl"
        rows = [json.loads(line) for line in source.read_text(encoding="utf-8").splitlines()]
        rows[300]["stamp_s"] += 0.02
        broken = self.root / "rgb_frames_with_gap.jsonl"
        broken.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "contiguous 30 fps"):
            CONTENT_VALIDATORS["rgb_frame_index"](broken)

    def test_technical_hash_revision_and_cross_source_attacks_are_rejected(self):
        technical = self.fixture["technical_manifest"]
        forged_hash = technical.with_name("forged_hash_manifest.json")
        value = json.loads(technical.read_text(encoding="utf-8"))
        value["outputs"][0]["sha256"] = "0" * 64
        forged_hash.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            validate_technical_delivery(
                forged_hash, ROOT, str(FFPROBE), self.fixture["technical_catalog"]
            )

        forged_revision = technical.with_name("forged_revision_manifest.json")
        value = json.loads(technical.read_text(encoding="utf-8"))
        value["source"]["producer_revisions"]["slam"] = "f" * 40
        forged_revision.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "producer revisions"):
            validate_technical_delivery(
                forged_revision, ROOT, str(FFPROBE), self.fixture["technical_catalog"]
            )

        catalog_value = json.loads(self.fixture["technical_catalog"].read_text(encoding="utf-8"))
        catalog_value["sources"][0]["capture_sha256"] = "e" * 64
        cross_catalog = self.root / "cross_source_catalog.json"
        cross_catalog.write_text(json.dumps(catalog_value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "source_catalog hash"):
            emit_complete_bundle(
                self.fixture["rgb_bundle"], technical, self.root / "cross_inputs.json", ROOT, str(FFPROBE),
                rgb_capture_catalog=self.fixture["rgb_catalog"], technical_source_catalog=cross_catalog,
            )

        complete_value = json.loads(self.fixture["complete_inputs"].read_text(encoding="utf-8"))
        complete_value["capture_id"] = "cross-source-substitution"
        forged_complete = self.fixture["complete_inputs"].with_name("forged_complete_inputs.json")
        forged_complete.write_text(json.dumps(complete_value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "capture identity is forged"):
            self._report(forged_complete)

        complete_value = json.loads(self.fixture["complete_inputs"].read_text(encoding="utf-8"))
        complete_value["presentation_classification"] = {"kind": "reviewed_production"}
        forged_classification = self.fixture["complete_inputs"].with_name("forged_classification_inputs.json")
        forged_classification.write_text(json.dumps(complete_value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "classification is forged"):
            self._report(forged_classification)

    def test_complete_preview_is_exact_and_applies_declared_rgb_hflip(self):
        output = self.root / "preview"
        mutated_inputs = self.fixture["complete_inputs"].with_name("label_mutated_complete_inputs.json")
        mutated_value = json.loads(self.fixture["complete_inputs"].read_text(encoding="utf-8"))
        mutated_value["label"] = "validated complete presentation inputs"
        mutated_inputs.write_text(json.dumps(mutated_value), encoding="utf-8")
        report = self._report(mutated_inputs)
        self.assertEqual(report.presentation_classification["kind"], "generated_test_fixture")
        manifest = render_presentation(
            PLAN,
            mutated_inputs,
            output,
            "preview",
            "complete",
            str(FFMPEG),
            str(FFPROBE),
            self.fixture["rgb_catalog"],
            self.fixture["technical_catalog"],
        )
        self.assertEqual(manifest["status"], "test_demonstration")
        self.assertIn("not production capture", manifest["claim"])
        self.assertEqual((manifest["probe"]["width"], manifest["probe"]["height"]), (1280, 720))
        self.assertEqual((manifest["probe"]["frame_count"], manifest["probe"]["audio_stream_count"]), (1350, 0))
        self.assertEqual(len(manifest["representative_frames"]), 36)
        self.assertEqual([item["boundary_frame"] for item in manifest["transitions"]], list(SMOOTH_TRANSITION_BOUNDARIES))
        self.assertTrue(all(item["frame_budget_delta"] == 0 for item in manifest["transitions"]))
        self.assertTrue(all(item["presentation_transform"] == PRESENTATION_TRANSFORM_HFLIP for item in manifest["shots"][:5]))
        self.assertTrue(all(item["presentation_transform"] is None for item in manifest["shots"][5:]))
        from PIL import Image

        frame = Image.open(output / manifest["representative_frames"][0]["path"])
        left = frame.getpixel((80, 360))
        right = frame.getpixel((1200, 360))
        self.assertGreater(left[2], left[0], "hflip must move the blue raw right half to presentation left")
        self.assertGreater(right[0], right[2], "hflip must move the red raw left half to presentation right")

        transition_dir = output / "transition_boundary_frames"
        transition_dir.mkdir()
        selected_frames = [frame for boundary in SMOOTH_TRANSITION_BOUNDARIES for frame in (boundary - 1, boundary, boundary + 1)]
        expression = "+".join(f"eq(n\\,{frame})" for frame in selected_frames)
        subprocess.run(
            [
                str(FFMPEG), "-hide_banner", "-loglevel", "error", "-i", str(output / manifest["video"]),
                "-vf", f"select='{expression}'", "-fps_mode", "vfr", "-y", str(transition_dir / "frame_%02d.png"),
            ],
            check=True,
        )
        from PIL import ImageChops, ImageStat

        selected = sorted(transition_dir.glob("frame_*.png"))
        self.assertEqual(len(selected), len(selected_frames))
        for index, boundary in enumerate(SMOOTH_TRANSITION_BOUNDARIES):
            trio = [Image.open(path).convert("RGB") for path in selected[index * 3:index * 3 + 3]]
            adjacent_rms = [
                max(ImageStat.Stat(ImageChops.difference(first, second)).rms)
                for first, second in zip(trio, trio[1:])
            ]
            self.assertLess(max(adjacent_rms), 48.0, f"frame {boundary} retains an abrupt generated-fixture cut")

    def test_reviewed_orientation_operations_drive_filters_and_reject_forgery(self):
        plan = load_plan(PLAN)
        shot = plan.shots[0]
        base = dict(
            shot=shot,
            kind="genuine",
            source_role="rgb_capture",
            video_path=Path("rgb.mp4"),
            source_start_frame=0,
            missing_roles=(),
        )
        _, no_flip_filter = _build_filter(
            plan, "preview", (PlannedSegment(**base, presentation_transform=PRESENTATION_TRANSFORM_NONE),)
        )
        _, flip_filter = _build_filter(
            plan, "preview", (PlannedSegment(**base, presentation_transform=PRESENTATION_TRANSFORM_HFLIP),)
        )
        self.assertNotIn("hflip", no_flip_filter)
        self.assertIn("hflip", flip_filter)
        for forged in (
            {**PRESENTATION_TRANSFORM_NONE, "reason": "label says no flip"},
            {"operation": "vflip", "reason": "label says so", "raw_capture_bytes_modified": False},
            {"operation": "none"},
        ):
            with self.subTest(forged=forged), self.assertRaisesRegex(ValueError, "exact supported reviewed transform"):
                validate_presentation_transform(forged)

        forged_complete = self.fixture["complete_inputs"].with_name("forged_transform_complete_inputs.json")
        complete_value = json.loads(self.fixture["complete_inputs"].read_text(encoding="utf-8"))
        complete_value["shots"][0]["presentation_transform"] = PRESENTATION_TRANSFORM_NONE
        forged_complete.write_text(json.dumps(complete_value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "shot mapping is forged"):
            self._report(forged_complete)

    def test_delivery_failures_leave_prior_generation_byte_exact_and_remove_staging(self):
        expected_probe = {
            "width": 1920, "height": 1080, "fps_num": 30, "fps_den": 1,
            "real_fps_num": 30, "real_fps_den": 1, "frame_count": 1350,
            "video_stream_count": 1, "audio_stream_count": 0,
            "video_codec": "ffv1", "duration_seconds": 45.0,
        }

        def seed_prior(target: Path) -> dict[str, bytes]:
            target.mkdir()
            (target / "presentation_delivery_manifest.json").write_text(
                '{"status":"complete","generation":"prior"}\n', encoding="utf-8"
            )
            (target / "presentation_final.mp4").write_bytes(b"prior-final")
            prior_frames = target / "presentation_delivery_representative_frames"
            prior_frames.mkdir()
            (prior_frames / "prior.png").write_bytes(b"prior-frame")
            return {
                path.relative_to(target).as_posix(): path.read_bytes()
                for path in target.rglob("*") if path.is_file()
            }

        def assert_prior_unchanged(target: Path, expected: dict[str, bytes]) -> None:
            observed = {
                path.relative_to(target).as_posix(): path.read_bytes()
                for path in target.rglob("*") if path.is_file()
            }
            self.assertEqual(observed, expected)
            leftovers = [
                path.name for path in target.parent.iterdir()
                if path.name.startswith(f".{target.name}.staging-")
                or path.name.startswith(f".{target.name}.previous-")
            ]
            self.assertEqual(leftovers, [])

        second_encode = self.root / "atomic_second_encode"
        prior = seed_prior(second_encode)
        calls = []

        def fail_second_run(command):
            calls.append(command)
            output = Path(command[-1])
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_bytes(b"new-partial")
            if len(calls) == 2:
                raise RuntimeError("injected second encode failure")

        with patch("simulator.presentation.render_video._run", fail_second_run), patch(
            "simulator.presentation.render_video._validate_output", return_value=expected_probe
        ):
            with self.assertRaisesRegex(RuntimeError, "second encode"):
                render_presentation(
                    PLAN, self.fixture["complete_inputs"], second_encode, "delivery", "complete",
                    str(FFMPEG), str(FFPROBE), self.fixture["rgb_catalog"], self.fixture["technical_catalog"],
                )
        assert_prior_unchanged(second_encode, prior)

        late_failure = self.root / "atomic_late_failure"
        prior = seed_prior(late_failure)

        def fake_run(command):
            output = Path(command[-1])
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_bytes(b"new-output")

        def fake_audio(path, _duration):
            Path(path).write_bytes(b"new-audio")

        def fake_contact(_video, path, _ffmpeg, _plan):
            Path(path).write_bytes(b"new-contact")

        with patch("simulator.presentation.render_video._run", fake_run), patch(
            "simulator.presentation.render_video._validate_output", return_value=expected_probe
        ), patch("simulator.presentation.render_video._write_deterministic_ambience", fake_audio), patch(
            "simulator.presentation.render_video._contact_sheet", fake_contact
        ), patch(
            "simulator.presentation.render_video._representative_frames",
            side_effect=RuntimeError("injected representative-frame failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "representative-frame"):
                render_presentation(
                    PLAN, self.fixture["complete_inputs"], late_failure, "delivery", "complete",
                    str(FFMPEG), str(FFPROBE), self.fixture["rgb_catalog"], self.fixture["technical_catalog"],
                )
        assert_prior_unchanged(late_failure, prior)

    def test_delivery_package_has_lossless_silent_audio_final_and_review_outputs(self):
        output = self.root / "delivery"
        manifest = render_presentation(
            PLAN,
            self.fixture["complete_inputs"],
            output,
            "delivery",
            "complete",
            str(FFMPEG),
            str(FFPROBE),
            self.fixture["rgb_catalog"],
            self.fixture["technical_catalog"],
        )
        outputs = manifest["outputs"]
        self.assertEqual(set(outputs), {"lossless_master", "silent_mp4", "audio_bed", "final_mp4", "review_mp4"})
        self.assertTrue(outputs["lossless_master"]["lossless"])
        self.assertEqual(outputs["lossless_master"]["probe"]["video_codec"], "ffv1")
        self.assertEqual(outputs["silent_mp4"]["probe"]["audio_stream_count"], 0)
        self.assertEqual(outputs["final_mp4"]["probe"]["audio_stream_count"], 1)
        self.assertEqual(outputs["review_mp4"]["probe"]["audio_stream_count"], 1)
        self.assertEqual((outputs["review_mp4"]["probe"]["width"], outputs["review_mp4"]["probe"]["height"]), (1280, 720))
        self.assertTrue(all(item["probe"]["frame_count"] == 1350 for name, item in outputs.items() if "probe" in item))


if __name__ == "__main__":
    unittest.main()
