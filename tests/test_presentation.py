import hashlib
import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from simulator.capture.export_metadata import export_metadata
from simulator.presentation.rgb_bundle import emit_rgb_bundle
from simulator.presentation.render_video import plan_segments, render_presentation
from simulator.presentation.provenance import DIAGNOSTIC_BASELINE_IDENTITY, ROLE_SPECS, schema_catalog
from simulator.presentation.timeline import EXPECTED_SHOT_BOUNDARIES, inspect_inputs, load_plan


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config" / "presentation" / "storyboard.yaml"
BASELINE_INPUTS = ROOT / "config" / "presentation" / "diagnostic_baseline_inputs.json"
BASELINE_VIDEO = ROOT / "demo" / "walking_aisle_final_hifi.mp4"
SCENARIO = ROOT / "config" / "scenarios" / "baseline_straight.yaml"
FFMPEG = Path(r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe")
FFPROBE = FFMPEG.with_name("ffprobe.exe")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _descriptor(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": _sha256(path)}


def _write_configured_camera_info_from_export(capture: Path) -> None:
    """Compatibility for the pre-recorder branch; the accepted base writes this directly."""

    path = capture / "camera_info.json"
    if path.exists():
        return
    transforms = json.loads((capture / "sensor_transforms.json").read_text(encoding="utf-8"))
    value = {
        "schema_version": 1,
        "provenance": "configured_intrinsics",
        "observed_ros_message": False,
        "source": "sensor_transforms.json",
        "topic": transforms["topics"]["rgb_camera_info"],
        "frame_id": transforms["frames"]["camera_optical"],
        "model": "ideal_pinhole",
        **transforms["intrinsics"],
    }
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


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
        "producer_source_sha256": _sha256(ROOT / spec.producer_source_path),
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
        self.assertTrue(catalog["roles"]["rgb"]["view_producer_available"])
        for role in ("lidar", "map", "reconstruction"):
            self.assertFalse(catalog["roles"][role]["view_producer_available"])

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
        self.assertEqual(_sha256(BASELINE_VIDEO), DIAGNOSTIC_BASELINE_IDENTITY["video_sha256"])
        with self.assertRaises(ValueError):
            plan_segments(plan, report, "complete")

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
        reference = ROOT / "references" / "storyboard" / "01_enter_aisle_rgb.png"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
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
                inspect_inputs(load_plan(PLAN), manifest)
            with self.assertRaisesRegex(ValueError, "canonical immutable input manifest"):
                render_presentation(PLAN, manifest, root / "output", "preview", "diagnostic", "unreachable", "unreachable")

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
                        "producer_source_sha256": _sha256(ROOT / ROLE_SPECS["rgb"].view_producer_source_path),
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
        plan = load_plan(PLAN)
        reference = ROOT / "references" / "storyboard" / "01_enter_aisle_rgb.png"
        with tempfile.TemporaryDirectory() as directory:
            copied_reference = Path(directory) / "unrelated-looking-video.mp4"
            shutil.copyfile(reference, copied_reference)
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

    def test_rgb_bundle_emitter_accepts_canonical_capture_outputs_end_to_end(self):
        self.assertTrue(FFMPEG.is_file())
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "run"
            capture = run / "capture"
            capture.mkdir(parents=True)
            export_metadata(SCENARIO, capture, ROOT)
            _write_configured_camera_info_from_export(capture)

            video = capture / "rgb_camera.mp4"
            subprocess.run(
                [
                    str(FFMPEG), "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
                    "color=c=black:s=1280x720:r=30", "-frames:v", "1350", "-c:v", "libx264",
                    "-preset", "ultrafast", "-pix_fmt", "yuv420p", "-y", str(video),
                ],
                check=True,
            )
            frames = capture / "rgb_frames.jsonl"
            with frames.open("w", encoding="utf-8", newline="\n") as handle:
                for frame_index in range(1350):
                    handle.write(
                        json.dumps(
                            {
                                "frame_index": frame_index,
                                "stamp_s": frame_index / 30.0,
                                "frame_id": "camera_optical_frame",
                                "width": 1280,
                                "height": 720,
                                "encoding": "rgb8",
                            },
                            separators=(",", ":"),
                            sort_keys=True,
                        )
                        + "\n"
                    )
            metadata = capture / "rgb_video.json"
            metadata.write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "frame_count": 1350,
                        "first_image_stamp_s": 0.0,
                        "last_image_stamp_s": 1349 / 30.0,
                        "nominal_fps": 30.0,
                        "width": 1280,
                        "height": 720,
                    }
                ),
                encoding="utf-8",
            )
            files = []
            for path in sorted(item for item in capture.rglob("*") if item.is_file()):
                files.append(
                    {
                        "path": path.relative_to(capture).as_posix(),
                        "sha256": _sha256(path),
                        "size_bytes": path.stat().st_size,
                    }
                )
            capture_manifest = capture / "capture_manifest.json"
            capture_manifest.write_text(
                json.dumps(
                    {
                        "manifest_version": 1,
                        "status": "complete",
                        "capture_id": "emitter-fixture",
                        "duration_s": 45.0,
                        "capture_sha256": "a" * 64,
                        "bag": {
                            "uri": "sensors_bag",
                            "topics": [
                                "/clock", "/sim/camera/rgb/image_raw", "/sim/camera/rgb/camera_info",
                                "/sim/lidar/points", "/tf", "/tf_static",
                            ],
                        },
                        "rgb": {
                            "video": video.name,
                            "timestamp_index": frames.name,
                            "camera_info": "camera_info.json",
                            "camera_info_provenance": "configured_intrinsics",
                            "metadata": metadata.name,
                            "frame_count": 1350,
                            "first_stamp_s": 0.0,
                            "last_stamp_s": 1349 / 30.0,
                        },
                        "files": files,
                    }
                ),
                encoding="utf-8",
            )
            inputs = run / "presentation_rgb_inputs.json"
            emit_rgb_bundle(capture_manifest, inputs, repo_root=ROOT, ffprobe=str(FFPROBE))
            report = inspect_inputs(load_plan(PLAN), inputs)
            self.assertEqual(report.ready_genuine_roles, frozenset({"rgb"}))
            self.assertEqual(report.role_errors["rgb"], ())
            receipt = json.loads((capture / "rgb_presentation_view_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(receipt["producer_id"], "simulator.presentation.rgb_bundle_emitter.v1")
            self.assertEqual(receipt["output_video_sha256"], _sha256(video))
            self.assertEqual(set(receipt["source_artifact_sha256"]), {"frame_index", "camera_info", "rgb_metadata", "capture_manifest"})

    def test_diagnostic_mode_cannot_write_delivery_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "restricted to the 1280x720 preview"):
                render_presentation(PLAN, BASELINE_INPUTS, directory, "delivery", "diagnostic", "ffmpeg", "ffprobe")

    def test_diagnostic_output_claim_is_limited_to_the_pinned_source_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = render_presentation(
                PLAN,
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


if __name__ == "__main__":
    unittest.main()
