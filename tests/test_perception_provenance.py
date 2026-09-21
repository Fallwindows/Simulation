import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from simulator.perception.provenance import (
    build_perception_input_bindings,
    validate_perception_frame_coverage,
    validate_perception_manifest_bindings,
)
from simulator.perception.rgb_tracking import run_rgb_tracking
from simulator.capture.manifest import capture_hash, sha256_file, write_json
from simulator.presentation.provenance import Artifact, _role_associations, _validate_perception_manifest
from tests.perception_provenance_fixture import create_perception_run, refresh_perception_manifest


class PerceptionProvenanceTests(unittest.TestCase):
    def test_bag_layout_accepts_root_storage_and_rejects_nested_relocation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            valid = create_perception_run(root / "valid", frame_count=540, capture_id="root-bag")
            binding = build_perception_input_bindings(valid["capture"], valid["slam"], valid["perception"])
            self.assertEqual(
                [item["path"] for item in binding["raw_lidar"]["bag"]["files"]],
                ["../capture/sensors_bag/capture_0.db3", "../capture/sensors_bag/metadata.yaml"],
            )

            fixture = create_perception_run(root / "nested", frame_count=540, capture_id="nested-bag")
            capture = Path(fixture["capture"])
            nested = capture / "sensors_bag/nested"
            nested.mkdir()
            for name in ("capture_0.db3", "metadata.yaml"):
                (capture / "sensors_bag" / name).replace(nested / name)
            capture_manifest_path = capture / "capture_manifest.json"
            capture_manifest = json.loads(capture_manifest_path.read_text(encoding="utf-8"))
            for item in capture_manifest["files"]:
                if item["path"].startswith("sensors_bag/"):
                    name = Path(item["path"]).name
                    relocated = nested / name
                    item.update(
                        path=f"sensors_bag/nested/{name}",
                        sha256=sha256_file(relocated),
                        size_bytes=relocated.stat().st_size,
                    )
            capture_manifest["capture_sha256"] = capture_hash(capture_manifest)
            write_json(capture_manifest_path, capture_manifest)
            slam_manifest_path = Path(fixture["slam"]) / "slam_manifest.json"
            slam_manifest = json.loads(slam_manifest_path.read_text(encoding="utf-8"))
            slam_manifest["capture_sha256"] = capture_manifest["capture_sha256"]
            write_json(slam_manifest_path, slam_manifest)
            with self.assertRaisesRegex(ValueError, "cannot contain nested storage files"):
                build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])

    def test_preflight_ignores_removed_and_mutated_evaluation_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = create_perception_run(Path(temporary), frame_count=540)
            before = build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])
            truth_paths = [
                Path(fixture["capture"]) / "inventory_ground_truth.csv",
                Path(fixture["capture"]) / "inventory_ground_truth.json",
            ]
            for path in truth_paths:
                path.write_bytes(b"mutated evaluation truth that is not an estimator input")
            after_mutation = build_perception_input_bindings(
                fixture["capture"], fixture["slam"], fixture["perception"]
            )
            self.assertEqual(after_mutation, before)
            for path in truth_paths:
                path.unlink()
            after_removal = build_perception_input_bindings(
                fixture["capture"], fixture["slam"], fixture["perception"]
            )
            self.assertEqual(after_removal, before)

    def test_meaningful_rgb_estimator_succeeds_when_truth_access_is_denied(self):
        import numpy as np

        with tempfile.TemporaryDirectory() as temporary:
            fixture = create_perception_run(
                Path(temporary), frame_count=540, capture_id="denied-truth", decodable_video=True
            )
            original_open = Path.open
            attempted_truth_access: list[str] = []

            def deny_truth(path: Path, *args, **kwargs):
                if path.name in {"inventory_ground_truth.csv", "inventory_ground_truth.json"}:
                    attempted_truth_access.append(path.name)
                    raise PermissionError(f"evaluation truth denied: {path}")
                return original_open(path, *args, **kwargs)

            scans = [
                (stamp, np.asarray([
                    [-0.04, -0.04, 2.0], [0.0, -0.04, 2.0], [0.04, -0.04, 2.0],
                    [-0.04, 0.04, 2.0], [0.0, 0.04, 2.0], [0.04, 0.04, 2.0],
                ], dtype=np.float64))
                for stamp in (0.0, 4.0, 8.0, 12.0, 16.0)
            ]
            with mock.patch.object(Path, "open", new=deny_truth), mock.patch(
                "simulator.perception.rgb_tracking._read_lidar_scans", return_value=iter(scans)
            ):
                result = run_rgb_tracking(
                    fixture["capture"], fixture["slam"], fixture["perception"], Path(__file__).resolve().parents[1]
                )
            self.assertEqual(attempted_truth_access, [])
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["frame_count"], 540)
            self.assertGreater(result["detection_count"], 0)
            self.assertGreater(result["track_count"], 0)
            self.assertFalse(result["ground_truth_consumed"])

    def test_offline_launcher_preflights_exact_inputs_before_creating_output(self):
        script = (Path(__file__).resolve().parents[1] / "scripts/run_inventory_offline.ps1").read_text(encoding="utf-8")
        preflight = script.index('"--validate-inputs-only"')
        create_output = script.index("New-Item -ItemType Directory -Force -Path $perception")
        actual_run = script.index("$args = @")
        self.assertLess(preflight, create_output)
        self.assertLess(create_output, actual_run)

    def test_613_contiguous_frames_and_exact_inputs_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = create_perception_run(Path(temporary), frame_count=613)
            observed = build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])
            manifest = fixture["perception_manifest"]
            self.assertEqual(observed, manifest["inputs"])
            self.assertNotIn("slam_artifact", manifest)
            self.assertFalse(manifest["ground_truth_consumed"])
            validate_perception_manifest_bindings(manifest, fixture["capture"], fixture["slam"], fixture["perception"])
            coverage = validate_perception_frame_coverage(
                manifest, fixture["capture"], Path(fixture["perception"]) / "frame_annotations.jsonl"
            )
            self.assertEqual((coverage["frame_count"], coverage["minimum_required_frames"]), (613, 540))
            _validate_perception_manifest(Path(fixture["perception"]) / "perception_manifest.json")

    def test_manifest_rejects_mixed_capture_and_slam_hashes(self):
        fields = ("capture_id", "capture_sha256", "capture_manifest_sha256", "slam_manifest_sha256")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index, field in enumerate(fields):
                with self.subTest(field=field):
                    fixture = create_perception_run(root / str(index), capture_id=f"capture-{index}")
                    manifest = copy.deepcopy(fixture["perception_manifest"])
                    manifest[field] = "other-run" if field == "capture_id" else "f" * 64
                    with self.assertRaisesRegex(ValueError, "perception .*match"):
                        validate_perception_manifest_bindings(
                            manifest, fixture["capture"], fixture["slam"], fixture["perception"]
                        )

    def test_rejects_cross_run_trajectory_and_raw_bag_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = create_perception_run(root / "trajectory", capture_id="capture-a")
            (Path(fixture["slam"]) / "slam_poses.csv").write_text("cross-run\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "trajectory bytes"):
                validate_perception_manifest_bindings(
                    fixture["perception_manifest"], fixture["capture"], fixture["slam"], fixture["perception"]
                )

    def test_missing_or_tampered_consumed_sensor_inputs_fail_closed(self):
        consumed = (
            "rgb_camera.mp4",
            "rgb_frames.jsonl",
            "rgb_video.json",
            "sensor_transforms.json",
            "sensors_bag/capture_0.db3",
            "sensors_bag/metadata.yaml",
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case = 0
            for relative in consumed:
                for attack in ("missing", "tampered"):
                    with self.subTest(relative=relative, attack=attack):
                        fixture = create_perception_run(
                            root / str(case), frame_count=540, capture_id=f"sensor-{case}"
                        )
                        case += 1
                        path = Path(fixture["capture"]) / relative
                        if attack == "missing":
                            path.unlink()
                        else:
                            data = path.read_bytes()
                            path.write_bytes((b"X" + data[1:]) if data else b"X")
                        with self.assertRaisesRegex(ValueError, "(missing|required|mismatch|storage|do not exactly match)"):
                            build_perception_input_bindings(
                                fixture["capture"], fixture["slam"], fixture["perception"]
                            )

            fixture = create_perception_run(root / "capture-manifest", frame_count=540, capture_id="capture-manifest")
            capture_manifest_path = Path(fixture["capture"]) / "capture_manifest.json"
            capture_payload = json.loads(capture_manifest_path.read_text(encoding="utf-8"))
            capture_payload["duration_s"] = 1.0
            capture_manifest_path.write_text(json.dumps(capture_payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "capture manifest hash mismatch"):
                build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])

            fixture = create_perception_run(root / "slam-manifest", frame_count=540, capture_id="slam-manifest")
            slam_manifest_path = Path(fixture["slam"]) / "slam_manifest.json"
            slam_payload = json.loads(slam_manifest_path.read_text(encoding="utf-8"))
            slam_payload["capture_sha256"] = "f" * 64
            slam_manifest_path.write_text(json.dumps(slam_payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SLAM manifest capture_sha256"):
                build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])

            fixture = create_perception_run(root / "missing-slam", frame_count=540, capture_id="missing-slam")
            (Path(fixture["slam"]) / "slam_manifest.json").unlink()
            with self.assertRaisesRegex(ValueError, "requires capture_manifest.json and slam_manifest.json"):
                build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])
            fixture = create_perception_run(root / "unlisted-bag", frame_count=540, capture_id="unlisted-bag")
            (Path(fixture["capture"]) / "sensors_bag/unlisted_1.db3").write_bytes(b"unlisted bag bytes")
            with self.assertRaisesRegex(ValueError, "bag files do not exactly match"):
                build_perception_input_bindings(fixture["capture"], fixture["slam"], fixture["perception"])
            fixture = create_perception_run(root / "bag", capture_id="capture-b")
            (Path(fixture["capture"]) / "sensors_bag/capture_0.db3").write_bytes(b"cross-run")
            with self.assertRaisesRegex(ValueError, "(size|hash) mismatch"):
                validate_perception_manifest_bindings(
                    fixture["perception_manifest"], fixture["capture"], fixture["slam"], fixture["perception"]
                )

    def test_rejects_short_truncated_and_mismatched_frame_contracts(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            short = create_perception_run(root / "short", frame_count=539, capture_id="short")
            with self.assertRaisesRegex(ValueError, "shorter"):
                validate_perception_frame_coverage(
                    short["perception_manifest"], short["capture"], Path(short["perception"]) / "frame_annotations.jsonl"
                )
            fixture = create_perception_run(root / "truncated", frame_count=613, capture_id="truncated")
            annotations = Path(fixture["perception"]) / "frame_annotations.jsonl"
            lines = annotations.read_text(encoding="utf-8").splitlines()
            annotations.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly match"):
                validate_perception_frame_coverage(fixture["perception_manifest"], fixture["capture"], annotations)
            fixture = create_perception_run(root / "stamp", frame_count=613, capture_id="stamp")
            annotations = Path(fixture["perception"]) / "frame_annotations.jsonl"
            rows = [json.loads(line) for line in annotations.read_text(encoding="utf-8").splitlines()]
            rows[400]["stamp_s"] += 0.01
            annotations.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "timestamps"):
                validate_perception_frame_coverage(fixture["perception_manifest"], fixture["capture"], annotations)

    def test_presentation_manifest_gate_rejects_legacy_1350_assumption_and_missing_binding(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = create_perception_run(Path(temporary), frame_count=540)
            path = Path(fixture["perception"]) / "perception_manifest.json"
            _validate_perception_manifest(path)
            payload = copy.deepcopy(fixture["perception_manifest"])
            del payload["inputs"]
            refresh_perception_manifest(fixture, payload)
            with self.assertRaisesRegex(ValueError, "exact capture and SLAM"):
                _validate_perception_manifest(path)

    def test_presentation_reconstruction_checks_exact_run_and_frame_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = create_perception_run(Path(temporary), frame_count=613)
            capture_manifest = Path(fixture["capture"]) / "capture_manifest.json"
            slam_manifest = Path(fixture["slam"]) / "slam_manifest.json"
            perception_manifest = Path(fixture["perception"]) / "perception_manifest.json"
            inventory = Path(fixture["perception"]) / "estimated_inventory.csv"
            annotations = Path(fixture["perception"]) / "frame_annotations.jsonl"
            artifacts = {
                "capture_manifest": Artifact("capture_manifest", capture_manifest, sha256_file(capture_manifest), "capture_manifest"),
                "slam_manifest": Artifact("slam_manifest", slam_manifest, sha256_file(slam_manifest), "slam_manifest"),
                "perception_manifest": Artifact("perception_manifest", perception_manifest, sha256_file(perception_manifest), "perception_manifest"),
                "object_records": Artifact("object_records", inventory, sha256_file(inventory), "object_records"),
                "observation_links": Artifact("observation_links", annotations, sha256_file(annotations), "observation_links"),
            }
            item = {
                "capture_id": fixture["capture_manifest"]["capture_id"],
                "map_version": artifacts["slam_manifest"].sha256,
                "object_state_version": artifacts["perception_manifest"].sha256,
            }
            _role_associations("reconstruction", item, artifacts)
            lines = annotations.read_text(encoding="utf-8").splitlines()
            annotations.write_text("\n".join(lines[:-1]) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly match"):
                _role_associations("reconstruction", item, artifacts)


if __name__ == "__main__":
    unittest.main()
