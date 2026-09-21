import copy
import json
from pathlib import Path
import tempfile
import unittest

from simulator.perception.provenance import (
    build_perception_input_bindings,
    validate_perception_frame_coverage,
    validate_perception_manifest_bindings,
)
from simulator.capture.manifest import sha256_file
from simulator.presentation.provenance import Artifact, _role_associations, _validate_perception_manifest
from tests.perception_provenance_fixture import create_perception_run, refresh_perception_manifest


class PerceptionProvenanceTests(unittest.TestCase):
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
