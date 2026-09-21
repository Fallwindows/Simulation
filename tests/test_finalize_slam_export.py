from __future__ import annotations

import csv
import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from simulator.presentation.provenance import _validate_slam_manifest


class FinalizeSlamExportTests(unittest.TestCase):
    def _fixture(self, root: Path, *, empty_cloud: bool = False) -> tuple[Path, ...]:
        output = root / "output"
        native = output / "native_export"
        native.mkdir(parents=True)
        poses = native / "slam_map_poses.txt"
        poses.write_text(
            "#timestamp x y z qx qy qz qw\n"
            "0.2 0 0 0 0 0 0 1\n"
            "1.3 1.1 -0.03 0.01 0 0 0.01 0.999949999\n",
            encoding="ascii",
        )
        count = 0 if empty_cloud else 2
        vertices = "" if empty_cloud else "1 2 3 8\n4 5 6 9\n"
        cloud = native / "slam_map_cloud.ply"
        cloud.write_text(
            "ply\nformat ascii 1.0\ncomment fixture\n"
            f"element vertex {count}\nproperty float x\nproperty float y\nproperty float z\n"
            "property float intensity\nelement camera 1\nproperty float view_px\nend_header\n"
            f"{vertices}0\n",
            encoding="ascii",
        )
        database = output / "rtabmap.db"
        database.write_bytes(b"fresh rtabmap database fixture")
        version = root / "version.log"
        version.write_text("RTAB-Map:              0.23.11\nPCL: 1.15.1\n", encoding="utf-8")
        commands = root / "commands.json"
        commands.write_text(
            json.dumps(
                {
                    "optimize": [
                        "rtabmap-export.exe", "--poses", "--poses_format", "10", "--opt", "0", "--save_in_db",
                        "--output", "slam_optimized", "--output_dir", str(native), str(database),
                    ],
                    "export": [
                        "rtabmap-export.exe", "--cloud", "--scan", "--poses", "--poses_format", "10", "--ascii",
                        "--opt", "2", "--max_range", "100", "--voxel", "0.03", "--output", "slam_map",
                        "--output_dir", str(native), str(database),
                    ],
                }
            ),
            encoding="utf-8",
        )
        return poses, cloud, database, version, commands

    def _run(self, root: Path, *, empty_cloud: bool = False) -> subprocess.CompletedProcess[str]:
        poses, cloud, database, version, commands = self._fixture(root, empty_cloud=empty_cloud)
        return subprocess.run(
            [
                sys.executable,
                "-m",
                "simulator.capture.finalize_slam_export",
                "--poses",
                str(poses),
                "--cloud",
                str(cloud),
                "--database",
                str(database),
                "--output-dir",
                str(root / "output"),
                "--exporter-version-log",
                str(version),
                "--commands-json",
                str(commands),
            ],
            cwd=Path(__file__).resolve().parents[1],
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_cli_converts_and_validates_native_export_with_explicit_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            completed = self._run(root)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = root / "output"
            producer = json.loads((output / "slam_producer.json").read_text(encoding="utf-8"))
            self.assertEqual(producer["producer_mode"], "rtabmap_database_export")
            self.assertEqual(producer["pose_count"], 2)
            self.assertEqual(producer["map_point_count"], 2)
            self.assertFalse(producer["ground_truth_subscribed"])
            self.assertFalse(producer["ground_truth_consumed"])
            self.assertEqual(producer["tool"], {"name": "rtabmap-export", "version": "0.23.11"})
            with (output / "slam_poses.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([row["timestamp_s"] for row in rows], ["0.2", "1.3"])
            self.assertEqual((output / "slam_map.ply").read_text(encoding="ascii").splitlines()[-2:], ["1 2 3", "4 5 6"])
            pcd = (output / "slam_map.pcd").read_text(encoding="ascii")
            self.assertIn("POINTS 2\n", pcd)

            manifest = {
                "status": "complete",
                "capture_id": "fixture-capture",
                "capture_sha256": "a" * 64,
                "producer_mode": "rtabmap_database_export",
                "producer": producer,
            }
            manifest_path = output / "slam_manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            _validate_slam_manifest(manifest_path)

            attacks = {
                "forged hashes": lambda value: (
                    value["database"].update(sha256="0" * 64),
                    [artifact.update(sha256="0" * 64) for artifact in value["artifacts"].values()],
                ),
                "wrong optimization": lambda value: value["optimization"]["command"].__setitem__(5, "99"),
                "wrong map export": lambda value: value["map_export"]["command"].__setitem__(8, "0"),
            }
            for name, mutate in attacks.items():
                with self.subTest(name=name):
                    forged = copy.deepcopy(producer)
                    mutate(forged)
                    (output / "slam_producer.json").write_text(json.dumps(forged), encoding="utf-8")
                    forged_manifest = dict(manifest, producer=forged)
                    manifest_path.write_text(json.dumps(forged_manifest), encoding="utf-8")
                    with self.assertRaises(ValueError):
                        _validate_slam_manifest(manifest_path)

            unknown = {
                "status": "complete",
                "capture_id": "fixture-capture",
                "capture_sha256": "a" * 64,
                "producer_mode": "unsupported_forged_mode",
                "observer": {
                    "fresh_odom": True,
                    "fresh_map": True,
                    "ground_truth_subscribed": False,
                    "map_point_count": 1,
                    "odom_sample_count": 2,
                },
            }
            manifest_path.write_text(json.dumps(unknown), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported explicit SLAM producer mode"):
                _validate_slam_manifest(manifest_path)

    def test_cli_rejects_empty_cloud_without_writing_success_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            completed = self._run(root, empty_cloud=True)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("native cloud has no vertices", completed.stderr)
            self.assertFalse((root / "output" / "slam_producer.json").exists())


if __name__ == "__main__":
    unittest.main()
