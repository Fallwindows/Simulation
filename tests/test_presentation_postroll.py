from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
import unittest
from unittest import mock
import uuid

from simulator.presentation.complete_bundle import _rgb_sources
from simulator.presentation.provenance import (
    CONTENT_VALIDATORS,
    PRESENTATION_TRANSFORM_NONE,
    sha256_path,
)
from simulator.presentation.render_video import PlannedSegment, _build_filter, _transition_manifest
from simulator.presentation.timeline import load_plan


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "config" / "presentation" / "storyboard.yaml"
FFMPEG = os.environ.get(
    "SIM_FFMPEG",
    r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe",
)
FFPROBE = os.environ.get(
    "SIM_FFPROBE",
    r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe",
)


def _numbered_video(path: Path, frame_count: int) -> None:
    subprocess.run(
        [
            FFMPEG,
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "testsrc2=s=640x360:r=30",
            "-vf",
            (
                "drawtext=text='SOURCE FRAME %{n}':x=24:y=24:fontsize=28:"
                "fontcolor=white:box=1:boxcolor=black@0.7"
            ),
            "-frames:v",
            str(frame_count),
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-crf",
            "22",
            "-pix_fmt",
            "yuv420p",
            "-y",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _write_index(path: Path, frame_count: int) -> None:
    path.write_text(
        "".join(
            json.dumps(
                {
                    "frame_index": frame,
                    "stamp_s": 42.0 + frame / 30.0,
                    "frame_id": "camera_optical_frame",
                    "width": 640,
                    "height": 360,
                    "encoding": "rgb8",
                },
                sort_keys=True,
            )
            + "\n"
            for frame in range(frame_count)
        ),
        encoding="utf-8",
        newline="\n",
    )


class PresentationPostrollTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not Path(FFMPEG).is_file() or not Path(FFPROBE).is_file():
            raise unittest.SkipTest("the presentation post-roll test requires ffmpeg and ffprobe")
        cls.root = ROOT / "runs" / "presentation-postroll-tests" / uuid.uuid4().hex
        cls.root.mkdir(parents=True)
        cls.video = cls.root / "moving_numbered_rgb_000_549.mp4"
        cls.short_video = cls.root / "moving_numbered_rgb_000_547.mp4"
        _numbered_video(cls.video, 550)
        _numbered_video(cls.short_video, 548)

    @classmethod
    def tearDownClass(cls) -> None:
        shutil.rmtree(cls.root)

    def setUp(self) -> None:
        self.plan = load_plan(PLAN_PATH)
        self.case = self.root / self._testMethodName
        self.case.mkdir()

    def _role_report(self, video: Path, frame_count: int = 549):
        frame_index = self.case / "rgb_frames.jsonl"
        _write_index(frame_index, frame_count)
        receipt = self.case / "rgb_view_receipt.json"
        receipt.write_text(
            json.dumps(
                {
                    "presentation_transform": PRESENTATION_TRANSFORM_NONE,
                    "source_time_range_s": [42.0, 42.0 + (frame_count - 1) / 30.0],
                    "capture_acceptance": {
                        "presentation_classification": {"kind": "generated_test_fixture"}
                    },
                }
            ),
            encoding="utf-8",
        )
        capture = self.case / "capture_manifest.json"
        capture.write_text(json.dumps({"capture_sha256": "a" * 64}), encoding="utf-8")
        bundle = self.case / "rgb_bundle.json"
        bundle.write_text("{}\n", encoding="utf-8")
        artifacts = {
            "view_video": video,
            "frame_index": frame_index,
            "view_manifest": receipt,
            "capture_manifest": capture,
        }
        role = SimpleNamespace(
            artifacts=artifacts,
            artifact_sha256={name: sha256_path(path) for name, path in artifacts.items()},
            capture_id="postroll-test-capture",
        )
        report = SimpleNamespace(
            ready_genuine_roles=frozenset({"rgb"}),
            role_errors={"rgb": ()},
            roles={"rgb": role},
        )
        return report, bundle, role

    def _accepted_rgb_sources(self):
        report, bundle, role = self._role_report(self.video)
        with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
            result = _rgb_sources(self.plan, bundle, None, FFPROBE)
        return result, role

    def test_plan_declares_only_frame_540_as_moving_postroll(self):
        self.assertEqual((self.plan.fps, self.plan.frame_count, len(self.plan.shots)), (30, 1350, 12))
        self.assertEqual(len(self.plan.transitions), 7)
        first, *later = self.plan.transitions
        self.assertEqual((first.boundary_frame, first.start_frame, first.end_frame_exclusive), (540, 531, 549))
        self.assertEqual(first.outgoing_sampling.mode, "contiguous_postroll")
        self.assertEqual(
            (first.outgoing_sampling.source_frame_start, first.outgoing_sampling.source_frame_end_exclusive),
            (540, 549),
        )
        self.assertIn("1.13889 m/s", first.outgoing_sampling.reason or "")
        self.assertEqual(first.incoming_sampling.mode, "edge_clone")
        self.assertIn("no validated frame-level preroll", first.incoming_sampling.reason or "")
        self.assertTrue(
            all(
                transition.outgoing_sampling.mode == "edge_clone"
                and transition.incoming_sampling.mode == "edge_clone"
                for transition in later
            )
        )

    def test_plan_rejects_short_postroll_or_unvalidated_preroll(self):
        source = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
        for name, mutation, error in (
            (
                "short-postroll",
                lambda item: item["source_sampling"]["outgoing"].update(source_frame_end_exclusive=548),
                "must cover source frames 540-548",
            ),
            (
                "invented-preroll",
                lambda item: item["source_sampling"]["incoming"].update(mode="contiguous_preroll"),
                "incoming source sampling mode is unsupported",
            ),
        ):
            candidate = json.loads(json.dumps(source))
            mutation(candidate["transition_policy"]["boundaries"][0])
            path = self.case / f"{name}.json"
            path.write_text(json.dumps(candidate), encoding="utf-8")
            with self.subTest(name), self.assertRaisesRegex(ValueError, error):
                load_plan(path)

    def test_complete_bundle_binds_observed_transition_and_postroll_samples(self):
        (capture_id, _, shots, binding), _ = self._accepted_rgb_sources()
        self.assertEqual(capture_id, "postroll-test-capture")
        shot5 = shots[4]
        self.assertEqual(shot5.transition_boundary_frame, 540)
        self.assertEqual(shot5.source_time_range_s, (42.0 + 420 / 30.0, 42.0 + 548 / 30.0))
        self.assertEqual([frame for frame, _ in shot5.transition_source_samples], list(range(531, 549)))
        self.assertEqual(
            [stamp for _, stamp in shot5.transition_source_samples],
            [42.0 + frame / 30.0 for frame in range(531, 549)],
        )
        self.assertEqual(binding["required_source_frame_end_exclusive"], 549)
        self.assertEqual(
            [sample["source_frame"] for sample in binding["postroll"]["samples"]],
            list(range(540, 549)),
        )
        self.assertEqual(binding["frame_index_sha256"], sha256_path(self.case / "rgb_frames.jsonl"))
        self.assertEqual(binding["view_video_sha256"], sha256_path(self.video))

    def test_general_rgb_role_can_cover_540_but_complete_bundle_requires_549(self):
        report, bundle, _ = self._role_report(self.video, frame_count=540)
        CONTENT_VALIDATORS["rgb_frame_index"](self.case / "rgb_frames.jsonl")
        with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
            with self.assertRaisesRegex(ValueError, "0-548"):
                _rgb_sources(self.plan, bundle, None, FFPROBE)

    def test_complete_bundle_rejects_missing_short_and_mutated_rgb_sources(self):
        with self.subTest("missing index"):
            report, bundle, role = self._role_report(self.video)
            role.artifacts["frame_index"].unlink()
            with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
                with self.assertRaisesRegex(ValueError, "frame_index is missing"):
                    _rgb_sources(self.plan, bundle, None, FFPROBE)

        missing_case = self.case / "short-video"
        missing_case.mkdir()
        self.case = missing_case
        with self.subTest("short video"):
            report, bundle, _ = self._role_report(self.short_video)
            with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
                with self.assertRaisesRegex(ValueError, "must contain source frames 0-548"):
                    _rgb_sources(self.plan, bundle, None, FFPROBE)

        mutated_index_case = missing_case / "mutated-index"
        mutated_index_case.mkdir()
        self.case = mutated_index_case
        with self.subTest("mutated index"):
            report, bundle, role = self._role_report(self.video)
            with role.artifacts["frame_index"].open("a", encoding="utf-8") as handle:
                handle.write("\n")
            with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
                with self.assertRaisesRegex(ValueError, "frame_index changed"):
                    _rgb_sources(self.plan, bundle, None, FFPROBE)

        mutated_video_case = mutated_index_case / "mutated-video"
        mutated_video_case.mkdir()
        self.case = mutated_video_case
        copied_video = self.case / "mutated.mp4"
        copied_video.write_bytes(self.video.read_bytes())
        with self.subTest("mutated video"):
            report, bundle, role = self._role_report(copied_video)
            with copied_video.open("ab") as handle:
                handle.write(b"post-validation mutation")
            with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
                with self.assertRaisesRegex(ValueError, "view_video changed"):
                    _rgb_sources(self.plan, bundle, None, FFPROBE)

    def test_complete_bundle_rejects_noncontiguous_observed_postroll_stamp(self):
        report, bundle, role = self._role_report(self.video)
        index_path = role.artifacts["frame_index"]
        rows = [json.loads(line) for line in index_path.read_text(encoding="utf-8").splitlines()]
        rows[545]["stamp_s"] += 0.01
        index_path.write_text(
            "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
            encoding="utf-8",
            newline="\n",
        )
        role.artifact_sha256["frame_index"] = sha256_path(index_path)
        with mock.patch("simulator.presentation.timeline.inspect_inputs", return_value=report):
            with self.assertRaisesRegex(ValueError, "contiguous at 30 fps"):
                _rgb_sources(self.plan, bundle, None, FFPROBE)

    def test_filter_and_per_film_frame_manifest_use_moving_531_through_548(self):
        (_, _, rgb_sources, _), _ = self._accepted_rgb_sources()
        segments = []
        for shot, source in zip(self.plan.shots[:5], rgb_sources):
            segments.append(
                PlannedSegment(
                    shot,
                    "genuine",
                    source.source_kind,
                    source.video_path,
                    source.source_start_frame,
                    (),
                    source.source_time_range_s,
                    source.source_time_basis,
                    source.view_id,
                    source.video_sha256,
                    source.receipt_sha256,
                    source.presentation_transform,
                    source.transition_boundary_frame,
                    source.transition_source_samples,
                )
            )
        for shot in self.plan.shots[5:]:
            segments.append(
                PlannedSegment(
                    shot,
                    "genuine",
                    "technical_view",
                    self.video,
                    0,
                    (),
                    (0.2, 20.4),
                    "aggregate technical receipt simulation-time extent",
                    f"view-{shot.number}",
                    sha256_path(self.video),
                    "b" * 64,
                )
            )
        segments_tuple = tuple(segments)
        _, graph = _build_filter(self.plan, "preview", segments_tuple)
        shot5_chain = graph.split("[4:v]", 1)[1].split("[v05]", 1)[0]
        shot6_chain = graph.split("[5:v]", 1)[1].split("[v06]", 1)[0]
        self.assertIn("trim=start_frame=420:end_frame=549", shot5_chain)
        self.assertNotIn("stop_mode=clone", shot5_chain)
        self.assertIn("tpad=start_mode=clone:start=9", shot6_chain)

        transitions = _transition_manifest(self.plan, segments_tuple)
        self.assertEqual(len(transitions), 7)
        first = transitions[0]
        samples = first["weight_law"]["per_frame"]
        self.assertEqual([sample["film_frame"] for sample in samples], list(range(531, 549)))
        self.assertEqual(
            [sample["sources"]["outgoing"]["source_frame"] for sample in samples],
            list(range(531, 549)),
        )
        self.assertEqual(
            [sample["sources"]["outgoing"]["measurement_timestamp_s"] for sample in samples],
            [42.0 + frame / 30.0 for frame in range(531, 549)],
        )
        self.assertEqual(
            [sample["sources"]["incoming"]["source_frame"] for sample in samples[:9]],
            [0] * 9,
        )
        self.assertTrue(
            all(sample["sources"]["incoming"]["measurement_timestamp_s"] is None for sample in samples)
        )
        self.assertFalse(first["classification"]["co_timed"])
        self.assertFalse(first["classification"]["sensor_fusion"])
        self.assertFalse(first["classification"]["geometry_fusion"])
        self.assertEqual(
            [sample["sources"]["outgoing"]["source_frame"] for sample in transitions[1]["weight_law"]["per_frame"]],
            list(range(111, 120)) + [119] * 9,
        )
        self.assertTrue(
            all(
                abs(sample["outgoing_weight"] + sample["incoming_weight"] - 1.0) < 1e-12
                for transition in transitions
                for sample in transition["weight_law"]["per_frame"]
            )
        )


if __name__ == "__main__":
    unittest.main()
