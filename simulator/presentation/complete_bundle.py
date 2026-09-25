"""Combine one reviewed RGB bundle with reviewed technical views for all 12 shots."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .provenance import (
    RGB_TIMESTAMP_PERIOD_S,
    RGB_TIMESTAMP_TOLERANCE_S,
    sha256_path,
    source_text_sha256,
    validate_presentation_transform,
)
from .technical_bundle import ValidatedTechnicalDelivery, validate_technical_delivery


PRODUCER_ID = "simulator.presentation.complete_bundle_emitter.v1"


@dataclass(frozen=True)
class CompleteShotSource:
    shot_number: int
    video_path: Path
    video_sha256: str
    source_start_frame: int
    frame_count: int
    source_time_range_s: tuple[float, float]
    source_time_basis: str
    source_kind: str
    view_id: str
    receipt_path: Path
    receipt_sha256: str
    presentation_transform: dict[str, object] | None
    transition_boundary_frame: int | None
    transition_source_samples: tuple[tuple[int, float], ...]


@dataclass(frozen=True)
class ValidatedCompleteBundle:
    manifest_path: Path
    capture_id: str
    capture_sha256: str
    presentation_classification: dict[str, str]
    shots: tuple[CompleteShotSource, ...]
    source_bindings: dict[str, Any]


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON mapping")
    return value


def _descriptor(path: Path, manifest_path: Path) -> dict[str, str]:
    relative = os.path.relpath(path, manifest_path.parent).replace("\\", "/")
    return {"path": relative, "sha256": sha256_path(path)}


def _safe_descriptor(manifest_path: Path, descriptor: object, field: str) -> Path:
    if not isinstance(descriptor, dict) or set(descriptor) != {"path", "sha256"}:
        raise ValueError(f"{field} must be a path/SHA-256 descriptor")
    text = str(descriptor.get("path", ""))
    pure = PurePosixPath(text)
    if not text or pure.is_absolute() or "\\" in text or any(part in ("", ".", "..") for part in text.split("/")):
        raise ValueError(f"{field} path must be canonical and relative")
    path = (manifest_path.parent / Path(*pure.parts)).resolve()
    if not path.is_file() or sha256_path(path) != descriptor.get("sha256"):
        raise ValueError(f"{field} file/hash does not match")
    return path


def _rgb_sources(
    plan: Any,
    rgb_bundle_path: Path,
    rgb_capture_catalog: Path | None,
    ffprobe: str,
) -> tuple[str, str, list[CompleteShotSource], dict[str, Any]]:
    from .render_video import probe_video
    from .timeline import inspect_inputs

    report = inspect_inputs(plan, rgb_bundle_path, rgb_capture_catalog=rgb_capture_catalog)
    if report.ready_genuine_roles != frozenset({"rgb"}) or report.role_errors.get("rgb"):
        raise ValueError(f"RGB bundle is not accepted: {report.role_errors.get('rgb', ())}")
    role = report.roles["rgb"]
    receipt_path = role.artifacts["view_manifest"]
    receipt = _json(receipt_path)
    presentation_transform = validate_presentation_transform(receipt.get("presentation_transform"))
    capture_manifest = _json(role.artifacts["capture_manifest"])
    for artifact_name in ("view_video", "frame_index"):
        if not role.artifacts[artifact_name].is_file():
            raise ValueError(f"RGB {artifact_name} is missing after hash-bound role validation")
        if sha256_path(role.artifacts[artifact_name]) != role.artifact_sha256[artifact_name]:
            raise ValueError(f"RGB {artifact_name} changed after hash-bound role validation")
    rows = []
    with role.artifacts["frame_index"].open(encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    boundary = plan.transition_for_boundary(540)
    if boundary is None or boundary.outgoing_sampling.mode != "contiguous_postroll":
        raise ValueError("production plan does not declare the required frame-540 RGB post-roll")
    required_end = int(boundary.outgoing_sampling.source_frame_end_exclusive or -1)
    if required_end != 549 or len(rows) < required_end:
        raise ValueError("RGB frame index must cover contiguous observed source frames 0-548")
    observed_indices = [int(row.get("frame_index", -1)) for row in rows[:required_end]]
    observed_stamps = [float(row.get("stamp_s", "nan")) for row in rows[:required_end]]
    if observed_indices != list(range(required_end)):
        raise ValueError("RGB frame index must cover contiguous observed source frames 0-548")
    if any(not (first < second) for first, second in zip(observed_stamps, observed_stamps[1:])):
        raise ValueError("RGB observed stamps through source frame 548 must be strictly increasing")
    if any(
        abs((second - first) - RGB_TIMESTAMP_PERIOD_S) > RGB_TIMESTAMP_TOLERANCE_S
        for first, second in zip(observed_stamps, observed_stamps[1:])
    ):
        raise ValueError("RGB observed stamps through source frame 548 must be contiguous at 30 fps")
    video_probe = probe_video(role.artifacts["view_video"], ffprobe)
    if video_probe["frame_count"] < required_end:
        raise ValueError("RGB video must contain source frames 0-548 for the moving post-roll")
    shots = []
    for shot in plan.shots[:5]:
        source_start = shot.start_frame
        source_end = source_start + shot.frame_count
        if source_end > len(rows):
            raise ValueError(f"RGB source does not cover shot {shot.number:02d} frames {source_start}-{source_end - 1}")
        selected = rows[source_start:source_end]
        if [int(row["frame_index"]) for row in selected] != list(range(source_start, source_end)):
            raise ValueError(f"RGB source-frame mapping is not contiguous for shot {shot.number:02d}")
        transition_boundary_frame = None
        transition_source_samples: tuple[tuple[int, float], ...] = ()
        source_last_stamp = float(selected[-1]["stamp_s"])
        outgoing_transition = plan.transition_for_boundary(shot.end_frame_exclusive)
        if outgoing_transition is not None and outgoing_transition.outgoing_sampling.mode == "contiguous_postroll":
            transition_boundary_frame = outgoing_transition.boundary_frame
            transition_start = source_end - outgoing_transition.half_duration_frames
            transition_end = int(outgoing_transition.outgoing_sampling.source_frame_end_exclusive or -1)
            transition_rows = rows[transition_start:transition_end]
            if [int(row["frame_index"]) for row in transition_rows] != list(range(transition_start, transition_end)):
                raise ValueError("RGB transition source mapping must be contiguous through post-roll frame 548")
            transition_source_samples = tuple(
                (int(row["frame_index"]), float(row["stamp_s"])) for row in transition_rows
            )
            source_last_stamp = transition_source_samples[-1][1]
        shots.append(
            CompleteShotSource(
                shot.number,
                role.artifacts["view_video"],
                role.artifact_sha256["view_video"],
                source_start,
                shot.frame_count,
                (float(selected[0]["stamp_s"]), source_last_stamp),
                "rgb_frames.jsonl:stamp_s",
                "rgb_capture",
                "rgb",
                receipt_path,
                role.artifact_sha256["view_manifest"],
                presentation_transform,
                transition_boundary_frame,
                transition_source_samples,
            )
        )
    return (
        role.capture_id,
        str(capture_manifest["capture_sha256"]),
        shots,
        {
            "bundle_sha256": sha256_path(rgb_bundle_path),
            "capture_manifest_sha256": role.artifact_sha256["capture_manifest"],
            "view_receipt_sha256": role.artifact_sha256["view_manifest"],
            "view_video_sha256": role.artifact_sha256["view_video"],
            "frame_index_sha256": role.artifact_sha256["frame_index"],
            "required_source_frame_end_exclusive": required_end,
            "postroll": {
                "boundary_frame": boundary.boundary_frame,
                "source_frame_start": boundary.outgoing_sampling.source_frame_start,
                "source_frame_end_exclusive": boundary.outgoing_sampling.source_frame_end_exclusive,
                "source_time_basis": "rgb_frames.jsonl:stamp_s",
                "samples": [
                    {"source_frame": index, "measurement_timestamp_s": observed_stamps[index]}
                    for index in range(
                        int(boundary.outgoing_sampling.source_frame_start or -1),
                        required_end,
                    )
                ],
            },
            "source_time_range_s": receipt["source_time_range_s"],
            "presentation_transform": receipt["presentation_transform"],
            "acceptance": receipt["capture_acceptance"],
            "presentation_classification": receipt["capture_acceptance"]["presentation_classification"],
        },
    )


def _technical_sources(delivery: ValidatedTechnicalDelivery) -> list[CompleteShotSource]:
    return [
        CompleteShotSource(
            source.shot_number,
            source.video_path,
            source.video_sha256,
            0,
            source.frame_count,
            source.source_time_range_s,
            "validated technical receipt simulation_time data extent",
            "technical_view",
            source.view_id,
            source.receipt_path,
            source.receipt_sha256,
            None,
            None,
            (),
        )
        for source in delivery.shots
    ]


def _shot_value(source: CompleteShotSource, manifest_path: Path) -> dict[str, Any]:
    transition_source = None
    if source.transition_boundary_frame is not None:
        transition_source = {
            "boundary_frame": source.transition_boundary_frame,
            "source_time_basis": source.source_time_basis,
            "samples": [
                {"source_frame": frame, "measurement_timestamp_s": stamp}
                for frame, stamp in source.transition_source_samples
            ],
        }
    return {
        "number": source.shot_number,
        "source_kind": source.source_kind,
        "view_id": source.view_id,
        "video": _descriptor(source.video_path, manifest_path),
        "receipt": _descriptor(source.receipt_path, manifest_path),
        "source_start_frame": source.source_start_frame,
        "frame_count": source.frame_count,
        "source_time_range_s": list(source.source_time_range_s),
        "source_time_basis": source.source_time_basis,
        "presentation_transform": source.presentation_transform,
        "transition_source": transition_source,
    }


def _validate_sources(
    plan: Any,
    rgb_bundle_path: Path,
    technical_manifest_path: Path,
    repo_root: Path,
    ffprobe: str,
    rgb_capture_catalog: Path | None,
    technical_source_catalog: Path | None,
) -> tuple[str, str, dict[str, str], tuple[CompleteShotSource, ...], dict[str, Any]]:
    capture_id, capture_sha256, rgb_shots, rgb_binding = _rgb_sources(
        plan, rgb_bundle_path, rgb_capture_catalog, ffprobe
    )
    technical = validate_technical_delivery(
        technical_manifest_path,
        repo_root,
        ffprobe,
        technical_source_catalog,
    )
    if technical.capture_id != capture_id or technical.capture_sha256 != capture_sha256:
        raise ValueError("RGB and technical inputs do not share one reviewed capture identity")
    presentation_classification = rgb_binding["presentation_classification"]
    if technical.presentation_classification != presentation_classification:
        raise ValueError("RGB and technical inputs do not share one reviewed presentation classification")
    shots = tuple(rgb_shots + _technical_sources(technical))
    if [source.shot_number for source in shots] != list(range(1, 13)):
        raise ValueError("combined presentation inputs do not cover shots 1-12 in order")
    bindings = {
        "capture_id": capture_id,
        "capture_sha256": capture_sha256,
        "presentation_classification": presentation_classification,
        "rgb": rgb_binding,
        "technical": {
            "delivery_manifest_sha256": technical.manifest_sha256,
            "source_id": technical.source_id,
            "source": technical.source,
        },
    }
    return capture_id, capture_sha256, presentation_classification, shots, bindings


def emit_complete_bundle(
    rgb_bundle_path: str | Path,
    technical_manifest_path: str | Path,
    output_manifest_path: str | Path,
    repo_root: str | Path | None = None,
    ffprobe: str = "ffprobe",
    rgb_capture_catalog: str | Path | None = None,
    technical_source_catalog: str | Path | None = None,
) -> dict[str, Any]:
    """Validate both accepted producer bundles and emit shot-specific inputs."""

    root = Path(repo_root).resolve() if repo_root else Path(__file__).resolve().parents[2]
    output = Path(output_manifest_path).resolve()
    rgb_path = Path(rgb_bundle_path).resolve()
    technical_path = Path(technical_manifest_path).resolve()
    from .timeline import load_plan

    plan = load_plan(root / "config" / "presentation" / "storyboard.yaml")
    capture_id, capture_sha256, presentation_classification, shots, bindings = _validate_sources(
        plan,
        rgb_path,
        technical_path,
        root,
        ffprobe,
        Path(rgb_capture_catalog).resolve() if rgb_capture_catalog else None,
        Path(technical_source_catalog).resolve() if technical_source_catalog else None,
    )
    value = {
        "schema_version": 4,
        "status": "complete",
        "label": f"validated complete presentation inputs for capture {capture_id}",
        "producer_id": PRODUCER_ID,
        "producer_source_sha256": source_text_sha256(Path(__file__).resolve()),
        "ground_truth_consumed": False,
        "capture_id": capture_id,
        "capture_sha256": capture_sha256,
        "presentation_classification": presentation_classification,
        "rgb_bundle": _descriptor(rgb_path, output),
        "technical_delivery": _descriptor(technical_path, output),
        "source_bindings": bindings,
        "shots": [_shot_value(source, output) for source in shots],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.partial")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, output)
    validate_complete_bundle(
        plan,
        output,
        root,
        ffprobe,
        Path(rgb_capture_catalog).resolve() if rgb_capture_catalog else None,
        Path(technical_source_catalog).resolve() if technical_source_catalog else None,
    )
    return value


def validate_complete_bundle(
    plan: Any,
    manifest_path: str | Path,
    repo_root: str | Path,
    ffprobe: str,
    rgb_capture_catalog: Path | None = None,
    technical_source_catalog: Path | None = None,
) -> ValidatedCompleteBundle:
    path = Path(manifest_path).resolve()
    root = Path(repo_root).resolve()
    data = _json(path)
    expected_keys = {
        "schema_version", "status", "label", "producer_id", "producer_source_sha256",
        "ground_truth_consumed", "capture_id", "capture_sha256", "rgb_bundle",
        "technical_delivery", "presentation_classification", "source_bindings", "shots",
    }
    if set(data) != expected_keys or data.get("schema_version") != 4 or data.get("status") != "complete":
        raise ValueError("complete presentation input manifest schema is invalid")
    if data.get("producer_id") != PRODUCER_ID or data.get("producer_source_sha256") != source_text_sha256(Path(__file__).resolve()):
        raise ValueError("complete presentation input producer identity is invalid")
    if data.get("ground_truth_consumed") is not False:
        raise ValueError("complete presentation inputs must be ground-truth-free")
    rgb_path = _safe_descriptor(path, data.get("rgb_bundle"), "rgb_bundle")
    technical_path = _safe_descriptor(path, data.get("technical_delivery"), "technical_delivery")
    capture_id, capture_sha256, presentation_classification, shots, bindings = _validate_sources(
        plan,
        rgb_path,
        technical_path,
        root,
        ffprobe,
        rgb_capture_catalog,
        technical_source_catalog,
    )
    if data.get("capture_id") != capture_id or data.get("capture_sha256") != capture_sha256:
        raise ValueError("complete presentation capture identity is forged")
    if data.get("presentation_classification") != presentation_classification:
        raise ValueError("complete presentation classification is forged")
    if data.get("source_bindings") != bindings:
        raise ValueError("complete presentation source bindings are forged")
    expected_shots = [_shot_value(source, path) for source in shots]
    if data.get("shots") != expected_shots:
        raise ValueError("complete presentation shot mapping is forged or incomplete")
    return ValidatedCompleteBundle(
        path,
        capture_id,
        capture_sha256,
        presentation_classification,
        shots,
        bindings,
    )


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-bundle", required=True)
    parser.add_argument("--technical-delivery", required=True)
    parser.add_argument("--output-manifest", required=True)
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    result = emit_complete_bundle(
        args.rgb_bundle,
        args.technical_delivery,
        args.output_manifest,
        root,
        args.ffprobe,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
