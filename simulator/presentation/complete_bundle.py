"""Combine one reviewed RGB bundle with reviewed technical views for all 12 shots."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .provenance import sha256_path, source_text_sha256, validate_presentation_transform
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
) -> tuple[str, str, list[CompleteShotSource], dict[str, Any]]:
    from .timeline import inspect_inputs

    report = inspect_inputs(plan, rgb_bundle_path, rgb_capture_catalog=rgb_capture_catalog)
    if report.ready_genuine_roles != frozenset({"rgb"}) or report.role_errors.get("rgb"):
        raise ValueError(f"RGB bundle is not accepted: {report.role_errors.get('rgb', ())}")
    role = report.roles["rgb"]
    receipt_path = role.artifacts["view_manifest"]
    receipt = _json(receipt_path)
    presentation_transform = validate_presentation_transform(receipt.get("presentation_transform"))
    capture_manifest = _json(role.artifacts["capture_manifest"])
    rows = []
    with role.artifacts["frame_index"].open(encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    shots = []
    for shot in plan.shots[:5]:
        source_start = shot.start_frame
        source_end = source_start + shot.frame_count
        if source_end > len(rows):
            raise ValueError(f"RGB source does not cover shot {shot.number:02d} frames {source_start}-{source_end - 1}")
        selected = rows[source_start:source_end]
        if [int(row["frame_index"]) for row in selected] != list(range(source_start, source_end)):
            raise ValueError(f"RGB source-frame mapping is not contiguous for shot {shot.number:02d}")
        shots.append(
            CompleteShotSource(
                shot.number,
                role.artifacts["view_video"],
                role.artifact_sha256["view_video"],
                source_start,
                shot.frame_count,
                (float(selected[0]["stamp_s"]), float(selected[-1]["stamp_s"])),
                "rgb_frames.jsonl:stamp_s",
                "rgb_capture",
                "rgb",
                receipt_path,
                role.artifact_sha256["view_manifest"],
                presentation_transform,
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
        )
        for source in delivery.shots
    ]


def _shot_value(source: CompleteShotSource, manifest_path: Path) -> dict[str, Any]:
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
        plan, rgb_bundle_path, rgb_capture_catalog
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
        "schema_version": 3,
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
    if set(data) != expected_keys or data.get("schema_version") != 3 or data.get("status") != "complete":
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
