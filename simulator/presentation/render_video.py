"""CPU/FFmpeg renderer for validated presentation timelines.

Complete mode stitches timeline-aligned views only after every shot's genuine
RGB/LiDAR/pose/map/reconstruction contracts pass.  Diagnostic mode may use an
explicitly declared baseline for selected shots and replaces unavailable
technical views with labelled slates; it never substitutes storyboard pixels.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

from .timeline import InputReport, PresentationPlan, RoleInput, Shot, inspect_inputs, load_plan


@dataclass(frozen=True)
class PlannedSegment:
    shot: Shot
    kind: str
    source_role: str | None
    video_path: Path | None
    source_start_frame: int
    missing_roles: tuple[str, ...]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _role_is_ready(report: InputReport, name: str) -> bool:
    return name in report.roles and not report.role_errors.get(name, ())


def _fallback_is_usable(role: RoleInput | None, errors: tuple[str, ...]) -> bool:
    if role is None or role.status != "complete" or role.provenance != "diagnostic_baseline":
        return False
    video = role.artifacts.get("view_video")
    if video is None or not video.is_file():
        return False
    return not any("view_video" in error and "undeclared" not in error for error in errors)


def plan_segments(plan: PresentationPlan, report: InputReport, mode: str) -> tuple[PlannedSegment, ...]:
    if mode not in {"diagnostic", "complete"}:
        raise ValueError("mode must be diagnostic or complete")
    if mode == "complete" and not report.complete:
        details = "; ".join(
            f"shot {number:02d}: {', '.join(missing)}"
            for number, missing in report.missing_by_shot.items()
            if missing
        )
        raise ValueError(f"complete render inputs are unavailable ({details})")

    segments: list[PlannedSegment] = []
    for shot in plan.shots:
        missing = report.missing_by_shot[shot.number]
        if _role_is_ready(report, shot.render_role):
            role = report.roles[shot.render_role]
            segments.append(
                PlannedSegment(shot, "genuine", role.name, role.artifacts["view_video"], shot.start_frame, missing)
            )
            continue
        if mode == "complete":
            raise ValueError(f"shot {shot.number:02d} has no ready render role {shot.render_role}")
        fallback = shot.diagnostic_fallback
        if fallback is not None:
            role = report.roles.get(fallback.role)
            if _fallback_is_usable(role, report.role_errors.get(fallback.role, ())):
                assert role is not None
                segments.append(
                    PlannedSegment(
                        shot,
                        "diagnostic_baseline",
                        role.name,
                        role.artifacts["view_video"],
                        fallback.source_start_frame,
                        missing,
                    )
                )
                continue
        segments.append(PlannedSegment(shot, "missing_input_slate", None, None, 0, missing))
    return tuple(segments)


def _run(command: list[str]) -> None:
    completed = subprocess.run(command, text=True, capture_output=True)
    if completed.returncode:
        tail = "\n".join((completed.stderr or completed.stdout).splitlines()[-30:])
        raise RuntimeError(f"command failed ({completed.returncode}): {' '.join(command[:4])}\n{tail}")


def probe_video(path: Path, ffprobe: str) -> dict[str, Any]:
    command = [
        ffprobe,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,avg_frame_rate,r_frame_rate,nb_frames,duration,pix_fmt:format=duration,size",
        "-of",
        "json",
        str(path),
    ]
    completed = subprocess.run(command, text=True, capture_output=True)
    if completed.returncode:
        raise ValueError(f"ffprobe could not read {path}: {completed.stderr.strip()}")
    value = json.loads(completed.stdout)
    streams = value.get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"{path} must contain exactly one video stream")
    stream = streams[0]
    rate = Fraction(str(stream["avg_frame_rate"]))
    real_rate = Fraction(str(stream["r_frame_rate"]))
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps_num": rate.numerator,
        "fps_den": rate.denominator,
        "real_fps_num": real_rate.numerator,
        "real_fps_den": real_rate.denominator,
        "frame_count": int(stream["nb_frames"]),
        "duration_seconds": float(value.get("format", {}).get("duration", stream.get("duration", 0.0))),
        "pix_fmt": str(stream.get("pix_fmt", "")),
        "size_bytes": int(value.get("format", {}).get("size", path.stat().st_size)),
    }


def _escape_text(value: str) -> str:
    return value.replace("\\", "\\\\").replace("'", "\\'").replace(":", "\\:").replace("%", "\\%")


def _drawtext(text: str, x: str, y: str, size: int, color: str = "white") -> str:
    return f"drawtext=text='{_escape_text(text)}':x={x}:y={y}:fontsize={size}:fontcolor={color}"


def _timestamp(frame: int, fps: int) -> str:
    seconds = frame // fps
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _validate_source(
    segment: PlannedSegment,
    profile_width: int,
    profile_height: int,
    fps: int,
    mode: str,
    ffprobe: str,
    cache: dict[Path, dict[str, Any]],
) -> dict[str, Any]:
    assert segment.video_path is not None
    metadata = cache.setdefault(segment.video_path, probe_video(segment.video_path, ffprobe))
    if (
        metadata["fps_num"],
        metadata["fps_den"],
        metadata["real_fps_num"],
        metadata["real_fps_den"],
    ) != (fps, 1, fps, 1):
        raise ValueError(f"{segment.video_path} must be constant {fps} fps")
    required_end = segment.source_start_frame + segment.shot.frame_count
    if metadata["frame_count"] < required_end:
        raise ValueError(
            f"{segment.video_path} has {metadata['frame_count']} frames; shot {segment.shot.number:02d} needs {required_end}"
        )
    if mode == "complete" and (metadata["width"] < profile_width or metadata["height"] < profile_height):
        raise ValueError(f"complete {profile_width}x{profile_height} output cannot upscale {segment.video_path}")
    return metadata


def _build_filter(
    plan: PresentationPlan,
    profile_name: str,
    segments: tuple[PlannedSegment, ...],
) -> tuple[list[str], str]:
    profile = plan.profiles[profile_name]
    inputs: list[str] = []
    filters: list[str] = []
    input_index = 0
    labels: list[str] = []
    for segment in segments:
        shot = segment.shot
        output_label = f"v{shot.number:02d}"
        labels.append(f"[{output_label}]")
        title = f"SHOT {shot.number:02d}  |  {_timestamp(shot.start_frame, plan.fps)}-{_timestamp(shot.end_frame_exclusive, plan.fps)}  |  {shot.title}"
        if segment.video_path is not None:
            inputs.extend(
                [
                    "-ss",
                    f"{segment.source_start_frame / plan.fps:.6f}",
                    "-i",
                    str(segment.video_path),
                ]
            )
            chain = [
                f"[{input_index}:v]trim=start_frame=0:end_frame={shot.frame_count}",
                "setpts=PTS-STARTPTS",
                f"fps={plan.fps}",
                f"scale={profile.width}:{profile.height}:force_original_aspect_ratio=decrease",
                f"pad={profile.width}:{profile.height}:(ow-iw)/2:(oh-ih)/2:color=black",
                "setsar=1",
            ]
            if segment.kind == "diagnostic_baseline":
                chain.extend(
                    [
                        "drawbox=x=0:y=0:w=iw:h=52:color=0x36d9ff:t=fill",
                        _drawtext("DIAGNOSTIC PREVIEW - EXISTING SIMULATED RGB BASELINE - NOT FINAL CAPTURE", "22", "13", 22, "black"),
                        "drawbox=x=0:y=h-48:w=iw:h=48:color=black@0.72:t=fill",
                        _drawtext(title, "22", "h-36", 20),
                    ]
                )
            filters.append(",".join(chain) + f"[{output_label}]")
            input_index += 1
            continue

        missing = "  /  ".join(segment.missing_roles) if segment.missing_roles else "undeclared render view"
        filters.append(
            ",".join(
                [
                    f"color=c=0x07111f:s={profile.width}x{profile.height}:r={plan.fps}:d={shot.frame_count / plan.fps:.6f}",
                    f"trim=start_frame=0:end_frame={shot.frame_count}",
                    "setpts=PTS-STARTPTS",
                    "drawbox=x=0:y=0:w=iw:h=8:color=0x36d9ff:t=fill",
                    _drawtext(title, "64", "72", 28, "0x8beeff"),
                    _drawtext("TECHNICAL INPUTS NOT AVAILABLE", "64", "210", 46),
                    _drawtext(missing, "64", "295", 30, "0x58dfff"),
                    _drawtext("No LiDAR, map, or reconstruction imagery is synthesized for this interval.", "64", "375", 23, "0xb7c4d6"),
                    _drawtext("DIAGNOSTIC TIMELINE ONLY - INCOMPLETE", "64", "h-86", 22, "0xffc857"),
                    "format=yuv420p",
                ]
            )
            + f"[{output_label}]"
        )
    filters.append("".join(labels) + f"concat=n={len(segments)}:v=1:a=0,format=yuv420p[outv]")
    return inputs, ";".join(filters)


def _contact_sheet(video: Path, output: Path, ffmpeg: str, plan: PresentationPlan) -> None:
    midpoint_frames = [shot.start_frame + shot.frame_count // 2 for shot in plan.shots]
    expression = "+".join(f"eq(n\\,{frame})" for frame in midpoint_frames)
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(video),
        "-vf",
        f"select='{expression}',scale=320:180,tile=4x3:nb_frames=12",
        "-frames:v",
        "1",
        "-y",
        str(output),
    ]
    _run(command)


def render_presentation(
    plan_path: str | Path,
    inputs_path: str | Path,
    output_dir: str | Path,
    profile_name: str,
    mode: str,
    ffmpeg: str,
    ffprobe: str,
) -> dict[str, Any]:
    plan = load_plan(plan_path)
    report = inspect_inputs(plan, inputs_path)
    if profile_name not in plan.profiles:
        raise ValueError(f"unknown profile {profile_name}")
    if mode == "diagnostic" and profile_name != "preview":
        raise ValueError("diagnostic output is restricted to the 1280x720 preview profile")
    segments = plan_segments(plan, report, mode)
    profile = plan.profiles[profile_name]
    source_probes: dict[Path, dict[str, Any]] = {}
    for segment in segments:
        if segment.video_path is not None:
            _validate_source(segment, profile.width, profile.height, plan.fps, mode, ffprobe, source_probes)

    target_dir = Path(output_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    stem = "diagnostic_preview" if mode == "diagnostic" else f"presentation_{profile_name}"
    video = target_dir / f"{stem}.mp4"
    temporary_video = target_dir / f".{stem}.partial.mp4"
    inputs, filter_graph = _build_filter(plan, profile_name, segments)
    command = [ffmpeg, "-hide_banner", "-loglevel", "warning", *inputs, "-filter_complex", filter_graph, "-map", "[outv]", "-an", "-frames:v", str(plan.frame_count), "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-y", str(temporary_video)]
    _run(command)
    output_probe = probe_video(temporary_video, ffprobe)
    expected = (profile.width, profile.height, plan.fps, 1, plan.fps, 1, plan.frame_count)
    observed = (
        output_probe["width"],
        output_probe["height"],
        output_probe["fps_num"],
        output_probe["fps_den"],
        output_probe["real_fps_num"],
        output_probe["real_fps_den"],
        output_probe["frame_count"],
    )
    if observed != expected or abs(output_probe["duration_seconds"] - plan.duration_seconds) > 0.001:
        raise RuntimeError(f"encoded output does not match the profile: expected {expected}, observed {observed}")
    os.replace(temporary_video, video)

    contact_sheet = target_dir / f"{stem}_contact_sheet.png"
    _contact_sheet(video, contact_sheet, ffmpeg, plan)
    manifest = {
        "schema_version": 1,
        "status": "diagnostic_incomplete" if mode == "diagnostic" and not report.complete else "complete",
        "claim": "diagnostic timeline; unavailable technical intervals are labelled slates" if mode == "diagnostic" else "validated genuine-input presentation",
        "mode": mode,
        "profile": profile_name,
        "width": profile.width,
        "height": profile.height,
        "fps": plan.fps,
        "frame_count": plan.frame_count,
        "duration_seconds": plan.duration_seconds,
        "ground_truth_consumed": False,
        "storyboard_pixels_consumed": False,
        "plan": str(plan.path),
        "plan_sha256": _file_sha256(plan.path),
        "inputs": str(report.manifest_path),
        "inputs_sha256": _file_sha256(report.manifest_path),
        "renderer_files": {
            str(Path(__file__).resolve()): _file_sha256(Path(__file__).resolve()),
            str(Path(__file__).with_name("timeline.py").resolve()): _file_sha256(Path(__file__).with_name("timeline.py").resolve()),
        },
        "video": video.name,
        "video_sha256": _file_sha256(video),
        "contact_sheet": contact_sheet.name,
        "contact_sheet_sha256": _file_sha256(contact_sheet),
        "probe": output_probe,
        "sources": {
            str(path): {**metadata, "sha256": _file_sha256(path)}
            for path, metadata in source_probes.items()
        },
        "shots": [
            {
                "number": segment.shot.number,
                "slug": segment.shot.slug,
                "start_frame": segment.shot.start_frame,
                "end_frame_exclusive": segment.shot.end_frame_exclusive,
                "kind": segment.kind,
                "source_role": segment.source_role,
                "source_start_frame": segment.source_start_frame if segment.video_path is not None else None,
                "missing_genuine_roles": list(segment.missing_roles),
            }
            for segment in segments
        ],
    }
    manifest_path = target_dir / f"{stem}_manifest.json"
    temporary_manifest = target_dir / f".{stem}_manifest.partial.json"
    temporary_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary_manifest, manifest_path)
    return manifest


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", default=str(root / "config" / "presentation" / "storyboard.yaml"))
    parser.add_argument("--inputs", default=str(root / "config" / "presentation" / "diagnostic_baseline_inputs.json"))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--profile", choices=("preview", "delivery"), default="preview")
    parser.add_argument("--mode", choices=("diagnostic", "complete"), default="diagnostic")
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    result = render_presentation(args.plan, args.inputs, args.output_dir, args.profile, args.mode, args.ffmpeg, args.ffprobe)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
