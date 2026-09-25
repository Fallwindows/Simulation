"""CPU/FFmpeg renderer for validated presentation timelines.

Complete mode stitches timeline-aligned views only after every shot's genuine
RGB/LiDAR/pose/map/reconstruction contracts pass.  Diagnostic mode may use an
explicitly declared baseline for selected shots and replaces unavailable
technical views with labelled slates; it never substitutes storyboard pixels.
"""

from __future__ import annotations

import argparse
from array import array
import hashlib
import json
import os
import math
import shutil
import subprocess
import uuid
import wave
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

from .provenance import (
    DIAGNOSTIC_BASELINE_IDENTITY,
    validate_diagnostic_identity,
    validate_presentation_transform,
)
from .timeline import InputReport, PresentationPlan, RoleInput, Shot, inspect_inputs, load_plan


@dataclass(frozen=True)
class PlannedSegment:
    shot: Shot
    kind: str
    source_role: str | None
    video_path: Path | None
    source_start_frame: int
    missing_roles: tuple[str, ...]
    source_time_range_s: tuple[float, float] | None = None
    source_time_basis: str | None = None
    view_id: str | None = None
    video_sha256: str | None = None
    receipt_sha256: str | None = None
    presentation_transform: dict[str, object] | None = None


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
        if mode == "complete":
            source = report.shot_sources.get(shot.number)
            if source is None:
                raise ValueError(f"shot {shot.number:02d} has no validated shot-specific source")
            if source.frame_count != shot.frame_count:
                raise ValueError(f"shot {shot.number:02d} source frame count does not match the timeline")
            segments.append(
                PlannedSegment(
                    shot,
                    "genuine",
                    source.source_kind,
                    source.video_path,
                    source.source_start_frame,
                    missing,
                    source.source_time_range_s,
                    source.source_time_basis,
                    source.view_id,
                    source.video_sha256,
                    source.receipt_sha256,
                    source.presentation_transform,
                )
            )
            continue
        if _role_is_ready(report, shot.render_role):
            role = report.roles[shot.render_role]
            segments.append(
                PlannedSegment(shot, "genuine", role.name, role.artifacts["view_video"], shot.start_frame, missing)
            )
            continue
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


def probe_media(path: Path, ffprobe: str) -> dict[str, Any]:
    command = [
        ffprobe,
        "-v",
        "error",
        "-count_frames",
        "-show_entries",
        "stream=codec_type,codec_name,width,height,avg_frame_rate,r_frame_rate,nb_frames,nb_read_frames,duration,pix_fmt,sample_rate,channels:format=duration,size",
        "-of",
        "json",
        str(path),
    ]
    value = None
    completed = None
    for _attempt in range(2):
        completed = subprocess.run(command, text=True, capture_output=True)
        if completed.returncode:
            raise ValueError(f"ffprobe could not read {path}: {completed.stderr.strip()}")
        try:
            value = json.loads(completed.stdout)
            break
        except json.JSONDecodeError:
            continue
    if value is None:
        assert completed is not None
        raise ValueError(f"ffprobe returned malformed JSON for {path}: {completed.stdout[-500:]}")
    streams = value.get("streams", [])
    videos = [stream for stream in streams if stream.get("codec_type") == "video"]
    audio = [stream for stream in streams if stream.get("codec_type") == "audio"]
    if len(videos) != 1:
        raise ValueError(f"{path} must contain exactly one video stream")
    stream = videos[0]
    rate = Fraction(str(stream["avg_frame_rate"]))
    real_rate = Fraction(str(stream["r_frame_rate"]))
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps_num": rate.numerator,
        "fps_den": rate.denominator,
        "real_fps_num": real_rate.numerator,
        "real_fps_den": real_rate.denominator,
        "frame_count": int(stream.get("nb_read_frames") or stream.get("nb_frames")),
        "duration_seconds": float(value.get("format", {}).get("duration", stream.get("duration", 0.0))),
        "pix_fmt": str(stream.get("pix_fmt", "")),
        "video_codec": str(stream.get("codec_name", "")),
        "video_stream_count": len(videos),
        "audio_stream_count": len(audio),
        "audio_codec": str(audio[0].get("codec_name", "")) if audio else None,
        "audio_sample_rate": int(audio[0].get("sample_rate", 0)) if audio else None,
        "audio_channels": int(audio[0].get("channels", 0)) if audio else None,
        "size_bytes": int(value.get("format", {}).get("size", path.stat().st_size)),
    }


def probe_video(path: Path, ffprobe: str) -> dict[str, Any]:
    return probe_media(path, ffprobe)


def _escape_text(value: str) -> str:
    return value.replace("\\", "\\\\").replace("'", "\\'").replace(":", "\\:").replace("%", "\\%")


def _drawtext(text: str, x: str, y: str, size: int, color: str = "white") -> str:
    return f"drawtext=text='{_escape_text(text)}':x={x}:y={y}:fontsize={size}:fontcolor={color}"


def _timestamp(frame: int, fps: int) -> str:
    seconds = frame // fps
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _seconds(frames: int, fps: int) -> str:
    value = f"{frames / fps:.9f}".rstrip("0").rstrip(".")
    return value or "0"


def _smoothstep(value: float) -> float:
    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return value * value * (3.0 - 2.0 * value)


def _transition_manifest(plan: PresentationPlan, segments: tuple[PlannedSegment, ...]) -> list[dict[str, Any]]:
    by_shot = {segment.shot.number: segment for segment in segments}
    result = []
    for transition in plan.transitions:
        outgoing = by_shot[transition.from_shot]
        incoming = by_shot[transition.to_shot]
        outgoing_timestamp = outgoing.source_time_range_s[1] if outgoing.source_time_range_s else None
        incoming_timestamp = incoming.source_time_range_s[0] if incoming.source_time_range_s else None
        samples = []
        for relative_frame in range(transition.duration_frames):
            progress = 1.0 - relative_frame / transition.duration_frames
            outgoing_weight = _smoothstep(progress)
            film_frame = transition.start_frame + relative_frame
            samples.append(
                {
                    "film_frame": film_frame,
                    "nominal_shot": plan.shot_for_frame(film_frame).number,
                    "descending_progress": round(progress, 12),
                    "outgoing_weight": round(outgoing_weight, 12),
                    "incoming_weight": round(1.0 - outgoing_weight, 12),
                }
            )
        result.append(
            {
                "from_shot": transition.from_shot,
                "to_shot": transition.to_shot,
                "boundary_frame": transition.boundary_frame,
                "start_frame": transition.start_frame,
                "end_frame_exclusive": transition.end_frame_exclusive,
                "duration_frames": transition.duration_frames,
                "style": transition.style,
                "easing": transition.easing,
                "intent": transition.intent,
                "frame_budget_delta": 0,
                "classification": {
                    "kind": "editorial_temporal_blend",
                    "co_timed": False,
                    "sensor_fusion": False,
                    "geometry_fusion": False,
                    "claim": "presentation-only transition between independently rendered source clips; not a sensor-fusion or co-timed measurement product",
                },
                "shot_assignment": {
                    "exclusive": False,
                    "shared_between_shots": [transition.from_shot, transition.to_shot],
                    "policy": "every frame in this window is an editorial mixture even when its film index falls inside one nominal shot interval",
                },
                "held_sources": {
                    "outgoing": {
                        "shot": transition.from_shot,
                        "view_id": outgoing.view_id,
                        "source_role": outgoing.source_role,
                        "source_video_sha256": outgoing.video_sha256,
                        "source_receipt_sha256": outgoing.receipt_sha256,
                        "source_frame": outgoing.source_start_frame + outgoing.shot.frame_count - 1,
                        "source_time_basis": outgoing.source_time_basis,
                        "source_frame_timestamp_s": outgoing_timestamp,
                        "co_timed_with_other_source": False,
                        "held_film_frames": {
                            "start_frame": transition.boundary_frame,
                            "end_frame_exclusive": transition.end_frame_exclusive,
                        },
                    },
                    "incoming": {
                        "shot": transition.to_shot,
                        "view_id": incoming.view_id,
                        "source_role": incoming.source_role,
                        "source_video_sha256": incoming.video_sha256,
                        "source_receipt_sha256": incoming.receipt_sha256,
                        "source_frame": incoming.source_start_frame,
                        "source_time_basis": incoming.source_time_basis,
                        "source_frame_timestamp_s": incoming_timestamp,
                        "co_timed_with_other_source": False,
                        "held_film_frames": {
                            "start_frame": transition.start_frame,
                            "end_frame_exclusive": transition.boundary_frame,
                        },
                    },
                },
                "weight_law": {
                    "ffmpeg_progress_direction": "P descends from 1 toward 0",
                    "progress": "P = 1 - relative_frame / duration_frames",
                    "smoothstep": "s(P) = P*P*(3-2*P)",
                    "outgoing": "s(P)",
                    "incoming": "1-s(P)",
                    "entry_guard": {
                        "film_frame": transition.start_frame - 1,
                        "outgoing_weight": 1.0,
                        "incoming_weight": 0.0,
                    },
                    "exit_guard": {
                        "film_frame": transition.end_frame_exclusive,
                        "outgoing_weight": 0.0,
                        "incoming_weight": 1.0,
                    },
                    "per_frame": samples,
                },
            }
        )
    return result


def _shot_transition_windows(plan: PresentationPlan, shot: Shot) -> list[dict[str, Any]]:
    windows = []
    for transition in plan.transitions:
        if shot.number not in (transition.from_shot, transition.to_shot):
            continue
        other = transition.to_shot if shot.number == transition.from_shot else transition.from_shot
        windows.append(
            {
                "boundary_frame": transition.boundary_frame,
                "start_frame": transition.start_frame,
                "end_frame_exclusive": transition.end_frame_exclusive,
                "with_shot": other,
                "source_side": "outgoing" if shot.number == transition.from_shot else "incoming",
                "exclusive_assignment": False,
            }
        )
    return windows


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
    if segment.video_sha256 is not None and _file_sha256(segment.video_path) != segment.video_sha256:
        raise ValueError(f"source video hash changed after provenance validation: {segment.video_path}")
    return metadata


def _build_filter(
    plan: PresentationPlan,
    profile_name: str,
    segments: tuple[PlannedSegment, ...],
    test_fixture_label: bool = False,
) -> tuple[list[str], str]:
    profile = plan.profiles[profile_name]
    inputs: list[str] = []
    filters: list[str] = []
    input_index = 0
    active_transitions = {}
    for index in range(1, len(segments)):
        previous, current = segments[index - 1], segments[index]
        transition = plan.transition_for_boundary(current.shot.start_frame)
        if transition is not None and (previous.shot.number, current.shot.number) == (transition.from_shot, transition.to_shot):
            active_transitions[index] = transition
    incoming_pad = {index: item.half_duration_frames for index, item in active_transitions.items()}
    outgoing_pad = {index - 1: item.half_duration_frames for index, item in active_transitions.items()}
    for index, segment in enumerate(segments):
        shot = segment.shot
        output_label = f"v{shot.number:02d}"
        title = f"SHOT {shot.number:02d}  |  {_timestamp(shot.start_frame, plan.fps)}-{_timestamp(shot.end_frame_exclusive, plan.fps)}  |  {shot.title}"
        if segment.video_path is not None:
            inputs.extend(["-i", str(segment.video_path)])
            chain = [
                f"[{input_index}:v]trim=start_frame={segment.source_start_frame}:end_frame={segment.source_start_frame + shot.frame_count}",
                "setpts=PTS-STARTPTS",
                f"fps={plan.fps}",
            ]
            if segment.source_role == "rgb_capture":
                transform = validate_presentation_transform(segment.presentation_transform)
                if transform["operation"] == "hflip":
                    chain.append("hflip")
            elif segment.presentation_transform is not None:
                raise ValueError("only reviewed RGB capture segments may declare a presentation transform")
            chain.extend(
                [
                    f"scale={profile.width}:{profile.height}:force_original_aspect_ratio=decrease",
                    f"pad={profile.width}:{profile.height}:(ow-iw)/2:(oh-ih)/2:color=black",
                    "setsar=1",
                    f"settb=1/{plan.fps}",
                    "format=yuv420p",
                ]
            )
            if segment.kind == "diagnostic_baseline":
                chain.extend(
                    [
                        "drawbox=x=0:y=0:w=iw:h=52:color=0x36d9ff:t=fill",
                        _drawtext("DIAGNOSTIC PREVIEW - EXISTING SIMULATED RGB BASELINE - NOT FINAL CAPTURE", "22", "13", 22, "black"),
                        "drawbox=x=0:y=h-48:w=iw:h=48:color=black@0.72:t=fill",
                        _drawtext(title, "22", "h-36", 20),
                    ]
                )
            start_pad = incoming_pad.get(index, 0)
            stop_pad = outgoing_pad.get(index, 0)
            if start_pad or stop_pad:
                chain.append(f"tpad=start_mode=clone:start={start_pad}:stop_mode=clone:stop={stop_pad}")
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
                    f"settb=1/{plan.fps}",
                ]
            )
            + (
                f",tpad=start_mode=clone:start={incoming_pad.get(index, 0)}:stop_mode=clone:stop={outgoing_pad.get(index, 0)}"
                if incoming_pad.get(index, 0) or outgoing_pad.get(index, 0)
                else ""
            )
            + f"[{output_label}]"
        )
    if not segments:
        raise ValueError("presentation filter requires at least one segment")
    current_label = f"v{segments[0].shot.number:02d}"
    logical_frames = segments[0].shot.frame_count
    for index, segment in enumerate(segments[1:], 1):
        next_label = f"v{segment.shot.number:02d}"
        mixed_label = f"mix{index:02d}"
        transition = active_transitions.get(index)
        if transition is None:
            filters.append(f"[{current_label}][{next_label}]concat=n=2:v=1:a=0,settb=1/{plan.fps}[{mixed_label}]")
        else:
            half = transition.half_duration_frames
            duration_s = _seconds(transition.duration_frames, plan.fps)
            offset_s = _seconds(logical_frames - half, plan.fps)
            expression = "A*(P*P*(3-2*P))+B*(1-(P*P*(3-2*P)))"
            filters.append(
                f"[{current_label}][{next_label}]xfade=transition=custom:duration={duration_s}:offset={offset_s}:expr='{expression}',settb=1/{plan.fps}[{mixed_label}]"
            )
        current_label = mixed_label
        logical_frames += segment.shot.frame_count
    final_chain = f"[{current_label}]trim=start_frame=0:end_frame={logical_frames},setpts=PTS-STARTPTS,format=yuv420p"
    if test_fixture_label:
        final_chain += ",drawbox=x=0:y=0:w=iw:h=44:color=0x7d1538@0.92:t=fill," + _drawtext(
            "GENERATED TEST FIXTURE - NOT PRODUCTION CAPTURE", "18", "10", 20, "white"
        )
    filters.append(final_chain + "[outv]")
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


def _write_deterministic_ambience(path: Path, duration_s: int, sample_rate: int = 48_000) -> None:
    """Create restrained synthetic store tone, footsteps, and cart-wheel texture."""

    samples = array("h")
    for index in range(duration_s * sample_rate):
        t = index / sample_rate
        store_tone = 0.007 * math.sin(2.0 * math.pi * 57.0 * t) + 0.003 * math.sin(2.0 * math.pi * 121.0 * t)
        cart = 0.004 * math.sin(2.0 * math.pi * 29.0 * t) * (0.7 + 0.3 * math.sin(2.0 * math.pi * 0.31 * t))
        foot_phase = t % 0.56
        foot_envelope = max(0.0, 1.0 - foot_phase / 0.07) ** 4 if t < 18.0 else 0.0
        footstep = 0.018 * foot_envelope * math.sin(2.0 * math.pi * 92.0 * t)
        value = max(-1.0, min(1.0, store_tone + cart + footstep))
        left = int(round(value * 32767.0))
        right = int(round((store_tone + 0.85 * cart + 0.92 * footstep) * 32767.0))
        samples.extend((left, max(-32768, min(32767, right))))
    with wave.open(str(path), "wb") as output:
        output.setnchannels(2)
        output.setsampwidth(2)
        output.setframerate(sample_rate)
        output.writeframes(samples.tobytes())


def _validate_output(
    path: Path,
    ffprobe: str,
    width: int,
    height: int,
    frame_count: int,
    duration_s: float,
    *,
    video_codec: str,
    audio_streams: int,
) -> dict[str, Any]:
    probe = probe_media(path, ffprobe)
    observed = (
        probe["width"], probe["height"], probe["fps_num"], probe["fps_den"],
        probe["real_fps_num"], probe["real_fps_den"], probe["frame_count"],
        probe["video_stream_count"], probe["audio_stream_count"], probe["video_codec"],
    )
    expected = (width, height, 30, 1, 30, 1, frame_count, 1, audio_streams, video_codec)
    if observed != expected or abs(probe["duration_seconds"] - duration_s) > 0.05:
        raise RuntimeError(f"encoded output contract mismatch for {path.name}: expected {expected}, observed {observed}")
    if audio_streams and (probe["audio_codec"], probe["audio_sample_rate"], probe["audio_channels"]) != ("aac", 48000, 2):
        raise RuntimeError(f"audio contract mismatch for {path.name}: {probe}")
    return probe


def _representative_frames(video: Path, output_dir: Path, ffmpeg: str, plan: PresentationPlan) -> list[dict[str, Any]]:
    frames = [
        frame
        for shot in plan.shots
        for frame in (shot.start_frame, shot.start_frame + shot.frame_count // 2, shot.end_frame_exclusive - 1)
    ]
    output_dir.mkdir(parents=True, exist_ok=True)
    for old in output_dir.glob("*.png"):
        old.unlink()
    expression = "+".join(f"eq(n\\,{frame})" for frame in frames)
    pattern = output_dir / "selected_%02d.png"
    _run(
        [
            ffmpeg, "-hide_banner", "-loglevel", "error", "-i", str(video),
            "-vf", f"select='{expression}'", "-fps_mode", "vfr", "-y", str(pattern),
        ]
    )
    selected = sorted(output_dir.glob("selected_*.png"))
    if len(selected) != len(frames):
        raise RuntimeError(f"expected {len(frames)} representative frames, received {len(selected)}")
    result = []
    for temporary, frame in zip(selected, frames):
        shot = plan.shot_for_frame(frame)
        target = output_dir / f"shot_{shot.number:02d}_frame_{frame:04d}.png"
        temporary.replace(target)
        result.append({"shot": shot.number, "film_frame": frame, "path": f"{output_dir.name}/{target.name}", "sha256": _file_sha256(target)})
    return result


def _render_presentation_generation(
    plan_path: str | Path,
    inputs_path: str | Path,
    output_dir: str | Path,
    profile_name: str,
    mode: str,
    ffmpeg: str,
    ffprobe: str,
    rgb_capture_catalog: str | Path | None = None,
    technical_source_catalog: str | Path | None = None,
) -> dict[str, Any]:
    plan = load_plan(plan_path)
    if mode == "diagnostic":
        validate_diagnostic_identity(plan.path, Path(inputs_path).resolve(), plan.path.parents[2])
    report = inspect_inputs(
        plan,
        inputs_path,
        rgb_capture_catalog=rgb_capture_catalog,
        technical_source_catalog=technical_source_catalog,
        ffprobe=ffprobe,
    )
    if not report.provenance_validated:
        raise ValueError("presentation provenance validation did not complete")
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
    test_fixture = (
        mode == "complete"
        and report.presentation_classification.get("kind") == "generated_test_fixture"
    )
    inputs, filter_graph = _build_filter(plan, profile_name, segments, test_fixture)
    outputs: dict[str, dict[str, Any]] = {}
    if mode == "complete" and profile_name == "delivery":
        master = target_dir / "presentation_master_lossless.mkv"
        partial_master = target_dir / ".presentation_master_lossless.partial.mkv"
        _run(
            [
                ffmpeg, "-hide_banner", "-loglevel", "warning", *inputs,
                "-filter_complex", filter_graph, "-map", "[outv]", "-an", "-frames:v", str(plan.frame_count),
                "-c:v", "ffv1", "-level", "3", "-pix_fmt", "yuv420p", "-y", str(partial_master),
            ]
        )
        master_probe = _validate_output(partial_master, ffprobe, 1920, 1080, plan.frame_count, plan.duration_seconds, video_codec="ffv1", audio_streams=0)
        os.replace(partial_master, master)
        silent = target_dir / "presentation_silent.mp4"
        _run(
            [
                ffmpeg, "-hide_banner", "-loglevel", "warning", "-i", str(master), "-an",
                "-c:v", "libx264", "-preset", "slow", "-crf", "14", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", "-y", str(silent),
            ]
        )
        silent_probe = _validate_output(silent, ffprobe, 1920, 1080, plan.frame_count, plan.duration_seconds, video_codec="h264", audio_streams=0)
        ambience = target_dir / "presentation_ambience.wav"
        _write_deterministic_ambience(ambience, plan.duration_seconds)
        final = target_dir / "presentation_final.mp4"
        _run(
            [
                ffmpeg, "-hide_banner", "-loglevel", "warning", "-i", str(silent), "-i", str(ambience),
                "-map", "0:v:0", "-map", "1:a:0", "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
                "-t", str(plan.duration_seconds), "-movflags", "+faststart", "-y", str(final),
            ]
        )
        final_probe = _validate_output(final, ffprobe, 1920, 1080, plan.frame_count, plan.duration_seconds, video_codec="h264", audio_streams=1)
        review = target_dir / "presentation_review_720p.mp4"
        _run(
            [
                ffmpeg, "-hide_banner", "-loglevel", "warning", "-i", str(final),
                "-vf", "scale=1280:720", "-c:v", "libx264", "-preset", "fast", "-crf", "20",
                "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "128k", "-frames:v", str(plan.frame_count),
                "-movflags", "+faststart", "-y", str(review),
            ]
        )
        review_probe = _validate_output(review, ffprobe, 1280, 720, plan.frame_count, plan.duration_seconds, video_codec="h264", audio_streams=1)
        outputs = {
            "lossless_master": {"path": master.name, "sha256": _file_sha256(master), "probe": master_probe, "lossless": True},
            "silent_mp4": {"path": silent.name, "sha256": _file_sha256(silent), "probe": silent_probe},
            "audio_bed": {"path": ambience.name, "sha256": _file_sha256(ambience), "description": "deterministic self-created store tone with restrained footsteps and cart texture"},
            "final_mp4": {"path": final.name, "sha256": _file_sha256(final), "probe": final_probe},
            "review_mp4": {"path": review.name, "sha256": _file_sha256(review), "probe": review_probe},
        }
        video = final
        output_probe = final_probe
    else:
        video = target_dir / f"{stem}.mp4"
        temporary_video = target_dir / f".{stem}.partial.mp4"
        _run(
            [
                ffmpeg, "-hide_banner", "-loglevel", "warning", *inputs, "-filter_complex", filter_graph,
                "-map", "[outv]", "-an", "-frames:v", str(plan.frame_count), "-c:v", "libx264",
                "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                "-y", str(temporary_video),
            ]
        )
        output_probe = _validate_output(
            temporary_video,
            ffprobe,
            profile.width,
            profile.height,
            plan.frame_count,
            plan.duration_seconds,
            video_codec="h264",
            audio_streams=0,
        )
        os.replace(temporary_video, video)
        outputs = {"preview" if profile_name == "preview" else "silent_mp4": {"path": video.name, "sha256": _file_sha256(video), "probe": output_probe}}

    contact_sheet = target_dir / f"{stem}_contact_sheet.png"
    _contact_sheet(video, contact_sheet, ffmpeg, plan)
    representative_frames = _representative_frames(video, target_dir / f"{stem}_representative_frames", ffmpeg, plan)
    manifest = {
        "schema_version": 1,
        "status": "test_demonstration" if test_fixture else ("diagnostic_incomplete" if mode == "diagnostic" and not report.complete else "complete"),
        "claim": (
            "generated fixture presentation contract demonstration; not production capture"
            if test_fixture
            else ("diagnostic timeline from the pinned pre-storyboard RGB baseline; unavailable technical intervals are labelled slates" if mode == "diagnostic" else "validated genuine-input presentation")
        ),
        "mode": mode,
        "profile": profile_name,
        "width": profile.width,
        "height": profile.height,
        "fps": plan.fps,
        "frame_count": plan.frame_count,
        "duration_seconds": plan.duration_seconds,
        "transitions": _transition_manifest(plan, segments),
        "ground_truth_consumed": False,
        "provenance_validation": {
            "status": "validated",
            "storyboard_manifest_sha256": report.storyboard_manifest_sha256,
            "artifact_hashes_verified": True,
            "exact_storyboard_file_hashes_rejected": True,
        },
        "plan": str(plan.path),
        "plan_sha256": _file_sha256(plan.path),
        "inputs": str(report.manifest_path),
        "inputs_sha256": _file_sha256(report.manifest_path),
        "source_bindings": report.source_bindings,
        "renderer_files": {
            str(Path(__file__).resolve()): _file_sha256(Path(__file__).resolve()),
            str(Path(__file__).with_name("timeline.py").resolve()): _file_sha256(Path(__file__).with_name("timeline.py").resolve()),
            str(Path(__file__).with_name("provenance.py").resolve()): _file_sha256(Path(__file__).with_name("provenance.py").resolve()),
            str(Path(__file__).with_name("complete_bundle.py").resolve()): _file_sha256(Path(__file__).with_name("complete_bundle.py").resolve()),
            str(Path(__file__).with_name("technical_bundle.py").resolve()): _file_sha256(Path(__file__).with_name("technical_bundle.py").resolve()),
        },
        "video": video.name,
        "video_sha256": _file_sha256(video),
        "outputs": outputs,
        "contact_sheet": contact_sheet.name,
        "contact_sheet_sha256": _file_sha256(contact_sheet),
        "probe": output_probe,
        "representative_frames": representative_frames,
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
                "source_time_range_s": list(segment.source_time_range_s) if segment.source_time_range_s else None,
                "source_time_basis": segment.source_time_basis,
                "view_id": segment.view_id,
                "source_video_sha256": segment.video_sha256,
                "source_receipt_sha256": segment.receipt_sha256,
                "presentation_transform": segment.presentation_transform,
                "missing_genuine_roles": list(segment.missing_roles),
                "editorial_shared_transition_frames": _shot_transition_windows(plan, segment.shot),
            }
            for segment in segments
        ],
    }
    if mode == "diagnostic":
        manifest["diagnostic_source_identity"] = {
            "claim": "source is the repository baseline blob introduced before the canonical storyboard manifest",
            **DIAGNOSTIC_BASELINE_IDENTITY,
        }
    manifest_path = target_dir / f"{stem}_manifest.json"
    temporary_manifest = target_dir / f".{stem}_manifest.partial.json"
    temporary_manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary_manifest, manifest_path)
    return manifest


def _remove_generation(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def _publish_generation(staging: Path, target: Path) -> None:
    """Publish one validated directory generation, restoring the old one on swap failure."""

    backup = target.parent / f".{target.name}.previous-{uuid.uuid4().hex}"
    had_previous = target.exists()
    if had_previous and not target.is_dir():
        raise ValueError("presentation output path exists and is not a directory")
    try:
        if had_previous:
            os.replace(target, backup)
        os.replace(staging, target)
    except BaseException:
        if had_previous and backup.exists():
            if target.exists():
                _remove_generation(target)
            os.replace(backup, target)
        raise
    if backup.exists():
        _remove_generation(backup)


def render_presentation(
    plan_path: str | Path,
    inputs_path: str | Path,
    output_dir: str | Path,
    profile_name: str,
    mode: str,
    ffmpeg: str,
    ffprobe: str,
    rgb_capture_catalog: str | Path | None = None,
    technical_source_catalog: str | Path | None = None,
) -> dict[str, Any]:
    """Render and validate a full package before publishing one directory generation."""

    target = Path(output_dir).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and not target.is_dir():
        raise ValueError("presentation output path exists and is not a directory")
    staging = target.parent / f".{target.name}.staging-{uuid.uuid4().hex}"
    try:
        result = _render_presentation_generation(
            plan_path,
            inputs_path,
            staging,
            profile_name,
            mode,
            ffmpeg,
            ffprobe,
            rgb_capture_catalog,
            technical_source_catalog,
        )
        _publish_generation(staging, target)
        return result
    except BaseException:
        if staging.exists():
            _remove_generation(staging)
        raise


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan")
    parser.add_argument("--inputs", default=str(root / "config" / "presentation" / "diagnostic_baseline_inputs.json"))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--profile", choices=("preview", "delivery"), default="preview")
    parser.add_argument("--mode", choices=("diagnostic", "complete"), default="diagnostic")
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    plan = args.plan or str(
        root / "config" / "presentation"
        / ("diagnostic_storyboard_legacy.yaml" if args.mode == "diagnostic" else "storyboard.yaml")
    )
    result = render_presentation(plan, args.inputs, args.output_dir, args.profile, args.mode, args.ffmpeg, args.ffprobe)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
