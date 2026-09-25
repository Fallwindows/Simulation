"""Lossless representative-frame capture inside an existing Isaac app."""

from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping


MAX_CAPTURE_FRAMES = 24
MIN_CAPTURE_DIMENSION = 64
MAX_CAPTURE_DIMENSION = 3840
MIN_RT_SUBFRAMES = 1
MAX_RT_SUBFRAMES = 32


def parse_capture_frames(value: str, total_frames: int) -> tuple[int, ...]:
    """Parse a bounded, chronological CSV list of simulation frame indices."""

    if total_frames < 1:
        raise ValueError("total_frames must be positive")
    if not value.strip():
        return ()
    try:
        frames = tuple(int(token.strip()) for token in value.split(","))
    except ValueError as exc:
        raise ValueError("--capture-frames must be a comma-separated list of integers") from exc
    if len(frames) > MAX_CAPTURE_FRAMES:
        raise ValueError(f"--capture-frames supports at most {MAX_CAPTURE_FRAMES} entries")
    if len(set(frames)) != len(frames):
        raise ValueError("--capture-frames must not contain duplicates")
    if any(frame < 0 or frame >= total_frames for frame in frames):
        raise ValueError(f"--capture-frames must stay within 0..{total_frames - 1}")
    return tuple(sorted(frames))


def validate_capture_dimensions(width: int, height: int, rt_subframes: int) -> None:
    for value, name in ((width, "width"), (height, "height")):
        if not MIN_CAPTURE_DIMENSION <= value <= MAX_CAPTURE_DIMENSION:
            raise ValueError(
                f"capture {name} must be within {MIN_CAPTURE_DIMENSION}..{MAX_CAPTURE_DIMENSION}"
            )
    if not MIN_RT_SUBFRAMES <= rt_subframes <= MAX_RT_SUBFRAMES:
        raise ValueError(f"capture rt_subframes must be within {MIN_RT_SUBFRAMES}..{MAX_RT_SUBFRAMES}")


def png_dimensions(path: Path) -> tuple[int, int]:
    """Read dimensions from the lossless PNG IHDR without image dependencies."""

    with path.open("rb") as handle:
        header = handle.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError(f"capture is not a valid PNG: {path}")
    return struct.unpack(">II", header[16:24])


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class CapturedPose:
    simulation_frame: int
    timeline_seconds: float
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


class RepresentativeFrameCapture:
    """Own one dedicated Replicator render product and BasicWriter."""

    def __init__(
        self,
        output_dir: Path,
        camera_path: str,
        resolution: tuple[int, int],
        selected_frames: Iterable[int],
        rt_subframes: int,
        seed: int = 1,
        provenance: Mapping[str, object] | None = None,
    ) -> None:
        import omni.replicator.core as rep  # type: ignore

        self.output_dir = output_dir.resolve()
        self.camera_path = camera_path
        self.resolution = tuple(int(value) for value in resolution)
        self.selected_frames = tuple(selected_frames)
        self.rt_subframes = int(rt_subframes)
        self.seed = int(seed)
        self.provenance = dict(provenance or {})
        validate_capture_dimensions(*self.resolution, self.rt_subframes)
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise ValueError(f"representative capture directory must be empty: {self.output_dir}")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        rep.set_global_seed(self.seed)
        rep.orchestrator.set_capture_on_play(False)
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=str(self.output_dir))
        self._writer = rep.writers.get("BasicWriter")
        self._writer.initialize(backend=backend, rgb=True)
        self._render_product = rep.create.render_product(
            camera_path,
            self.resolution,
            name="GroceryRepresentativeCapture",
            force_new=True,
        )
        self._writer.attach([self._render_product])
        self._rep = rep
        self._poses: list[CapturedPose] = []
        self._closed = False

    def capture(self, frame: int, timeline_seconds: float, sample, timeline) -> None:
        before = float(timeline_seconds)
        self._rep.orchestrator.step(
            rt_subframes=self.rt_subframes,
            delta_time=0.0,
            pause_timeline=False,
        )
        after = float(timeline.get_current_time())
        if abs(after - before) > 1e-9:
            raise RuntimeError(
                f"representative capture advanced simulation time from {before:.9f} to {after:.9f}"
            )
        self._poses.append(
            CapturedPose(
                simulation_frame=int(frame),
                timeline_seconds=before,
                position_m=tuple(float(value) for value in sample.position_m),
                orientation_xyzw=tuple(float(value) for value in sample.orientation_xyzw),
            )
        )

    def finalize(self) -> dict[str, object]:
        if self._closed:
            raise RuntimeError("representative capture is already closed")
        try:
            self._rep.orchestrator.wait_until_complete()
        finally:
            self.close()
        png_paths = sorted(self.output_dir.rglob("*.png"))
        if len(png_paths) != len(self._poses):
            raise RuntimeError(
                f"BasicWriter produced {len(png_paths)} PNGs for {len(self._poses)} selected poses"
            )
        width, height = self.resolution
        files = []
        for pose, path in zip(self._poses, png_paths, strict=True):
            actual_dimensions = png_dimensions(path)
            if actual_dimensions != self.resolution:
                raise RuntimeError(
                    f"capture dimensions {actual_dimensions} do not match requested {self.resolution}: {path}"
                )
            files.append(
                {
                    "simulation_frame": pose.simulation_frame,
                    "timeline_seconds": pose.timeline_seconds,
                    "camera_position_m": list(pose.position_m),
                    "camera_orientation_xyzw": list(pose.orientation_xyzw),
                    "path": path.relative_to(self.output_dir).as_posix(),
                    "sha256": sha256_file(path),
                    "bytes": path.stat().st_size,
                    "width_px": actual_dimensions[0],
                    "height_px": actual_dimensions[1],
                }
            )
        manifest = {
            "schema_version": 2,
            "format": "lossless_png",
            "writer": "Isaac Replicator BasicWriter with DiskBackend",
            "camera_path": self.camera_path,
            "resolution": {"width_px": width, "height_px": height},
            "rt_subframes": self.rt_subframes,
            "delta_time_seconds": 0.0,
            "capture_on_play": False,
            "replicator_global_seed": self.seed,
            "provenance": self.provenance,
            "selected_simulation_frames": list(self.selected_frames),
            "frames": files,
        }
        manifest_path = self.output_dir / "representative_capture_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        manifest["manifest_path"] = str(manifest_path)
        manifest["manifest_sha256"] = sha256_file(manifest_path)
        return manifest

    def close(self) -> None:
        if self._closed:
            return
        try:
            self._writer.detach()
        finally:
            self._render_product.destroy()
            self._closed = True
