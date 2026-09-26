"""Build the labeled Current-vs-R7 RGB review packet from preserved raw evidence."""

from __future__ import annotations

import json
import hashlib
import math
import os
import shutil
import struct
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, __version__ as PILLOW_VERSION


ROOT = Path(__file__).resolve().parents[3]
PACKET = Path(__file__).resolve().parent
RAW = ROOT / "runs/20260925-183307101/outputs/rgb-vs-r7"
RAW_VIDEO = RAW / "current-left_r7-right_3840x1080.mp4"
RAW_SHEET = RAW / "paired-contact-sheet-6times.png"
CURRENT_INDEX = ROOT / "runs/20260925-183307101/capture/rgb_frames.jsonl"
CURRENT_CAPTURE = ROOT / "runs/20260925-183307101/capture"
CURRENT_SLAM = ROOT / "runs/20260925-183307101/slam/slam_manifest.json"
CURRENT_PERCEPTION = ROOT / "runs/20260925-183307101/perception/perception_manifest.json"
R7_CAPTURE_LOGICAL = Path(
    "runs/production-quality-20260924/scene-r7-full-capture/"
    "runs/20260925-041644489/capture"
)
R7_CAPTURE = Path(os.environ.get("R7_CAPTURE_ROOT", ROOT / R7_CAPTURE_LOGICAL))
FFMPEG = Path(
    os.environ.get("EVIDENCE_FFMPEG")
    or shutil.which("ffmpeg")
    or r"C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe"
)
FONT_BOLD = Path(os.environ.get("EVIDENCE_FONT_BOLD", r"C:\Windows\Fonts\segoeuib.ttf"))
FONT_REGULAR = Path(os.environ.get("EVIDENCE_FONT_REGULAR", r"C:\Windows\Fonts\segoeui.ttf"))

CURRENT_LABEL = "Current | capture 20260925-183307101 | source 48de461"
R7_LABEL = "R7 | capture 20260925-041644489 | source 3dc5107"
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
EXPECTED_FRAME_COUNT = 613
EXPECTED_FIRST_STAMP_S = 0.1
EXPECTED_LAST_STAMP_S = 20.5
EXPECTED_FPS = 30.0
TIMESTAMP_TOLERANCE_S = 3e-9
MIN_PANEL_PSNR_DB = 38.0
EXPECTED_PANEL_SSE_RECEIPT_SHA256 = "4726e227de7b57197871f0536566cb4e6650d827ff526d74a2b8b1c9df5a1efe"
EXPECTED_FIXED_INPUT_SHA256 = {
    "raw_comparison_video": "d0d1a2796b2b2e9c53d2e3633ac8c3f46cb995ba6002a83121ecf3909057aae4",
    "raw_six_sample_sheet": "b8cfcb5a5c4519b253b25da253809da37dced9065bed8ea583f03da5caa3bbec",
    "current_capture_manifest": "49de286dc0a26220ebfa9e3d141cb972e35bcdf5cc3e33e9ef506fd8474bfe3b",
    "current_rgb_video": "465cb43619a3ac58bea359840ae779ffc5ed588191832d9448d568dd2e834738",
    "current_rgb_index": "69f171b691d35061185c31378598f0fc4dfdc7c44b2e260e404d452fbcf88915",
    "current_effective_config": "6e8882240a061d288013b3a21db0fc737fdaf515e946a2db0cbc30e401584d86",
    "r7_capture_manifest": "76da43e3acfea0559b81cd4462667e95d7f1bdaacf00471b9e001db114d2174b",
    "r7_rgb_video": "b36904fde78e44e65107d7ef3fe3eff5342f8b0d011fd589c3238e13682d1b6d",
    "r7_rgb_index": "69f171b691d35061185c31378598f0fc4dfdc7c44b2e260e404d452fbcf88915",
    "r7_effective_config": "0bac353757bb1674c8bf15a3424f347cf201aa1a275da4a5a54f451d64f77de1",
    "current_slam_manifest": "803ca96a2fcb4e8d294ed6adc6f73a366e9b2fea83d343c7d2087ce435b53ad4",
    "current_perception_manifest": "ce05a2e0654fb3e00b264a9aea1b065ae1eca1a2e7513e44b4c1fe4160119295",
}


def validate_runtime() -> None:
    required = {
        "raw comparison video": RAW_VIDEO,
        "raw six-sample sheet": RAW_SHEET,
        "current capture manifest": CURRENT_CAPTURE / "capture_manifest.json",
        "current RGB video": CURRENT_CAPTURE / "rgb_camera.mp4",
        "current RGB index": CURRENT_INDEX,
        "current effective config": CURRENT_CAPTURE / "effective_config.json",
        "current SLAM manifest": CURRENT_SLAM,
        "current perception manifest": CURRENT_PERCEPTION,
        "R7 capture manifest": R7_CAPTURE / "capture_manifest.json",
        "R7 RGB video": R7_CAPTURE / "rgb_camera.mp4",
        "R7 RGB index": R7_CAPTURE / "rgb_frames.jsonl",
        "R7 effective config": R7_CAPTURE / "effective_config.json",
        "FFmpeg executable": FFMPEG,
        "bold font": FONT_BOLD,
        "regular font": FONT_REGULAR,
    }
    missing = [f"{label}: {path}" for label, path in required.items() if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Evidence rebuild prerequisites are missing:\n- "
            + "\n- ".join(missing)
            + "\nSet R7_CAPTURE_ROOT to <original-checkout>/"
            + R7_CAPTURE_LOGICAL.as_posix()
            + "; EVIDENCE_FFMPEG, EVIDENCE_FONT_BOLD, and EVIDENCE_FONT_REGULAR "
            "override tool/font discovery."
        )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(
    path: Path,
    relative_to: Path | None = None,
    logical_path: str | None = None,
) -> dict[str, object]:
    return {
        "path": logical_path or (
            str(path.relative_to(relative_to)).replace("\\", "/") if relative_to else path.name
        ),
        "sha256": sha256(path),
        "size_bytes": path.stat().st_size,
    }


def load_timestamps() -> list[float]:
    return [json.loads(line)["stamp_s"] for line in CURRENT_INDEX.read_text(encoding="utf-8").splitlines()]


def fixed_input_paths() -> dict[str, Path]:
    return {
        "raw_comparison_video": RAW_VIDEO,
        "raw_six_sample_sheet": RAW_SHEET,
        "current_capture_manifest": CURRENT_CAPTURE / "capture_manifest.json",
        "current_rgb_video": CURRENT_CAPTURE / "rgb_camera.mp4",
        "current_rgb_index": CURRENT_INDEX,
        "current_effective_config": CURRENT_CAPTURE / "effective_config.json",
        "r7_capture_manifest": R7_CAPTURE / "capture_manifest.json",
        "r7_rgb_video": R7_CAPTURE / "rgb_camera.mp4",
        "r7_rgb_index": R7_CAPTURE / "rgb_frames.jsonl",
        "r7_effective_config": R7_CAPTURE / "effective_config.json",
        "current_slam_manifest": CURRENT_SLAM,
        "current_perception_manifest": CURRENT_PERCEPTION,
    }


def verify_fixed_inputs() -> tuple[list[float], dict[str, object]]:
    """Fail closed unless every fixed input and capture/index binding is exact."""
    paths = fixed_input_paths()
    actual_sha256 = {name: sha256(path) for name, path in paths.items()}
    if actual_sha256 != EXPECTED_FIXED_INPUT_SHA256:
        mismatches = {
            name: {
                "expected": EXPECTED_FIXED_INPUT_SHA256.get(name),
                "actual": actual_sha256.get(name),
            }
            for name in sorted(set(EXPECTED_FIXED_INPUT_SHA256) | set(actual_sha256))
            if EXPECTED_FIXED_INPUT_SHA256.get(name) != actual_sha256.get(name)
        }
        raise RuntimeError(f"fixed RGB evidence input hash mismatch: {mismatches}")

    current_index_bytes = paths["current_rgb_index"].read_bytes()
    r7_index_bytes = paths["r7_rgb_index"].read_bytes()
    if current_index_bytes != r7_index_bytes:
        raise RuntimeError("Current and R7 rgb_frames.jsonl bytes differ")
    try:
        index_rows = [
            json.loads(line)
            for line in current_index_bytes.decode("utf-8").splitlines()
        ]
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"invalid RGB frame index: {error}") from error
    if len(index_rows) != EXPECTED_FRAME_COUNT:
        raise RuntimeError(
            f"unexpected RGB frame-index row count: {len(index_rows)} != {EXPECTED_FRAME_COUNT}"
        )

    timestamps: list[float] = []
    max_schedule_error_s = 0.0
    previous_stamp = -math.inf
    expected_index_keys = {
        "encoding", "frame_id", "frame_index", "height", "stamp_s", "width"
    }
    for expected_index, row in enumerate(index_rows):
        if set(row) != expected_index_keys:
            raise RuntimeError(
                f"unexpected RGB frame-index schema at row {expected_index}: "
                f"{sorted(row)}"
            )
        if row.get("frame_index") != expected_index:
            raise RuntimeError(
                f"non-contiguous RGB frame index at row {expected_index}: "
                f"{row.get('frame_index')!r}"
            )
        try:
            stamp = float(row["stamp_s"])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f"invalid RGB timestamp at row {expected_index}") from error
        if not math.isfinite(stamp) or stamp <= previous_stamp:
            raise RuntimeError(
                f"RGB timestamps must be finite and strictly increasing; row "
                f"{expected_index} has {stamp!r} after {previous_stamp!r}"
            )
        expected_stamp = EXPECTED_FIRST_STAMP_S + expected_index / EXPECTED_FPS
        schedule_error_s = abs(stamp - expected_stamp)
        if schedule_error_s > TIMESTAMP_TOLERANCE_S:
            raise RuntimeError(
                f"RGB timestamp at row {expected_index} is off the 30-fps schedule: "
                f"{stamp:.9f} vs {expected_stamp:.9f}"
            )
        if (
            row.get("width") != FRAME_WIDTH
            or row.get("height") != FRAME_HEIGHT
            or row.get("encoding") != "rgb8"
            or row.get("frame_id") != "camera_optical_frame"
        ):
            raise RuntimeError(f"unexpected RGB frame metadata at row {expected_index}: {row}")
        timestamps.append(stamp)
        previous_stamp = stamp
        max_schedule_error_s = max(max_schedule_error_s, schedule_error_s)

    if (
        abs(timestamps[0] - EXPECTED_FIRST_STAMP_S) > TIMESTAMP_TOLERANCE_S
        or abs(timestamps[-1] - EXPECTED_LAST_STAMP_S) > TIMESTAMP_TOLERANCE_S
    ):
        raise RuntimeError(
            f"unexpected RGB timestamp range: {timestamps[0]}..{timestamps[-1]}"
        )

    expected_captures = {
        "current": {
            "capture_id": "20260925-183307101",
            "status": "complete",
            "git_sha": "48de461b42d5e0945b21432cef9f5523cc5b0874",
            "git_tree": "74fc8d8457799a49b0e2ddaccef139e86aac99c1",
            "capture_sha256": "a89471e34fb490727c182601c631d4a2ba95bb37499cbdddf03a1829829579b0",
        },
        "r7": {
            "capture_id": "20260925-041644489",
            "status": "complete",
            "git_sha": "3dc5107e8fedee3835259282e1655749ec7438c0",
            "git_tree": "048f7d9d50d6339864ac0bc9e4b2d3a4fbb7ccb4",
            "capture_sha256": "5429fb8cfbbae8bc5bca6c2c688515eb49e3b81c5f8eae2bad1de836730e86d9",
        },
    }
    capture_paths = {
        "current": paths["current_capture_manifest"],
        "r7": paths["r7_capture_manifest"],
    }
    expected_fov_degrees = {"current": 75.0, "r7": 90.0}
    capture_receipt: dict[str, dict[str, object]] = {}
    for name, path in capture_paths.items():
        try:
            capture = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"invalid {name} capture manifest: {error}") from error
        identity = {key: capture.get(key) for key in expected_captures[name]}
        if identity != expected_captures[name]:
            raise RuntimeError(
                f"{name} capture identity mismatch: expected={expected_captures[name]}, "
                f"actual={identity}"
            )
        rgb = capture.get("rgb", {})
        expected_rgb = {
            "video": "rgb_camera.mp4",
            "timestamp_index": "rgb_frames.jsonl",
            "frame_count": EXPECTED_FRAME_COUNT,
            "first_stamp_s": EXPECTED_FIRST_STAMP_S,
            "last_stamp_s": EXPECTED_LAST_STAMP_S,
            "width_px": FRAME_WIDTH,
            "height_px": FRAME_HEIGHT,
            "fps": EXPECTED_FPS,
        }
        actual_rgb = {key: rgb.get(key) for key in expected_rgb}
        if actual_rgb != expected_rgb:
            raise RuntimeError(
                f"{name} capture RGB binding mismatch: expected={expected_rgb}, "
                f"actual={actual_rgb}"
            )
        expected_file_sha256 = {
            "rgb_camera.mp4": actual_sha256[f"{name}_rgb_video"],
            "rgb_frames.jsonl": actual_sha256[f"{name}_rgb_index"],
            "effective_config.json": actual_sha256[f"{name}_effective_config"],
        }
        declared_files = {
            item.get("path"): item.get("sha256")
            for item in capture.get("files", [])
            if item.get("path") in expected_file_sha256
        }
        if declared_files != expected_file_sha256:
            raise RuntimeError(
                f"{name} capture file binding mismatch: expected={expected_file_sha256}, "
                f"actual={declared_files}"
            )
        effective_config_path = paths[f"{name}_effective_config"]
        try:
            effective_config = json.loads(effective_config_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"invalid {name} effective config: {error}") from error
        camera = effective_config.get("camera", {})
        expected_camera = {
            "width_px": FRAME_WIDTH,
            "height_px": FRAME_HEIGHT,
            "fps": EXPECTED_FPS,
            "horizontal_fov_deg": expected_fov_degrees[name],
        }
        actual_camera = {key: camera.get(key) for key in expected_camera}
        if actual_camera != expected_camera:
            raise RuntimeError(
                f"{name} effective camera binding mismatch: expected={expected_camera}, "
                f"actual={actual_camera}"
            )
        if name == "current":
            expected_source_bindings = {
                "git_commit": expected_captures[name]["git_sha"],
                "git_tree": expected_captures[name]["git_tree"],
            }
            source_bindings = effective_config.get("source_bindings", {})
            actual_source_bindings = {
                key: source_bindings.get(key) for key in expected_source_bindings
            }
            if actual_source_bindings != expected_source_bindings:
                raise RuntimeError(
                    "current effective-config source binding mismatch: "
                    f"expected={expected_source_bindings}, actual={actual_source_bindings}"
                )
        capture_receipt[name] = {
            "identity": identity,
            "rgb": actual_rgb,
            "declared_file_sha256": declared_files,
            "effective_camera": actual_camera,
        }

    current_capture = expected_captures["current"]
    try:
        slam = json.loads(paths["current_slam_manifest"].read_text(encoding="utf-8"))
        perception = json.loads(
            paths["current_perception_manifest"].read_text(encoding="utf-8")
        )
    except json.JSONDecodeError as error:
        raise RuntimeError(f"invalid SLAM/perception receipt: {error}") from error
    expected_slam = {
        "status": "complete",
        "experiment": "offline_slam",
        "capture_id": current_capture["capture_id"],
        "capture_sha256": current_capture["capture_sha256"],
        "git_sha": "b9ade5b904c8c0cbea959650cc86620957d0de5e",
        "ground_truth_subscribed": False,
        "publish_map_service_acknowledged": True,
    }
    actual_slam = {key: slam.get(key) for key in expected_slam}
    if actual_slam != expected_slam:
        raise RuntimeError(
            f"SLAM receipt identity/linkage mismatch: expected={expected_slam}, "
            f"actual={actual_slam}"
        )
    expected_perception = {
        "status": "complete",
        "capture_id": current_capture["capture_id"],
        "capture_manifest_sha256": actual_sha256["current_capture_manifest"],
        "capture_sha256": current_capture["capture_sha256"],
        "git_sha": "b9ade5b904c8c0cbea959650cc86620957d0de5e",
        "frame_count": EXPECTED_FRAME_COUNT,
        "frames": "../capture/rgb_frames.jsonl",
        "ground_truth_consumed": False,
    }
    actual_perception = {key: perception.get(key) for key in expected_perception}
    if actual_perception != expected_perception:
        raise RuntimeError(
            f"perception receipt identity/linkage mismatch: expected={expected_perception}, "
            f"actual={actual_perception}"
        )
    expected_perception_capture_input = {
        "capture_id": current_capture["capture_id"],
        "capture_sha256": current_capture["capture_sha256"],
        "manifest": {
            "path": "../capture/capture_manifest.json",
            "sha256": actual_sha256["current_capture_manifest"],
        },
    }
    actual_perception_capture_input = perception.get("inputs", {}).get("capture")
    if actual_perception_capture_input != expected_perception_capture_input:
        raise RuntimeError(
            "perception nested capture linkage mismatch: "
            f"expected={expected_perception_capture_input}, "
            f"actual={actual_perception_capture_input}"
        )

    return timestamps, {
        "status": "passed",
        "rule": (
            "All 12 declared raw inputs must match pinned SHA-256 values before any "
            "output write. Current and R7 RGB indices must be byte-identical and contain "
            "exactly 613 schema-exact, contiguous rgb8 camera_optical_frame 1920x1080 "
            "rows with finite, strictly increasing "
            "timestamps on the 30-fps 0.1..20.5 s schedule within 3e-9 s. Each pinned "
            "capture manifest must bind the expected identity, RGB/config files, "
            "dimensions, FOV, count, rate, and timestamp range. The pinned SLAM and "
            "perception receipts must be complete and exactly linked to Current capture."
        ),
        "artifact_count": len(actual_sha256),
        "expected_artifact_sha256": EXPECTED_FIXED_INPUT_SHA256,
        "artifact_sha256": actual_sha256,
        "rgb_index": {
            "current_and_r7_bytes_identical": True,
            "sha256": actual_sha256["current_rgb_index"],
            "schema_keys": sorted(expected_index_keys),
            "frame_id": "camera_optical_frame",
            "frame_count": len(index_rows),
            "frame_indices_contiguous": True,
            "timestamps_finite_and_strictly_increasing": True,
            "fps": EXPECTED_FPS,
            "first_stamp_s": timestamps[0],
            "last_stamp_s": timestamps[-1],
            "timestamp_tolerance_s": TIMESTAMP_TOLERANCE_S,
            "maximum_schedule_error_s": max_schedule_error_s,
        },
        "capture_binding": capture_receipt,
        "pipeline_receipt_binding": {
            "slam": actual_slam,
            "perception": {
                **actual_perception,
                "capture_input": actual_perception_capture_input,
            },
        },
    }


def _read_exact(stream: object, byte_count: int) -> bytes:
    payload = bytearray()
    while len(payload) < byte_count:
        chunk = stream.read(byte_count - len(payload))  # type: ignore[attr-defined]
        if not chunk:
            break
        payload.extend(chunk)
    return bytes(payload)


def verify_raw_comparison_sources() -> dict[str, object]:
    """Verify every comparison panel frame against its exact declared RGB source."""
    sources = {
        "comparison": RAW_VIDEO,
        "current": CURRENT_CAPTURE / "rgb_camera.mp4",
        "r7": R7_CAPTURE / "rgb_camera.mp4",
    }
    expected_artifact_sha256 = {
        "comparison": EXPECTED_FIXED_INPUT_SHA256["raw_comparison_video"],
        "current": EXPECTED_FIXED_INPUT_SHA256["current_rgb_video"],
        "r7": EXPECTED_FIXED_INPUT_SHA256["r7_rgb_video"],
    }
    actual_artifact_sha256 = {name: sha256(path) for name, path in sources.items()}
    if actual_artifact_sha256 != expected_artifact_sha256:
        raise RuntimeError(
            "panel-source verification artifact hash mismatch: "
            f"expected={expected_artifact_sha256}, "
            f"actual={actual_artifact_sha256}"
        )
    commands = {
        name: [
            str(FFMPEG), "-hide_banner", "-loglevel", "error", "-i", str(path),
            "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
        ]
        for name, path in sources.items()
    }
    processes = {
        name: subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for name, command in commands.items()
    }
    for name, process in processes.items():
        if process.stdout is None or process.stderr is None:
            for candidate in processes.values():
                candidate.kill()
            raise RuntimeError(f"could not open FFmpeg verification stream: {name}")

    panel_bytes = FRAME_WIDTH * FRAME_HEIGHT * 3
    comparison_bytes = panel_bytes * 2
    pixel_values = panel_bytes
    max_sse = math.floor(pixel_values * (255.0 ** 2) / (10.0 ** (MIN_PANEL_PSNR_DB / 10.0)))
    receipt_digest = hashlib.sha256()
    psnr_values: dict[str, list[float]] = {"current": [], "r7": []}
    try:
        for frame_index in range(EXPECTED_FRAME_COUNT):
            comparison_payload = _read_exact(processes["comparison"].stdout, comparison_bytes)
            current_payload = _read_exact(processes["current"].stdout, panel_bytes)
            r7_payload = _read_exact(processes["r7"].stdout, panel_bytes)
            sizes = {
                "comparison": len(comparison_payload),
                "current": len(current_payload),
                "r7": len(r7_payload),
            }
            expected_sizes = {
                "comparison": comparison_bytes,
                "current": panel_bytes,
                "r7": panel_bytes,
            }
            if sizes != expected_sizes:
                raise RuntimeError(
                    f"truncated panel-source verification at frame {frame_index}: {sizes}"
                )

            comparison = np.frombuffer(comparison_payload, dtype=np.uint8).reshape(
                FRAME_HEIGHT, FRAME_WIDTH * 2, 3
            )
            current = np.frombuffer(current_payload, dtype=np.uint8).reshape(
                FRAME_HEIGHT, FRAME_WIDTH, 3
            )
            r7 = np.frombuffer(r7_payload, dtype=np.uint8).reshape(
                FRAME_HEIGHT, FRAME_WIDTH, 3
            )
            panel_sse: dict[str, int] = {}
            for panel_name, panel, source in (
                ("current", comparison[:, :FRAME_WIDTH, :], current),
                ("r7", comparison[:, FRAME_WIDTH:, :], r7),
            ):
                delta = panel.astype(np.int16) - source.astype(np.int16)
                sse = int(np.sum(delta.astype(np.int32) ** 2, dtype=np.int64))
                if sse > max_sse:
                    mse = sse / pixel_values
                    psnr = 10.0 * math.log10((255.0 ** 2) / mse)
                    raise RuntimeError(
                        f"{panel_name} panel source mismatch at frame {frame_index}: "
                        f"PSNR {psnr:.6f} dB is below {MIN_PANEL_PSNR_DB:.1f} dB"
                    )
                panel_sse[panel_name] = sse
                psnr_values[panel_name].append(
                    float("inf") if sse == 0 else 10.0 * math.log10(
                        (255.0 ** 2) / (sse / pixel_values)
                    )
                )
            receipt_digest.update(struct.pack(
                "<IQQ", frame_index, panel_sse["current"], panel_sse["r7"]
            ))

        trailing = {
            name: len(_read_exact(process.stdout, 1))
            for name, process in processes.items()
        }
        if any(trailing.values()):
            raise RuntimeError(
                f"panel-source verification found frames beyond {EXPECTED_FRAME_COUNT}: {trailing}"
            )
    except BaseException:
        for process in processes.values():
            if process.poll() is None:
                process.kill()
        raise

    failures: list[str] = []
    for name, process in processes.items():
        stderr = process.stderr.read().decode("utf-8", errors="replace")
        return_code = process.wait()
        if return_code != 0:
            failures.append(f"{name} decoder exited {return_code}: {stderr}")
    if failures:
        raise RuntimeError("; ".join(failures))

    logical_inputs = {
        "comparison": "runs/20260925-183307101/outputs/rgb-vs-r7/current-left_r7-right_3840x1080.mp4",
        "current": "runs/20260925-183307101/capture/rgb_camera.mp4",
        "r7": f"{R7_CAPTURE_LOGICAL.as_posix()}/rgb_camera.mp4",
    }
    receipt_sha256 = receipt_digest.hexdigest()
    if receipt_sha256 != EXPECTED_PANEL_SSE_RECEIPT_SHA256:
        raise RuntimeError(
            "panel-source verification SSE receipt mismatch: "
            f"{receipt_sha256} != {EXPECTED_PANEL_SSE_RECEIPT_SHA256}"
        )

    return {
        "status": "passed",
        "rule": (
            "Every one of 613 synchronized decoded frames is compared at 1920x1080 RGB24; "
            "the comparison left half must match Current and the right half must match R7 "
            "with per-frame PSNR >= 38.0 dB. The three artifact hashes and exact "
            "per-frame SSE receipt digest must match their pinned values. Missing, extra, "
            "truncated, substituted, or lower-PSNR frames fail the rebuild."
        ),
        "decoder_argv": {
            name: [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-i", logical_inputs[name],
                "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
            ]
            for name in sources
        },
        "ffmpeg_version": subprocess.run(
            [str(FFMPEG), "-version"], check=True, capture_output=True, text=True
        ).stdout.splitlines()[0],
        "frame_count": EXPECTED_FRAME_COUNT,
        "frame_width": FRAME_WIDTH,
        "frame_height": FRAME_HEIGHT,
        "minimum_allowed_psnr_db": MIN_PANEL_PSNR_DB,
        "observed_psnr_db": {
            name: {
                "minimum": min(values),
                "average": sum(values) / len(values),
            }
            for name, values in psnr_values.items()
        },
        "per_frame_sse_receipt_sha256": receipt_sha256,
        "expected_per_frame_sse_receipt_sha256": EXPECTED_PANEL_SSE_RECEIPT_SHA256,
        "artifact_sha256": {
            "raw_comparison_output": actual_artifact_sha256["comparison"],
            "current_rgb_source": actual_artifact_sha256["current"],
            "r7_rgb_source": actual_artifact_sha256["r7"],
        },
    }


def match_sheet_frames(sheet: Image.Image) -> list[int]:
    rgb = np.asarray(sheet.convert("RGB"))
    cells = [rgb[y:y + 360, x:x + 1280] for y in (8, 376, 744) for x in (8, 1296)]
    best: list[tuple[float, int]] = [(float("inf"), -1) for _ in cells]
    command = [
        str(FFMPEG), "-hide_banner", "-loglevel", "error", "-i", str(RAW_VIDEO),
        "-vf", "scale=1280:360:flags=area", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if process.stdout is None or process.stderr is None:
        raise RuntimeError("could not open FFmpeg comparison stream")
    frame_bytes = 1280 * 360 * 3
    frame_index = 0
    while True:
        payload = process.stdout.read(frame_bytes)
        if not payload:
            break
        if len(payload) != frame_bytes:
            process.kill()
            raise RuntimeError(f"truncated FFmpeg comparison frame: {len(payload)} bytes")
        frame = np.frombuffer(payload, dtype=np.uint8).reshape(360, 1280, 3).astype(np.float32)
        for cell_index, cell in enumerate(cells):
            mse = float(np.mean((frame - cell.astype(np.float32)) ** 2))
            if mse < best[cell_index][0]:
                best[cell_index] = (mse, frame_index)
        frame_index += 1
    stderr = process.stderr.read().decode("utf-8", errors="replace")
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"FFmpeg comparison decode failed ({return_code}): {stderr}")
    matches = [frame for _, frame in best]
    if frame_index != 613 or any(frame < 0 for frame in matches):
        raise RuntimeError(f"unexpected comparison video shape: frames={frame_index}, matches={matches}")
    return matches


def draw_box(draw: ImageDraw.ImageDraw, xy: tuple[int, int, int, int], fill: tuple[int, int, int, int]) -> None:
    draw.rectangle(xy, fill=fill)


def build_sheet(timestamps: list[float]) -> list[dict[str, object]]:
    sheet = Image.open(RAW_SHEET).convert("RGBA")
    matches = match_sheet_frames(sheet)
    overlay = Image.new("RGBA", sheet.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    panel_font = ImageFont.truetype(str(FONT_BOLD), 17)
    time_font = ImageFont.truetype(str(FONT_BOLD), 19)
    small_font = ImageFont.truetype(str(FONT_REGULAR), 15)
    records: list[dict[str, object]] = []
    for sample_index, ((x, y), frame_index) in enumerate(zip(
        [(x, y) for y in (8, 376, 744) for x in (8, 1296)], matches,
    ), start=1):
        stamp = float(timestamps[frame_index])
        draw_box(draw, (x, y, x + 640, y + 48), (7, 20, 34, 218))
        draw_box(draw, (x + 640, y, x + 1280, y + 48), (47, 23, 8, 218))
        draw.text((x + 12, y + 5), CURRENT_LABEL, font=panel_font, fill=(255, 255, 255, 255))
        draw.text((x + 652, y + 5), R7_LABEL, font=panel_font, fill=(255, 255, 255, 255))
        time_text = f"Sample {sample_index}/6 | frame {frame_index:03d} | synchronized sensor stamp {stamp:.9f} s"
        bbox = draw.textbbox((0, 0), time_text, font=time_font)
        text_width = bbox[2] - bbox[0]
        draw_box(draw, (x + 220, y + 316, x + 1060, y + 356), (0, 0, 0, 205))
        draw.text((x + 640 - text_width / 2, y + 319), time_text, font=time_font, fill=(255, 255, 255, 255))
        draw.text((x + 12, y + 28), "75 degree horizontal FOV", font=small_font, fill=(191, 226, 255, 255))
        draw.text((x + 652, y + 28), "90 degree horizontal FOV", font=small_font, fill=(255, 218, 181, 255))
        records.append({"sample": sample_index, "frame_index": frame_index, "timestamp_s": stamp})
    Image.alpha_composite(sheet, overlay).convert("RGB").save(
        PACKET / "current-vs-r7-labeled-6-samples.png", optimize=True
    )
    return records


def ffmpeg_escape(value: str) -> str:
    return value.replace("\\", "/").replace(":", r"\:").replace("'", r"\'")


def build_video(timestamps: list[float]) -> None:
    font = ffmpeg_escape(str(FONT_BOLD))
    common = f"fontfile='{font}':fontcolor=white:borderw=2:bordercolor=black"
    filters = [
        "drawbox=x=0:y=0:w=1920:h=82:color=0x071422@0.82:t=fill",
        "drawbox=x=1920:y=0:w=1920:h=82:color=0x2f1708@0.82:t=fill",
        f"drawtext={common}:fontsize=34:text='{CURRENT_LABEL}':x=24:y=18",
        f"drawtext={common}:fontsize=34:text='{R7_LABEL}':x=1944:y=18",
        "drawbox=x=1160:y=993:w=1520:h=70:color=black@0.72:t=fill",
    ]
    for frame_index, stamp in enumerate(timestamps):
        label = (
            f"Frame {frame_index:03d} / {len(timestamps) - 1:03d}  |  "
            f"synchronized sensor stamp {stamp:.9f} s"
        )
        filters.append(
            f"drawtext={common}:fontsize=34:text='{label}':"
            f"x=(w-text_w)/2:y=1010:enable='eq(n,{frame_index})'"
        )
    filter_path = PACKET / "video-label-filter.txt"
    filter_path.write_text(",\n".join(filters) + "\n", encoding="utf-8")
    command = [
        str(FFMPEG), "-y", "-hide_banner", "-loglevel", "warning",
        "-i", str(RAW_VIDEO), "-filter_script:v", filter_path.name,
        "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "16",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(PACKET / "current-vs-r7-labeled-3840x1080.mp4"),
    ]
    try:
        subprocess.run(command, check=True, cwd=PACKET)
    finally:
        filter_path.unlink(missing_ok=True)


def main() -> None:
    validate_runtime()
    timestamps, fixed_input_verification = verify_fixed_inputs()
    source_verification = verify_raw_comparison_sources()
    samples = build_sheet(timestamps)
    build_video(timestamps)
    (PACKET / "sample-times.json").write_text(
        json.dumps({
            "timestamp_source": "identical capture rgb_frames.jsonl rows",
            "current_rgb_index_sha256": sha256(CURRENT_INDEX),
            "r7_rgb_index_sha256": sha256(R7_CAPTURE / "rgb_frames.jsonl"),
            "samples": samples,
        }, indent=2) + "\n", encoding="utf-8"
    )

    for stale_copy in (
        "current-capture-manifest.json",
        "r7-capture-manifest.json",
        "current-slam-manifest.json",
        "current-perception-manifest.json",
    ):
        (PACKET / stale_copy).unlink(missing_ok=True)

    raw_inputs = {
        "raw_comparison_video": file_record(RAW_VIDEO, logical_path="runs/20260925-183307101/outputs/rgb-vs-r7/current-left_r7-right_3840x1080.mp4"),
        "raw_six_sample_sheet": file_record(RAW_SHEET, logical_path="runs/20260925-183307101/outputs/rgb-vs-r7/paired-contact-sheet-6times.png"),
        "current_capture_manifest": file_record(CURRENT_CAPTURE / "capture_manifest.json", logical_path="runs/20260925-183307101/capture/capture_manifest.json"),
        "current_rgb_video": file_record(CURRENT_CAPTURE / "rgb_camera.mp4", logical_path="runs/20260925-183307101/capture/rgb_camera.mp4"),
        "current_rgb_index": file_record(CURRENT_INDEX, logical_path="runs/20260925-183307101/capture/rgb_frames.jsonl"),
        "current_effective_config": file_record(CURRENT_CAPTURE / "effective_config.json", logical_path="runs/20260925-183307101/capture/effective_config.json"),
        "r7_capture_manifest": file_record(R7_CAPTURE / "capture_manifest.json", logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/capture_manifest.json"),
        "r7_rgb_video": file_record(R7_CAPTURE / "rgb_camera.mp4", logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/rgb_camera.mp4"),
        "r7_rgb_index": file_record(R7_CAPTURE / "rgb_frames.jsonl", logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/rgb_frames.jsonl"),
        "r7_effective_config": file_record(R7_CAPTURE / "effective_config.json", logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/effective_config.json"),
        "current_slam_manifest": file_record(CURRENT_SLAM, logical_path="runs/20260925-183307101/slam/slam_manifest.json"),
        "current_perception_manifest": file_record(CURRENT_PERCEPTION, logical_path="runs/20260925-183307101/perception/perception_manifest.json"),
    }
    labeled_video = file_record(PACKET / "current-vs-r7-labeled-3840x1080.mp4", PACKET)
    labeled_sheet = file_record(PACKET / "current-vs-r7-labeled-6-samples.png", PACKET)
    sample_rows = "\n".join(
        f"| {item['sample']} | {item['frame_index']} | {item['timestamp_s']:.9f} |"
        for item in samples
    )
    readme = f"""# Current vs R7 RGB visual evidence

This packet labels the preserved side-by-side RGB comparison of **Current** capture
`20260925-183307101` at source `48de461b42d5e0945b21432cef9f5523cc5b0874`
and **R7** capture `20260925-041644489` at source
`3dc5107e8fedee3835259282e1655749ec7438c0`. Current is the left panel and R7 is
the right panel.

## Review media

- `current-vs-r7-labeled-3840x1080.mp4` — 613 synchronized 30 fps frames with
  panel/run/source labels and each frame's exact sensor timestamp from the identical
  `rgb_frames.jsonl` rows. SHA-256 `{labeled_video['sha256']}`.
- `current-vs-r7-labeled-6-samples.png` — six labeled matched-frame samples.
  SHA-256 `{labeled_sheet['sha256']}`.
- `sample-times.json` — machine-readable matched frame indices and timestamps.

| Sample | Frame index | Sensor timestamp (s) |
| ---: | ---: | ---: |
{sample_rows}

## Provenance and scope

The unlabelled inputs remain unchanged under
`runs/20260925-183307101/outputs/rgb-vs-r7/`. The raw comparison video SHA-256 is
`{raw_inputs['raw_comparison_video']['sha256']}` and the raw six-sample sheet SHA-256
is `{raw_inputs['raw_six_sample_sheet']['sha256']}`. The packet manifest binds these
inputs, both exact capture manifests, both RGB videos and timestamp indices, and all
packet outputs.

Before any packet output is written, all 12 declared raw inputs must match their
pinned SHA-256 values. Both RGB frame indices must be byte-identical and must contain
exactly 613 contiguous rows with finite, strictly increasing timestamps on the
30 fps 0.1 through 20.5 second schedule. The pinned capture manifests must also bind
the expected capture identity, source commit/tree/seal, RGB/config files, dimensions,
FOV, frame count, rate, and timestamp range. The pinned SLAM and perception receipts
must remain complete and linked to the exact Current capture. Any mismatch aborts
the rebuild.

Before labeling, the builder decodes all 613 frames of the raw comparison and both
hash-bound RGB sources as RGB24. Every left panel frame must match Current and every
right panel frame must match R7 at least 38.0 dB PSNR; the three artifact hashes and
the exact per-frame SSE receipt digest are pinned. Missing, extra, truncated,
substituted, or lower-PSNR frames fail the rebuild. `manifest.json` records the decoder command,
FFmpeg version, artifact hashes, observed minima/averages, and an exact per-frame SSE
receipt digest.

The exact producer capture, SLAM, and perception receipts remain in the local logical
run paths recorded by `manifest.json`. They are referenced by raw-byte SHA-256 and
size but omitted from this portable packet because their otherwise valid provenance
contains machine-local absolute paths. The original receipts in `runs/` are unchanged.

This is **RGB visual-review evidence only**. Current uses a 75 degree horizontal FOV;
R7 uses 90 degrees, so framing and apparent scale are not directly equivalent. The
comparison supports inspection of visible composition and appearance only. It does
not establish spatial correspondence, LiDAR quality, SLAM accuracy, perception
accuracy, or final-film acceptance.

No generative image or video model was used to create or alter this packet, so there
is no generation receipt. Pillow adds the sheet labels and FFmpeg adds the video
labels while preserving the unlabelled inputs. Visible synthetic limitations remain,
including procedural package shapes and labels, repeated facings, simplified
materials, and flat artificial lighting.

## Current pipeline checkpoint

The current run has sealed complete SLAM manifest SHA-256
`803ca96a2fcb4e8d294ed6adc6f73a366e9b2fea83d343c7d2087ce435b53ad4` from replay
source `b9ade5b904c8c0cbea959650cc86620957d0de5e`, attempt
`20260926T042347915Z-d146ed0c1bbd4edfb296f655a0b5fe36` (204 nodes, exact 204-stamp
coverage, 203 directed Neighbor links, ground-truth subscription false), and a complete
perception manifest (613 RGB frames, 204 raw/valid/usable LiDAR scans, 82,097 projected
points, 604 localized inventory rows, ground-truth consumption false). The original
manifests are bound by hash in `manifest.json`. RGB and perception are complete; the technical
preview is independently reviewed (see `technical-lidar-preview/README.md`); it remains preview-only. This packet does not claim final production
completion.

## Rebuild requirements

Run `build_evidence.py` from a Python environment with NumPy and Pillow. FFmpeg must
provide H.264 decoding, `libx264`, and `drawtext`. The script uses `ffmpeg` from `PATH`
when available, then the approved Isaac ROS workspace fallback. Set `EVIDENCE_FFMPEG`
to use another executable and `EVIDENCE_FONT_BOLD` / `EVIDENCE_FONT_REGULAR` to use
other TrueType fonts.

This g02 worktree does not contain the preserved R7 capture. Set `R7_CAPTURE_ROOT`
to the directory at this exact logical suffix under the original checkout:
`runs/production-quality-20260924/scene-r7-full-capture/runs/20260925-041644489/capture`.
That directory must contain `capture_manifest.json`, `rgb_camera.mp4`,
`rgb_frames.jsonl`, and `effective_config.json`. The script's default checks the same
logical path below its checkout and fails with a prerequisite list when it is absent.
"""
    (PACKET / "README.md").write_text(readme, encoding="utf-8", newline="\n")

    ffmpeg_version = source_verification["ffmpeg_version"]
    outputs = {
        name: file_record(PACKET / name, PACKET)
        for name in [
            "current-vs-r7-labeled-3840x1080.mp4",
            "current-vs-r7-labeled-6-samples.png",
            "sample-times.json",
            "README.md",
            "build_evidence.py",
        ]
    }
    current_capture_receipt = json.loads((CURRENT_CAPTURE / "capture_manifest.json").read_text(encoding="utf-8"))
    r7_capture_receipt = json.loads((R7_CAPTURE / "capture_manifest.json").read_text(encoding="utf-8"))
    slam_receipt = json.loads(CURRENT_SLAM.read_text(encoding="utf-8"))
    perception_receipt = json.loads(CURRENT_PERCEPTION.read_text(encoding="utf-8"))
    slam_artifacts = {item["path"]: item for item in slam_receipt["artifacts"]}
    manifest = {
        "schema": "grocery.rgb_current_vs_r7_evidence",
        "schema_version": 1,
        "created_date": "2026-09-25",
        "claim_scope": "RGB visual review only",
        "generation_receipt": None,
        "generation_receipt_reason": "No generative image or video model was used; only labels were added.",
        "captures": {
            "current": {
                "capture_id": "20260925-183307101",
                "source_commit": "48de461b42d5e0945b21432cef9f5523cc5b0874",
                "source_tree": current_capture_receipt["git_tree"],
                "capture_seal_sha256": current_capture_receipt["capture_sha256"],
                "status": current_capture_receipt["status"],
                "rgb_frame_count": current_capture_receipt["rgb"]["frame_count"],
                "rgb_first_stamp_s": current_capture_receipt["rgb"]["first_stamp_s"],
                "rgb_last_stamp_s": current_capture_receipt["rgb"]["last_stamp_s"],
                "panel": "left",
                "horizontal_fov_degrees": 75,
            },
            "r7": {
                "capture_id": "20260925-041644489",
                "source_commit": "3dc5107e8fedee3835259282e1655749ec7438c0",
                "source_tree": r7_capture_receipt["git_tree"],
                "capture_seal_sha256": r7_capture_receipt["capture_sha256"],
                "status": r7_capture_receipt["status"],
                "rgb_frame_count": r7_capture_receipt["rgb"]["frame_count"],
                "rgb_first_stamp_s": r7_capture_receipt["rgb"]["first_stamp_s"],
                "rgb_last_stamp_s": r7_capture_receipt["rgb"]["last_stamp_s"],
                "panel": "right",
                "horizontal_fov_degrees": 90,
            },
        },
        "timestamp_binding": {
            "frame_count": len(timestamps),
            "current_and_r7_index_bytes_identical": sha256(CURRENT_INDEX) == sha256(R7_CAPTURE / "rgb_frames.jsonl"),
            "samples": samples,
        },
        "fixed_input_verification": fixed_input_verification,
        "panel_source_verification": source_verification,
        "inputs": raw_inputs,
        "outputs": outputs,
        "current_pipeline_checkpoint": {
            "slam": {
                "status": "complete",
                "manifest_sha256": raw_inputs["current_slam_manifest"]["sha256"],
                "manifest_size_bytes": raw_inputs["current_slam_manifest"]["size_bytes"],
                "replay_source_commit": slam_receipt["git_sha"],
                "attempt_id": slam_receipt["slam_attempt"]["attempt_id"],
                "node_count": slam_receipt["scan_coverage"]["final_graph_node_count"],
                "node_stamp_sha256": slam_receipt["scan_coverage"]["final_graph_stamp_sha256"],
                "directed_neighbor_link_count": slam_receipt["scan_coverage"]["neighbor_edge_count"],
                "exact_sequence_coverage": slam_receipt["scan_coverage"]["exact_sequence_coverage"],
                "ground_truth_subscribed": slam_receipt["ground_truth_subscribed"],
                "publish_map_service_acknowledged": slam_receipt["publish_map_service_acknowledged"],
                "final_source_graph_identity": slam_receipt["final_source_graph_identity"],
                "graph_pose_version": slam_receipt["graph_pose_version"],
                "dense_pose_version": slam_receipt["dense_pose_version"],
                "map_version": slam_receipt["map_version"],
                "slam_cloud_sha256": slam_artifacts["slam_map.pcd"]["sha256"],
                "slam_cloud_size_bytes": slam_artifacts["slam_map.pcd"]["size_bytes"],
                "slam_map_pose_sha256": slam_artifacts["slam_map_poses.csv"]["sha256"],
                "slam_map_pose_size_bytes": slam_artifacts["slam_map_poses.csv"]["size_bytes"],
                "slam_map_keyframes_sha256": slam_artifacts["slam_map_keyframes.csv"]["sha256"],
                "slam_map_keyframes_size_bytes": slam_artifacts["slam_map_keyframes.csv"]["size_bytes"],
            },
            "perception": {
                "status": "complete",
                "manifest_sha256": raw_inputs["current_perception_manifest"]["sha256"],
                "manifest_size_bytes": raw_inputs["current_perception_manifest"]["size_bytes"],
                "producer_commit": perception_receipt["git_sha"],
                "capture_manifest_sha256": perception_receipt["capture_manifest_sha256"],
                "rgb_frame_count": perception_receipt["frame_count"],
                "raw_lidar_scan_count": perception_receipt["localization"]["raw_lidar_message_count"],
                "valid_lidar_scan_count": perception_receipt["localization"]["valid_decoded_scan_count"],
                "usable_lidar_scan_count": perception_receipt["localization"]["usable_finite_scan_count"],
                "projected_point_count": perception_receipt["localization"]["projected_point_count"],
                "localized_inventory_row_count": perception_receipt["track_count"],
                "ground_truth_consumed": perception_receipt["ground_truth_consumed"],
                "graph_pose_version": perception_receipt["graph_pose_version"],
                "dense_pose_version": perception_receipt["dense_pose_version"],
                "map_version": perception_receipt["map_version"],
            },
            "technical_lidar_preview_review": "preview independently reviewed; see technical-lidar-preview packet",
        },
        "source_receipt_policy": {
            "included_in_portable_packet": False,
            "reason": "Original producer receipts contain machine-local absolute paths.",
            "integrity": "Original bytes are unchanged in the logical run paths and bound above by SHA-256 and size.",
        },
        "method": {
            "pillow_version": PILLOW_VERSION,
            "ffmpeg_version": ffmpeg_version,
            "sheet_frame_matching": "minimum RGB MSE after FFmpeg area scale to each preserved 1280x360 sheet cell",
            "video_timestamp_labels": "one FFmpeg drawtext overlay per exact rgb_frames.jsonl row, enabled on its frame index",
        },
        "limitations": [
            "Current and R7 use different horizontal FOV values (75 and 90 degrees).",
            "Visible appearance remains synthetic, with procedural assets, repeated facings, simplified materials, and flat lighting.",
            "The separate LiDAR preview is preview-only and does not claim final technical delivery.",
            "This packet makes no LiDAR, SLAM-accuracy, perception-accuracy, or final-film claim.",
        ],
    }
    (PACKET / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n"
    )


if __name__ == "__main__":
    main()
