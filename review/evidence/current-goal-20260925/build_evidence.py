"""Build the labeled Current-vs-R7 RGB review packet from preserved raw evidence."""

from __future__ import annotations

import contextlib
import json
import hashlib
import math
import os
import shutil
import stat
import struct
import subprocess
import uuid
from pathlib import Path
from typing import Callable

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
GENERATED_OUTPUT_NAMES = (
    "current-vs-r7-labeled-3840x1080.mp4",
    "current-vs-r7-labeled-6-samples.png",
    "sample-times.json",
    "README.md",
    "manifest.json",
)
EXPECTED_STABLE_OUTPUT_SHA256 = {
    "current-vs-r7-labeled-3840x1080.mp4": "ab4071bb11d6590d5c21c5d379f7a756ed917c159c785499e0ae3ba048568e7f",
    "current-vs-r7-labeled-6-samples.png": "bc1d0c517a6df0ae039a307919aa5aadd84d1ad38efc424783aa1580705b760a",
    "sample-times.json": "09584844769c1d2b6ffca54d339624c06da7772280654523a7e318c44adb82b7",
}
TRANSACTION_NAMESPACE_NAME = ".rgb-evidence-transactions"
TRANSACTION_NAME_PREFIX = "tx-"
GARBAGE_TRANSACTION_NAME_PREFIX = "gc-"
LEGACY_TRANSACTION_NAME_PREFIX = ".rgb-evidence-transaction-"
PUBLICATION_JOURNAL_NAME = "publication-journal.json"
PUBLICATION_JOURNAL_SCHEMA = "grocery.rgb_evidence_publication_transaction"
PUBLICATION_PREPARED_MARKER = "PREPARED"
PUBLICATION_COMMITTED_MARKER = "COMMITTED"
PUBLICATION_GC_MARKER = "GC_READY"
PUBLICATION_LOCK_NAME = "publication.lock"
PUBLICATION_ORDER = tuple(
    name for name in GENERATED_OUTPUT_NAMES if name != "manifest.json"
) + ("manifest.json",)


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


def capture_builder_source() -> tuple[Path, bytes, dict[str, object]]:
    builder_path = Path(__file__).absolute()
    if is_link_or_junction(builder_path):
        raise RuntimeError(f"builder source must not be a link or junction: {builder_path}")
    builder_bytes = builder_path.read_bytes()
    try:
        git_root = Path(subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
            cwd=builder_path.parent,
        ).stdout.strip())
        relative_path = builder_path.relative_to(git_root).as_posix()
        tracked_blob = subprocess.run(
            ["git", "rev-parse", f"HEAD:{relative_path}"],
            check=True,
            capture_output=True,
            text=True,
            cwd=git_root,
        ).stdout.strip()
        if (
            len(tracked_blob) not in {40, 64}
            or any(character not in "0123456789abcdef" for character in tracked_blob)
        ):
            raise RuntimeError(f"invalid tracked builder blob identity: {tracked_blob!r}")
        tracked_bytes = subprocess.run(
            ["git", "cat-file", "blob", tracked_blob],
            check=True,
            capture_output=True,
            cwd=git_root,
        ).stdout
    except (OSError, subprocess.CalledProcessError, ValueError) as error:
        raise RuntimeError("cannot resolve executing builder to the tracked HEAD blob") from error
    if tracked_bytes != builder_bytes:
        raise RuntimeError(
            "executing builder bytes differ from the externally tracked HEAD blob"
        )
    record = {
        "path": "build_evidence.py",
        "sha256": hashlib.sha256(builder_bytes).hexdigest(),
        "size_bytes": len(builder_bytes),
        "tracked_head_blob": tracked_blob,
        "tracked_head_bytes_identical": True,
    }
    return builder_path, builder_bytes, record


def verify_builder_source(
    builder_path: Path,
    captured_bytes: bytes,
    captured_record: dict[str, object],
) -> None:
    current_bytes = builder_path.read_bytes()
    current_record = {
        "path": "build_evidence.py",
        "sha256": hashlib.sha256(current_bytes).hexdigest(),
        "size_bytes": len(current_bytes),
        "tracked_head_blob": captured_record["tracked_head_blob"],
        "tracked_head_bytes_identical": True,
    }
    if current_bytes != captured_bytes or current_record != captured_record:
        raise RuntimeError(
            f"executing builder source changed during generation: "
            f"captured={captured_record}, current={current_record}"
        )


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


def verify_expected_hashes(
    paths: dict[str, Path],
    *,
    label: str,
) -> dict[str, str]:
    if set(paths) != set(EXPECTED_FIXED_INPUT_SHA256):
        raise RuntimeError(
            f"{label} input set mismatch: expected={sorted(EXPECTED_FIXED_INPUT_SHA256)}, "
            f"actual={sorted(paths)}"
        )
    actual = {name: sha256(path) for name, path in paths.items()}
    if actual != EXPECTED_FIXED_INPUT_SHA256:
        mismatches = {
            name: {
                "expected": EXPECTED_FIXED_INPUT_SHA256.get(name),
                "actual": actual.get(name),
            }
            for name in sorted(set(EXPECTED_FIXED_INPUT_SHA256) | set(actual))
            if EXPECTED_FIXED_INPUT_SHA256.get(name) != actual.get(name)
        }
        raise RuntimeError(f"{label} RGB evidence input hash mismatch: {mismatches}")
    return actual


def snapshot_fixed_inputs(
    original_paths: dict[str, Path],
    snapshot_dir: Path,
) -> tuple[dict[str, Path], dict[str, object]]:
    """Copy and hash all fixed inputs into a transaction-local snapshot set."""
    if set(original_paths) != set(EXPECTED_FIXED_INPUT_SHA256):
        raise RuntimeError("cannot snapshot an incomplete fixed-input set")
    snapshot_dir.mkdir(parents=True, exist_ok=False)
    snapshot_paths: dict[str, Path] = {}
    copied_sha256: dict[str, str] = {}
    copied_size_bytes: dict[str, int] = {}
    for name, source in original_paths.items():
        destination = snapshot_dir / f"{name}{source.suffix}"
        digest = hashlib.sha256()
        size_bytes = 0
        with source.open("rb") as source_stream, destination.open("xb") as snapshot_stream:
            for chunk in iter(lambda: source_stream.read(1024 * 1024), b""):
                digest.update(chunk)
                snapshot_stream.write(chunk)
                size_bytes += len(chunk)
            snapshot_stream.flush()
            os.fsync(snapshot_stream.fileno())
        copied_hash = digest.hexdigest()
        expected_hash = EXPECTED_FIXED_INPUT_SHA256[name]
        if copied_hash != expected_hash:
            raise RuntimeError(
                f"input changed or was substituted while snapshotting {name}: "
                f"{copied_hash} != {expected_hash}"
            )
        snapshot_paths[name] = destination
        copied_sha256[name] = copied_hash
        copied_size_bytes[name] = size_bytes
    snapshot_sha256 = verify_expected_hashes(
        snapshot_paths, label="transaction snapshot"
    )
    if os.name != "nt":
        snapshot_dir.chmod(0o700)
    for path in snapshot_paths.values():
        path.chmod(stat.S_IREAD)
    return snapshot_paths, {
        "status": "passed",
        "rule": (
            "Each of the 12 fixed inputs is copied once into a transaction-local "
            "snapshot while "
            "the copied bytes are hashed. Generation reads only those snapshots. The "
            "snapshot files are rehashed against the same pins before use and again "
            "before publication."
        ),
        "copied_while_hashing": True,
        "git_ignored_same_volume_transaction": True,
        "directory_acl_policy": "Inherited host ACLs; no restrictive Windows ACL is claimed.",
        "read_only_during_generation": True,
        "artifact_count": len(snapshot_paths),
        "copied_sha256": copied_sha256,
        "snapshot_sha256": snapshot_sha256,
        "size_bytes": copied_size_bytes,
    }


def verify_fixed_inputs(
    paths: dict[str, Path] | None = None,
) -> tuple[list[float], dict[str, object], dict[str, dict[str, object]]]:
    """Fail closed unless every fixed input and capture/index binding is exact."""
    paths = fixed_input_paths() if paths is None else paths
    actual_sha256 = verify_expected_hashes(paths, label="fixed")

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
    parsed_captures: dict[str, dict[str, object]] = {}
    for name, path in capture_paths.items():
        try:
            capture = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"invalid {name} capture manifest: {error}") from error
        parsed_captures[name] = capture
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

    verification = {
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
    parsed_receipts = {
        "current_capture": parsed_captures["current"],
        "r7_capture": parsed_captures["r7"],
        "slam": slam,
        "perception": perception,
    }
    return timestamps, verification, parsed_receipts


def _read_exact(stream: object, byte_count: int) -> bytes:
    payload = bytearray()
    while len(payload) < byte_count:
        chunk = stream.read(byte_count - len(payload))  # type: ignore[attr-defined]
        if not chunk:
            break
        payload.extend(chunk)
    return bytes(payload)


def verify_raw_comparison_sources(
    paths: dict[str, Path] | None = None,
) -> dict[str, object]:
    """Verify every comparison panel frame against its exact declared RGB source."""
    paths = fixed_input_paths() if paths is None else paths
    sources = {
        "comparison": paths["raw_comparison_video"],
        "current": paths["current_rgb_video"],
        "r7": paths["r7_rgb_video"],
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


def match_sheet_frames(sheet: Image.Image, raw_video: Path) -> list[int]:
    rgb = np.asarray(sheet.convert("RGB"))
    cells = [rgb[y:y + 360, x:x + 1280] for y in (8, 376, 744) for x in (8, 1296)]
    best: list[tuple[float, int]] = [(float("inf"), -1) for _ in cells]
    command = [
        str(FFMPEG), "-hide_banner", "-loglevel", "error", "-i", str(raw_video),
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


def build_sheet(
    timestamps: list[float],
    inputs: dict[str, Path],
    output_dir: Path,
) -> list[dict[str, object]]:
    sheet = Image.open(inputs["raw_six_sample_sheet"]).convert("RGBA")
    matches = match_sheet_frames(sheet, inputs["raw_comparison_video"])
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
        output_dir / "current-vs-r7-labeled-6-samples.png", optimize=True
    )
    return records


def ffmpeg_escape(value: str) -> str:
    return value.replace("\\", "/").replace(":", r"\:").replace("'", r"\'")


def build_video(
    timestamps: list[float],
    inputs: dict[str, Path],
    output_dir: Path,
) -> None:
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
    filter_path = output_dir / "video-label-filter.txt"
    filter_path.write_text(",\n".join(filters) + "\n", encoding="utf-8")
    command = [
        str(FFMPEG), "-y", "-hide_banner", "-loglevel", "warning",
        "-i", str(inputs["raw_comparison_video"]), "-filter_script:v", filter_path.name,
        "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "16",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(output_dir / "current-vs-r7-labeled-3840x1080.mp4"),
    ]
    try:
        subprocess.run(command, check=True, cwd=output_dir)
    finally:
        filter_path.unlink(missing_ok=True)


def build_staged_packet(
    staging_dir: Path,
    snapshot_paths: dict[str, Path],
    original_paths: dict[str, Path],
    snapshot_verification: dict[str, object],
    builder_source_record: dict[str, object],
) -> dict[str, object]:
    timestamps, fixed_input_verification, parsed_receipts = verify_fixed_inputs(
        snapshot_paths
    )
    source_verification = verify_raw_comparison_sources(snapshot_paths)
    samples = build_sheet(timestamps, snapshot_paths, staging_dir)
    build_video(timestamps, snapshot_paths, staging_dir)
    (staging_dir / "sample-times.json").write_text(
        json.dumps({
            "timestamp_source": "identical capture rgb_frames.jsonl rows",
            "current_rgb_index_sha256": sha256(snapshot_paths["current_rgb_index"]),
            "r7_rgb_index_sha256": sha256(snapshot_paths["r7_rgb_index"]),
            "samples": samples,
        }, indent=2) + "\n", encoding="utf-8"
    )

    raw_inputs = {
        "raw_comparison_video": file_record(snapshot_paths["raw_comparison_video"], logical_path="runs/20260925-183307101/outputs/rgb-vs-r7/current-left_r7-right_3840x1080.mp4"),
        "raw_six_sample_sheet": file_record(snapshot_paths["raw_six_sample_sheet"], logical_path="runs/20260925-183307101/outputs/rgb-vs-r7/paired-contact-sheet-6times.png"),
        "current_capture_manifest": file_record(snapshot_paths["current_capture_manifest"], logical_path="runs/20260925-183307101/capture/capture_manifest.json"),
        "current_rgb_video": file_record(snapshot_paths["current_rgb_video"], logical_path="runs/20260925-183307101/capture/rgb_camera.mp4"),
        "current_rgb_index": file_record(snapshot_paths["current_rgb_index"], logical_path="runs/20260925-183307101/capture/rgb_frames.jsonl"),
        "current_effective_config": file_record(snapshot_paths["current_effective_config"], logical_path="runs/20260925-183307101/capture/effective_config.json"),
        "r7_capture_manifest": file_record(snapshot_paths["r7_capture_manifest"], logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/capture_manifest.json"),
        "r7_rgb_video": file_record(snapshot_paths["r7_rgb_video"], logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/rgb_camera.mp4"),
        "r7_rgb_index": file_record(snapshot_paths["r7_rgb_index"], logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/rgb_frames.jsonl"),
        "r7_effective_config": file_record(snapshot_paths["r7_effective_config"], logical_path=f"{R7_CAPTURE_LOGICAL.as_posix()}/effective_config.json"),
        "current_slam_manifest": file_record(snapshot_paths["current_slam_manifest"], logical_path="runs/20260925-183307101/slam/slam_manifest.json"),
        "current_perception_manifest": file_record(snapshot_paths["current_perception_manifest"], logical_path="runs/20260925-183307101/perception/perception_manifest.json"),
    }
    prepublication_input_verification = {
        "status": "passed",
        "snapshot_sha256": verify_expected_hashes(
            snapshot_paths, label="prepublication snapshot"
        ),
        "original_sha256": verify_expected_hashes(
            original_paths, label="prepublication original"
        ),
    }
    labeled_video = file_record(
        staging_dir / "current-vs-r7-labeled-3840x1080.mp4", staging_dir
    )
    labeled_sheet = file_record(
        staging_dir / "current-vs-r7-labeled-6-samples.png", staging_dir
    )
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

The builder copies all 12 inputs into read-only files in a Git-ignored, same-volume
transaction namespace while hashing the copied bytes. It inherits host directory
ACLs and makes no restrictive Windows ACL claim. All decoding, matching, labels,
and metadata use only those verified snapshots. It
rehashes both snapshots and original logical inputs before publication, builds the
five generated files in transaction-local staging, verifies stable media hashes and
all manifest output bindings, and rechecks the executing builder against its tracked
HEAD blob and startup byte snapshot. Publication uses a process-termination-recoverable
journal and rollback
backups in the same ignored namespace: it backs up the old manifest first, publishes
the other four files, and publishes the new manifest last. A restart completes an
already coherent new packet or restores the complete old packet with its manifest
last. Malformed residue or unexpected bytes are preserved and fail closed.

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
    (staging_dir / "README.md").write_text(readme, encoding="utf-8", newline="\n")

    ffmpeg_version = source_verification["ffmpeg_version"]
    outputs = {
        name: file_record(staging_dir / name, staging_dir)
        for name in [
            "current-vs-r7-labeled-3840x1080.mp4",
            "current-vs-r7-labeled-6-samples.png",
            "sample-times.json",
            "README.md",
        ]
    }
    outputs["build_evidence.py"] = builder_source_record
    current_capture_receipt = parsed_receipts["current_capture"]
    r7_capture_receipt = parsed_receipts["r7_capture"]
    slam_receipt = parsed_receipts["slam"]
    perception_receipt = parsed_receipts["perception"]
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
            "current_and_r7_index_bytes_identical": True,
            "samples": samples,
        },
        "input_snapshot_verification": snapshot_verification,
        "builder_source_verification": {
            "status": "passed",
            "rule": (
                "Capture the executing builder bytes at startup, require exact equality "
                "to the externally tracked HEAD blob, bind that immutable record in the "
                "manifest, and rehash the original builder path immediately before publication."
            ),
            "startup_record": builder_source_record,
            "final_original_path_rehash_required": True,
        },
        "fixed_input_verification": fixed_input_verification,
        "prepublication_input_verification": prepublication_input_verification,
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
            "publication_transaction": (
                "Generate five files in a Git-ignored same-volume runs namespace; validate "
                "stable output hashes and manifest bindings; rehash snapshots, originals, "
                "and the startup-captured tracked builder; write an allowlisted journal "
                "and PREPARED marker; back up the old manifest first; publish four data/docs "
                "files and the new manifest last; then write COMMITTED. Startup recovery either "
                "accepts a fully new packet or restores the fully old packet with its manifest last."
            ),
            "windows_durability_scope": (
                "Recoverable after process termination and restart; no sudden power-loss or "
                "filesystem-cache durability claim is made on Windows."
            ),
            "transaction_namespace": "runs/.rgb-evidence-transactions/tx-<32-lowercase-hex>",
            "directory_acl_policy": "Inherited host ACLs; no restrictive Windows ACL is claimed.",
        },
        "staged_output_validation": {
            "status": "passed",
            "expected_stable_output_sha256": EXPECTED_STABLE_OUTPUT_SHA256,
            "manifest_output_bindings_verified": True,
            "final_input_rehash_required_before_promotion": True,
        },
        "limitations": [
            "Current and R7 use different horizontal FOV values (75 and 90 degrees).",
            "Visible appearance remains synthetic, with procedural assets, repeated facings, simplified materials, and flat lighting.",
            "The separate LiDAR preview is preview-only and does not claim final technical delivery.",
            "This packet makes no LiDAR, SLAM-accuracy, perception-accuracy, or final-film claim.",
        ],
    }
    (staging_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return manifest


def validate_staged_packet(
    staging_dir: Path,
    manifest: dict[str, object],
    builder_source_record: dict[str, object],
) -> dict[str, str]:
    staged_names = {path.name for path in staging_dir.iterdir() if path.is_file()}
    if staged_names != set(GENERATED_OUTPUT_NAMES):
        raise RuntimeError(
            f"staged output set mismatch: expected={sorted(GENERATED_OUTPUT_NAMES)}, "
            f"actual={sorted(staged_names)}"
        )
    stable_hashes = {
        name: sha256(staging_dir / name) for name in EXPECTED_STABLE_OUTPUT_SHA256
    }
    if stable_hashes != EXPECTED_STABLE_OUTPUT_SHA256:
        raise RuntimeError(
            "stable staged output hash mismatch: "
            f"expected={EXPECTED_STABLE_OUTPUT_SHA256}, actual={stable_hashes}"
        )
    parsed_manifest = json.loads(
        (staging_dir / "manifest.json").read_text(encoding="utf-8")
    )
    if parsed_manifest != manifest:
        raise RuntimeError("staged manifest bytes do not parse to the validated manifest")
    outputs = manifest.get("outputs")
    if not isinstance(outputs, dict):
        raise RuntimeError("staged manifest outputs must be an object")
    expected_output_names = {
        "current-vs-r7-labeled-3840x1080.mp4",
        "current-vs-r7-labeled-6-samples.png",
        "sample-times.json",
        "README.md",
        "build_evidence.py",
    }
    if set(outputs) != expected_output_names:
        raise RuntimeError(
            f"manifest output set mismatch: expected={sorted(expected_output_names)}, "
            f"actual={sorted(outputs)}"
        )
    for name, declared in outputs.items():
        actual = (
            builder_source_record
            if name == "build_evidence.py"
            else file_record(staging_dir / name, staging_dir)
        )
        if declared != actual:
            raise RuntimeError(
                f"staged manifest output binding mismatch for {name}: "
                f"declared={declared}, actual={actual}"
            )
    return {name: sha256(staging_dir / name) for name in GENERATED_OUTPUT_NAMES}


def promote_staged_outputs(
    staging_dir: Path,
    destination_dir: Path,
    transaction_root: Path,
    *,
    expected_hashes: dict[str, str],
    replace: Callable[[Path, Path], object] = os.replace,
) -> None:
    """Promote manifest last; restart recovery rolls back process interruption."""
    current_staged_hashes = {
        name: sha256(staging_dir / name) for name in GENERATED_OUTPUT_NAMES
    }
    if current_staged_hashes != expected_hashes:
        raise RuntimeError(
            f"staged outputs changed after validation: expected={expected_hashes}, "
            f"actual={current_staged_hashes}"
        )
    journal = initialize_publication_journal(
        transaction_root, staging_dir, destination_dir, expected_hashes
    )
    rollback_dir = transaction_root / "rollback"
    journal["phase"] = "promoting"
    write_publication_journal(transaction_root, journal)
    try:
        manifest_entry = journal["entries"]["manifest.json"]
        manifest_destination = destination_dir / "manifest.json"
        if manifest_entry["old_exists"]:
            replace(manifest_destination, rollback_dir / "manifest.json")
            sync_directory(destination_dir)
            sync_directory(rollback_dir)
            manifest_entry["state"] = "old_backed_up"
            write_publication_journal(transaction_root, journal)

        for name in PUBLICATION_ORDER:
            entry = journal["entries"][name]
            source = staging_dir / name
            destination = destination_dir / name
            backup = rollback_dir / name
            if name != "manifest.json" and entry["old_exists"]:
                replace(destination, backup)
                sync_directory(destination_dir)
                sync_directory(rollback_dir)
                entry["state"] = "old_backed_up"
                write_publication_journal(transaction_root, journal)
            replace(source, destination)
            sync_directory(staging_dir)
            sync_directory(destination_dir)
            entry["state"] = "new_published"
            write_publication_journal(transaction_root, journal)

        if not packet_matches_plan(destination_dir, journal, version="new"):
            raise RuntimeError("published packet does not match the complete new plan")
        journal["phase"] = "committed"
        write_publication_journal(transaction_root, journal)
        write_marker(transaction_root / PUBLICATION_COMMITTED_MARKER, transaction_root.name)
    except BaseException as error:
        try:
            outcome = recover_publication_transaction(transaction_root, destination_dir)
        except BaseException as recovery_error:
            raise RuntimeError(
                f"publication failed and recovery was preserved for inspection: "
                f"publication={error}; recovery={recovery_error}"
            ) from error
        if outcome != "rolled_back":
            raise RuntimeError(
                f"publication failed with unexpected recovery outcome {outcome}: {error}"
            ) from error
        raise RuntimeError(f"publication failed and was rolled back: {error}") from error


def is_transaction_name(name: str, prefix: str) -> bool:
    suffix = name.removeprefix(prefix)
    return (
        name.startswith(prefix)
        and len(suffix) == 32
        and all(character in "0123456789abcdef" for character in suffix)
    )


def is_link_or_junction(path: Path) -> bool:
    try:
        if stat.S_ISLNK(os.lstat(path).st_mode):
            return True
    except FileNotFoundError:
        return False
    is_junction = getattr(path, "is_junction", None)
    return bool(is_junction and is_junction())


def require_real_directory(path: Path, *, label: str) -> None:
    try:
        mode = os.lstat(path).st_mode
    except FileNotFoundError as error:
        raise RuntimeError(f"missing {label}: {path}") from error
    if not stat.S_ISDIR(mode) or is_link_or_junction(path):
        raise RuntimeError(f"{label} must be a real, non-link directory: {path}")


def require_regular_file(path: Path, *, label: str) -> None:
    try:
        mode = os.lstat(path).st_mode
    except FileNotFoundError as error:
        raise RuntimeError(f"missing {label}: {path}") from error
    if not stat.S_ISREG(mode) or is_link_or_junction(path):
        raise RuntimeError(f"{label} must be a real, non-link file: {path}")


def sync_directory(path: Path) -> None:
    """Best-effort directory metadata flush; Windows does not expose this via os.open."""
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_atomic_bytes(path: Path, payload: bytes) -> None:
    temporary_path = path.with_name(f".{path.name}.tmp")
    if os.path.lexists(temporary_path):
        require_regular_file(temporary_path, label="atomic-write temporary file")
        os.unlink(temporary_path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_BINARY"):
        flags |= os.O_BINARY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(temporary_path, flags, 0o600)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(descriptor)
    os.replace(temporary_path, path)
    sync_directory(path.parent)


def write_marker(path: Path, transaction_name: str) -> None:
    write_atomic_bytes(path, (transaction_name + "\n").encode("ascii"))


def write_publication_journal(
    transaction_root: Path, journal: dict[str, object]
) -> None:
    journal_path = transaction_root / PUBLICATION_JOURNAL_NAME
    payload = (json.dumps(journal, indent=2, sort_keys=True) + "\n").encode("utf-8")
    write_atomic_bytes(journal_path, payload)


def output_record_or_absent(path: Path) -> dict[str, object]:
    if not os.path.lexists(path):
        return {"exists": False, "sha256": None, "size_bytes": None}
    require_regular_file(path, label="packet output")
    return {"exists": True, "sha256": sha256(path), "size_bytes": path.stat().st_size}


def initialize_publication_journal(
    transaction_root: Path,
    staging_dir: Path,
    destination_dir: Path,
    expected_hashes: dict[str, str],
) -> dict[str, object]:
    require_real_directory(transaction_root, label="transaction root")
    require_real_directory(staging_dir, label="transaction staging directory")
    require_real_directory(destination_dir, label="packet destination directory")
    if set(expected_hashes) != set(GENERATED_OUTPUT_NAMES):
        raise RuntimeError("publication plan output set is incomplete")
    rollback_dir = transaction_root / "rollback"
    rollback_dir.mkdir(exist_ok=False)
    entries: dict[str, dict[str, object]] = {}
    for name in GENERATED_OUTPUT_NAMES:
        source = staging_dir / name
        require_regular_file(source, label=f"staged output {name}")
        old = output_record_or_absent(destination_dir / name)
        entries[name] = {
            "name": name,
            "new_sha256": expected_hashes[name],
            "new_size_bytes": source.stat().st_size,
            "old_exists": old["exists"],
            "old_sha256": old["sha256"],
            "old_size_bytes": old["size_bytes"],
            "state": "prepared",
        }
    journal: dict[str, object] = {
        "schema": PUBLICATION_JOURNAL_SCHEMA,
        "schema_version": 1,
        "transaction_name": transaction_root.name,
        "phase": "prepared",
        "publication_order": list(PUBLICATION_ORDER),
        "manifest_backup_first": True,
        "manifest_published_last": True,
        "entries": entries,
    }
    write_publication_journal(transaction_root, journal)
    write_marker(transaction_root / PUBLICATION_PREPARED_MARKER, transaction_root.name)
    return journal


def validate_publication_journal(
    transaction_root: Path, journal: object
) -> dict[str, object]:
    if not isinstance(journal, dict):
        raise RuntimeError("publication journal must be an object")
    required_keys = {
        "schema", "schema_version", "transaction_name", "phase",
        "publication_order", "manifest_backup_first", "manifest_published_last",
        "entries",
    }
    if set(journal) != required_keys:
        raise RuntimeError(f"publication journal keys are invalid: {sorted(journal)}")
    if (
        journal["schema"] != PUBLICATION_JOURNAL_SCHEMA
        or journal["schema_version"] != 1
        or journal["transaction_name"] != transaction_root.name
        or journal["publication_order"] != list(PUBLICATION_ORDER)
        or journal["manifest_backup_first"] is not True
        or journal["manifest_published_last"] is not True
        or journal["phase"] not in {"prepared", "promoting", "recovering", "rolled_back", "committed"}
    ):
        raise RuntimeError("publication journal identity or transaction policy is invalid")
    entries = journal["entries"]
    if not isinstance(entries, dict) or set(entries) != set(GENERATED_OUTPUT_NAMES):
        raise RuntimeError("publication journal entry set is invalid")
    for name in GENERATED_OUTPUT_NAMES:
        entry = entries[name]
        expected_entry_keys = {
            "name", "new_sha256", "new_size_bytes", "old_exists",
            "old_sha256", "old_size_bytes", "state",
        }
        if not isinstance(entry, dict) or set(entry) != expected_entry_keys:
            raise RuntimeError(f"publication journal entry schema is invalid for {name}")
        valid_hash = lambda value: (
            isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )
        if (
            entry["name"] != name
            or not valid_hash(entry["new_sha256"])
            or not isinstance(entry["new_size_bytes"], int)
            or entry["new_size_bytes"] < 0
            or not isinstance(entry["old_exists"], bool)
            or entry["state"] not in {"prepared", "old_backed_up", "new_published", "rolled_back"}
        ):
            raise RuntimeError(f"publication journal entry values are invalid for {name}")
        if entry["old_exists"]:
            if not valid_hash(entry["old_sha256"]) or not isinstance(entry["old_size_bytes"], int):
                raise RuntimeError(f"publication old binding is invalid for {name}")
        elif entry["old_sha256"] is not None or entry["old_size_bytes"] is not None:
            raise RuntimeError(f"absent old output has a binding for {name}")
    return journal


def load_publication_journal(transaction_root: Path) -> dict[str, object]:
    journal_path = transaction_root / PUBLICATION_JOURNAL_NAME
    require_regular_file(journal_path, label="publication journal")
    try:
        parsed = json.loads(journal_path.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"publication journal is malformed: {error}") from error
    return validate_publication_journal(transaction_root, parsed)


def path_matches_entry(path: Path, entry: dict[str, object], version: str) -> bool:
    if version == "new":
        expected_exists = True
        expected_hash = entry["new_sha256"]
        expected_size = entry["new_size_bytes"]
    elif version == "old":
        expected_exists = entry["old_exists"]
        expected_hash = entry["old_sha256"]
        expected_size = entry["old_size_bytes"]
    else:
        raise ValueError(version)
    if not os.path.lexists(path):
        return expected_exists is False
    if expected_exists is False:
        return False
    require_regular_file(path, label=f"{version} packet output")
    return path.stat().st_size == expected_size and sha256(path) == expected_hash


def packet_matches_plan(
    destination_dir: Path, journal: dict[str, object], *, version: str
) -> bool:
    return all(
        path_matches_entry(destination_dir / name, journal["entries"][name], version)
        for name in GENERATED_OUTPUT_NAMES
    )


def validate_transaction_tree(transaction_root: Path) -> None:
    require_real_directory(transaction_root, label="transaction root")
    for directory, child_directories, files in os.walk(transaction_root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        require_real_directory(directory_path, label="transaction directory")
        for child_name in child_directories + files:
            child = directory_path / child_name
            if is_link_or_junction(child):
                raise RuntimeError(f"transaction residue contains a link or junction: {child}")


def recover_publication_transaction(
    transaction_root: Path,
    destination_dir: Path,
    *,
    replace: Callable[[Path, Path], object] = os.replace,
    unlink: Callable[[Path], object] = os.unlink,
) -> str:
    """Finish a complete publish or idempotently restore the prior complete packet."""
    validate_transaction_tree(transaction_root)
    journal_path = transaction_root / PUBLICATION_JOURNAL_NAME
    prepared_path = transaction_root / PUBLICATION_PREPARED_MARKER
    committed_path = transaction_root / PUBLICATION_COMMITTED_MARKER
    if not journal_path.exists():
        if prepared_path.exists() or committed_path.exists():
            raise RuntimeError("transaction marker exists without a complete journal")
        rollback_dir = transaction_root / "rollback"
        if rollback_dir.exists() and any(rollback_dir.iterdir()):
            raise RuntimeError("pre-journal transaction has rollback data; manual inspection required")
        allowed = {"snapshots", "staging", "rollback", f".{PUBLICATION_JOURNAL_NAME}.tmp"}
        unexpected = {path.name for path in transaction_root.iterdir()} - allowed
        if unexpected:
            raise RuntimeError(f"unrecognized pre-journal transaction residue: {sorted(unexpected)}")
        return "prepublication"

    journal = load_publication_journal(transaction_root)
    allowed_children = {
        "snapshots", "staging", "rollback", PUBLICATION_JOURNAL_NAME,
        f".{PUBLICATION_JOURNAL_NAME}.tmp", PUBLICATION_PREPARED_MARKER,
        f".{PUBLICATION_PREPARED_MARKER}.tmp", PUBLICATION_COMMITTED_MARKER,
        f".{PUBLICATION_COMMITTED_MARKER}.tmp", PUBLICATION_GC_MARKER,
        f".{PUBLICATION_GC_MARKER}.tmp",
    }
    unexpected_children = {
        path.name for path in transaction_root.iterdir()
    } - allowed_children
    if unexpected_children:
        raise RuntimeError(
            f"unexpected journaled transaction residue: {sorted(unexpected_children)}"
        )
    rollback_dir = transaction_root / "rollback"
    if not prepared_path.exists():
        rollback_empty = rollback_dir.is_dir() and not any(rollback_dir.iterdir())
        if rollback_empty and packet_matches_plan(destination_dir, journal, version="old"):
            return "prepublication"
        raise RuntimeError(
            "journal exists without PREPARED marker and is not provably prepublication"
        )
    require_regular_file(prepared_path, label="PREPARED marker")
    if prepared_path.read_text(encoding="ascii") != transaction_root.name + "\n":
        raise RuntimeError("PREPARED marker identity mismatch")

    if packet_matches_plan(destination_dir, journal, version="new"):
        if not committed_path.exists():
            journal["phase"] = "committed"
            write_publication_journal(transaction_root, journal)
            write_marker(committed_path, transaction_root.name)
        else:
            require_regular_file(committed_path, label="COMMITTED marker")
            if committed_path.read_text(encoding="ascii") != transaction_root.name + "\n":
                raise RuntimeError("COMMITTED marker identity mismatch")
        return "committed"
    if committed_path.exists() or journal["phase"] == "committed":
        raise RuntimeError("committed transaction does not match the complete new packet")

    if packet_matches_plan(destination_dir, journal, version="old"):
        journal["phase"] = "rolled_back"
        for entry in journal["entries"].values():
            entry["state"] = "rolled_back"
        write_publication_journal(transaction_root, journal)
        return "rolled_back"

    require_real_directory(rollback_dir, label="transaction rollback directory")
    journal["phase"] = "recovering"
    write_publication_journal(transaction_root, journal)
    rollback_order = tuple(reversed(PUBLICATION_ORDER[:-1])) + ("manifest.json",)
    for name in rollback_order:
        entry = journal["entries"][name]
        destination = destination_dir / name
        backup = rollback_dir / name
        destination_is_old = path_matches_entry(destination, entry, "old")
        destination_is_new = path_matches_entry(destination, entry, "new")
        if os.path.lexists(destination) and not destination_is_old and not destination_is_new:
            raise RuntimeError(f"unexpected destination bytes during recovery: {name}")
        if entry["old_exists"]:
            if os.path.lexists(backup):
                if not path_matches_entry(backup, entry, "old"):
                    raise RuntimeError(f"rollback backup binding mismatch: {name}")
                if os.path.lexists(destination):
                    unlink(destination)
                    sync_directory(destination_dir)
                replace(backup, destination)
                sync_directory(rollback_dir)
                sync_directory(destination_dir)
            elif not destination_is_old:
                raise RuntimeError(f"missing old destination and rollback backup: {name}")
        else:
            if os.path.lexists(backup):
                raise RuntimeError(f"unexpected rollback backup for absent old output: {name}")
            if os.path.lexists(destination):
                unlink(destination)
                sync_directory(destination_dir)
        entry["state"] = "rolled_back"
        write_publication_journal(transaction_root, journal)
    if not packet_matches_plan(destination_dir, journal, version="old"):
        raise RuntimeError("rollback did not restore the complete old packet")
    journal["phase"] = "rolled_back"
    write_publication_journal(transaction_root, journal)
    return "rolled_back"


def remove_guarded_transaction_tree(
    transaction_root: Path,
    allowed_parent: Path,
    *,
    prefix: str,
    root_file_last: str | None = None,
) -> None:
    """Remove one validated transaction tree without following links or junctions."""
    transaction_root = Path(os.path.abspath(transaction_root))
    allowed_parent = Path(os.path.abspath(allowed_parent))
    if (
        transaction_root.parent != allowed_parent
        or not is_transaction_name(transaction_root.name, prefix)
    ):
        raise RuntimeError(f"refusing to clean unverified transaction root: {transaction_root}")
    if not os.path.lexists(transaction_root):
        return
    validate_transaction_tree(transaction_root)

    def remove_directory_no_follow(directory: Path) -> None:
        directory = Path(os.path.abspath(directory))
        if os.path.commonpath((str(transaction_root), str(directory))) != str(
            transaction_root
        ):
            raise RuntimeError(f"transaction cleanup escaped its root: {directory}")
        require_real_directory(directory, label="transaction directory")
        with os.scandir(directory) as entries:
            children = list(entries)
        if directory == transaction_root and root_file_last is not None:
            children.sort(key=lambda entry: entry.name == root_file_last)
        for entry in children:
            child = directory / entry.name
            if entry.is_symlink() or getattr(child, "is_junction", lambda: False)():
                raise RuntimeError(f"refusing cleanup of linked transaction child: {child}")
            if entry.is_dir(follow_symlinks=False):
                remove_directory_no_follow(child)
            else:
                child.chmod(stat.S_IREAD | stat.S_IWRITE)
                os.unlink(child)
        directory.chmod(stat.S_IREAD | stat.S_IWRITE | stat.S_IEXEC)
        os.rmdir(directory)

    remove_directory_no_follow(transaction_root)


def retire_transaction_for_cleanup(transaction_root: Path, namespace: Path) -> None:
    """Atomically retire a verified-safe transaction, then delete restart-safely."""
    transaction_root = Path(os.path.abspath(transaction_root))
    namespace = Path(os.path.abspath(namespace))
    if (
        transaction_root.parent != namespace
        or not is_transaction_name(transaction_root.name, TRANSACTION_NAME_PREFIX)
    ):
        raise RuntimeError(f"refusing to retire unsafe transaction root: {transaction_root}")
    suffix = transaction_root.name.removeprefix(TRANSACTION_NAME_PREFIX)
    garbage_root = namespace / f"{GARBAGE_TRANSACTION_NAME_PREFIX}{suffix}"
    if os.path.lexists(garbage_root):
        raise RuntimeError(f"garbage transaction target already exists: {garbage_root}")
    write_marker(transaction_root / PUBLICATION_GC_MARKER, transaction_root.name)
    os.replace(transaction_root, garbage_root)
    sync_directory(namespace)
    remove_guarded_garbage_tree(garbage_root, namespace)


def remove_guarded_garbage_tree(garbage_root: Path, namespace: Path) -> None:
    garbage_root = Path(os.path.abspath(garbage_root))
    namespace = Path(os.path.abspath(namespace))
    if (
        garbage_root.parent != namespace
        or not is_transaction_name(garbage_root.name, GARBAGE_TRANSACTION_NAME_PREFIX)
    ):
        raise RuntimeError(f"refusing to clean unsafe garbage root: {garbage_root}")
    validate_transaction_tree(garbage_root)
    marker = garbage_root / PUBLICATION_GC_MARKER
    if not marker.exists():
        if any(garbage_root.iterdir()):
            raise RuntimeError(f"nonempty garbage transaction lacks GC_READY: {garbage_root}")
    else:
        require_regular_file(marker, label="GC_READY marker")
        expected_tx_name = (
            TRANSACTION_NAME_PREFIX
            + garbage_root.name.removeprefix(GARBAGE_TRANSACTION_NAME_PREFIX)
            + "\n"
        )
        if marker.read_text(encoding="ascii") != expected_tx_name:
            raise RuntimeError(f"GC_READY marker identity mismatch: {garbage_root}")
    remove_guarded_transaction_tree(
        garbage_root,
        namespace,
        prefix=GARBAGE_TRANSACTION_NAME_PREFIX,
        root_file_last=PUBLICATION_GC_MARKER,
    )


def prepare_transaction_namespace(packet_dir: Path, runs_root: Path) -> Path:
    packet_dir = Path(os.path.abspath(packet_dir))
    runs_root = Path(os.path.abspath(runs_root))
    expected_runs_root = Path(os.path.abspath(ROOT / "runs"))
    if runs_root != expected_runs_root:
        raise RuntimeError(f"transaction runs root is not the fixed allowed root: {runs_root}")
    require_real_directory(packet_dir, label="evidence packet directory")
    require_real_directory(runs_root, label="ignored runs directory")
    legacy_namespace = packet_dir / TRANSACTION_NAMESPACE_NAME
    if os.path.lexists(legacy_namespace):
        raise RuntimeError(
            f"legacy packet transaction namespace requires manual inspection: {legacy_namespace}"
        )
    with os.scandir(packet_dir.parent) as entries:
        legacy_roots = [
            entry.path for entry in entries
            if is_transaction_name(entry.name, LEGACY_TRANSACTION_NAME_PREFIX)
        ]
    if legacy_roots:
        raise RuntimeError(
            f"legacy pre-journal transaction residue requires manual inspection: {legacy_roots}"
        )
    try:
        subprocess.run(
            ["git", "check-ignore", "--quiet", "--", str(runs_root / TRANSACTION_NAMESPACE_NAME / "probe")],
            check=True,
            cwd=ROOT,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError("transaction namespace is not confirmed Git-ignored") from error
    namespace = runs_root / TRANSACTION_NAMESPACE_NAME
    if os.path.lexists(namespace):
        require_real_directory(namespace, label="transaction namespace")
    else:
        os.mkdir(namespace)
        sync_directory(runs_root)
    if namespace.parent != runs_root:
        raise RuntimeError(f"transaction namespace escaped runs directory: {namespace}")
    if os.stat(namespace, follow_symlinks=False).st_dev != os.stat(
        packet_dir, follow_symlinks=False
    ).st_dev:
        raise RuntimeError("transaction namespace is not on the packet filesystem")
    return namespace


@contextlib.contextmanager
def publication_lock(namespace: Path):
    """Hold one OS-released exclusive lock across recovery, build, and publication."""
    require_real_directory(namespace, label="transaction namespace")
    lock_path = namespace / PUBLICATION_LOCK_NAME
    flags = os.O_CREAT | os.O_RDWR
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(lock_path, flags, 0o600)
    acquired = False
    try:
        require_regular_file(lock_path, label="publication lock")
        os.lseek(descriptor, 0, os.SEEK_SET)
        if os.fstat(descriptor).st_size == 0:
            os.write(descriptor, b"0")
            os.fsync(descriptor)
        os.lseek(descriptor, 0, os.SEEK_SET)
        if os.name == "nt":
            import msvcrt
            try:
                msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
            except OSError as error:
                raise RuntimeError("another RGB evidence publication holds the lock") from error
        else:
            import fcntl
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise RuntimeError("another RGB evidence publication holds the lock") from error
        acquired = True
        yield
    finally:
        try:
            if acquired:
                os.lseek(descriptor, 0, os.SEEK_SET)
                if os.name == "nt":
                    import msvcrt
                    msvcrt.locking(descriptor, msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def recover_stale_transactions(namespace: Path, destination_dir: Path) -> None:
    require_real_directory(namespace, label="transaction namespace")
    with os.scandir(namespace) as entries:
        stale_entries = list(entries)
    for entry in stale_entries:
        if entry.name == PUBLICATION_LOCK_NAME:
            if entry.is_symlink() or entry.is_dir(follow_symlinks=False):
                raise RuntimeError(f"unsafe publication lock entry: {entry.path}")
            continue
        if is_transaction_name(entry.name, GARBAGE_TRANSACTION_NAME_PREFIX):
            remove_guarded_garbage_tree(namespace / entry.name, namespace)
            continue
        if not is_transaction_name(entry.name, TRANSACTION_NAME_PREFIX):
            raise RuntimeError(
                f"unexpected entry in transaction namespace; refusing cleanup: {entry.path}"
            )
        transaction_root = namespace / entry.name
        require_real_directory(transaction_root, label="stale transaction root")
        outcome = recover_publication_transaction(transaction_root, destination_dir)
        if outcome not in {"prepublication", "rolled_back", "committed"}:
            raise RuntimeError(f"unsafe stale transaction outcome: {outcome}")
        retire_transaction_for_cleanup(transaction_root, namespace)


def create_transaction_root(namespace: Path) -> Path:
    namespace = Path(os.path.abspath(namespace))
    require_real_directory(namespace, label="transaction namespace")
    transaction_root = namespace / f"{TRANSACTION_NAME_PREFIX}{uuid.uuid4().hex}"
    if (
        transaction_root.parent != namespace
        or not is_transaction_name(transaction_root.name, TRANSACTION_NAME_PREFIX)
    ):
        raise RuntimeError(f"unsafe transaction root: {transaction_root}")
    os.mkdir(transaction_root)
    require_real_directory(transaction_root, label="new transaction root")
    if os.name != "nt":
        transaction_root.chmod(0o700)
    if os.stat(transaction_root, follow_symlinks=False).st_dev != os.stat(
        namespace, follow_symlinks=False
    ).st_dev:
        raise RuntimeError("transaction root is not on the namespace filesystem")
    return transaction_root


def transaction_is_safe_to_clean(
    transaction_root: Path, destination_dir: Path
) -> bool:
    if not transaction_root.exists():
        return True
    try:
        outcome = recover_publication_transaction(transaction_root, destination_dir)
    except BaseException:
        return False
    return outcome in {"prepublication", "rolled_back", "committed"}


def main() -> None:
    builder_path, builder_bytes, builder_source_record = capture_builder_source()
    validate_runtime()
    original_paths = fixed_input_paths()
    namespace = prepare_transaction_namespace(PACKET, ROOT / "runs")
    with publication_lock(namespace):
        recover_stale_transactions(namespace, PACKET)
        transaction_root = create_transaction_root(namespace)
        try:
            snapshot_paths, snapshot_verification = snapshot_fixed_inputs(
                original_paths, transaction_root / "snapshots"
            )
            staging_dir = transaction_root / "staging"
            staging_dir.mkdir()
            manifest = build_staged_packet(
                staging_dir,
                snapshot_paths,
                original_paths,
                snapshot_verification,
                builder_source_record,
            )
            validated_output_hashes = validate_staged_packet(
                staging_dir, manifest, builder_source_record
            )
            verify_expected_hashes(snapshot_paths, label="final prepromotion snapshot")
            verify_expected_hashes(original_paths, label="final prepromotion original")
            verify_builder_source(builder_path, builder_bytes, builder_source_record)
            promote_staged_outputs(
                staging_dir,
                PACKET,
                transaction_root,
                expected_hashes=validated_output_hashes,
            )
        finally:
            if transaction_is_safe_to_clean(transaction_root, PACKET):
                retire_transaction_for_cleanup(transaction_root, namespace)


if __name__ == "__main__":
    main()
