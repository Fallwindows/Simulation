"""Build the labeled Current-vs-R7 RGB review packet from preserved raw evidence."""

from __future__ import annotations

import json
import hashlib
import os
import shutil
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


def validate_runtime() -> None:
    required = {
        "raw comparison video": RAW_VIDEO,
        "raw six-sample sheet": RAW_SHEET,
        "current RGB index": CURRENT_INDEX,
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
    timestamps = load_timestamps()
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
LiDAR preview and review remain pending. This packet does not claim final production
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
    (PACKET / "README.md").write_text(readme, encoding="utf-8")

    ffmpeg_version = subprocess.run(
        [str(FFMPEG), "-version"], check=True, capture_output=True, text=True
    ).stdout.splitlines()[0]
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
            "technical_lidar_preview_review": "pending",
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
            "Technical LiDAR preview and review remain pending.",
            "This packet makes no LiDAR, SLAM-accuracy, perception-accuracy, or final-film claim.",
        ],
    }
    (PACKET / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
