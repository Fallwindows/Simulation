"""Render clean and tracked-overlay MP4s from the same captured RGB frames."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import av
import cv2


def _load_annotations(path: Path) -> dict[int, dict[str, object]]:
    result: dict[int, dict[str, object]] = {}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                record = json.loads(line)
                result[int(record["frame_index"])] = record
    return result


def _draw_overlay(image, record: dict[str, object]) -> None:
    for detection in record.get("detections", []):
        x0, y0, x1, y1 = [int(value) for value in detection["bbox_xyxy"]]
        cx, cy = [int(round(value)) for value in detection["center_px"]]
        # BGR cyan keeps the overlay readable against both dark shelves and
        # bright package faces.  Track IDs stay in the spreadsheet/JSONL so
        # the video remains a simple visual proof instead of a text wall.
        cv2.rectangle(image, (x0, y0), (x1, y1), (255, 255, 0), 2)
        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)


def render_outputs(run_dir: str | Path) -> dict[str, object]:
    run = Path(run_dir).resolve()
    capture = run / "capture"
    perception = run / "perception"
    outputs = run / "outputs"
    outputs.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((perception / "perception_manifest.json").read_text(encoding="utf-8"))
    if manifest.get("status") != "complete":
        raise ValueError("perception is not complete")
    clean_path = outputs / "clean_walkthrough.mp4"
    annotated_path = outputs / "tracking_overlay.mp4"
    shutil.copyfile(capture / "rgb_camera.mp4", clean_path)
    annotations = _load_annotations(perception / "frame_annotations.jsonl")

    with av.open(str(capture / "rgb_camera.mp4")) as source, av.open(str(annotated_path), mode="w") as target:
        source_stream = source.streams.video[0]
        rate = source_stream.average_rate or 30
        target_stream = target.add_stream("libx264", rate=rate)
        target_stream.width = source_stream.width
        target_stream.height = source_stream.height
        target_stream.pix_fmt = "yuv420p"
        target_stream.options = {"crf": "18", "preset": "medium"}
        for index, frame in enumerate(source.decode(video=0)):
            image = frame.to_ndarray(format="bgr24")
            _draw_overlay(image, annotations.get(index, {"detections": []}))
            out_frame = av.VideoFrame.from_ndarray(image, format="bgr24")
            out_frame.pts = frame.pts
            out_frame.time_base = frame.time_base
            for packet in target_stream.encode(out_frame):
                target.mux(packet)
        for packet in target_stream.encode():
            target.mux(packet)

    frame_count = len(annotations)
    metadata = {
        "status": "complete",
        "source_frame_sequence": "../capture/rgb_frames.jsonl",
        "source_video": "../capture/rgb_camera.mp4",
        "clean_video": "clean_walkthrough.mp4",
        "annotated_video": "tracking_overlay.mp4",
        "resolution": [int(source_stream.width), int(source_stream.height)],
        "fps": float(rate),
        "frame_count": frame_count,
        "duration_s": (frame_count - 1) / float(rate) if frame_count > 1 else 0.0,
        "annotation_source": "../perception/frame_annotations.jsonl",
        "box_style": "2px cyan rectangle from RGB connected-component bounds",
        "center_dot_style": "5px filled red circle at measured component centroid",
        "text_labels": "none; persistent IDs are in the tracking/export artifacts",
        "ground_truth_consumed": False,
    }
    (outputs / "render_manifest.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    print(json.dumps(render_outputs(args.run_dir), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
