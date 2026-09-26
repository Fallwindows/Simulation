# Selective LiDAR technical preview

This packet contains a review preview from simulated capture `20260925-183307101` (`production-20260925-183307101-v1`). It uses the recorded PointCloud2 scans with the matching RGB capture, estimated SLAM map/trajectory, and ground-truth-free perception output.

## Review media

- `technical_views_preview_720p.mp4` — one combined 27-second clip, 1280×720, 30 fps, 810 frames, with seven ordered views: `sensor_activation`, `lidar_environment`, `persistent_map`, `object_association`, `object_detail`, `observed_aisle_overview`, and `final_technical_view`.
- `technical_views_contact_sheet.png` — 21 representative frames, three from each view.
- `representative_frames/` — the same 21 stills at their full 1280×720 resolution.
- `technical_camera_trace.jsonl` — timestamped camera-pose and source-view trace, one row per preview frame.
- `evidence-manifest.json` — portable output hashes/sizes, source identities, and review limitations.
- `technical_views_preview_manifest.portable.json` — the producer receipt with only machine-specific absolute paths replaced by equivalent repository-relative paths.

## Provenance

The original producer receipt remains unchanged locally at the logical path in `evidence-manifest.json`; its exact raw-byte SHA-256 and size are recorded there. It is omitted from this portable packet because it contains three absolute paths into the user's profile and disposable worktree. The portable receipt preserves the remaining fields and source/output hashes. The capture, SLAM, perception, source-catalog, and artifact hashes are in `evidence-manifest.json` and the portable receipt.

The receipt marks this source `preview_only` and `delivery_eligible: false`. RGB and LiDAR are synchronized to the same simulated capture; ground-truth, future returns, storyboard pixels, and scene/asset metadata are not consumed in the technical render. The raw scan source is actual recorded PointCloud2 output from the simulator, not a physical-world LiDAR recording.

## How to read the imagery

The point returns show selected simulated sensor support. The first segment pairs RGB context with co-timed scan returns; later views isolate scan geometry, past-only mapped observations, estimated regions, and one selected region. Callout lines and the sensor-origin/ray graphics explain selection and alignment; they do not depict literal visible laser beams. Object regions represent nearest-surface support only. Their class and full extent are not estimated. Each scan is projected at its header timestamp with a rigid pose; per-return timing and deskew are absent.

This combined preview is for visual review. It is not the seven final 1080p per-view clips, the 1,350-frame final film, or final delivery acceptance. The scene remains visibly synthetic, and this packet makes no claim of photorealistic appearance, SLAM accuracy, or perception accuracy.
