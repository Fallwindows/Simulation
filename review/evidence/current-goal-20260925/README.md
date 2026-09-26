# Current vs R7 RGB visual evidence

This packet labels the preserved side-by-side RGB comparison of **Current** capture
`20260925-183307101` at source `48de461b42d5e0945b21432cef9f5523cc5b0874`
and **R7** capture `20260925-041644489` at source
`3dc5107e8fedee3835259282e1655749ec7438c0`. Current is the left panel and R7 is
the right panel.

## Review media

- `current-vs-r7-labeled-3840x1080.mp4` — 613 synchronized 30 fps frames with
  panel/run/source labels and each frame's exact sensor timestamp from the identical
  `rgb_frames.jsonl` rows. SHA-256 `ab4071bb11d6590d5c21c5d379f7a756ed917c159c785499e0ae3ba048568e7f`.
- `current-vs-r7-labeled-6-samples.png` — six labeled matched-frame samples.
  SHA-256 `bc1d0c517a6df0ae039a307919aa5aadd84d1ad38efc424783aa1580705b760a`.
- `sample-times.json` — machine-readable matched frame indices and timestamps.

| Sample | Frame index | Sensor timestamp (s) |
| ---: | ---: | ---: |
| 1 | 0 | 0.100000000 |
| 2 | 90 | 3.100000000 |
| 3 | 180 | 6.100000000 |
| 4 | 360 | 12.100000000 |
| 5 | 510 | 17.100000000 |
| 6 | 600 | 20.100000000 |

## Provenance and scope

The unlabelled inputs remain unchanged under
`runs/20260925-183307101/outputs/rgb-vs-r7/`. The raw comparison video SHA-256 is
`d0d1a2796b2b2e9c53d2e3633ac8c3f46cb995ba6002a83121ecf3909057aae4` and the raw six-sample sheet SHA-256
is `b8cfcb5a5c4519b253b25da253809da37dced9065bed8ea583f03da5caa3bbec`. The packet manifest binds these
inputs, both exact capture manifests, both RGB videos and timestamp indices, and all
packet outputs.

The builder copies all 12 inputs into private, read-only files while hashing the
copied bytes. All decoding, matching, labels, and metadata use only those verified snapshots. It
rehashes both snapshots and original logical inputs before publication, builds the
five generated files in a private staging directory, verifies stable media hashes and
all manifest output bindings, then promotes the complete set with rollback backups.
Failed input, output, or promotion checks leave the previously published packet intact.

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
