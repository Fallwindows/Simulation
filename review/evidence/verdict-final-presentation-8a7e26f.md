# Final independent presentation artifact review

**Disposition: ACCEPTED**

## Exact scope

- Candidate commit: `8a7e26f815db41c627ba3a972a98b81c979db0c0`
- Candidate tree: `85ab580a6e7365ec857a54dca34661f68343c07c`
- Parent: `bc810fc04b3775b66beaa90f9b499ab88a955714`
- Production run: `runs/20260921-053814804`
- Delivery directory: `runs/20260921-053814804/presentation-delivery`
- Runtime routing: GPT-5.6 Sol, high reasoning
- No implementation, candidate, or production media bytes were edited.

The assigned commit and tree match exactly. Commit `32e77b62ce5f655a075f3c608a14a9c00b622f54`, which pins the production technical source, is an ancestor. The integration checkout is clean and `git diff --check` reports no error.

## Exact final artifacts

- `presentation_delivery_manifest.json`: SHA-256 `b53b90ec58676d0cc6d0a8393087084bcc58f4ec94d3b9f8473928ccce5a0402`, 33,880 bytes
- `presentation_final.mp4`: SHA-256 `007b97082233ee537dce897424110ac42b0c4fd9a91bb74e6bec89268c37a2e7`, 166,101,580 bytes
- `presentation_master_lossless.mkv`: SHA-256 `4ff0ba8cdafbb806bf7b119e731c288d23d48c5606ac088b2e58abe8099eaba9`, 962,194,435 bytes
- `presentation_silent.mp4`: SHA-256 `3f452d446cfe03cd9c7d9509da7a89c2006c804efaf4aed7d76e23b463567b2c`, 165,167,369 bytes
- `presentation_review_720p.mp4`: SHA-256 `0bb7ac9c672100f6c680b9c21bc48755df0f8f0df0cfdf2a02fe30590e972c5c`, 49,406,920 bytes
- `presentation_ambience.wav`: SHA-256 `015a9e9984800652ed3ee6bf1cd6d33425259514ad31ff5de393812b50c9938a`, 8,640,044 bytes
- `presentation_delivery_contact_sheet.png`: SHA-256 `c3fcb3c0a9a4d26997150b52b6d9b16e5195e7d1d5d7feb0cd27226f07fd9af7`, 991,434 bytes

Every declared output and source hash in the delivery manifest was recomputed from the referenced file. Every declared source and output media probe reproduced exactly. The final MP4 independently probes as H.264/yuv420p, native 1920×1080, exact 30/1 fps, 1,350 frames, 45.0 seconds, one video stream, and one AAC stereo 48 kHz audio stream. The lossless master is FFV1 1920×1080/30, 1,350 frames, 45.0 seconds, with no audio. The review MP4 is 1280×720/30, 1,350 frames, with stereo audio.

A full independent decode of both final video and audio completed with no errors. Audio is non-silent: measured mean `-44.8 dB` and peak `-31.9 dB`; the WAV is exactly 2-channel, 16-bit, 48 kHz, 2,160,000 samples per channel, and 45.0 seconds.

## Timeline, complete bundle, and source binding

The package contains exactly 12 genuine shots with half-open boundaries:

`0, 90, 180, 300, 420, 540, 660, 750, 840, 960, 1080, 1200, 1350`.

Their durations total exactly 1,350 frames / 45.0 seconds. Shots 1–5 map RGB source frames 0–539. Each of shots 6–12 begins at local technical frame 0 and uses a distinct reviewed technical video and receipt. No shot reports a missing genuine role.

The complete input bundle SHA-256 is `72f1548ca1bef5cc024f8dbeefad57b481dbd46ef240918cfaeab0849459750d`. The accepted complete-bundle validator returned `complete=true`, classification `reviewed_production`, and all five ready roles: RGB, LiDAR, pose, map, and reconstruction. It reopened and verified the underlying artifacts, including the large raw bag.

Key bindings are exact and same-run:

- capture id `20260921-053814804`
- canonical capture hash `ab84ccc1f71790f896bab1375062ed1db9f40bf9a1531d582f7cd0e589121eba`
- capture manifest `3d82955332e326376b1f3e683a34ff589e4f0324cf17015682d336f282db3599`
- SLAM manifest `bb79ed9155f40d27621512f6b3309aabe80aed1ab88e3783478aaf518c02ed3e`
- finalized map `587043f3f84c7dfc4449379a2bd96ca801a63f2a12b432d4d3816c73c08b0452`
- estimated trajectory `46e144e0519b7320096ba2735cca1395068c116124e4ca58f5f750cd58205160`
- perception manifest `0fef77178e555286b2570695df2674e2b833a15592c05f735834b3571c294aab`
- estimated object-state CSV `3247c4d5dbcab08f166c8da495243aca8723725bc89b457bfedc6e0948bb8cbe`
- RGB bundle `b2b5264a765adecc2679a1c125f15bc823d92fe59af8ad1aa7479c9cc41817d8`
- RGB source video `c453b46e40acd06928b384fd13a5a7822c16f46ca5f30b79fc829d1d8e05c703`
- technical delivery manifest `f7cc236b915eac7b48a191334d8f9ca5839b8cbd10260a32b5732d6c08f0a909`

`config/technical_source_catalog.json` is byte-identical to the blob introduced by commit `32e77b6`: Git blob `8164bd08d21a0c95ff6ae05f51ee90bb076157fa`, file SHA-256 `28f70726ccb3b3163cc5c214508e9f28a0d3222ec0c8af1a735d6f2a9f8ddee6`. Its production entry matches all artifacts and producer revisions above.

## Ground-truth isolation and RGB orientation

The final manifest, complete bundle, technical source receipt, and perception manifest all declare and validate `ground_truth_consumed=false`. Perception also declares `ground_truth_required=false`. Evaluation-only inventory artifacts are absent from the technical-source binding; the film uses the ground-truth-free estimated inventory.

RGB acceptance is bound to the exact reviewed production allowlist and checked-in capture verdict. The catalog, RGB bundle, receipt, complete bundle, and shots all agree on `presentation_transform.operation=none` and `raw_capture_bytes_modified=false`. Visual inspection confirms the prominent FRESH MARKET sign reads left-to-right; a global horizontal flip would be wrong.

## Visual inspection and regenerated evidence

The checked-in contact sheet and all 36 start/mid/end representative frames were visually inspected. They show:

- coherent, properly oriented RGB aisle motion for shots 1–5;
- seven distinct technical views for shots 6–12;
- legible titles and callouts without clipping;
- no black, torn, corrupt, fixture-watermarked, or missing-input frames;
- explicit visual qualifications such as `ESTIMATED CENTERS · EXTENTS NOT ESTIMATED` and `Class unknown · Extent not estimated`;
- a final statement limited to simulated sensor data, finalized offline map, estimated trajectory, and persistent item centers.

All 36 frames were independently re-extracted from `presentation_final.mp4`; all 36 PNG SHA-256 values exactly matched the manifest. The 12-shot contact sheet was independently regenerated and was byte-identical at SHA-256 `c3fcb3c0a9a4d26997150b52b6d9b16e5195e7d1d5d7feb0cd27226f07fd9af7`.

## Required interpretation limits

Acceptance applies to this exact presentation package and its honest technical-view claims. It does not establish a complete or SKU-level inventory.

- All 454 estimator records are `unknown_product`; SKU and semantic class are not estimated.
- Object extents are not estimated. Crosshair markers represent estimated centers, not reconstructed product bounds.
- The technical renderer selects 28 centers for legibility from the 454-row estimated source. The visualization is intentionally incomplete and must not be described as exhaustive shelf occupancy or product counting.
- The estimator has sparse and uncertain observations, including single-observation tracks and previously measured positional outliers. Its markers are diagnostic estimates.
- The reviewed technical source covers simulation time 0.2–18.6 seconds. Shots 6–12 remap that source span into seven edited technical sequences; they are not a chronological continuation of the 20.5-second capture or a claim of 45 seconds of simulated sensor acquisition.
- `complete` describes satisfaction of the delivery and provenance contract. It does not mean complete inventory recovery.

These limits are consistent with the visible captions and manifest semantics; no unsupported SKU, extent, ground-truth-free accuracy, or inventory-completeness claim appears in the final film.

## Independent checks

- Independent artifact audit: `final-presentation-audit-8a7e26f.json`, SHA-256 `af49adb213de35a64b8fb765f10d2727975885fdff5790758b8ee0d29de37be7`
- Independent 36-frame review sheet: SHA-256 `5c2574a6f9bfa961b0884a3bdf996414e80f5165283d5794f22281da4dcef17a`
- Focused presentation, technical-view, and perception-provenance suite: **43/43 passed**
- No partial, staging, or backup artifact remains in the delivery directory
- No GPU or Isaac execution was used

No blocking artifact, provenance, media, visual, or claim defect was found for this exact candidate and run.
