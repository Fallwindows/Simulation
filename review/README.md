# Review and evidence index

This directory is the stable entry point for independent review and published
evidence for the 45-second simulation video.

## Current production run

- Branch: `codex/storyboard-video`
- Accepted implementation candidate: `8a7e26f815db41c627ba3a972a98b81c979db0c0`
- Accepted tree: `85ab580a6e7365ec857a54dca34661f68343c07c`
- Original source base: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Run window: 2026-09-20 23:21:35 PDT through approximately 2026-09-21
  11:21:35 PDT
- Requirements authority: [`../EXECUTION_OVERRIDES.md`](../EXECUTION_OVERRIDES.md)
- Implementation specification:
  [`../SIMULATION_VIDEO_IMPLEMENTATION_SPEC.md`](../SIMULATION_VIDEO_IMPLEMENTATION_SPEC.md)
- Status: final 1080p delivery accepted by an independent Sol/high reviewer.

## Final delivery

The retained local delivery is under
`runs/20260921-053814804/presentation-delivery/` (runtime media is intentionally
ignored by Git). The exact final artifacts are:

- `presentation_final.mp4`: 1920x1080, 30 fps, 1,350 frames, 45.0 seconds,
  H.264 with 48 kHz stereo AAC; SHA-256
  `007b97082233ee537dce897424110ac42b0c4fd9a91bb74e6bec89268c37a2e7`.
- `presentation_master_lossless.mkv`: FFV1 lossless video master; SHA-256
  `4ff0ba8cdafbb806bf7b119e731c288d23d48c5606ac088b2e58abe8099eaba9`.
- `presentation_review_720p.mp4`: 1280x720 review copy; SHA-256
  `0bb7ac9c672100f6c680b9c21bc48755df0f8f0df0cfdf2a02fe30590e972c5c`.
- `presentation_delivery_manifest.json`: SHA-256
  `b53b90ec58676d0cc6d0a8393087084bcc58f4ec94d3b9f8473928ccce5a0402`.

The published [12-shot contact sheet](evidence/presentation-delivery-contact-sheet.png)
contains actual frames extracted from the accepted final video.

## Independent evidence

- [Final presentation verdict](evidence/verdict-final-presentation-8a7e26f.md)
- [Production perception verdict](evidence/verdict-production-perception-def059b.md)
- [RGB orientation verdict](evidence/verdict-rgb-orientation-5b2521d.md)
- [Capture verdict](evidence/verdict-capture-20260921-053814804.md)
- [Full integrated test transcript](evidence/test-results-8a7e26f.txt)

The final reviewer recomputed every declared output, source, renderer, contact
sheet, and representative-frame hash; fully decoded the final media; and
validated the same-run capture, SLAM, perception, RGB, and technical-source
chain with ground truth excluded. The full integrated suite passes 157 tests.

## Representation limits

The output shows sensor-derived estimated centers rather than SKU-level
inventory. The accepted perception artifact has 454 `unknown_product` tracks,
does not estimate product extents, and is incomplete. The technical shots map
the reviewed 0.2-18.6 second estimator observation span across film shots 6-12.
These limits are stated in the video and the final independent verdict.

## Preserved setup evidence

- [Setup report](setup-20260920/report.md)
- [Setup evidence](setup-20260920/evidence/)

The setup report is historical evidence. Its former Astra/Luna assignments are
superseded by `EXECUTION_OVERRIDES.md`. The recorder `libcblas.dll` failure is an
implementation repair under active work and must not be described as passed
until a real capture succeeds.

The large runtime outputs remain local and are bound by the hashes above; the
source, acceptance logic, verdicts, contact sheet, and test transcript are
published on the branch for review.
