# Review index

## Master review checkpoint (2026-09-25)

- The initial renderer/evidence candidate was `f3bec7e7965887c31fdd8c6158bcd8dc7e6dd8a6` (tree `0d25ec2c99b5ba93c9f320797f91521c462aa80c`, parent `961abd0b30c239697f46a6099621f24ea5588b0e`). Its only production-source delta is the masthead/footer presentation repair in `simulator/technical_views.py`, and independent focused review of that exact renderer-only diff returned **APPROVE** with zero findings. Exact integrated review of `f3bec7e` subsequently found `RGB-F01`, `RGB-F02`, `IDXJ-F01`, `IDXJ-F03`, and `TECH-EVID-F02`; therefore `f3bec7e` has no exact combined-candidate approval.
- Exact review of successor `1be5e41d0ae4971baa2dd7ab43e0257ebf31488f` (tree `45fc1d2215377bc0b3818f55059972086aa1f962`) accepted the documentation fixes but returned **REQUEST_CHANGES** with blockers `RGB-1BE-F01` (claim inputs and timestamps were not all pinned and validated) and `RGB-1BE-F02` (the raw six-sample sheet was not pinned before writes). The current successor implements fail-closed validation of all 12 inputs and the index/capture/configuration/SLAM/perception bindings, and pins the sheet before output writes. Its exact combined review is pending; no final combined approval or production delivery is claimed.
- The combined code/test checkpoint is `6f24f38edf9f009db56f44a858f3e49565df6e85` (tree `d1280efd3fdf6dab86f07f316f0350ad544b0947`); this review-index/journal update follows it. It includes the G02 trial, Scene R13, standalone cart, M5 capture-validation WIP, dynamic technical-consumer WIP, SLAM replay-safety WIP, Hero R4, and approved fixture repairs.
- Hero R4 R2, the strict-RGB and presentation fixture fixes, the storyboard-path fix, and the SLAM R13 R2 series are independently approved. The integrated full suite passed 269 tests with two expected skips (173.174 seconds) on code-identical checkpoint `6f24f38`. The final combined tree `734737d` was independently reviewed and approved with no findings, then pushed to `origin/master`. See the latest entry in the [implementation journal](../IMPLEMENTATION_JOURNAL.md) for candidate identities and scope.
- Those historical approvals and the focused renderer approval apply only to their stated candidates and scopes. They do not close the fresh exact-candidate source reviews still marked open in the [implementation journal](../IMPLEMENTATION_JOURNAL.md), and the visual/artifact reviews below are not broader source approval. The fresh current-scene capture, SLAM, and perception runs are complete. The independently reviewed 720p LiDAR preview is available; seven final 1080p technical clips and the 1,350-frame final film remain open. Generated runs and temporary review scratch are kept out of Git.

## Production-quality grocery demo (G02, in progress)

- [Current G02 checkpoint](PRODUCTION_QUALITY_G02.md)
- Authoritative progress ledger: [implementation journal](../IMPLEMENTATION_JOURNAL.md)
- The isolated integration source has accepted asset, motion, scan-diagnostic, presentation-port, and transition slices. Scene visual realism and final selective-LiDAR delivery remain open.
- This section records the earlier isolated checkpoint. No G02 production film, paired final sensor capture, or release is claimed.

## Forensic repair checkpoint

- [Forensic repair pass — integration checkpoint](FORENSIC_REPAIR_PASS.md)
- Accepted source candidate: `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25` (tree `ce016082190e387a7b5e4a584db1d839063ea363`, parent `c780edff0cfa3c582da57545f3592703a892a0f9`)
- Review status: five independent exact-candidate approvals, including the completed SLAM-path audit and final exact source review with zero blockers
- Validation: full discovery 127/127, evaluator 22/22, reviewer selections 81/81 and 71/71, generator 69/69 byte exact, compile, PowerShell parse, and diff checks passed
- Installed replay: **BLOCKED by Windows Application Control**; no policy bypass was attempted

## Evidence

- [`evidence/current-goal-20260925/technical-lidar-preview/`](evidence/current-goal-20260925/technical-lidar-preview/README.md): independently reviewed combined 1280×720, 810-frame LiDAR preview, 21 representative stills, camera trace, and portable provenance summary. Smooth playback remains unverified; this is not final technical delivery.

- [`evidence/current-goal-20260925/`](evidence/current-goal-20260925/README.md): hash-bound, self-identifying Current-vs-R7 RGB video and six-sample sheet. Current is capture `20260925-183307101` at source `48de461`; R7 is capture `20260925-041644489` at source `3dc5107`. This is RGB visual evidence only: the views use 75-degree and 90-degree horizontal FOVs, and synthetic appearance limitations remain. The current sealed SLAM run is complete at manifest SHA-256 `803ca96a...` and attempt `20260926T042347915Z-d146ed0c1bbd4edfb296f655a0b5fe36`, with 204 nodes and exact sequence/topology gates. Perception is complete at 613 RGB frames, 204 usable LiDAR scans, 82,097 projected points, and 604 localized inventory rows. A separate bounded LiDAR preview is independently reviewed and remains preview-only.
- `evidence/forensic-preview-20260922/`: bounded 1280×720 before/after stills and capture receipts at 0 s and 8 s
- `evidence/forensic-baseline-20260922/`: failed ordinary capture evidence; Application Control blocked native OpenCV/ROS modules and no RGB/LiDAR artifact was produced
- Historical setup materials remain locally preserved outside this checkpoint and are not part of its committed evidence set.

This index records accepted source/integration status and bounded visual evidence. The preview packet does not claim a full film, final production delivery, owner acceptance, or installed Isaac/ROS production replay.
