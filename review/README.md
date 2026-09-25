# Review index

## Master review checkpoint (2026-09-25)

- The combined code/test checkpoint is `6f24f38edf9f009db56f44a858f3e49565df6e85` (tree `d1280efd3fdf6dab86f07f316f0350ad544b0947`); this review-index/journal update follows it. It includes the G02 trial, Scene R13, standalone cart, M5 capture-validation WIP, dynamic technical-consumer WIP, SLAM replay-safety WIP, Hero R4, and approved fixture repairs.
- Hero R4 R2, the strict-RGB and presentation fixture fixes, the storyboard-path fix, and the SLAM R13 R2 series are independently approved. The integrated full suite passed 269 tests with two expected skips (173.174 seconds) on code-identical checkpoint `6f24f38`. The final combined tree `734737d` was independently reviewed and approved with no findings, then pushed to `origin/master`. See the latest entry in the [implementation journal](../IMPLEMENTATION_JOURNAL.md) for candidate identities and scope.
- These source/test results do not claim visual acceptance or production delivery. The fresh current-scene RGB/LiDAR capture, completed SLAM and perception, seven technical clips, and 1,350-frame final film remain open. Generated runs and temporary review scratch are kept out of Git.

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

- `evidence/forensic-preview-20260922/`: bounded 1280×720 before/after stills and capture receipts at 0 s and 8 s
- `evidence/forensic-baseline-20260922/`: failed ordinary capture evidence; Application Control blocked native OpenCV/ROS modules and no RGB/LiDAR artifact was produced
- Historical setup materials remain locally preserved outside this checkpoint and are not part of its committed evidence set.

This index records accepted source/integration status in the dependency-light scope. It does not claim a full film, installed Isaac/ROS production replay, owner acceptance, merge, push, release, or remote publication.
