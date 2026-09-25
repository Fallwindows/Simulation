# Review index

## Master review checkpoint (2026-09-25)

- The owner requested a reviewable Git checkpoint on `master`. It combines the G02 trial, Scene R13, the standalone cart asset, and clearly labeled WIP checkpoints for M5 camera capture validation, the dynamic camera technical consumer, SLAM replay safety, and Hero R4 artwork.
- The WIP checkpoints were interrupted before independent review and final integration validation. Their presence on `master` is for review, not acceptance.
- The full current-scene RGB/LiDAR capture, completed offline SLAM and perception, seven technical clips, and 1,350-frame final film remain open. Generated runs and temporary review scratch are kept out of Git.

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
