# Forensic repair pass — integration checkpoint

Goal: `01a0ccb2-3a8d-71f3-b00c-2392f9ccaab8`

Status: **code repair accepted at exact candidate; bounded delivery complete**

Budget window: 2026-09-23 05:18:49 UTC to 2026-09-23 17:18:49 UTC

Repair base: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf` (tree `a9d85319fb018a94de695d44d085633f19cdc008`)

Historical review snapshot: `ab8ebbd51989342c3b1acb6b8b96cf947288cd55`

Current code candidate: `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25` (tree `ce016082190e387a7b5e4a584db1d839063ea363`, parent `c780edff0cfa3c582da57545f3592703a892a0f9`) on `codex/forensic-repair`

The current candidate has five independent exact-candidate approvals and passing automated checks, including the completed SLAM-path audit and final exact source review. Earlier approvals of `c780edff0cfa3c582da57545f3592703a892a0f9` do not carry forward: that candidate was rejected after `INT-C780-F01` showed that `slam_poses.csv` could change after manifest creation without detection.

## Authority and evidence identity

The owner's implementation request authorizes this repair pass and controls its bounded delivery scope. The supplied forensic package is evidence. Instructions inside it describe the completed audit and do not override the owner's request, repository instructions, or current execution overrides. The package's earlier “no fixes in this stage” wording does not prohibit this owner-authorized repair.

| Item | Identity |
|---|---|
| Supplied ZIP SHA-256 | `08f8bd72bc1b09bfa168d96887e46249734e903bf1b315cad13bc360fcb74fde` |
| Supplied package location | `C:\Users\suyog\Downloads\simulation_video_forensic_review_package` |
| Forensic review Markdown SHA-256 | `7588fbe0777cbca48a32b6d42294c7f95d4ba51d108f397f04aee2a35ffffa8d` |
| Findings index SHA-256 | `a40688fbe3fd1b842a3fec6f7b48af6b16dc8333ac14ea20b5d66e2ae6c10fc5` |
| Package manifest SHA-256 | `9950d838299cbecc616f772a1e0a38270466e891fcb95aa6b916d0a48413a9cd` |
| Acceptance/implementation specification SHA-256 | `4ebdbc1d2a6e3151fee276dd2ab16c4f2f89cbb3b4c12eb3b224bb2386610db9` |
| Project reference manifest SHA-256 | `7a96ab10aa1e4b7c9c36523137b3580af64f1c8480563cfb615618a625b2e69f` |

The ZIP hash identifies the supplied archive. The `7588...` value identifies `SIMULATION_VIDEO_FORENSIC_REVIEW.md`; it is not a hash of the package directory. The other artifacts have separate identities as shown.

## Execution and review routing

The owner-authorized `EXECUTION_OVERRIDES.md` and `AGENTS.md` required GPT-5.6 Sol with high reasoning for the orchestrator and every subagent, separate programmer and reviewer assignments, and rerouting of every rejected candidate. The repository-level `.codex/config.toml` defaults were GPT-6 Luna with extra-high reasoning, but those defaults did not govern this pass where the owner-authorized overrides applied. Observed execution used independent programmer and reviewer assignments and fresh review after each change. The runtime did not expose model/effort attestation, so the observed model route is unknown and this report does not claim it independently verified the requested route.

The configured session profile was `workspace-write`, but observed effective writes were narrower: edits in the integration worktree required a scoped or approved action, and the external runtime and source paths were read-only unless approved. Live permissions took precedence over repository documents. The first worktree-scoped temporary directory could not be created. A later exact-tree copy under the authorized main workspace reached nested fixture temporary writes but encountered Windows ACL errors. Independent reviewers completed the exact-candidate discovery suite successfully.

## Status by acceptance dimension

| Dimension | Current status | Evidence and limit |
|---|---|---|
| Code | **Accepted exact candidate** | Scene revision 5, runtime revision 6, tracking revision 12, and the evaluation provenance fix are integrated. Five independent reviewers approved `2b8fb77` at tree `ce01608`, including the SLAM-path audit and final exact source review. |
| Integration | **Accepted in dependency-light scope** | The candidate binds evaluation to the exact seven-artifact SLAM manifest, verifies the consumed PLY and pose CSV, preserves P13 matching semantics, and integrates sorted runtime pose production with the strict consumer contract. `c780edf` remains rejected in the chronology. |
| Technical validation | **Dependency-light pass; installed replay blocked** | Independent discovery passed 127/127 on the exact candidate; evaluator passed 22/22; reviewer integration selections passed 81/81 and 71/71. Generator, compile, PowerShell parse, and diff checks passed. Windows Application Control blocked the installed Isaac/ROS production replay. |
| Visual validation | **Bounded pass only** | Independent review passed two comparable 1280×720 stills at 0 s and 8 s for limited scene changes. It did not assess motion, temporal continuity, every storyboard shot, or the complete film. |
| Owner/external acceptance | **Not reviewed** | No owner acceptance of this integration checkpoint is recorded. |
| Full film | **Outside this repair pass and not produced** | The owner limited this pass; no 45-second, 1,350-frame film, 1080p final, silent master, or 12-shot acceptance package was produced. |
| Publication | **Local only** | No push, merge to `master`, release, or remotely verified publication is claimed. |

## Capability/readiness boundary

Artifact integrity is separate from feature readiness:

| Capability | Current result |
|---|---|
| Declared artifact integrity | The accepted candidate requires the exact seven-artifact SLAM set; validates role, frame, optimization state, map version, size, and SHA-256 for the consumed pose CSV; verifies the PLY; and derives alignment from retained verified pose bytes. |
| Current sensor returns | Raw measurement paths are preserved. The installed ordinary ROS/OpenCV replay was **BLOCKED by Windows Application Control** before usable RGB/LiDAR output; this is distinct from an ordinary baseline result. |
| Persistent map history | No time-indexed map-revision history is implemented; a finalized cloud alone is insufficient. |
| Product recognition | Missing. Outputs must remain `unknown_product`; simulator identity may not fill the gap. |
| Supported 3D extents | Missing. Surface support must not be relabeled as a full object center or cuboid. |
| Visual proof | Two frozen RGB stills provide bounded scene evidence only. No motion or full-film readiness claim is supported. |

An unknown or center-only output cannot pass a capability-ready gate merely because its files are well formed.

## Integrated and candidate history

### Evaluation

| Revision | Exact identity | Verdict and evidence | Integration state |
|---|---|---|---|
| 1 | `ac21aecec2b6eb6244f61ef276b79e4bcb066556`; tree `3ec668fa70b04de5766c47bfe88022f3a4e78cfd` | **REQUEST_CHANGES**. Nine tests passed, but review reproduced nonmaximum greedy matching, invalid earliest-pose selection, and unfrozen eligible-recall provenance. | Not integrated as accepted content. |
| 2 | `86277e63a28190e8aebb8138be4463ca8a348fb9`; tree `c4e3c401529bd99fc6e89a15c5e5e4f22eea5959` | **REQUEST_CHANGES**. Matching/provenance were corrected; invalid or nonfinite estimator-start handling still required fail-closed behavior. | Not accepted. |
| 3 | `18c6578969350cb1e75907e2a28bb14d73002efe`; tree `d2c95221ae81217d7942b4e2d7994c051f880bb7` | **APPROVE**. `tests.test_inventory_evaluation`: 15/15. Maximum-cardinality/minimum-cost matching, hash-bound frozen eligibility, estimator-aligned first pose, and NaN/Inf failure behavior were reviewed. | Exact approved content imported through `5cef12e`, `3040782`, `12657f1` and retained in the accepted candidate. |
| 4 | `e3c131edbe0ba1f679ab040d4c1cda43d7d29dcb`; tree `b8555768d86a817c9c6b502a0fc9495bbf4be641` | **REQUEST_CHANGES**. An initial approval was withdrawn after `EVAL-R4-F01`: the allowlist did not require equality with the exact seven-artifact set, and a PCD+PLY-only manifest could complete. | Rejected; preserved here because the later approval does not erase this verdict. |
| 5 | `f9c3651202aab9f46c014fb6868831cb33e68773`; tree `38e2b1f03c50c55ebc9fdac88591bc94621680f4` | **APPROVE by two independent reviews**. It requires all seven artifact identities and rejects every single-artifact omission before truth access while retaining the revision-3 matching and eligibility behavior. Focused evaluator suite: 5/5. | Provenance logic ported into the P13-preserving integration as `8fc9ff9a352bdcbc65a5ee1113bb666ba8f25cd2` (tree `c443370e6d2092458349d329ee1db11437457509`). |
| 6 | `c780edff0cfa3c582da57545f3592703a892a0f9`; tree `f35ecf335968a68c77990b7a2002a19baeb28cec` | **REQUEST_CHANGES** despite earlier independent approvals. `INT-C780-F01` confirmed that `slam_poses.csv` was reopened without checking its declared metadata/size/SHA; replacement after manifest creation could change score alignment while evaluation reported complete. | Rejected overall. Its runtime, tracking, scene, and P13 work was retained as the parent of the targeted fix. |
| 7 | `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25`; tree `ce016082190e387a7b5e4a584db1d839063ea363`; parent `c780edff0cfa3c582da57545f3592703a892a0f9` | **APPROVE by five independent exact-candidate reviews.** The fix validates `slam_poses.csv` role, frame, optimized flag, map version, path, size, and SHA-256 before truth access, retains the verified bytes, and parses the start timestamp from those bytes. Pre-validation replacement and post-validation mutation regressions close the observed TOCTOU path. Evaluator suite: 22/22. | Accepted current integration candidate; the SLAM-path audit and final source review approved the exact tree with zero blockers. |

Changed paths: `simulator/perception/inventory_evaluation.py`, `tests/test_inventory_evaluation.py`.

### Scene and retail assets

| Revision | Exact identity | Verdict and evidence | Integration state |
|---|---|---|---|
| 1 | `9e093be44e2c89b2312cc691befaab02056edec5`; tree `7f8997aa583083a0a07ae0be98dbb20f00c7d3ef` | **REQUEST_CHANGES**. Eleven tests and 34/34 regeneration passed, but review found 355 fruit intersections plus invalid USD schema/orientation coverage. | Rejected. |
| 2 | `d1aca688a62cf604dd20f8cb1cba43db7cd07b1b`; tree `7524fb514fb759b0e8ac34e8d78116bc5c834655` | Append-only packing/USD correction; no standalone approval claimed. | Superseded by cumulative approved candidate. |
| 3 | `1dd27c24255993a2c5ce58416b9df1ee12af6ab1`; tree `856cf7a283ef538174a3e696ee7c69aada22968a` | Append-only rail-clearance/orientation correction; no standalone approval claimed. | Superseded. |
| 4 | `22bcef1d3bc1648e6b1cbe2bb90fd7d999e603b6`; tree `b4f148671fd05c18cc33217fd83e44c69b88a4bb` | Append-only bottle-facing/crate-test correction; no standalone approval claimed. | Superseded. |
| 5 | `91340cfc47a02f18341ddc5a80963337ed6aa394`; tree `1db83f7bc0a763bc01512f447c6bb1ddc8276e88` | **APPROVE**. Geometry/retail tests 12/12; generator 34/34 byte exact; 1,248 fruit-versus-crate component checks; 1,995 instance IDs preserved. | Exact approved files imported as `42ead110e26c269289e8770d3530851bfa0447c2` and accepted in the combined tree. |

Changed paths: `simulator/environment/aisle_builder.py`, `tools/retail_assets/generate_packaging.py`, `tests/test_retail_assets.py`, and 34 generated `assets/retail/usd/` files.

### Tracking and perception

| Revision | Exact identity | Verdict and evidence | Integration state |
|---|---|---|---|
| 1 | `90ab2dd7a675754f142cc12eda76da235e7408a2`; tree `87d1cfd36968ab1a39b7b87f475ffb40ff56cdfe` | Initial repair candidate; reviewer regressions were addressed in revision 2. No approval claimed. | Superseded. |
| 2 | `a8a37b22dd1a009b3d86ec0dc5557471445d5299`; tree `ea4642e321cd883bbc2d65e31b5ac6caa65e2ab3` | **APPROVE**. `tests.test_perception`: 18/18. | Exact approved content imported through `83a4697` and `29d5847`. |
| 3 | `109efaa2c992a6974ee3809fe8b5ca77adbf68c1`; tree `01841121055e2087d705fee3aa97a0f022d6681b` | **REQUEST_CHANGES**. Same-appearance crossings/occlusion switched IDs and production corrected-map use was incomplete. | Rejected. |
| 4 | `294b7ebd2101e5660f5e91acb53ad63193b20175`; tree `b67d91fde052b2a4aa582c5266b676e8f9353f6a` | **REQUEST_CHANGES**. Long-gap stale velocity, changed-view fragmentation, missing producer compatibility, and nonfinite values remained. | Rejected. |
| 5 | `9cf878fb2f330768b33b4889207f5ebfa718fa2f`; tree `adb9cbc9a7c98dd1286f250fbf87cdf517a753d4` | **REQUEST_CHANGES**. Archive threshold was one frame late and expired-track export metadata was lost; map-based revisit coverage remained incomplete. | Rejected. |
| 6 | `dd83f098ddca0ba7fabd5fa0efa481c13dbdac2b`; tree `d7bccd8c5966e773bce19fc708fc3546588f1b3c` | **APPROVE by two independent Sol reviews**. `tests.test_perception`: 28/28. Reviewer routing was configured GPT-5.6 Sol/high; runtime model attestation unavailable. | Not integrated because later map-artifact work changed the chain. |
| 7 | `1ba345591709532801b9b2600948c75a26338898`; tree `72bcf8e4f1937516b7e9ca19665b06fc35f35e33` | **REQUEST_CHANGES by two independent reviews**. Consumer accepted incomplete top-level manifests, unequal graph versions, and Boolean `map_pose_sample_count` because `True == 1`. Producer/consumer timestamp compatibility remained an unverified integration residual. | Rejected. |
| 8 | `0e27bf77558ceb50d4a26e212e1a6dde1b4856e5`; tree `9487418702688895a63ef1224940c4adcd935750` | **Split verdict: one APPROVE, one REQUEST_CHANGES**. 29/29 tests; manifest/version fixes passed one review, but strict positive-integer/non-Boolean count validation remained. | Rejected overall; not integrated. |
| 9 | `db1cc1b1084b7bbdb11aa3d3684b05386f4d9f67`; tree `d353ae45ce2672a70c5f0a8abba2d8186afc9052` | Cumulative strict pose-provenance hardening after revision 8; no standalone approval claimed. | Superseded. |
| 10 | `f4b74364c5867bb4823bfadd9cda8d342dad064a`; tree `7b2707d1b0dcf28b26566ff1fdf691a1a6e36a7b` | Producer-contract alignment candidate; no standalone approval claimed. | Superseded. |
| 11 | `a45d9516b4c268b31b673ea6e785bd592274c239`; tree `9eb2cc6d2acd6df3ca35238eadfade4af21671ff` | **REQUEST_CHANGES.** Consumer validation was strict, but runtime revision 5 could serialize unique decreasing timestamps that the consumer correctly rejected. | Rejected for producer-consumer incompatibility. |
| 12 | `cf997b059dfce7c608890acdaa30594e8f5033da`; tree `e8432a28e891f0259d3afadf7d4503a47f45c931`; parent revision 11 | **APPROVE by three independent reviews.** The append-only regression confirms the sorted runtime revision-6 stream is accepted while decreasing, duplicate, and unequal timestamp streams remain rejected. `tests.test_perception`: 31/31; direct runtime-to-tracker probe passed. | Exact files imported in `5d64ec2c08cd57cd31feab3302bff25ad0f37c66`. |

Changed paths: `simulator/perception/rgb_tracking.py`, `tests/test_perception.py`.

### Capture and SLAM runtime

| Revision | Exact identity | Verdict and evidence | Integration state |
|---|---|---|---|
| 1 | `b63fc97d3814f0c3a65c833da633a2a586d81f78`; tree `cf8c88a6631986020fbfd6c3ad906980d2b9eb26` | Initial repair candidate; no approval claimed. | Superseded. |
| 2 | `7d1672bf26e80b38034470f5f7456143bd1ded42`; tree `2d331d25ec7b2e8e5d8c16af066d7e82cdc3aafe` | Append-only map-pose boundary correction; no approval claimed. | Superseded. |
| 3 | `9e7889f4109f01e9e9e35db9195166ba99c85741`; tree `0748bc212bc196d4ea64de332e1b9c2dea2c8601` | **One APPROVE and one REQUEST_CHANGES**. Candidate tests reported 16/16 capture architecture and 14/14 contracts/static. Blocking review found incomplete appearance dependency hashing and incremental rather than version-bound final optimized graph poses. | File content imported as `686821b1f86ab62ea575657cfc97c878fef9ba31`, but not accepted. |
| 4 | `7b6a2127a06b6379497d0a9201c39a009809c7e4`; tree `d25bbc32b07c49d12a5a860dd2f4ea8285306ee8` | **REQUEST_CHANGES.** Although its focused checks passed, exact review found map-latching/final-state behavior that did not establish a single auditable final graph/cloud/pose state. | Rejected; the later revision supersedes it. |
| 5 | Runtime state carried by the cumulative `e3c131edbe0ba1f679ab040d4c1cda43d7d29dcb` tree | **REQUEST_CHANGES at integration.** It closed the map-latching issue but could serialize unique out-of-order callback timestamps, conflicting with tracking revision 11's required monotonic pose stream. | Rejected for producer-consumer mismatch. |
| 6 | `71eabd2195dc6640d545f4786e40a83e48755631`; tree `14d89e4047848ac4edc0af8d1be0acbf30141a9c`; parent `e3c131edbe0ba1f679ab040d4c1cda43d7d29dcb` | **APPROVE by two independent exact reviews.** Full callback odometry rows are sorted by timestamp before dense pose generation, hashing, and raw/corrected/legacy CSV serialization; duplicates still fail. Capture suite: 18/18; reviewer span suite: 22/22; reversed-order probe passed. | Exact runtime files imported in `5d64ec2c08cd57cd31feab3302bff25ad0f37c66`. Installed replay remains blocked. |

Changed paths across the integrated runtime chain: `evaluation/rgb_video_recorder.py`, `scripts/capture_simulation.ps1`, `scripts/run_inventory_offline.ps1`, `scripts/run_slam_offline.ps1`, `scripts/validate_rtabmap_db.py`, `simulator/capture/manifest.py`, `simulator/capture/rosbag_capture.py`, `simulator/capture/slam_observer.py`, `config/mapping/rtabmap/params.yaml`, `ros2_ws/src/grocery_sim_mapping/config/params.yaml`, `ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py`, and `tests/test_capture_architecture.py`.

### Combined integration checkpoints

| Checkpoint | Exact identity | Verdict and evidence |
|---|---|---|
| Provenance port | `8fc9ff9a352bdcbc65a5ee1113bb666ba8f25cd2`; tree `c443370e6d2092458349d329ee1db11437457509` | Preserved P13 maximum-cardinality/minimum-cost matching and frozen eligibility while porting the approved exact-set provenance guard. |
| Runtime/tracking import | `5d64ec2c08cd57cd31feab3302bff25ad0f37c66`; tree `7088b402bd37dbe64edd7bb83d83f2bc73e64a2e` | Imported only the exact approved runtime revision-6 and tracking revision-12 files. |
| USDA regression correction | `c780edff0cfa3c582da57545f3592703a892a0f9`; tree `f35ecf335968a68c77990b7a2002a19baeb28cec` | The normals assertion was corrected to match current USDA representation. Later overall verdict: **REQUEST_CHANGES** for `INT-C780-F01`. |
| Current candidate | `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25`; tree `ce016082190e387a7b5e4a584db1d839063ea363`; parent `c780edff0cfa3c582da57545f3592703a892a0f9` | Changes only `simulator/perception/inventory_evaluation.py` and `tests/test_inventory_evaluation.py`. Five independent exact approvals, including the completed SLAM-path audit and final source review. |

## Visual evidence

The before images are from the immutable repair base. The after images are from the approved scene candidate. All four are direct Isaac RGB annotator captures at 1280×720, headless, `RaytracedLighting`, 8 ray-tracing subframes, ROS disabled, and no OpenCV dependency.

| Evidence | Source/time | SHA-256 | What it supports |
|---|---|---|---|
| `before_rgb_t000000ms.png` | Base, 0 s | `56906e52539be949f6c3d98dc35b6a88de8bc864e5890ee440a1c3cc3f73566b` | Comparable baseline composition. |
| `after_scene_91340_t000000ms.png` | Scene revision 5, 0 s | `3cec305dae8f5bb55c21bf59f2dd967ae0888a1a3991a5c5bfed31ab96767512` | Bounded early-aisle comparison. |
| `before_rgb_t008000ms.png` | Base, 8 s | `534d88292512a109a3e9ec726d3b9dbebdde6cc0f6a9a8d37d6f682d9164fa84` | Comparable baseline produce view. |
| `after_scene_91340_t008000ms.png` | Scene revision 5, 8 s | `b173c149a3e3f8d909931151dac1f01e82304c1726ab117b08671dcfd797ef08` | More open rail-sided crate and lower fruit protrusion in this sample. |

Matching JSON receipt hashes, in row order, are `9f443075751fec6fa89d28cafc0d413f13fda2976ade0846f81d543005e2d6b9`, `5ac9649457d9078fe89dae79797098c3642e975b23c7fd1217c1ffb573ba5885`, `3092e7405e35e61cfb5bf47faff2065ff7c412231957457de51aa82d4ee020e4`, and `c19f8792413576f0f4e203d4bd6d961c57fd066924f4dd3e82651f53d366d6c5`. The after capture script hash is `6a91eed22929ca89584ae07756a5ecb1c17d6c50d22939bcf5247f3759ceb7ea`. Full receipts/scripts are under `review/evidence/forensic-preview-20260922/`.

Independent verdict: **BOUNDED VISUAL PASS** for only 0 s and 8 s. The reviewer observed the improved crate/fruit silhouette and visible mixed package families. Much variety already existed, and the samples do not isolate every new bottle facing. No person or cart is visible in the four images. This evidence does not establish motion, transitions, tracking, temporal consistency, every camera view, hero-quality materials, or full-film acceptance.

The ordinary baseline attempt under `review/evidence/forensic-baseline-20260922/` simulated 1,230 frames, but Windows Application Control blocked native OpenCV/ROS modules. It produced zero observed RGB frames and zero LiDAR clouds. This is failure evidence only; no policy bypass, security change, or reinstall was attempted.

## Finding disposition matrix

Every supplied finding is listed once below. “Approved” applies only to the exact candidate and stated scope; it does not imply combined integration, visual, owner, or film acceptance.

| ID | Applicability and evidence | Action; changed files/commits | Verification | Remaining dependency; independent review |
|---|---|---|---|---|
| A01 | **Open/partial.** Base lacked a capability-ready gate; file integrity could be mistaken for capability. | Added this report's readiness boundary; runtime/evaluation now carry exact-set, version, size, and hash checks through `2b8fb77`. | Report audit plus candidate provenance tests and five exact-candidate approvals. | Machine-readable capability gate still needed; artifact integrity alone does not supply recognition, extents, history, or presentation. |
| A02 | **Historical process finding.** The pinned film review accepted a narrower result than the earlier specification; the owner now bounds this pass. | Preserved verdict history and separated acceptance dimensions here. | Source/spec/hash comparison. | No source defect; owner acceptance unrecorded. |
| A03 | **Partial/open.** No presentation renderer expresses returns, history, supported extents, cards, and shared occlusion. | Boundary math/data improved; no renderer added. | Static source/dataflow audit. | Presentation architecture/visual proof missing; no approval. |
| A04 | **Applied process correction.** Earlier work reduced delivery uncertainty before capability risk. | Repaired contracts and used two low-cost 720p stills before long rendering. | Candidate history/evidence receipts. | No full preview requested or produced. |
| A05 | **Owner-bounded.** 720p is preview, 1080p remains later final target, 4K excluded. | Produced 1280×720 stills only. | PNG receipts verify dimensions/config. | Image formation beyond two samples/native 1080p film unverified. |
| V01 | **Repaired in source.** Product geometry/schema and scene use corrected; assets remain simple proxies. | Scene revision 5; `42ead11`; generator plus 34 USDA files. | 12/12, 69/69 current-tree byte regeneration, bounded visual pass. | Accepted in the combined tree; hero realism remains limited. |
| V02 | **Repaired in source.** Winding, normals/UV conventions, and independent orientation checks corrected. | Generator, USDA assets, tests; `42ead11`. | Byte equality and orientation/schema regressions. | Scene approved; evaluated-stage material behavior not separately proved. |
| V03 | **Repaired in source.** Invalid texture type, normals interpolation authoring, and unsupported cone property replaced. | Scene cumulative revision 5 / `42ead11`. | Schema assertions and 34/34 regeneration. | Approved; no pxr evaluated-stage acceptance run. |
| V04 | **Partial.** Package geometry is more coherent, but close-up objects/materials remain proxy quality. | Scene revisions; no hero-asset replacement. | Two 720p stills. | Close-up/high-resolution visual acceptance open. |
| V05 | **Repaired analytically.** Fruit placement satisfies component clearance, shelf contact, and upper clearance. | Aisle builder, crate assets, tests; `42ead11`. | 1,248 component checks; 12/12. | Approved; authored USD tests are not evaluated-physics proof. |
| V06 | **Bounded improvement.** Early aisle exposes soda while preserving 1,995 IDs; counts changed only cereal 402→398, snacks 285→279, soda 346→356. | Aisle builder; `42ead11`. | Layout tests and two stills. | Samples do not isolate every facing/all camera coverage. |
| V07 | **Partial.** Sampled aisle is more coherent, but fixture/material/lighting language remains simple. | Scene candidate/stills; no broad overhaul. | Bounded visual pass. | Photorealism and motion review open. |
| V08 | **Not reproduced in active source; sample absence confirmed.** World builder has no shopper/cart assembly; neither appears in four stills. | No new character code; owner-directed sampled result excludes actor/cart. | Source plus still inspection. | Absence across motion/full film unproved. |
| V09 | **Repaired in sampled scene.** Ninety-six fruits use four rail-sided crates without prior measured overlaps. | Scene revision 5 / `42ead11`. | Analytic checks and 8 s comparison. | Approved; evaluated collision/physics unverified. |
| V10 | **Partial.** Direct Isaac still capture proved ray-traced scene rendering. | Preview scripts/receipts. | Two after frames, 1280×720, 8 subframes. | Ordinary capture blocked; no sequence/final render. |
| P01 | **Missing capability.** Color components do not establish product identity. | No model; preserve `unknown_product`. | Dataflow audit. | Needs model/runtime/weights/license/labeled set. |
| P02 | **Repaired.** Tracks initialize from first detection, not image origin. | Tracking revision 2 via `83a4697`/`29d5847`, retained through revision 12. | Stationary/corner/velocity regressions; accepted combined tree. | No remaining reproduced source defect. |
| P03 | **Repaired for tested proposal behavior.** Resolution-aware thresholds and no hard truncation address reproduced defect. | Same tracking files/commits. | Multi-resolution regressions. | Real shelf recall/instance separation unmeasured. |
| P04 | **Substantially repaired in tested scope.** Assignment/archive/retired-summary/same-appearance cases strengthened; changed-view re-entry remains conservative without world support. | Tracking revisions 3–12; approved exact files integrated in `5d64ec2`. | Revision 12 received three component approvals; accepted combined tree; perception suite 31/31. | Real changed-view/world-supported canonicalization remains unmeasured. |
| P05 | **Partial.** Co-visible veto prevents reproduced adjacent merge; non-co-visible consolidation remains heuristic. | Approved revision 2. | Co-visible-neighbor tests. | Real shelf GT/changed-view metrics absent. |
| P06 | **Repaired.** Raw/canonical ID namespaces separated at annotation/export boundaries. | Approved revision 2, retained through revision 12. | Collision regression and accepted combined tree. | No remaining reproduced source defect. |
| P07 | **Repaired.** Spatial-grid membership refreshes when centers move. | Approved revision 2, retained through revision 12. | Grid movement regression and accepted combined tree. | No remaining reproduced source defect. |
| P08 | **Partial.** Supporting LiDAR measurements are separate from RGB counts; raw points stay unmodified. | Tracking revisions 2–12. | Support/count, retired-export, and accepted combined-tree tests. | Confidence remains heuristic and lacks real shelf calibration. |
| P09 | **Repaired in the accepted source candidate; runtime replay blocked.** Final graph poses/cloud, manifest status, versions, hashes, counts, and sorted timestamps now describe one checked state; evaluation consumes retained verified pose bytes. | Runtime revision 6, tracking revision 12, exact-set port `8fc9ff9`, and pose binding `2b8fb77`. | Runtime 18/18, perception 31/31, evaluator 22/22, cross-contract probes, and five current-candidate approvals including the SLAM-path and final source reviews. | Installed Isaac/ROS replay was blocked by Windows Application Control. |
| P10 | **Partial.** Transform/interpolation checks and observation support improved. | Tracking/runtime chain. | Nonidentity/malformed-input regressions. | No per-return deskew or supported box-center/extents. |
| P11 | **Repaired for tested layouts.** PointCloud2 parsing validates endianness, fields, offsets, steps, and finite values. | Approved revision 2, retained through revision 12. | Binary-layout regressions and accepted combined tree. | Live sensor-format diversity remains outside the dependency-light evidence. |
| P12 | **Missing core capability.** Recognition and evidence-backed 3D extents absent. | No synthetic identity/cuboid added. | Dataflow audit. | Explicit recognition/cuboid model/schema decision needed. |
| P13 | **Repaired in the accepted candidate.** Maximum-cardinality/minimum-cost matching, aligned valid poses, finite inputs, frozen eligibility, exact artifact-set validation, and retained verified pose bytes. | Evaluation revisions 3–7 via `5cef12e`, `3040782`, `12657f1`, `8fc9ff9`, and `2b8fb77`. | Evaluator 22/22; five exact-candidate approvals. | Real labeled data remains absent. |
| R01 | **Open missing capability.** No time-indexed map revisions/history for current-versus-persistent rendering. | Runtime improves final state only. | Dataflow audit. | Map-history schema/correction history/renderer absent. |
| R02 | **Partial boundary only.** Raw endpoints/calibration exist; no data-driven ray presentation. | No renderer change. | Contract inspection. | Origin/endpoints/occlusion/visual acceptance open. |
| R03 | **Partial by owner direction.** Foreground actor/cart absent in samples; no general dynamic mask/history. | Scene evidence plus runtime final-state work. | Source/still inspection. | Dynamic residue test/map history missing. |
| R04 | **Partial evidence.** Receipts record camera path, rig pose, exact time. | Preview JSON/scripts. | Receipts at 0 s/8 s. | No five-shot technical camera/motion acceptance. |
| R05 | **Open presentation gap.** Raw scans preserved; filtering limited to defensible support. | Tracking only; no point renderer. | Dataflow tests. | Point-density/color/depth treatment absent. |
| R06 | **Open presentation gap.** No shared world/time/occlusion model for paths/markers. | No renderer/card change. | Static audit. | Presentation interface/tests needed. |
| R07 | **Open capability/presentation gap.** IDs/centers are not a product-detail view. | No card renderer. | Static audit. | Depends on recognition/extents/provenance/presentation review. |
| R08 | **Deferred by owner scope.** Five-shot camera diversity/full storyboard excluded. | No film camera timeline change. | Scope comparison. | Required for later full-film run. |
| R09 | **Deferred by owner scope.** No continuous presentation timeline/transitions added. | No timeline change. | Static audit. | Required for later full-film work. |
| R10 | **Open for film delivery.** Encoder remains lossy/coarsely resumable; no lossless master. | No master-frame/resume work. | Pipeline inspection. | Full film outside this pass. |
| C01 | **Repaired in source/tests.** Sensor-only manifests validate declared inputs without truth; archive mode retains truth checks. | Runtime chain through revision 6 and accepted evaluation candidate. | Deleted-shard/truth-free and pre-truth rejection regressions; accepted exact combined tree. | Installed withheld-truth execution was blocked. |
| C02 | **Repaired in source/tests.** Appearance dependency hashing includes normals interpolation and relevant index/interpolation declarations. | Runtime revision 6 chain; manifest/tests integrated through `e2c7b85` and `5d64ec2`. | Independent mutation coverage within capture suite; generator 69/69 byte exact; accepted combined tree. | No pxr evaluated-stage acceptance run. |
| C03 | **Bounded source fix accepted at component scope.** Writers are synchronous; close ownership, idempotence, failure metadata, and stalled-close behavior are covered. | Runtime chain through revision 6. | Capture architecture 18/18 and reviewer span 22/22. | Installed failure smoke blocked by Windows Application Control. |
| C04 | **Repaired in source/tests.** One replay clock, done signal, map acknowledgment, mapper-before-DB shutdown, final graph/cloud pairing, sorted dense corrected rows, and strict consumer checks are integrated. | Runtime revision 6 plus tracking revision 12; imported in `5d64ec2`. | Component approvals, runtime-to-tracker probe, 127/127 full discovery, 81/81 and 71/71 reviewer selections, and five exact-candidate approvals. | Installed replay/restart was blocked by Windows Application Control. |
| C05 | **Not reproduced on repair base.** One PyAV decode context; no eager probe call. | No change. | Call-path inspection. | Recheck if output path changes. |
| C06 | **Not a source defect here; delivery open.** No film artifact required/published. | Local report and small evidence only. | Listed hashes. | Remote access/film review unverified. |
| T01 | **Improved synthetic coverage.** Stationary/resolution/layout/identity/archive/revisit/malformed and runtime-stream compatibility cases added. | Perception tests through revision 12. | Revision 12: 31/31 and three approvals. | No real shelf ground truth. |
| T02 | **Partial.** Authored checks cover support/separation/clearance/schema/orientation/regeneration. | Retail tests; scene revision 5 / `42ead11`. | 12/12 plus 34/34. | Not pxr-evaluated bounds/physics. |
| T03 | **Partial dependency coverage; representation coverage absent.** Cutoff/transform/manifest/schema, exact-set, retained-byte, and producer-consumer tests added. | Capture, perception, and evaluation tests. | Exact candidate: evaluator 22/22; full discovery 127/127; reviewer selections 81/81 and 71/71. | No ray/history/occlusion/card/extent renderer tests. |
| T04 | **Process correction applied.** Implementation and review used separate assignments; every rejection was routed back and every changed candidate received fresh review. | This review history; no product code. | Exact commit/tree/verdict records, including the withdrawn eval-rev4 approval, `c780edf` rejection, and five approvals of `2b8fb77`. | Owner-authorized overrides required GPT-5.6 Sol/high; runtime model/effort attestation was unavailable, so the observed route is unknown. |
| T05 | **Owner correction applied.** Bounded proof, one report, focused tests/two stills; no film-completion claim. | This report/evidence index. | Scope/status audit. | Owner acceptance and later film requirements remain separate. |

## Verification recorded so far

- Exact current candidate `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25` / tree `ce016082190e387a7b5e4a584db1d839063ea363`: five independent exact approvals; final source review reported zero blockers.
- Independent full discovery: 127/127. The final source reviewer independently repeated 127/127 on the exact candidate.
- Evaluation: `tests.test_inventory_evaluation` — 22/22.
- Component/integration reviewer selections: 81/81 and 71/71.
- Runtime component: `tests.test_capture_architecture` — 18/18; runtime review span — 22/22; reversed-order callback probe passed.
- Tracking component: `tests.test_perception` — 31/31; runtime-revision-6 to tracking-revision-12 compatibility probe passed.
- Retail generator: 69/69 generated files byte-identical to `assets/retail`; final source review repeated 69/69.
- `compileall` passed for `simulator`, `tests`, `tools`, and `ros2_ws/src/grocery_sim_mapping/launch`.
- PowerShell parser passed all ten checked scripts.
- `git diff --check d5e825c8f6dab77aa6a1007c9731c226b588dfcf..2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25` passed.

The first orchestrator attempt to create a worktree-scoped `TEMP` directory was denied, so that directory was never created. A later exact-tree copy run under the authorized main workspace reached nested fixture temporary writes, where Windows ACL failures produced 46 environment errors and no assertion failures. Its exact uniquely named copy folder was removed after the resolved target was verified and scoped elevated cleanup was approved. These environment failures do not replace the independent successful 127/127 exact-tree reviewer runs.

## Delivery limits and remaining work

- Installed Isaac/ROS production replay and restart validation were **BLOCKED by Windows Application Control**. No policy bypass, security change, or reinstall was attempted.
- No 45-second or 1,350-frame full film, native 1080p final, silent master, twelve-shot acceptance package, or motion/transition review was produced.
- No product-recognition model, evidence-backed full 3D extents, time-indexed map history, or technical presentation renderer was added.
- Visual evidence remains limited to the two comparable 1280×720 timestamps at 0 s and 8 s.
- No owner acceptance, merge to `master`, push, release, or remote publication is claimed.

This checkpoint completes the owner-authorized source repair and bounded evidence pass. Production replay, full-film work, and owner acceptance remain separate future decisions.
