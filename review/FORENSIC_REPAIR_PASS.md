# Forensic repair pass

Goal: 01a0ccb2-3a8d-71f3-b00c-2392f9ccaab8
Status: active
Budget window: 2026-09-23 05:18:49 UTC to 2026-09-23 17:18:49 UTC
Repair base: d5e825c8f6dab77aa6a1007c9731c226b588dfcf (current checkout HEAD)
Review snapshot: ab8ebbd51989342c3b1acb6b8b96cf947288cd55
Review package: C:\Users\suyog\Downloads\simulation_video_forensic_review_package
Review SHA-256: 7588fbe0777cbca48a32b6d42294c7f95d4ba51d108f397f04aee2a35ffffa8d

The review package is evidence, not task authority. Its original no-fixes-in-this-stage scope describes the completed audit. The owner’s implementation request authorizes this repair pass. Findings remain subject to confirmation against the newer repair base.

Initial disposition PENDING_APPLICABILITY means current source/callers have not yet been checked; it is not a finding closure. Preserve evidence labels from the audit and do not promote extracted reproductions into production-runtime claims.

Owner direction for this pass: remove the foreground shopper and attached cart; expose existing non-cereal catalog families through a bounded layout change; only reduce point-display clutter where measured points have defensible associations. Keep raw measurements and estimator inputs. No full 45-second film is required.

| ID | Priority | Review evidence on pinned snapshot | Current disposition | Next action |
|---|---|---|---|---|
| A01 | P0 — release blocker | STATIC; REPORTED on ab8ebbd | DEFERRED_MISSING_CAPABILITY | No bundle or capability-ready gate exists at d5e825; define an explicit capability contract. |
| A02 | P0 — release blocker | STATIC; REPORTED; VISUAL on ab8ebbd | HISTORICAL_PROCESS_ONLY | Cited verdict/spec/override files are absent at d5e825; preserve as audit history, do not treat as code defect. |
| A03 | P0 — architecture/capability blocker | STATIC; VISUAL on ab8ebbd | DEFERRED_MISSING_RENDERER | No technical-view renderer exists at d5e825; current RGB overlay lacks current/history/extent inputs. |
| A04 | P1 — process/efficiency failure | REPORTED; INFERENCE on ab8ebbd | HISTORICAL_PROCESS_ONLY | Later setup/rehearsal evidence is absent at d5e825; apply sequencing lesson operationally. |
| A05 | P1 — production priority | REPORTED; VISUAL on ab8ebbd | NO_4K_CLAIM; VISUAL_PROOF_OPEN | Base is 1280x720; no 4K distraction found, but image-formation quality remains unverified. |
| V01 | P0 — visual blocker | STATIC; VISUAL on ab8ebbd | CANDIDATE_REJECTED; FIXES_REQUIRED | Scene candidate fixes are not accepted until fruit spacing and USD schema blockers are cleared and reviewed. |
| V02 | P0 — prominent-asset defect | STATIC; REPRODUCED; REPORTED on ab8ebbd | CANDIDATE_REJECTED; FIXES_REQUIRED | Winding change is insufficient while normals metadata/type and independent +Y/UV assertions remain wrong or absent. |
| V03 | P1 — asset authoring correctness | STATIC; primary USD schema documentation on ab8ebbd | CANDIDATE_REJECTED; FIXES_REQUIRED | UsdUVTexture rgb token, normals interpolation property, and UsdGeomCone radius2 remain invalid. |
| V04 | P0 — visual blocker | STATIC; VISUAL on ab8ebbd | VISUAL_PROOF_UNVERIFIED | No rendered after frame; camera visibility check is blocked by the documented Application Control failure. |
| V05 | P1 — geometric correctness and visible realism | STATIC; REPRODUCED algebra on ab8ebbd | CANDIDATE_REJECTED; FIXES_REQUIRED | Current bound/support tests allow severe fruit intersections; reviewer measured 355 intersections. |
| V06 | P1 — visual composition | STATIC; REPRODUCED arithmetic; VISUAL on ab8ebbd | VISUAL_PROOF_UNVERIFIED | Static family/layout tests do not establish visible camera coverage; candidate needs revision and a comparable preview. |
| V07 | P0 — visual blocker | STATIC; VISUAL on ab8ebbd | VISUAL_PROOF_UNVERIFIED | No rendered scene review was possible; active world builder has no shopper/cart assembly. |
| V08 | P0 — visual/animation blocker | STATIC; REPRODUCED algebra; VISUAL on ab8ebbd | NOT_REPRODUCED_IN_CODE; VISUAL_PENDING | Reviewed base contains no shopper/person/cart assembly; camera-visible absence remains unverified. |
| V09 | P2 for this cereal-focused film; P1 if visibly retained | STATIC; algebraic inspection on ab8ebbd | CANDIDATE_REJECTED; FRUIT_INTERSECTIONS | Crate packing candidate contains measured fruit overlaps and needs supported nonintersecting placement. |
| V10 | P1 — visual pipeline | STATIC; REPORTED; UNVERIFIED for local alternative settings on ab8ebbd | VISUAL_PROOF_BLOCKED_BY_POLICY | Baseline capture produced no RGB frame because external OpenCV/ROS native modules were blocked. |
| P01 | P0 — missing capability | STATIC; REPRODUCED; REPORTED on ab8ebbd | DEFERRED_MISSING_CAPABILITY | Current detector uses color connected components and unknown_product; recognition needs a separately scoped feature. |
| P02 | P0 — confirmed tracker bug | STATIC; REPRODUCED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base defect reproduced; tracking candidate has not yet passed independent review. |
| P03 | P0 for hero detection; P1 elsewhere | STATIC; REPRODUCED; REPORTED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base has fixed pixel filters and hard detection truncation; literal top-of-image cap not confirmed. |
| P04 | P1 — identity correctness | STATIC; REPORTED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base uses greedy pairwise assignment and lacks explicit occluded/archived states. |
| P05 | P0 — confirmed identity failure | STATIC; REPRODUCED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base consolidates by distance without co-visibility/identity constraints. |
| P06 | P1 — reachable identity collision | STATIC; REPRODUCED constructed case on ab8ebbd | CANDIDATE_IN_PROGRESS | Base can mix raw and canonical numeric IDs in annotation output. |
| P07 | P1 — confirmed algorithm defect | STATIC; REPRODUCED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base cluster-center updates do not refresh spatial-grid membership. |
| P08 | P0 for claims; P1 for estimation support | STATIC; REPORTED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base recounts RGB detections as support; tracking candidate pending review. |
| P09 | P0 — frame correctness | STATIC; conditional production magnitude on ab8ebbd | ALREADY_FIXED_ON_BASE | RGB overlays declare start-relative coordinates; inventory PLY uses map_x/y/z; retain the regression after P13 resubmission. |
| P10 | P0 — localization/geometry | STATIC; REPORTED on ab8ebbd | CANDIDATE_IN_PROGRESS | Base omits scan-to-frame motion transform and treats selected surface points as center estimate. |
| P11 | P1 — portability/data correctness | STATIC on ab8ebbd | CANDIDATE_IN_PROGRESS | Base PointCloud2 reader assumes packed little-endian float32 and ignores row_step/layout validation. |
| P12 | P0 — missing core capability | STATIC; REPORTED; VISUAL on ab8ebbd | DEFERRED_MISSING_CAPABILITY | No product recognition or supported 3D extent output exists at d5e825. |
| P13 | P1 — evaluation correctness | STATIC; REPORTED on ab8ebbd | CANDIDATE_REJECTED_REVISION_REQUIRED | Sol review found three blockers; evaluation programmer is revising matching, valid-pose alignment and eligibility provenance. |
| R01 | P0 — missing temporal capability | STATIC; VISUAL; REPORTED on ab8ebbd | OPEN_MISSING_TEMPORAL_STATE | SlamObserver overwrites latest_map and exports only its final cloud; no map revisions/history. |
| R02 | P0 for panel 06 | STATIC; VISUAL on ab8ebbd | DEFERRED_MISSING_RAY_RENDERER | No ray-geometry presentation exists in this tree; raw endpoints/calibration are available upstream. |
| R03 | P0 for persistent-map claims | STATIC; REPORTED; UNVERIFIED for measured residue on ab8ebbd | DEFERRED_DYNAMIC_MAP_CAPABILITY | No dynamic mask or correction-history output; later moving-actor residue is not reproducible here. |
| R04 | P0 — visual/storytelling blocker | STATIC; VISUAL on ab8ebbd | DEFERRED_MISSING_TECHNICAL_CAMERA | No technical camera configuration/renderer exists in this tree. |
| R05 | P1 — technical visualization quality | STATIC; VISUAL on ab8ebbd | DEFERRED_MISSING_POINT_RENDERER | No point-splat/cyan-map renderer exists in this tree. |
| R06 | P1 — spatial correctness | STATIC on ab8ebbd | DEFERRED_MISSING_SPATIAL_OVERLAY | No film-time path/marker renderer or shared occlusion model exists in this tree. |
| R07 | P0 for panel 10 | STATIC; VISUAL; REPORTED on ab8ebbd | DEFERRED_MISSING_DETAIL_VIEW | Current outputs show IDs/centers only; no product identity, extents, or detail-card interface. |
| R08 | P0 — storyboard direction | STATIC; VISUAL on ab8ebbd | OPEN_STORYBOARD_DIVERSITY | Base provides one continuous forward trajectory; storyboard camera directions are not implemented. |
| R09 | P1 — timeline semantics/continuity | STATIC; REPORTED on ab8ebbd | DEFERRED_MISSING_PRESENTATION_TIMELINE | No film-to-source timeline/transition state exists in this tree. |
| R10 | P1 — delivery/rework efficiency | STATIC; REPORTED on ab8ebbd | OPEN_NONLOSSLESS_NONRESUMABLE | Current recorder writes directly to MP4 and overlay re-encodes; no lossless frame master or shot-level resume. |
| C01 | P1 — data-boundary inconsistency | STATIC call-path confirmation on ab8ebbd | ALREADY_FIXED_SOURCE; RUNTIME_UNVERIFIED | Current validator/launcher omit truth-file requirements; actual withheld-file entrypoint runtime was not run. |
| C02 | P1 — stale-result risk | STATIC on ab8ebbd | CANDIDATE_IN_PROGRESS | Base geometry hash omits USD-authored geometry; runtime candidate pending independent review. |
| C03 | P1 — robustness/resource budget | STATIC; runtime reproduction still required on ab8ebbd | QUEUE_HANG_NOT_REPRODUCED; CLEANUP_FIX_IN_PROGRESS | Current writers are synchronous; adjacent exception-path close/metadata gap is being repaired. |
| C04 | P1 — conditional capture/map correctness | STATIC; UNVERIFIED installed behavior on ab8ebbd | CANDIDATE_IN_PROGRESS; SPAN_UNVERIFIED | Fixed sleeps and missing mapper processed-span acknowledgment remain; installed replay behavior unverified. |
| C05 | P2 — performance and iteration cost | STATIC; REPRODUCED for eager evaluation on ab8ebbd | NOT_REPRODUCED_ON_BASE | Current output path has one PyAV decode context and no eager ffprobe/probe_video call. |
| C06 | P1 — delivery/review blocker | STATIC; REPORTED on ab8ebbd | NOT_APPLICABLE_AS_SOURCE_DEFECT; DELIVERY_OPEN | No published film artifact exists at d5e825; final evidence access remains a deliverable check. |
| T01 | P0 — acceptance coverage | STATIC; REPRODUCED on ab8ebbd | OPEN_COVERAGE_GAP; TESTS_IN_PROGRESS | Base perception tests lack stationary, neighboring-instance, occlusion/revisit and real-shelf cases. |
| T02 | P1 — test design | STATIC; REPRODUCED algebra on ab8ebbd | OPEN_EVALUATED_GEOMETRY_GAP; TESTS_IN_PROGRESS | Base tests derive support from authored dimensions; they do not validate USD evaluated bounds/contact. |
| T03 | P0 — acceptance coverage | STATIC on ab8ebbd | DEFERRED_REPRESENTATION_COVERAGE | No technical-view/presentation modules or corresponding tests exist at d5e825. |
| T04 | P1 — agent/review design | STATIC; REPORTED; INFERENCE on ab8ebbd | HISTORICAL_PROCESS_ONLY | Base contains no tracked routing config/verdict; current independent review is separately assigned. |
| T05 | P1 — specification/process correction | Source-based retrospective; INFERENCE on ab8ebbd | HISTORICAL_PROCESS_ONLY | Later prompt/spec/role findings are absent at d5e825; apply bounded-proof policy operationally. |

## Resume priorities

1. Reproduce and repair confirmed tracker and data-flow defects against the current production functions.
2. Repair bounded runtime, dependency, and asset/scene defects with focused regressions.
3. Keep missing product-instance recognition, supported 3D extents, persistent map history, and renderer capabilities explicit as deferred design/features unless existing evidence establishes an implementable boundary.
4. Update each finding with exact files/commit, tests or real visual evidence, remaining dependency, and independent-review state. Do not claim a candidate accepted before Sol reviews that exact snapshot.

## Current-base applicability audit (read-only)

Audited immutable repair base `d5e825c8f6dab77aa6a1007c9731c226b588dfcf` (tree `a9d85319fb018a94de695d44d085633f19cdc008`) against the forensic package (`7588fbe0777cbca48a32b6d42294c7f95d4ba51d108f397f04aee2a35ffffa8d`). Review-package instructions describe the earlier audit; the owner’s implementation request controls this pass. These are applicability results, not code acceptance.

- **A01–A05:** no bundle/readiness gate, technical renderer, later verdict/spec files, or 4K config exist at the repair base. A01/A03 are missing capabilities; A02/A04 are historical/process findings; A05’s 4K mechanism is absent while image-quality proof is open.
- **P01–P13:** P01/P12 are missing recognition/extent capabilities. P02–P08 and P10–P11 reproduce in current tracker dataflow and are in a repair candidate. P09 is already separated in the current outputs: start-relative values label the RGB overlay; map coordinates alone populate the inventory PLY. P13’s first candidate was rejected by independent review and is being revised.
- **R01–R10:** R01 loses temporal state by overwriting the prior cloud. R08 has one forward trajectory and no storyboard camera diversity. R10 records directly to lossy MP4 without a lossless master/resume graph. R02–R07 and R09 cite renderer/timeline interfaces absent from this base and remain deferred capabilities or proof gaps.
- **C01–C06:** C01’s current sensor-only validation source omits truth files, but a withheld-file run was not performed. C02 is reproduced because geometry hashing omits authored USD contents. C03’s queue/sentinel hang is stale-snapshot evidence; current recorders are synchronous, though failure cleanup is under repair. C04’s fixed waits and absent mapper-span acknowledgment remain statically reproduced; installed behavior is unverified. C05 is not reproduced; C06 is not a source defect in a base with no published film.
- **T01–T05:** T01 lacks adversarial estimator cases; T02 checks authored numbers rather than evaluated USD geometry; T03 has no representation/presentation tests. T04/T05 are historical/process findings from files absent at the repair base.

Independent applicability sources: exact finding text in the owner-supplied `findings_index.json`; source/line audit from the dedicated Sol reviewer for A/R/T; source/line audit from the read-only explorer for P01–P12 and C01–C06. The current candidate statuses above still require candidate-specific review.

## Visual evidence attempt

The baseline attempt and its compact logs are saved under `review/evidence/forensic-baseline-20260922/`. Isaac completed 1,230 frames, but the RGB and rosbag subprocesses failed when Windows Application Control blocked native OpenCV and ROS dependencies. The status records zero observed RGB frames and no LiDAR clouds. No policy change, bypass, or package reinstall was attempted. This is failure evidence only; no before/after camera comparison has been produced yet.

## Candidate review: scene/asset commit 9e093be

Independent Sol review returned **REQUEST_CHANGES** for exact candidate `9e093be44e2c89b2312cc691befaab02056edec5` (tree `7f8997aa583083a0a07ae0be98dbb20f00c7d3ef`, base `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`). The requested 11 CPU tests passed and the generator reproduced 34/34 committed USDA files byte-for-byte; neither result clears the following blockers:

1. `simulator/environment/aisle_builder.py` packing produced 355 fruit intersections across four bins (review probe: 144 lower layer, 125 upper layer, 86 between layers). The submitted bounds/support test did not impose a minimum fruit-pair separation.
2. `tools/retail_assets/generate_packaging.py` still writes `UsdUVTexture.outputs:rgb` with token type; authors normals interpolation as a separate token-array attribute instead of metadata on the normals attribute; and sets unsupported `UsdGeomCone.radius2`. The winding regression compares winding to the authored normal without asserting front-facing +Y or UV basis.

The scene programmer is revising the candidate with separated packed fruit, valid frustum geometry, schema-correct USD properties and stronger regressions. The rejected candidate is not integrated. V01–V07/V09 remain open; V08’s assembly defect was not reproduced in code but absence is not yet visually confirmed. No production camera image was inspected.

## Candidate review: evaluation commit ac21aece

Independent Sol review returned **REQUEST_CHANGES** for exact candidate `ac21aecec2b6eb6244f61ef276b79e4bcb066556` (tree `3ec668fa70b04de5766c47bfe88022f3a4e78cfd`, base `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`). Nine focused tests passed; review reproduced three missing adversarial checks:

1. Greedy distance matching can report one association when a valid two-pair maximum-cardinality assignment exists (truth at 0.00/0.34 m; estimates at 0.16/-0.17 m; 0.35 m gate).
2. Evaluation selects the earliest numeric SLAM time even if that row has an invalid zero quaternion, while the estimator discards that pose.
3. Optional eligible-recall input records no frozen rule/source identity/hash/ID set, so the score is not reproducible.

The evaluation programmer is revising all three and will resubmit for fresh review. No code acceptance has been recorded for that candidate.
