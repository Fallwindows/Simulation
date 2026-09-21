# Production perception source review — ACCEPT

## Exact scope

- Source code commit: `def059bbfb06449648f5a7411e36b333823d6c7c`
- Source tree: `4531cae426565a0299d0cb46a4ae82e24e843edf`
- Run: `runs/20260921-053814804`
- Perception directory: `runs/20260921-053814804/perception`
- Perception manifest SHA-256: `0fef77178e555286b2570695df2674e2b833a15592c05f735834b3571c294aab`
- Disposition: **ACCEPT** as the production estimated-object source for technical views, subject to the representation limits below. There is no blocking provenance, frame-contract, or estimator/evaluation separation defect in the reviewed artifacts.

## Binding and frame-contract evidence

The accepted provenance validator and frame-coverage validator both passed read-only against the production bytes, including the 15,822,295,040-byte raw ROS bag. The manifest binds one capture and one SLAM result:

- capture id `20260921-053814804`
- canonical capture hash `ab84ccc1f71790f896bab1375062ed1db9f40bf9a1531d582f7cd0e589121eba`
- capture manifest file hash `3d82955332e326376b1f3e683a34ff589e4f0324cf17015682d336f282db3599`
- SLAM manifest hash `bb79ed9155f40d27621512f6b3309aabe80aed1ab88e3783478aaf518c02ed3e`
- estimated trajectory hash `46e144e0519b7320096ba2735cca1395068c116124e4ca58f5f750cd58205160`
- RGB video/index/metadata/transforms hashes `c453b46e40acd06928b384fd13a5a7822c16f46ca5f30b79fc829d1d8e05c703`, `539977f7498135b5cfd06d36de7841aff932dc1e66511303f764b4e42306d0ea`, `e4cd5f3ec89a8821653dc52fb114407e305cf116f8c6f1ede364f483a4b57725`, and `b925f05007126b4a755ba6659ba2b8756247d1069feb67276fbe265baa2b430f`
- raw bag metadata and DB hashes `96df34d053e0ad541fb08179af7d28814fe6fe2f76eac9994464b7ada4c80d69` and `722f92986b302bdf86399a3416f2d5a9582a7c6d119c330d6a93e10eaa7e565d`

All 613 annotation rows are contiguous at indices 0–612. Their timestamps and dimensions exactly match the production RGB frame index. The manifest reports 94,801 detections, 454 canonical tracks, 46,675 raw RGB tracks, 22 LiDAR scans, and 142,294 projected LiDAR points. The actual JSONL totals agree: every frame has detections, with 86/160/160 minimum/median/maximum detections per frame. Focused provenance, perception, and technical-view tests passed 22/22.

## Estimator outputs and rejected archive comparison

Current production estimator artifacts:

- `estimated_inventory.csv`: `3247c4d5dbcab08f166c8da495243aca8723725bc89b457bfedc6e0948bb8cbe` (67,720 bytes)
- `estimated_inventory.json`: `3ee423af43c36d969c2b6a16ebb550a68cefab4f7fea2ce469554b6357ba5185` (240,485 bytes)
- `frame_annotations.jsonl`: `daf86228830661854a342dde894cbbad9bf34c4141aacede16fc8298ea43c1c8` (22,671,888 bytes)
- `tracks.csv`: `3247c4d5dbcab08f166c8da495243aca8723725bc89b457bfedc6e0948bb8cbe`

These four estimator outputs are byte-for-byte identical to the archived `failed-readiness-evidence/perception-unbound-rejected-20260921` bundle. The archived manifest hash was `320009cef0826ff636b18b3939c9ad1da0567d1a8f7a8d490634e2adb5f3ff13`; the corrected manifest adds exact capture, RGB, raw-bag, SLAM, trajectory, artifact, and frame-contract provenance. Thus the correction establishes trust in the existing estimates; it does not claim an accuracy improvement.

CSV and JSON inventory representations agree semantically. All 454 IDs are unique and all coordinates are finite. All depth sources are `lidar_projected_with_slam_pose`; no ground-truth, SKU, or semantic-category fields occur in the estimator inventory or annotations.

## Ground-truth isolation and evaluation separation

The estimator manifest declares `ground_truth_consumed=false` and `ground_truth_required=false`. Filesystem timestamps place the estimator inventory and annotations before the corrected manifest, and the corrected manifest before the later evaluation artifacts. The only ground-truth indicators in the core estimator outputs are those explicit false flags.

Evaluation is a separate later stage and truthfully declares ground-truth consumption. Its hash is `7117fdb9b4fe3c17624c6c2114b875c85555d7e5ec1075b444082b7359609bb`. It reports 454 estimates versus 1,995 ground-truth objects, 238 one-to-one matches within 0.35 m, precision `0.524229`, median matched error `0.2133 m`, p95 matched error `0.3389 m`, and zero correct SKU/category classifications. Its `0.119298` visible-recall field is based on proximity matching and must not be described as an independently visibility-conditioned recall measurement. Evaluation-only outputs, including `inventory.csv`, `inventory.xlsx`, `inventory_evaluation.json`, and `slam_map_with_inventory.ply`, are not estimator inputs and should not be used as the technical-view source.

## Technical-view suitability and limits

The accepted technical-source inspector passed with an exact scratch catalog binding the capture, SLAM, perception manifest, map, trajectory, and inventory hashes. It accepted the source interval 0.2–18.6 s and the sole depth source `lidar_projected_with_slam_pose`. A checked-in production catalog entry remains required before producing final technical-view receipts.

The source is suitable for an honest estimated-object diagnostic: reconstructed map context, estimated trajectory, projected detections, and persistent unknown-product locations. It is not evidence of SKU identity, product class, object extents, shelf occupancy completeness, or full-run coverage after 18.6 s.

Material quality limits:

- all 454 objects are labeled `unknown_product`
- only 2,056 of 94,801 detection records contain localized centers; the other 92,745 are RGB-only with no LiDAR association
- 40 tracks have one observation; observation counts range 1–19 with median 3
- a non-gating diagnostic envelope (`x 0–25.5`, `y -3–3`, `z 0–2.5`) contains 315 estimates and excludes 139; 130 estimates exceed `z=2.5`, 11 exceed `|y|=3`, and one has negative z
- estimated coordinate ranges are x `2.307–23.782`, y `-4.223–3.639`, z `-0.073–3.908`

These are rendering and narration constraints, not provenance failures. Technical views should depict points/uncertain object locations rather than inferred cuboids or named products, and receipts should disclose the 0.2–18.6 s source interval.

## Independent evidence

- `production-perception-binding-def059b.json` SHA-256 `9f7dcc3c44e9023b05353d42a2f7c774fedc3eb5f9d1874267ada656828c8308`
- `production-perception-stats-def059b.json` SHA-256 `fe4b083ef00a883a15475682141033089677dd30eb5b56d287f467bd9416dfa4`
- scratch catalog SHA-256 `cae02d14dd9addd326784f02001f47ff9f938533d3853f16ca9339917e6beaf0`
- technical-source inspector result SHA-256 `691bd4c22ac5134c2613229f8e5bf76290a0e23c5ac1469c6a544da31c72d552`

The integration checkout remained at the exact accepted commit/tree and had no tracked or untracked status changes from this review.
