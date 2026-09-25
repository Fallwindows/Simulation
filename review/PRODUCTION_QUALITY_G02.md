# Production-quality grocery demo: working checkpoint

Status on 2026-09-25: **in progress**. This is a source and visual evidence checkpoint, not final goal acceptance. The owner goal requires at least 30 genuinely distinct new store items, materially more realistic RGB in actual video camera shots, selective and spatially correct LiDAR views, smooth motion throughout, preserved technical clarity, and rendered visual inspection.

## Current approved source

The isolated integration branch `codex/production-quality-upgrade` is at `6a7a355b54bfdac7213c9254fd6d94f9011cd50a` (tree `db72e9e8760649524b13dccb24ddbc7ffa7be07c`). It combines independently approved motion M1, presentation port P1, historical scan diagnostic L1, retail assets R5, and film transitions P3. Its focused combined suite passed **98 tests with two documented dependency skips**. The user's main checkout is untouched.

| Goal component | Evidence | State |
|---|---|---|
| Item variety | 45 new distinct assembly profiles, 79 catalog keys total, 2,017 placed semantic instances; R5 exact source/texture and combined technical reviews approved | Source accepted; final camera appearance still under review |
| Smooth source motion | Deterministic eased 20.5 m path and stable pose interpolation; M1 exact review approved | Source accepted; final sequence review pending |
| Film transitions | Seven reviewed smoothstep windows in a 1,350-frame, 30 fps, 45 s plan; editorial blend provenance is explicit | Source accepted; final production film pending |
| RGB scene | Actual combined 1280×720 frames at approximately 3, 9, and 17 s from `e0ceb30d840ca98c30756570fa0caa83223dbcea` | **Visual review requested changes**; scene R2 underway |
| Selective LiDAR | Historical paired-scan diagnostic and selective render prototypes; P2 R3 validator candidate `6e49df678509d475b6c96b8ce9b5bad1b7ae0836` | Exact R3 review and fresh current-scene paired capture pending |

## RGB visual finding

The three actual combined camera frames show more products and a bounded, lit store, but remain stylized. Shelves and lower products are underlit; structural colors, shelving, ceiling, and floor repeat uniformly; the 17-second end wall is a placeholder colored grid. Independent exact scene review **requested changes** despite its focused source/runtime checks passing (46 tests). The scene programmer is revising shelf-facing light, fixture materials and density, and the end wall, with fresh rendered review required. CaptureOnly stills are visual evidence and do not establish ROS camera or LiDAR capture completeness.

## Sensor and film truth gates

The earlier 15.8 GB RGB/LiDAR bag is paired but predates both the G02 scene and eased motion. It remains historical diagnostic evidence and is delivery-ineligible. Final selective technical views must derive from a new capture whose RGB, PointCloud2, CameraInfo, sensor transforms, scene manifest, and estimated trajectory belong to the same current run. Final film review must inspect the 12-shot 1,350-frame output, transition boundaries, technical overlays, and motion across the relevant sequence.

The complete chronology, exact candidate identities, reviewed findings, and unresolved work are in the [implementation journal](../IMPLEMENTATION_JOURNAL.md).
