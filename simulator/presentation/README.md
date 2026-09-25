# Presentation timeline

`config/presentation/storyboard.yaml` is the executable 12-shot contract. It
fixes the half-open frame intervals, the native 1920x1080 delivery profile, and
the 1280x720 preview profile at 30 fps and 1,350 frames.

Complete rendering accepts a schema-v3 shot bundle only when its RGB capture
and seven technical-view receipts validate against reviewed source catalogs. The
machine-readable catalog in `config/presentation/input_schemas.json` names the
exact schemas, repository producer IDs/source files, and artifact types for
each role. Every artifact uses a `{path, sha256}` descriptor. The validator
parses the repaired capture-v1 six-topic contract, observed RGB calibration,
technical delivery, optimized `slam_map_poses.csv`, map, trajectory,
and object-state bindings; requires one capture identity; and checks each
derivation receipt against its video, repository producer, producer revision,
source catalog, and source hashes.

After a production capture completes, emit the RGB role and receipt from the
capture producer's own manifest rather than writing presentation evidence by
hand:

```powershell
C:\isaacsim\python.bat `
  -m simulator.presentation.rgb_bundle `
  --capture-manifest runs\<capture-id>\capture\capture_manifest.json `
  --output-manifest runs\<capture-id>\outputs\presentation_rgb_inputs.json
```

The emitter checks the recorder video, at least 540 contiguous timestamp rows, recorder
metadata, the observed `camera_info.json`, its capture inventory binding, and
`sensor_transforms.json`, and
the capture manifest's per-file hashes before writing anything. It recomputes
the repository input bindings, resolves the capture's full Git commit, and
verifies the capture-producer blob at that revision. It then
requires exact membership in
`config/presentation/accepted_rgb_captures.json`, including capture hash,
producer revision/blob, and required artifact hashes, before writing a
receipt. The repaired capture manifest v1 contains exactly six raw topics,
including `/sim/camera/rgb/camera_info`. RGB shots
consume source frames 0-539, so a genuine 20.5-second capture is sufficient.
The receipt records its actual first/last simulation stamps and copies the
exact `presentation_transform` from the accepted catalog entry. Only the
reviewed `none` and legacy/test `hflip` operations are supported, with exact
reasons and an explicit statement that raw capture bytes remain unchanged.
The retained historical capture `20260921-053814804` uses `none`, but its
producer blob predates the repaired source base. It is kept as provenance and
is rejected as a current repaired-base delivery. A new capture requires a new
reviewed allowlist entry. The generated presentation fixture retains `hflip`
for legacy test coverage.

The historical catalog contains that one previously accepted capture. Its review record
names a checked-in, repository-relative verdict whose bytes must match the
cataloged SHA-256. The record is evidence of its old candidate, not authority
for the repaired source base or the new owner goal. Missing evidence, path escapes, hash mismatches, and captures
without an exact catalog entry are rejected before a receipt is written.

This mechanism is a checked-in reviewed allowlist with deterministic integrity
checks. It is not cryptographic attestation that the producer executed. Tests
may inject an isolated temporary catalog for generated fixtures; normal
inspection and rendering always use the checked-in catalog.

Known storyboard hashes from `references/manifest.json` are rejected regardless
of an artifact's current filename or directory. Reusing one view across roles,
mixing captures or SLAM versions, missing hashes, arbitrary placeholder text,
and incomplete view associations all keep the corresponding role unavailable.
Technical delivery validation consumes the repository renderer's seven distinct
1080p outputs and receipts in fixed order: 120/90/90/120/120/120/150 frames.
Each technical shot starts at local frame zero. The shared technical source
receipt binds capture, map, trajectory, object-state versions, simulation-time
extent, producer commits, and the reviewed source catalog. Hand-written or
cross-source replacements fail before rendering.

Combine accepted producer outputs into schema-v3 shot inputs:

```powershell
C:\isaacsim\python.bat -m simulator.presentation.complete_bundle `
  --rgb-bundle runs\<capture-id>\outputs\presentation_rgb_inputs.json `
  --technical-delivery runs\<capture-id>\technical\technical_views_delivery_manifest.json `
  --output-manifest runs\<capture-id>\outputs\presentation_complete_inputs.json `
  --ffprobe C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe
```

The production CLI uses only checked-in catalogs. Tests may inject temporary
reviewed catalogs for generated fixtures; those outputs must stay labelled as
test evidence. Fixture classification comes from an exact marker ID and SHA-256
inside both reviewed source catalogs, and those catalog hashes are bound into
the producer receipts and complete bundle. Display labels never control output
status, claims, or the visible fixture watermark.

The checked-in `diagnostic_baseline_inputs.json` intentionally supplies only
`demo/walking_aisle_final_hifi.mp4`. Diagnostic mode accepts that exact
repository path, SHA-256, Git blob, canonical manifest, and the frozen
pre-transition `diagnostic_storyboard_legacy.yaml` plan. The production
`storyboard.yaml` carries the current transition policy and is rejected for
this historical diagnostic identity.
The baseline was introduced at commit `d5e825c8` before the storyboard
manifest at `d315aa9c`; alternate manifests and videos are rejected even when
they claim the same diagnostic producer. Its provenance is
`diagnostic_baseline`, so it can never satisfy the genuine RGB contract or a
complete render. Diagnostic mode uses that real simulation output for shots
01-05 and displays explicit missing-input slates for shots 06-12. It does not
use the cluttered legacy tracking overlay or synthesize sensor processing.

Run the inexpensive CPU preview with the Isaac Python environment. The Pixi
Python currently remains blocked from loading OpenCV by Windows Application
Control, while Isaac Python can load the installed renderer dependencies:

```powershell
C:\isaacsim\python.bat `
  -m simulator.presentation.render_video `
  --plan config/presentation/diagnostic_storyboard_legacy.yaml `
  --inputs config/presentation/diagnostic_baseline_inputs.json `
  --output-dir runs/presentation-preview `
  --profile preview --mode diagnostic `
  --ffmpeg C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe `
  --ffprobe C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe
```

The renderer verifies dimensions, CFR, frame count, duration, stream count, and
hashes with ffprobe and records source hashes plus per-shot
provenance in `diagnostic_preview_manifest.json`. The manifest makes the narrow
claim that diagnostic source pixels come from the pinned pre-storyboard
baseline blob. It does not make a universal no-storyboard-pixels claim from
exact file hashes. Complete mode fails before rendering until every required
genuine role is present. Native delivery rejects source views smaller than
1920x1080 and writes an FFV1 lossless local master, high-quality silent H.264
MP4, deterministic self-created ambience/footstep/cart WAV, audio-muxed final
MP4, 720p review MP4, contact sheet, and start/mid/end frames for every shot.
Every artifact and the completion manifest are first written and validated in
a unique sibling staging directory. The renderer publishes the package with a
directory-generation swap only after all checks pass; a failed encode, probe,
hash, contact sheet, or representative-frame step removes staging and leaves
the prior completed generation byte-for-byte unchanged.

Render retained technical views for a reviewed run, then assemble the complete
bundle:

```powershell
C:\isaacsim\python.bat -m simulator.technical_views `
  --run-root runs\<capture-id> `
  --profile delivery `
  --output-dir runs\<capture-id>\technical `
  --ffmpeg C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe `
  --ffprobe C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe
```

Renderer code, technical plan, and source-catalog receipt hashes canonicalize
text line endings to LF. Producer receipts therefore validate identically from
Windows CRLF and LF checkouts.

The production `storyboard.yaml` applies short symmetric smoothstep blends at
frames 540, 660, 750, 840, 960, 1080, and 1200. Each window freezes only the
edge frames needed to straddle its boundary, consumes no extra timeline frames,
and preserves the exact 1,350-frame delivery budget. RGB shots 01-05 remain
frame-contiguous from the original capture without added dissolves.
FFmpeg's custom-xfade progress `P` descends from one toward zero, so the
renderer weights the outgoing source with `smoothstep(P)` and the incoming
source with `1-smoothstep(P)`. The generated moving-geometry fixture checks
every adjacent output frame from one frame before each transition through the
first frame after it, including both entry and exit edges.

The delivery manifest classifies each transition as an editorial temporal
blend between independently rendered clips. It explicitly records that the
frames are neither co-timed measurements nor sensor, geometry, or map fusion.
Each mixed frame is shared between both shots; its nominal timeline shot is
retained only as an index. The manifest binds the exact outgoing and incoming
videos and receipts, the cloned edge-frame numbers, their independent time
bases and data extents, the clone ranges, and the smoothstep weights for every
mixed frame. Source clip PTS is recorded independently from sensor time. RGB
edge frames retain their validated frame-index-to-simulation-stamp binding.
Technical receipts currently provide only aggregate source-data extents, so
their held-frame measurement timestamps remain explicitly unavailable rather
than reusing an extent endpoint.

## Current new-goal limitations

The retained technical renderer remains operational but does not satisfy the
new selective, feature-specific LiDAR requirement. It projects the complete
finalized SLAM map, and `sensor_activation` draws explanatory rays to selected
final-map points rather than timestamped current-scan returns. Every technical
receipt and delivery manifest records this status as `unfinished`.

The transition regression proves the renderer's editorial continuity policy
with structured generated inputs. It does not certify the content, geometry,
motion quality, or selective-scan truth of production technical clips. Those
properties still require review of the P2 current-scan outputs and the final
assembled film.
