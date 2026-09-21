# Presentation timeline

`config/presentation/storyboard.yaml` is the executable 12-shot contract. It
fixes the half-open frame intervals, the native 1920x1080 delivery profile, and
the 1280x720 preview profile at 30 fps and 1,350 frames.

Complete rendering accepts a schema-v3 shot bundle only when its RGB capture
and seven technical-view receipts validate against reviewed source catalogs. The
machine-readable catalog in `config/presentation/input_schemas.json` names the
exact schemas, repository producer IDs/source files, and artifact types for
each role. Every artifact uses a `{path, sha256}` descriptor. The validator
parses capture-v2, RGB index/calibration, technical delivery, map, trajectory,
and object-state bindings; requires one capture identity; and checks each
derivation receipt against its video, repository producer, producer revision,
source catalog, and source hashes.

After a production capture completes, emit the RGB role and receipt from the
capture producer's own manifest rather than writing presentation evidence by
hand:

```powershell
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe `
  -m simulator.presentation.rgb_bundle `
  --capture-manifest runs\<capture-id>\capture\capture_manifest.json `
  --output-manifest runs\<capture-id>\outputs\presentation_rgb_inputs.json
```

The emitter checks the recorder video, at least 540 contiguous timestamp rows, recorder
metadata, configured camera intrinsics against `sensor_transforms.json`, and
the capture manifest's per-file hashes before writing anything. It recomputes
the repository's canonical `capture_hash`, resolves the capture's full Git
commit, and verifies the capture-producer blob at that revision. It then
requires exact membership in
`config/presentation/accepted_rgb_captures.json`, including capture hash,
producer revision/blob, and required artifact hashes, before writing a
receipt. Capture manifest v2 contains exactly five raw topics; camera_info is a
configured calibration artifact and is never claimed as a bag topic. RGB shots
consume source frames 0-539, so a genuine 20.5-second capture is sufficient.
The receipt records its actual first/last simulation stamps and declares the
reviewed `hflip` presentation transform required to correct mirrored native RGB
text without modifying raw capture bytes. The production catalog is
intentionally empty until a real capture passes independent review, so the command currently refuses every
capture.

This mechanism is a checked-in reviewed allowlist with deterministic integrity
checks. It is not cryptographic attestation that the producer executed. Tests
may inject an isolated temporary catalog for generated fixtures; normal
inspection and rendering always use the checked-in production catalog.

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
python -m simulator.presentation.complete_bundle `
  --rgb-bundle runs\<capture-id>\outputs\presentation_rgb_inputs.json `
  --technical-delivery runs\<capture-id>\technical\technical_views_delivery_manifest.json `
  --output-manifest runs\<capture-id>\outputs\presentation_complete_inputs.json `
  --ffprobe C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe
```

The production CLI uses only checked-in catalogs. Tests may inject temporary
reviewed catalogs for generated fixtures; those outputs must stay labelled as
test evidence.

The checked-in `diagnostic_baseline_inputs.json` intentionally supplies only
`demo/walking_aisle_final_hifi.mp4`. Diagnostic mode accepts that exact
repository path, SHA-256, Git blob, canonical manifest, and canonical plan.
The baseline was introduced at commit `d5e825c8` before the storyboard
manifest at `d315aa9c`; alternate manifests and videos are rejected even when
they claim the same diagnostic producer. Its provenance is
`diagnostic_baseline`, so it can never satisfy the genuine RGB contract or a
complete render. Diagnostic mode uses that real simulation output for shots
01-05 and displays explicit missing-input slates for shots 06-12. It does not
use the cluttered legacy tracking overlay or synthesize sensor processing.

Run the inexpensive CPU preview with the Python and FFmpeg paths available in
the ROS Pixi environment:

```powershell
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe `
  -m simulator.presentation.render_video `
  --plan config/presentation/storyboard.yaml `
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
