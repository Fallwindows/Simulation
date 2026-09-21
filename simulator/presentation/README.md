# Presentation timeline

`config/presentation/storyboard.yaml` is the executable 12-shot contract. It
fixes the half-open frame intervals, the native 1920x1080 delivery profile, and
the 1280x720 preview profile at 30 fps and 1,350 frames.

Complete rendering accepts timeline-aligned view videos only when their input
manifest also supplies the genuine data products behind the view. The
machine-readable catalog in `config/presentation/input_schemas.json` names the
exact schemas, repository producer IDs/source files, and artifact types for
each role. Every artifact uses a `{path, sha256}` descriptor. The validator
parses capture, RGB index/calibration, ROS bag, pose, map, and perception
records; requires one capture plus matching map/object versions; and checks a
view-derivation receipt that binds the view-video hash to every source hash and
the producer source hash.

Known storyboard hashes from `references/manifest.json` are rejected regardless
of an artifact's current filename or directory. Reusing one view across roles,
mixing captures or SLAM versions, missing hashes, arbitrary placeholder text,
and incomplete view associations all keep the corresponding role unavailable.
The technical LiDAR/map/reconstruction view producer flags remain false until
real data-to-pixels implementations exist, so complete mode currently fails
closed even if someone writes plausible-looking manifests by hand.

The checked-in `diagnostic_baseline_inputs.json` intentionally supplies only
`demo/walking_aisle_final_hifi.mp4`. Its provenance is
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

The renderer writes the MP4 atomically, verifies its dimensions, frame rate,
frame count, and duration with ffprobe, and records source hashes plus per-shot
provenance in `diagnostic_preview_manifest.json`. Its
`storyboard_pixels_consumed=false` result is emitted only after artifact hashes
have been checked against the canonical storyboard manifest. Complete mode
fails before rendering until every required genuine role is present. Native
delivery also rejects source views smaller than 1920x1080 to prevent an upscale
from being reported as native rendering.
