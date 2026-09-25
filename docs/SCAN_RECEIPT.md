# LiDAR scan projection receipt

`scripts/extract_scan_receipt.py` creates a bounded JSON receipt that traces a
selected RGB observation back to raw `PointCloud2` return indices. It uses only
CPU NumPy and Python's read-only SQLite interface. ROS Python deserialization is
not required.

The extractor validates a complete capture/perception manifest chain, hashes
the exact bag, calibration, transform, frame index, annotation, track, and
legacy SLAM inputs, and requires the caller to supply the aggregate binding
SHA-256. It fails when an input changes during the run, when timestamps or frame
IDs disagree, or when the expected observation cannot be identified uniquely.

Projection uses the written notation `T_A_from_B`, resolves
`T_camera_optical_frame_from_lidar_link` from the recorded rig transforms, and
uses the recorded configured pinhole intrinsics. Raw point indices survive the
finite, forward-depth, in-image, bbox, nearest-surface, and bounded-decimation
stages. Every stage has an index count and SHA-256 in the receipt; selected
points include raw XYZ, optical XYZ, pixel coordinates, depth, and raw index.

First inventory an input set and have its digest checked against the intended
run:

```powershell
& C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe `
  scripts\extract_scan_receipt.py `
  --run-dir <run-directory> `
  --inspect-binding
```

Then extract with that approved digest and an output below the run directory:

```powershell
& C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe `
  scripts\extract_scan_receipt.py `
  --run-dir <run-directory> `
  --output <run-directory>\diagnostics\scan-selection\frame-000003-track-000429.json `
  --expected-binding-sha256 <approved-sha256>
```

The historical `20260921-053814804` run predates the repaired canonical
`slam_map_poses.csv`. Receipts from it are therefore labeled
`diagnostic_legacy_source`. Its legacy pose is verified for provenance but is
not used for the co-timed rigid LiDAR-to-camera projection. Such a receipt is
not production acceptance and does not establish class, full extent, or true
object geometry.
