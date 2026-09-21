# Implementation Journal

## Phase 0 — live rediscovery

**Date:** 2026-09-12
**Status:** Isaac/ROS runtime and native RTAB-Map build verified; implementation is ready for the consolidated runtime test session.

Observed live state:

- Windows 11 Home v25H2, build 26200 (confirmed by Isaac's checker and runtime).
- NVIDIA GeForce RTX 4070 Ti, driver `595.71`, 11.99 GB VRAM; an AMD integrated adapter is also present.
- C: free space before installation: 841.6 GiB; Isaac archive MD5 matched NVIDIA's published `a07968e980072c9ca27b2166443e2d89`.
- Repository is `Fallwindows/Simulation`, branch `master`, remote `origin` points to the expected GitHub SSH URL.
- Port 8080 is free.
- Isaac Sim 6.1.0 installed at `C:\isaacsim`; compatibility checker result: **PASSED**.
- Pixi 0.80.0 and the current native Windows ROS 2 Jazzy workspace installed at `C:\IsaacSim-ros_workspaces\jazzy_ws`.
- `rclpy`, `sensor_msgs_py`, FastAPI, Uvicorn, and OpenCV are available in the Pixi environment.
- `rtabmap_ros` and `rtabmap` source are checked out in the external Pixi workspace and built with Visual Studio Build Tools 2022, MSVC v143, CMake/Ninja, PCL, and the Windows 11 SDK.

Security cleanup:

- Removed the previous handoff document's private-key path, fingerprint, and credential details.
- Repository documents now refer to the existing local Git configuration without exposing credentials.
- Runtime logs, bags, maps, caches, and secrets remain ignored.

## Frozen contracts

- Frames: truth visualization `sim_world -> truth_sensor_rig`; estimator tree `map -> odom -> sensor_rig -> camera_link -> camera_optical_frame` and `sensor_rig -> lidar_link`.
- SLAM owns `map -> odom`; the simulator never publishes that relationship.
- Ground truth is `geometry_msgs/msg/PoseStamped` on `/sim/ground_truth/pose` in `sim_world`.
- RGB: `/sim/camera/rgb/image_raw` and `/sim/camera/rgb/camera_info`.
- LiDAR: `/sim/lidar/points`.
- Clock: `/clock`.
- Estimated odometry: `/slam/odom`; map cloud: `/slam/map_cloud`.
- Dashboard: `127.0.0.1:8080`.
- Internal units: SI; aisle axes are x-forward, y-width, z-up.

## Implemented milestones

- Contracts and JSON-compatible YAML configuration loader with typed dataclasses and early validation.
- Deterministic procedural aisle geometry: floor, two shelf rows, bays, levels, uprights, and seeded product proxies.
- Straight and walking trajectories with deterministic bob, sway, yaw, pitch, and speed variation.
- Quaternion and rigid-transform helpers, including camera optical-frame conversion.
- Optional Isaac USD geometry adapter with delayed imports plus the real Isaac 6.1 runner in `simulator/runtime/isaac_sim_runner.py`.
- Dashboard state/cache, FastAPI endpoints, MJPEG route, LiDAR/map WebSocket routes, and minimal browser page.
- ROS 2 topic contract and RTAB-Map LiDAR launch boundary with simulator truth kept separate from odometry.
- ATE/RPE/distance metrics and run artifact serialization.
- Windows-oriented scripts for native Isaac runtime, Zenoh router, dashboard, mapping, and baseline orchestration.
- Walking USD runtime applies the complete sampled trajectory orientation; `run_sim.ps1` supports GUI by default and explicit `-Headless` mode; Pixi/workspace paths are resolved from PATH/environment or explicit parameters.
- Browser LiDAR and accumulated-map viewers consume `/ws/lidar` and `/ws/map` through separate WebSocket/viewer instances.
- Layered testing guide and implementation handoff.

## Checks

- `python -m unittest discover -s tests -v` — passing after the runtime, launcher, and dashboard additions.
- deterministic preflight via `python -m simulator.runtime.sim_runner` — baseline, walking, and sensor-realism scenarios pass.
- `python -m compileall -q simulator dashboard evaluation tests ros2_ws/src/grocery_sim_mapping` — passing.
- staged `git diff --check` — passing before commit.
- Isaac compatibility checker — passed.
- Isaac headless runtime smoke — passed; actual aisle USD, camera ROS graph, native OmniLidar, clock graph, TF publisher, and ground-truth publisher executed.
- Cross-process ROS via Zenoh — verified: camera image, camera info, LiDAR cloud, ground truth, `/clock`, and TF were discovered and received by Pixi ROS CLI processes.
- Dashboard live integration — verified: `/api/health` passed and `/api/status` reported RGB and LiDAR connected with fresh samples and 29,498 LiDAR points.
- Visual Studio Build Tools verification — Build Tools 17.14.40, MSVC `14.44.35207`, CMake tools, and Windows SDK `10.0.22621.0` present.
- RTAB-Map native build — `rtabmap`, `rtabmap_msgs`, `rtabmap_conversions`, `rtabmap_sync`, `rtabmap_util`, `rtabmap_odom`, and `rtabmap_slam` built successfully with Ninja/MSVC; `rtabmap_odom` executables and `rtabmap_slam/rtabmap.exe` verified in the install tree and as live ROS processes.
- ROS package verification — `ros2 pkg prefix rtabmap_odom` and `ros2 pkg prefix rtabmap_slam` resolve successfully; `ros2 pkg executables` lists `icp_odometry.exe`, `rgbd_odometry.exe`, `stereo_odometry.exe`, and `rtabmap.exe`.
- RTAB-Map live launch — initial Windows parameter typing caused `rtabmap` to exit with `3221226505`; changing slash-qualified RTAB-Map internal parameters to strings and using the native LiDAR-only 3D path fixed startup. A real Isaac run produced `/slam/odom` processing and nonzero `/slam/map_cloud` data accepted by the consolidated launcher.

## Version-sensitive runtime assumptions

The runner follows the installed Isaac 6.1 examples: camera helpers and camera info use the current ROS bridge graph, LiDAR uses a schema-created native `OmniLidar` with the current tick-rate API and `RtxLidarROS2PublishPointCloud`, and `/clock` uses `ROS2PublishClock`. TF is published as a ROS `TFMessage` with the frozen contract frame IDs because Isaac's computed-tree helper emits `world` for the stage root.

## Known risks

1. RTAB-Map optional integrations such as `grid_map`, AprilTag, and ArUco were not required for the LiDAR ICP/SLAM baseline.
2. The dashboard uses raw ROS Image decoding with OpenCV and does not require `cv_bridge` at runtime.
3. The browser page uses a pinned Three.js module for point display; ROS/backend integration is verified, while browser rendering and map quality remain visual/runtime QA steps.

## Git synchronization

Earlier implementation milestones and handoff cleanup were pushed to `origin/master`. The current native Isaac/ROS runtime milestone is the verified working-tree update described above and is pushed after the final test pass; the exact SHA is recorded by the final handoff message.

## Final correctness and integration pass

**Date:** 2026-09-12

- Separated truth TF from estimator TF: the simulator publishes `sim_world -> truth_sensor_rig`; RTAB-Map owns `map -> odom -> sensor_rig` and static sensor children.
- Made the mapping baseline explicitly LiDAR-only and passed scenario-selected mapping parameters into the packaged launch. Root and packaged contracts/configs are equality-tested.
- Corrected the USD camera basis to the ROS-compatible optical convention and added basis regression coverage. The walking runtime applies the complete sampled quaternion to the actual USD `SensorRig`.
- Added initial SE(3) evaluation alignment, orientation RMSE, quaternion CSV fields, bounded tracking freshness states, one-second live metric throttling, and Isaac clock/sensor timestamp-offset status.
- Added run-isolated status/artifact validation, scenario-derived collector duration, realtime-by-default launch behavior with `-Fast`, explicit `run_sim.ps1 -Gui/-Headless`, and RMW/ROS-domain defaults for the dashboard.
- Added separate orbit/autofit map/LiDAR viewers using independent `/ws/map` and `/ws/lidar` streams.
- Final live baseline `runs/20260912-155756059/`: 1,230 Isaac frames, RGB/LiDAR/clock/ground-truth samples, native RTAB-Map database, nonzero map stream, quaternion CSVs, `initial_se3` metrics, and no runtime errors in the preserved logs beyond the Windows ROS log-symlink warning.
- Live dashboard probe during `runs/20260912-142156510/`: `/api/health` passed; RGB, LiDAR, and map caches were connected with 23,924 LiDAR points and 3,561 accumulated map points; `/api/metrics` reported `tracking_state=tracking` and `initial_se3` alignment.
- Live walking smoke `runs/final_walking_runtime_smoke.json`: 120 frames completed and the final rig quaternion was non-identity, confirming full sampled orientation reached USD runtime state. Live sensor-realism smoke `runs/final_sensor_realism_runtime_smoke.json`: 21 noisy clouds received and published with timestamp offsets recorded. The GUI smoke `runs/final_gui_runtime_smoke.json` completed 120 frames without `--no-window`.
- The collector now waits for the first live sample before measuring its scenario-derived capture window, preventing Isaac startup time from consuming the evaluation interval.
- Final automated suite: 39 tests passing; Python compileall and PowerShell parse checks passing; native RTAB package prefixes and executables verified.

## 2026-09-20 — specification startup preparation (G00)

Owner scope: setup only; no full production run. Main checkout preserved at
master/d5e825c8f6dab77aa6a1007c9731c226b588dfcf. Supplied untracked specification
and handoff remain intact. Added short AGENTS.md pointer, copied verified roles
and all 12 hash-matching storyboard images to expected root paths, configured
project Astra/medium, Luna/high and dedicated Sol/high using installed Codex
0.155.0-alpha.9.2. Actual host turn-context routing confirms those settings.
All six disposable rejection/revision/approval/stale/base-move/missing-evidence
rehearsal cases completed with independent Sol verdicts. Main source unchanged.

G00: workflow verified; technical baseline partially verified; saved visual
baseline BLOCKED. A 180-frame Isaac smoke observed 88 RGB callbacks/29 LiDAR
clouds. Windows Code Integrity blocks the recorder dependency libcblas.dll in
the ROS Pixi environment; policy owner approval or an approved compatible build
is required before the recorder smoke can yield an inspectable frame. No bypass.
Teardown InvalidHandle traceback remains recorded, not certified as clean exit.

Scoped setup repair: backed up installed grocery_sim_mapping surfaces and rebuilt
only that pure-Python package with Pixi --as-is. Installed launch/config hashes,
package prefix/import and launch argument expansion now match/pass. No upgrades.

Start a fresh Codex task/session to pick up project defaults; live limits still
need checking (this session: four total slots, full-access override). Existing
uncommitted setup files require verified bootstrap into new worktrees. G01-G09
not started; no pushes or production source commits. Full evidence and exact
remaining blocker: [setup report](review/setup-20260920/report.md).

## 2026-09-20 — production run opened

- Start: 2026-09-20 23:21:35 PDT (UTC-07:00).
- Working deadline: 2026-09-21 11:21:35 PDT (approximately 12 hours).
- Integration branch: `codex/storyboard-video`, isolated from the preserved dirty
  `master` checkout at `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`.
- Active authority: `EXECUTION_OVERRIDES.md`; every role uses GPT-5.6 Sol with
  high reasoning. Two implementation workers maximum, with a separate reviewer.
- Delivery target: native 1920x1080, 30 fps, exactly 45 seconds / 1,350 frames,
  preserving all 12 storyboard shots. Development previews use 1280x720.
- First implementation priorities: replace the recorder's blocked NumPy/OpenCV
  path with the smallest policy-compliant mechanism, and build the data-driven
  12-shot presentation/timeline path. Expensive simulator and GPU work remains
  serialized.

## 2026-09-21 — raw capture recorder policy repair

- `RawCaptureWriter` now loads the required generated Clock, Image,
  PointCloud2, and TF message submodules under their canonical package names.
  It no longer imports aggregate `sensor_msgs.msg` or CameraInfo, so the
  dedicated capture process does not load NumPy or the blocked BLAS library.
- Capture manifest version 2 stores `/clock`, RGB Image, PointCloud2, `/tf`, and
  `/tf_static` in the raw SQLite bag. Camera calibration remains the separate
  scenario-derived `camera_info.json`, explicitly labeled
  `configured_intrinsics` and `observed_ros_message=false`.
- A separate-process Zenoh publisher/player test delivered serialized messages
  for every retained topic into the real writer. Persisted counts and types,
  clean SQLite closure, `ros2 bag info`, and `numpy_loaded=false` all passed.
  No Isaac or GPU workload was run for this repair.

### Fixed-horizon completion correction

- Production capture now gives the raw writer the simulator's absolute
  trajectory end clock. A nonzero first observed clock therefore no longer
  moves the target beyond the fixed Isaac frame horizon; relative-duration
  mode remains available for general replay use.
- Reaching the requested clock is required for `status=complete`. A bounded
  post-start clock-progress timeout records an incomplete result and exits on
  a stopped source instead of waiting for the outer production timeout.
- The capture wrapper checks both recorder exit codes before reading metadata.
  A Pixi/Zenoh regression reached an exact `2.1 s` horizon from a first clock
  of `2.0 s`, retained all five raw topics, and kept NumPy unloaded. CPU-only
  tests passed; no Isaac or GPU workload was run.
