# One-Sitting Layer-by-Layer Testing Guide

This guide is intentionally ordered from lowest-level runtime sanity to the full stack. On the current host, the Isaac/ROS runtime, dashboard backend, native RTAB-Map binaries, and a complete non-interactive baseline have passed. Browser rendering, estimator behavior, map quality, and walking motion remain part of the consolidated user session.

Before starting, open a terminal for each long-running process, keep logs visible, and record screenshots plus the exact command for every failed layer. Use simulation time everywhere.

## 1. Environment/runtime sanity

**Goal:** Confirm the target can run the intended stack.
**Prerequisites:** Isaac Sim, ROS 2, RTAB-Map binaries, NVIDIA driver.
**Commands:** `C:\isaacsim\isaac-sim.compatibility_check.bat`; `pixi run ros2 --help`; `pixi run ros2 pkg prefix grocery_sim_mapping`; `pixi run ros2 pkg prefix rtabmap_odom`; `pixi run ros2 pkg prefix rtabmap_slam`.
**What should open:** Nothing; commands should return tool/version information.
**What success looks like:** Target OS/GPU and all required commands are present.
**What failure looks like:** Missing command, unsupported runtime, or GPU initialization error.
**Evidence:** Command output and driver/runtime versions.
**Stop/cleanup:** None.

## 2. Grocery aisle rendering

**Goal:** Verify deterministic procedural geometry.
**Prerequisites:** Layer 1; Isaac Sim session.
**Commands:** `./scripts/run_sim.ps1 -Frames 1200 -Realtime -StartZenohRouter -Gui`; use `-Headless` instead for a non-interactive run; run `scripts/run_sim.ps1 -PreflightOnly` first if desired.
**What should open:** The Isaac runtime log should show a built stage with a floor, two shelf rows, multiple bays/levels, and simple product proxies; the repository launcher opens the viewport by default and supports `-Headless` for repeatable ROS testing.
**What success looks like:** Shelves are upright, aisle is navigable, lighting is usable, and repeated fixed-seed loads match.
**What failure looks like:** Empty stage, overlapping shelves, missing products, or runtime import errors.
**Evidence:** Scene screenshot and preflight JSON.
**Stop/cleanup:** Close the Isaac stage without saving generated caches.

## 3. Sensor rig movement

**Goal:** Verify the rig owns pose and moves independently of sensor code.
**Prerequisites:** Layer 2.
**Commands:** The native runner consumes `config/trajectories/straight.yaml`; inspect `/tf` and `/sim/ground_truth/pose` over 5–10 seconds.
**What should open:** The camera and LiDAR children move with `/World/SensorRig` down the aisle.
**What success looks like:** Forward motion is smooth and deterministic; sensor offsets remain fixed.
**What failure looks like:** Sensor floats, rotates unexpectedly, or motion is embedded only in camera behavior.
**Evidence:** Short screen recording and start/end rig pose.
**Stop/cleanup:** Stop simulation and reset stage.

## 4. ROS RGB publication

**Goal:** Verify camera data is visible on ROS.
**Prerequisites:** Layers 1–3; ROS environment sourced.
**Commands:** `ros2 topic list`; `ros2 topic hz /sim/camera/rgb/image_raw`; `ros2 topic echo --once /sim/camera/rgb/camera_info`; `ros2 topic hz /clock`.
**What should open:** No new window; terminal receives image rate, camera info, and clock.
**What success looks like:** Image rate is near 30 Hz, camera info is non-empty, and timestamps advance.
**What failure looks like:** Topic absent, zero rate, stale timestamps, or wrong frame.
**Evidence:** Topic list, one camera-info message, and rate output.
**Stop/cleanup:** Stop the simulator after capturing evidence.

## 5. Localhost RGB stream

**Goal:** Verify browser data comes from the ROS camera cache.
**Prerequisites:** Layer 4; run the Zenoh router and native Pixi dashboard.
**Commands:** `./scripts/run_dashboard.ps1`; browse to `http://localhost:8080`; in a second terminal `Invoke-RestMethod http://localhost:8080/api/health`.
**What should open:** Diagnostic page with a live RGB panel and status JSON.
**What success looks like:** Frame updates, source status becomes connected, and receive age stays low.
**What failure looks like:** Blank MJPEG, 404, stale age, or dashboard connected without ROS frames.
**Evidence:** Browser screenshot plus `/api/status`.
**Stop/cleanup:** Ctrl+C dashboard; stop simulator if proceeding no further.

## 6. LiDAR publication

**Goal:** Verify LiDAR points exist on the public ROS topic.
**Prerequisites:** Layer 3; simulator and Zenoh router running.
**Commands:** `ros2 topic hz /sim/lidar/points`; `ros2 topic echo --once /sim/lidar/points`; inspect `frame_id`.
**What should open:** PointCloud2 messages near 10 Hz.
**What success looks like:** Non-zero point count, ranges within configured limits, frame `lidar_link`.
**What failure looks like:** Empty cloud, wrong frame, no cadence, or points only when the browser is open.
**Evidence:** One message summary, rate, and point count.
**Stop/cleanup:** Keep simulator running for Layer 7 or stop it.

## 7. TF / ground-truth sanity

**Goal:** Verify transform ownership and truth isolation.
**Prerequisites:** Layers 4 and 6; simulator and Zenoh router running.
**Commands:** `ros2 topic echo --once /sim/ground_truth/pose`; `ros2 topic list` (confirm `/tf` and `/tf_static`); `ros2 run tf2_ros tf2_echo sim_world truth_sensor_rig`; `ros2 run tf2_ros tf2_echo map sensor_rig`; `ros2 run tf2_tools view_frames`.
**What should open:** Ground-truth pose plus a TF graph for sensor frames.
**What success looks like:** `sim_world -> truth_sensor_rig` is a separate truth branch; the estimator tree is `map -> odom -> sensor_rig -> camera_link/camera_optical_frame/lidar_link`; no simulator `map -> odom`; ground truth is not `/slam/odom`.
**What failure looks like:** Missing static TF, optical axes flipped, or truth wired into SLAM odometry.
**Evidence:** TF graph, one pose message, and topic remapping output.
**Stop/cleanup:** Remove generated TF graph image only if it is outside tracked files.

## 8. Localhost LiDAR viewer

**Goal:** Verify browser LiDAR transport and decimation.
**Prerequisites:** Layers 5–7; dashboard running.
**Commands:** Refresh `http://localhost:8080`; watch `/api/status`; inspect browser console only if the canvas is blank.
**What should open:** Point cloud panel with visible points and a stable browser frame rate.
**What success looks like:** Browser point budget is bounded while `/sim/lidar/points` remains at full configured rate.
**What failure looks like:** Browser freezes, no points, or simulator ROS rate changes when rendering is slow.
**Evidence:** Screenshot and status point counts.
**Stop/cleanup:** Reset view; leave simulator/dashboard running for mapping.

## 9. RTAB-Map odometry

**Goal:** Verify estimator odometry is produced from LiDAR, not truth.
**Prerequisites:** Layers 6–7; the native `rtabmap_odom` and `rtabmap_slam` packages verified in Layer 1.
**Commands:** `./scripts/run_mapping.ps1`; `ros2 topic hz /slam/odom`; `ros2 topic echo --once /slam/odom`.
**What should open:** RTAB-Map/ICP logs and odometry messages. The baseline explicitly uses LiDAR only (`subscribe_scan_cloud=true`, `subscribe_rgb=false`, `subscribe_depth=false`, `subscribe_odom_info=true`, `Reg/Strategy=1`); RGB remains available for the separate ROS/dashboard camera path.
**What success looks like:** `/slam/odom` advances from LiDAR input and has the expected `odom` relationship.
**What failure looks like:** Node fails to launch, no odometry, or launch remaps `/sim/ground_truth/pose` into odometry.
**Evidence:** Launch log, odom message, and `ros2 node info`.
**Stop/cleanup:** Ctrl+C mapping; clear only target runtime caches.

## 10. 3D map accumulation

**Goal:** Verify RTAB-Map accumulates a map.
**Prerequisites:** Layer 9.
**Commands:** Run mapping and simulator for a full straight pass; inspect `map`/map cloud topics.
**What should open:** Map data grows as the rig moves.
**What success looks like:** Aisle structure is recognizable and map frame is owned by SLAM.
**What failure looks like:** No growth, immediate tracking loss, or simulator truth used to remove drift.
**Evidence:** Before/after point counts and a map screenshot.
**Stop/cleanup:** Stop mapping and simulator after the pass.

## 11. Localhost map viewer

**Goal:** Verify map cloud reaches the dashboard.
**Prerequisites:** Layers 8–10.
**Commands:** Keep dashboard running and reload the browser.
**What should open:** LiDAR and accumulated-map panels update independently; `/api/status` map age remains low.
**What success looks like:** Map transport is independent from LiDAR browser decimation and the two panels remain independently resettable.
**What failure looks like:** Live map topic exists but browser remains stale.
**Evidence:** Screenshot, `/api/status`, and map topic rate.
**Stop/cleanup:** Stop long-running processes.

## 12. Evaluation metrics

**Goal:** Produce defensible trajectory metrics.
**Prerequisites:** A run with truth and estimated odometry.
**Commands:** The consolidated launcher runs `evaluation.ros_collector` and writes `runs/<run_id>/`; for an isolated report, export synchronized CSVs and call `evaluation.metrics.compute_metrics` followed by `evaluation.run_io.write_run`.
**What should open:** No GUI; JSON/CSV artifacts.
**What success looks like:** ATE RMSE/median, max error, documented RPE interval, distance, duration, tracking-loss count, orientation RMSE, and `alignment_policy: initial_se3` are present. CSVs contain `qx,qy,qz,qw`; the Isaac status records clock-to-sensor timestamp offsets.
**What failure looks like:** Missing timestamps, invented map-completeness percentage, or truth/estimate misalignment.
**Evidence:** `metrics.json` and metadata.
**Stop/cleanup:** Keep run artifacts local; they are ignored by Git.

## 13. Walking trajectory

**Goal:** Verify human-like sensor motion without a humanoid.
**Prerequisites:** Layers 1–12.
**Commands:** Load `config/scenarios/walking_baseline.yaml`; repeat Layers 3–12.
**What should open:** Rig follows the aisle with small vertical bob, lateral sway, and orientation oscillation.
**What success looks like:** Motion is fixed-seed reproducible, the actual USD `SensorRig` follows the full sampled pitch/yaw orientation, and sensors remain attached.
**What failure looks like:** Excessive oscillation, aisle collisions, or non-deterministic replay.
**Evidence:** Pose trace and metrics comparison.
**Stop/cleanup:** Stop scenario and restore baseline config.

## 14. Sensor-realism toggle

**Goal:** Verify ideal mode remains unchanged and realism is explicit.
**Prerequisites:** Baseline tested.
**Commands:** Load `config/scenarios/sensor_realism.yaml`; compare with baseline.
**What should open:** Same dashboard/ROS paths with modest range noise, dropout, and timing jitter.
**What success looks like:** `noise.enabled: false` reproduces ideal behavior; enabled variation is measurable and bounded.
**What failure looks like:** Noise is always on, browser-only noise changes SLAM input, or config is ignored.
**Evidence:** Config snapshot, point counts, and metric comparison.
**Stop/cleanup:** Stop scenario and return to baseline.

### Production 4K sensor capture

Keep `walking_baseline.yaml` at 1280×720 for previews and routine tests. The production sensor capture uses native 3840×2160 RGB at 30 Hz with the same accepted LiDAR and 20.5-second walking trajectory:

```powershell
.\scripts\capture_simulation.ps1 -Scenario config/scenarios/production_walking_4k.yaml -Realtime -Headless
```

`-Realtime` preserves 30 Hz simulation timestamps while the capture launcher limits simulation to 0.125× wall time so both independent raw 4K subscribers can drain each sample. The completed capture must report 3840×2160 in `camera_info.json` and `rgb_video.json`, and `rgb_cadence.json` must report a complete, matching 30 Hz common recorder/bag window. Verify `capture_manifest.json` before using the bag or video downstream.

## 15. Scenario switching

**Goal:** Verify composition of environment, sensors, trajectory, and mapping config.
**Prerequisites:** Layers 2–14.
**Commands:** Run preflight for each scenario: `python -m simulator.runtime.sim_runner --scenario config/scenarios/baseline_straight.yaml`, then `walking_baseline.yaml`, `sensor_realism.yaml`, and `production_walking_4k.yaml`.
**What should open:** Four valid preflight summaries with expected trajectory, noise, and production-resolution differences.
**What success looks like:** No hard-coded paths or stale configuration leaks.
**What failure looks like:** One scenario changes the public contract or cannot resolve its components.
**Evidence:** Four JSON summaries.
**Stop/cleanup:** None.

## 16. Full baseline launch

**Goal:** Exercise the complete stack after isolated layers pass.
**Prerequisites:** All prior layers.
**Commands:** `./scripts/run_baseline.ps1` (realtime by default); pass `-Gui` to show the Isaac viewport or `-Fast` for an accelerated run.
**What should open:** Dashboard, simulator, ROS graph, RTAB-Map, and accumulating map.
**What success looks like:** One coherent run produces RGB, LiDAR, TF, odom, map, dashboard state, and evaluation artifacts.
**What failure looks like:** A failure whose layer cannot be isolated from the start; return to the first failed layer.
**Evidence:** Full command logs, screenshots, topic list, and run directory.
**Stop/cleanup:** Stop mapping, simulator, and dashboard in that order; confirm no orphaned processes remain.
