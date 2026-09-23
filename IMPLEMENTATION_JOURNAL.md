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

## 2026-09-23 — forensic repair pass (G01)

Owner-authorized implementation goal started at 2026-09-23 05:18:49 UTC with a 12-hour checkpoint deadline of 2026-09-23 17:18:49 UTC. Repair work is isolated from the user's dirty main checkout; base is d5e825c8f6dab77aa6a1007c9731c226b588dfcf. The review is pinned to ab8ebbd51989342c3b1acb6b8b96cf947288cd55; every finding must be rechecked against the repair base. Working ledger: review/FORENSIC_REPAIR_PASS.md. No code candidate is accepted yet; independent Sol review is required. Local commits only; no push or merge authorized by this entry.

## 2026-09-23 — forensic repair pass completion (G01)

The accepted local source candidate is `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25` (tree `ce016082190e387a7b5e4a584db1d839063ea363`, parent `c780edff0cfa3c582da57545f3592703a892a0f9`) on `codex/forensic-repair`. Documentation/evidence commit is pending after independent audit of `review/FORENSIC_REPAIR_PASS.md`, `review/README.md`, this journal appendix, and the scoped evidence directory.

Five independent exact-candidate reviews approved the source tree, including the completed SLAM-path audit and final exact source review with zero blockers. The final review identity is the exact commit/tree above. Historical rejections remain authoritative: `c780edff0cfa3c582da57545f3592703a892a0f9` was rejected for `INT-C780-F01`; `2b8fb77` closes it by validating the consumed pose artifact's role, frame, optimization state, map version, path, size, and SHA-256 and deriving alignment from retained verified bytes.

Recorded checks on the exact candidate: independent full discovery 127/127; evaluator 22/22; reviewer integration selections 81/81 and 71/71; runtime capture 18/18; tracking 31/31; runtime-review span 22/22; runtime-to-tracker and reversed-order probes passed; 69/69 generated retail files were byte-identical; Python compileall passed; PowerShell parsing passed for ten scripts; and `git diff --check` passed. The first orchestrator attempt could not create its worktree-scoped temporary directory, and a second isolated exact-tree copy run under the authorized main workspace produced 46 Windows fixture-`TEMP` permission errors with no assertion failures; an independent exact-tree reviewer completed 127/127 successfully.

Installed Isaac/ROS production replay and restart validation were blocked by Windows Application Control. No 45-second/1,350-frame or 1080p full film, silent master, twelve-shot acceptance package, product-recognition model, evidence-backed full 3D extents, or time-indexed map-history presentation was produced. Visual evidence remains bounded to comparable 1280×720 stills at 0 s and 8 s. No owner acceptance, merge to `master`, push, release, or publication is recorded.

## 2026-09-23 — forensic report and evidence delivery

The independent report/evidence audit passed. Local delivery commit `0310fcf0762fe178d93c3109857c6ed16ad9e263` (tree `a625659c2f650bcb8d87077054ce1eafe0ac2943`) contains the reviewed forensic report, review README, journal snapshot current at that commit, and 11 preview evidence files. This delivery entry supersedes the earlier “documentation/evidence commit is pending” state.

The accepted source commit remains `2b8fb77aa12ff649bf5fc0eaaccb5efc2007cb25`; the delivery commit changed no source code. Nothing was pushed or merged.
