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

- Frames: `sim_world -> sensor_rig -> camera_link -> camera_optical_frame` and `sensor_rig -> lidar_link`.
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
- RTAB-Map native build — `rtabmap`, `rtabmap_msgs`, `rtabmap_conversions`, `rtabmap_sync`, `rtabmap_util`, `rtabmap_odom`, and `rtabmap_slam` built successfully with Ninja/clang-cl; `rtabmap_odom` executables and `rtabmap_slam/rtabmap.exe` verified in the install tree.
- ROS package verification — `ros2 pkg prefix rtabmap_odom` and `ros2 pkg prefix rtabmap_slam` resolve successfully; `ros2 pkg executables` lists `icp_odometry.exe`, `rgbd_odometry.exe`, `stereo_odometry.exe`, and `rtabmap.exe`.
- RTAB-Map live launch — initial Windows parameter typing caused `rtabmap` to exit with `3221226505`; changing slash-qualified RTAB-Map internal parameters to strings and using the native LiDAR-only 3D path fixed startup. A real Isaac 180-frame run produced `/slam/odom` processing and 297 `/slam/map_cloud` points consumed by the dashboard.

## Version-sensitive runtime assumptions

The runner follows the installed Isaac 6.1 examples: camera helpers and camera info use the current ROS bridge graph, LiDAR uses a schema-created native `OmniLidar` with the current tick-rate API and `RtxLidarROS2PublishPointCloud`, and `/clock` uses `ROS2PublishClock`. TF is published as a ROS `TFMessage` with the frozen contract frame IDs because Isaac's computed-tree helper emits `world` for the stage root.

## Known risks

1. RTAB-Map optional integrations such as `grid_map`, AprilTag, and ArUco were not required for the LiDAR ICP/SLAM baseline.
2. The dashboard uses raw ROS Image decoding with OpenCV and does not require `cv_bridge` at runtime.
3. The browser page uses a pinned Three.js module for point display; ROS/backend integration is verified, while browser rendering and map quality remain visual/runtime QA steps.

## Git synchronization

Earlier implementation milestones and handoff cleanup were pushed to `origin/master`. The current native Isaac/ROS runtime milestone is the verified working-tree update described above and is pushed after the final test pass; the exact SHA is recorded by the final handoff message.
