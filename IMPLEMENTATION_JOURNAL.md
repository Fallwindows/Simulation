# Implementation Journal

## Phase 0 — live rediscovery

**Date:** 2026-09-12
**Status:** Isaac/ROS runtime verified; RTAB-Map native Windows build remains blocked by the missing Windows C++ SDK.

Observed live state:

- Windows 11 Home v25H2, build 26200 (confirmed by Isaac's checker and runtime).
- NVIDIA GeForce RTX 4070 Ti, driver `595.71`, 11.99 GB VRAM; an AMD integrated adapter is also present.
- C: free space before installation: 841.6 GiB; Isaac archive MD5 matched NVIDIA's published `a07968e980072c9ca27b2166443e2d89`.
- Repository is `Fallwindows/Simulation`, branch `master`, remote `origin` points to the expected GitHub SSH URL.
- Port 8080 is free.
- Isaac Sim 6.1.0 installed at `C:\isaacsim`; compatibility checker result: **PASSED**.
- Pixi 0.80.0 and the current native Windows ROS 2 Jazzy workspace installed at `C:\IsaacSim-ros_workspaces\jazzy_ws`.
- `rclpy`, `sensor_msgs_py`, FastAPI, Uvicorn, and OpenCV are available in the Pixi environment.
- `rtabmap_ros` and `rtabmap` source are checked out in the external Pixi workspace, but CMake cannot link without Windows SDK libraries.

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
- Layered testing guide and implementation handoff.

## Checks

- `python -m unittest discover -s tests -v` — 17 tests passing after the runtime additions.
- deterministic preflight via `python -m simulator.runtime.sim_runner` — baseline, walking, and sensor-realism scenarios pass.
- `python -m compileall -q simulator dashboard evaluation tests ros2_ws/src/grocery_sim_mapping` — passing.
- staged `git diff --check` — passing before commit.
- Isaac compatibility checker — passed.
- Isaac headless runtime smoke — passed; actual aisle USD, camera ROS graph, native OmniLidar, clock graph, TF publisher, and ground-truth publisher executed.
- Cross-process ROS via Zenoh — verified: camera image, camera info, LiDAR cloud, ground truth, `/clock`, and TF were discovered and received by Pixi ROS CLI processes.
- Dashboard live integration — verified: `/api/health` passed and `/api/status` reported RGB and LiDAR connected with fresh samples and 29,498 LiDAR points.
- RTAB-Map source build — attempted with Pixi clang-cl/Ninja; blocked at linker setup because `kernel32.lib`, `user32.lib`, `msvcrtd.lib`, and related Windows SDK libraries are absent.

## Version-sensitive runtime assumptions

The runner follows the installed Isaac 6.1 examples: camera helpers and camera info use the current ROS bridge graph, LiDAR uses a schema-created native `OmniLidar` with the current tick-rate API and `RtxLidarROS2PublishPointCloud`, and `/clock` uses `ROS2PublishClock`. TF is published as a ROS `TFMessage` with the frozen contract frame IDs because Isaac's computed-tree helper emits `world` for the stage root.

## Known risks

1. RTAB-Map native Windows compilation still needs an elevated Visual Studio Build Tools/Windows SDK install; the user-scoped installer found no applicable installer and the earlier elevated attempt was canceled.
2. RTAB-Map source is present externally, but no `rtabmap_odom`/`rtabmap_slam` binaries are claimed until that toolchain completes.
3. The dashboard uses raw ROS Image decoding with OpenCV and does not require `cv_bridge` at runtime.
4. The browser page uses a pinned Three.js module for point display; ROS/backend integration is verified, while browser rendering remains a visual QA step.

## Git synchronization

Earlier implementation milestones and handoff cleanup were pushed to `origin/master`. The current native Isaac/ROS runtime milestone is the verified working-tree update described above and is pushed after the final test pass; the exact SHA is recorded by the final handoff message.
