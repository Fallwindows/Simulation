# Implementation Journal

## Phase 0 — live rediscovery

**Date:** 2026-09-12
**Status:** complete; runtime dependencies are not installed on this host.

Observed live state:

- Windows 10 Home, build 26200 (the prompt's Windows 11 assumption was not confirmed).
- NVIDIA GeForce RTX 4070 Ti, driver `32.0.15.9571`; an AMD integrated adapter is also present.
- Repository is `Fallwindows/Simulation`, branch `master`, remote `origin` points to the expected GitHub SSH URL.
- Port 8080 is free.
- `ros2`, Isaac Sim, RTAB-Map, `rtabmap-console`, and `npm` are not on `PATH`.
- Bundled Python 3.12 runtime is available under the local Codex runtime cache.

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
- Optional Isaac USD geometry adapter with delayed imports; runtime-specific sensor graph code remains isolated.
- Dashboard state/cache, FastAPI endpoints, MJPEG route, LiDAR/map WebSocket routes, and minimal browser page.
- ROS 2 topic contract and RTAB-Map LiDAR launch boundary with simulator truth kept separate from odometry.
- ATE/RPE/distance metrics and run artifact serialization.
- Windows-oriented scripts for dashboard, simulator preflight, mapping, and baseline orchestration.
- Layered testing guide and implementation handoff.

## Checks

- `python -m unittest discover -s tests -v` — 17 tests passing.
- deterministic preflight via `python -m simulator.runtime.sim_runner` — baseline, walking, and sensor-realism scenarios pass.
- `python -m compileall -q simulator dashboard evaluation tests ros2_ws/src/grocery_sim_mapping` — passing.
- staged `git diff --check` — passing before commit.
- Isaac Sim sensor graph, ROS 2 message publication, FastAPI runtime, RTAB-Map, browser rendering, and GPU performance — not interactively verified on this host.

## Version-sensitive runtime assumptions

The live host has no Isaac/ROS installation to inspect. The runtime integration therefore avoids claiming a verified Isaac API version. The intended target must validate current RTX camera/LiDAR sensor graph creation, `omni:sensor:tickRate`, ROS 2 image/point-cloud writers, `/clock`, and TF publication before the first runtime session. The RTAB-Map launch is a boundary based on current ROS 2 naming and must be checked with the installed distribution.

## Known risks

1. Windows 10 and RTX 4070 Ti may be outside the current supported Isaac Sim target, depending on the exact release.
2. ROS 2 and RTAB-Map launch argument/executable names vary by distribution.
3. The dashboard ROS bridge deliberately stops at the message-specific subscription boundary because `rclpy`, `cv_bridge`, and PointCloud2 packages are absent here.
4. The browser page uses a pinned Three.js module for point display; live browser performance and offline CDN behavior remain to be verified.

## Git synchronization

Implementation milestone committed as `9b123b677400541d1d849c4160ccff23624e449d` and pushed to `origin/master`; remote SHA matched local SHA after verification.
