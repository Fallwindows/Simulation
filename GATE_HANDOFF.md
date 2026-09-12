# Implementation Handoff

## Repository state

- Repository: `Fallwindows/Simulation`
- Branch: `master`
- Base commit: repository history before the native runtime milestone.
- Current verified commit: see `git log -1` and `origin/master` after this handoff update.

## Implemented

- Typed contracts and scenario composition under `config/` and `simulator/config/`.
- Deterministic aisle geometry under `simulator/environment/`.
- Straight and walking motion under `simulator/motion/`.
- Transform and optical-frame helpers under `simulator/sensors/`.
- Optional Isaac USD geometry adapter under `simulator/environment/isaac_builder.py`.
- ROS topic/frame constants and RTAB-Map launch boundary under `simulator/ros/` and `ros2_ws/`.
- FastAPI dashboard cache/routes and browser diagnostic page under `dashboard/`.
- Evaluation metrics and run artifacts under `evaluation/`.
- Windows scripts under `scripts/`.
- Automated core tests under `tests/`.

## Verified on the current host

- Isaac Sim 6.1 startup and actual aisle USD creation.
- RTX camera graph, native OmniLidar creation, and ROS 2 writers.
- `/clock`, TF, RGB, LiDAR, and ground-truth messages across the live native ROS/Zenoh graph.
- Browser backend `/api/health` plus live RGB/LiDAR cache status.

## Remaining external gate

- RTAB-Map ICP odometry and 3D map accumulation are not runnable yet because the native Windows source build has no Windows SDK/MSVC libraries. The ROS 2 source checkout and repository launch/remappings are ready at the external Jazzy workspace; install Visual Studio Build Tools 2022 Desktop C++/Windows SDK, rerun the source build, then run Layers 9–11 of `TESTING_GUIDE.md`.

## Automated/development checks

- `python -m unittest discover -s tests -v`
- deterministic scenario preflight through `simulator.runtime.sim_runner`
- Python bytecode compilation for project modules
- `git diff --check`

## Expected localhost outputs

- `GET /api/health`
- `GET /api/status`
- `GET /api/metrics`
- `GET /stream/rgb.mjpg`
- `WS /ws/lidar`
- `WS /ws/map`
- browser page at `http://localhost:8080`

## Important architecture

The simulator publishes sensor observations and ground truth. Ground truth is never remapped to `/slam/odom`. RTAB-Map/ICP owns estimator odometry and `map -> odom` once its native binaries are installed. The dashboard consumes ROS-visible caches, not Isaac internals.

## Known risks

- The live machine is Windows 11 Home v25H2 build 26200 with Isaac Sim 6.1 and native ROS 2 Jazzy installed; RTAB-Map binaries remain gated on the Windows C++ toolchain.
- RTAB-Map source is available in the external Jazzy workspace, but its native build still needs Visual Studio Build Tools 2022 Desktop C++ and the Windows SDK.
- The browser point viewer uses a pinned Three.js module URL; backend ROS integration is verified, while browser rendering remains a visual QA step.

## Files to review first

1. `config/contracts.yaml`
2. `simulator/config/loader.py`
3. `simulator/environment/aisle_builder.py`
4. `simulator/motion/trajectory.py`
5. `ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py`
6. `TESTING_GUIDE.md`

## Recommended first user test

Run Layers 1–5 of `TESTING_GUIDE.md` in order on the supported Isaac Sim/ROS 2 machine. Do not start RTAB-Map until RGB, LiDAR, TF, and ground-truth isolation have been observed.
