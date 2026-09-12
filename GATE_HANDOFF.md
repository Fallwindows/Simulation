# Implementation Handoff

## Repository state

- Repository: `Fallwindows/Simulation`
- Branch: `master`
- Base commit: `d428baa335a7e0375b9ec9362026dc824c8905e5`
- Final commit: `daabc6a880aa205a121c1b6ecb8858022f177fbe`

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

## Not yet interactively verified

- Isaac Sim startup and actual aisle rendering.
- RTX camera/LiDAR creation and ROS 2 writers.
- `/clock`, TF, RGB, LiDAR, and ground-truth messages on a live ROS graph.
- RTAB-Map ICP odometry and 3D map accumulation.
- Browser MJPEG/WebSocket updates fed by live ROS subscriptions.
- GPU performance and walking/noise behavior on the target runtime.

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

The simulator publishes sensor observations and ground truth. Ground truth is never remapped to `/slam/odom`. RTAB-Map/ICP owns estimator odometry and `map -> odom`. The dashboard consumes ROS-visible caches, not Isaac internals.

## Known risks

- The live machine is Windows 10 Home build 26200 and lacks Isaac Sim, ROS 2, and RTAB-Map on `PATH`.
- Exact runtime support and current Isaac/ROS API behavior require a supported installation.
- The browser point viewer uses a pinned Three.js module URL; live browser performance and offline CDN behavior remain to be verified.

## Files to review first

1. `config/contracts.yaml`
2. `simulator/config/loader.py`
3. `simulator/environment/aisle_builder.py`
4. `simulator/motion/trajectory.py`
5. `ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py`
6. `TESTING_GUIDE.md`

## Recommended first user test

Run Layers 1–5 of `TESTING_GUIDE.md` in order on the supported Isaac Sim/ROS 2 machine. Do not start RTAB-Map until RGB, LiDAR, TF, and ground-truth isolation have been observed.
