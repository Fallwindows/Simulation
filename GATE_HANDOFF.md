# Implementation Handoff

## Repository state

- Repository: `Fallwindows/Simulation`
- Branch: `master`
- Base commit: repository history before the native runtime milestone.
- Current verified commit: recorded below after the final push.

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
- Native `rtabmap_odom` and `rtabmap_slam` launch with the LiDAR-only simulator stream; the consolidated launcher requires a live `/slam/map_cloud` sample before accepting the run.
- The final consolidated smoke produced an isolated RTAB-Map database, ground-truth/estimate CSVs with quaternion fields, `initial_se3` metrics, and a unique Isaac status artifact under `runs/20260912-191841478/`.
- The final live artifact used the native Isaac Sim 6.1 `Example_Rotary` RTX LiDAR asset with explicit ROS `xyzw` to Isaac `wxyz` conversion and a rig-relative local mount; it recorded 1,230 simulation frames, 30 Hz RGB, dense LiDAR, 73 valid odometry samples, 18 map updates, and zero invalid quaternions.
- TF ownership is split cleanly: `sim_world -> truth_sensor_rig` is visualization-only, while the estimator tree is `map -> odom -> sensor_rig -> camera_link/lidar_link`.

## Remaining manual verification

- The consolidated user session should visually observe GUI rendering, walking motion, RTAB-Map ICP odometry, 3D map accumulation, and the browser's two independent point-cloud panels end to end.

## Automated/development checks

- `python -m unittest discover -s tests -v`
- deterministic scenario preflight through `simulator.runtime.sim_runner`
- Python bytecode compilation for project modules
- Visual Studio Build Tools/MSVC/CMake/Windows SDK verification and native RTAB-Map package/executable checks
- Real Isaac baseline, walking, and sensor-realism headless runtime smokes
- Full dashboard + Zenoh + native RTAB-Map + evaluation-collector smoke with `/tf_static`, RGB, LiDAR, odometry, map, and metrics artifacts
- Live dashboard + Zenoh + RTAB-Map smoke with RGB, LiDAR, odometry, and map caches
- `git diff --check`
- Final live smoke: Isaac 6.1 headless runtime, RGB/LiDAR/clock/TF, Zenoh, dashboard health/status, native RTAB-Map, map publication, collector artifacts, quaternion CSVs, timestamp-offset status, and cleanup.

## Expected localhost outputs

- `GET /api/health`
- `GET /api/status`
- `GET /api/metrics`
- `GET /stream/rgb.mjpg`
- `WS /ws/lidar`
- `WS /ws/map`
- browser page at `http://localhost:8080`

## Important architecture

The simulator publishes sensor observations and ground truth. Ground truth is never remapped to `/slam/odom` or inserted into the estimator TF tree. RTAB-Map/ICP owns estimator odometry and `map -> odom`; the dashboard consumes ROS-visible caches, not Isaac internals. Live metrics interpolate high-rate ground truth at estimator timestamps, reject invalid quaternions, and record explicit initial SE(3) alignment and orientation RMSE.

## Known risks

- The live machine is Windows 11 Home v25H2 build 26200 with Isaac Sim 6.1, native ROS 2 Jazzy, and native RTAB-Map binaries installed.
- Optional RTAB-Map integrations such as `grid_map`, AprilTag, and ArUco were not required for the LiDAR ICP/SLAM baseline and remain outside this build.
- The browser point viewer uses a pinned Three.js module URL; both `/ws/lidar` and `/ws/map` have independent viewers, while browser rendering remains a visual QA step.
- The collector waits for its first live truth/estimate sample before starting the scenario-derived capture window, with a bounded 90-second startup deadline.

## Files to review first

1. `config/contracts.yaml`
2. `simulator/config/loader.py`
3. `simulator/environment/aisle_builder.py`
4. `simulator/motion/trajectory.py`
5. `ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py`
6. `TESTING_GUIDE.md`

## Recommended first user test

Run Layers 1–16 of `TESTING_GUIDE.md` in order on the supported Isaac Sim/ROS 2 machine. Do not interpret the binary checks as map-quality validation; the consolidated session should observe RGB, LiDAR, TF, RTAB-Map odometry, map accumulation, both browser viewers, walking motion, and cleanup.
