# Grocery Aisle RGB + LiDAR Simulation

This repository is a layered simulation harness for a grocery-store aisle sensor rig. It keeps the simulator's ground truth separate from SLAM, publishes a stable ROS 2 contract, and exposes ROS-visible RGB, LiDAR, map, and evaluation state through a localhost diagnostic dashboard.

## Current implementation state

The dependency-light core and the native Isaac runtime are implemented. Isaac Sim 6.1.0 is installed at `C:\isaacsim`; the compatibility checker passed on this Windows 11 Home 25H2 host with an RTX 4070 Ti, and the native ROS 2 Jazzy Pixi workspace is installed at `C:\IsaacSim-ros_workspaces\jazzy_ws`.

The real runtime is `simulator/runtime/isaac_sim_runner.py`. It builds the USD aisle, publishes RGB and camera info through the Isaac ROS 2 bridge, publishes an RTX LiDAR PointCloud2 writer, publishes `/clock`, publishes the required TF chain, and publishes ground truth separately from SLAM. `scripts/run_dashboard.ps1` runs the ROS-fed localhost dashboard at `http://localhost:8080`.

RTAB-Map ROS 2 and its native `rtabmap` core are now built in the external Pixi workspace with Visual Studio Build Tools 2022, MSVC v143, CMake/Ninja, PCL, and the Windows 11 SDK. The installed `rtabmap_odom` and `rtabmap_slam` executables are verified; the consolidated runtime session remains the place to validate estimator/map quality.

## Architecture

```text
config/*.yaml
       |
simulator/config + geometry + motion + sensor contracts
       |
Isaac runtime adapter -> ROS 2 topics -> dashboard ROS bridge
                                      |\
                                      | +-- MJPEG RGB
                                      | +-- WebSocket LiDAR/map
                                      | +-- JSON status/metrics
                                      |
                         RTAB-Map odometry/SLAM -> evaluation
```

Coordinate convention: `x` follows the aisle, `y` is aisle width, and `z` is up. All internal distances are SI units. The simulator owns `sim_world -> sensor_rig`; SLAM owns `map -> odom`.

## Quick checks

From the repository root:

```powershell
python -m unittest discover -s tests -v
python -m simulator.runtime.sim_runner --scenario config/scenarios/baseline_straight.yaml --steps 10
```

On a supported ROS 2 + Isaac Sim environment, use the PowerShell scripts in `scripts/`. The dashboard is intended to run at [http://localhost:8080](http://localhost:8080). `run_sim.ps1` opens the Isaac viewport by default; add `-Headless` for non-interactive runs. Pixi is resolved from PATH, then the normal per-user Pixi installation, or an explicit `-PixiPath`.

## Configuration

The `.yaml` files are JSON-compatible YAML so the core loader remains usable without a third-party parser. If PyYAML is installed, ordinary YAML is also accepted. Begin with `config/contracts.yaml` and `config/scenarios/baseline_straight.yaml`.

## Safety and repository hygiene

Runtime bags, maps, caches, logs, and secrets are ignored. GitHub credentials are deliberately not documented in the repository; use the existing local Git configuration.
