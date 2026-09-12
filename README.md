# Grocery Aisle RGB + LiDAR Simulation

This repository is a layered simulation harness for a grocery-store aisle sensor rig. It keeps the simulator's ground truth separate from SLAM, publishes a stable ROS 2 contract, and exposes ROS-visible RGB, LiDAR, map, and evaluation state through a localhost diagnostic dashboard.

## Current implementation state

The dependency-light configuration, geometry, trajectory, transform, dashboard-state, and evaluation layers are implemented and covered by non-interactive tests. Isaac Sim, ROS 2, and RTAB-Map runtime integration is present as explicit launch/adaptor boundaries but is **implemented, not yet interactively verified** on this machine.

The live host audit found Windows 10 Home build 26200, an NVIDIA RTX 4070 Ti, and no `ros2`, Isaac Sim, or `rtabmap` executable on `PATH`. See [GATE_HANDOFF.md](GATE_HANDOFF.md) and [TESTING_GUIDE.md](TESTING_GUIDE.md).

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

On a supported ROS 2 + Isaac Sim environment, use the PowerShell scripts in `scripts/`. The dashboard is intended to run at [http://localhost:8080](http://localhost:8080).

## Configuration

The `.yaml` files are JSON-compatible YAML so the core loader remains usable without a third-party parser. If PyYAML is installed, ordinary YAML is also accepted. Begin with `config/contracts.yaml` and `config/scenarios/baseline_straight.yaml`.

## Safety and repository hygiene

Runtime bags, maps, caches, logs, and secrets are ignored. GitHub credentials are deliberately not documented in the repository; use the existing local Git configuration.
