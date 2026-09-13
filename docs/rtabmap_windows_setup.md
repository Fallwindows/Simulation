# Windows RTAB-Map setup boundary

The Windows ROS 2/RTAB-Map build is intentionally driven by the selected
Pixi workspace, not by a user-specific path committed to this repository.
`scripts/setup_rtabmap_windows.ps1` performs the following in that workspace:

1. clones or reuses pinned `rtabmap` and `rtabmap_ros` source checkouts;
2. synchronizes and installs `ros2_ws/src/grocery_sim_mapping` into the
   workspace source tree and records the repository SHA;
3. runs `pixi install` from the workspace manifest;
4. verifies the C++ toolchain, PCL/ROS dependencies, package prefixes, and
   `install/Lib/rtabmap_odom/icp_odometry.exe` plus
   `install/Lib/rtabmap_slam/rtabmap.exe` when `-Build` is used.

The repository does not claim that the entire RTAB-Map source checkout is
portable inside `ros2_ws` yet: a clean machine must provide a Windows Pixi
workspace manifest and network access for those pinned upstream clones. Pass
`-RosWorkspace` (or set `ISAACSIM_ROS_WORKSPACE`) to select that workspace;
the default is only a convenience for the configured workstation.
