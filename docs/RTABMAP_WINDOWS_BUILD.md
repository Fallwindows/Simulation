# Native RTAB-Map on Windows 11

The reproducible native build uses the Windows ROS 2 Jazzy Pixi workspace, Visual Studio Build Tools 2022 Desktop development with C++, MSVC v143, CMake tools, and a Windows 11 SDK. The pinned source revisions are:

- `rtabmap`: `2fbbe19d707e6b9fada74bc6b74c284117197a1c`
- `rtabmap_ros`: `61edb4ee85e35f4cc967fa6b502b58d8e3bc6e4f`

Run `scripts/setup_rtabmap_windows.ps1 -Build` after setting `ISAACSIM_ROS_WORKSPACE` or passing `-RosWorkspace`. The script clones missing sources, checks out those revisions, applies `patches/rtabmap_ros_windows.patch`, verifies MSVC/SDK discovery plus `pcl_conversions`, `image_geometry`, `sensor_msgs`, and `tf2_ros`, and builds `rtabmap_odom`, `rtabmap_slam`, and the packaged `grocery_sim_mapping` launch through the active Pixi manifest. It verifies:

```text
install/Lib/rtabmap_odom/icp_odometry.exe
install/Lib/rtabmap_slam/rtabmap.exe
ros2 pkg prefix grocery_sim_mapping
```

The build intentionally does not set the global `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON`; only the small patched RTAB-Map DLL targets use explicit export visibility where needed. The patch is limited to Windows export/import visibility, `NOMINMAX`/`ERROR` macro collisions, and the MSVC-compatible covariance buffer API. It does not alter RTAB-Map algorithms.

`scripts/run_mapping.ps1` creates `runs/<run-id>/rtabmap.db` and passes that path to the installed launch file. The launch file packages and consumes `config/contracts.yaml` and `config/mapping/rtabmap/params.yaml`, so the frame/topic contract does not depend on a developer's source checkout.
