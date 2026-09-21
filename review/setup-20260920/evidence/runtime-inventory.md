# Runtime inventory and bounded Isaac smoke

Date: 2026-09-20 (America/Los_Angeles)

This record covers read-only setup checks plus one explicitly authorized, one-frame headless Isaac startup smoke. No packages, drivers, source files, or configuration were changed. The smoke did not start RTAB-Map, the dashboard, Zenoh router, or the consolidated production launcher.

## Hardware and host

Commands:

```powershell
Get-CimInstance Win32_OperatingSystem | Select Caption,Version,BuildNumber,OSArchitecture,TotalVisibleMemorySize,FreePhysicalMemory
Get-CimInstance Win32_Processor | Select Name,NumberOfCores,NumberOfLogicalProcessors
Get-CimInstance Win32_VideoController | Select Name,AdapterRAM,DriverVersion,DriverDate
nvidia-smi --query-gpu=name,memory.total,driver_version,compute_cap --format=csv,noheader
Get-CimInstance Win32_LogicalDisk -Filter "DriveType=3" | Select DeviceID,FileSystem,Size,FreeSpace
```

Observed:

- Windows 11 Home 10.0.26200, 64-bit; Ryzen 7 7700X, 8 cores/16 logical processors.
- Total visible RAM 32,670,328 KB (~31.16 GiB); free at inventory 20,098,412 KB (~19.17 GiB).
- `C:` NTFS 930.55 GB total, 708.55 GB free.
- NVIDIA GeForce RTX 4070 Ti, `12282 MiB` reported by `nvidia-smi`, driver `595.71`, compute capability `8.9`. Integrated AMD Radeon was also enumerated; Isaac reported it as unsupported and selected the NVIDIA GPU.

## Installed runtimes and tools

Commands and observed results:

```powershell
Get-Content C:\isaacsim\VERSION
C:\isaacsim\python.bat --version
pixi --version
pixi info --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe --version
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffmpeg.exe -version
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ffprobe.exe -version
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\cmake.exe --version
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\ninja.exe --version
```

- Isaac version file: `6.1.0-rc.26+release.49347.2d230af4.gl`; Kit Python 3.12.13.
- Pixi 0.80.0; resolved workspace platform `win-64` with Pixi virtual CUDA `12.9`; env Python 3.12.13 (conda-forge), `rclpy` importable.
- FFmpeg/ffprobe 8.1.2, CMake 4.3.4, Ninja 1.13.2. These are present in the Pixi environment but are not on the ordinary process PATH.
- Installed RTAB-Map package XML reports `rtabmap_odom` and `rtabmap_slam` version `0.23.7`. `icp_odometry.exe`, `rtabmap.exe`, and `RTABMap.exe` exist in the external workspace install. No executable version command was run because it could start a node or GUI.

## Launcher and package relationship

`scripts/resolve_runtime_paths.ps1` defaults ROS to `C:\IsaacSim-ros_workspaces\jazzy_ws` and resolves Pixi from PATH or `%LOCALAPPDATA%\pixi\bin`. `scripts/run_sim.ps1` calls the repository `simulator/runtime/isaac_sim_runner.py` through `C:\isaacsim\python.bat`. `scripts/run_mapping.ps1` launches the installed `grocery_sim_mapping` package through `ros2 launch` and passes the repository mapping parameter path.

SHA-256 checks show the current package source and installed package are not identical:

| File | SHA-256 prefix | Result |
|---|---|---|
| repository `ros2_ws/src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py` | `C242F309...` | current source |
| workspace `src/grocery_sim_mapping/launch/rtabmap_lidar.launch.py` | `C242F309...` | same as repository |
| installed `install/share/grocery_sim_mapping/launch/rtabmap_lidar.launch.py` | `8632B227...` | stale |
| repository/workspace `config/contracts.yaml` | `276D0AA9...` | same source input |
| installed `config/contracts.yaml` | `49650586...` | stale |

The current source launch includes `scan_cloud_max_points: 50000`; the installed launch does not. A rebuild/install is required before claims about current source mapping behavior can be made. No rebuild or install was performed.

## Bounded smoke

Exact command:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_sim.ps1 `
  -IsaacPython C:\isaacsim\python.bat `
  -Scenario .\config\scenarios\baseline_straight.yaml `
  -Frames 1 -Headless `
  -StatusPath .\runs\setup-20260920\evidence\isaac_runtime_status.json
```

Result: exit code `0`; startup reached `Simulation App Startup Complete`, built aisle USD, sensor rig, camera graph, RTX LiDAR writer, clock/TF graph, simulated one frame, and shut down. Isaac log selected RTX 4070 Ti with 11,994 MB available to Kit; Warp reported CUDA 12.9 and the 4070 Ti (12 GiB, sm_89). The AMD integrated GPU was skipped as unsupported.

Status artifact: [isaac_runtime_status.json](isaac_runtime_status.json)

The artifact reports `isaac_version: 6.1.0`, `frames_simulated: 1`, `primitive_count: 2305`, `product_count: 1995`, expected topic/frame declarations, `ground_truth_odometry_leakage: false`, and no synthetic timestamp fallback. It reports zero observed RGB/LiDAR/ground-truth callback samples because one frame is only a startup smoke; this is not evidence of sensor-stream success.

Warnings/failures observed during the smoke:

- Zenoh router was absent, so RMW warned that peers could not discover each other. This is expected for the isolated simulator smoke because no router was requested.
- Isaac's ROS bridge first failed to import system `rclpy`, then loaded its internal Jazzy `rclpy` successfully.
- The ROS executor emitted an `InvalidHandle` traceback during teardown after simulation completed. The launcher still exited 0. This should be treated as a runtime warning to investigate, not as a clean end-to-end ROS pass.
- Isaac emitted expected/deprecation and sensor warnings (camera `frameSkipCount`, USD diagnostics, LiDAR MotionBVH). No source/config repair was attempted.

After shutdown, `Get-Process` and `nvidia-smi --query-compute-apps` showed no Isaac/Kit/RTAB-Map processes remaining.

## Routing

Configured handoff routing: GPT-5.6 Luna, high effort. The setup routing record independently reports this worker turn as `gpt-5.6-luna` with `effort: high` in `runs/setup-20260920/routing-observed.json`; the Isaac/Pixi launcher itself exposes no model identity.

## Scope conclusion

This smoke verifies that Isaac Sim starts on the installed RTX/driver stack, creates the project stage, and exits cleanly at process level for one frame. It does not verify RGB/LiDAR delivery, ROS graph discovery, RTAB-Map, dashboard integration, mapping quality, or production capture.

## Bounded RGB capture attempt

After the one-frame startup check, one bounded capture was authorized using the existing `evaluation.rgb_video_recorder` entry point and an owned hidden Zenoh router. The simulator was run for 180 frames in realtime (about three simulated seconds), while the recorder requested one second of simulation time and a 120-second startup timeout. All three child process IDs were tracked and cleaned up: router `13472`, recorder `2052`, simulator wrapper `20056`.

The attempt used these existing entry-point arguments (with output paths under this evidence directory):

```text
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml ros2 run rmw_zenoh_cpp rmw_zenohd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml python -m evaluation.rgb_video_recorder --output rgb_smoke.mp4 --metadata rgb_smoke.json --frames-jsonl rgb_smoke_frames.jsonl --camera-info-json rgb_smoke_camera_info.json --duration-seconds 1 --startup-timeout-seconds 120
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_sim.ps1 -IsaacPython C:\isaacsim\python.bat -Scenario config\scenarios\baseline_straight.yaml -PixiPath C:\Users\suyog\AppData\Local\pixi\bin\pixi.exe -RosWorkspace C:\IsaacSim-ros_workspaces\jazzy_ws -Frames 180 -Realtime -Headless -StatusPath isaac_rgb_smoke_status.json
```

The simulator exited `0` and its status artifact proves actual callback activity: `frames_simulated=180`, `observed_rgb_frames=88`, `observed_rgb_hz=30.0`, `lidar.sample_count=29` with 134,625--162,258 points, and 180 ground-truth samples. This is stronger runtime evidence than the one-frame check, while remaining too short for mapping or production acceptance.

The recorder exited `1` before subscribing because importing OpenCV pulled NumPy and Windows Application Control blocked NumPy's `_multiarray_umath` DLL:

```text
ImportError: DLL load failed while importing _multiarray_umath: An Application Control policy has blocked this file.
```

Read-only Code Integrity log inspection identified the blocked dependency precisely. At `2026-09-20 22:34:20` (events 3077 and 3033), `C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe` attempted to load `C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\libcblas.dll`; Windows reported that it did not meet the enterprise signing level or violated policy ID `{0283ac0f-fff1-49ae-ada1-8a933130cad6}`. No policy change, replacement, or reinstall was attempted.

Consequently `rgb_smoke.mp4`, recorder metadata, camera-info JSON, and the requested PNG extraction were not produced. The exact recorder, simulator, and Zenoh stdout/stderr streams are preserved as faithful byte copies under [`rgb-smoke-logs/`](rgb-smoke-logs/) with `.txt` filenames so Git tracks them: `rgb.err.txt`, `rgb.out.txt`, `sim.err.txt`, `sim.out.txt`, `zenoh.err.txt`, and `zenoh.out.txt`. This is a genuine environment blocker for the existing recorder, with no package install or repair attempted. The raw recorder traceback remains in `rgb-smoke-logs/rgb.err.txt`. The simulator status also recorded a ROS executor `InvalidHandle` traceback during teardown in the same run; despite that traceback, the wrapper exited `0` and the process shut down.

Post-run checks found no Isaac, Kit, RTAB-Map, ROS 2, recorder, or Zenoh processes remaining. No further runtime launch should occur until the parent coordinates the scoped package reinstall and decides how to address the recorder DLL policy block.
