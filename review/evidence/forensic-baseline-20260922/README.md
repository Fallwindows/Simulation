# Baseline preview attempt (repair base d5e825c)

- Command: `scripts/capture_simulation.ps1 -PixiPath C:\Users\suyog\AppData\Local\pixi\bin\pixi.exe -RosWorkspace C:\IsaacSim-ros_workspaces\jazzy_ws -IsaacPython C:\isaacsim\python.bat -Scenario config/scenarios/baseline_straight.yaml -Realtime -Headless`
- Result: Isaac completed 1,230 frames (20.5 s), but the wrapper exited 1 before finalization because the raw bag and RGB recorder subprocesses could not initialize.
- Status evidence: `isaac_runtime_status.json` reports `observed_rgb_frames: 0`, zero LiDAR clouds, and no produced image stream. No MP4, bag, or saved frame exists.
- Policy evidence: `rgb.err.log` reports Application Control blocking OpenCV's native dependency; `bag.err.log` and `zenoh.err.log` report Application Control blocking the Pixi `rclpy` extension. The existing setup report already records the policy issue. The original raw logs remain in the ignored `runs/20260922-225811631/logs/` directory. No policy change, bypass, or package reinstall was attempted.
- This run is failure evidence only, not visual evidence or a successful capture. The scene visibility check remains open.
