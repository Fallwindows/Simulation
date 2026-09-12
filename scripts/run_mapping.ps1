$ErrorActionPreference = "Stop"
$ros2 = Get-Command ros2 -ErrorAction SilentlyContinue
if (-not $ros2) { throw "ROS 2 is not installed or sourced on PATH. Run this script on the supported ROS 2 environment." }
ros2 launch grocery_sim_mapping rtabmap_lidar.launch.py use_sim_time:=true
