$ErrorActionPreference = "Stop"
$pixi = "C:\Users\suyog\AppData\Local\pixi\bin\pixi.exe"
$workspace = "C:\IsaacSim-ros_workspaces\jazzy_ws"
if (-not (Test-Path -LiteralPath $pixi)) { throw "Pixi launcher not found: $pixi" }
if (-not (Test-Path -LiteralPath (Join-Path $workspace "pixi.toml"))) { throw "Pixi ROS workspace not found: $workspace" }
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"
$pixiArgs = @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "ros2")
$rtabmapPrefix = $null
$prefixExit = 0
try {
  $rtabmapPrefix = & $pixi @pixiArgs pkg prefix rtabmap_odom 2>$null
  $prefixExit = $LASTEXITCODE
} catch {
  $prefixExit = 1
}
if ($prefixExit -ne 0) {
  throw "RTAB-Map ROS 2 binaries are not installed in $workspace. Build the checked-out rtabmap_ros sources after installing Visual Studio Build Tools 2022 Desktop C++ and the Windows SDK."
}
$pixiArgs += @("launch", "grocery_sim_mapping", "rtabmap_lidar.launch.py", "use_sim_time:=true")
& $pixi @pixiArgs
if ($LASTEXITCODE -ne 0) { throw "ROS 2 mapping launch exited with code $LASTEXITCODE" }
