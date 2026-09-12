param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$RunId = "",
  [string]$Scenario = "baseline_straight"
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runId = if ($RunId) { $RunId } else { Get-Date -Format "yyyyMMdd-HHmmss" }
$runDir = Join-Path $repo (Join-Path "runs" $runId)
New-Item -ItemType Directory -Force -Path $runDir | Out-Null
$database = Join-Path $runDir "rtabmap.db"
[ordered]@{
  run_id = $runId
  scenario = $Scenario
  database_path = $database
  topics = @{ lidar = "/sim/lidar/points"; odom = "/slam/odom"; map = "/slam/map_cloud" }
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $runDir "mapping_metadata.json") -Encoding UTF8
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
$pixiArgs += @("launch", "grocery_sim_mapping", "rtabmap_lidar.launch.py", "use_sim_time:=true", "database_path:=$database")
& $pixi @pixiArgs
if ($LASTEXITCODE -ne 0) { throw "ROS 2 mapping launch exited with code $LASTEXITCODE" }
