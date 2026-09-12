param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$RunId = "",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml"
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) {
  $Scenario
} elseif (Test-Path -LiteralPath (Join-Path $repo $Scenario)) {
  Join-Path $repo $Scenario
} else {
  Join-Path $repo ("config\scenarios\" + [IO.Path]::GetFileNameWithoutExtension($Scenario) + ".yaml")
}
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
$scenarioPath = (Resolve-Path -LiteralPath $scenarioPath).Path
$scenarioData = Get-Content -LiteralPath $scenarioPath -Raw | ConvertFrom-Json
$scenarioDir = Split-Path -Parent $scenarioPath
$mappingPath = (Resolve-Path -LiteralPath (Join-Path $scenarioDir $scenarioData.mapping)).Path
$contractPath = Join-Path $repo "config\contracts.yaml"
$contractData = Get-Content -LiteralPath $contractPath -Raw | ConvertFrom-Json
$runId = if ($RunId) { $RunId } else { Get-Date -Format "yyyyMMdd-HHmmssfff" }
$runDir = Join-Path $repo (Join-Path "runs" $runId)
New-Item -ItemType Directory -Force -Path $runDir | Out-Null
$database = Join-Path $runDir "rtabmap.db"
[ordered]@{
  run_id = $runId
  scenario = [IO.Path]::GetFileNameWithoutExtension($scenarioPath)
  scenario_path = $scenarioPath
  mapping_params_path = $mappingPath
  database_path = $database
  topics = @{ lidar = $contractData.topics.lidar_points; odom = $contractData.topics.estimated_odom; map = $contractData.topics.map_points }
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $runDir "mapping_metadata.json") -Encoding UTF8
$env:RMW_IMPLEMENTATION = if ($env:RMW_IMPLEMENTATION) { $env:RMW_IMPLEMENTATION } else { "rmw_zenoh_cpp" }
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
$pixiBaseArgs = @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "ros2")

foreach ($packageName in @("rtabmap_odom", "rtabmap_slam", "grocery_sim_mapping")) {
  $prefix = & $pixi @($pixiBaseArgs + @("pkg", "prefix", $packageName)) 2>$null
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace(($prefix -join ""))) {
    throw "Required ROS 2 package '$packageName' is not installed in $workspace."
  }
}

$launchArgs = @(
  "launch", "grocery_sim_mapping", "rtabmap_lidar.launch.py",
  "use_sim_time:=true",
  "database_path:=$database",
  "mapping_params_path:=$mappingPath"
)
& $pixi @($pixiBaseArgs + $launchArgs)
if ($LASTEXITCODE -ne 0) { throw "ROS 2 mapping launch exited with code $LASTEXITCODE" }
