param(
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml",
  [int]$Frames = 3600,
  [switch]$Realtime,
  [switch]$PreflightOnly,
  [switch]$StartZenohRouter
)
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\")).Path
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
if ($PreflightOnly) {
  & "C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe" -m simulator.runtime.sim_runner --scenario $scenarioPath
  exit $LASTEXITCODE
}
$pixiWorkspace = "C:\IsaacSim-ros_workspaces\jazzy_ws"
$pixi = "C:\Users\suyog\AppData\Local\pixi\bin\pixi.exe"
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
if ($StartZenohRouter) {
  if (-not (Test-Path -LiteralPath $pixi)) { throw "Pixi launcher not found: $pixi" }
  Start-Process -FilePath $pixi -ArgumentList @("run", "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") -WorkingDirectory $pixiWorkspace -WindowStyle Hidden | Out-Null
  Start-Sleep -Seconds 2
}

$runtime = Join-Path $repo "simulator\runtime\isaac_sim_runner.py"
$status = Join-Path $repo "runs\isaac_runtime_status.json"
$arguments = @($runtime, "--scenario", $scenarioPath, "--frames", $Frames, "--headless", "--status-path", $status)
if ($Realtime) { $arguments += "--realtime" }
& $IsaacPython @arguments
if ($LASTEXITCODE -ne 0) { throw "Isaac Sim runtime exited with code $LASTEXITCODE" }
