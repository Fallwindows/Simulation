param(
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml",
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [int]$Frames = 3600,
  [switch]$Realtime,
  [switch]$PreflightOnly,
  [switch]$StartZenohRouter,
  [switch]$Headless
)
$ErrorActionPreference = "Stop"
$pathHelper = Join-Path $PSScriptRoot "resolve_runtime_paths.ps1"
. $pathHelper
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\")).Path
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
$pixiWorkspace = Resolve-RosWorkspace $RosWorkspace
$pixi = Resolve-PixiExecutable $PixiPath
if ($PreflightOnly) {
  Push-Location $repo
  try {
    & $pixi run --manifest-path (Join-Path $pixiWorkspace "pixi.toml") python -m simulator.runtime.sim_runner --scenario $scenarioPath
  } finally {
    Pop-Location
  }
  exit $LASTEXITCODE
}
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
if ($StartZenohRouter) {
  Start-Process -FilePath $pixi -ArgumentList @("run", "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") -WorkingDirectory $pixiWorkspace -WindowStyle Hidden | Out-Null
  Start-Sleep -Seconds 2
}

$runtime = Join-Path $repo "simulator\runtime\isaac_sim_runner.py"
$status = Join-Path $repo "runs\isaac_runtime_status.json"
$arguments = @($runtime, "--scenario", $scenarioPath, "--frames", $Frames, "--status-path", $status)
if ($Headless) { $arguments += "--headless" }
if ($Realtime) { $arguments += "--realtime" }
Push-Location $repo
try {
  & $IsaacPython @arguments
} finally {
  Pop-Location
}
if ($LASTEXITCODE -ne 0) { throw "Isaac Sim runtime exited with code $LASTEXITCODE" }
