param(
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/walking_baseline.yaml",
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [int]$Frames = 0,
  [switch]$Realtime,
  [string]$StatusPath = "",
  [switch]$PreflightOnly,
  [switch]$StartZenohRouter,
  [switch]$Gui,
  [switch]$Headless
)
$ErrorActionPreference = "Stop"
if ($Gui -and $Headless) { throw "Choose either -Gui or -Headless, not both." }
$pathHelper = Join-Path $PSScriptRoot "resolve_runtime_paths.ps1"
. $pathHelper
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\")).Path
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
$pixiWorkspace = Resolve-RosWorkspace $RosWorkspace
$pixi = Resolve-PixiExecutable $PixiPath
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"
if ($PreflightOnly) {
  Push-Location $repo
  try {
    & $pixi run --manifest-path (Join-Path $pixiWorkspace "pixi.toml") python -m simulator.runtime.sim_runner --scenario $scenarioPath
  } finally {
    Pop-Location
  }
  exit $LASTEXITCODE
}
$zenohRouter = $null

function Stop-ProcessTree([int]$RootPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $RootPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($childPid in $children) {
    Stop-ProcessTree -RootPid ([int]$childPid)
    Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue
  }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
if ($StartZenohRouter) {
  $zenohRouter = Start-Process -FilePath $pixi -ArgumentList @("run", "--manifest-path", (Join-Path $pixiWorkspace "pixi.toml"), "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") -WorkingDirectory $pixiWorkspace -WindowStyle Hidden -PassThru
  Start-Sleep -Seconds 2
}

$runtime = Join-Path $repo "simulator\runtime\isaac_sim_runner.py"
$status = if ($StatusPath) { $StatusPath } else { Join-Path $repo "runs\isaac_runtime_status.json" }
$arguments = @($runtime, "--scenario", $scenarioPath, "--status-path", $status)
if ($Frames -gt 0) { $arguments += @("--frames", $Frames) }
if ($Headless) { $arguments += "--headless" }
if ($Realtime) { $arguments += "--realtime" }
Push-Location $repo
try {
  & $IsaacPython @arguments
} finally {
  Pop-Location
  if ($zenohRouter -and -not $zenohRouter.HasExited) { Stop-ProcessTree -RootPid $zenohRouter.Id }
}
if ($LASTEXITCODE -ne 0) { throw "Isaac Sim runtime exited with code $LASTEXITCODE" }
