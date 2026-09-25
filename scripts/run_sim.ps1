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
  [switch]$Headless,
  [ValidateSet("RaytracedLighting", "PathTracing")]
  [string]$Renderer = "RaytracedLighting",
  [string]$CaptureDir = "",
  [string]$CaptureFrames = "",
  [ValidateRange(64, 3840)]
  [int]$CaptureWidth = 1280,
  [ValidateRange(64, 3840)]
  [int]$CaptureHeight = 720,
  [ValidateRange(1, 32)]
  [int]$CaptureRtSubframes = 4,
  [switch]$CaptureOnly,
  [switch]$RequireSensorSamples
)
$ErrorActionPreference = "Stop"
if ($Gui -and $Headless) { throw "Choose either -Gui or -Headless, not both." }
if ([bool]$CaptureDir -ne [bool]$CaptureFrames) { throw "-CaptureDir and -CaptureFrames must be supplied together." }
$pathHelper = Join-Path $PSScriptRoot "resolve_runtime_paths.ps1"
. $pathHelper
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\")).Path
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
$pixiWorkspace = Resolve-RosWorkspace $RosWorkspace
$pixi = Resolve-PixiExecutable $PixiPath
Remove-Item Env:ROS_DISTRO -ErrorAction SilentlyContinue
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"
$env:ROS_LOG_DIR = Join-Path $repo "runs\ros_logs"
New-Item -ItemType Directory -Force -Path $env:ROS_LOG_DIR | Out-Null
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
$status = if ($StatusPath) { $StatusPath } else { Join-Path $repo "runs\isaac_runtime_status.json" }

function Stop-ProcessTree([int]$RootPid) {
  & taskkill.exe /PID $RootPid /T /F 2>$null | Out-Null
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
if ($StartZenohRouter) {
  $routerLogBase = [IO.Path]::GetFullPath("$status.zenoh")
  $routerLogParent = Split-Path -Parent $routerLogBase
  New-Item -ItemType Directory -Force -Path $routerLogParent | Out-Null
  $zenohRouter = Start-Process -FilePath $pixi -ArgumentList @("run", "--manifest-path", (Join-Path $pixiWorkspace "pixi.toml"), "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") -WorkingDirectory $pixiWorkspace -WindowStyle Hidden -RedirectStandardOutput "$routerLogBase.stdout.log" -RedirectStandardError "$routerLogBase.stderr.log" -PassThru
  Start-Sleep -Seconds 6
  if ($zenohRouter.HasExited) { throw "Zenoh router exited during startup; see $routerLogBase.stderr.log" }
}

$runtime = Join-Path $repo "simulator\runtime\isaac_sim_runner.py"
$arguments = @($runtime, "--scenario", $scenarioPath, "--status-path", $status, "--renderer", $Renderer)
if ($Frames -gt 0) { $arguments += @("--frames", $Frames) }
if ($Headless) { $arguments += "--headless" }
if ($Realtime) { $arguments += "--realtime" }
if ($CaptureOnly) { $arguments += "--capture-only" }
if ($CaptureDir) {
  $capturePathCandidate = if ([IO.Path]::IsPathRooted($CaptureDir)) { $CaptureDir } else { Join-Path $repo $CaptureDir }
  $resolvedCaptureDir = [IO.Path]::GetFullPath($capturePathCandidate)
  $arguments += @(
    "--capture-dir", $resolvedCaptureDir,
    "--capture-frames", $CaptureFrames,
    "--capture-width", ([string]$CaptureWidth),
    "--capture-height", ([string]$CaptureHeight),
    "--capture-rt-subframes", ([string]$CaptureRtSubframes)
  )
}
Push-Location $repo
$isaacExitCode = 0
try {
  & $IsaacPython @arguments
  $isaacExitCode = $LASTEXITCODE
} finally {
  Pop-Location
  if ($zenohRouter -and -not $zenohRouter.HasExited) { Stop-ProcessTree -RootPid $zenohRouter.Id }
}
if ($isaacExitCode -ne 0) { throw "Isaac Sim runtime exited with code $isaacExitCode" }
if (-not (Test-Path -LiteralPath $status)) { throw "Isaac Sim runtime did not write status: $status" }
$runtimeStatus = Get-Content -LiteralPath $status -Raw | ConvertFrom-Json
if ($runtimeStatus.status -eq "error") { throw "Isaac Sim runtime reported $($runtimeStatus.error_type): $($runtimeStatus.error)" }
if ($RequireSensorSamples) {
  if ($runtimeStatus.observed_rgb_frames -le 0) { throw "Isaac Sim produced no observed RGB frames." }
  if ($runtimeStatus.observed_clock_samples -le 0) { throw "Isaac Sim produced no observed /clock messages." }
  if ($runtimeStatus.observed_lidar_clouds -le 0) { throw "Isaac Sim produced no observed RTX LiDAR clouds." }
  if ($runtimeStatus.lidar_cloud_points.min_points -le 0) { throw "Isaac Sim RTX LiDAR clouds contained no real returns." }
}
