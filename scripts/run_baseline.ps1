param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml",
  [int]$Frames = 0,
  [int]$Port = 8080,
  [switch]$Gui,
  [switch]$Realtime,
  [switch]$NoZenohRouter
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }

$runId = Get-Date -Format "yyyyMMdd-HHmmss"
$runDir = Join-Path $repo (Join-Path "runs" $runId)
$logDir = Join-Path $runDir "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$isaacStatusPath = Join-Path $repo "runs\isaac_runtime_status.json"
if (Test-Path -LiteralPath $isaacStatusPath) {
  Remove-Item -LiteralPath $isaacStatusPath -Force
}
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

function Get-DescendantPids([int]$ParentPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $ParentPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($child in $children) {
    $child
    Get-DescendantPids -ParentPid ([int]$child)
  }
}

function Stop-ProcessTree([int]$RootPid) {
  $descendants = @(Get-DescendantPids -ParentPid $RootPid | Sort-Object -Descending)
  foreach ($childPid in $descendants) {
    Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue
  }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

function Start-LoggedProcess([string]$Name, [string]$FilePath, [string[]]$Arguments, [string]$WorkingDirectory) {
  $out = Join-Path $logDir "$Name.out.log"
  $err = Join-Path $logDir "$Name.err.log"
  $process = Start-Process -FilePath $FilePath -ArgumentList $Arguments -WorkingDirectory $WorkingDirectory -WindowStyle Hidden -RedirectStandardOutput $out -RedirectStandardError $err -PassThru
  return [pscustomobject]@{ Name = $Name; Process = $process }
}

$services = @()
try {
  if (-not $NoZenohRouter) {
    $services += Start-LoggedProcess "zenoh" $pixi @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") $workspace
    Start-Sleep -Seconds 2
  }

  $dashboardArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_dashboard.ps1"), "-PixiPath", $pixi, "-RosWorkspace", $workspace, "-Port", "$Port")
  $mappingArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_mapping.ps1"), "-PixiPath", $pixi, "-RosWorkspace", $workspace, "-RunId", $runId, "-Scenario", ([IO.Path]::GetFileNameWithoutExtension($scenarioPath)))
  $collectorArgs = @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "python", "-m", "evaluation.ros_collector", "--run-dir", $runDir, "--scenario", ([IO.Path]::GetFileNameWithoutExtension($scenarioPath)), "--duration-seconds", "40")
  $services += Start-LoggedProcess "dashboard" "powershell.exe" $dashboardArgs $repo
  $services += Start-LoggedProcess "mapping" "powershell.exe" $mappingArgs $repo
  $services += Start-LoggedProcess "collector" $pixi $collectorArgs $repo

  $health = $false
  for ($i = 0; $i -lt 30; $i++) {
    try {
      $response = Invoke-WebRequest -UseBasicParsing -Uri "http://127.0.0.1:$Port/api/health" -TimeoutSec 2
      if ($response.StatusCode -eq 200) { $health = $true; break }
    } catch { Start-Sleep -Milliseconds 500 }
  }
  if (-not $health) { throw "Dashboard health endpoint did not become ready; see $logDir\dashboard.err.log" }

  $simArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_sim.ps1"), "-IsaacPython", $IsaacPython, "-Scenario", $scenarioPath, "-PixiPath", $pixi, "-RosWorkspace", $workspace)
  if ($Frames -gt 0) { $simArgs += @("-Frames", "$Frames") }
  if ($Realtime) { $simArgs += "-Realtime" }
  if (-not $Gui) { $simArgs += "-Headless" }
  $services += Start-LoggedProcess "isaac" "powershell.exe" $simArgs $repo
  $isaac = $services | Where-Object Name -eq "isaac" | Select-Object -ExpandProperty Process
  Wait-Process -Id $isaac.Id
  $isaac.WaitForExit()
  $isaac.Refresh()
  $isaacExitCode = $isaac.ExitCode
  if ($null -eq $isaacExitCode -and (Test-Path -LiteralPath $isaacStatusPath)) {
    $isaacStatus = Get-Content -LiteralPath $isaacStatusPath -Raw | ConvertFrom-Json
    if ($isaacStatus.runtime -eq "isaac_sim" -and $isaacStatus.status -ne "error" -and $isaacStatus.frames_simulated -gt 0) {
      $isaacExitCode = 0
    }
  }
  if ($isaacExitCode -ne 0) { throw "Isaac runtime exited with code $isaacExitCode; see $logDir\isaac.err.log" }

  $collector = $services | Where-Object Name -eq "collector" | Select-Object -ExpandProperty Process
  Wait-Process -Id $collector.Id -Timeout 55 -ErrorAction SilentlyContinue
  if (-not (Test-Path -LiteralPath $isaacStatusPath)) { throw "Isaac runtime status was not written: $isaacStatusPath" }
  Write-Host "Run complete: $runId"
  Write-Host "Artifacts: $runDir"
  Write-Host "Dashboard: http://127.0.0.1:$Port"
} finally {
  foreach ($service in $services) {
    if ($service.Process -and -not $service.Process.HasExited) {
      Stop-ProcessTree -RootPid $service.Process.Id
    }
  }
}
