param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml",
  [int]$Frames = 0,
  [int]$Port = 8080,
  [switch]$Gui,
  [switch]$Realtime,
  [switch]$Fast,
  [switch]$NoZenohRouter
)
$ErrorActionPreference = "Stop"
if ($Realtime -and $Fast) { throw "Choose either -Realtime or -Fast, not both." }
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } elseif (Test-Path -LiteralPath (Join-Path $repo $Scenario)) { Join-Path $repo $Scenario } else { Join-Path $repo ("config\scenarios\" + [IO.Path]::GetFileNameWithoutExtension($Scenario) + ".yaml") }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
$scenarioPath = (Resolve-Path -LiteralPath $scenarioPath).Path
if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
$scenarioData = Get-Content -LiteralPath $scenarioPath -Raw | ConvertFrom-Json
$trajectoryPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path -Parent $scenarioPath) $scenarioData.trajectory)).Path
$trajectoryData = Get-Content -LiteralPath $trajectoryPath -Raw | ConvertFrom-Json
$collectorDuration = [int][math]::Ceiling([double]$trajectoryData.duration_s + 5.0)
$collectorStartupTimeout = 90

$runId = Get-Date -Format "yyyyMMdd-HHmmssfff"
$runDir = Join-Path $repo (Join-Path "runs" $runId)
$logDir = Join-Path $runDir "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$isaacStatusPath = Join-Path $runDir "isaac_runtime_status.json"
$env:RMW_IMPLEMENTATION = if ($env:RMW_IMPLEMENTATION) { $env:RMW_IMPLEMENTATION } else { "rmw_zenoh_cpp" }
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }

function Get-DescendantPids([int]$ParentPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $ParentPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($child in $children) {
    $child
    Get-DescendantPids -ParentPid ([int]$child)
  }
}

function Stop-ProcessTree([int]$RootPid) {
  $descendants = @(Get-DescendantPids -ParentPid $RootPid | Sort-Object -Descending)
  foreach ($childPid in $descendants) { Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

function Start-LoggedProcess([string]$Name, [string]$FilePath, [string[]]$Arguments, [string]$WorkingDirectory) {
  $out = Join-Path $logDir "$Name.out.log"
  $err = Join-Path $logDir "$Name.err.log"
  $process = Start-Process -FilePath $FilePath -ArgumentList $Arguments -WorkingDirectory $WorkingDirectory -WindowStyle Hidden -RedirectStandardOutput $out -RedirectStandardError $err -PassThru
  return [pscustomobject]@{ Name = $Name; Process = $process }
}

function Assert-Running($service, [string]$Name) {
  if ($null -eq $service -or $null -eq $service.Process -or $service.Process.HasExited) {
    throw "$Name exited before the consolidated run completed; see $logDir\$Name.err.log"
  }
}

$services = @()
try {
  if (-not $NoZenohRouter) {
    $services += Start-LoggedProcess "zenoh" $pixi @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd") $workspace
    Start-Sleep -Seconds 2
  }

  $scenarioName = [IO.Path]::GetFileNameWithoutExtension($scenarioPath)
  $dashboardArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_dashboard.ps1"), "-PixiPath", $pixi, "-RosWorkspace", $workspace, "-Port", "$Port")
  $mappingArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_mapping.ps1"), "-PixiPath", $pixi, "-RosWorkspace", $workspace, "-RunId", $runId, "-Scenario", $scenarioPath)
  $collectorArgs = @("run", "--manifest-path", (Join-Path $workspace "pixi.toml"), "python", "-m", "evaluation.ros_collector", "--run-dir", $runDir, "--scenario", $scenarioName, "--duration-seconds", "$collectorDuration", "--startup-timeout-seconds", "$collectorStartupTimeout")
  $services += Start-LoggedProcess "dashboard" "powershell.exe" $dashboardArgs $repo
  $services += Start-LoggedProcess "mapping" "powershell.exe" $mappingArgs $repo
  $services += Start-LoggedProcess "collector" $pixi $collectorArgs $repo

  $health = $false
  for ($i = 0; $i -lt 30; $i++) {
    Assert-Running (@($services | Where-Object Name -eq "dashboard")[0]) "dashboard"
    Assert-Running (@($services | Where-Object Name -eq "mapping")[0]) "mapping"
    Assert-Running (@($services | Where-Object Name -eq "collector")[0]) "collector"
    try {
      $response = Invoke-WebRequest -UseBasicParsing -Uri "http://127.0.0.1:$Port/api/health" -TimeoutSec 2
      if ($response.StatusCode -eq 200) { $health = $true; break }
    } catch { Start-Sleep -Milliseconds 500 }
  }
  if (-not $health) { throw "Dashboard health endpoint did not become ready; see $logDir\dashboard.err.log" }

  $simArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", (Join-Path $repo "scripts\run_sim.ps1"), "-IsaacPython", $IsaacPython, "-Scenario", $scenarioPath, "-PixiPath", $pixi, "-RosWorkspace", $workspace, "-StatusPath", $isaacStatusPath)
  if ($Frames -gt 0) { $simArgs += @("-Frames", "$Frames") }
  if (-not $Fast) { $simArgs += "-Realtime" }
  if ($Gui) { $simArgs += "-Gui" } else { $simArgs += "-Headless" }
  $services += Start-LoggedProcess "isaac" "powershell.exe" $simArgs $repo
  $isaac = @($services | Where-Object Name -eq "isaac")[0]
  Wait-Process -Id $isaac.Process.Id
  $isaac.Process.WaitForExit()
  $isaac.Process.Refresh()
  $isaacExitCode = $isaac.Process.ExitCode
  if ($null -eq $isaacExitCode -and (Test-Path -LiteralPath $isaacStatusPath)) {
    $isaacStatus = Get-Content -LiteralPath $isaacStatusPath -Raw | ConvertFrom-Json
    if ($isaacStatus.runtime -eq "isaac_sim" -and $isaacStatus.status -ne "error" -and $isaacStatus.frames_simulated -gt 0) { $isaacExitCode = 0 }
  }
  if ($isaacExitCode -ne 0) { throw "Isaac runtime exited with code $isaacExitCode; see $logDir\isaac.err.log" }

  Assert-Running (@($services | Where-Object Name -eq "dashboard")[0]) "dashboard"
  Assert-Running (@($services | Where-Object Name -eq "mapping")[0]) "mapping"
  $collector = @($services | Where-Object Name -eq "collector")[0]
  Wait-Process -Id $collector.Process.Id -Timeout ($collectorDuration + $collectorStartupTimeout + 15) -ErrorAction SilentlyContinue
  if (-not $collector.Process.HasExited) { throw "collector did not finish within the scenario-derived deadline; see $logDir\collector.err.log" }
  $collector.Process.Refresh()
  if ($null -ne $collector.Process.ExitCode -and $collector.Process.ExitCode -ne 0) { throw "collector exited with code $($collector.Process.ExitCode); see $logDir\collector.err.log" }
  if (-not (Test-Path -LiteralPath $isaacStatusPath)) { throw "Isaac runtime status was not written: $isaacStatusPath" }

  $dashboardStatus = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/api/status" -TimeoutSec 5
  if ([int]$dashboardStatus.map_point_count -le 0) { throw "No /slam/map_cloud data was observed; refusing to claim a successful mapping run." }

  $requiredArtifacts = @("metadata.json", "metrics.json", "ground_truth.csv", "estimate.csv", "rtabmap.db")
  foreach ($artifact in $requiredArtifacts) {
    $path = Join-Path $runDir $artifact
    if (-not (Test-Path -LiteralPath $path)) { throw "Required run artifact missing: $path" }
  }
  $metrics = Get-Content -LiteralPath (Join-Path $runDir "metrics.json") -Raw | ConvertFrom-Json
  if ($metrics.status -eq "insufficient_samples" -or [int]$metrics.sample_count -lt 2) { throw "Evaluation metrics are incomplete; see $runDir\metrics.json" }
  if ($metrics.alignment_policy -ne "initial_se3") { throw "Live metrics did not record initial_se3 alignment." }

  Write-Host "Run complete: $runId"
  Write-Host "Artifacts: $runDir"
  Write-Host "Dashboard: http://127.0.0.1:$Port"
} finally {
  foreach ($service in $services) {
    if ($service.Process -and -not $service.Process.HasExited) { Stop-ProcessTree -RootPid $service.Process.Id }
  }
}
