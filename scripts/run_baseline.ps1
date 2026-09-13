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
$environmentPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path -Parent $scenarioPath) $scenarioData.environment)).Path
$sensorPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path -Parent $scenarioPath) $scenarioData.sensors)).Path
$mappingPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path -Parent $scenarioPath) $scenarioData.mapping)).Path
$contractPath = Join-Path $repo "config\contracts.yaml"
$trajectoryData = Get-Content -LiteralPath $trajectoryPath -Raw | ConvertFrom-Json
$environmentData = Get-Content -LiteralPath $environmentPath -Raw | ConvertFrom-Json
$sensorData = Get-Content -LiteralPath $sensorPath -Raw | ConvertFrom-Json
$mappingData = Get-Content -LiteralPath $mappingPath -Raw | ConvertFrom-Json
$contractData = Get-Content -LiteralPath $contractPath -Raw | ConvertFrom-Json
$collectorDuration = [double]$trajectoryData.duration_s
$collectorStartupTimeout = 90
$isWalkingScenario = [IO.Path]::GetFileNameWithoutExtension($scenarioPath) -eq "walking_baseline"

$runId = Get-Date -Format "yyyyMMdd-HHmmssfff"
$runDir = Join-Path $repo (Join-Path "runs" $runId)
$logDir = Join-Path $runDir "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$isaacStatusPath = Join-Path $runDir "isaac_runtime_status.json"
$rgbVideoPath = Join-Path $runDir "rgb_camera.mp4"
$rgbVideoMetadataPath = Join-Path $runDir "rgb_camera_video.json"
$manifestPath = Join-Path $runDir "run_manifest.json"
$effectiveConfigPath = Join-Path $runDir "effective_config.json"
$pixiManifest = Join-Path $workspace "pixi.toml"
# The consolidated run owns these values; every child process inherits them.
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

$gitSha = (& git -C $repo rev-parse HEAD 2>$null | Select-Object -First 1).ToString().Trim()
$pixiVersion = (& $pixi --version 2>$null) -join " "
$rosCliPrefix = (& $pixi run --manifest-path $pixiManifest ros2 pkg prefix ros2cli 2>$null) -join " "
$rosVersion = "ROS_DISTRO=jazzy; ros2cli_prefix=$rosCliPrefix"
$pythonVersion = (& $pixi run --manifest-path $pixiManifest python --version 2>$null) -join " "
$effectiveConfig = [ordered]@{
  scenario = $scenarioData
  environment = $environmentData
  sensors = $sensorData
  noise = if ($null -ne $scenarioData.sensor_overrides) { $scenarioData.sensor_overrides.noise } else { $null }
  trajectory = $trajectoryData
  mapping = $mappingData
  contract = $contractData
}
$effectiveConfig | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath $effectiveConfigPath -Encoding UTF8
Copy-Item -LiteralPath $scenarioPath -Destination (Join-Path $runDir "scenario.yaml") -Force
Copy-Item -LiteralPath $trajectoryPath -Destination (Join-Path $runDir "trajectory.yaml") -Force
Copy-Item -LiteralPath $environmentPath -Destination (Join-Path $runDir "environment.yaml") -Force
Copy-Item -LiteralPath $sensorPath -Destination (Join-Path $runDir "sensors.yaml") -Force
Copy-Item -LiteralPath $mappingPath -Destination (Join-Path $runDir "mapping.yaml") -Force
Copy-Item -LiteralPath $contractPath -Destination (Join-Path $runDir "contracts.yaml") -Force

$runManifest = [ordered]@{
  status = "running"
  run_id = $runId
  created_utc = [DateTime]::UtcNow.ToString("o")
  git_sha = $gitSha
  scenario = $scenarioPath
  effective_config = $effectiveConfigPath
  software = [ordered]@{
    os = [System.Environment]::OSVersion.Version.ToString()
    pixi = $pixiVersion
    ros2 = $rosVersion
    python = $pythonVersion
    rmw_implementation = $env:RMW_IMPLEMENTATION
    ros_domain_id = $env:ROS_DOMAIN_ID
    isaac_python = $IsaacPython
    ros_workspace = $workspace
    pixi_manifest = $pixiManifest
  }
}
function Save-RunManifest {
  $runManifest | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
}
function Write-LauncherProgress([string]$Message) {
  Add-Content -LiteralPath (Join-Path $runDir "launcher_progress.log") -Value ("{0} {1}" -f [DateTime]::UtcNow.ToString("o"), $Message)
}
Save-RunManifest

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

function Wait-TrackedProcess($service, [int]$TimeoutSeconds, [string]$Name) {
  Write-LauncherProgress "wait_${Name}_start"
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while ($true) {
    $service.Process.Refresh()
    if ($service.Process.HasExited) {
      Write-LauncherProgress "wait_${Name}_done"
      return
    }
    if ((Get-Date) -ge $deadline) { throw "$Name did not exit within $TimeoutSeconds seconds; see $logDir\$Name.err.log" }
    Start-Sleep -Milliseconds 500
  }
}

$pixiBaseArgs = @("run", "--manifest-path", $pixiManifest, "ros2")
function Invoke-RosGraphQuery([string]$Name, [string[]]$Arguments) {
  $queryOut = Join-Path $logDir "ros_graph_$Name.out.log"
  $queryErr = Join-Path $logDir "ros_graph_$Name.err.log"
  $queryProcess = Start-Process -FilePath $pixi -ArgumentList @($pixiBaseArgs + $Arguments) -WorkingDirectory $workspace -WindowStyle Hidden -RedirectStandardOutput $queryOut -RedirectStandardError $queryErr -PassThru
  if (-not $queryProcess.WaitForExit(15000)) {
    Stop-Process -Id $queryProcess.Id -Force -ErrorAction SilentlyContinue
    return ""
  }
  $queryProcess.Refresh()
  for ($readAttempt = 0; $readAttempt -lt 25; $readAttempt++) {
    if ((Test-Path -LiteralPath $queryOut) -and ((Get-Item -LiteralPath $queryOut).Length -gt 0)) {
      $queryText = Get-Content -LiteralPath $queryOut -Raw -ErrorAction SilentlyContinue
      if (-not [string]::IsNullOrWhiteSpace($queryText)) { return $queryText }
    }
    Start-Sleep -Milliseconds 200
  }
  return ""
}

function Assert-RosGraphReady($IsaacService) {
  $requiredNodes = @("icp_odometry", "rtabmap")
  $requiredTopics = @("/clock", "/sim/camera/rgb/image_raw", "/sim/lidar/points", "/slam/odom", "/slam/map_cloud", "/tf", "/tf_static")
  for ($i = 0; $i -lt 45; $i++) {
    $IsaacService.Process.Refresh()
    $isaacExited = $IsaacService.Process.HasExited
    $nodes = Invoke-RosGraphQuery "nodes" @("node", "list")
    $topics = Invoke-RosGraphQuery "topics" @("topic", "list")
    $nodesReady = ($requiredNodes | Where-Object { $nodes -match [regex]::Escape($_) }).Count -eq $requiredNodes.Count
    $topicsReady = ($requiredTopics | Where-Object { $topics -match [regex]::Escape($_) }).Count -eq $requiredTopics.Count
    if ($nodesReady -and $topicsReady) { return }
    if ($isaacExited) {
      # Isaac owns /clock and the sensor topics, and the graph CLI can race
      # the final simulator update.  Defer the RTAB node assertion until the
      # collector has completed, while the mapping process is still alive.
      Write-LauncherProgress "isaac_exited_graph_check_deferred"
      return
    }
    Start-Sleep -Seconds 1
  }
  throw "ROS graph did not expose the required RTAB-Map nodes/topics within 45 seconds."
}

function Assert-RosNodesReady {
  $requiredNodes = @("icp_odometry", "rtabmap")
  for ($i = 0; $i -lt 10; $i++) {
    $nodes = Invoke-RosGraphQuery "nodes_final" @("node", "list")
    $nodesReady = ($requiredNodes | Where-Object { $nodes -match [regex]::Escape($_) }).Count -eq $requiredNodes.Count
    if ($nodesReady) {
      Write-LauncherProgress "rtab_nodes_fresh"
      return
    }
    Start-Sleep -Seconds 1
  }
  throw "Required RTAB-Map ROS nodes were not fresh after collector completion; see $logDir\ros_graph_nodes_final.out.log"
}

function Invoke-RosEmptyService([string]$ServiceName, [string]$LogStem) {
  $services = (& $pixi @($pixiBaseArgs + @("service", "list")) 2>$null) -join "`n"
  if ($services -notmatch [regex]::Escape($ServiceName)) { throw "RTAB-Map service '$ServiceName' was not advertised; refusing to claim graceful shutdown." }
  $serviceOut = Join-Path $logDir "$LogStem.out.log"
  $serviceErr = Join-Path $logDir "$LogStem.err.log"
  $serviceArgs = @("run", "--manifest-path", $pixiManifest, "ros2", "service", "call", $ServiceName, "std_srvs/srv/Empty", "{}")
  $serviceProcess = Start-Process -FilePath $pixi -ArgumentList $serviceArgs -WorkingDirectory $workspace -WindowStyle Hidden -RedirectStandardOutput $serviceOut -RedirectStandardError $serviceErr -PassThru
  if (-not $serviceProcess.WaitForExit(30000)) {
    Stop-Process -Id $serviceProcess.Id -Force -ErrorAction SilentlyContinue
    throw "RTAB-Map service '$ServiceName' did not return within 30 seconds; see $serviceErr"
  }
  $serviceProcess.Refresh()
  $serviceText = Get-Content -LiteralPath $serviceOut -Raw -ErrorAction SilentlyContinue
  if ($null -ne $serviceProcess.ExitCode -and $serviceProcess.ExitCode -ne 0) { throw "RTAB-Map service '$ServiceName' failed with code $($serviceProcess.ExitCode); see $serviceErr" }
  if ($null -eq $serviceProcess.ExitCode -and $serviceText -notmatch "response:") { throw "RTAB-Map service '$ServiceName' did not return a recognizable response; see $serviceErr" }
}

function Invoke-RtabmapBackup {
  Invoke-RosEmptyService "/rtabmap/pause" "rtabmap_pause"
  Invoke-RosEmptyService "/rtabmap/backup" "rtabmap_backup"
}

function Invoke-RtabmapPublishMap {
  $serviceName = "/rtabmap/publish_map"
  $services = (& $pixi @($pixiBaseArgs + @("service", "list")) 2>$null) -join "`n"
  if ($services -notmatch [regex]::Escape($serviceName)) { throw "RTAB-Map service '$serviceName' was not advertised; refusing to claim a fresh final map." }
  $serviceOut = Join-Path $logDir "rtabmap_publish_map.out.log"
  $serviceErr = Join-Path $logDir "rtabmap_publish_map.err.log"
  $serviceArgs = @("run", "--manifest-path", $pixiManifest, "ros2", "service", "call", $serviceName, "rtabmap_msgs/srv/PublishMap", '"{global_map: true, optimized: true, graph_only: false}"')
  $serviceProcess = Start-Process -FilePath $pixi -ArgumentList $serviceArgs -WorkingDirectory $workspace -WindowStyle Hidden -RedirectStandardOutput $serviceOut -RedirectStandardError $serviceErr -PassThru
  if (-not $serviceProcess.WaitForExit(30000)) {
    Stop-Process -Id $serviceProcess.Id -Force -ErrorAction SilentlyContinue
    throw "RTAB-Map service '$serviceName' did not return within 30 seconds; see $serviceErr"
  }
  $serviceProcess.Refresh()
  $serviceText = Get-Content -LiteralPath $serviceOut -Raw -ErrorAction SilentlyContinue
  if ($null -ne $serviceProcess.ExitCode -and $serviceProcess.ExitCode -ne 0) { throw "RTAB-Map service '$serviceName' failed with code $($serviceProcess.ExitCode); see $serviceErr" }
  if ($null -eq $serviceProcess.ExitCode -and $serviceText -notmatch "response:") { throw "RTAB-Map service '$serviceName' did not return a recognizable response; see $serviceErr" }
}

function Assert-RtabmapDatabase([string]$DatabasePath) {
  if (-not (Test-Path -LiteralPath $DatabasePath)) { throw "RTAB-Map database is missing: $DatabasePath" }
  $databaseFile = Get-Item -LiteralPath $DatabasePath
  if ($databaseFile.Length -le 4096) { throw "RTAB-Map database is unexpectedly small: $($databaseFile.Length) bytes" }
  $validator = Join-Path $repo "scripts\validate_rtabmap_db.py"
  $validatorArgument = '"' + $validator + '"'
  $databaseArgument = '"' + $DatabasePath + '"'
  $checkOut = Join-Path $logDir "rtabmap_db_check.out.log"
  $checkErr = Join-Path $logDir "rtabmap_db_check.err.log"
  $nodeCountText = ""
  for ($attempt = 0; $attempt -lt 10; $attempt++) {
    $checkProcess = Start-Process -FilePath $pixi -ArgumentList @("run", "--manifest-path", $pixiManifest, "python", $validatorArgument, $databaseArgument) -WorkingDirectory $repo -WindowStyle Hidden -RedirectStandardOutput $checkOut -RedirectStandardError $checkErr -PassThru
    if (-not $checkProcess.WaitForExit(30000)) {
      Stop-Process -Id $checkProcess.Id -Force -ErrorAction SilentlyContinue
      $sqliteExitCode = 1
      $nodeCountText = "database validator timed out"
    } else {
      $checkProcess.Refresh()
      $sqliteExitCode = $checkProcess.ExitCode
      $nodeCountText = ((Get-Content -LiteralPath $checkOut -Raw -ErrorAction SilentlyContinue), (Get-Content -LiteralPath $checkErr -Raw -ErrorAction SilentlyContinue) -join "`n").Trim()
    }
    $nodeCountLine = ($nodeCountText -split "`r?`n" | Where-Object { $_ -match '^\s*\d+\s*$' } | Select-Object -Last 1)
    if ($nodeCountLine -and ($sqliteExitCode -eq 0 -or $null -eq $sqliteExitCode)) { return [int]$nodeCountLine.Trim() }
    Start-Sleep -Seconds 1
  }
  throw "RTAB-Map database validation failed after retries: $nodeCountText"
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
  $recorder = $null
  if ($isWalkingScenario) {
    $recorderArgs = @("run", "--manifest-path", $pixiManifest, "python", "-m", "evaluation.rgb_video_recorder", "--output", $rgbVideoPath, "--metadata", $rgbVideoMetadataPath, "--duration-seconds", "$collectorDuration", "--startup-timeout-seconds", "$collectorStartupTimeout")
    $services += Start-LoggedProcess "rgb_recorder" $pixi $recorderArgs $repo
    $recorder = @($services | Where-Object Name -eq "rgb_recorder")[0]
  }

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
  Write-LauncherProgress "isaac_process_started"
  Assert-RosGraphReady $isaac
  Wait-TrackedProcess $isaac 300 "isaac"
  $isaac.Process.Refresh()
  $isaacExitCode = $isaac.Process.ExitCode
  if ($null -eq $isaacExitCode -and (Test-Path -LiteralPath $isaacStatusPath)) {
    $isaacStatus = Get-Content -LiteralPath $isaacStatusPath -Raw | ConvertFrom-Json
    if ($isaacStatus.runtime -eq "isaac_sim" -and $isaacStatus.status -ne "error" -and $isaacStatus.frames_simulated -gt 0) { $isaacExitCode = 0 }
  }
  if ($isaacExitCode -ne 0) { throw "Isaac runtime exited with code $isaacExitCode; see $logDir\isaac.err.log" }
  Write-LauncherProgress "isaac_completed"
  Write-Host "Isaac runtime completed; waiting for collector artifact"

  # Freeze the final sim clock and ask RTAB-Map to publish its accumulated
  # graph/cloud while the collector subscription is still alive.  This gives
  # the run a timestamped final map rather than only an initial latched cloud.
  Write-LauncherProgress "publish_map_start"
  Invoke-RtabmapPublishMap
  Write-LauncherProgress "publish_map_done"

  Assert-Running (@($services | Where-Object Name -eq "dashboard")[0]) "dashboard"
  Assert-Running (@($services | Where-Object Name -eq "mapping")[0]) "mapping"
  if ($null -ne $recorder) {
    Wait-TrackedProcess $recorder ($collectorDuration + $collectorStartupTimeout + 15) "rgb_recorder"
    $recorder.Process.Refresh()
    if ($null -ne $recorder.Process.ExitCode -and $recorder.Process.ExitCode -ne 0) { throw "RGB recorder exited with code $($recorder.Process.ExitCode); see $logDir\rgb_recorder.err.log" }
  }
  $collector = @($services | Where-Object Name -eq "collector")[0]
  Wait-TrackedProcess $collector ($collectorDuration + $collectorStartupTimeout + 15) "collector"
  $collector.Process.Refresh()
  if ($null -ne $collector.Process.ExitCode -and $collector.Process.ExitCode -ne 0) { throw "collector exited with code $($collector.Process.ExitCode); see $logDir\collector.err.log" }
  if (-not (Test-Path -LiteralPath $isaacStatusPath)) { throw "Isaac runtime status was not written: $isaacStatusPath" }
  Write-LauncherProgress "collector_completed"
  Write-Host "Collector completed; validating dashboard and artifacts"
  Assert-Running (@($services | Where-Object Name -eq "mapping")[0]) "mapping"
  Assert-RosNodesReady

  Write-LauncherProgress "dashboard_status_start"
  $dashboardStatus = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/api/status" -TimeoutSec 5
  Write-LauncherProgress "dashboard_status_done"
  if ([int]$dashboardStatus.map_point_count -le 0) { throw "No /slam/map_cloud data was observed; refusing to claim a successful mapping run." }

  $requiredArtifacts = @("metadata.json", "metrics.json", "ground_truth.csv", "estimate.csv", "rtabmap.db", "run_manifest.json", "effective_config.json")
  if ($isWalkingScenario) { $requiredArtifacts += @("rgb_camera.mp4", "rgb_camera_video.json") }
  foreach ($artifact in $requiredArtifacts) {
    $path = Join-Path $runDir $artifact
    if (-not (Test-Path -LiteralPath $path)) { throw "Required run artifact missing: $path" }
  }
  Write-LauncherProgress "artifacts_present"
  $metrics = Get-Content -LiteralPath (Join-Path $runDir "metrics.json") -Raw | ConvertFrom-Json
  if ($metrics.status -eq "insufficient_samples" -or [int]$metrics.sample_count -lt 3 -or [double]$metrics.duration_s -lt ($collectorDuration * 0.5)) { throw "Evaluation metrics are incomplete or too short after quaternion filtering; see $runDir\metrics.json" }
  if ($metrics.alignment_policy -ne "initial_se3") { throw "Live metrics did not record initial_se3 alignment." }

  $metadata = Get-Content -LiteralPath (Join-Path $runDir "metadata.json") -Raw | ConvertFrom-Json
  if (-not [bool]$metadata.simulation_time_complete) { throw "Collector did not reach the requested simulation-time target; completion=$($metadata.completion_reason)" }
  foreach ($topicName in $metadata.required_live_topics) {
    $observation = $metadata.topic_observations.$topicName
    if ($null -eq $observation -or [int]$observation.count -le 0) { throw "Required live ROS topic '$topicName' was not fresh in the collector." }
  }
  foreach ($topicName in @("clock", "rgb", "lidar", "tf")) {
    $observation = $metadata.topic_observations.$topicName
    if ($null -eq $observation.first_stamp_s -or $null -eq $observation.last_stamp_s -or [double]$observation.last_stamp_s -le [double]$observation.first_stamp_s) {
      throw "Required live ROS topic '$topicName' did not show advancing message timestamps."
    }
  }
  $clockObservation = $metadata.topic_observations.clock
  $odomObservation = $metadata.topic_observations.estimate
  $mapObservation = $metadata.topic_observations.map
  if ([int]$odomObservation.count -lt 2 -or [double]$odomObservation.last_stamp_s -le [double]$odomObservation.first_stamp_s) { throw "Fresh advancing /slam/odom data was not observed." }
  # RTAB-Map publishes its accumulated cloud at 1 Hz and can be a few
  # hundred milliseconds behind the final /clock sample while still having
  # fresh map data.  Keep this explicit, bounded age budget separate from the
  # advancing sensor checks above.
  $mapFreshnessMaxAgeS = 1.5
  if ($null -eq $mapObservation.last_stamp_s -or ([double]$clockObservation.last_stamp_s - [double]$mapObservation.last_stamp_s) -gt $mapFreshnessMaxAgeS) { throw "Fresh /slam/map_cloud data was not observed near the final simulation clock (age budget ${mapFreshnessMaxAgeS}s)." }
  $videoInfo = $null
  if ($isWalkingScenario) {
    $videoInfo = Get-Content -LiteralPath $rgbVideoMetadataPath -Raw | ConvertFrom-Json
    if ($videoInfo.status -ne "complete" -or [int]$videoInfo.frame_count -le 0 -or [int]$videoInfo.width -ne 1280 -or [int]$videoInfo.height -ne 720) { throw "RGB walking video validation failed; see $rgbVideoMetadataPath" }
    if ([double]$videoInfo.duration_s -lt ($collectorDuration * 0.8)) { throw "RGB walking video is too short; see $rgbVideoMetadataPath" }
  }
  Write-LauncherProgress "topic_freshness_passed"
  Write-Host "ROS topic freshness passed; backing up RTAB-Map database"
  Write-LauncherProgress "backup_start"
  Invoke-RtabmapBackup
  Write-LauncherProgress "backup_done"
  Write-Host "RTAB-Map backup completed; validating SQLite database"
  $databaseNodeCount = Assert-RtabmapDatabase (Join-Path $runDir "rtabmap.db")
  $runManifest.status = "passed"
  $runManifest.completed_utc = [DateTime]::UtcNow.ToString("o")
  $runManifest.isaac_status = Get-Content -LiteralPath $isaacStatusPath -Raw | ConvertFrom-Json
  $runManifest.database_validation = [ordered]@{ node_count = $databaseNodeCount; pause_service = "/rtabmap/pause"; backup_service = "/rtabmap/backup"; shutdown_policy = "RTAB-Map pause and backup services before launcher cleanup" }
  if ($isWalkingScenario) {
    $demoDir = Join-Path $repo "demo"
    New-Item -ItemType Directory -Force -Path $demoDir | Out-Null
    $demoVideoPath = Join-Path $demoDir "current_walking_aisle.mp4"
    Copy-Item -LiteralPath $rgbVideoPath -Destination $demoVideoPath -Force
    $runManifest.video = [ordered]@{ source = $rgbVideoPath; repository_path = "demo/current_walking_aisle.mp4"; metadata = $rgbVideoMetadataPath; codec = $videoInfo.codec; width = $videoInfo.width; height = $videoInfo.height; nominal_fps = $videoInfo.nominal_fps; frame_count = $videoInfo.frame_count; duration_s = $videoInfo.duration_s; file_size_bytes = $videoInfo.file_size_bytes }
  }
  Save-RunManifest

  Write-Host "Run complete: $runId"
  Write-Host "Artifacts: $runDir"
  Write-Host "Dashboard: http://127.0.0.1:$Port"
} catch {
  $runManifest.status = "failed"
  $runManifest.error = $_.Exception.Message
  Save-RunManifest
  throw
} finally {
  foreach ($service in $services) {
    if ($service.Process -and -not $service.Process.HasExited) { Stop-ProcessTree -RootPid $service.Process.Id }
  }
}
