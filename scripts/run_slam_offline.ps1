param(
  [Alias("Run")]
  [string]$RunDir = "",
  [string]$CaptureDir = "",
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$IsaacPython = "C:/isaacsim/python.bat",
  [string]$ExperimentName = "offline_slam"
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
if (-not $RunDir -and -not $CaptureDir) { throw "Pass -RunDir or -CaptureDir." }
if (-not $CaptureDir) { $CaptureDir = Join-Path (Resolve-Path $RunDir).Path "capture" }
$captureDir = (Resolve-Path -LiteralPath $CaptureDir).Path
if (-not $RunDir) { $RunDir = Split-Path -Parent $captureDir }
$runDir = (Resolve-Path -LiteralPath $RunDir).Path
$slamDir = Join-Path $runDir "slam"
if ($ExperimentName -and $ExperimentName -ne "offline_slam") { $slamDir = Join-Path $slamDir $ExperimentName }
$logsDir = Join-Path $runDir "logs"
New-Item -ItemType Directory -Force -Path $slamDir,$logsDir | Out-Null

$manifestPath = Join-Path $captureDir "capture_manifest.json"
if (-not (Test-Path -LiteralPath $manifestPath)) { throw "Capture manifest not found: $manifestPath" }
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") python -m simulator.capture.manifest --validate-for-slam $captureDir | Set-Content -LiteralPath (Join-Path $logsDir "sensor_capture_validation.json") -Encoding UTF8
if ($LASTEXITCODE -ne 0) { throw "Sensor-only capture validation failed." }
$bagUri = Join-Path $captureDir ([string]$manifest.bag.uri)
$duration = [double]$manifest.duration_s
$bagMetadata = Get-Content -LiteralPath (Join-Path $captureDir "bag_metadata.json") -Raw | ConvertFrom-Json
$lastLidarProperty = $bagMetadata.last_stamp_s.PSObject.Properties["/sim/lidar/points"]
if (-not $lastLidarProperty -or $null -eq $lastLidarProperty.Value) { throw "Capture bag metadata has no final LiDAR timestamp." }
$lastLidarStamp = [double]$lastLidarProperty.Value
$effectiveConfig = Get-Content -LiteralPath (Join-Path $captureDir "effective_config.json") -Raw | ConvertFrom-Json
$scanPeriod = 1.0 / [double]$effectiveConfig.lidar.hz
$database = Join-Path $slamDir "rtabmap.db"
$mappingPath = (Join-Path $repo "config/mapping/rtabmap/params.yaml").Replace([char]92, "/")
$databaseArg = $database.Replace([char]92, "/")
$mappingArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2","launch","grocery_sim_mapping","rtabmap_lidar.launch.py","use_sim_time:=true","database_path:=$databaseArg","mapping_params_path:=$mappingPath")
$baseArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2")

$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

function Stop-ProcessTree([int]$RootPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $RootPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($childPid in $children) { Stop-ProcessTree -RootPid ([int]$childPid); Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}
function Invoke-ProcessProbe {
  param(
    [string]$ExecutablePath,
    [string[]]$ArgumentList,
    [int]$TimeoutMilliseconds,
    [string]$StdoutPath,
    [string]$StderrPath
  )
  Remove-Item -LiteralPath $StdoutPath,$StderrPath -Force -ErrorAction SilentlyContinue
  $startedUtc = [DateTime]::UtcNow
  try {
    $process = Start-Process -FilePath $ExecutablePath -ArgumentList $ArgumentList -WindowStyle Hidden -PassThru -RedirectStandardOutput $StdoutPath -RedirectStandardError $StderrPath
  } catch {
    return [pscustomobject]@{
      timed_out=$false; exit_code=$null; stdout=""; stderr=""; start_error=$_.Exception.Message
      elapsed_ms=[int]([DateTime]::UtcNow - $startedUtc).TotalMilliseconds
    }
  }
  $completed = $process.WaitForExit([Math]::Max(1, $TimeoutMilliseconds))
  if (-not $completed) {
    Stop-ProcessTree -RootPid $process.Id
    [void]$process.WaitForExit(5000)
  } else {
    # A parameterless wait flushes redirected stdout/stderr after process exit.
    $process.WaitForExit()
  }
  $stdout = if (Test-Path -LiteralPath $StdoutPath) { (Get-Content -LiteralPath $StdoutPath -Raw -ErrorAction SilentlyContinue) } else { "" }
  $stderr = if (Test-Path -LiteralPath $StderrPath) { (Get-Content -LiteralPath $StderrPath -Raw -ErrorAction SilentlyContinue) } else { "" }
  [pscustomobject]@{
    timed_out=(-not $completed)
    exit_code=$(if ($completed) { $process.ExitCode } else { $null })
    stdout=[string]$stdout
    stderr=[string]$stderr
    start_error=$null
    elapsed_ms=[int]([DateTime]::UtcNow - $startedUtc).TotalMilliseconds
  }
}
function Test-RequiredRosNodes {
  param([string]$NodeListText, [string[]]$RequiredNodeNames)
  $observed = @($NodeListText -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ })
  foreach ($required in $RequiredNodeNames) {
    if ($observed -notcontains $required) { return $false }
  }
  return $true
}
function Wait-ForRosNodes {
  param(
    [string]$ExecutablePath,
    [string[]]$ArgumentList,
    [string[]]$RequiredNodeNames,
    [DateTime]$DeadlineUtc,
    [int]$PerProbeTimeoutMilliseconds,
    [int]$PollIntervalMilliseconds,
    [string]$DiagnosticLogPath,
    [string]$Phase,
    [System.Diagnostics.Process]$AbortIfExitedProcess = $null
  )
  $attempt = 0
  $lastProbe = $null
  while ([DateTime]::UtcNow -lt $DeadlineUtc) {
    if ($AbortIfExitedProcess -and $AbortIfExitedProcess.HasExited) { break }
    $attempt += 1
    if ($PollIntervalMilliseconds -gt 0) { Start-Sleep -Milliseconds $PollIntervalMilliseconds }
    $remainingMs = [int][Math]::Floor(($DeadlineUtc - [DateTime]::UtcNow).TotalMilliseconds)
    if ($remainingMs -le 0) { break }
    $probeTimeoutMs = [Math]::Max(1, [Math]::Min($PerProbeTimeoutMilliseconds, $remainingMs))
    $probeOut = "$DiagnosticLogPath.$Phase.out"
    $probeErr = "$DiagnosticLogPath.$Phase.err"
    $lastProbe = Invoke-ProcessProbe -ExecutablePath $ExecutablePath -ArgumentList $ArgumentList -TimeoutMilliseconds $probeTimeoutMs -StdoutPath $probeOut -StderrPath $probeErr
    $ready = (-not $lastProbe.timed_out) -and ($null -eq $lastProbe.start_error) -and ($lastProbe.exit_code -eq 0) -and (Test-RequiredRosNodes -NodeListText $lastProbe.stdout -RequiredNodeNames $RequiredNodeNames)
    [ordered]@{
      phase=$Phase; attempt=$attempt; recorded_utc=[DateTime]::UtcNow.ToString("o")
      deadline_utc=$DeadlineUtc.ToString("o"); probe_timeout_ms=$probeTimeoutMs
      elapsed_ms=$lastProbe.elapsed_ms; timed_out=$lastProbe.timed_out; exit_code=$lastProbe.exit_code
      start_error=$lastProbe.start_error; required_nodes=$RequiredNodeNames
      observed_nodes=@($lastProbe.stdout -split "`r?`n" | ForEach-Object { $_.Trim() } | Where-Object { $_ })
      stderr=$lastProbe.stderr; ready=$ready
    } | ConvertTo-Json -Compress -Depth 5 | Add-Content -LiteralPath $DiagnosticLogPath -Encoding UTF8
    if ($ready) {
      return [pscustomobject]@{ready=$true; attempts=$attempt; last_probe=$lastProbe}
    }
    if ($AbortIfExitedProcess -and $AbortIfExitedProcess.HasExited) { break }
  }
  return [pscustomobject]@{ready=$false; attempts=$attempt; last_probe=$lastProbe}
}
function Wait-ProcessWithTimeout($Process, [int]$TimeoutSeconds, [string]$Name) {
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while (-not $Process.HasExited -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
  if (-not $Process.HasExited) { throw "$Name did not finish within $TimeoutSeconds seconds." }
  if ($Process.ExitCode -ne 0) { throw "$Name exited with code $($Process.ExitCode)." }
  return $Process.ExitCode
}

Push-Location $repo
$router = $null; $mapping = $null; $observer = $null; $player = $null
try {
  foreach ($packageName in @("rtabmap_odom","rtabmap_slam","grocery_sim_mapping")) {
    $prefix = & $pixi @($baseArgs + @("pkg","prefix",$packageName)) 2>$null
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace(($prefix -join ""))) { throw "Required ROS package '$packageName' is unavailable." }
  }
  $router = Start-Process -FilePath $pixi -ArgumentList @($baseArgs + @("run","rmw_zenoh_cpp","rmw_zenohd")) -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_zenoh.out.log") -RedirectStandardError (Join-Path $logsDir "offline_zenoh.err.log")
  # Start mapping immediately; the bounded ROS graph probe below, not a delay,
  # gates bag playback on both estimator nodes being visible.
  $mapping = Start-Process -FilePath $pixi -ArgumentList $mappingArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_mapping.out.log") -RedirectStandardError (Join-Path $logsDir "offline_mapping.err.log")
  $nodeReadinessLog = Join-Path $logsDir "offline_node_readiness.jsonl"
  Remove-Item -LiteralPath $nodeReadinessLog -Force -ErrorAction SilentlyContinue
  $nodeListArgs = @($baseArgs + @("node","list"))
  $nodeDeadline = [DateTime]::UtcNow.AddSeconds(90)
  $mappingReady = Wait-ForRosNodes -ExecutablePath $pixi -ArgumentList $nodeListArgs -RequiredNodeNames @("/icp_odometry","/rtabmap") -DeadlineUtc $nodeDeadline -PerProbeTimeoutMilliseconds 10000 -PollIntervalMilliseconds 250 -DiagnosticLogPath $nodeReadinessLog -Phase "mapping"
  if (-not $mappingReady.ready) {
    $lastMappingProbe = $mappingReady.last_probe
    throw "RTAB-Map nodes did not become ready before the absolute deadline after $($mappingReady.attempts) probe(s). Last probe timed_out=$($lastMappingProbe.timed_out), exit_code=$($lastMappingProbe.exit_code), start_error=$($lastMappingProbe.start_error), nodes=$($lastMappingProbe.stdout), stderr=$($lastMappingProbe.stderr)"
  }

  $observerArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.capture.slam_observer","--output-dir",$slamDir,"--duration-seconds",([string]$duration),"--startup-timeout-seconds","180","--expected-sensor-last-stamp-seconds",([string]$lastLidarStamp),"--sensor-scan-period-seconds",([string]$scanPeriod),"--database-path",$database)
  $replaySignal = Join-Path $slamDir "bag_replay.complete"
  Remove-Item -LiteralPath $replaySignal -Force -ErrorAction SilentlyContinue
  $observerArgs += @("--replay-complete-signal",$replaySignal)
  $observer = Start-Process -FilePath $pixi -ArgumentList $observerArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_observer.out.log") -RedirectStandardError (Join-Path $logsDir "offline_observer.err.log")
  $observerReadyDeadline = [DateTime]::UtcNow.AddSeconds(45)
  $observerReady = Wait-ForRosNodes -ExecutablePath $pixi -ArgumentList $nodeListArgs -RequiredNodeNames @("/grocery_sim_offline_slam_observer") -DeadlineUtc $observerReadyDeadline -PerProbeTimeoutMilliseconds 10000 -PollIntervalMilliseconds 250 -DiagnosticLogPath $nodeReadinessLog -Phase "observer" -AbortIfExitedProcess $observer
  if ($observer.HasExited -or -not $observerReady.ready) {
    $lastObserverProbe = $observerReady.last_probe
    throw "Offline SLAM observer did not become ready before replay after $($observerReady.attempts) probe(s). observer_exited=$($observer.HasExited), last_probe_timed_out=$($lastObserverProbe.timed_out), exit_code=$($lastObserverProbe.exit_code), start_error=$($lastObserverProbe.start_error), nodes=$($lastObserverProbe.stdout), stderr=$($lastObserverProbe.stderr)"
  }
  # /clock has exactly one source: rosbag2's playback clock. Keep the recorded
  # /clock topic out of the bag topic selection to prevent a second publisher.
  $replayTopics = @("--topics","/sim/camera/rgb/image_raw","/sim/camera/rgb/camera_info","/sim/lidar/points","/tf","/tf_static")
  $player = Start-Process -FilePath $pixi -ArgumentList @($baseArgs + @("bag","play",$bagUri.Replace([char]92, "/"),"--clock") + $replayTopics) -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_bag_play.out.log") -RedirectStandardError (Join-Path $logsDir "offline_bag_play.err.log")
  $waitSeconds = [Math]::Max(240, [int]($duration * 20) + 120)
  Wait-ProcessWithTimeout $player $waitSeconds "bag replay" | Out-Null
  New-Item -ItemType File -Force -Path $replaySignal | Out-Null
  Wait-ProcessWithTimeout $observer $waitSeconds "offline SLAM observer" | Out-Null

  if (Test-Path -LiteralPath (Join-Path $slamDir "slam_observer.json")) {
    $observerMeta = Get-Content -LiteralPath (Join-Path $slamDir "slam_observer.json") -Raw | ConvertFrom-Json
  } else { throw "SLAM observer metadata is missing." }
  if (-not $observerMeta.fresh_odom -or -not $observerMeta.fresh_map) { throw "Offline SLAM did not produce fresh odom and map data." }
  if (-not $observerMeta.replay_complete_signal_observed -or -not $observerMeta.clock_target_reached -or -not $observerMeta.replay_drained -or -not $observerMeta.processed_sensor_span) { throw "Offline SLAM did not confirm replay completion, target clock coverage, ROS drain, and mapper input processing." }
  if (-not $observerMeta.publish_map_acknowledged -or -not $observerMeta.final_map_span -or -not $observerMeta.mapper_database_span) { throw "RTAB-Map did not acknowledge and commit the final optimized map across the captured sensor span." }
  if (-not $observerMeta.map_pose_correction_complete -or -not $observerMeta.optimized_pose_graph_complete -or -not $observerMeta.map_graph_matches_final_cloud -or [int]$observerMeta.map_pose_sample_count -le 0 -or $observerMeta.map_pose_frame_id -ne "map" -or $observerMeta.pose_source -ne "rtabmap_optimized_graph" -or [string]::IsNullOrWhiteSpace([string]$observerMeta.graph_pose_version) -or $observerMeta.graph_pose_version -ne $observerMeta.pre_publish_graph_version -or [string]::IsNullOrWhiteSpace([string]$observerMeta.dense_pose_version) -or [string]::IsNullOrWhiteSpace([string]$observerMeta.map_version) -or [double]$observerMeta.final_map_graph_stamp_s -ne [double]$observerMeta.final_cloud_stamp_s -or $observerMeta.final_map_graph_frame_id -ne "map") { throw "RTAB-Map did not provide poses from the same versioned optimized map graph as the final cloud." }
  foreach ($poseArtifact in @("slam_map_poses.csv","slam_map_keyframes.csv","slam_odom_poses.csv","map_to_odom.csv","slam_poses.csv","slam_map.pcd","slam_map.ply")) {
    $artifactPath = Join-Path $slamDir $poseArtifact
    $artifactRecord = @($observerMeta.files | Where-Object { $_.path -eq $poseArtifact }) | Select-Object -First 1
    if (-not (Test-Path -LiteralPath $artifactPath) -or -not $artifactRecord) { throw "SLAM pose artifact is missing from observer manifest: $poseArtifact" }
    $artifactInfo = Get-Item -LiteralPath $artifactPath
    $artifactHash = (Get-FileHash -LiteralPath $artifactPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ([long]$artifactInfo.Length -ne [long]$artifactRecord.size_bytes -or $artifactHash -ne [string]$artifactRecord.sha256) { throw "SLAM pose artifact failed observer manifest verification: $poseArtifact" }
    if ($artifactRecord.optimized -and ($artifactRecord.map_version -ne $observerMeta.map_version -or $artifactRecord.frame_id -ne "map" -or [string]::IsNullOrWhiteSpace([string]$artifactRecord.role))) { throw "SLAM optimized artifact identity differs from observer manifest: $poseArtifact" }
    if ($poseArtifact -eq "slam_map_poses.csv" -and $artifactRecord.dense_pose_version -ne $observerMeta.dense_pose_version) { throw "Dense trajectory artifact version differs from observer manifest." }
  }
  $mapPoseRows = @(Import-Csv -LiteralPath (Join-Path $slamDir "slam_map_poses.csv"))
  if ($mapPoseRows.Count -ne [int]$observerMeta.map_pose_sample_count -or $mapPoseRows.Count -ne [int]$observerMeta.odom_sample_count -or $mapPoseRows.Count -le 0 -or @($mapPoseRows | Where-Object { $_.frame_id -ne "map" }).Count -ne 0) { throw "Dense optimized map pose artifact is empty, incomplete, or mislabeled." }
  $rawOdomRows = @(Import-Csv -LiteralPath (Join-Path $slamDir "slam_odom_poses.csv"))
  if ($rawOdomRows.Count -ne [int]$observerMeta.odom_sample_count -or @($rawOdomRows | Where-Object { $_.frame_id -ne "odom" }).Count -ne 0) { throw "Raw odometry diagnostic artifact is incomplete or mislabeled." }
  $mapPoseTimes = @($mapPoseRows | ForEach-Object { [double]$_.timestamp_s } | Sort-Object)
  $rawOdomTimes = @($rawOdomRows | ForEach-Object { [double]$_.timestamp_s } | Sort-Object)
  if (Compare-Object -ReferenceObject $rawOdomTimes -DifferenceObject $mapPoseTimes) { throw "Dense map poses do not cover the exact raw odometry timestamp multiset." }
  $keyframeRows = @(Import-Csv -LiteralPath (Join-Path $slamDir "slam_map_keyframes.csv"))
  $keyframeArtifact = @($observerMeta.files | Where-Object { $_.path -eq "slam_map_keyframes.csv" }) | Select-Object -First 1
  if ($keyframeRows.Count -le 0 -or @($keyframeRows | Where-Object { $_.frame_id -ne "map" }).Count -ne 0 -or [int]$keyframeArtifact.schema_version -ne 2) { throw "Optimized keyframe artifact is empty, mislabeled, or has an unsupported schema." }
  if ([int]$observerMeta.clock_regressions -ne 0) { throw "Offline replay clock regressed $($observerMeta.clock_regressions) time(s)." }
  if ([double]$observerMeta.last_clock_s -lt [double]$observerMeta.target_clock_s - 0.001) { throw "Offline replay ended before the target simulation time." }
  if ([double]$observerMeta.last_odom_stamp_s -lt [double]$observerMeta.expected_sensor_last_stamp_s - [double]$observerMeta.sensor_scan_period_s - 0.001) { throw "SLAM odometry did not process the final captured LiDAR scan span." }
  if (-not (Test-Path -LiteralPath $database)) { throw "RTAB-Map database was not created." }
  # Reopen the DB after stopping the mapper to verify persisted input-span rows.
  Stop-ProcessTree -RootPid $mapping.Id
  $mapping = $null
  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") python (Join-Path $repo "scripts/validate_rtabmap_db.py") $database --minimum-node-stamp $lastLidarStamp --scan-period-seconds $scanPeriod | Set-Content -LiteralPath (Join-Path $slamDir "database_validation.txt") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "RTAB-Map database validation failed." }
  [ordered]@{
    ros_distro="jazzy"; rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    ros2_cli_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix ros2cli) -join " ").Trim()
    rtabmap_odom_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_odom) -join " ").Trim()
    rtabmap_slam_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_slam) -join " ").Trim()
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $slamDir "software_versions.json") -Encoding UTF8
  $observerRecord = Get-Content -LiteralPath (Join-Path $slamDir "slam_observer.json") -Raw | ConvertFrom-Json
  $observerArtifactPath = Join-Path $slamDir "slam_observer.json"
  $observerArtifactInfo = Get-Item -LiteralPath $observerArtifactPath
  $observerArtifactHash = (Get-FileHash -LiteralPath $observerArtifactPath -Algorithm SHA256).Hash.ToLowerInvariant()
  [ordered]@{
    status="complete"; experiment=$ExperimentName; capture_id=$manifest.capture_id; capture_sha256=$manifest.capture_sha256
    git_sha=(& git -C $repo rev-parse HEAD).Trim(); rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    bag_replayed=$bagUri; ground_truth_subscribed=$false; publish_map_service_acknowledged=$observerMeta.publish_map_acknowledged; database_path=$database
    pre_publish_graph_version=$observerMeta.pre_publish_graph_version; pre_publish_graph_version_source=$observerMeta.pre_publish_graph_version_source; graph_pose_version=$observerMeta.graph_pose_version
    dense_pose_version=$observerMeta.dense_pose_version; map_version=$observerMeta.map_version
    map_frame_id=$observerMeta.map_pose_frame_id; optimized=$observerMeta.optimized_pose_graph_complete
    observer_artifact=[ordered]@{path="slam_observer.json"; size_bytes=[long]$observerArtifactInfo.Length; sha256=$observerArtifactHash}
    artifacts=$observerMeta.files; observer=$observerRecord
  } | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $slamDir "slam_manifest.json") -Encoding UTF8
  Write-Host "Offline SLAM complete: $slamDir"
} finally {
  foreach ($process in @($player,$observer)) { if ($process -and -not $process.HasExited) { Stop-ProcessTree -RootPid $process.Id } }
  if ($mapping -and -not $mapping.HasExited) { Stop-ProcessTree -RootPid $mapping.Id }
  if ($router -and -not $router.HasExited) { Stop-ProcessTree -RootPid $router.Id }
  Pop-Location
}
