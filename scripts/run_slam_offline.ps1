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
. (Join-Path $PSScriptRoot "process_status.ps1")
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
$logPrefix = if ($ExperimentName -and $ExperimentName -ne "offline_slam") {
  "offline_" + ($ExperimentName -replace '[^A-Za-z0-9_.-]', '_')
} else { "offline" }

$manifestPath = Join-Path $captureDir "capture_manifest.json"
if (-not (Test-Path -LiteralPath $manifestPath)) { throw "Capture manifest not found: $manifestPath" }
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if ($manifest.status -ne "complete") { throw "Capture is not complete." }
$required = @("/clock","/sim/camera/rgb/image_raw","/sim/lidar/points","/tf","/tf_static")
foreach ($topic in $required) { if (@($manifest.bag.topics) -notcontains $topic) { throw "Capture bag is missing required topic $topic." } }
$cameraInfoPath = Join-Path $captureDir ([string]$manifest.rgb.camera_info)
if ($manifest.rgb.camera_info_provenance -ne "configured_intrinsics" -or -not (Test-Path -LiteralPath $cameraInfoPath -PathType Leaf)) {
  throw "Capture is missing its configured camera intrinsics artifact."
}
$bagUri = Join-Path $captureDir ([string]$manifest.bag.uri)
if (-not (Test-Path -LiteralPath $bagUri -PathType Container)) { throw "Capture bag not found: $bagUri" }
$duration = [double]$manifest.duration_s
$database = Join-Path $slamDir "rtabmap.db"
if (Test-Path -LiteralPath $database) { throw "Refusing to reuse an existing RTAB-Map database: $database" }
$nativeExportDir = Join-Path $slamDir "native_export"
New-Item -ItemType Directory -Force -Path $nativeExportDir | Out-Null
$mappingPath = (Join-Path $repo "config/mapping/rtabmap/params.yaml").Replace([char]92, "/")
$databaseArg = $database.Replace([char]92, "/")
$mappingArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2","launch","grocery_sim_mapping","rtabmap_lidar.launch.py","use_sim_time:=true","database_path:=$databaseArg","mapping_params_path:=$mappingPath")
$baseArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2")

$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"

function Wait-ProcessWithTimeout($Process, [int]$TimeoutSeconds, [string]$Name) {
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while (-not $Process.HasExited -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
  if (-not $Process.HasExited) { throw "$Name did not finish within $TimeoutSeconds seconds." }
  return $Process.ExitCode
}

$script:offlineProbeSequence = 0
function Invoke-RosCliProbe([string]$Name, [string[]]$Arguments, [int]$TimeoutSeconds) {
  $script:offlineProbeSequence += 1
  $safeName = $Name -replace '[^A-Za-z0-9_.-]', '_'
  $stem = "{0}_probe_{1:D2}_{2}" -f $logPrefix,$script:offlineProbeSequence,$safeName
  return Invoke-BoundedProcess -FilePath $pixi -ArgumentList @($baseArgs + $Arguments) `
    -WorkingDirectory $workspace -TimeoutSeconds $TimeoutSeconds -Name $Name `
    -RedirectStandardOutput (Join-Path $logsDir "$stem.out.log") `
    -RedirectStandardError (Join-Path $logsDir "$stem.err.log")
}

Push-Location $repo
$router = $null; $mapping = $null; $player = $null
try {
  $packagePrefixes = [ordered]@{}
  foreach ($packageName in @("ros2cli","rtabmap_odom","rtabmap_slam","grocery_sim_mapping")) {
    $prefixResult = Invoke-RosCliProbe "package prefix $packageName" @("pkg","prefix",$packageName) 15
    if ($prefixResult.ExitCode -ne 0 -or [string]::IsNullOrWhiteSpace($prefixResult.Stdout)) {
      throw "Required ROS package '$packageName' is unavailable; see $($prefixResult.StderrPath)."
    }
    $packagePrefixes[$packageName] = $prefixResult.Stdout.Trim()
  }
  $router = Start-Process -FilePath $pixi -ArgumentList @($baseArgs + @("run","rmw_zenoh_cpp","rmw_zenohd")) -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_zenoh.out.log") -RedirectStandardError (Join-Path $logsDir "${logPrefix}_zenoh.err.log")
  Start-Sleep -Seconds 6
  $mapping = Start-Process -FilePath $pixi -ArgumentList $mappingArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_mapping.out.log") -RedirectStandardError (Join-Path $logsDir "${logPrefix}_mapping.err.log")
  $nodeDeadline = (Get-Date).AddSeconds(90)
  $nodes = ""
  do {
    Start-Sleep -Seconds 2
    $mapping.Refresh()
    if ($mapping.HasExited) { throw "RTAB-Map launch exited before readiness; see ${logPrefix}_mapping.err.log." }
    $remainingSeconds = [Math]::Max(1, [int][Math]::Ceiling(($nodeDeadline - (Get-Date)).TotalSeconds))
    $probeTimeoutSeconds = [Math]::Min(10, $remainingSeconds)
    try {
      $nodeResult = Invoke-RosCliProbe "RTAB-Map node readiness" @("node","list","--no-daemon","--spin-time","2") $probeTimeoutSeconds
      $nodes = if ($nodeResult.ExitCode -eq 0) { $nodeResult.Stdout } else { "" }
    } catch {
      $nodes = "probe failure: $($_.Exception.Message)"
    }
  } while ((($nodes -notmatch "icp_odometry") -or ($nodes -notmatch "rtabmap")) -and (Get-Date) -lt $nodeDeadline)
  if (($nodes -notmatch "icp_odometry") -or ($nodes -notmatch "rtabmap")) { throw "RTAB-Map nodes did not become ready. Nodes: $nodes" }

  $replayTopics = @("/clock","/sim/lidar/points","/tf","/tf_static")
  $player = Start-Process -FilePath $pixi -ArgumentList @($baseArgs + @("bag","play",$bagUri.Replace([char]92, "/"),"--clock","--topics") + $replayTopics) -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_bag_play.out.log") -RedirectStandardError (Join-Path $logsDir "${logPrefix}_bag_play.err.log")
  $waitSeconds = [Math]::Max(240, [int]($duration * 20) + 120)
  $playerExit = Wait-ProcessWithTimeout $player $waitSeconds "bag replay"
  if ($playerExit -ne 0) { throw "Bag replay failed with exit code $playerExit; see ${logPrefix}_bag_play.err.log." }

  # Let RTAB-Map drain the DDS subscription queue and stop accepting data.
  # The RTAB-Map backup service copies only the SQLite main file and can omit
  # live WAL content on Windows, so stop the writer before validating/exporting.
  $settleSeconds = [Math]::Max(15, [Math]::Min(60, [int]($duration * 0.5)))
  Start-Sleep -Seconds $settleSeconds
  $pauseResult = Invoke-RosCliProbe "pause RTAB-Map" @("service","call","/rtabmap/pause","std_srvs/srv/Empty") 30
  if ($pauseResult.ExitCode -ne 0) { throw "RTAB-Map pause service failed; see $($pauseResult.StderrPath)." }
  if ($mapping -and -not $mapping.HasExited) { Stop-BoundedProcessTree -RootPid $mapping.Id }
  if ($router -and -not $router.HasExited) { Stop-BoundedProcessTree -RootPid $router.Id }
  Start-Sleep -Seconds 3
  if (-not (Test-Path -LiteralPath $database -PathType Leaf)) { throw "RTAB-Map database was not created." }
  $databaseValidation = Invoke-BoundedProcess -FilePath $pixi `
    -ArgumentList @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python",(Join-Path $repo "scripts/validate_rtabmap_db.py"),$database) `
    -WorkingDirectory $repo -TimeoutSeconds 60 -Name "RTAB-Map database validation" `
    -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_database_validation.out.log") `
    -RedirectStandardError (Join-Path $logsDir "${logPrefix}_database_validation.err.log")
  $databaseValidation.Stdout | Set-Content -LiteralPath (Join-Path $slamDir "database_validation.txt") -Encoding UTF8
  if ($databaseValidation.ExitCode -ne 0) { throw "RTAB-Map database validation failed; see $($databaseValidation.StderrPath)." }

  $exporterBaseArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"rtabmap-export.exe")
  $versionResult = Invoke-BoundedProcess -FilePath $pixi -ArgumentList @($exporterBaseArgs + @("--version")) `
    -WorkingDirectory $repo -TimeoutSeconds 30 -Name "RTAB-Map exporter version" `
    -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_exporter_version.out.log") `
    -RedirectStandardError (Join-Path $logsDir "${logPrefix}_exporter_version.err.log")
  if ($versionResult.ExitCode -ne 0 -or $versionResult.Stdout -notmatch "RTAB-Map:") { throw "Unable to identify rtabmap-export; see $($versionResult.StderrPath)." }

  $optimizeArgs = @("--poses","--poses_format","10","--opt","0","--save_in_db","--output","slam_optimized","--output_dir",$nativeExportDir,$database)
  $optimizeResult = Invoke-BoundedProcess -FilePath $pixi -ArgumentList @($exporterBaseArgs + $optimizeArgs) `
    -WorkingDirectory $repo -TimeoutSeconds 180 -Name "RTAB-Map global optimization" `
    -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_optimize.out.log") `
    -RedirectStandardError (Join-Path $logsDir "${logPrefix}_optimize.err.log")
  if ($optimizeResult.ExitCode -ne 0) { throw "RTAB-Map global optimization failed; see $($optimizeResult.StderrPath)." }

  $exportArgs = @("--cloud","--scan","--poses","--poses_format","10","--ascii","--opt","2","--max_range","100","--voxel","0.03","--output","slam_map","--output_dir",$nativeExportDir,$database)
  $exportResult = Invoke-BoundedProcess -FilePath $pixi -ArgumentList @($exporterBaseArgs + $exportArgs) `
    -WorkingDirectory $repo -TimeoutSeconds 300 -Name "RTAB-Map optimized map export" `
    -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_export.out.log") `
    -RedirectStandardError (Join-Path $logsDir "${logPrefix}_export.err.log")
  if ($exportResult.ExitCode -ne 0) { throw "RTAB-Map optimized map export failed; see $($exportResult.StderrPath)." }

  $commandPath = Join-Path $nativeExportDir "commands.json"
  [ordered]@{
    optimize=@("rtabmap-export.exe") + $optimizeArgs
    export=@("rtabmap-export.exe") + $exportArgs
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $commandPath -Encoding UTF8
  $finalizeResult = Invoke-BoundedProcess -FilePath $pixi `
    -ArgumentList @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.capture.finalize_slam_export","--poses",(Join-Path $nativeExportDir "slam_map_poses.txt"),"--cloud",(Join-Path $nativeExportDir "slam_map_cloud.ply"),"--database",$database,"--output-dir",$slamDir,"--exporter-version-log",$versionResult.StdoutPath,"--commands-json",$commandPath) `
    -WorkingDirectory $repo -TimeoutSeconds 180 -Name "SLAM native export validation" `
    -RedirectStandardOutput (Join-Path $logsDir "${logPrefix}_finalize.out.log") `
    -RedirectStandardError (Join-Path $logsDir "${logPrefix}_finalize.err.log")
  if ($finalizeResult.ExitCode -ne 0) { throw "SLAM native export validation failed; see $($finalizeResult.StderrPath)." }
  $producerPath = Join-Path $slamDir "slam_producer.json"
  if (-not (Test-Path -LiteralPath $producerPath -PathType Leaf)) { throw "SLAM producer metadata is missing." }
  $producerMeta = Get-Content -LiteralPath $producerPath -Raw | ConvertFrom-Json
  if ($producerMeta.status -ne "complete" -or $producerMeta.producer_mode -ne "rtabmap_database_export") { throw "SLAM database export metadata is incomplete." }
  [ordered]@{
    ros_distro="jazzy"; rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    ros2_cli_prefix=$packagePrefixes["ros2cli"]
    rtabmap_odom_prefix=$packagePrefixes["rtabmap_odom"]
    rtabmap_slam_prefix=$packagePrefixes["rtabmap_slam"]
    grocery_sim_mapping_prefix=$packagePrefixes["grocery_sim_mapping"]
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $slamDir "software_versions.json") -Encoding UTF8
  [ordered]@{
    status="complete"; experiment=$ExperimentName; capture_id=$manifest.capture_id; capture_sha256=$manifest.capture_sha256
    git_sha=(& git -C $repo rev-parse HEAD).Trim(); rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    bag_replayed=$bagUri; replay_topics=$replayTopics; capture_topics_verified=$required
    ground_truth_subscribed=$false; ground_truth_consumed=$false; database_path=$database
    producer_mode="rtabmap_database_export"; producer=$producerMeta
  } | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $slamDir "slam_manifest.json") -Encoding UTF8
  Write-Host "Offline SLAM complete: $slamDir"
} finally {
  if ($player -and -not $player.HasExited) { Stop-BoundedProcessTree -RootPid $player.Id }
  if ($mapping -and -not $mapping.HasExited) { Stop-BoundedProcessTree -RootPid $mapping.Id }
  if ($router -and -not $router.HasExited) { Stop-BoundedProcessTree -RootPid $router.Id }
  Pop-Location
}
