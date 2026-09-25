param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/walking_production_1080p.yaml",
  [int]$Frames = 0,
  [int]$StartupTimeoutSeconds = 600,
  [int]$ProgressTimeoutSeconds = 60,
  [switch]$Realtime,
  [ValidateRange(0.05, 1.0)]
  [double]$RealtimeFactor = 0.25,
  [switch]$Gui,
  [switch]$Headless
)
$ErrorActionPreference = "Stop"
if ($Gui -and $Headless) { throw "Choose either -Gui or -Headless, not both." }
if ($StartupTimeoutSeconds -lt 300) { throw "StartupTimeoutSeconds must allow a cold Isaac startup (minimum 300)." }
if ($ProgressTimeoutSeconds -lt 10) { throw "ProgressTimeoutSeconds must be at least 10." }
if (-not $Realtime) { throw "Production capture requires -Realtime pacing for lossless ROS consumers." }
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
. (Join-Path $PSScriptRoot "capture_source_guard.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
$scenarioPath = (Resolve-Path -LiteralPath $scenarioPath).Path
$scenarioData = Get-Content -LiteralPath $scenarioPath -Raw | ConvertFrom-Json
$trajectoryPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path $scenarioPath -Parent) $scenarioData.trajectory)).Path
$sensorPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path $scenarioPath -Parent) $scenarioData.sensors)).Path
$sensorData = Get-Content -LiteralPath $sensorPath -Raw | ConvertFrom-Json
$expectedWidth = [int]$sensorData.camera.width_px
$expectedHeight = [int]$sensorData.camera.height_px
$expectedFps = [double]$sensorData.camera.fps
$expectedFx = ($expectedWidth / 2.0) / [Math]::Tan(([double]$sensorData.camera.horizontal_fov_deg) * [Math]::PI / 360.0)
$expectedCx = ($expectedWidth - 1.0) / 2.0
$expectedCy = ($expectedHeight - 1.0) / 2.0
if ($expectedWidth -lt 1920 -or $expectedHeight -lt 1080) { throw "Production capture requires native camera resolution of at least 1920x1080." }
Write-Host "Production capture scenario: $scenarioPath"
Write-Host "Production camera config: $sensorPath ($expectedWidth x $expectedHeight at $expectedFps Hz)"
$duration = [double](Get-Content -LiteralPath $trajectoryPath -Raw | ConvertFrom-Json).duration_s
$captureHorizon = if ($Frames -gt 0) { [Math]::Min($duration, ([double]$Frames / 60.0)) } else { $duration }
$captureId = Get-Date -Format "yyyyMMdd-HHmmssfff"
$runDir = Join-Path $repo (Join-Path "runs" $captureId)
$captureDir = Join-Path $runDir "capture"
$slamDir = Join-Path $runDir "slam"
$perceptionDir = Join-Path $runDir "perception"
$outputsDir = Join-Path $runDir "outputs"
$logsDir = Join-Path $runDir "logs"
New-Item -ItemType Directory -Force -Path $captureDir,$slamDir,$perceptionDir,$outputsDir,$logsDir | Out-Null

$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"
$env:RCUTILS_LOGGING_BUFFERED_STREAM = "0"
$zenohSessionConfig = (Resolve-Path -LiteralPath (Join-Path $repo "config\ros2\production_zenoh_session.json5")).Path
$zenohRouterConfig = (Resolve-Path -LiteralPath (Join-Path $repo "config\ros2\production_zenoh_router.json5")).Path
$env:ZENOH_SESSION_CONFIG_URI = $zenohSessionConfig
$env:ZENOH_ROUTER_CONFIG_URI = $zenohRouterConfig

function Stop-ProcessTree([int]$RootPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $RootPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($childPid in $children) {
    Stop-ProcessTree -RootPid ([int]$childPid)
    Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue
  }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

function Wait-ProcessWithTimeout($Process, [int]$TimeoutSeconds, [string]$Name) {
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while (-not $Process.HasExited -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
  if (-not $Process.HasExited) { throw "$Name did not finish within $TimeoutSeconds seconds." }
  return $Process.ExitCode
}

Push-Location $repo
$router = $null
$bag = $null
$videoBuilder = $null
$isaacExit = 1
try {
  & $IsaacPython -c "from simulator.capture.export_metadata import export_metadata; import json; print(json.dumps(export_metadata(r'$scenarioPath', r'$captureDir', r'$repo')))" | Set-Content -LiteralPath (Join-Path $logsDir "capture_metadata_export.json") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "Capture metadata export failed." }

  Copy-Item -LiteralPath (Join-Path $repo "config\contracts.yaml") -Destination (Join-Path $captureDir "contracts.yaml")
  Copy-Item -LiteralPath $scenarioPath -Destination (Join-Path $captureDir "scenario.yaml")
  $gitSha = (& git -C $repo rev-parse HEAD).Trim()
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($gitSha)) { throw "Could not resolve capture source commit." }
  $gitTree = (& git -C $repo rev-parse 'HEAD^{tree}').Trim()
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($gitTree)) { throw "Could not resolve capture source tree." }
  $gitStatus = @(& git -C $repo status --porcelain=v1 2>$null)
  if ($LASTEXITCODE -ne 0) { throw "Could not verify capture source worktree state." }
  if ($gitStatus.Count -ne 0) { throw "Production capture requires a clean source worktree." }
  [ordered]@{
    capture_id=$captureId; git_sha=$gitSha; git_tree=$gitTree; git_worktree_clean=$true; scenario=$scenarioPath
    scenario_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $scenarioPath).Hash.ToLowerInvariant()
    sensor_config=$sensorPath; sensor_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $sensorPath).Hash.ToLowerInvariant()
    camera=[ordered]@{ width_px=$expectedWidth; height_px=$expectedHeight; fps=$expectedFps; horizontal_fov_deg=[double]$sensorData.camera.horizontal_fov_deg }
    rmw=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID; started_utc=(Get-Date).ToUniversalTime().ToString("o"); duration_s=$captureHorizon; scenario_duration_s=$duration; frame_limit=$Frames
    pacing=[ordered]@{ enabled=$true; requested_realtime_factor=$RealtimeFactor }
    zenoh=[ordered]@{
      session_config=$zenohSessionConfig; session_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohSessionConfig).Hash.ToLowerInvariant()
      router_config=$zenohRouterConfig; router_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohRouterConfig).Hash.ToLowerInvariant()
    }
  } |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $captureDir "provenance.json") -Encoding UTF8
  [ordered]@{
    isaac_sim="6.1.0"; ros_distro="jazzy"; rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    pixi_path=$pixi; ros_workspace=$workspace; ros2_cli_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix ros2cli) -join " ").Trim()
    rtabmap_odom_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_odom) -join " ").Trim()
    rtabmap_slam_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_slam) -join " ").Trim()
    zenoh_session_config=$zenohSessionConfig; zenoh_session_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohSessionConfig).Hash.ToLowerInvariant()
    zenoh_router_config=$zenohRouterConfig; zenoh_router_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohRouterConfig).Hash.ToLowerInvariant()
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runDir "software_versions.json") -Encoding UTF8

  $router = Start-Process -FilePath $pixi -ArgumentList @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2","run","rmw_zenoh_cpp","rmw_zenohd") -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "zenoh.out.log") -RedirectStandardError (Join-Path $logsDir "zenoh.err.log")
  # Give rmw_zenohd time to become a real router before the first capture
  # subscriber is created.  A subscriber started during router bootstrap can
  # remain undiscoverable on Windows even though later ROS nodes connect.
  Start-Sleep -Seconds 6

  $bagUri = Join-Path $captureDir "sensors_bag"
  $bagArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.capture.rosbag_capture","--output",$bagUri,"--metadata",(Join-Path $captureDir "bag_metadata.json"),"--duration-seconds",([string]$captureHorizon),"--startup-timeout-seconds",([string]$StartupTimeoutSeconds),"--progress-timeout-seconds",([string]$ProgressTimeoutSeconds))
  $bag = Start-Process -FilePath $pixi -ArgumentList $bagArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "bag.out.log") -RedirectStandardError (Join-Path $logsDir "bag.err.log")
  Start-Sleep -Seconds 2

  $simScript = Join-Path $PSScriptRoot "run_sim.ps1"
  $simStatus = Join-Path $captureDir "isaac_runtime_status.json"
  $simArgs = @{
    Scenario=$scenarioPath; PixiPath=$pixi; RosWorkspace=$workspace; IsaacPython=$IsaacPython
    StatusPath=$simStatus; RequireSensorSamples=$true; RealtimeFactor=$RealtimeFactor
  }
  if ($Frames -gt 0) { $simArgs.Frames = $Frames }
  if ($Realtime) { $simArgs.Realtime = $true }
  if ($Gui) { $simArgs.Gui = $true } else { $simArgs.Headless = $true }
  & $simScript @simArgs
  $isaacExit = $LASTEXITCODE
  if ($isaacExit -ne 0) { throw "Isaac runtime failed with exit code $isaacExit." }

  $waitSeconds = [Math]::Max(120, [int]($captureHorizon * 4) + $ProgressTimeoutSeconds)
  $bagExit = Wait-ProcessWithTimeout $bag $waitSeconds "raw ROS bag writer"
  if ($bagExit -ne 0) { throw "Raw ROS bag writer failed with exit code $bagExit." }
  $bagMeta = Get-Content -LiteralPath (Join-Path $captureDir "bag_metadata.json") -Raw | ConvertFrom-Json
  if ($bagMeta.status -ne "complete") { throw "Raw bag capture did not complete." }
  if (-not [bool]$bagMeta.async_write_queue_integrity -or -not [bool]$bagMeta.async_write_queue.drained -or [int]$bagMeta.async_write_queue.overflow_count -ne 0 -or $null -ne $bagMeta.async_write_queue.worker_error) { throw "Raw bag asynchronous writer did not drain losslessly." }
  if (-not [bool]$bagMeta.camera_pairing_exact) { throw "Raw bag RGB and CameraInfo stamps are not paired exactly." }

  # Encode only after the raw bag is closed.  The video and timestamp index
  # are therefore derived from the exact accepted raw RGB messages instead of
  # competing as a second live 6 MiB ROS subscriber.
  $videoArgs = @(
    "run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","evaluation.rgb_video_from_bag",
    "--bag",$bagUri,"--bag-metadata",(Join-Path $captureDir "bag_metadata.json"),
    "--output",(Join-Path $captureDir "rgb_camera.mp4"),"--metadata",(Join-Path $captureDir "rgb_video.json"),
    "--frames-jsonl",(Join-Path $captureDir "rgb_frames.jsonl"),"--camera-info-json",(Join-Path $captureDir "camera_info.json"),
    "--duration-seconds",([string]$captureHorizon),"--width",([string]$expectedWidth),"--height",([string]$expectedHeight),"--fps",([string]$expectedFps)
  )
  $videoBuilder = Start-Process -FilePath $pixi -ArgumentList $videoArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "rgb.out.log") -RedirectStandardError (Join-Path $logsDir "rgb.err.log")
  $rgbExit = Wait-ProcessWithTimeout $videoBuilder $waitSeconds "offline RGB video builder"
  if ($rgbExit -ne 0) { throw "Offline RGB video builder failed with exit code $rgbExit." }
  if (-not (Test-Path -LiteralPath $simStatus)) { throw "Isaac runtime status receipt is missing." }
  $runtime = Get-Content -LiteralPath $simStatus -Raw | ConvertFrom-Json
  $rgb = Get-Content -LiteralPath (Join-Path $captureDir "rgb_video.json") -Raw | ConvertFrom-Json
  $cameraInfo = Get-Content -LiteralPath (Join-Path $captureDir "camera_info.json") -Raw | ConvertFrom-Json
  if ([int]$runtime.observed_rgb_frames -le 0 -or [int]$runtime.observed_clock_samples -le 0 -or [int]$runtime.observed_lidar_clouds -le 0) { throw "Isaac runtime status contains an empty required sensor stream." }
  if ([int]$runtime.lidar_cloud_points.sample_count -le 0 -or [int]$runtime.lidar_cloud_points.min_points -le 0) { throw "Isaac runtime status has no real RTX LiDAR returns." }
  if ([bool]$runtime.ground_truth_odometry_leakage) { throw "Isaac runtime reported ground-truth odometry leakage." }
  if (-not [bool]$runtime.pacing.enabled -or [Math]::Abs([double]$runtime.pacing.requested_realtime_factor - $RealtimeFactor) -gt 1e-9) { throw "Isaac runtime pacing receipt does not match the requested realtime factor." }
  if ([double]$runtime.timestamp_alignment.rgb.max_abs_offset_s -gt (1.0 / 60.0 + 1e-9) -or [double]$runtime.timestamp_alignment.lidar.max_abs_offset_s -gt (1.0 / 60.0 + 1e-9)) { throw "Isaac sensor timestamps are not aligned to /clock within one simulation tick." }
  if ($rgb.status -ne "complete") { throw "RGB capture did not complete." }
  if (-not [bool]$rgb.decoded_video_valid -or [int]$rgb.decoded_video.frame_count -ne [int]$rgb.frame_count) { throw "Closed RGB video decode audit did not match the source frame index." }
  if (-not (Test-Path -LiteralPath (Join-Path $captureDir "camera_info.json"))) { throw "CameraInfo was not captured." }
  $requiredTopics = @("/clock","/sim/camera/rgb/image_raw","/sim/camera/rgb/camera_info","/sim/lidar/points","/tf","/tf_static")
  $actualTopics = @($bagMeta.topics | Sort-Object)
  if (($actualTopics -join "|") -ne (($requiredTopics | Sort-Object) -join "|")) { throw "Raw bag topic contract must contain exactly the six required topics." }
  foreach ($topic in $requiredTopics) {
    $countProperty = $bagMeta.counts.PSObject.Properties[$topic]
    if ($null -eq $countProperty -or [int]$countProperty.Value -le 0) { throw "Raw bag topic is empty: $topic" }
  }
  if ([int]$bagMeta.counts.PSObject.Properties["/sim/lidar/points"].Value -lt 2) { throw "Raw bag requires at least two LiDAR scans." }
  if (-not [bool]$bagMeta.target_reached -or [Math]::Abs([double]$bagMeta.target_clock_s - $captureHorizon) -gt 1e-6 -or [double]$bagMeta.last_clock_s -lt $captureHorizon - 1e-3) { throw "Raw bag did not reach the requested capture horizon." }
  if ([int]$bagMeta.lidar_point_count_min -le 0) { throw "Raw bag contains an empty LiDAR cloud." }
  if ([double]$bagMeta.max_stamp_gap_s.PSObject.Properties["/sim/lidar/points"].Value -gt 0.2 + 1e-9) { throw "Raw bag LiDAR cadence gap exceeds 0.2 seconds." }
  $rawRgbTopic = "/sim/camera/rgb/image_raw"
  $rawRgbGapProperty = $bagMeta.max_stamp_gap_s.PSObject.Properties[$rawRgbTopic]
  if ($null -eq $rawRgbGapProperty -or $null -eq $rawRgbGapProperty.Value) { throw "Raw bag RGB cadence receipt is missing." }
  $rawRgbCadenceLimitS = (1.0 / $expectedFps) + 0.001
  if ([double]$rawRgbGapProperty.Value -gt $rawRgbCadenceLimitS + 1e-9) { throw "Raw bag RGB cadence gap exceeds the configured frame period plus 1 ms." }
  if ([double]$bagMeta.max_rgb_lidar_skew_s -gt 0.017000001) { throw "Raw bag RGB/LiDAR timestamp skew exceeds 17,000,001 ns." }
  if ([int]$bagMeta.stamp_nonincreasing.PSObject.Properties["/sim/camera/rgb/image_raw"].Value -ne 0) { throw "Raw bag RGB timestamps must be strictly increasing with no duplicates." }
  if ([int]$bagMeta.stamp_nonincreasing.PSObject.Properties["/sim/camera/rgb/camera_info"].Value -ne 0) { throw "Raw bag CameraInfo timestamps must be strictly increasing with no duplicates." }
  if (@($bagMeta.camera_info_frame_ids) -notcontains "camera_optical_frame") { throw "Observed CameraInfo frame is not camera_optical_frame." }
  if (@($bagMeta.lidar_frame_ids) -notcontains "lidar_link") { throw "Observed LiDAR frame is not lidar_link." }
  if (-not [bool]$rgb.cadence_contiguous -or [int]$rgb.invalid_frames -ne 0 -or [int]$rgb.nonincreasing_frames -ne 0) { throw "RGB video cadence is not contiguous and valid." }
  $rawRgbCount = [int]$bagMeta.counts.PSObject.Properties[$rawRgbTopic].Value
  $rawCameraInfoCount = [int]$bagMeta.counts.PSObject.Properties["/sim/camera/rgb/camera_info"].Value
  $videoRgbCount = [int]$rgb.frame_count
  # All subscribers start before Isaac and observe the same absolute horizon;
  # exact counts are required.  CameraInfo is the publisher-side cadence
  # witness, so an image-only raw drop cannot hide behind a boundary allowance.
  $rgbCountBoundaryTolerance = 0
  if ($rawRgbCount -ne $rawCameraInfoCount) { throw "Raw bag RGB image and CameraInfo counts do not match." }
  if ($rawRgbCount -ne $videoRgbCount) { throw "Raw bag RGB count does not reconcile with the RGB video frame count." }
  if ($rawRgbCount -ne [int]$runtime.observed_rgb_frames) { throw "Raw bag RGB count does not reconcile with Isaac's observed RGB callback count." }
  $rawRgbStampHash = [string]$bagMeta.stamp_sha256.PSObject.Properties[$rawRgbTopic].Value
  $cameraInfoStampHash = [string]$bagMeta.stamp_sha256.PSObject.Properties["/sim/camera/rgb/camera_info"].Value
  if ($rawRgbStampHash -ne $cameraInfoStampHash -or $rawRgbStampHash -ne [string]$rgb.stamp_sha256 -or $rawRgbStampHash -ne [string]$runtime.observed_rgb_stamp_sha256) { throw "Isaac, raw RGB, CameraInfo, and offline video stamp sequences do not match exactly." }
  $rawRgbFirstS = [double]$bagMeta.first_stamp_s.PSObject.Properties[$rawRgbTopic].Value
  $rawRgbLastS = [double]$bagMeta.last_stamp_s.PSObject.Properties[$rawRgbTopic].Value
  $rgbFirstBoundarySkewS = [Math]::Abs($rawRgbFirstS - [double]$rgb.first_image_stamp_s)
  $rgbLastBoundarySkewS = [Math]::Abs($rawRgbLastS - [double]$rgb.last_image_stamp_s)
  if ($rgbFirstBoundarySkewS -gt $rawRgbCadenceLimitS + 1e-9 -or $rgbLastBoundarySkewS -gt $rawRgbCadenceLimitS + 1e-9) { throw "Raw bag and RGB video timestamp windows do not reconcile at their boundaries." }
  if ([int]$rgb.width -ne $expectedWidth -or [int]$rgb.height -ne $expectedHeight -or [Math]::Abs([double]$rgb.nominal_fps - $expectedFps) -gt 1e-9) { throw "RGB video dimensions or cadence do not match the production sensor config." }
  if ([double]$rgb.first_image_stamp_s -gt 0.1 + 1e-6 -or [double]$rgb.last_image_stamp_s -lt $captureHorizon - 1e-3) { throw "RGB video does not cover the requested capture horizon." }
  if ($cameraInfo.frame_id -ne "camera_optical_frame" -or [int]$cameraInfo.width -ne $expectedWidth -or [int]$cameraInfo.height -ne $expectedHeight) { throw "Observed CameraInfo calibration does not match the RGB stream." }
  if (@($cameraInfo.k).Count -ne 9 -or [Math]::Abs([double]$cameraInfo.k[0] - $expectedFx) -gt 1e-3 -or [Math]::Abs([double]$cameraInfo.k[4] - $expectedFx) -gt 1e-3 -or [Math]::Abs([double]$cameraInfo.k[2] - $expectedCx) -gt 0.51 -or [Math]::Abs([double]$cameraInfo.k[5] - $expectedCy) -gt 0.51) { throw "Observed CameraInfo intrinsics do not match the configured 90-degree camera model." }

  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 bag info $bagUri | Set-Content -LiteralPath (Join-Path $captureDir "bag_info.txt") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "ros2 bag info failed for $bagUri." }
  Assert-CaptureSourceUnchanged -Repo $repo -ExpectedSha $gitSha -ExpectedTree $gitTree
  $files = @()
  Get-ChildItem -LiteralPath $captureDir -File -Recurse | Where-Object { $_.Name -notin @("capture_manifest.json") } | ForEach-Object {
    $relative = $_.FullName.Substring($captureDir.Length + 1).Replace("\", "/")
    $files += [ordered]@{ path=$relative; sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $_.FullName).Hash.ToLowerInvariant(); size_bytes=$_.Length }
  }
  $topics = @($bagMeta.topics)
  $manifest = [ordered]@{
    manifest_version=1; status="complete"; capture_id=$captureId; scenario=$scenarioPath; git_sha=$gitSha; git_tree=$gitTree; git_worktree_clean=$true
    rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID; duration_s=$captureHorizon; scenario_duration_s=$duration; frame_limit=$Frames
    pacing=$runtime.pacing
    zenoh=[ordered]@{
      session_config="config/ros2/production_zenoh_session.json5"; session_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohSessionConfig).Hash.ToLowerInvariant()
      router_config="config/ros2/production_zenoh_router.json5"; router_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $zenohRouterConfig).Hash.ToLowerInvariant()
    }
    bag=[ordered]@{
      uri="sensors_bag"; storage_id="sqlite3"; topics=$topics; counts=$bagMeta.counts; first_clock_s=$bagMeta.first_clock_s; last_clock_s=$bagMeta.last_clock_s
      rgb_reconciliation=[ordered]@{
        raw_count=$rawRgbCount; camera_info_count=$rawCameraInfoCount; video_count=$videoRgbCount; boundary_count_tolerance=$rgbCountBoundaryTolerance
        stamp_sha256=$rawRgbStampHash
        raw_max_gap_s=[double]$rawRgbGapProperty.Value; max_allowed_gap_s=$rawRgbCadenceLimitS
        first_boundary_skew_s=$rgbFirstBoundarySkewS; last_boundary_skew_s=$rgbLastBoundarySkewS
      }
    }
    rgb=[ordered]@{
      video="rgb_camera.mp4"; timestamp_index="rgb_frames.jsonl"; camera_info="camera_info.json"; metadata="rgb_video.json"
      frame_count=$rgb.frame_count; first_stamp_s=$rgb.first_image_stamp_s; last_stamp_s=$rgb.last_image_stamp_s
      width_px=$expectedWidth; height_px=$expectedHeight; fps=$expectedFps; camera_info_provenance="observed_ros_message"
      sensor_config=$sensorPath; sensor_config_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $sensorPath).Hash.ToLowerInvariant()
    }
    ground_truth=[ordered]@{ inventory_csv="inventory_ground_truth.csv"; inventory_json="inventory_ground_truth.json"; pose_topic="/sim/ground_truth/pose"; evaluation_only=$true }
    hashes=(Get-Content -LiteralPath (Join-Path $captureDir "experiment_hashes.json") -Raw | ConvertFrom-Json)
    software_versions="../software_versions.json"
    files=$files
  }
  $canonical = $manifest | ConvertTo-Json -Depth 20 -Compress
  $manifest.capture_sha256 = ([System.BitConverter]::ToString(([Security.Cryptography.SHA256]::Create().ComputeHash([Text.Encoding]::UTF8.GetBytes($canonical)))).Replace("-","")).ToLowerInvariant()
  $manifest | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $captureDir "capture_manifest.json") -Encoding UTF8
  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") python -m simulator.capture.manifest --validate-archive $captureDir | Set-Content -LiteralPath (Join-Path $logsDir "capture_archive_validation.json") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "Full capture archive validation failed." }
  Assert-CaptureSourceUnchanged -Repo $repo -ExpectedSha $gitSha -ExpectedTree $gitTree
  New-Item -ItemType File -Force -Path (Join-Path $captureDir "CAPTURE_COMPLETE") | Out-Null
  Write-Host "Capture complete: $captureDir"
} finally {
  foreach ($process in @($videoBuilder,$bag)) {
    if ($process -and -not $process.HasExited) { Stop-ProcessTree -RootPid $process.Id }
  }
  if ($router -and -not $router.HasExited) { Stop-ProcessTree -RootPid $router.Id }
  if (-not (Test-Path -LiteralPath (Join-Path $captureDir "CAPTURE_COMPLETE"))) {
    Remove-Item -LiteralPath (Join-Path $captureDir "capture_manifest.json") -Force -ErrorAction SilentlyContinue
  }
  Pop-Location
}
