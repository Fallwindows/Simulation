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
function Wait-ProcessWithTimeout($Process, [int]$TimeoutSeconds, [string]$Name) {
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while (-not $Process.HasExited -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
  if (-not $Process.HasExited) { throw "$Name did not finish within $TimeoutSeconds seconds." }
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
  Start-Sleep -Seconds 6
  $mapping = Start-Process -FilePath $pixi -ArgumentList $mappingArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_mapping.out.log") -RedirectStandardError (Join-Path $logsDir "offline_mapping.err.log")
  $nodeDeadline = (Get-Date).AddSeconds(90)
  do {
    Start-Sleep -Seconds 2
    $nodes = (& $pixi @baseArgs node list 2>$null) -join "`n"
  } while ((($nodes -notmatch "icp_odometry") -or ($nodes -notmatch "rtabmap")) -and (Get-Date) -lt $nodeDeadline)
  if (($nodes -notmatch "icp_odometry") -or ($nodes -notmatch "rtabmap")) { throw "RTAB-Map nodes did not become ready. Nodes: $nodes" }

  $observerArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.capture.slam_observer","--output-dir",$slamDir,"--duration-seconds",([string]$duration),"--startup-timeout-seconds","180")
  $observer = Start-Process -FilePath $pixi -ArgumentList $observerArgs -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_observer.out.log") -RedirectStandardError (Join-Path $logsDir "offline_observer.err.log")
  Start-Sleep -Seconds 1
  $player = Start-Process -FilePath $pixi -ArgumentList @($baseArgs + @("bag","play",$bagUri.Replace([char]92, "/"),"--clock")) -WorkingDirectory $repo -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "offline_bag_play.out.log") -RedirectStandardError (Join-Path $logsDir "offline_bag_play.err.log")
  $waitSeconds = [Math]::Max(240, [int]($duration * 20) + 120)
  Wait-ProcessWithTimeout $player $waitSeconds "bag replay" | Out-Null
  Wait-ProcessWithTimeout $observer $waitSeconds "offline SLAM observer" | Out-Null

  $publishRequest = "{global_map: true, optimized: true, graph_only: false}"
  & $pixi @baseArgs service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap $publishRequest | Set-Content -LiteralPath (Join-Path $slamDir "publish_map_response.txt") -Encoding UTF8
  $publishExit = $LASTEXITCODE
  Start-Sleep -Seconds 2
  if (Test-Path -LiteralPath (Join-Path $slamDir "slam_observer.json")) {
    $observerMeta = Get-Content -LiteralPath (Join-Path $slamDir "slam_observer.json") -Raw | ConvertFrom-Json
  } else { throw "SLAM observer metadata is missing." }
  if (-not $observerMeta.fresh_odom -or -not $observerMeta.fresh_map) { throw "Offline SLAM did not produce fresh odom and map data." }
  if (-not (Test-Path -LiteralPath $database)) { throw "RTAB-Map database was not created." }
  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") python (Join-Path $repo "scripts/validate_rtabmap_db.py") $database | Set-Content -LiteralPath (Join-Path $slamDir "database_validation.txt") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "RTAB-Map database validation failed." }
  [ordered]@{
    ros_distro="jazzy"; rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    ros2_cli_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix ros2cli) -join " ").Trim()
    rtabmap_odom_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_odom) -join " ").Trim()
    rtabmap_slam_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_slam) -join " ").Trim()
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $slamDir "software_versions.json") -Encoding UTF8
  [ordered]@{
    status="complete"; experiment=$ExperimentName; capture_id=$manifest.capture_id; capture_sha256=$manifest.capture_sha256
    git_sha=(& git -C $repo rev-parse HEAD).Trim(); rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    bag_replayed=$bagUri; ground_truth_subscribed=$false; publish_map_service_exit=$publishExit; database_path=$database
    observer=(Get-Content -LiteralPath (Join-Path $slamDir "slam_observer.json") -Raw | ConvertFrom-Json)
  } | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $slamDir "slam_manifest.json") -Encoding UTF8
  Write-Host "Offline SLAM complete: $slamDir"
} finally {
  foreach ($process in @($player,$observer)) { if ($process -and -not $process.HasExited) { Stop-ProcessTree -RootPid $process.Id } }
  if ($mapping -and -not $mapping.HasExited) { Stop-ProcessTree -RootPid $mapping.Id }
  if ($router -and -not $router.HasExited) { Stop-ProcessTree -RootPid $router.Id }
  Pop-Location
}
