param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [string]$IsaacPython = "C:\isaacsim\python.bat",
  [string]$Scenario = "config/scenarios/walking_baseline.yaml",
  [int]$Frames = 0,
  [switch]$Realtime,
  [switch]$Gui,
  [switch]$Headless
)
$ErrorActionPreference = "Stop"
if ($Gui -and $Headless) { throw "Choose either -Gui or -Headless, not both." }
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
. (Join-Path $PSScriptRoot "process_status.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$scenarioPath = if ([IO.Path]::IsPathRooted($Scenario)) { $Scenario } else { Join-Path $repo $Scenario }
if (-not (Test-Path -LiteralPath $scenarioPath)) { throw "Scenario file not found: $scenarioPath" }
if (-not (Test-Path -LiteralPath $IsaacPython)) { throw "Isaac Python launcher not found: $IsaacPython" }
$scenarioPath = (Resolve-Path -LiteralPath $scenarioPath).Path
$scenarioData = Get-Content -LiteralPath $scenarioPath -Raw | ConvertFrom-Json
$trajectoryPath = (Resolve-Path -LiteralPath (Join-Path (Split-Path $scenarioPath -Parent) $scenarioData.trajectory)).Path
$duration = [double](Get-Content -LiteralPath $trajectoryPath -Raw | ConvertFrom-Json).duration_s
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

function Stop-ProcessTree([int]$RootPid) {
  $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId = $RootPid" | Select-Object -ExpandProperty ProcessId)
  foreach ($childPid in $children) {
    Stop-ProcessTree -RootPid ([int]$childPid)
    Stop-Process -Id $childPid -Force -ErrorAction SilentlyContinue
  }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

Push-Location $repo
$router = $null
$bag = $null
$recorder = $null
$isaacExit = 1
try {
  & $IsaacPython -c "from simulator.capture.export_metadata import export_metadata; import json; print(json.dumps(export_metadata(r'$scenarioPath', r'$captureDir', r'$repo')))" | Set-Content -LiteralPath (Join-Path $logsDir "capture_metadata_export.json") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "Capture metadata export failed." }

  Copy-Item -LiteralPath (Join-Path $repo "config\contracts.yaml") -Destination (Join-Path $captureDir "contracts.yaml")
  Copy-Item -LiteralPath $scenarioPath -Destination (Join-Path $captureDir "scenario.yaml")
  $gitSha = (& git -C $repo rev-parse HEAD).Trim()
  [ordered]@{ capture_id=$captureId; git_sha=$gitSha; scenario=$scenarioPath; rmw=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID; started_utc=(Get-Date).ToUniversalTime().ToString("o"); duration_s=$duration } |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $captureDir "provenance.json") -Encoding UTF8
  [ordered]@{
    isaac_sim="6.1.0"; ros_distro="jazzy"; rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID
    pixi_path=$pixi; ros_workspace=$workspace; ros2_cli_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix ros2cli) -join " ").Trim()
    rtabmap_odom_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_odom) -join " ").Trim()
    rtabmap_slam_prefix=((& $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 pkg prefix rtabmap_slam) -join " ").Trim()
  } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runDir "software_versions.json") -Encoding UTF8

  $router = Start-Process -FilePath $pixi -ArgumentList @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"ros2","run","rmw_zenoh_cpp","rmw_zenohd") -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logsDir "zenoh.out.log") -RedirectStandardError (Join-Path $logsDir "zenoh.err.log")
  # Give rmw_zenohd time to become a real router before the first capture
  # subscriber is created.  A subscriber started during router bootstrap can
  # remain undiscoverable on Windows even though later ROS nodes connect.
  Start-Sleep -Seconds 6

  $bagUri = Join-Path $captureDir "sensors_bag"
  $bagArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.capture.rosbag_capture","--output",$bagUri,"--metadata",(Join-Path $captureDir "bag_metadata.json"),"--duration-seconds",([string]$duration),"--end-clock-seconds",([string]$duration),"--startup-timeout-seconds","120","--clock-stall-timeout-seconds","30")
  $bagStatus = Join-Path $logsDir "bag.exit-status.json"
  $bag = Start-TrackedProcess -FilePath $pixi -ArgumentList $bagArgs -WorkingDirectory $repo -StatusPath $bagStatus -RedirectStandardOutput (Join-Path $logsDir "bag.out.log") -RedirectStandardError (Join-Path $logsDir "bag.err.log")

  $recorderArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","evaluation.rgb_video_recorder","--output",(Join-Path $captureDir "rgb_camera.mp4"),"--metadata",(Join-Path $captureDir "rgb_video.json"),"--frames-jsonl",(Join-Path $captureDir "rgb_frames.jsonl"),"--duration-seconds",([string]$duration),"--startup-timeout-seconds","120")
  $recorderStatus = Join-Path $logsDir "rgb.exit-status.json"
  $recorder = Start-TrackedProcess -FilePath $pixi -ArgumentList $recorderArgs -WorkingDirectory $repo -StatusPath $recorderStatus -RedirectStandardOutput (Join-Path $logsDir "rgb.out.log") -RedirectStandardError (Join-Path $logsDir "rgb.err.log")
  Start-Sleep -Seconds 2

  $simScript = Join-Path $PSScriptRoot "run_sim.ps1"
  $simStatus = Join-Path $captureDir "isaac_runtime_status.json"
  if ($Gui) {
    if ($Realtime) {
      if ($Frames -gt 0) { & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Frames $Frames -Realtime -Gui }
      else { & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Realtime -Gui }
    } elseif ($Frames -gt 0) {
      & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Frames $Frames -Gui
    } else {
      & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Gui
    }
  } elseif ($Realtime) {
    if ($Frames -gt 0) { & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Frames $Frames -Realtime -Headless }
    else { & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Realtime -Headless }
  } elseif ($Frames -gt 0) {
    & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Frames $Frames -Headless
  } else {
    & $simScript -Scenario $scenarioPath -PixiPath $pixi -RosWorkspace $workspace -IsaacPython $IsaacPython -StatusPath $simStatus -Headless
  }
  $isaacExit = $LASTEXITCODE
  if ($isaacExit -ne 0) { throw "Isaac runtime failed with exit code $isaacExit." }

  $waitSeconds = [Math]::Max(180, [int]($duration * 15) + 60)
  $recorderExit = Wait-ProcessWithTimeout -Process $recorder -TimeoutSeconds $waitSeconds -Name "RGB capture recorder" -StatusPath $recorderStatus
  if ($recorderExit -ne 0) { throw "RGB capture recorder failed with exit code $recorderExit." }
  $bagExit = Wait-ProcessWithTimeout -Process $bag -TimeoutSeconds $waitSeconds -Name "raw ROS bag writer" -StatusPath $bagStatus
  if ($bagExit -ne 0) { throw "Raw ROS bag writer failed with exit code $bagExit." }
  $rgb = Get-Content -LiteralPath (Join-Path $captureDir "rgb_video.json") -Raw | ConvertFrom-Json
  $bagMeta = Get-Content -LiteralPath (Join-Path $captureDir "bag_metadata.json") -Raw | ConvertFrom-Json
  if ($rgb.status -ne "complete") { throw "RGB capture did not complete." }
  if ($bagMeta.status -ne "complete") { throw "Raw bag capture did not complete." }
  $cameraInfo = Get-Content -LiteralPath (Join-Path $captureDir "camera_info.json") -Raw | ConvertFrom-Json
  if ($cameraInfo.provenance -ne "configured_intrinsics" -or [bool]$cameraInfo.observed_ros_message) { throw "Configured camera intrinsics artifact is missing or has invalid provenance." }

  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") ros2 bag info $bagUri | Set-Content -LiteralPath (Join-Path $captureDir "bag_info.txt") -Encoding UTF8
  if ($LASTEXITCODE -ne 0) { throw "ros2 bag info failed for $bagUri." }
  $files = @()
  Get-ChildItem -LiteralPath $captureDir -File -Recurse | Where-Object { $_.Name -notin @("capture_manifest.json") } | ForEach-Object {
    $relative = $_.FullName.Substring($captureDir.Length + 1).Replace("\", "/")
    $files += [ordered]@{ path=$relative; sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $_.FullName).Hash.ToLowerInvariant(); size_bytes=$_.Length }
  }
  $topics = @($bagMeta.topics)
  $manifest = [ordered]@{
    manifest_version=2; status="complete"; capture_id=$captureId; scenario=$scenarioPath; git_sha=$gitSha
    rmw_implementation=$env:RMW_IMPLEMENTATION; ros_domain_id=[int]$env:ROS_DOMAIN_ID; duration_s=$duration
    bag=[ordered]@{ uri="sensors_bag"; storage_id="sqlite3"; topics=$topics; counts=$bagMeta.counts; first_clock_s=$bagMeta.first_clock_s; last_clock_s=$bagMeta.last_clock_s }
    rgb=[ordered]@{ video="rgb_camera.mp4"; timestamp_index="rgb_frames.jsonl"; camera_info="camera_info.json"; camera_info_provenance="configured_intrinsics"; metadata="rgb_video.json"; frame_count=$rgb.frame_count; first_stamp_s=$rgb.first_image_stamp_s; last_stamp_s=$rgb.last_image_stamp_s }
    ground_truth=[ordered]@{ inventory_csv="inventory_ground_truth.csv"; inventory_json="inventory_ground_truth.json"; pose_topic="/sim/ground_truth/pose"; evaluation_only=$true }
    hashes=(Get-Content -LiteralPath (Join-Path $captureDir "experiment_hashes.json") -Raw | ConvertFrom-Json)
    software_versions="../software_versions.json"
    files=$files
  }
  $canonical = $manifest | ConvertTo-Json -Depth 20 -Compress
  $manifest.capture_sha256 = ([System.BitConverter]::ToString(([Security.Cryptography.SHA256]::Create().ComputeHash([Text.Encoding]::UTF8.GetBytes($canonical)))).Replace("-","")).ToLowerInvariant()
  $manifest | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $captureDir "capture_manifest.json") -Encoding UTF8
  New-Item -ItemType File -Force -Path (Join-Path $captureDir "CAPTURE_COMPLETE") | Out-Null
  Write-Host "Capture complete: $captureDir"
} finally {
  foreach ($process in @($recorder,$bag)) {
    if ($process -and -not $process.HasExited) { Stop-ProcessTree -RootPid $process.Id }
  }
  if ($router -and -not $router.HasExited) { Stop-ProcessTree -RootPid $router.Id }
  Pop-Location
}
