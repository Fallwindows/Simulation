param(
  [string]$RunDir = "",
  [string]$CaptureDir = ""
)
$ErrorActionPreference = "Stop"
if (-not $RunDir -and -not $CaptureDir) { throw "Pass -RunDir or -CaptureDir." }
if (-not $CaptureDir) { $CaptureDir = Join-Path (Resolve-Path $RunDir).Path "capture" }
$capture = (Resolve-Path -LiteralPath $CaptureDir).Path
if (-not $RunDir) { $RunDir = Split-Path -Parent $capture }
$run = (Resolve-Path -LiteralPath $RunDir).Path
$manifest = Get-Content -LiteralPath (Join-Path $capture "capture_manifest.json") -Raw | ConvertFrom-Json
if ($manifest.status -ne "complete") { throw "Capture is not complete." }
$slam = Join-Path $run "slam"
$slamManifestPath = Join-Path $slam "slam_manifest.json"
if (-not (Test-Path -LiteralPath $slamManifestPath)) { throw "Offline SLAM manifest is missing: $slamManifestPath" }
$slamManifest = Get-Content -LiteralPath $slamManifestPath -Raw | ConvertFrom-Json
if ($slamManifest.status -ne "complete") { throw "Offline SLAM is not complete." }
$perception = Join-Path $run "perception"
New-Item -ItemType Directory -Force -Path $perception | Out-Null
[ordered]@{
  status="interface_ready"; detector_status="not_implemented"; estimated_inventory_written=$false
  capture_id=$manifest.capture_id; capture_sha256=$manifest.capture_sha256; slam_manifest="../slam/slam_manifest.json"
  inputs=[ordered]@{ rgb_video="../capture/rgb_camera.mp4"; rgb_frames="../capture/rgb_frames.jsonl"; camera_info="../capture/camera_info.json"; slam_poses="../slam/slam_poses.csv"; slam_map="../slam/slam_map.pcd" }
  ground_truth_consumed=$false; ground_truth_required=$false
} | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath (Join-Path $perception "perception_manifest.json") -Encoding UTF8
Write-Host "Perception interface validated; detector/tracker is intentionally not implemented and no fake inventory was written."
