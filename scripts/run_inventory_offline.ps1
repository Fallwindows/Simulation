param(
  [Alias("Run")]
  [string]$RunDir = "",
  [string]$CaptureDir = "",
  [string]$PixiPath = "",
  [string]$RosWorkspace = ""
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
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
$preflightArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.perception.rgb_tracking","--capture-dir",$capture,"--slam-dir",$slam,"--output-dir",$perception,"--repo-root",$repo,"--validate-inputs-only")
Push-Location $repo
try {
  & $pixi @preflightArgs
  if ($LASTEXITCODE -ne 0) { throw "Offline RGB perception input provenance failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
New-Item -ItemType Directory -Force -Path $perception | Out-Null
$args = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.perception.rgb_tracking","--capture-dir",$capture,"--slam-dir",$slam,"--output-dir",$perception,"--repo-root",$repo)
Push-Location $repo
try {
  & $pixi @args
  if ($LASTEXITCODE -ne 0) { throw "Offline RGB perception failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
$result = Get-Content -LiteralPath (Join-Path $perception "perception_manifest.json") -Raw | ConvertFrom-Json
if ($result.status -ne "complete" -or -not (Test-Path -LiteralPath (Join-Path $perception "estimated_inventory.csv"))) { throw "Offline RGB perception did not produce a complete estimate." }
$evaluationArgs = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.perception.inventory_evaluation","--capture-dir",$capture,"--slam-dir",$slam,"--perception-dir",$perception,"--repo-root",$repo)
Push-Location $repo
try {
  & $pixi @evaluationArgs
  if ($LASTEXITCODE -ne 0) { throw "Inventory evaluation failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
if (-not (Test-Path -LiteralPath (Join-Path $perception "inventory.xlsx")) -or -not (Test-Path -LiteralPath (Join-Path $slam "slam_map_with_inventory.ply"))) { throw "Inventory evaluation did not produce the spreadsheet and inventory map." }
Write-Host "Offline RGB perception and post-estimation inventory evaluation complete: $perception"
