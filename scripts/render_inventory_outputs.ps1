param(
  [string]$RunDir = "",
  [string]$PixiPath = "",
  [string]$RosWorkspace = ""
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
if (-not $RunDir) { throw "Pass -RunDir." }
$run = (Resolve-Path -LiteralPath $RunDir).Path
$manifestPath = Join-Path $run "perception\perception_manifest.json"
if (-not (Test-Path -LiteralPath $manifestPath)) { throw "Perception manifest not found." }
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if ($manifest.status -ne "complete") { throw "No completed perception result is available." }
$args = @("run","--manifest-path",(Join-Path $workspace "pixi.toml"),"python","-m","simulator.perception.render_video","--run-dir",$run)
Push-Location $repo
try {
  & $pixi @args
  if ($LASTEXITCODE -ne 0) { throw "Inventory output rendering failed with exit code $LASTEXITCODE." }
} finally { Pop-Location }
if (-not (Test-Path -LiteralPath (Join-Path $run "outputs\clean_walkthrough.mp4")) -or -not (Test-Path -LiteralPath (Join-Path $run "outputs\tracking_overlay.mp4"))) { throw "Expected clean and annotated MP4 outputs are missing." }
Write-Host "Inventory outputs rendered: $(Join-Path $run 'outputs')"
