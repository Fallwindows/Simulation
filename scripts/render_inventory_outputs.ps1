param([string]$RunDir = "")
$ErrorActionPreference = "Stop"
if (-not $RunDir) { throw "Pass -RunDir." }
$run = (Resolve-Path -LiteralPath $RunDir).Path
$manifestPath = Join-Path $run "perception\perception_manifest.json"
if (-not (Test-Path -LiteralPath $manifestPath)) { throw "Perception manifest not found." }
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if ($manifest.status -ne "complete") { throw "No completed perception result is available; refusing to render fake inventory output." }
throw "Inventory output rendering is pending the real detector/tracker implementation."
