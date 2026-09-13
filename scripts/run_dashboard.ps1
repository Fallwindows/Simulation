param(
  [string]$PixiPath = "",
  [string]$RosWorkspace = "",
  [int]$Port = 8080
)
$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "resolve_runtime_paths.ps1")
$pixi = Resolve-PixiExecutable $PixiPath
$workspace = Resolve-RosWorkspace $RosWorkspace
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\")).Path
$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_DOMAIN_ID = "0"
Push-Location $repo
try {
  & $pixi run --manifest-path (Join-Path $workspace "pixi.toml") python -m dashboard.backend.serve --host 127.0.0.1 --port $Port
} finally {
  Pop-Location
}
if ($LASTEXITCODE -ne 0) { throw "Dashboard exited with code $LASTEXITCODE" }
