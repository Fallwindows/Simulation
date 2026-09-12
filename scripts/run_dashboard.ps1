param(
  [string]$Python = "C:\Users\suyog\AppData\Local\pixi\bin\pixi.exe",
  [int]$Port = 8080
)
$ErrorActionPreference = "Stop"
if ($Python -like "*pixi.exe") {
  & $Python run --manifest-path "C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml" python -m dashboard.backend.serve --host 127.0.0.1 --port $Port
} else {
  & $Python -m dashboard.backend.serve --host 127.0.0.1 --port $Port
}
if ($LASTEXITCODE -ne 0) { throw "Dashboard exited with code $LASTEXITCODE" }
