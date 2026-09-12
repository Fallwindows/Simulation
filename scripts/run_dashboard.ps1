param(
  [string]$Python = "python",
  [int]$Port = 8080
)
$ErrorActionPreference = "Stop"
& $Python -m dashboard.backend.serve --host 127.0.0.1 --port $Port
if ($LASTEXITCODE -ne 0) { throw "Dashboard exited with code $LASTEXITCODE" }
