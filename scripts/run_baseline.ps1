param(
  [string]$Python = "python",
  [int]$Port = 8080
)
$ErrorActionPreference = "Stop"
$logRoot = Join-Path (Get-Location) "runs\launcher-logs"
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
$dashboard = Start-Process -FilePath $Python -ArgumentList @("-m", "dashboard.backend.serve", "--host", "127.0.0.1", "--port", "$Port") -PassThru -RedirectStandardOutput (Join-Path $logRoot "dashboard.out.log") -RedirectStandardError (Join-Path $logRoot "dashboard.err.log")
try {
  & $Python -m simulator.runtime.sim_runner --scenario "config/scenarios/baseline_straight.yaml"
  if ($LASTEXITCODE -ne 0) { throw "Simulation preflight failed with code $LASTEXITCODE" }
  Write-Host "Dashboard PID: $($dashboard.Id). Use scripts/run_mapping.ps1 separately on a ROS 2 host."
  Wait-Process -Id $dashboard.Id
} finally {
  if ($dashboard -and -not $dashboard.HasExited) { Stop-Process -Id $dashboard.Id -Force }
}
