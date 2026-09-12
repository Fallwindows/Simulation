param(
  [string]$Python = "python",
  [string]$Scenario = "config/scenarios/baseline_straight.yaml",
  [switch]$PreflightOnly
)
$ErrorActionPreference = "Stop"
if ($PreflightOnly) {
  & $Python -m simulator.runtime.sim_runner --scenario $Scenario
  exit $LASTEXITCODE
}
Write-Warning "Isaac Sim runtime launch is target-machine specific. Running deterministic preflight; use the Isaac adapter from the supported host integration."
& $Python -m simulator.runtime.sim_runner --scenario $Scenario
if ($LASTEXITCODE -ne 0) { throw "Simulation preflight exited with code $LASTEXITCODE" }
