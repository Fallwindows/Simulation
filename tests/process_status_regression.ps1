$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
. (Join-Path $root "scripts\process_status.ps1")
$temporary = Join-Path ([IO.Path]::GetTempPath()) ("grocery-process-status-" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Force -Path $temporary | Out-Null
try {
  $child = Join-Path $temporary "child.ps1"
  @'
param([int]$Code)
Write-Output "stdout-$Code"
[Console]::Error.WriteLine("stderr-$Code")
exit $Code
'@ | Set-Content -LiteralPath $child -Encoding UTF8

  foreach ($expected in @(0, 7)) {
    $stdout = Join-Path $temporary "$expected.out.txt"
    $stderr = Join-Path $temporary "$expected.err.txt"
    $status = Join-Path $temporary "$expected.status.json"
    $process = Start-TrackedProcess -FilePath "powershell.exe" `
      -ArgumentList @("-NoProfile", "-NonInteractive", "-File", $child, "-Code", [string]$expected) `
      -WorkingDirectory $temporary -RedirectStandardOutput $stdout -RedirectStandardError $stderr -StatusPath $status
    $actual = Wait-ProcessWithTimeout -Process $process -TimeoutSeconds 10 -Name "fixture $expected" -StatusPath $status
    if ($actual -ne $expected) { throw "Expected exit $expected, received $actual." }
    if ((Get-Content -LiteralPath $stdout -Raw) -notmatch "stdout-$expected") { throw "stdout was not redirected for $expected." }
    if ((Get-Content -LiteralPath $stderr -Raw) -notmatch "stderr-$expected") { throw "stderr was not redirected for $expected." }
  }

  $missingStatus = Join-Path $temporary "missing.status.json"
  $direct = Start-Process -FilePath "powershell.exe" -ArgumentList @("-NoProfile", "-Command", "exit 0") -PassThru -WindowStyle Hidden
  $unavailableRejected = $false
  try {
    Wait-ProcessWithTimeout -Process $direct -TimeoutSeconds 10 -Name "missing status fixture" -StatusPath $missingStatus | Out-Null
  } catch {
    $unavailableRejected = $_.Exception.Message -match "unavailable"
  }
  if (-not $unavailableRejected) { throw "Unavailable status was not rejected." }
  Write-Output "process status regression passed: zero=0 nonzero=7 unavailable=rejected"
} finally {
  Remove-Item -LiteralPath $temporary -Recurse -Force -ErrorAction SilentlyContinue
}
