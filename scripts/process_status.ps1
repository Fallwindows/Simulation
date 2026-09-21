function Start-TrackedProcess {
  param(
    [Parameter(Mandatory = $true)][string]$FilePath,
    [Parameter(Mandatory = $true)][string[]]$ArgumentList,
    [Parameter(Mandatory = $true)][string]$WorkingDirectory,
    [Parameter(Mandatory = $true)][string]$RedirectStandardOutput,
    [Parameter(Mandatory = $true)][string]$RedirectStandardError,
    [Parameter(Mandatory = $true)][string]$StatusPath
  )
  Remove-Item -LiteralPath $StatusPath -Force -ErrorAction SilentlyContinue
  $runner = Join-Path $PSScriptRoot "run_with_exit_status.ps1"
  if (-not (Test-Path -LiteralPath $runner)) { throw "Exit-status runner not found: $runner" }
  $payload = [ordered]@{
    executable = $FilePath
    arguments = @($ArgumentList)
    status_path = $StatusPath
  } | ConvertTo-Json -Depth 5 -Compress
  $payloadBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($payload))
  $escapedRunner = $runner.Replace("'", "''")
  $command = "& '$escapedRunner' -PayloadBase64 '$payloadBase64'"
  $encodedCommand = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
  return Start-Process -FilePath "powershell.exe" `
    -ArgumentList @("-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-EncodedCommand", $encodedCommand) `
    -WorkingDirectory $WorkingDirectory -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput $RedirectStandardOutput -RedirectStandardError $RedirectStandardError
}

function Wait-ProcessWithTimeout {
  param(
    [Parameter(Mandatory = $true)]$Process,
    [Parameter(Mandatory = $true)][int]$TimeoutSeconds,
    [Parameter(Mandatory = $true)][string]$Name,
    [Parameter(Mandatory = $true)][string]$StatusPath
  )
  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while (-not $Process.HasExited -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 250 }
  if (-not $Process.HasExited) { throw "$Name did not finish within $TimeoutSeconds seconds." }
  [void]$Process.WaitForExit()
  if (-not (Test-Path -LiteralPath $StatusPath)) {
    throw "$Name exit status is unavailable: $StatusPath was not written."
  }
  try {
    $status = Get-Content -LiteralPath $StatusPath -Raw | ConvertFrom-Json
  } catch {
    throw "$Name exit status is unavailable: $StatusPath is invalid JSON."
  }
  if (-not [bool]$status.available -or $null -eq $status.exit_code) {
    throw "$Name exit status is unavailable: $($status.failure)"
  }
  return [int]$status.exit_code
}
