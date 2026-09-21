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

function Stop-BoundedProcessTree {
  param([Parameter(Mandatory = $true)][int]$RootPid)
  $children = @(
    Get-CimInstance Win32_Process -Filter "ParentProcessId = $RootPid" -ErrorAction SilentlyContinue |
      Select-Object -ExpandProperty ProcessId
  )
  foreach ($childPid in $children) {
    Stop-BoundedProcessTree -RootPid ([int]$childPid)
  }
  Stop-Process -Id $RootPid -Force -ErrorAction SilentlyContinue
}

function Invoke-BoundedProcess {
  param(
    [Parameter(Mandatory = $true)][string]$FilePath,
    [Parameter(Mandatory = $true)][string[]]$ArgumentList,
    [Parameter(Mandatory = $true)][string]$WorkingDirectory,
    [Parameter(Mandatory = $true)][int]$TimeoutSeconds,
    [Parameter(Mandatory = $true)][string]$Name,
    [Parameter(Mandatory = $true)][string]$RedirectStandardOutput,
    [Parameter(Mandatory = $true)][string]$RedirectStandardError
  )
  if ($TimeoutSeconds -le 0) { throw "$Name requires a positive timeout." }
  Remove-Item -LiteralPath $RedirectStandardOutput,$RedirectStandardError -Force -ErrorAction SilentlyContinue
  $process = Start-Process -FilePath $FilePath -ArgumentList $ArgumentList `
    -WorkingDirectory $WorkingDirectory -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput $RedirectStandardOutput -RedirectStandardError $RedirectStandardError
  try {
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    while (-not $process.HasExited -and (Get-Date) -lt $deadline) {
      Start-Sleep -Milliseconds 100
      $process.Refresh()
    }
    if (-not $process.HasExited) {
      Stop-BoundedProcessTree -RootPid $process.Id
      try { [void]$process.WaitForExit(5000) } catch {}
      throw "$Name timed out after $TimeoutSeconds seconds; stdout=$RedirectStandardOutput stderr=$RedirectStandardError"
    }
    [void]$process.WaitForExit()
    $stdout = if (Test-Path -LiteralPath $RedirectStandardOutput) {
      Get-Content -LiteralPath $RedirectStandardOutput -Raw -ErrorAction SilentlyContinue
    } else { "" }
    $stderr = if (Test-Path -LiteralPath $RedirectStandardError) {
      Get-Content -LiteralPath $RedirectStandardError -Raw -ErrorAction SilentlyContinue
    } else { "" }
    return [pscustomobject]@{
      ExitCode = [int]$process.ExitCode
      Stdout = [string]$stdout
      Stderr = [string]$stderr
      StdoutPath = $RedirectStandardOutput
      StderrPath = $RedirectStandardError
    }
  } finally {
    if (-not $process.HasExited) { Stop-BoundedProcessTree -RootPid $process.Id }
  }
}
