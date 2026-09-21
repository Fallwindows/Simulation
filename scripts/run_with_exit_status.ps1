param(
  [Parameter(Mandatory = $true)]
  [string]$PayloadBase64
)
$ErrorActionPreference = "Stop"
$payload = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($PayloadBase64)) | ConvertFrom-Json
$statusPath = [string]$payload.status_path
$temporaryStatus = "$statusPath.tmp.$PID"
$exitCode = $null
$available = $false
$failure = $null
try {
  $arguments = @($payload.arguments | ForEach-Object { [string]$_ })
  & ([string]$payload.executable) @arguments
  if ($null -eq $LASTEXITCODE) {
    $failure = "native child did not publish an exit code"
  } else {
    $exitCode = [int]$LASTEXITCODE
    $available = $true
  }
} catch {
  $failure = $_.Exception.Message
  $exitCode = 126
  $available = $true
  Write-Error $_
} finally {
  $directory = Split-Path -Parent $statusPath
  if ($directory) { New-Item -ItemType Directory -Force -Path $directory | Out-Null }
  [ordered]@{
    available = $available
    exit_code = $exitCode
    failure = $failure
    wrapper_pid = $PID
  } | ConvertTo-Json -Compress | Set-Content -LiteralPath $temporaryStatus -Encoding UTF8
  Move-Item -LiteralPath $temporaryStatus -Destination $statusPath -Force
}
if (-not $available) { exit 125 }
exit $exitCode
