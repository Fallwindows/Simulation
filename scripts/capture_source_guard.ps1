function Assert-CaptureSourceUnchanged(
  [Parameter(Mandatory=$true)][string]$Repo,
  [Parameter(Mandatory=$true)][string]$ExpectedSha,
  [Parameter(Mandatory=$true)][string]$ExpectedTree
) {
  $currentSha = (& git -C $Repo rev-parse HEAD).Trim()
  $currentTree = (& git -C $Repo rev-parse 'HEAD^{tree}').Trim()
  $currentStatus = @(& git -C $Repo status --porcelain=v1 2>$null)
  if ($LASTEXITCODE -ne 0 -or $currentSha -ne $ExpectedSha -or $currentTree -ne $ExpectedTree -or $currentStatus.Count -ne 0) {
    throw "Capture source changed or became dirty during the run."
  }
}
