import json
import os
import shutil
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts/run_slam_offline.ps1"


class SlamLauncherReadinessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pwsh = shutil.which("pwsh")
        if not cls.pwsh:
            raise unittest.SkipTest("PowerShell is unavailable")

    def _run_probe_harness(self, worker_source: str, mode: str) -> tuple[dict, float, list[dict]]:
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            worker = scratch / "node-list-worker.ps1"
            worker.write_text(worker_source, encoding="utf-8")
            result_path = scratch / "result.json"
            child_pids = scratch / "child-pids.txt"
            diagnostic = scratch / "readiness.jsonl"
            harness = scratch / "probe-harness.ps1"
            harness.write_text(
                r'''param(
  [string]$LauncherPath,
  [string]$WorkerPath,
  [string]$Mode,
  [string]$ResultPath,
  [string]$DiagnosticPath,
  [string]$ChildPidPath
)
$ErrorActionPreference = "Stop"
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($LauncherPath, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw ($parseErrors | ForEach-Object Message) -join "`n" }
foreach ($functionName in @("Stop-ProcessTree","Invoke-ProcessProbe","Test-RequiredRosNodes","Wait-ForRosNodes")) {
  $definition = @($ast.FindAll({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $functionName }, $true))
  if ($definition.Count -ne 1) { throw "Expected one function definition for $functionName, found $($definition.Count)." }
  Invoke-Expression $definition[0].Extent.Text
}
$env:SLAM_TEST_CHILD_PIDS = $ChildPidPath
$workerArgument = '"' + $WorkerPath + '"'
$arguments = @("-NoProfile","-File",$workerArgument)
$deadline = if ($Mode -eq "ready") { [DateTime]::UtcNow.AddSeconds(2) } else { [DateTime]::UtcNow.AddMilliseconds(450) }
$perProbeMs = if ($Mode -eq "ready") { 1000 } else { 400 }
$status = Wait-ForRosNodes -ExecutablePath (Join-Path $PSHOME "pwsh.exe") -ArgumentList $arguments -RequiredNodeNames @("/icp_odometry","/rtabmap") -DeadlineUtc $deadline -PerProbeTimeoutMilliseconds $perProbeMs -PollIntervalMilliseconds 0 -DiagnosticLogPath $DiagnosticPath -Phase $Mode
Start-Sleep -Milliseconds 300
$recordedPids = if (Test-Path -LiteralPath $ChildPidPath) { @(Get-Content -LiteralPath $ChildPidPath | ForEach-Object { [int]$_ }) } else { @() }
$leakedPids = @($recordedPids | Where-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue })
[ordered]@{
  ready=$status.ready
  attempts=$status.attempts
  last_probe=$status.last_probe
  recorded_child_pids=@($recordedPids)
  leaked_child_pids=@($leakedPids)
} | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ResultPath -Encoding UTF8
''',
                encoding="utf-8",
            )
            started = time.monotonic()
            process = subprocess.run(
                [
                    self.pwsh,
                    "-NoProfile",
                    "-File",
                    str(harness),
                    "-LauncherPath",
                    str(LAUNCHER),
                    "-WorkerPath",
                    str(worker),
                    "-Mode",
                    mode,
                    "-ResultPath",
                    str(result_path),
                    "-DiagnosticPath",
                    str(diagnostic),
                    "-ChildPidPath",
                    str(child_pids),
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=15,
            )
            elapsed = time.monotonic() - started
            self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
            result = json.loads(result_path.read_text(encoding="utf-8-sig"))
            diagnostics = [json.loads(line) for line in diagnostic.read_text(encoding="utf-8-sig").splitlines()]
            return result, elapsed, diagnostics

    def test_launcher_parses_without_powershell_errors(self):
        command = (
            "$tokens=$null; $errors=$null; "
            "[void][System.Management.Automation.Language.Parser]::ParseFile($env:SLAM_LAUNCHER_PATH,[ref]$tokens,[ref]$errors); "
            "if($errors.Count){$errors | ForEach-Object Message; exit 1}"
        )
        environment = os.environ.copy()
        environment["SLAM_LAUNCHER_PATH"] = str(LAUNCHER)
        result = subprocess.run(
            [self.pwsh, "-NoProfile", "-Command", command],
            cwd=ROOT,
            env=environment,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_hanging_node_list_is_bounded_rejected_and_has_no_leaked_child(self):
        result, elapsed, diagnostics = self._run_probe_harness(
            r'''$child = Start-Process -FilePath (Join-Path $PSHOME "pwsh.exe") -ArgumentList @("-NoProfile","-Command","Start-Sleep -Seconds 60") -WindowStyle Hidden -PassThru
Add-Content -LiteralPath $env:SLAM_TEST_CHILD_PIDS -Value $child.Id
Write-Output "/icp_odometry"
Write-Output "/rtabmap"
Wait-Process -Id $child.Id
''',
            "hang",
        )
        self.assertFalse(result["ready"], "partial stdout from a timed-out probe must not satisfy readiness")
        self.assertLess(elapsed, 5.0, "the absolute readiness deadline must bound a hanging probe")
        self.assertGreaterEqual(len(result["recorded_child_pids"]), 1, "fixture must spawn a real child")
        self.assertEqual(result["leaked_child_pids"], [])
        self.assertGreaterEqual(result["attempts"], 1)
        self.assertTrue(all(item["timed_out"] for item in diagnostics))
        self.assertTrue(all(not item["ready"] for item in diagnostics))

    def test_ready_node_list_passes_and_records_diagnostic(self):
        result, elapsed, diagnostics = self._run_probe_harness(
            'Write-Output "/icp_odometry"\nWrite-Output "/rtabmap"\nexit 0\n',
            "ready",
        )
        self.assertTrue(result["ready"])
        self.assertLess(elapsed, 5.0)
        self.assertEqual(result["leaked_child_pids"], [])
        self.assertEqual(result["attempts"], 1)
        self.assertEqual(len(diagnostics), 1)
        self.assertTrue(diagnostics[0]["ready"])
        self.assertFalse(diagnostics[0]["timed_out"])
        self.assertEqual(diagnostics[0]["exit_code"], 0)
        self.assertEqual(diagnostics[0]["observed_nodes"], ["/icp_odometry", "/rtabmap"])


if __name__ == "__main__":
    unittest.main()
