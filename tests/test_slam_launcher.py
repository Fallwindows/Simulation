import hashlib
import json
import os
import sqlite3
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
$deadline = if ($Mode -eq "ready") { [DateTime]::UtcNow.AddSeconds(2) } else { [DateTime]::UtcNow.AddSeconds(3) }
$perProbeMs = if ($Mode -eq "ready") { 1000 } else { 2500 }
$status = Wait-ForRosNodes -ExecutablePath (Join-Path $PSHOME "pwsh.exe") -ArgumentList $arguments -RequiredNodeNames @("/icp_odometry","/rtabmap") -DeadlineUtc $deadline -PerProbeTimeoutMilliseconds $perProbeMs -PollIntervalMilliseconds 0 -DiagnosticLogPath $DiagnosticPath -Phase $Mode
$recordedPids = if (Test-Path -LiteralPath $ChildPidPath) { @(Get-Content -LiteralPath $ChildPidPath | ForEach-Object { [int]$_ }) } else { @() }
$cleanupDeadline = [DateTime]::UtcNow.AddSeconds(2)
do {
  $leakedPids = @($recordedPids | Where-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue })
  if ($leakedPids.Count -gt 0) { Start-Sleep -Milliseconds 100 }
} while ($leakedPids.Count -gt 0 -and [DateTime]::UtcNow -lt $cleanupDeadline)
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

    def _run_attempt_harness(self, slam_directory: Path) -> dict:
        result_path = slam_directory.parent / f"attempt-result-{time.time_ns()}.json"
        harness = slam_directory.parent / f"attempt-harness-{time.time_ns()}.ps1"
        harness.write_text(
            r'''param(
  [string]$LauncherPath,
  [string]$SlamDirectory,
  [string]$ResultPath
)
$ErrorActionPreference = "Stop"
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($LauncherPath, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw ($parseErrors | ForEach-Object Message) -join "`n" }
foreach ($functionName in @("Assert-NoReparseDirectory","Assert-SafeDirectoryChain","Start-SlamAttempt")) {
  $definition = @($ast.FindAll({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $functionName }, $true))
  if ($definition.Count -ne 1) { throw "Expected one function definition for $functionName, found $($definition.Count)." }
  Invoke-Expression $definition[0].Extent.Text
}
$attempt = Start-SlamAttempt -SlamDirectory $SlamDirectory -ContainmentRoot $SlamDirectory
[ordered]@{
  attempt_id=$attempt.attempt_id
  attempt_directory=$attempt.attempt_directory
  prior_directory=$attempt.prior_directory
  database_path=$attempt.database_path
  rotated_prior_artifacts=@($attempt.rotated_prior_artifacts)
  manifest_exists=(Test-Path -LiteralPath (Join-Path $SlamDirectory "slam_manifest.json"))
  completion_temps=@(Get-ChildItem -LiteralPath $attempt.attempt_directory -Filter "completion-*.json.tmp" -ErrorAction SilentlyContinue | ForEach-Object FullName)
} | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ResultPath -Encoding UTF8
''',
            encoding="utf-8",
        )
        process = subprocess.run(
            [
                self.pwsh,
                "-NoProfile",
                "-File",
                str(harness),
                "-LauncherPath",
                str(LAUNCHER),
                "-SlamDirectory",
                str(slam_directory),
                "-ResultPath",
                str(result_path),
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
        )
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        result = json.loads(result_path.read_text(encoding="utf-8-sig"))
        harness.unlink()
        result_path.unlink()
        return result

    def _publish_attempt_harness(self, slam_directory: Path, attempt: dict) -> dict:
        result_path = slam_directory.parent / f"publish-result-{time.time_ns()}.json"
        harness = slam_directory.parent / f"publish-harness-{time.time_ns()}.ps1"
        harness.write_text(
            r'''param(
  [string]$LauncherPath,
  [string]$AttemptDatabase,
  [string]$CanonicalDatabase,
  [string]$ManifestPath,
  [string]$StagingDirectory,
  [string]$AttemptId,
  [string]$ResultPath
)
$ErrorActionPreference = "Stop"
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($LauncherPath, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw ($parseErrors | ForEach-Object Message) -join "`n" }
foreach ($functionName in @("Publish-ValidatedDatabase","Write-AtomicJson")) {
  $definition = @($ast.FindAll({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $functionName }, $true))
  if ($definition.Count -ne 1) { throw "Expected one function definition for $functionName, found $($definition.Count)." }
  Invoke-Expression $definition[0].Extent.Text
}
$publication = Publish-ValidatedDatabase -AttemptDatabasePath $AttemptDatabase -CanonicalDatabasePath $CanonicalDatabase
$payload = [ordered]@{
  status="complete"
  attempt_id=$AttemptId
  database_artifact=[ordered]@{path="rtabmap.db"; size_bytes=$publication.size_bytes; sha256=$publication.sha256}
}
Write-AtomicJson -Value $payload -DestinationPath $ManifestPath -StagingDirectory $StagingDirectory
[ordered]@{
  size_bytes=$publication.size_bytes
  sha256=$publication.sha256
  manifest_exists=(Test-Path -LiteralPath $ManifestPath)
  attempt_database_exists=(Test-Path -LiteralPath $AttemptDatabase)
  completion_temps=@(Get-ChildItem -LiteralPath $StagingDirectory -Filter "completion-*.json.tmp" -ErrorAction SilentlyContinue | ForEach-Object FullName)
} | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ResultPath -Encoding UTF8
''',
            encoding="utf-8",
        )
        process = subprocess.run(
            [
                self.pwsh,
                "-NoProfile",
                "-File",
                str(harness),
                "-LauncherPath",
                str(LAUNCHER),
                "-AttemptDatabase",
                attempt["database_path"],
                "-CanonicalDatabase",
                str(slam_directory / "rtabmap.db"),
                "-ManifestPath",
                str(slam_directory / "slam_manifest.json"),
                "-StagingDirectory",
                attempt["attempt_directory"],
                "-AttemptId",
                attempt["attempt_id"],
                "-ResultPath",
                str(result_path),
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
        )
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        result = json.loads(result_path.read_text(encoding="utf-8-sig"))
        harness.unlink()
        result_path.unlink()
        return result

    def _run_experiment_path_harness(self, run_directory: Path, experiment_name: str) -> dict:
        result_path = run_directory.parent / f"path-result-{time.time_ns()}.json"
        harness = run_directory.parent / f"path-harness-{time.time_ns()}.ps1"
        harness.write_text(
            r'''param(
  [string]$LauncherPath,
  [string]$RunDirectory,
  [string]$ExperimentName,
  [string]$ResultPath
)
$ErrorActionPreference = "Stop"
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($LauncherPath, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw ($parseErrors | ForEach-Object Message) -join "`n" }
foreach ($functionName in @("Assert-NoReparseDirectory","Assert-SafeDirectoryChain","Resolve-SafeSlamDirectory","Start-SlamAttempt")) {
  $definition = @($ast.FindAll({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $functionName }, $true))
  if ($definition.Count -ne 1) { throw "Expected one function definition for $functionName, found $($definition.Count)." }
  Invoke-Expression $definition[0].Extent.Text
}
$selection = $null
$attempt = $null
$failure = $null
try {
  $selection = Resolve-SafeSlamDirectory -RunDirectory $RunDirectory -RequestedExperimentName $ExperimentName
  New-Item -ItemType Directory -Force -Path $selection.slam_directory | Out-Null
  $attempt = Start-SlamAttempt -SlamDirectory $selection.slam_directory -ContainmentRoot $RunDirectory
} catch {
  $failure = $_.Exception.Message
}
[ordered]@{
  failure=$failure
  slam_root=$(if ($selection) { $selection.slam_root } else { $null })
  slam_directory=$(if ($selection) { $selection.slam_directory } else { $null })
  attempt_id=$(if ($attempt) { $attempt.attempt_id } else { $null })
} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $ResultPath -Encoding UTF8
''',
            encoding="utf-8",
        )
        process = subprocess.run(
            [
                self.pwsh,
                "-NoProfile",
                "-File",
                str(harness),
                "-LauncherPath",
                str(LAUNCHER),
                "-RunDirectory",
                str(run_directory),
                "-ExperimentName",
                experiment_name,
                "-ResultPath",
                str(result_path),
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
        )
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        result = json.loads(result_path.read_text(encoding="utf-8-sig"))
        harness.unlink()
        result_path.unlink()
        return result

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
        self.assertLess(elapsed, 7.0, "the absolute readiness deadline and cleanup wait must bound a hanging probe")
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

    def test_fresh_attempt_rejects_seeded_old_valid_database_when_current_has_no_nodes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            slam = root / "slam"
            slam.mkdir()
            old_database = slam / "rtabmap.db"
            connection = sqlite3.connect(old_database)
            with connection:
                connection.execute("create table Node (stamp real)")
                connection.executemany("insert into Node values (?)", [(index * 20.5 / 17,) for index in range(18)])
            connection.close()
            (slam / "slam_manifest.json").write_text(json.dumps({"status": "complete"}), encoding="utf-8")

            attempt = self._run_attempt_harness(slam)
            self.assertFalse((slam / "rtabmap.db").exists())
            self.assertFalse((slam / "slam_manifest.json").exists())
            self.assertEqual(set(attempt["rotated_prior_artifacts"]), {"rtabmap.db", "slam_manifest.json"})
            prior_database = Path(attempt["prior_directory"]) / "rtabmap.db"
            old_validation = subprocess.run(
                [
                    str(Path(os.environ.get("ISAAC_TEST_PYTHON", os.sys.executable))),
                    str(ROOT / "scripts/validate_rtabmap_db.py"),
                    str(prior_database),
                    "--minimum-node-stamp",
                    "20.5",
                    "--scan-period-seconds",
                    "0.1",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            self.assertEqual(old_validation.returncode, 0, old_validation.stdout + old_validation.stderr)
            old_receipt = json.loads(old_validation.stdout)
            self.assertEqual(old_receipt["node_count"], 18)
            self.assertAlmostEqual(old_receipt["last_node_stamp_s"], 20.5)

            current_database = Path(attempt["database_path"])
            connection = sqlite3.connect(current_database)
            with connection:
                connection.execute("create table Node (stamp real)")
            connection.close()
            current_validation = subprocess.run(
                [
                    str(Path(os.environ.get("ISAAC_TEST_PYTHON", os.sys.executable))),
                    str(ROOT / "scripts/validate_rtabmap_db.py"),
                    str(current_database),
                    "--minimum-node-stamp",
                    "20.5",
                    "--scan-period-seconds",
                    "0.1",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(current_validation.returncode, 0)
            self.assertIn("contains no Node rows", current_validation.stdout + current_validation.stderr)
            self.assertFalse((slam / "slam_manifest.json").exists(), "late validation failure must not restore stale authority")

    def test_prior_complete_manifest_is_absent_after_early_and_late_attempt_failure(self):
        for phase in ("early", "late"):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                slam = Path(directory) / "slam"
                slam.mkdir()
                (slam / "slam_manifest.json").write_text(json.dumps({"status": "complete", "source": "old"}), encoding="utf-8")
                attempt = self._run_attempt_harness(slam)
                self.assertFalse((slam / "slam_manifest.json").exists())
                self.assertTrue((Path(attempt["prior_directory"]) / "slam_manifest.json").is_file())
                if phase == "late":
                    (slam / "slam_observer.json").write_text(json.dumps({"status": "pending_database_validation"}), encoding="utf-8")
                    connection = sqlite3.connect(attempt["database_path"])
                    with connection:
                        connection.execute("create table Node (stamp real)")
                    connection.close()
                self.assertFalse((slam / "slam_manifest.json").exists())

    def test_fresh_attempt_can_publish_one_atomic_completion_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            slam = Path(directory) / "slam"
            slam.mkdir()
            attempt = self._run_attempt_harness(slam)
            attempt_database = Path(attempt["database_path"])
            connection = sqlite3.connect(attempt_database)
            with connection:
                connection.execute("create table Node (stamp real)")
                connection.executemany("insert into Node values (?)", [(index * 20.5 / 17,) for index in range(18)])
            connection.close()
            validation = subprocess.run(
                [
                    str(Path(os.environ.get("ISAAC_TEST_PYTHON", os.sys.executable))),
                    str(ROOT / "scripts/validate_rtabmap_db.py"),
                    str(attempt_database),
                    "--minimum-node-stamp",
                    "20.5",
                    "--scan-period-seconds",
                    "0.1",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            self.assertEqual(validation.returncode, 0, validation.stdout + validation.stderr)
            expected_bytes = attempt_database.read_bytes()
            publication = self._publish_attempt_harness(slam, attempt)
            self.assertTrue(publication["manifest_exists"])
            self.assertFalse(publication["attempt_database_exists"])
            self.assertEqual(publication["completion_temps"], [])
            self.assertEqual(publication["size_bytes"], len(expected_bytes))
            self.assertEqual(publication["sha256"], hashlib.sha256(expected_bytes).hexdigest())
            manifest = json.loads((slam / "slam_manifest.json").read_text(encoding="utf-8-sig"))
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["attempt_id"], attempt["attempt_id"])
            self.assertEqual(manifest["database_artifact"]["sha256"], publication["sha256"])

    def test_experiment_path_is_one_safe_contained_segment_before_attempt_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            run.mkdir()
            shared = root / "shared"
            rooted = root / "rooted-target"
            for target in (shared, rooted):
                target.mkdir()
                (target / "slam_manifest.json").write_text(json.dumps({"status": "complete", "sentinel": target.name}), encoding="utf-8")
                (target / "rtabmap.db").write_bytes((target.name + "-database").encode("utf-8"))
            sentinels = {
                path: (path.read_bytes(), path.stat().st_mtime_ns)
                for target in (shared, rooted)
                for path in (target / "slam_manifest.json", target / "rtabmap.db")
            }

            invalid_names = ("..", r"..\..\shared", str(rooted), "CON", "trailing.")
            for name in invalid_names:
                with self.subTest(name=name):
                    result = self._run_experiment_path_harness(run, name)
                    self.assertIsNotNone(result["failure"])
                    self.assertIn("safe", result["failure"].lower())
                    self.assertIsNone(result["attempt_id"])
            self.assertFalse((run / "slam").exists(), "invalid names must fail before output directory creation")
            for path, (expected_bytes, expected_mtime) in sentinels.items():
                self.assertEqual(path.read_bytes(), expected_bytes)
                self.assertEqual(path.stat().st_mtime_ns, expected_mtime)

            default = self._run_experiment_path_harness(run, "offline_slam")
            self.assertIsNone(default["failure"])
            self.assertEqual(Path(default["slam_directory"]), (run / "slam").resolve())
            self.assertIsNotNone(default["attempt_id"])
            named = self._run_experiment_path_harness(run, "named experiment")
            self.assertIsNone(named["failure"])
            self.assertEqual(Path(named["slam_directory"]), (run / "slam" / "named experiment").resolve())
            self.assertIsNotNone(named["attempt_id"])

    def test_reparse_redirects_are_rejected_before_attempt_writes(self):
        for location in ("slam_root", "selected_experiment", "attempts_directory"):
            with self.subTest(location=location), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                run = root / "run"
                run.mkdir()
                outside = root / f"outside-{location}"
                outside.mkdir()
                manifest = outside / "slam_manifest.json"
                database = outside / "rtabmap.db"
                manifest.write_text(json.dumps({"status": "complete", "sentinel": location}), encoding="utf-8")
                database.write_bytes((location + "-database").encode("utf-8"))
                sentinel_state = {
                    path: (hashlib.sha256(path.read_bytes()).hexdigest(), path.stat().st_mtime_ns)
                    for path in (manifest, database)
                }

                if location == "slam_root":
                    junction = run / "slam"
                    experiment = "offline_slam"
                elif location == "selected_experiment":
                    (run / "slam").mkdir()
                    junction = run / "slam" / "named"
                    experiment = "named"
                else:
                    (run / "slam").mkdir()
                    junction = run / "slam" / "attempts"
                    experiment = "offline_slam"
                environment = os.environ.copy()
                environment["SLAM_TEST_JUNCTION"] = str(junction)
                environment["SLAM_TEST_JUNCTION_TARGET"] = str(outside)
                created = subprocess.run(
                    [
                        self.pwsh,
                        "-NoProfile",
                        "-Command",
                        "New-Item -ItemType Junction -Path $env:SLAM_TEST_JUNCTION -Target $env:SLAM_TEST_JUNCTION_TARGET | Out-Null",
                    ],
                    cwd=ROOT,
                    env=environment,
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                self.assertEqual(created.returncode, 0, created.stdout + created.stderr)
                self.assertTrue(os.path.isjunction(junction), "fixture must create a real Windows junction")
                try:
                    result = self._run_experiment_path_harness(run, experiment)
                    self.assertIsNotNone(result["failure"])
                    self.assertIn("reparse point", result["failure"])
                    self.assertIsNone(result["attempt_id"])
                    for path, (expected_hash, expected_mtime) in sentinel_state.items():
                        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected_hash)
                        self.assertEqual(path.stat().st_mtime_ns, expected_mtime)
                    self.assertFalse((outside / "attempts").exists())
                finally:
                    os.rmdir(junction)


if __name__ == "__main__":
    unittest.main()
