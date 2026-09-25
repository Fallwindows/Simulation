# Sol setup audit resolution

This packet supplements, without altering, `sol-setup-audit-20260921T052416Z.md`.

## Evidence added

- The installed app-server strict configuration inspection saved as `runs/setup-20260920/effective-config.json` has SHA-256 `dd975ebb7c49b06d6a1e81c39c0d92617524daee63f23eae1c9a1b79686b7dd0`.
- It records the project `.codex` layer as loaded, main model `gpt-6-astra`, effort `medium`, agents enabled, maximum configured spawned threads `6`, default subagent model `gpt-5.6-luna`, and default effort `high`.

## Finding disposition

`SETUP-F01`: resolved for the local setup/rehearsal scope by the orchestrator's explicit bootstrap contract:

1. Launch custom agents from the trusted repository root so the project `.codex` layer is discovered.
2. Give each agent absolute paths to the authoritative root `AGENTS.md`, specification, role files, and reference manifest, plus the distinct assigned task worktree path.
3. For a fresh task worktree that needs local copies, copy only absent setup targets from the verified root; if a target exists, compare hashes and stop on mismatch rather than overwriting it.
4. Record the copied/source hashes and effective runtime limitations in setup evidence.
5. Continue to treat candidate source identity as the commit/tree in the assigned worktree, independent of these setup-only bootstrap inputs.

This resolution preserves the supplied untracked files, requires no commit or publication in the current setup-only scope, and is sufficient to run the six-case local rehearsal. It does not yet demonstrate that a worktree created solely from Git independently contains the setup; production startup must apply the same verified bootstrap or use a later base that includes the files.

## Updated setup verdict

`PASS WITH RECORDED LIMITATIONS` for starting the local rehearsal.

Limitations remain:

- live concurrency is four total slots despite the project setting of six;
- effective child permissions may override custom-agent sandbox defaults;
- current reviewer runtime model/effort provenance remains `unknown`;
- every toy candidate, changed candidate, and moved-base combined snapshot still requires a separate exact-identity verdict.
