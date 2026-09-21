# Sol setup audit

Audit time: 2026-09-21T05:24:16.9505612Z  
Scope: startup configuration and routing suitability only; no production review  
Reviewer session: `/root/sol_reviewer`  
Configured reviewer: `gpt-5.6-sol`, effort `high`  
Runtime-observed reviewer model/effort: `unknown` (the host did not expose per-session provenance)  
Configured sandbox: `read-only`  
Effective session policy observed from the parent context: unrestricted filesystem, approval policy `never`; reviewer source access remained read-only by conduct

## Identity

- Git base: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Git base tree: `5452e44e92cc0f27ae90fecaef3a995c99c8c28f`
- `AGENTS.md`: `3784169c42bdda1501cd1000ca1b4df997e75a8b249fe8d15409019b172728f0`
- Specification: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Role README: `c71575186a5f8dbff99289eb245a87e3bd938fe175619c337996f55131adda34`
- Reviewer role: `71afca1e2f0ae64c8be7719ce8f1e0a8ea2cbebaea30ebbf1ea62f3459836023`
- Project config: `105c954608b1afa45d714919b19cf422992ea863ffe19c83d0c5bfb8ab636b0d`
- Luna custom agent: `4fff1cbf0bd20cef3181c9cae08a9e9e33b93811c04314d97e535d7d0dd629d3`
- Sol custom agent: `fdc9f9632bfc8645d97229cc39182dc747387ffd31255e141b918e5197350ca4`
- Reference manifest: `7a96ab10aa1e4b7c9c36523137b3580af64f1c8480563cfb615618a625b2e69f`

## Verdict

`BLOCKED` for declaring the setup durable across fresh worktrees. The syntax and role design are suitable for the installed client, but the setup files are not part of the assigned Git base and are absent from the rehearsal worker worktree.

The current in-session rehearsal may proceed if every worker is explicitly directed to read the root copies and all candidate identity remains tied to the toy branch. This exception tests message routing and exact-snapshot review, not fresh-worktree discovery of committed project configuration.

## Verified checks

- Read `AGENTS.md`, `agent_roles/README.md`, `agent_roles/reviewer_agent.md`, and specification sections 3-4.
- Installed Codex CLI: `0.155.0-alpha.9.2`; desktop: `26.915.4065.0`.
- `codex doctor --json` reported configuration load `ok` and effective configured main model `gpt-6-astra` from this project invocation.
- Refreshed `codex debug models` catalog contained `gpt-6-astra`, `gpt-5.6-luna`, and `gpt-5.6-sol`. Requested `medium`/`high` efforts are supported for each applicable role.
- Official OpenAI subagent documentation confirms standalone project custom-agent files under `.codex/agents/`, the required `name`, `description`, and `developer_instructions` fields, explicit `model`/`model_reasoning_effort`, the `[agents]` keys used here, and live parent permission overrides: <https://developers.openai.com/codex/subagents/>.
- Root `AGENTS.md` correctly limits current work to setup/rehearsal and points to the authoritative role/spec documents.
- Rehearsal worker worktree `runs/setup-20260920/worker` was at base `d5e825c8...` on branch `codex/setup-rehearsal` and lacked all seven checked setup/input paths: `AGENTS.md`, the specification, both required role documents, project config, Luna agent config, and `references/manifest.json`.

## Findings

### SETUP-F01 — BLOCKER

Trigger: start a fresh worktree from the current assigned base.  
Consequence: project `AGENTS.md`, custom-agent routing, role instructions, specification, and reference manifest are unavailable inside that worktree, so their automatic discovery cannot be demonstrated and later work may run without the intended controls.  
Required behavior: commit/include the authoritative setup files in the base used for future worktrees, then verify discovery from a fresh worktree before production relies on the setup.  
Current rehearsal disposition: permitted only as an explicitly documented in-session exercise using the root copies.

### SETUP-F02 — NON_BLOCKING

The project config permits six spawned threads, while the live orchestration environment exposes four total concurrent slots including the parent. Scheduling must follow the observed live limit. The setup rehearsal fits within the available parent + Luna + Sol slots.

### SETUP-F03 — NON_BLOCKING

The Sol custom-agent file requests `read-only`, but this spawned review session inherited the parent's unrestricted/`never` policy. Official behavior allows live parent overrides. Record configured versus effective permissions for every review and preserve read-only source conduct; only authorized evidence and isolated review metadata may be written.

## Known limitations

- The installed tools exposed available model identifiers, configuration load, and requested effort support, but did not expose the current reviewer session's actual model or effort. Those remain `unknown`.
- `codex --strict-config features list` is not supported by this CLI subcommand, so validation used `codex doctor --json`, the refreshed model catalog, current official schema, and the fact that this session was successfully delegated.
- This packet does not approve a toy candidate or any production change. Each rehearsal candidate requires a separate exact-identity verdict.
