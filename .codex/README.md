# Project agent configuration

Validated against Codex CLI `0.155.0-alpha.9.2`. Project config uses `[agents]`
defaults and standalone `.codex/agents/*.toml` files, as documented at
https://learn.chatgpt.com/docs/agent-configuration/subagents.

The current native collaboration API accepts explicit model/effort but has no
custom-role selector. For that API, spawn with `fork_turns="none"`, the exact
model and effort, and explicit instructions to read the shared role README and
assigned role. Do not assume full-history forks apply model overrides. Use the
custom role names when a future client exposes their selection directly, but
still pass explicit Sol/high routing when the spawn API supports it.

Defaults: Sol/high orchestrator, Sol/high workers, and a separate Sol/high
dedicated reviewer. A Sol implementation specialist must be a different session
from the reviewer. The configured six-child limit is a ceiling, not a promise:
the prior setup session exposed four total slots. Reserve a reviewer slot and
start with at most two implementation workers where capacity permits.

Start a fresh session from this repository to load the new defaults. The current
session can use explicit routing. Live permission overrides may supersede the
role sandbox defaults; record effective permissions and never call a full-access
reviewer technically sandboxed read-only.

## Fresh worktree bootstrap

Commit `d315aa9c21fa1bfb3aec284687c6ed4a49151941` made the canonical
specification, roles, references, and setup files part of the production
baseline. Worktrees created at that commit or any descendant must use their
committed copies directly.

Only worktrees based before `d315aa9c21fa1bfb3aec284687c6ed4a49151941`, or
worktrees where a required canonical file is missing, need the original
bootstrap procedure: copy only absent files, compare SHA-256 for existing files
and stop on a mismatch, then revalidate all manifest image hashes. Never
overwrite, auto-stage, or auto-commit pre-existing owner files during bootstrap.

Keep review snapshots detached at their submitted SHA. Setup instructions read
from outside that snapshot must have their hashes recorded in the task packet.
Use separate scratch/output paths. Route reviewer findings unchanged to the
assigned programmer; append revision commits and obtain a new exact-identity
verdict.
