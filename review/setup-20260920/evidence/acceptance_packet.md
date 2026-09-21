# Fixed setup rehearsal acceptance packet

- Task: `setup-rehearsal`
- Owner: GPT-5.6 Luna programmer, configured effort `high`
- Runtime model/effort observed: `unknown`
- Repository baseline: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Worktree: `runs/setup-20260920/worker`
- Branch: `codex/setup-rehearsal`
- Acceptance specification SHA-256: `56d0b16f8be409da4ff40aad9431bca1bee4042b9cbbbe36f33319007cc1e3ca`
- Governing implementation specification SHA-256: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Relevant input packet SHA-256: `9cbc92fd500f03dfc3b6863af02a18cd5c17d95d7a071095e81d700e98056a0e`

## Frozen behavior

`Get-FrameCount(seconds, fps)` is a simple PowerShell function that returns
`seconds * fps` for every nonnegative integer `seconds` and positive integer
`fps`. Required fixed cases are `0 * 30 = 0`, `1 * 30 = 30`, and
`45 * 30 = 1350`. Tests must execute all three cases.

## Required workflow proof

Revision 1 deliberately contains a `+1` off-by-one defect and must be
rejected by the dedicated Sol reviewer. Luna then appends a corrective commit,
retests, and resubmits the exact new snapshot. After approval, a later toy edit
must be shown to invalidate the old approval through a stale-approval check.

This packet is fixed before implementation. The inaccessible-evidence and
changed-integration-base variants are tracked separately by the parent task.
