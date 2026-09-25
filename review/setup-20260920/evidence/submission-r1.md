# Luna submission packet — revision 1

- Task: `setup-rehearsal`
- Revision: `r1`
- Reviewer: dedicated GPT-5.6 Sol, configured effort `high`; observed runtime: `unknown`
- Base SHA: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Candidate SHA: `5331026bcca4c9fe90e553bd38863f0fb1d30d9b`
- Candidate tree SHA: `684cbc8941f5e4ce6ef0c6f7ae6d034065a5e9d5`
- Governing spec SHA-256: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Acceptance spec SHA-256: `56d0b16f8be409da4ff40aad9431bca1bee4042b9cbbbe36f33319007cc1e3ca`
- Input packet SHA-256: `9cbc92fd500f03dfc3b6863af02a18cd5c17d95d7a071095e81d700e98056a0e`
- Worktree snapshot: `runs/setup-20260920/worker`
- Owned files: `setup_rehearsal/Get-FrameCount.ps1`, `setup_rehearsal/Get-FrameCount.Tests.ps1`

## Candidate

The function intentionally contains the requested revision-1 `+1` off-by-one
defect. The tests cover all three fixed input cases and therefore expose it.

## Test evidence

Command, run from the detached candidate worktree:

```text
pwsh -NoProfile -File .\setup_rehearsal\Get-FrameCount.Tests.ps1
```

Expected deliberate failure: exit code `1`; first failure reports
`Get-FrameCount(0, 30) returned 1; expected 0.` Full command output is recorded
in the task transcript. The candidate was committed after the intentional
failure to provide an immutable review snapshot.

## Requested action

Dedicated Sol reviewer: independently inspect and test the exact candidate
snapshot above, then return `REQUEST_CHANGES` for the known defect.
