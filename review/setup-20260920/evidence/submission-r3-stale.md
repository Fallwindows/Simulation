# Luna submission packet — revision 3 stale-approval check

- Task: `setup-rehearsal`
- Revision: `r3-stale-approval-check`
- Reviewer: dedicated GPT-5.6 Sol, configured effort `high`; observed runtime: `unknown`
- Base SHA: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Candidate SHA: `d0497f282e84765c05676adafbb62e4950a664d8`
- Candidate tree SHA: `f63d4d267d22065e48cdf84c7a031ab0b152da2b`
- Parent approved candidate SHA: `deee6973d210778c0d1ddff2d2276dd2974da2d9`
- Parent approved tree SHA: `42462247266c8644d527461b3c7f85cc9f93443d`
- Governing spec SHA-256: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Acceptance spec SHA-256: `56d0b16f8be409da4ff40aad9431bca1bee4042b9cbbbe36f33319007cc1e3ca`
- Input packet SHA-256: `9cbc92fd500f03dfc3b6863af02a18cd5c17d95d7a071095e81d700e98056a0e`
- Worktree snapshot: `runs/setup-20260920/worker`

## Later edit

After Sol approved r2 exactly, Luna appended commit
`d0497f282e84765c05676adafbb62e4950a664d8`, adding one inert explanatory
comment to `setup_rehearsal/Get-FrameCount.ps1`. The fixed test suite still
passes, but the source tree and candidate identity changed.

## Test evidence

```text
pwsh -NoProfile -File .\setup_rehearsal\Get-FrameCount.Tests.ps1
```

Result: exit code `0`, output `All fixed Get-FrameCount cases passed.`

## Requested action

Dedicated Sol reviewer: perform a stale-approval check. Confirm that the r2
`APPROVE` applies only to candidate `deee6973d210778c0d1ddff2d2276dd2974da2d9`
and tree `42462247266c8644d527461b3c7f85cc9f93443d`, and therefore cannot
authorize this changed r3 candidate. Return a fresh rejection or blocked
verdict for reuse of the old approval, with the changed identity recorded.
