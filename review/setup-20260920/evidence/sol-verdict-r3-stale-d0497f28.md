# Sol verdict — revision 3 stale-approval check

- Review ID: `setup-rehearsal-sol-r3-stale-d0497f28`
- Task/revision: `setup-rehearsal` / `r3-stale-approval-check`
- Reviewer session: `/root/sol_reviewer` (`01a0c269-d5ca-7a20-ab83-97b8d063651b`)
- Runtime-observed model/effort: `gpt-5.6-sol` / `high`, from host-selected `turn_context` evidence; not provider attestation
- Effective permissions: sandbox `danger-full-access`, approval policy `never`; source held read-only by reviewer conduct
- Base SHA: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Historically approved r2 SHA/tree: `deee6973d210778c0d1ddff2d2276dd2974da2d9` / `42462247266c8644d527461b3c7f85cc9f93443d`
- Changed r3 SHA/tree: `d0497f282e84765c05676adafbb62e4950a664d8` / `f63d4d267d22065e48cdf84c7a031ab0b152da2b`
- Acceptance specification SHA-256: `56d0b16f8be409da4ff40aad9431bca1bee4042b9cbbbe36f33319007cc1e3ca`
- Relevant input packet SHA-256: `9cbc92fd500f03dfc3b6863af02a18cd5c17d95d7a071095e81d700e98056a0e`
- Governing specification SHA-256: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Detached review snapshot: `runs/setup-20260920/review-r3-stale`
- Verdict on reuse of the r2 approval: `REQUEST_CHANGES`
- Blocking identity findings: 1

## Evidence

Resolved the exact r3 Git object. It is an append commit whose parent is approved r2, but its commit and tree identities differ. The delta adds one explanatory source comment to `setup_rehearsal/Get-FrameCount.ps1`.

The fixed-case test independently exits `0` with `All fixed Get-FrameCount cases passed.` Git status remained clean. Passing tests do not expand an approval beyond the SHA/tree it names.

### SETUP-REHEARSAL-F02 — BLOCKER TO APPROVAL REUSE

- Trigger: attempt to authorize r3 using review `setup-rehearsal-sol-r2-deee6973`.
- Consequence: the approval would be applied to source identity `d0497f28...` / `f63d4d26...`, which that review did not approve.
- Requirement: every changed source candidate requires a fresh exact-identity review; historical approval remains evidence only for r2.
- Required action: do not integrate or label r3 approved under the r2 verdict. If r3 needs authorization, submit it for a distinct full technical review and obtain a verdict naming the r3 SHA/tree.

## Approval boundary

This packet deliberately rejects stale approval reuse. It does not allege a behavioral defect in the comment-only edit, and it does not issue a fresh technical approval for r3. The r2 approval remains valid only for `deee6973...` / `42462247...`.
