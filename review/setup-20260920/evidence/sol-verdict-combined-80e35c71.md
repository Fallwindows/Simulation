# Sol verdict — moved-base combined rehearsal

- Review ID: `setup-rehearsal-sol-combined-80e35c71`
- Task: `setup-rehearsal-integration`
- Reviewer session: `/root/sol_reviewer` (`01a0c269-d5ca-7a20-ab83-97b8d063651b`)
- Runtime-observed model/effort: `gpt-5.6-sol` / `high`, from host-selected `turn_context` evidence; not provider attestation
- Effective permissions: sandbox `danger-full-access`, approval policy `never`; source held read-only by reviewer conduct
- Original task base: `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`
- Moved integration base / first parent: `282b14fcaa9141d3b83b0f600ea7b421650334fd`
- Previously approved r2 / second parent: `deee6973d210778c0d1ddff2d2276dd2974da2d9`
- Combined candidate SHA: `80e35c71b3af1477e2beee9eeec7edea097b0ee0`
- Combined candidate tree SHA: `0e66ec65c5afe7a9721cd5ee84a8c4449e55e19c`
- Integration acceptance variant SHA-256: `588e652da3d2a9d9975baf0b047eb97b58b4391829ae7598a20be4ba837d83c9`
- Original acceptance specification SHA-256: `56d0b16f8be409da4ff40aad9431bca1bee4042b9cbbbe36f33319007cc1e3ca`
- Relevant input packet SHA-256: `9cbc92fd500f03dfc3b6863af02a18cd5c17d95d7a071095e81d700e98056a0e`
- Governing specification SHA-256: `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`
- Detached review snapshot: `runs/setup-20260920/review-combined-80e35c71`

## Variant A — moved-base combined state

- Verdict: `APPROVE`
- Blocking findings: 0

The earlier r2 approval cannot authorize this candidate because the integration base moved and the combined commit has a distinct SHA/tree. This is a new review of the exact two-parent state.

Inspected both parents and their interactions. Relative to the moved base, the merge adds the previously approved function and fixed-case tests. Relative to r2, it adds only `setup_integration/consumer.ps1`, which imports the merged function and requires `Get-FrameCount(2, 24) = 48`.

Independent commands and outcomes:

```text
pwsh -NoProfile -File .\setup_rehearsal\Get-FrameCount.Tests.ps1
# exit 0: All fixed Get-FrameCount cases passed.

pwsh -NoProfile -File .\setup_integration\consumer.ps1
# exit 0: Integration consumer passed: 2 seconds at 24 fps = 48 frames.
```

Git status was clean before and after. The exact combined state satisfies the unchanged original requirements and the moved-base consumer requirement.

Approval applies only to candidate `80e35c71...`, tree `0e66ec65...`, against moved base `282b14fc...` under the recorded acceptance/input hashes. Any further source/base/input change requires a new review.

## Variant B — deliberately inaccessible required evidence

- Verdict: `BLOCKED`
- Blocking conditions: 1

Additional required path:

```text
C:\Users\suyog\OneDrive\Documents\ChatGPT\Simulation\runs\setup-20260920\evidence\intentionally-unavailable-required-result.json
```

Independent access attempt found `Test-Path = False`; `Get-Content -ErrorAction Stop` raised `System.Management.Automation.ItemNotFoundException`. No external-test result can be inspected. The missing required evidence was not fabricated or treated as optional.

### SETUP-INTEGRATION-B01 — BLOCKER

- Trigger: evaluate the inaccessible-evidence variant.
- Consequence: the independently executed external-test claim cannot be verified.
- Requirement: the named JSON must be readable and contain the required independently executed external-test result.
- Resolution: supply the exact required evidence under a new review request, or explicitly end this deliberate negative variant. No technical pass is implied by Variant A's approval.

## Known limitations

These verdicts exercise the setup protocol only. They do not approve production code, the simulation baseline, or final media.
