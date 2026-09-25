# Simulation setup verification — 2026-09-20

Workflow setup is verified. Full G00 remains incomplete because Windows Code
Integrity blocks the existing RGB recorder's BLAS DLL, preventing a saved review
frame. No production implementation or full film run was started.

## Repository and inputs

- Main checkout remains `master`, HEAD `d5e825c8f6dab77aa6a1007c9731c226b588dfcf`.
  Remote: `git@github.com:Fallwindows/Simulation.git`. No pushes, resets, stashes,
  main-checkout commits, or production source edits were performed.
- Initial tracked tree was clean. The root specification and
  `simulation_video_codex_handoff/` were pre-existing untracked owner inputs;
  both remain intact and uncommitted. Root/package specification hashes match:
  `6d0fae8094f5abfbf9e5180e8120e795752a54775d9914a6e006cd1fb9706966`.
- Copied previously absent `agent_roles/` and `references/` from the handoff.
  All four role documents are readable. All 12 PNGs decode at 1672×941 and match
  their individual manifest SHA-256 values. No required input file is missing.
- Opened all 12 original images: RGB cereal aisle and shopper/cart (01–05),
  lower-center sensor and cyan rays (06), pure cyan point treatment (07),
  elevated persistent-map view (08), separate product bounds/cards (09),
  selected box with right-side detail card (10), wide map/path (11), and
  right-side ending typography (12). Illustrative IDs/measurements and blanket
  claims remain governed by the specification's corrections.
- No pre-existing applicable `AGENTS.md` or `AGENTS.override.md` was found in
  the repository, ancestor chain, or user Codex instruction location. Added a
  short root pointer. Added a narrow `.gitignore` exception for the original
  reference PNGs so they can be versioned later.

## Agent configuration and actual routing

Installed CLI: `0.155.0-alpha.9.2`. `codex --strict-config doctor --summary`
loads the project config without configuration errors. Installed app-server
`config/read` with this cwd explicitly returns the project layer and:

| Role | Configured | Host turn-context record |
|---|---|---|
| Orchestrator | `gpt-6-astra`, medium | Astra, medium |
| Programmer/inventory | `gpt-5.6-luna`, high | Luna, high |
| Dedicated reviewer | `gpt-5.6-sol`, high | Sol, high |

The refreshed local model catalog exposes all three exact identifiers and
supports those efforts. Runtime records are host-selected routing evidence,
not independent provider attestation. Earlier packets marked identity unknown
before this evidence was available; the routing supplement supersedes that
uncertainty.

New `.codex/config.toml` sets agents enabled, six maximum child sessions and
Luna/high defaults. Standalone role TOMLs define Luna programmer and Sol reviewer.
This follows the installed client and
[official subagent documentation](https://learn.chatgpt.com/docs/agent-configuration/subagents).
The current native collaboration API has no role selector, so the rehearsal used
explicit model/effort overrides with fresh contexts and role-document pointers.
No assumption was made that writing TOML changes an already-running session.

Current session limits: four total slots; effective full-access filesystem and
approval `never` for all three models. Thus Sol's configured read-only sandbox
was overridden by live permissions. Sol remained read-only with respect to
candidate source, tested detached snapshots, and wrote only rehearsal evidence.
These are workflow boundaries, not an OS-enforced read-only claim.

**Restart:** start a fresh Codex task/session from this repository to load the
new defaults and discover custom roles where the client supports role selection.
No workstation restart or Codex upgrade is required. A fresh session must still
check its actual slot limit; do not assume the six-child setting overrides a
platform cap. Reserve Sol's slot and reduce active writers if necessary.

The setup inputs are uncommitted. Fresh worktrees must use the documented
root-path or copy-and-hash bootstrap in [`.codex/README.md`](../../.codex/README.md).
Do not silently run a worktree that lacks its instructions and references.

## Completed disposable rehearsal

All commits below live only on disposable `codex/setup-*` branches; the main
checkout did not move. Exact full identities, spec/input hashes, commands,
reviewer session, findings and test results are in the linked packets.

| Required case | Observed result |
|---|---|
| Faulty candidate | Sol independently rejected `5331026b`, finding F01 |
| Active Luna revision | Luna appended correction `deee6973`, retested, resubmitted |
| Exact new approval | Sol independently approved `deee6973`, resolved F01 |
| Later edit/stale approval | Sol rejected reuse of that approval for `d0497f28`, F02 |
| Moved integration base | Base `282b14fc` required fresh combined review; Sol approved merge `80e35c71` after original and consumer tests |
| Inaccessible required check | Separate deliberate missing-evidence variant returned `BLOCKED`, B01 |

The absent file in the last case is a negative fixture, not a missing production
input. The unapproved r3 remains a negative fixture. No approval was reused.
The native parent relayed packets when the worker's app-server messaging attempt
was rejected; the actual revision loop completed without another owner request.

Evidence: [r1 rejection](evidence/sol-verdict-r1-5331026b.md),
[r2 approval](evidence/sol-verdict-r2-deee6973.md),
[stale rejection](evidence/sol-verdict-r3-stale-d0497f28.md),
[combined review and blocked variant](evidence/sol-verdict-combined-80e35c71.md).
Registered detached worktrees remain under `runs/setup-20260920/` for audit.

## Runtime and scoped environment repair

Verified Windows 11 Home 26200, Ryzen 7 7700X, about 31.16 GiB usable RAM,
RTX 4070 Ti with 12,282 MiB VRAM, driver 595.71 and about 708 GiB free disk.
Isaac installation is `6.1.0-rc.26+release.49347.2d230af4.gl`; runtime reports
6.1.0. Isaac and Pixi Python are 3.12.13; Pixi is 0.80.0; ROS is Jazzy;
RTAB-Map packages are 0.23.7; environment FFmpeg/ffprobe are 8.1.2.

The launcher runs repository Isaac source. ROS workspace source is a junction
to this repository's `grocery_sim_mapping`, but the installed launch/contracts
were stale. Preserved 41 installed package/index/setup files in
`runs/setup-20260920/installed-package-backup/`, then ran only:

```text
pixi run --as-is --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml colcon --log-base <repo>\runs\setup-20260920\colcon-log build --base-paths C:\IsaacSim-ros_workspaces\jazzy_ws\src\grocery_sim_mapping --build-base C:\IsaacSim-ros_workspaces\jazzy_ws\build --install-base C:\IsaacSim-ros_workspaces\jazzy_ws\install --merge-install --packages-select grocery_sim_mapping
```

Build exited 0, one package finished. Installed launch, contracts and params
now hash-match repository source. `ros2 pkg prefix`, package import, and
`ros2 launch grocery_sim_mapping rtabmap_lidar.launch.py --show-args` pass in
that exact environment. No dependency upgrades or native RTAB-Map rebuild were
performed. The nonfatal ROS latest-log symlink privilege warning remains.

A one-frame startup and a 180-frame realtime sensor smoke ran, not the full
production pipeline. The latter observed 88 RGB callbacks at 30 Hz, 29 LiDAR
clouds and 180 ground-truth samples. Status records the actual `Example_Rotary`
sensor asset. No synthetic timestamp fallback was used. A teardown
`InvalidHandle` traceback occurred despite process exit 0; clean ROS shutdown
and end-to-end mapping acceptance are not claimed. All owned runtime processes
were stopped. See [runtime inventory](evidence/runtime-inventory.md).

## Actual blocker and next action

Windows Code Integrity events 3077/3033 identify the recorder's blocked file:

```text
C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\Library\bin\libcblas.dll
```

The environment's `python.exe` cannot load it under policy
`{0283ac0f-fff1-49ae-ada1-8a933130cad6}`. NumPy import fails through OpenCV,
so the existing RGB recorder exits before subscribing. No MP4 or saved frame
was produced. The applicable policy owner must permit this trusted dependency
or provide an approved compatible build; no policy bypass was attempted.

After that is resolved, rerun the bounded recorder smoke and inspect the saved
frame before declaring full G00 verified. The rest of the production gates are
not started. Publication scope and owner execution authorization can be settled
when production is requested; neither was needed for this local setup.

Configuration, reference hashes and runtime-selected routing are saved beside
this report. The authoritative status entry is in `IMPLEMENTATION_JOURNAL.md`.
