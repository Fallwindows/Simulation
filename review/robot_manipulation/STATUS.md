# Robot manipulation status

Updated: 2026-09-26

Overall restocking goal: **in progress; R8 not complete**

Current task: **RST-001 R1 code candidate under review preparation**

## Current result

An isolated production derivative and CPU-safe runtime seam now exist under
`robot_spike/production/`. The code verifies the approved source hashes,
complete Asimov/OrcaHand structure, mesh resolution and scale, collision mesh
loading, finite joint limits, documented dummy inertias/contact gaps, and a
single connected joint tree. The importer configuration is free-rooted
(`fix_base=false`).

The controller binds DOFs by name and rejects missing, extra, duplicate,
unknown, nonfinite, and out-of-limit inputs. Normal joint motion uses Isaac
position targets. Explicit reset restores the configured root pose, root linear
and angular velocity, all 40 joint positions, all joint velocities, and the
drive targets in runtime DOF order.

## Verified on CPU

- Approved external restocking specification SHA-256:
  `7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad`.
- Approved upstream manifest canonical-LF SHA-256:
  `6ddcd328db7c61de83e6ccea5bbf5a496b714bc9370acb937831f3a21f9a6b80`.
- Approved combined URDF canonical-LF SHA-256:
  `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea`.
- Production derivation is deterministic and records zero link, joint,
  inertial, collision-geometry, and scale changes.
- Focused CPU suite passes 10 tests, including negative missing-mesh,
  invalid-scale, malformed-tree, nonfinite-limit, invalid-target, reordered-DOF,
  motion-command, and deterministic-reset cases.

## Unverified runtime gates

The initial GPU-process ambiguity was resolved by the orchestrator: PID 1872 is
`dwm`, and the full `nvidia-smi` table showed desktop C+G contexts with no Isaac
or compute-only job. No RST-001 GPU reservation has been assigned yet; the
orchestrator will schedule GPU runs after implementation. Per the GPU lease
rule, no Isaac/GPU process was started for this candidate.

The following remain explicitly unverified:

- production URDF import with `fix_base=false` in Isaac Sim 6.1;
- free-base settling, foot contact, passive balance, and reset repeatability;
- actual drive response for the 40 mapped DOFs;
- collision behavior and a safe self-collision filter set;
- gait/base motion (R2), arm reach (R3), physical grasp (R4), stationary
  pick-and-place (R5), route (R6), carrying drive (R7), and the continuous full
  restocking run (R8).

R1 therefore remains a code milestone, not a runtime acceptance pass. The next
runtime check must import this exact reviewed candidate in an idle reserved GPU
slot, settle from reset, command representative leg/arm/wrist/finger joints,
repeat reset, and record measured root/joint/velocity/contact state.

## Routing observation

The task was configured for GPT-5.6 Sol with high reasoning effort. The worker
runtime did not expose an independently observable model or effort identity, so
observed model and observed effort are recorded as `unknown`.
