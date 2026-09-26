# Robot Restocking Execution Notes

**Updated:** 2026-09-26
**Branch inspected:** `codex/robot-restocking` at `96f3845bf2af3cad05f93554b820f389539e7bd0`

> The requested root notes file was not present in this branch, the main project checkout, or the known project worktrees. This file records the requested focused upstream audit without changing or replacing existing robot code.

## Reuse-first execution rule

Before writing a new locomotion, grasp, manipulation, or control subsystem, first check whether the upstream robot/hand authors or a closely related open-source project already provide a usable implementation.

Preferred order:

1. Reuse a working pretrained implementation.
2. Adapt an upstream implementation.
3. Port an upstream implementation between simulators/frameworks.
4. Only then write a new controller from scratch.

Keep the current custom R2 locomotion and R3/R4 reach/grasp code as fallback and diagnostic infrastructure. Do not replace it until the candidate implementation has passed a physical Isaac/PhysX milestone on this derived Asimov+OrcaHand robot.

## Current integration facts relevant to reuse

- The current derived robot has 40 actuated DOFs: 23 Asimov body joints plus 17 right OrcaHand v1 DOFs. The Menlo Isaac task's body-joint name list has 23 entries and matches the Asimov body names in `robot_spike/production/robot_config.json`; the added hand is outside that policy action set.
- Our Isaac runtime adapter targets Isaac Sim 6.1. Menlo's inspected install instructions target Ubuntu 22.04+, Python 3.11, Isaac Sim 5.1.0, and its pinned Isaac Lab/RSL-RL stack. A policy checkpoint would still need an observation/action contract and actuator compatibility check before it can control our 40-DOF articulation.
- The last recorded physical R2 run on predecessor `d654aaf` failed pre-ramp after 144 samples. The current R2 successor has not been physically rerun. Current R3/R4 source checks do not prove physical grasp, carry, or placement. See `IMPLEMENTATION_JOURNAL.md`; the full continuous restock run remains the goal.

## Candidate implementations inspected

### 1. Menlo Research — Isaac Lab Asimov locomotion

- **Repository / inspected commit:** [menloresearch/isaac_asimov](https://github.com/menloresearch/isaac_asimov/tree/bdf28f5e8b60584fd6b8b50b7433d639c5d8b958), `bdf28f5e8b60584fd6b8b50b7433d639c5d8b958` (`main`). The README describes the repository as Asimov-1 locomotion training and evaluation code, built on Isaac Lab with PPO and AMP.
- **Relevant files:** `source/isaac_asimov/isaac_asimov/tasks/locomotion/velocity_env_cfg.py`; `amp_env_cfg.py`; `motions/policy_delay_walk_slow.npz`; `assets/robots/asimov_1.py`; `scripts/rsl_rl/play.py`; `scripts/rsl_rl/train.py`; `INSTALL.md`.
- **Capability:** velocity-command locomotion, contact sensing, PPO/AMP rewards and configs, Asimov joint/actuator definitions, evaluation and local log conventions. `ASIMOV_1_JOINT_NAMES` contains the same 23 Asimov body joints as our current combined model; our extra 17 hand joints can remain under the existing grasp controller. Policy observations, action scaling, normalization, joint order, default pose, delays, and gains still need exact matching.
- **License:** BSD-3-Clause for this repository. Its pinned `third_party/asimov-1` submodule points to `732cc60dcb8f2b4fd26c3d7346b35f9b89c3cd47`; the separately inspected Asimov-1 `main` at `ccf5326f0adffd65edf930cbbac052454cdd82be` labels software GPL-2 and hardware CERN-OHL-S. Keep those model/source terms distinct from the Isaac extension license.
- **Fit / effort / recommendation:** Closest locomotion source and the first policy path to check. Same Asimov body names, but the inspected install stack is Linux / Isaac Sim 5.1.0 while our runtime is Windows / Isaac Sim 6.1. Estimated effort: medium if a compatible pretrained policy is available; high if the policy must be ported or retrained under the current stack. **ADAPT**.

### 2. Menlo Research — Asimov MJLab locomotion

- **Repository / inspected commit:** [menloresearch/asimov-mjlab](https://github.com/menloresearch/asimov-mjlab/tree/98870d12f079c0b6313bb0fe459aa591a5e7f251), `98870d12f079c0b6313bb0fe459aa591a5e7f251` (`main`).
- **Relevant files:** `src/mjlab/asset_zoo/robots/asimov/xmls/asimov.xml`; `src/mjlab/tasks/velocity/velocity_env_cfg.py`; `src/mjlab/scripts/play.py`; `src/mjlab/scripts/train.py`; `src/mjlab/utils/wandb.py`.
- **Capability:** Asimov velocity tracking, reward/actuator and observation conventions, MuJoCo-Warp iteration, training and W&B integration. The repository README describes a 12-DOF leg model. No policy checkpoint file is tracked at the inspected commit.
- **License:** Apache-2.0.
- **Fit / effort / recommendation:** The 12 leg joints overlap R2, but MuJoCo policies cannot be dropped into the current Isaac runtime and this is not the 23-DOF full-body Isaac task. Low effort to mine settings; high effort to port/train. **REFERENCE**.

### 3. Menlo Research — Asimov-1 model source

- **Repository / inspected commit:** [menloresearch/asimov-1](https://github.com/menloresearch/asimov-1/tree/ccf5326f0adffd65edf930cbbac052454cdd82be), `ccf5326f0adffd65edf930cbbac052454cdd82be` (`main`). The Isaac locomotion repo pins a separate submodule revision, recorded above.
- **Relevant files:** `README.md`; `sim-model/urdf/asimov_1.urdf`; `sim-model/xmls/asimov_1.xml`; `SOFTWARE-LICENSE.txt`; `HARDWARE-LICENSE.txt`.
- **Capability:** Asimov-1 simulation model and original robot geometry/conventions. Our current derived robot already retains the Asimov body and adds the OrcaHand model.
- **License:** software GPL-2; hardware design CERN-OHL-S.
- **Fit / effort / recommendation:** Reuse as a geometry and joint-convention reference; do not rebuild the current combined model from it. Low effort for comparisons. **REFERENCE**.

### 4. OrcaHand — core controller and named poses

- **Repository / inspected commit:** [orcahand/orca_core](https://github.com/orcahand/orca_core/tree/cc607f06e8f16ffd6c71139c60e6d9471b84aa3d), `cc607f06e8f16ffd6c71139c60e6d9471b84aa3d` (`main`).
- **Relevant files:** `examples/main_demo.py`; `examples/main_demo_abduction.py`; `orca_core/data/demo_poses.yaml`; `orca_core/base_hand.py` / `pose_from_fractions`.
- **Capability:** reusable `open_hand`, `power_grasp`, `pinch`, and `spread_grasp` targets. The YAML explicitly stores fractions of each joint's range, not radians; `BaseHand.pose_from_fractions` skips pose keys absent from the selected hand configuration. The simple demo sequences open → power grasp → pinch and leaves timing/sequence orchestration to its caller.
- **License:** MIT.
- **Fit / effort / recommendation:** Directly useful as a starting preshape/closing target. Map v1 joint IDs to our `right_`-prefixed URDF joints and convert fractions through the exact v1 limits; compare against R4 `PRESHAPE_TARGETS` and `CLOSE_TARGETS`, then retain measured contact/lift gates. Estimated effort: low. **USE** for pose data after that mapping check, not as proof of a physical grasp.

### 5. OrcaHand — MuJoCo simulation

- **Repository / inspected commit:** [orcahand/orca_sim](https://github.com/orcahand/orca_sim/tree/05e00dafd0a6eb2186d69d4e351c252a7bcffadd), `05e00dafd0a6eb2186d69d4e351c252a7bcffadd` (`main`).
- **Relevant files:** `README.md`; `src/orca_sim/task_envs.py`; `src/orca_sim/scenes/v1/`; `src/orca_sim/models/v1/`; `random_policy.py`.
- **Capability:** Gymnasium/MuJoCo hand actuation and physics, explicit version pinning such as `version="v1"`, and an in-hand cube-orientation task. That task resets the cube onto the palm; it is not a pick-from-support, carry, or shelf-placement task.
- **License:** no `LICENSE` file or license field was present in the inspected repository snapshot; use as a behavioral reference and do not copy code/assets until reuse terms are clear.
- **Fit / effort / recommendation:** Useful for v1 reset, joint, and contact behavior. It cannot run inside the current Isaac/PhysX scene without a port, and it is not a restocking solution. Estimated effort: low to study, high to port. **REFERENCE**.

### 6. OrcaHand — teleoperation, recording, and policy hooks

- **Repository / inspected commit:** [orcahand/orca_teleop](https://github.com/orcahand/orca_teleop/tree/062ddeb8c2b182d5c9cd8bd406cb07a532c97fc8), `062ddeb8c2b182d5c9cd8bd406cb07a532c97fc8` (`main`).
- **Relevant files:** `scripts/teleop_sim.py`; `scripts/run_lerobot_policy_sim.py`; `scripts/record_dataset.py`; `scripts/replay_dataset.py`; `src/orca_teleop/sim.py`; `src/orca_teleop/policies.py`; `src/orca_teleop/recording.py`.
- **Capability:** v1 MuJoCo simulation sink, joint-ID mapping, LeRobot policy interface, and demo recording/replay. `sim.py` states that v1 MuJoCo joint names match Orca core config IDs; its simulation sink converts degree-valued Orca actions to MuJoCo radians. The README documents a `"pick up the block"` recording prompt. `run_lerobot_policy_sim.py` references `fracapuano/orca-panda-test-act` and `fracapuano/orca-panda-test-50x`, but those exact public Hub artifacts were not independently verified in this audit.
- **License:** no `LICENSE` file or license field was present in the inspected repository snapshot; the Hub policy/dataset have separate terms that must be checked if used.
- **Fit / effort / recommendation:** Strong reference for v1 name mapping, collecting/replaying hand demonstrations, and controller interface; MuJoCo/LeRobot path is not a direct Isaac runtime adapter. Estimated effort: low for pose/name reuse, medium-to-high for an Isaac demonstration/policy port. **REFERENCE**.

### 7. Isaac Lab-Arena — humanoid loco-manipulation reference

- **Repository / inspected commit:** [isaac-sim/IsaacLab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/tree/aa36f191d89b1ea65fbba62b3350d6fc7d0e9b9f), `aa36f191d89b1ea65fbba62b3350d6fc7d0e9b9f` (`main`). The relevant G1 loco-manipulation example is documented in `docs/pages/example_workflows/locomanipulation/`.
- **Capability:** composable Isaac Lab scenes/embodiments/tasks and a Unitree G1 sequence for walk, pick, and place; current documentation also describes G1 box pick/place with generated demonstrations. It provides a useful staged-task/data-generation pattern, not an Asimov/Orca controller.
- **License:** Apache-2.0.
- **Fit / effort / recommendation:** Different 29-DOF G1 embodiment; current `main` targets Linux, Isaac Lab 3.0 / Isaac Sim 6.0 while this task runs Windows / Isaac Sim 6.1. Direct port is not a quick path. Estimated effort: high. **REFERENCE** for phase sequencing and failure predicates only.

## Checkpoint and linked-resource scan (as of 2026-09-26)

- Menlo's Isaac repo contains no tracked `.pt`, `.pth`, `.onnx`, `.ckpt`, `.safetensors`, or zip policy checkpoint at inspected commit `bdf28f5e8b60584fd6b8b50b7433d639c5d8b958`; its GitHub Releases page reports no releases. It contains training/logging code and ignores generated checkpoints/log directories.
- `scripts/rsl_rl/play.py` has `--use_pretrained_checkpoint`, calling Isaac Lab's `get_published_pretrained_checkpoint("rsl_rl", train_task_name)`. Its own failure path prints that the pretrained checkpoint is unavailable. This is the highest-value remaining availability check; it has not yet been executed in a compatible Menlo Isaac Lab runtime.
- No public W&B run/artifact or Hugging Face checkpoint for the exact Asimov task names was located by the targeted search. The source supports logger choices including W&B, but the repo does not link a public experiment/checkpoint.
- Menlo's current software documentation says the supplied Asimov RPU firmware includes a 23-DOF locomotion policy, while its Asimov-1 firmware-download page still lists firmware images as “coming soon.” Treat that as evidence that a trained policy exists, not evidence that a simulation checkpoint is downloadable.
- Relevant official links: [Menlo Asimov software availability](https://docs.menlo.ai/asimov/1/overview/system-tour/software); [Menlo locomotion training docs](https://docs.menlo.ai/guides/locomotion-training); [Menlo Asimov zero-shot sim-to-real write-up](https://menlo.ai/research/zero-shot-sim2real-asimov); [Isaac Lab humanoid loco-manipulation workflow](https://github.com/isaac-sim/IsaacLab/blob/develop/docs/source/features/imitation-learning/humanoids_imitation.rst).

## Immediate reuse order

1. Before another gait redesign, check whether the Menlo `--use_pretrained_checkpoint` lookup actually resolves the Asimov AMP velocity task. If it does, evaluate the checkpoint on the matching upstream model/config first, then map its named 23 body outputs onto this 40-DOF Asimov+Orca model while leaving the 17 finger/wrist DOFs under R4 control. Verify observation/action ordering, normalization, gains, delay, and command limits before using it in the continuous run.
2. If no downloadable checkpoint is available, compare Menlo's upstream actuator, observation, contact, reward, velocity-command, and motion-reference settings against our current R2 runtime. Reuse/adapt the closest pieces; preserve our custom R2 as fallback. Do not invest in a broad RL training pipeline before a small upstream compatibility check.
3. For R4, convert Orca's v1 `power_grasp` fractions through the exact production URDF joint ranges, map names, and use it as the next physical preshape/close candidate. Keep physical-contact, object-lift, and relative-pose checks from the current R4 path.
4. Use `orca_sim` and `orca_teleop` for v1 joint/reset/actuator and data-interface references. They do not demonstrate grocery pickup, transport, placement, or stable shelf release in Isaac.

Keep the next investigation bounded to the checkpoint availability/mapping test and one power-grasp comparison. The existing continuous Isaac restocking run remains the only completion proof.