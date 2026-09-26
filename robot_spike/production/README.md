# Restocking production robot seam

This directory contains the isolated R1 derivative of the approved Asimov 1 +
OrcaHand V1 spike. The upstream spike URDF, assets, evidence, and manifest are
inputs and are never rewritten.

`build_production_model.py` makes two intentional URDF edits: it gives the
derived robot a production-specific name and changes each `assets/...` mesh URI
to `../assets/...` so the isolated file resolves the exact same approved mesh.
It does not change links, joints, inertials, collision geometry, or mesh scale.

`model.py` validates source/output hashes, the single rooted joint tree, finite
joint limits, mesh paths, mesh scales, required Asimov/OrcaHand links, and every
documented inertia/collision assumption. `runtime.py` binds imported DOFs by
name. Normal motion calls `set_dof_position_targets`; direct root/joint state
setters are confined to the explicit deterministic reset method.

`arm_reach.py` is the CPU-safe R3 code candidate. It validates the production
URDF chain from `waist_yaw_link` through the six right-arm joints and fixed
OrcaHand mount to the `right_palm` tool frame. It provides FK, joint-limited
damped-least-squares IK, and bounded interpolation for caller-supplied
pre-grasp and pre-place palm poses. Waypoints are sent through the existing
name-bound position-target controller and contain only the six right-arm DOFs;
waist and leg drive targets remain under their separate controller authority.
The planner does not approach the product, close the hand, infer contact,
attach an object, or claim physical reach.

The importer configuration uses `fix_base=false` for the supplied biped. The
neutral reset places the pelvis at 0.635 m, clears all root and DOF velocities,
and restores all 40 joints to zero radians. This is a deterministic starting
state, not evidence of passive balance or gait. Self-collision stays disabled
until the known broad collision candidates are filtered and tested in Isaac.

CPU rebuild and checks:

```powershell
C:\isaacsim\python.bat robot_spike\production\build_production_model.py
C:\isaacsim\python.bat -m unittest tests.test_robot_production_model
C:\isaacsim\python.bat -m unittest tests.test_robot_arm_reach
```

Isaac import, free-base settling, joint actuation, contact behavior, and
repeatable reset remain runtime checks. R3 additionally requires an allocated
Isaac run to establish actual convergence of the articulation drives to the
pre-grasp and pre-place palm poses while the biped is supported or balanced.
Do not run those checks without the allocated GPU slot.
